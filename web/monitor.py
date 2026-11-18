#!/usr/bin/env python3
"""Robot dashboard: host/ROS monitoring plus explicit control.

GET endpoints stay read-only (status, logs, camera frames). Control is limited
to two whitelisted actions that require an explicit {"confirm": true} body:
starting/stopping the tour_chat node and sending a navigation request for a
place defined in places.yaml.
"""
import argparse
import collections
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import urllib.request

import psutil
import yaml

ROOT = Path(__file__).resolve().parent
PROJECT = Path('/home/nvidia/guide_robot/learn/visitor_wayfinding_robot')
PLACES_FILE = PROJECT/'install/learn3_d/share/learn3_d/config/places.yaml'
PLACES_FALLBACK = PROJECT/'src/learn3_d/config/places.yaml'
VOICE_LOG = Path('/tmp/learn3d-tour-web.log')
VOICE_START_ARGS = ['ros2', 'launch', 'learn3_d', 'tour.launch.py']


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def redact(text):
    text = re.sub(r'(?i)((?:api[_-]?key|access[_-]?token|access[_-]?key(?:[_-]?secret|[_-]?id)?|secret|password|authorization)\s*[:=]\s*)[^\s,;]+', r'\1[redacted]', text)
    text = re.sub(r'(?i)Bearer\s+\S+', 'Bearer [redacted]', text)
    return text[:600]


def load_places():
    """Read the place list (display name per POI, main-line order) from places.yaml."""
    for path in (PLACES_FILE, PLACES_FALLBACK):
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        except Exception:
            continue
        aliases = data.get('aliases') or {}
        display = {}
        for alias, target in aliases.items():
            poi = str(target)
            display.setdefault(poi, str(alias))
        transfer = set(data.get('transfer_points') or [])
        order = [str(name) for name in (data.get('main_line') or [])]
        items = []
        for poi in order:
            if poi in transfer or poi not in display:
                continue
            items.append({'poi': poi, 'name': display[poi]})
        for poi, name in display.items():
            if poi not in order and poi not in transfer:
                items.append({'poi': poi, 'name': name})
        if items:
            return items
    return []


_START_RE = re.compile(r'开始(?:定点)?导航：(?P<poi>[^（(]+)')
_ROUTE_RE = re.compile(r'定点导航到 (?P<poi>[^：]+)：(?P<legs>\d+) 个导航段，(?P<route>.+)')
_ARRIVE_RE = re.compile(r'到达 (?P<poi>[^，]+)，用时')
_FAIL_RE = re.compile(r'(连续 \d+ 次导航任务失败|未通过到点位置校验|导航任务失败|定位持续不稳定|暂时不能移动|前方持续有障碍|没有点位)')


class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.started = time.time()
        self.logs = collections.deque(maxlen=200)
        self.seq = 0
        self.host = {}
        self.ros = {'available': False, 'error': 'ROS 正在连接', 'nodes': [], 'topics': [], 'services': [], 'graph_at': None}
        self.samples = {}
        self.frames = {}
        self.stopping = threading.Event()
        self.places = []
        # 当前导航任务：state=idle/running/done/failed
        self.task = {'state': 'idle', 'from': None, 'to': None, 'route': [], 'at': None}
        # 由本页启动的语音节点进程
        self.voice = None
        self.voice_started_at = None
        self.publisher = None
        self.publish_lock = threading.Lock()
        self.map_info = None
        # 底盘点位坐标：名字 -> {x, y}，用于示意图上的位置换算
        self.pois = {}
        self.add_log('INFO', 'monitor', '监测已启动；仅开放启动节点与快捷导航两个确认操作')

    def add_log(self, level, source, text):
        with self.lock:
            self.seq += 1
            self.logs.append({'id': self.seq, 'at': time.time(), 'level': level, 'source': source[:100], 'text': redact(text)})

    def sample(self, key, data):
        with self.lock:
            self.samples[key] = {'at': time.time(), **data}

    def voice_running(self):
        with self.lock:
            nodes = self.ros.get('nodes') or []
            process = self.voice
        if any(name.endswith('/tour_chat') for name in nodes):
            return True
        return bool(process is not None and process.poll() is None)

    def voice_managed(self):
        process = self.voice
        return bool(process is not None and process.poll() is None)

    def set_task(self, state, from_=None, to=None, route=None):
        with self.lock:
            self.task = {'state': state, 'from': from_, 'to': to,
                         'route': list(route or []), 'at': time.time()}

    def track_task(self, text):
        """把 tour_chat 的导航日志折叠成页面上的当前任务。"""
        route = _ROUTE_RE.search(text)
        if route:
            names = [re.sub(r'\s*\([^)]*\)$', '', part.strip())
                     for part in route.group('route').split('->')]
            names = [name for name in names if name]
            if names:
                self.set_task('running', names[0], names[-1], names)
            return
        start = _START_RE.search(text)
        if start:
            poi = start.group('poi').strip()
            self.set_task('running', '当前位置', poi, [poi])
            return
        arrive = _ARRIVE_RE.search(text)
        if arrive:
            with self.lock:
                task = dict(self.task)
            if task['state'] == 'running' and arrive.group('poi').strip() == task.get('to'):
                self.set_task('done', task.get('from'), task.get('to'), task.get('route'))
            return
        if _FAIL_RE.search(text):
            with self.lock:
                task = dict(self.task)
            if task['state'] == 'running':
                self.set_task('failed', task.get('from'), task.get('to'), task.get('route'))

    def snapshot(self):
        with self.lock:
            now = time.time()
            samples = {key: {**data, 'age_s': round(max(0, now-data['at']), 2)} for key, data in self.samples.items()}
            cameras = {key: {'at': data['at'], 'age_s': round(max(0, now-data['at']), 2), 'width': data['width'], 'height': data['height']} for key, data in self.frames.items()}
            graph_age = now-self.ros['graph_at'] if self.ros['graph_at'] else None
            return {'read_only': True, 'server_time': now, 'started_at': self.started, 'host': self.host.copy(), 'ros': {**self.ros, 'graph_age_s': graph_age}, 'samples': samples, 'cameras': cameras, 'task': dict(self.task), 'places': list(self.places), 'pois': dict(self.pois), 'node': {'running': self.voice_running(), 'managed': self.voice_managed()}, 'map': self.map_info}


def host_worker(state):
    psutil.cpu_percent()
    while not state.stopping.is_set():
        try:
            memory = psutil.virtual_memory()
            temps = {}
            for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
                try:
                    value = float((zone/'temp').read_text())/1000
                    if math.isfinite(value):
                        temps[(zone/'type').read_text().strip()] = round(value, 1)
                except (ValueError, OSError, TypeError):
                    pass
            cpu_temp = next((v for k, v in temps.items() if 'cpu' in k.lower()), None)
            with state.lock:
                state.host = {'at': time.time(), 'cpu_percent': psutil.cpu_percent(), 'memory_used_gb': round(memory.used/1024**3, 2), 'memory_total_gb': round(memory.total/1024**3, 2), 'memory_percent': memory.percent, 'cpu_temp_c': cpu_temp, 'uptime_s': round(time.time()-psutil.boot_time()), 'temperatures': temps}
        except Exception as exc:
            traceback.print_exc()
            state.add_log('WARN', 'monitor.host', type(exc).__name__)
        state.stopping.wait(2)



BATTERY_STATUS_URL = 'http://192.168.11.10:9090/api/core/system/v1/battery/pack'
POSE_URL = 'http://192.168.11.10:9090/api/core/navigation/v1/pose'
POIS_URL = 'http://192.168.11.10:9090/api/core/navigation/v1/poi'
MAP_CALIB_FILE = ROOT / 'map_calib.json'


def load_map_calibration():
    """地图像素标定：px = origin_x + x*scale，py = origin_y - y*scale。"""
    try:
        data = json.loads(MAP_CALIB_FILE.read_text(encoding='utf-8'))
    except Exception:
        return None
    for key in ('width', 'height', 'scale', 'origin_x', 'origin_y'):
        try:
            data[key] = float(data[key])
        except (KeyError, TypeError, ValueError):
            return None
    data['available'] = (ROOT / 'map.png').exists()
    return data


def parse_chassis_pose(data):
    if not isinstance(data, dict) or data.get('success') is not True:
        raise ValueError('Pose response was not successful')
    values = {}
    for key in ('x', 'y', 'yaw'):
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'Invalid pose {key}')
        values[key] = float(value)
    confidence = data.get('confidence')
    values['confidence'] = (finite(confidence)
                            if isinstance(confidence, (int, float)) and not isinstance(confidence, bool)
                            else None)
    values['topic'] = '底盘 GET /api/core/navigation/v1/pose'
    return values


def chassis_pose_worker(state):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    was_available = None
    while not state.stopping.is_set():
        try:
            with opener.open(POSE_URL, timeout=3) as response:
                data = json.loads(response.read(65536))
            state.sample('pose', parse_chassis_pose(data))
            if was_available is not True:
                state.add_log('INFO', 'monitor.pose', '底盘实时位姿已连接')
            was_available = True
        except Exception as exc:
            if was_available is not False:
                state.add_log('WARN', 'monitor.pose', '底盘位姿暂不可用：'+type(exc).__name__)
            was_available = False
        state.stopping.wait(1)


def chassis_poi_worker(state):
    """读取底盘点位坐标，供示意图按实际位置换算进度；随导航任务刷新。"""
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    while not state.stopping.is_set():
        try:
            with opener.open(POIS_URL, timeout=3) as response:
                data = json.loads(response.read(262144))
            pois = {}
            for item in data.get('pois') or []:
                try:
                    pois[str(item['name'])] = {'x': float(item['x']), 'y': float(item['y'])}
                except (KeyError, TypeError, ValueError):
                    continue
            if pois:
                with state.lock:
                    changed = pois != state.pois
                    state.pois = pois
            else:
                changed = False
        except Exception:
            changed = False
        state.stopping.wait(10 if changed else 30)


def parse_chassis_battery(data):
    if not isinstance(data, dict) or data.get('success') is not True:
        raise ValueError('Battery response was not successful')
    value = data.get('batteryPercentage')
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 100:
        raise ValueError('Invalid battery percentage')
    charging = data.get('isCharging')
    current = data.get('current')
    return {'percent': float(value), 'is_charging': charging if isinstance(charging, bool) else None,
            'current_ma': finite(current) if isinstance(current, (int, float)) and not isinstance(current, bool) else None,
            'topic': '底盘 GET /api/core/system/v1/battery/pack'}


def chassis_battery_worker(state):
    # One fixed, known read-only endpoint. No SDK constructor, command or proxy API.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    was_available = None
    previous_charging = None
    while not state.stopping.is_set():
        try:
            with opener.open(BATTERY_STATUS_URL, timeout=3) as response:
                data = json.loads(response.read(65536))
            sample = parse_chassis_battery(data)
            state.sample('chassis_battery', sample)
            if was_available is not True:
                state.add_log('INFO', 'monitor.battery', '底盘电池只读接口已连接')
            if sample['is_charging'] is not previous_charging:
                state.add_log('INFO', 'monitor.battery', '底盘报告：'+('正在充电' if sample['is_charging'] else '未充电' if sample['is_charging'] is False else '充电状态未知'))
            previous_charging = sample['is_charging']
            was_available = True
        except Exception as exc:
            if was_available is not False:
                state.add_log('WARN', 'monitor.battery', '底盘电池状态暂不可用：'+type(exc).__name__)
            was_available = False
        state.stopping.wait(5)


def ros_worker(state):
    node = None
    context = None
    executor = None
    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String

        context = Context()
        rclpy.init(context=context)
        node = Node('tour_web_readonly_monitor', context=context, enable_rosout=False, start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        state.publisher = node.create_publisher(String, '/tour/chat', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        subscriptions = {}
        last_process = {}
        last_graph = 0

        def image_callback(kind, msg):
            now = time.monotonic()
            if now-last_process.get(kind, 0) < (0.33 if kind == 'color' else 0.5):
                return
            last_process[kind] = now
            try:
                if kind == 'color':
                    if hasattr(msg, 'format'):
                        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    else:
                        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
                        if channels is None:
                            return
                        image = np.ndarray((msg.height, msg.width, channels), dtype=np.uint8, buffer=msg.data, strides=(msg.step, channels, 1))
                        conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
                        if msg.encoding in conversions:
                            image = cv2.cvtColor(image, conversions[msg.encoding])
                else:
                    dtype = {'16UC1': np.dtype('u2'), '32FC1': np.dtype('f4')}.get(msg.encoding)
                    if dtype is None:
                        return
                    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
                    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data, strides=(msg.step, dtype.itemsize)).astype(np.float32)
                    if msg.encoding == '16UC1':
                        depth /= 1000
                    h, w = depth.shape
                    roi = depth[int(h*.25):int(h*.60), int(w*.35):int(w*.65)]
                    valid = roi[np.isfinite(roi) & (roi > .05)]
                    state.sample('depth', {'distance_m': round(float(np.percentile(valid, 5)), 3) if valid.size else None, 'zero_ratio': round(float(np.count_nonzero(roi <= .05))/roi.size, 3) if roi.size else None, 'topic': '/ob_camera_head/depth/image_raw'})
                    scaled = (np.clip(np.nan_to_num(depth, nan=0, posinf=0, neginf=0), 0, 5)/5*255).astype(np.uint8)
                    image = cv2.applyColorMap(255-scaled, cv2.COLORMAP_TURBO)
                    image[~np.isfinite(depth) | (depth <= .05)] = 0
                if image is None:
                    return
                height, width = image.shape[:2]
                if width > 768:
                    image = cv2.resize(image, (768, round(height*768/width)), interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 65])
                if ok:
                    with state.lock:
                        state.frames[kind] = {'at': time.time(), 'jpeg': encoded.tobytes(), 'width': width, 'height': height}
            except Exception as exc:
                if now-last_process.get(kind+'_error', 0) > 30:
                    last_process[kind+'_error'] = now
                    state.add_log('WARN', 'monitor.camera', kind+': '+type(exc).__name__)

        def battery_callback(msg):
            state.sample('battery', {'master_percent': finite(msg.master_battery_power), 'little_percent': finite(msg.little_battery_power), 'master_voltage': finite(msg.master_battery_voltage), 'master_current': finite(msg.master_battery_current), 'installed': int(msg.battery_installed), 'working': int(msg.battery_working), 'topic': '/power/battery/status'})

        def board_callback(msg):
            state.sample('power', {'percent': finite(msg.battery_power), 'voltage': finite(msg.battery_voltage), 'current': finite(msg.battery_current), 'topic': '/power/board/status'})

        def ros_log(msg):
            # 导航任务状态来自 tour_chat 节点的 INFO 日志；其余只收基础设施告警。
            if 'tour_chat' in msg.name:
                state.track_task(msg.msg)
            allowed = ('ob_camera', 'knewbots_chassis', 'power_board', 'tour_chat', 'front_guard')
            if msg.level >= 30 and any(x in msg.name for x in allowed):
                now = time.monotonic()
                if now-last_process.get('ros_log', 0) >= 1:
                    last_process['ros_log'] = now
                    state.add_log('ERROR' if msg.level >= 40 else 'WARN', msg.name, msg.msg)

        def camera_status(msg):
            state.sample('camera_status', {'online': bool(msg.device_online), 'color_fps': finite(msg.color_frame_rate_cur), 'depth_fps': finite(msg.depth_frame_rate_cur), 'color_delay_ms': finite(msg.color_delay_ms_cur), 'connection': msg.connection_type})

        while not state.stopping.is_set():
            if time.monotonic()-last_graph > 5:
                last_graph = time.monotonic()
                nodes = sorted(set('/'.join((ns.rstrip('/'), name)) for name, ns in node.get_node_names_and_namespaces()))
                topics = dict(node.get_topic_names_and_types())
                services = [name for name, types in node.get_service_names_and_types()]
                with state.lock:
                    was_available = state.ros['available']
                    state.ros = {'available': True, 'error': None, 'nodes': nodes, 'topics': sorted(topics), 'services': sorted(services), 'graph_at': time.time()}
                if not was_available:
                    state.add_log('INFO', 'monitor.ros', 'ROS 图已连接；仅订阅已有状态与图像话题')
                color_topic = '/ob_camera_head/color/image_raw/compressed' if '/ob_camera_head/color/image_raw/compressed' in topics else '/ob_camera_head/color/image_raw'
                candidates = [(color_topic, lambda m: image_callback('color', m)), ('/ob_camera_head/depth/image_raw', lambda m: image_callback('depth', m)), ('/power/battery/status', battery_callback), ('/power/board/status', board_callback), ('/ob_camera_head/device_status', camera_status), ('/rosout', ros_log), ('/lyre/audio_stream', lambda m: state.sample('microphone', {'topic': '/lyre/audio_stream'}))]
                for topic, callback in candidates:
                    if topic in topics and topic not in subscriptions:
                        try:
                            msg_type = get_message(topics[topic][0])
                            subscriptions[topic] = node.create_subscription(msg_type, topic, callback, qos)
                            state.add_log('INFO', 'monitor.ros', '只读订阅 '+topic)
                        except Exception as exc:
                            state.add_log('WARN', 'monitor.ros', '无法订阅 '+topic+' ('+type(exc).__name__+')')
            executor.spin_once(timeout_sec=.1)
    except Exception as exc:
        traceback.print_exc()
        with state.lock:
            state.ros['available'] = False
            state.ros['error'] = 'ROS 监测中断：'+type(exc).__name__
        state.add_log('ERROR', 'monitor.ros', 'ROS 监测不可用：'+type(exc).__name__)
    finally:
        state.publisher = None
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()


def ros_supervisor(state):
    while not state.stopping.is_set():
        ros_worker(state)
        if not state.stopping.wait(5):
            state.add_log('INFO', 'monitor.ros', '正在重新连接 ROS 只读监测')


# --------------------------------------------------------------- 控制动作
def start_voice(state):
    if state.voice_running():
        return 409, '带路语音节点已经在运行'
    try:
        log = open(VOICE_LOG, 'ab', buffering=0)
        process = subprocess.Popen(
            VOICE_START_ARGS, stdout=log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True)
    except Exception as exc:
        state.add_log('ERROR', 'monitor.control', f'启动语音节点失败：{type(exc).__name__}')
        return 500, f'启动失败：{exc}'
    state.voice = process
    state.voice_started_at = time.time()
    state.add_log('INFO', 'monitor.control', f'已启动带路语音节点，PID={process.pid}')
    return 200, '已下发启动，节点正在初始化'


def stop_voice(state):
    process = state.voice
    if process is None or process.poll() is not None:
        state.voice = None
        return 409, '节点不是由本页启动，请在终端手动停止'
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except ProcessLookupError:
        state.add_log('WARN', 'monitor.control', '语音节点进程已不存在')
        state.voice = None
        return 200, '节点已停止'
    for _ in range(40):
        if process.poll() is not None:
            break
        time.sleep(.2)
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    state.voice = None
    state.set_task('idle')
    state.add_log('INFO', 'monitor.control', '已停止带路语音节点')
    return 200, '节点已停止'


def send_navigation(state, place):
    names = {item['name']: item for item in state.places}
    if place not in names:
        return 400, '地点不在配置清单里'
    if not state.voice_running():
        return 409, '带路语音节点未运行，请先启动节点'
    publisher = state.publisher
    if publisher is None:
        return 503, 'ROS 连接尚未就绪，稍后再试'
    try:
        from std_msgs.msg import String
        message = String()
        message.data = f'去{place}'
        with state.publish_lock:
            publisher.publish(message)
    except Exception as exc:
        state.add_log('ERROR', 'monitor.control', f'下发导航失败：{type(exc).__name__}')
        return 503, f'下发失败：{exc}'
    state.set_task('running', '当前位置', names[place]['poi'], [names[place]['poi']])
    state.add_log('INFO', 'monitor.control', f'已下发导航指令：去{place}')
    return 200, f'已下发：去{place}'


def send_command(state, text):
    text = (text or '').strip()
    if not text:
        return 400, '请输入要下发的内容'
    if len(text) > 200:
        return 400, '内容过长，请控制在 200 字以内'
    if not state.voice_running():
        return 409, '带路语音节点未运行，请先启动节点'
    publisher = state.publisher
    if publisher is None:
        return 503, 'ROS 连接尚未就绪，稍后再试'
    try:
        from std_msgs.msg import String
        message = String()
        message.data = text
        with state.publish_lock:
            publisher.publish(message)
    except Exception as exc:
        state.add_log('ERROR', 'monitor.control', f'下发指令失败：{type(exc).__name__}')
        return 503, f'下发失败：{exc}'
    state.add_log('INFO', 'monitor.control', f'已下发指令：{text}')
    return 200, f'已下发：{text}'


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, status, body, content_type='application/json; charset=utf-8'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' blob: data:; connect-src 'self'; frame-ancestors 'self'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        parsed = urlparse(self.path)
        state = self.server.state
        if parsed.path == '/api/status':
            return self.send(200, json.dumps(state.snapshot(), allow_nan=False).encode())
        if parsed.path == '/api/logs':
            with state.lock:
                logs = list(state.logs)
            return self.send(200, json.dumps({'logs': logs}, ensure_ascii=False).encode())
        if parsed.path == '/api/frame':
            kind = parse_qs(parsed.query).get('kind', ['color'])[0]
            with state.lock:
                frame = state.frames.get(kind)
            if not frame or time.time()-frame['at'] > 3:
                return self.send(503, b'{"error":"no_fresh_frame"}')
            return self.send(200, frame['jpeg'], 'image/jpeg')
        files = {'/': ('index.html', 'text/html; charset=utf-8'), '/index.html': ('index.html', 'text/html; charset=utf-8'), '/live.js': ('live.js', 'application/javascript; charset=utf-8'), '/schematic_base.png': ('schematic_base.png', 'image/png'), '/map.png': ('map.png', 'image/png'), '/favicon.ico': (None, '')}
        if parsed.path not in files:
            return self.send(404, b'{"error":"not_found"}')
        filename, mime = files[parsed.path]
        if filename is None:
            return self.send(204, b'')
        path = ROOT/filename
        if not path.exists():
            return self.send(404, b'{"error":"not_found"}')
        return self.send(200, path.read_bytes(), mime)

    def do_POST(self):
        parsed = urlparse(self.path)
        state = self.server.state
        routes = {'/api/node/start': lambda: start_voice(state),
                  '/api/node/stop': lambda: stop_voice(state),
                  '/api/navigate': lambda: send_navigation(state, str(body.get('place') or '')),
                  '/api/command': lambda: send_command(state, str(body.get('text') or ''))}
        if parsed.path not in routes:
            return self.send(404, b'{"error":"not_found"}')
        try:
            length = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(length) or b'{}') if length else {}
        except (ValueError, json.JSONDecodeError):
            return self.send(400, json.dumps({'ok': False, 'message': '请求格式错误'}).encode())
        if not isinstance(body, dict):
            return self.send(400, json.dumps({'ok': False, 'message': '请求格式错误'}).encode())
        if body.get('confirm') is not True:
            return self.send(400, json.dumps({'ok': False, 'message': '缺少确认标记'}).encode())
        status, message = routes[parsed.path]()
        return self.send(status, json.dumps({'ok': status == 200, 'message': message}, ensure_ascii=False).encode())
    do_PUT = do_DELETE = do_PATCH = lambda self: self.send(405, b'{"error":"method_not_allowed"}')


def map_calibration_worker(state):
    last = object()
    while not state.stopping.is_set():
        info = load_map_calibration()
        if info != last:
            state.map_info = info
            last = info
            if info:
                state.add_log('INFO', 'monitor.map', '地图标定已加载' if info.get('available') else '地图标定已加载，等待 map.png')
            else:
                state.add_log('WARN', 'monitor.map', '缺少 map.png 或 map_calib.json，地图页暂不显示位置')
        state.stopping.wait(3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--bind', default='0.0.0.0')
    args = parser.parse_args()
    state = State()
    state.map_info = load_map_calibration()
    state.places = load_places()
    if state.places:
        state.add_log('INFO', 'monitor', '地点配置已加载：' + '、'.join(item['name'] for item in state.places))
    else:
        state.add_log('WARN', 'monitor', '未读到 places.yaml，快捷导航不可用')
    server = Server((args.bind, args.port), Handler)
    server.state = state
    threads = [threading.Thread(target=target, args=(state,), daemon=True) for target in (host_worker, ros_supervisor, chassis_battery_worker, chassis_pose_worker, chassis_poi_worker, map_calibration_worker)]
    for thread in threads:
        thread.start()
    def stop(*unused):
        state.stopping.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(f'Robot dashboard listening on {args.bind}:{args.port}', flush=True)
    try:
        server.serve_forever()
    finally:
        state.stopping.set()
        server.server_close()
        for thread in threads:
            thread.join(timeout=5)


if __name__ == '__main__':
    main()
