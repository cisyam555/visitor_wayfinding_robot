#!/usr/bin/env python3
"""语音导览节点：语音/文本 -> 大模型理解 -> 定点导航 + TTS 播报。

订阅：
  /audio_asr/iat   lyre_msgs/AsrIat   平台语音识别结果（需先唤醒）
  /audio_asr/keyword lyre_msgs/AsrKeyword 唤醒词
  /tour/chat       std_msgs/String    文本直连，没有麦克风时测试用

注意：平台语音识别默认关闭，节点会以 0.5s 周期发布
  /intelligent_interaction/lyre/enable = true
停发就会被关回去；退出时发 false 还原。唤醒方式见
~/xos/lyre/share/lyre/config/aiui_params.yaml（当前是 face 人脸唤醒）。

播报：
  /intelligent_interaction/tts/play   interaction_msgs/TtsService（append）

说话方式：
  「带我去乐知教育」「去厕所」   -> 直接带路
  「我想练跆拳道」等间接表达     -> 大模型推断地点后询问是否带路
  「没事了」                     -> 回待命点（config/places.yaml 的 standby）
  其它任何话                     -> 大模型闲聊

用法：
  ros2 run learn3_d tour_chat
  ros2 topic pub --once /tour/chat std_msgs/msg/String "{data: '带我去创想谷'}"
"""

from __future__ import annotations

import difflib
import json
import math
import os
import fcntl
import queue
import re
import threading
import time
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from interaction_msgs.srv import InstructionService, TtsService
from lyre_msgs.msg import AsrIat, AsrKeyword
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from learn3_d.mic_asr import MicAsr
from learn3_d.nav_core import Navigator
from learn3_d.nav_route import DEFAULT_AUDIO_ID
from learn3_d.places import PlaceBook
from learn3_d.front_guard import GuardNode
from learn3_d.qwen import QwenClient

DEFAULT_SYSTEM_PROMPT = (
    "你的名字固定叫「小静」，不管谁问都不能改成别的名字。"
    "你是展厅里的导览机器人，正在接待访客。"
    "你可以像正常人一样自由聊天：闲聊、讲笑话、回答问题都可以，不要只聊展厅相关的话题。"
    "用简短、自然的中文口语回答，一般一到三句话，不要使用列表、编号或markdown。"
    "不知道的事情就老实说不知道，不要编造。"
)
_AUTO_HOME = object()  # 队列哨兵：送达后超时自动回待命点
# 平台 TTS 合成失败时会播固定提示音，麦克风会听到，不能当成用户说话
_PLATFORM_PROMPTS = ("音频生成失败",)
_LLM_FALLBACK = "不好意思，我没太听清，您再说一遍好吗？"
# 模型服务连不上时不要说成「没听清」，避免让人以为是麦克风问题
_LLM_DOWN = "我这边暂时联系不上大脑，请稍后再试"


def _llm_error_text(exc) -> str:
    """按失败原因选兜底话术：网络/超时/服务端错误 -> 联系不上，其它 -> 没听清。"""
    text = str(exc).lower()
    markers = (
        "timed out", "timeout", "urlopen", "connection", "network",
        "unreachable", "refused", "reset by peer", "http 5", "http 429",
    )
    return _LLM_DOWN if any(marker in text for marker in markers) else _LLM_FALLBACK


DEFAULT_GREET = (
    "您好呀，我是导览机器人小静！"
    "想去哪个点位看看？说一声，我这就带您过去。"
)


class Speech:
    """平台 TTS 播报：append 后轮询到播完，让下一条话不会互相打断。

    节点由 executor 自旋，所以这里用异步调用 + 事件等待，
    say() 可以安全地在工作线程里阻塞调用。
    """

    def __init__(self, node, service, audio_id=DEFAULT_AUDIO_ID,
                 enabled=True, voice_id="", voice_default=""):
        self._node = node
        self._log = node.get_logger()
        self._audio_id = audio_id
        self._enabled = enabled
        self._client = (node.create_client(TtsService, service)
                        if enabled else None)
        self._voice_id = (voice_id or "").strip()
        self._voice_default = (voice_default or "").strip()
        self._voice_retry_at = 0.0
        # 同一时刻只允许一句播报：否则两个线程会交错切音色，
        # 出现一句女声一句男声，甚至互相把对方的话当输入。
        self._say_lock = threading.Lock()
        self._voice_client = (
            node.create_client(
                InstructionService,
                "/intelligent_interaction/tts/update_voice_id")
            if enabled and self._voice_id else None)
        self.busy = False

    def _set_voice(self, voice):
        """切换全局音色。播报前切女声，播完切回默认，避免影响别的项目。"""
        if not self._voice_client or not voice:
            return
        # 音色服务不可用时跳过一段时间，避免每句话都白等
        if time.monotonic() < self._voice_retry_at:
            return
        if not self._voice_client.wait_for_service(timeout_sec=1.0):
            self._voice_retry_at = time.monotonic() + 60.0
            self._log.warn("音色服务未就绪，60 秒内不再尝试切换")
            return
        request = InstructionService.Request()
        request.cmd = voice
        done = threading.Event()
        future = self._voice_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        # 服务偶尔要 2~4 秒才响应，等久一点，避免误判失败后整分钟都是男声
        if not done.wait(5.0):
            self._voice_retry_at = time.monotonic() + 60.0
            self._log.warn(f"切换音色 {voice} 超时，60 秒内不再尝试")
            return
        try:
            if not future.result().success:
                self._voice_retry_at = time.monotonic() + 60.0
                self._log.warn(f"切换音色 {voice} 被拒绝，60 秒内不再尝试")
        except Exception as exc:  # noqa: BLE001
            self._voice_retry_at = time.monotonic() + 60.0
            self._log.warn(f"切换音色 {voice} 异常：{exc}")

    def say(self, text):
        text = (text or "").strip()
        if not text or not self._enabled:
            return False
        with self._say_lock:
            if not self._client.wait_for_service(timeout_sec=2.0):
                self._log.warn("TTS 服务未就绪，跳过播报")
                return False
            self._log.info(f"播报：{text}")
            self.busy = True
            try:
                self._set_voice(self._voice_id)
                response = self._call("append", text=text)
                if response is None or not response.success:
                    self._log.warn(f"播报下发失败：{response}")
                    return False
                self._wait_playback(text)
                return True
            finally:
                self._set_voice(self._voice_default)
                self.busy = False

    def stop_async(self):
        """打断当前播报。必须放到独立线程里发，避免卡住 executor。"""
        if not self._enabled:
            return
        threading.Thread(target=self._call, args=("stop",),
                         daemon=True).start()

    def _call(self, cmd, text=""):
        request = TtsService.Request()
        request.cmd = cmd
        request.type = "text" if text else ""
        request.text = text
        request.audio_id = self._audio_id
        request.extra = ""

        done = threading.Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.0):
            self._log.warn(f"TTS {cmd} 调用超时")
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001 - 播报失败不打断对话
            self._log.warn(f"TTS {cmd} 调用异常：{exc}")
            return None

    def _wait_playback(self, text):
        """轮询 TTS 状态，等这句话播完再处理下一条输入。"""
        started = time.monotonic()
        est = max(1.5, len(text) * 0.22)
        limit = min(est * 2.0 + 5.0, 40.0)
        seen_playing = False
        idle_since = None
        while time.monotonic() - started < limit:
            response = self._call("query")
            playing = bool(
                response is not None
                and response.success
                and response.status == "playing"
            )
            now = time.monotonic()
            if playing:
                seen_playing = True
                idle_since = None
            elif idle_since is None:
                idle_since = now
            elif seen_playing and now - idle_since >= 0.4:
                return
            elif not seen_playing and now - started >= est + 3.0:
                return
            time.sleep(0.2)


def _wrap_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]，用于比较转身前后的朝向差。"""
    return math.atan2(math.sin(angle), math.cos(angle))


_NAV_FAIL_TEXT = {
    "localization": "定位还没恢复，我先不走，稍等一下再试",
    "health": "底盘状态有问题，暂时不能移动",
    "blocked": "前面被挡住了，我先停一下",
    "network": "我连不上底盘了，请稍后再试",
    "timeout": "走了太久还没到，我先停下",
    "failure": "这次没能走到目的地",
}


class RouteNavigator:
    """把 nav_core.Navigator 包成 tour_chat 需要的同步接口。

    Navigator 按 places.yaml 的主线逐点下发 move_to，并在每段前检查
    定位与底盘健康、到点做位置校验；这里只做接口适配和异常转换。
    """

    def __init__(self, node, host, rest_port, map_name, timeout,
                 direct=True, guard=None):
        self._log = node.get_logger()
        self._map_name = str(map_name)
        self._timeout = float(timeout)
        self._cancelled = False
        self._guard = guard
        self._nav = Navigator(
            host=str(host), port=int(rest_port),
            timeout=self._timeout, logger=self._log, direct=direct)

    def set_guard(self, guard):
        """设置前向防撞对象；None 表示不使用防撞。"""
        self._guard = guard

    @property
    def map_name(self):
        return self._map_name

    def connect(self):
        return self

    def set_route(self, main_line, branches):
        self._nav.set_route(main_line, branches)

    def poi_names(self):
        return sorted(self._nav.poi_names())

    def pose(self):
        return self._nav.pose()

    def forward_yaw(self, poi):
        """主线上正向（上一站 -> 本站）进入该点位的朝向；取不到返回 None。"""
        return self._nav.poi_approach_yaw(poi)

    def check_ready(self):
        return self._nav.check_ready()

    def warm_localization(self):
        """空闲/被挡等待时预热定位，恢复导航时少等 3 秒。"""
        self._nav.warm_localization()

    def goto(self, name, timeout=None):
        """返回 success / blocked；其它失败转成异常交给上层提示。"""
        self._cancelled = False
        limit = self._timeout if timeout is None else float(timeout)
        result = self._nav.go(name, limit, guard=self._guard)
        if result in ("success", "blocked"):
            return result
        if result == "cancelled" or self._cancelled:
            self._nav.clear_cancel()
            return "cancelled"
        raise RuntimeError(_NAV_FAIL_TEXT.get(result, f"导航未完成：{result}"))

    def cancel(self):
        self._cancelled = True
        return self._nav.cancel()

    def turn(self, angle_deg, timeout=25.0):
        """原地转身，正为左转、负为右转；转身前确认正前方无人。

        转完用定位 yaw 校验实际角度，偏差超过 15° 补转一次。
        """
        if abs(angle_deg) < 1.0:
            return True
        if not self._wait_front_before_turn():
            self._log.warn("转身前正前方一直有人，暂不转身")
            return False
        target = math.radians(angle_deg)
        before = self._yaw_of(self.pose())
        result = self._nav.rotate_by(target, timeout_s=timeout)
        if result != "success":
            self._log.warn(f"原地转身 {angle_deg:+.0f}° 未完成：{result}")
            return False
        after = self._yaw_of(self.pose())
        if before is None or after is None:
            return True
        actual = _wrap_angle(after - before)
        remaining = _wrap_angle(target - actual)
        if abs(math.degrees(remaining)) > 15.0:
            self._log.warn(
                f"转身实际 {math.degrees(actual):+.0f}°，"
                f"补转 {math.degrees(remaining):+.0f}°")
            return self._nav.rotate_by(
                remaining, timeout_s=timeout) == "success"
        return True

    @staticmethod
    def _yaw_of(pose):
        try:
            return float(pose["yaw"])
        except (TypeError, KeyError, ValueError):
            return None

    def _wait_front_before_turn(self, timeout=4.0):
        """转身前等正前方清空，避免旁边有人时擦碰。

        到达点位时访客通常就站在正前方，所以不能用直行的 0.5m 阈值，
        这里只看「贴得特别近」的人（0.35m 内），否则转身永远会被挡掉。
        """
        guard = self._guard
        if guard is None:
            return True
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                distance = guard.distance()
                limit = min(0.35, float(guard.stop_distance()))
            except Exception:  # noqa: BLE001 - 读不到深度就按可转处理
                return True
            if distance is None:
                return True
            last = distance
            if distance >= limit:
                return True
            time.sleep(0.3)
        self._log.warn(
            f"转身前正前方仍有 {last:.2f}m 障碍（阈值 {limit:.2f}m），"
            f"{timeout:.0f} 秒内未让开")
        return False

    def was_cancelled(self):
        return self._cancelled

    def shutdown(self):
        self._nav.cancel()


class TourChat(Node):
    def __init__(self):
        super().__init__("tour_chat")

        self.declare_parameter("asr_topic", "/audio_asr/iat")
        self.declare_parameter("keyword_topic", "/audio_asr/keyword")
        self.declare_parameter("enable_asr", True)
        self.declare_parameter("enable_wakeword", True)
        self.declare_parameter("wake_word", "小静")
        self.declare_parameter("enable_topic",
                               "/intelligent_interaction/lyre/enable")
        self.declare_parameter("chat_topic", "/tour/chat")
        # 本地麦克风识别：不依赖唤醒词，直接听人说话
        self.declare_parameter("enable_mic", True)
        self.declare_parameter("mic_topic", "/lyre/audio_stream")
        self.declare_parameter("mic_control_service", "/lyre/audio_control")
        self.declare_parameter("mic_rate_in", 96000)
        self.declare_parameter("vad_threshold", 600.0)
        self.declare_parameter("vad_silence_s", 0.55)
        self.declare_parameter("asr_model", "small")
        self.declare_parameter("asr_compute_type", "int8")
        self.declare_parameter("asr_language", "zh")
        self.declare_parameter("asr_beam_size", 1)
        self.declare_parameter("asr_max_speech", 6.0)
        self.declare_parameter(
            "asr_hotwords",
            "乐知教育、核心力量跑酷、创想谷、厕所、洗手间、卫生间、"
            "跆拳道馆、带我去、我想去、回到、跑酷、乐知、创想、跆拳道、"
            "停一下、停下来、停下、别动、不要动、站住、等一下、回去")
        self.declare_parameter(
            "asr_initial_prompt",
            "以下是普通话展厅导览对话。常见指令：带我去创想谷、我要去厕所、"
            "停一下、停下来、别动、回去、回乐知教育。地点有乐知教育、"
            "核心力量跑酷、创想谷、厕所、跆拳道馆。")
        self.declare_parameter("tts_service", "/intelligent_interaction/tts/play")
        self.declare_parameter("audio_id", DEFAULT_AUDIO_ID)
        # 本项目的专属音色：报播时切过去，播完恢复平台默认，避免影响别的项目
        self.declare_parameter("voice_id", "tiangong_v4")
        self.declare_parameter("voice_default", "tiangong_v2")
        self.declare_parameter("enable_voice", True)
        self.declare_parameter("enable_greet", False)
        self.declare_parameter("greet_text", DEFAULT_GREET)
        # 有人靠近就主动问候：用头部深度图测正前方距离
        self.declare_parameter("enable_presence", True)
        self.declare_parameter("presence_distance", 0.8)
        self.declare_parameter("presence_confirm", 3)
        self.declare_parameter("greet_cooldown", 20.0)
        self.declare_parameter("depth_topic", "/ob_camera_head/depth/image_raw")
        self.declare_parameter("min_chars", 2)
        self.declare_parameter("dedupe_window", 2.0)

        self.declare_parameter("qwen_base_url", "http://120.48.75.178:7006/v1")
        self.declare_parameter("qwen_model", "Qwen3.5-122B-A10B")
        self.declare_parameter("qwen_api_key", "EMPTY")
        self.declare_parameter("qwen_timeout", 30.0)
        self.declare_parameter("history_turns", 6)
        self.declare_parameter("system_prompt", DEFAULT_SYSTEM_PROMPT)

        self.declare_parameter("enable_nav", True)
        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("map", "消防")
        self.declare_parameter("reloc_poi", "乐知教育")
        self.declare_parameter("track", "")
        self.declare_parameter("nav_timeout", 180.0)
        self.declare_parameter("nav_direct", True)
        self.declare_parameter("await_timeout", 35.0)
        self.declare_parameter("pause_timeout", 35.0)
        self.declare_parameter("pending_timeout", 90.0)
        self.declare_parameter("obstacle_text", "请让一让，我正在工作")
        self.declare_parameter("obstacle_home_text", "请让一让我正在回家")
        self.declare_parameter("obstacle_resume_text", "障碍物已移开，我继续导览")
        self.declare_parameter("obstacle_home_resume_text", "障碍物已移除，我继续回家")
        self.declare_parameter("front_guard_enabled", True)
        self.declare_parameter("front_guard_distance", 0.5)
        self.declare_parameter("obstacle_repeat", 15.0)
        # 到点后原地转身：格式 "点位:left,点位:right"，转完播报，
        # 播报结束后停 arrival_turn_hold 秒（默认 2）再转回来
        self.declare_parameter(
            "arrival_turns",
            "核心力量跑酷:right,创想谷:left,厕所:left,跆拳道馆:left")
        self.declare_parameter("arrival_turn_angle", 90.0)
        self.declare_parameter("arrival_turn_hold", 2.0)
        self.declare_parameter("places_file", "")
        self.declare_parameter("log_file", "")

        self._min_chars = int(self.get_parameter("min_chars").value)
        self._dedupe_window = float(self.get_parameter("dedupe_window").value)
        path = str(self.get_parameter("log_file").value or "").strip()
        if not path:
            path = os.path.join(os.path.expanduser("~"), "guide_logs",
                                time.strftime("tour_chat_visitor_%Y%m%d.log"))
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except OSError:
            pass
        self._log_file = path
        self._queue: queue.Queue = queue.Queue()
        self._closing = False
        self._last_key = ""
        self._last_at = 0.0
        # VAD 可能把「带我去 + 创想谷」切成两句，先存半句等下一句拼起来
        self._prefix = ""
        self._prefix_at = 0.0
        self._prefix_lock = threading.Lock()
        self._navigating = False
        self._pending = None
        self._awaiting = False
        self._awaiting_at = 0.0
        self._retarget = None
        self._pause_requested = False
        self._pause_kind = "ask"
        self._paused_target = None  # 暂停交流时保留的原目标
        self._paused_standby = False  # 暂停的是回待命点任务
        self._paused_at = 0.0      # 暂停后最后一次有人说话的时间
        # 正在做到达播报 / 已经有人在到达时喊「回去吧」
        self._nav_standby = False
        self._arriving = False
        self._home_requested = False
        self._pause_timeout = float(
            self.get_parameter("pause_timeout").value)
        self._pending_at = 0.0     # 提议地点的时间，过期就不认「好的」
        self._pending_timeout = float(
            self.get_parameter("pending_timeout").value)
        self._await_timeout = float(
            self.get_parameter("await_timeout").value)
        self._obstacle_text = str(
            self.get_parameter("obstacle_text").value or "请让一让，我正在工作")
        self._obstacle_home_text = str(
            self.get_parameter("obstacle_home_text").value
            or "请让一让我正在回家")
        self._obstacle_resume_text = str(
            self.get_parameter("obstacle_resume_text").value
            or "障碍物已移开，我继续导览")
        self._obstacle_home_resume_text = str(
            self.get_parameter("obstacle_home_resume_text").value
            or "障碍物已移除，我继续回家")
        self._obstacle_repeat = max(
            5.0, float(self.get_parameter("obstacle_repeat").value))
        self._arrival_turn_angle = float(
            self.get_parameter("arrival_turn_angle").value)
        self._arrival_turn_hold = max(
            0.0, float(self.get_parameter("arrival_turn_hold").value))
        self._arrival_turns = self._parse_arrival_turns(
            str(self.get_parameter("arrival_turns").value or ""))
        self._home_yaw = None       # 离开待命点时的朝向，回来要转回去
        self._here = ""            # 当前所在点位，供闲聊注入
        self._llm_down_until = 0.0  # 模型刚失败过就短暂停用，避免连等两次超时
        self._llm_down_text = _LLM_DOWN  # 冷却期内按上次失败原因播兜底话术
        self._last_spoken = ""
        self._speech_until = 0.0
        self._spoken_history = deque(maxlen=8)

        self._qwen = QwenClient(
            base_url=str(self.get_parameter("qwen_base_url").value),
            model=str(self.get_parameter("qwen_model").value),
            api_key=str(self.get_parameter("qwen_api_key").value),
            timeout=float(self.get_parameter("qwen_timeout").value),
            system_prompt=str(self.get_parameter("system_prompt").value),
            history_turns=int(self.get_parameter("history_turns").value),
        )
        self._speech = Speech(
            self,
            str(self.get_parameter("tts_service").value),
            audio_id=str(self.get_parameter("audio_id").value),
            enabled=bool(self.get_parameter("enable_voice").value),
            voice_id=str(self.get_parameter("voice_id").value),
            voice_default=str(self.get_parameter("voice_default").value),
        )

        # 本地麦克风识别：连续收音，不需要先喊唤醒词
        self._mic = None
        if bool(self.get_parameter("enable_mic").value):
            try:
                self._mic = MicAsr(
                    self, self._on_mic_text,
                    mic_topic=str(self.get_parameter("mic_topic").value),
                    control_service=str(
                        self.get_parameter("mic_control_service").value),
                    rate_in=int(self.get_parameter("mic_rate_in").value),
                    vad_threshold=float(
                        self.get_parameter("vad_threshold").value),
                    silence_s=float(
                        self.get_parameter("vad_silence_s").value),
                    model_name=str(self.get_parameter("asr_model").value),
                    compute_type=str(
                        self.get_parameter("asr_compute_type").value),
                    language=str(self.get_parameter("asr_language").value),
                    hotwords=str(self.get_parameter("asr_hotwords").value),
                    initial_prompt=str(
                        self.get_parameter("asr_initial_prompt").value),
                    beam_size=int(
                        self.get_parameter("asr_beam_size").value),
                    max_speech_s=float(
                        self.get_parameter("asr_max_speech").value),
                )
                self._mic.start()
            except Exception as exc:  # noqa: BLE001 - 麦克风不可用仍保留平台语音
                self._mic = None
                self.get_logger().error(f"本地麦克风识别启动失败：{exc}")

        self._nav = None
        self._places = None
        if bool(self.get_parameter("enable_nav").value):
            self._setup_navigation()

        self.create_subscription(
            AsrIat, str(self.get_parameter("asr_topic").value),
            self._on_asr, 10)
        self.create_subscription(
            AsrKeyword, str(self.get_parameter("keyword_topic").value),
            self._on_keyword, 10)
        self.create_subscription(
            String, str(self.get_parameter("chat_topic").value),
            self._on_chat, 10)

        # 平台 ASR 默认是关的（asr open status:0），必须持续使能，
        # 一旦停发就会被关回去，表现为「说话没有任何反应」。
        self._enable_pub = None
        if bool(self.get_parameter("enable_asr").value):
            self._enable_pub = self.create_publisher(
                Bool, str(self.get_parameter("enable_topic").value), 10)
            self.create_timer(0.5, self._publish_enable)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        if bool(self.get_parameter("enable_wakeword").value):
            threading.Thread(target=self._setup_wakeword, daemon=True).start()
        if self._await_timeout > 0 or self._pause_timeout > 0:
            threading.Thread(target=self._await_loop, daemon=True).start()

        # 深度测距：既用于靠近问候，也用于导航中的前向防撞
        self._guard = None
        want_presence = bool(self.get_parameter("enable_presence").value)
        want_guard = (bool(self.get_parameter("front_guard_enabled").value)
                      and self._nav is not None)
        if want_presence or want_guard:
            try:
                self._guard = GuardNode(
                    str(self.get_parameter("depth_topic").value),
                    float(self.get_parameter("front_guard_distance").value))
                if want_presence:
                    threading.Thread(target=self._presence_loop,
                                     daemon=True).start()
            except Exception as exc:  # noqa: BLE001 - 没有深度也不影响对话
                self.get_logger().error(
                    f"深度测距不可用（只保留对话）：{exc}")
        if self._nav is not None:
            self._nav.set_guard(self._guard if want_guard else None)

        greet = ""
        if bool(self.get_parameter("enable_greet").value):
            try:
                greet = str(self.get_parameter("greet_text").value or "").strip()
            except Exception:  # noqa: BLE001 - 空字符串覆盖时参数未初始化
                greet = ""
        if greet:
            threading.Thread(target=self._greet, args=(greet,),
                             daemon=True).start()

        self.get_logger().info(
            f"导览对话已就绪：导航={'开' if self._nav else '关'}，"
            f"模型={self.get_parameter('qwen_model').value}")

    # ------------------------------------------------------------- 初始化
    def _setup_navigation(self):
        try:
            path = str(self.get_parameter("places_file").value).strip()
            if not path:
                path = os.path.join(
                    get_package_share_directory("learn3_d"),
                    "config", "places.yaml")
            self._nav = RouteNavigator(
                self,
                host=str(self.get_parameter("host").value),
                rest_port=int(self.get_parameter("rest_port").value),
                map_name=str(self.get_parameter("map").value),
                timeout=float(self.get_parameter("nav_timeout").value),
                direct=bool(self.get_parameter("nav_direct").value),
            )
            self._nav.connect()
            self._places = PlaceBook.load(
                path, self._nav.poi_names(), self.get_logger())
            # 主线顺序交给导航器：逐段下发 move_to，不跳点
            self._nav.set_route(self._places.main_line,
                                self._places.branches)
            self.get_logger().info(f"地点配置：{path}")
            self.get_logger().info(f"可去的地点：{self._places.describe()}")
            # 兜底记录待命朝向：开机时可能不在待命点，先记下当前朝向，
            # 免得第一次送客前没记到，回程时无处可转。
            threading.Thread(target=self._remember_home_yaw,
                             daemon=True).start()
        except Exception as exc:  # noqa: BLE001 - 导航不可用时只保留聊天
            self._nav = None
            self._places = None
            self.get_logger().error(f"导航初始化失败（只保留聊天）：{exc}")

    # ------------------------------------------------------------- 订阅回调
    def _publish_enable(self) -> None:
        """持续打开平台语音识别，停发即自动关闭。"""
        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)

    def _setup_wakeword(self) -> None:
        """平台重启后唤醒词列表会清空，启动时补登记一次，语音唤醒才可用。"""
        word = str(self.get_parameter("wake_word").value or "").strip()
        if not word:
            return
        try:
            from lyre_msgs.srv import WakeWordControl
        except Exception as exc:  # noqa: BLE001 - 没有平台语音时只保留文字
            self.get_logger().warn(f"唤醒词服务不可用：{exc}")
            return
        client = self.create_client(
            WakeWordControl, "/audio_aiui/wake_word_control")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("唤醒词服务未就绪，跳过登记")
            return

        def call(kind):
            request = WakeWordControl.Request()
            request.type = kind
            request.word = word
            done = threading.Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _: done.set())
            if not done.wait(3.0):
                return None
            try:
                return future.result()
            except Exception:  # noqa: BLE001
                return None

        listed = call(WakeWordControl.Request.TYPE_LIST)
        if listed is None or not listed.success:
            self.get_logger().warn(f"读取唤醒词失败：{listed}")
            return
        if word not in list(listed.words or []):
            added = call(WakeWordControl.Request.TYPE_ADD)
            if added is None or not added.success:
                self.get_logger().warn(f"登记唤醒词失败：{added}")
                return
        if str(listed.current_word or "") != word:
            picked = call(WakeWordControl.Request.TYPE_SET_CURRENT)
            if picked is None or not picked.success:
                self.get_logger().warn(f"切换唤醒词失败：{picked}")
                return
        self.get_logger().info(f"语音唤醒词已就绪：{word}")

    def _on_keyword(self, msg: AsrKeyword) -> None:
        text = str(msg.keyword or "").strip()
        if text:
            self.get_logger().info(
                f"唤醒词：{text}（声源角度 {msg.angle}°），请继续说")

    def _on_asr(self, msg: AsrIat) -> None:
        self._enqueue(msg.text or "", "听到(平台)")

    def _on_mic_text(self, text: str) -> None:
        """本地麦克风识别结果：和平台识别走同一条入口，重复会去重。"""
        self._enqueue(text or "", "听到(麦克风)")

    def _on_chat(self, msg: String) -> None:
        self._enqueue(msg.data or "", "文本输入")

    def _enqueue(self, text: str, source: str) -> None:
        text = (text or "").strip()
        if not _meaningful(text, self._min_chars):
            return
        # 平台自己的合成失败提示音，不是访客在说话
        if any(prompt in text for prompt in _PLATFORM_PROMPTS):
            self.get_logger().info(f"忽略平台提示音：{text}")
            self._file_log("忽略(平台提示)", text)
            return
        # 喇叭声音被自己麦克风收回来时，不能当成用户指令。
        # 用最近 25 秒的播报历史比对：识别有延迟，只听最后一句会漏掉回声。
        if self._is_recent_echo(text):
            self.get_logger().info(f"忽略自己的播报回声：{text}")
            self._file_log("忽略(回声)", text)
            return
        # 「带我去」和「创想谷」被 VAD 切成两句时，先等下一句拼起来
        text = self._merge_prefix(text)
        if not text:
            return
        # 导航中只放行急停和改目的地，其它一律不理。
        # 到点转身/等待时 worker 还卡在到达流程里，指令也必须在这里
        # 直接处理，否则「停一下」会排到 10 秒后，表现为喊停不停。
        if self._navigating or self._arriving:
            # 先说清听到了什么，避免只看到「改道/忽略」无法回溯
            self._file_log(
                f"{source}({'导航中' if self._navigating else '到达中'})", text)
            action = self._interrupt(text)
            if action in ("retarget", "stop", "pause", "handled"):
                if self._speech.busy:
                    # 观众在播报中下指令：立刻掐掉当前这句，别让他等
                    self._speech.stop_async()
                if self._arriving and action in ("stop", "pause"):
                    self._speak_async("好的，已经停下了，您说")
                return
            if action is not None:  # replay_*：停下后还要接着处理这句话
                self._queue.put(text)
                return
            if self._navigating:
                self.get_logger().info(f"导航中忽略：{text}")
                self._file_log("忽略(导航中)", text)
                return
            # 到达流程中的闲聊/提问：等这段流程结束再处理，不能丢
        elif self._speech.busy or time.monotonic() < self._speech_until:
            # 播报中不理闲聊，但目的地/结束/继续/停车/换地方不能丢，
            # 否则观众在它说话时说的指令会被直接吞掉。
            plain = _plain(text)
            if (self._place_request(text) is None
                    and self._bare_place(text) is None
                    and not _wants_done(text)
                    and not _wants_go_home(plain)
                    and not _wants_resume(plain)
                    and not _is_stop(plain)
                    and not _wants_change_place(plain)):
                self.get_logger().info(f"忙（播报）中忽略：{text}")
                self._file_log("忽略(忙)", text)
                return
            self.get_logger().info(f"播报中收到指令，打断播报：{text}")
            self._speech.stop_async()
        key = re.sub(r"\s+", "", text)
        now = time.monotonic()
        if key == self._last_key and now - self._last_at < self._dedupe_window:
            return
        self._last_key, self._last_at = key, now
        if self._awaiting:
            self._awaiting_at = now  # 有人搭话就重新计时
        self.get_logger().info(f"{source}：{text}")
        self._file_log(source, text)
        self._queue.put(text)

    def _merge_prefix(self, text: str) -> str:
        """等半句拼成完整指令；返回空串表示先别处理这句。

        自然说话时「带我去……创想谷」中间常有停顿，VAD 会按时长切成
        两句。如果第一句立刻交给大模型，它会反问「想去哪」，第二句的
        地名就被当成新话题，表现为「识别不准」。
        """
        now = time.monotonic()
        with self._prefix_lock:
            if self._prefix:
                prefix, self._prefix = self._prefix, ""
                if now - self._prefix_at <= _PREFIX_WAIT_S:
                    text = prefix + text
                else:
                    # 等太久没等到地名，按原句交回去，让它自己反问
                    self._queue.put(prefix)
            if _is_go_prefix(text):
                self._prefix = text
                self._prefix_at = now
                return ""
        return text

    def _flush_prefix(self) -> None:
        """一直没有下一句时，把半句交回去，不能让观众干等。"""
        with self._prefix_lock:
            if not self._prefix:
                return
            if time.monotonic() - self._prefix_at < _PREFIX_WAIT_S:
                return
            text, self._prefix = self._prefix, ""
        self._queue.put(text)

    def _place_request(self, text: str):
        """明确的「去某地」指令（短句 + 有去意）才返回地点，否则 None。"""
        if (self._places is None or len(_plain(text)) > 20
                or not _explicit_go(text)):
            return None
        return self._places.resolve(text)

    def _bare_place(self, text: str):
        """导航中只说地名（如「创想谷」「乐知教育吧」）也能改道。"""
        if self._places is None:
            return None
        plain = _plain(text)
        if not 2 <= len(plain) <= 10:
            return None
        place = self._places.resolve(text)
        if place is None:
            return None
        if (self._places.standby is not None
                and place.poi == self._places.standby.poi):
            # 只报待命点名字可能是误识别（例如把「停一下」听成地名），
            # 回程必须说「回去 / 回待命 / 带我回乐知教育」这类明确指令
            return None
        names = {_plain(place.poi), _plain(place.spoken)}
        if plain in names:
            return place
        if any(plain == name + suffix
               for name in names for suffix in ("吧", "啊", "呀", "哦")):
            return place
        return None

    def _interrupt(self, text: str) -> str | None:
        """导航中放行：改目的地、停车、叫住它提问。

        retarget/stop/pause 表示已处理完；replay_* 表示停下后还要
        接着处理这句话；None 表示无关内容，继续导航。
        """
        plain = _plain(text)
        # 到达点位、正在播「我们到 X 了」时，观众说「没事了你回去吧」
        # 不能再当导航噪音丢掉，先记下来，播完立刻回待命点。
        if _wants_go_home(plain):
            if self._arriving:
                self._home_requested = True
                self.get_logger().info("到达播报中收到回程指令")
                self._file_log("回程", text)
                return "handled"
            if self._nav_standby:
                return "handled"  # 已经在回待命点，不必重复下发
            standby = self._places.standby if self._places else None
            if standby is not None:
                self._retarget = standby
                self._pause_requested = False
                self.get_logger().info("导航中要求回待命点")
                self._file_log("改道", f"{text} -> {standby.poi}")
                self._safe_cancel()
                return "retarget"
        place = self._place_request(text)
        if place is None:
            # 返程/送客途中只说「创想谷」也要能改道
            place = self._bare_place(text)
        if place is not None:
            self._retarget = place
            self._pause_requested = False
            self.get_logger().info(f"导航中改去：{place.poi}")
            self._file_log("改道", f"{text} -> {place.poi}")
            if self._arriving:
                # 已经到点位了，不用再取消（取消已结束的任务可能留下
                # 待取消标记，害得下一单导航直接失败）
                return "retarget"
            self._safe_cancel()
            return "retarget"
        if _wants_change_place(plain):
            self._pause_requested = True
            self._pause_kind = "vague"
            self._retarget = None
            self.get_logger().info("观众想换地方，暂停并询问新目标")
            self._file_log("暂停换地方", text)
            self._safe_cancel()
            return "pause"
        if _is_stop(plain):
            self._pause_requested = True
            self._pause_kind = "stop"
            self._retarget = None
            self.get_logger().info("收到停车，取消当前导航")
            self._file_log("暂停", text)
            self._safe_cancel()
            return "stop" if _is_call_only(plain) else "replay_stop"
        if _wants_pause(plain):
            self._pause_requested = True
            self._pause_kind = "ask"
            self._retarget = None
            self.get_logger().info("被访客叫住，暂停导航")
            self._file_log("暂停", text)
            self._safe_cancel()
            return "pause" if _is_call_only(plain) else "replay_pause"
        return None

    def _safe_cancel(self) -> None:
        """取消请求走独立线程，别让 ROS 回调线程等底盘的 HTTP。"""
        def _do():
            try:
                if self._nav is not None:
                    self._nav.cancel()
            except Exception as exc:  # noqa: BLE001 - 取消失败也不能让节点崩
                self.get_logger().error(f"取消导航失败：{exc}")
        threading.Thread(target=_do, daemon=True).start()

    # --------------------------------------------------------------- 工作线程
    def _worker_loop(self) -> None:
        while not self._closing:
            try:
                text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if text is None:
                return
            if text is _AUTO_HOME:
                self._auto_home()
                continue
            try:
                self._handle(text)
            except Exception as exc:  # noqa: BLE001 - 单次失败不影响后续对话
                self.get_logger().error(f"处理失败：{exc}")

    def _handle(self, text: str) -> None:
        # 0) 导航被叫停：先听观众要说什么，再决定继续还是改道
        if (self._nav is not None and not self._navigating
                and self._paused_target is not None):
            self._handle_paused(text)
            return

        if self._nav is not None and not self._navigating:
            request = self._place_request(text)
            place = request if request is not None else self._places.resolve(text)

            # 1) 直接说了地点：带「去/到」就出发，否则先问一句
            if place is not None:
                # 只认短指令，长串闲聊/回声不触发导航
                if request is not None:
                    self._pending = None
                    self._navigate(place)
                    return
                self._pending = place
                self._pending_at = time.monotonic()
                self._speak(f"可以去{place.spoken}，需要我带你去吗？")
                return

            # 2) 上一句提议了地点，这一句是同意还是拒绝
            if self._pending is not None:
                if (self._pending_timeout > 0
                        and time.monotonic() - self._pending_at
                        > self._pending_timeout):
                    self._pending = None  # 提议过期，不能凭一句「好的」就出发
                else:
                    pending, self._pending = self._pending, None
                    if _means_no(text):
                        self._speak("好的，想去的时候随时叫我")
                        return
                    if _means_yes(text):
                        self._navigate(pending)
                        return

            # 3) 到地方后问「还有什么事」，说没事就回待命点；
            #    即使等待计时被重置，只要人还站在点位上，说「你回去吧」也立即回。
            standby = self._places.standby
            at_point = bool(
                standby is not None and self._here
                and self._here != standby.spoken)
            if ((self._awaiting and _wants_done(text))
                    or (at_point and _wants_go_home(_plain(text)))):
                if standby is None:
                    self._awaiting = False
                    self._speak("好的，那我就在这儿等你")
                    return
                self._navigate(standby, standby=True)
                return

            # 3.5) 「我要去别的地方」但没说去哪：直接问，别绕模型
            if _wants_change_place(_plain(text)):
                self._awaiting_at = time.monotonic()
                self._speak("好的，您想去哪个点位？")
                return

            # 4) 没直接说地点（如「我想练跆拳道」），让大模型推断
            suggestion = self._suggest_place(text)
            if suggestion is not None:
                place, reply = suggestion
                self._pending = place
                self._pending_at = time.monotonic()
                self._speak(reply)
                return

        # 5) 都命不中就是闲聊
        if not self._here:
            self._qwen.set_context("你刚启动，还没走到任何点位，不要编造位置。")
        if time.monotonic() < self._llm_down_until:
            self._speak(self._llm_down_text)
            return
        try:
            reply = self._qwen.chat(text)
        except Exception as exc:  # noqa: BLE001 - 大模型不可用时给一句兜底
            self.get_logger().error(f"大模型请求失败：{exc}")
            self._llm_down_text = _llm_error_text(exc)
            self._llm_down_until = time.monotonic() + 15.0
            self._speak(self._llm_down_text)
            return
        self.get_logger().info(f"小静：{reply}")
        self._speak(reply)

    def _handle_paused(self, text: str) -> None:
        """暂停期间：继续/结束/改道优先，其它话回答后追问是否继续。"""
        target = self._paused_target
        plain = _plain(text)
        self._paused_at = time.monotonic()  # 处理输入期间不计时
        if _wants_resume(plain):
            standby, self._paused_standby = self._paused_standby, False
            self._paused_target = None
            # 暂停的是回程任务就继续回待命点，不要当成送客
            self._navigate(target, standby=standby, ready_checked=True)
            return
        # 已经暂停，再喊一次「别走了」就只是确认停下，不能当成「走了」
        if _is_stop(plain):
            self._speak("好的，我不动了，您说")
            self._paused_at = time.monotonic()
            return
        if _wants_done(text) or _wants_go_home(plain):
            self._paused_target = None
            self._paused_standby = False
            standby = self._places.standby if self._places else None
            if standby is None:
                self._speak("好的，那我就在这儿等你")
                return
            self._navigate(standby, standby=True)
            return
        place = self._place_request(text)
        if place is None:
            place = self._bare_place(text)
        if place is not None:
            self._paused_target = None
            self._paused_standby = False
            self._navigate(place, ready_checked=True)
            return

        if _wants_change_place(plain):
            self._speak("好的，您想去哪个点位？")
            self._paused_at = time.monotonic()
            return

        # 提问或闲聊：先回答，再问要不要接着走
        if not self._here:
            self._qwen.set_context("你刚启动，还没走到任何点位，不要编造位置。")
        if time.monotonic() < self._llm_down_until:
            self._speak(self._llm_down_text)
            self._paused_at = time.monotonic()
            return
        try:
            reply = self._qwen.chat(text)
        except Exception as exc:  # noqa: BLE001 - 大模型不可用时给一句兜底
            self.get_logger().error(f"大模型请求失败：{exc}")
            self._llm_down_text = _llm_error_text(exc)
            self._llm_down_until = time.monotonic() + 15.0
            self._speak(self._llm_down_text)
            self._paused_at = time.monotonic()
            return
        self.get_logger().info(f"小静：{reply}")
        self._speak(f"{reply} 还需要继续去{target.spoken}吗？")
        self._paused_at = time.monotonic()  # 回答播完再重新计时

    def _suggest_place(self, text: str):
        """用大模型从「我想练跆拳道」这类话里推断地点，返回 (Place, 回复)。"""
        if not any(word in text for word in _PLACE_HINTS):
            return None
        prompt = (
            f"可去的地点：{self._places.describe()}。\n"
            f"访客说：{text}\n"
            "判断访客是不是想去某个地点，可能用活动间接表达（想练跆拳道 = 去跆拳道馆）。"
            "只能从上面地点里选，不要编造；没把握就留空。"
            '只输出 JSON，不要其他文字：'
            '{"selected": "地点名或空字符串", "reply": "一句口语回复，'
            '选到地点时告诉对方可以去哪里并问要不要带路"}'
        )
        try:
            raw = self._qwen.one_shot(
                prompt, system="你是意图解析器，只输出 JSON，不解释。")
        except Exception as exc:  # noqa: BLE001 - 解析失败就当普通聊天
            self.get_logger().warn(f"地点意图识别失败：{exc}")
            self._llm_down_text = _llm_error_text(exc)
            self._llm_down_until = time.monotonic() + 15.0
            return None

        data = _extract_json(raw)
        if not isinstance(data, dict):
            return None
        selected = str(data.get("selected") or "").strip()
        if not selected:
            return None
        place = self._places.resolve(selected)
        if place is None:
            return None
        reply = str(data.get("reply") or "").strip()
        self.get_logger().info(f"推断地点：{selected} -> {place.poi}")
        return place, (reply or f"可以去{place.spoken}，需要我带你去吗？")

    # ------------------------------------------------------------------ 导航
    def _set_here(self, spoken: str) -> None:
        """记录当前点位，并告诉模型自己站在哪。"""
        self._here = spoken
        self._qwen.set_context(
            f"你此刻停在「{spoken}」这个点位，访客就在你面前。"
            "被问到「这是哪」时直接回答这个点位名，不要编造别的地点。")

    def _wait_front_clear(self, timeout: float) -> bool:
        """等前方连续 1.5 秒清空；期间允许观众改道/叫停。"""
        guard = self._guard
        if guard is None:
            return True
        deadline = time.monotonic() + timeout
        clear_since = None
        next_warm = 0.0
        while time.monotonic() < deadline:
            if self._nav is None or self._nav.was_cancelled():
                return False
            # 等路通的这段时间顺手预热定位，让恢复后不用再等 3 秒
            now = time.monotonic()
            if now >= next_warm:
                next_warm = now + 0.5
                self._nav.warm_localization()
            if guard.clear():
                clear_since = clear_since or time.monotonic()
                if time.monotonic() - clear_since >= 1.0:
                    return True
            else:
                clear_since = None
            time.sleep(0.2)
        return False

    def _run_goto(self, poi: str, standby: bool) -> str:
        """和 learn1_D 一致：能绕就让底盘自己绕，绕不过去才停+播报。

        防撞距离 0.5 米，正常避障交给底盘；真的被挡死（人站在车前）时
        立刻播报让行话术，等障碍移开后续行并播报续行话术。
        **不会放弃目标点**，也不会因为被挡次数多就不说话：每被挡一次都喊，
        一直不走就每隔 obstacle_repeat_s 秒再喊一次，直到路通继续。
        """
        yield_text = self._obstacle_home_text if standby else self._obstacle_text
        resume_text = (self._obstacle_home_resume_text if standby
                       else self._obstacle_resume_text)
        repeat_s = self._obstacle_repeat
        blocks = 0
        while True:
            result = self._nav.goto(poi)
            if result == "cancelled" or self._nav.was_cancelled():
                self._nav.clear_cancel()
                return "cancelled"
            if result != "blocked":
                return result
            blocks += 1
            # 绕不过去：立刻播报，和 learn1_D 的 on_blocked 一致
            self.get_logger().info(
                f"前方被挡，第 {blocks} 次让行，播报：{yield_text}")
            self._file_log("障碍", f"{poi} 第 {blocks} 次被挡，播报：{yield_text}")
            # 让行提示优先：打断正在播的出发/闲聊，别排在长句后面
            self._speech.stop_async()
            self._speak(yield_text, listen=True)
            if self._nav.was_cancelled():
                self._nav.clear_cancel()
                return "cancelled"
            # 一直等到路通；人不走就每隔 repeat_s 再喊一次
            last_nag = time.monotonic()
            while True:
                if self._wait_front_clear(min(5.0, repeat_s)):
                    break
                if self._nav.was_cancelled():
                    self._nav.clear_cancel()
                    return "cancelled"
                if time.monotonic() - last_nag >= repeat_s:
                    self.get_logger().info(f"{poi} 仍被挡，再次提醒让行")
                    self._file_log("障碍", f"{poi} 仍被挡，再次提醒让行")
                    self._speak_async(yield_text)
                    last_nag = time.monotonic()
            self.get_logger().info(f"前方已让开，重新前往 {poi}")
            self._file_log("障碍", f"{poi} 已让开，继续")
            # 续行播报放后台，不耽误底盘立刻走
            self._speak_async(resume_text)
            time.sleep(0.3)  # 给底盘一点结算时间

    def _navigate(self, place, standby: bool = False,
                  ready_checked: bool = False) -> None:
        # 目的地就是待命点（如「回到乐知教育」）也按回程处理：
        # 让行话术、到达播报、转回朝向都用回家逻辑。
        if (self._places is not None
                and self._places.standby is not None
                and place.poi == self._places.standby.poi):
            standby = True
        self._navigating = True
        self._nav_standby = bool(standby)
        self._home_requested = False
        self._awaiting = False  # 开始新导航，停止「等观众」计时
        self._paused_target = None
        self._paused_standby = False
        self._retarget = None
        self._pause_requested = False
        self._drain_queue()
        result = "success"
        blocked = False
        try:
            # 中途改道时定位状态没变，跳过检查避免多等几秒
            if not ready_checked:
                ok, message = self._nav.check_ready()
                if not ok:
                    self._speak(message)
                    return
            # 记下离开待命点时的朝向，回程结束后要转回去
            if not standby and self._nav is not None:
                home = self._places.standby if self._places else None
                at_home = bool(home and self._here == home.spoken)
                if self._home_yaw is None or at_home:
                    pose = self._nav.pose()
                    try:
                        self._home_yaw = float(pose["yaw"])
                    except (TypeError, KeyError, ValueError):
                        pass
            target = place.spoken
            if standby:
                # 回程先明确回应一句，不能一声不吭就走；后台播报，边说话边起步
                self._speak_async("好的，我先回待命点了")
            else:
                self._speak_async(f"好的，我这就带你去{target}")
            self.get_logger().info(
                f"开始导航：{place.poi}（{target}，"
                f"{'回待命点' if standby else '送访客'}）")
            self._file_log("导航", f"{place.poi} {'回待命点' if standby else '送访客'}")
            result = self._run_goto(place.poi, standby)
            if result != "success":
                self._file_log("导航", f"{place.poi} 未到达：{result}")
            if result == "blocked":
                if self._retarget is not None or self._pause_requested:
                    # 被挡期间观众又下了新指令，优先处理改道/停车
                    blocked = True
                elif standby:
                    # 回程被挡：已播报「请让一让我正在回家」，原地停着等人让开，
                    # 不再每 35 秒自动重试，避免反复播报。
                    self._awaiting = False
                    return
                else:
                    # 连续被挡、已经播报过让行提示：停在原地等对方让开，
                    # 说「继续」可以重试，35 秒没人管就自动回待命点。
                    self._pending = None
                    self._paused_target = place
                    self._paused_standby = standby
                    self._awaiting = False
                    self._qwen.set_context(
                        f"你正带访客去「{place.spoken}」的路上，"
                        "前方有人挡住停下来了。"
                        "被问到位置时如实说还在路上，不要编造周围是什么地方。")
                    self._paused_at = time.monotonic()
                    return
        except Exception as exc:  # noqa: BLE001 - 导航失败只提示，不退出节点
            if self._nav is not None and self._nav.was_cancelled():
                self.get_logger().info("导航已按观众要求中断")
            else:
                self.get_logger().error(f"导航失败：{exc}")
                if standby:
                    self._speak("我没能自己回到待命点，请人工过来处理一下")
                else:
                    self._speak("抱歉，这次没能带你去，底盘导航没有完成")
                # 送客失败就安排一次回程；回程本身失败不再重试，
                # 否则定位一直坏会每 35 秒反复播报
                if not standby:
                    self._awaiting = True
                    self._awaiting_at = time.monotonic()
                return
        finally:
            self._navigating = False

        # 中断可能是「停车」也可能是「改目的地」，改道优先
        retarget, self._retarget = self._retarget, None
        pause, self._pause_requested = self._pause_requested, False
        if retarget is not None:
            self._navigate(retarget, ready_checked=True)
            return
        if pause:
            # 暂停交流：保留原目标，等观众说完再决定
            self._pending = None
            self._paused_target = place
            self._paused_standby = standby
            self._awaiting = False
            # 半路被叫停，模型不能拿出发点当「这是哪」的答案
            self._qwen.set_context(
                f"你正带访客去「{place.spoken}」的路上，现在被叫住停在半路。"
                "被问到位置时如实说还在去该点位的路上，不要编造周围是什么地方。")
            if self._pause_kind == "stop":
                self._speak("好的，已经停下了，您说", listen=True)
            elif self._pause_kind == "vague":
                self._speak("好的，我不去原来那儿了，您想去哪个点位？",
                            listen=True)
            else:
                self._speak("我在，请问有什么事？", listen=True)
            # 提示播完才开始 35 秒倒计时，不然提示本身占掉两秒
            self._paused_at = time.monotonic()
            return

        if result == "cancelled" or blocked:
            # 已取消/被挡后又被新指令接管，不能误报「到了」
            return

        # 只有真正到达才会走到这里；日志先记一笔，便于和「出发」区分
        self._file_log("到达", place.poi)
        self._set_here(place.spoken)
        self._arriving = True
        try:
            if standby:
                self._awaiting = False
                self._qwen.reset()  # 一轮结束，忘掉这批观众的对话
                try:
                    self._face_home()  # 转回离开待命点时的方向
                except Exception as exc:  # noqa: BLE001 - 转回失败也要报到达
                    self.get_logger().error(f"回待命点转身异常：{exc}")
                    self._file_log("转身", f"回待命点转身异常：{exc}")
                self._speak(f"好的，我已经回到{place.spoken}待命，有事随时叫我",
                            listen=True)
            else:
                # 先按点位配置原地转身，再说到点；播报结束立刻转回原方向。
                # 转身失败或异常都不能吞掉到达播报。
                turn_angle = 0.0
                try:
                    turn_angle = self._turn_on_arrival(place.poi)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"{place.poi} 到达转身异常：{exc}")
                    self._file_log("转身", f"{place.poi} 转身异常：{exc}")
                # 固定短句而不是让模型自由发挥，避免到达后长篇大论把
                # 观众那句「没事了你回去吧」压在播报里听不到。
                self._speak(self._arrival_line(place.spoken), listen=True)
                if turn_angle:
                    try:
                        # 播报结束后按 arrival_turn_hold（默认 2 秒）稍停再转回；
                        # 观众喊停或已下新目的地时别再动，避免刚停又转
                        self._hold_arrival_pose()
                        if not self._pause_requested and self._retarget is None:
                            self._turn_back(turn_angle)
                    except Exception as exc:  # noqa: BLE001
                        self._file_log("转身", f"{place.poi} 转回异常：{exc}")
                self._awaiting = True
                self._awaiting_at = time.monotonic()
        finally:
            self._arriving = False

        # 到达播报期间又说了新目的地：直接改道，别再等 35 秒
        retarget, self._retarget = self._retarget, None
        if retarget is not None:
            self.get_logger().info("到达播报中收到改道指令，立即前往新点位")
            self._navigate(retarget, ready_checked=True)
            return

        # 到达播报期间听到「没事了你回去吧」：不再等 35 秒，播完立刻走
        home, self._home_requested = self._home_requested, False
        if home and not standby:
            target = self._places.standby if self._places else None
            if target is not None:
                self.get_logger().info("到达后收到回程指令，立即返回待命点")
                self._navigate(target, standby=True)

    @staticmethod
    def _parse_arrival_turns(raw: str) -> dict:
        """解析「点位:left,点位:right」；left=+1，right=-1。"""
        turns = {}
        for item in raw.replace("，", ",").split(","):
            name, _, direction = item.partition(":")
            name = name.strip()
            direction = direction.strip().lower()
            if direction in ("left", "l", "左", "左转"):
                turns[name] = 1
            elif direction in ("right", "r", "右", "右转"):
                turns[name] = -1
        return turns

    def _turn_on_arrival(self, poi: str) -> float:
        """到达点位后按配置原地转身，返回实际转过的角度。

        配置里的 left/right 是按「从待命点方向（主线正向）进来」标定的。
        返程从主线反方向进入时，同一个展品要朝另一侧转，左右自动对调。
        """
        sign = self._arrival_turns.get(poi)
        if sign is None or self._nav is None:
            self._file_log("转身", f"{poi} 没配转身方向，跳过")
            return 0.0
        if self._approach_reversed(poi):
            sign = -sign
            self._file_log("转身", f"{poi} 从返程方向进入，转向左右对调")
        angle = sign * self._arrival_turn_angle
        side = "左" if sign > 0 else "右"
        # 先落一条日志：转身卡住/异常时也能看出走到哪一步
        self._file_log("转身", f"{poi} 开始{side}转 {abs(angle):.0f}°")
        if not self._nav.turn(angle):
            self._file_log("转身", f"{poi} 转身未完成，已跳过")
            return 0.0
        self.get_logger().info(f"到达 {poi}，原地{side}转 {abs(angle):.0f}°")
        self._file_log("转身", f"{poi} {side}转 {abs(angle):.0f}°")
        return angle

    def _approach_reversed(self, poi: str) -> bool:
        """这次是否从主线反方向（返程）进入该点位。"""
        if self._nav is None:
            return False
        forward = self._nav.forward_yaw(poi)
        if forward is None:
            return False
        try:
            current = float(self._nav.pose()["yaw"])
        except (TypeError, KeyError, ValueError):
            return False
        delta = math.atan2(math.sin(current - forward),
                           math.cos(current - forward))
        return abs(delta) > math.radians(90.0)

    def _turn_back(self, angle: float) -> None:
        if abs(angle) < 1.0 or self._nav is None:
            return
        if self._nav.turn(-angle):
            self.get_logger().info(f"展示结束，转回原方向 {-angle:+.0f}°")
            self._file_log("转身", "转回原方向")
        else:
            self._file_log("转身", "展示结束转回原方向未完成")

    def _hold_arrival_pose(self) -> None:
        """保持转身姿态等待；有人下新指令就提前结束。"""
        deadline = time.monotonic() + self._arrival_turn_hold
        while time.monotonic() < deadline:
            if (self._retarget is not None or self._home_requested
                    or self._pause_requested or not self._queue.empty()):
                return
            time.sleep(0.2)

    def _remember_home_yaw(self, delay: float = 2.0) -> None:
        """启动兜底：延时读一次当前朝向，作为回程要转回的朝向。"""
        if delay > 0:
            time.sleep(delay)
        if self._home_yaw is not None or self._nav is None:
            return
        pose = self._nav.pose()
        try:
            self._home_yaw = float(pose["yaw"])
        except (TypeError, KeyError, ValueError):
            self._file_log("朝向", "启动时读不到朝向，回程将不转回")
            return
        self.get_logger().info(
            f"记下待命朝向 {math.degrees(self._home_yaw):+.0f}°")
        self._file_log(
            "朝向", f"记下待命朝向 {math.degrees(self._home_yaw):+.0f}°")

    def _face_home(self) -> None:
        """回到待命点后转回离开时的朝向。"""
        if self._nav is None:
            return
        if self._home_yaw is None:
            self._file_log("转身", "没有待命朝向记录，跳过转回")
            return
        pose = self._nav.pose()
        try:
            current = float(pose["yaw"])
        except (TypeError, KeyError, ValueError):
            self._file_log("转身", "读不到当前朝向，跳过转回")
            return
        delta = math.atan2(math.sin(self._home_yaw - current),
                           math.cos(self._home_yaw - current))
        if abs(delta) < math.radians(8):
            return
        if self._nav.turn(math.degrees(delta)):
            self.get_logger().info(
                f"回到待命点，转回原方向 {math.degrees(delta):+.0f}°")
            self._file_log(
                "转身", f"回到待命点转回原方向 {math.degrees(delta):+.0f}°")
        else:
            self._file_log("转身", "回到待命点转回原方向未完成")

    def _arrival_line(self, target: str) -> str:
        return f"我们到{target}了，还有什么事吗？"

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def _say(self, text: str, listen: bool = True) -> bool:
        """唯一的播报出口：记录话术与静默窗口，返回是否播报成功。

        listen=True 时播报期间不关麦克风，让观众可以直接打断
        （例如到达后马上说「没事了你回去吧」）。
        """
        text = (text or "").strip()
        if not text:
            return False
        self._last_spoken = text
        self._spoken_history.append((time.monotonic(), text))
        self._file_log("播报", text)
        if not listen:
            self._hold_mic(True)  # 长句播报时关麦克风，避免听见自己
        try:
            ok = bool(self._speech.say(text))
        finally:
            if not listen:
                self._hold_mic(False)
        # 播完再留 2 秒静默，等喇叭尾音和识别结果过去
        self._speech_until = time.monotonic() + 2.0
        return ok

    def _hold_mic(self, muted: bool) -> None:
        if self._mic is not None:
            self._mic.hold(muted)

    def _is_recent_echo(self, text: str) -> bool:
        """识别延迟可能有好几秒，按最近 25 秒的播报历史判断回声。"""
        now = time.monotonic()
        # 短片段只在「刚说完」的十秒内按回声处理；否则观众过了半分钟
        # 复述一句短话（比如「艺术长廊」）也会被误吞。
        recent_tail = (self._speech.busy
                       or now <= self._speech_until + 8.0)
        for stamp, said in tuple(self._spoken_history):
            if now - stamp > 25.0:
                continue
            if _echo_of(text, said):
                return True
            # 播报被切出来的短片段（「这就带您过去。」「我这就带您过去。」）
            # 长度不够 8 字，上面的规则拦不住。识别结果往往比播报晚 1~3 秒，
            # 不能只在「正在播报」时判断，否则尾音片段会漏进来。
            if (recent_tail and _echo_fragment(text, said)
                    and not self._has_intent(text)):
                return True
        # 正在播报时最后一句还没进历史窗口的边界情况
        if self._last_spoken:
            if _echo_of(text, self._last_spoken):
                return True
            if (recent_tail and _echo_fragment(text, self._last_spoken)
                    and not self._has_intent(text)):
                return True
        return False

    def _has_intent(self, text: str) -> bool:
        """是不是明确的停车/回程/问话/地点指令；用于区分回声和打断。"""
        plain = _plain(text)
        if _is_stop(plain) or _wants_go_home(text) or _wants_pause(plain):
            return True
        return bool(self._places is not None
                    and self._places.resolve(text) is not None)

    def _speak(self, text: str, listen: bool = True) -> None:
        self._say(text, listen=listen)

    def _speak_async(self, text: str) -> None:
        """后台播报：不阻塞导航，让机器人边说话边起步。"""
        if not text:
            return
        threading.Thread(target=self._speak, args=(text,),
                         daemon=True).start()

    def _file_log(self, kind: str, text: str) -> None:
        """把每句话落到日志文件，方便事后排查（终端滚走了也能查）。"""
        try:
            with open(self._log_file, "a", encoding="utf-8") as handle:
                handle.write(f"{time.strftime('%H:%M:%S')} {kind} {text}\n")
        except OSError:
            pass

    def _await_loop(self) -> None:
        """两个保底计时：送达后没人搭话、暂停后没人再说话，都自动回待命点。"""
        while not self._closing:
            now = time.monotonic()
            self._flush_prefix()
            if (self._awaiting and not self._navigating
                    and now - self._awaiting_at >= self._await_timeout):
                self.get_logger().info(
                    f"送达后 {self._await_timeout:.0f} 秒没人搭话，自动回待命点")
                self._awaiting = False
                self._queue.put(_AUTO_HOME)
            elif (self._paused_target is not None and not self._navigating
                  and not self._speech.busy
                  and self._pause_timeout > 0
                  and now - self._paused_at >= self._pause_timeout):
                self.get_logger().info(
                    f"暂停后 {self._pause_timeout:.0f} 秒没人再说话，自动回待命点")
                self._paused_target = None
                self._queue.put(_AUTO_HOME)
            time.sleep(0.5)

    def _pop_pending(self) -> str | None:
        """取出队列里最早的一条真实语音指令；忽略重复的自动回程哨兵。"""
        while True:
            try:
                text = self._queue.get_nowait()
            except queue.Empty:
                return None
            if isinstance(text, str) and text.strip():
                return text
            # _AUTO_HOME 之类的哨兵直接丢掉，继续找

    def _auto_home(self) -> None:
        """在工作线程里执行，保证回家导航和其他指令不会同时下发给底盘。"""
        standby = self._places.standby if self._places is not None else None
        if standby is None or self._navigating:
            return
        # 回程播报前后都检查有没有新指令：说了「带我去X」就先执行新任务，
        # 不能先把 35 秒回程跑完再理人。
        pending = self._pop_pending()
        if pending is not None:
            self.get_logger().info(f"回程前收到新指令，取消回程：{pending}")
            self._file_log("取消回程", pending)
            self._handle(pending)
            return
        # 回程话术统一由 _navigate(standby=True) 说「好的，我先回待命点了」，
        # 这里不再重复播报。
        self._navigate(standby, standby=True)

    def _presence_loop(self) -> None:
        """正前方距离小于阈值就主动问候；人离开后再有人来才重新问候。"""
        greet_m = float(self.get_parameter("presence_distance").value)
        need = max(1, int(self.get_parameter("presence_confirm").value))
        cooldown = float(self.get_parameter("greet_cooldown").value)
        greet_text = str(self.get_parameter("greet_text").value or "").strip()
        if not greet_text:
            return
        hits = 0
        welcomed = False
        last_greet = 0.0
        while not self._closing:
            distance = self._guard.distance()
            if distance is not None and distance <= greet_m:
                hits += 1
            else:
                hits = 0
            # 退到阈值外（留 0.4 m 迟滞）才允许下一位靠近时重新问候
            if distance is not None and distance > greet_m + 0.4:
                welcomed = False
            now = time.monotonic()
            if (hits >= need and not welcomed and not self._navigating
                    and not self._arriving
                    and not self._speech.busy and not self._awaiting
                    and self._paused_target is None
                    and now - last_greet >= cooldown):
                welcomed = True
                last_greet = now
                self.get_logger().info(
                    f"检测到有人靠近（{distance:.2f} m），主动问候")
                if self._say(greet_text):
                    self._qwen.reset()  # 新观众，别带上一位的聊天记忆
            time.sleep(0.2)

    def _greet(self, text: str) -> None:
        """开机自我介绍：直接播报，绝不走 _handle（自己的话不能当指令）。"""
        time.sleep(2.0)
        for _ in range(3):
            if self._say(text):
                return
            time.sleep(2.0)

    def shutdown(self) -> None:
        self._closing = True
        self._queue.put(None)
        self._worker.join(timeout=2.0)
        if self._mic is not None:
            self._mic.stop()
        if self._nav is not None:
            self._nav.shutdown()
        if self._guard is not None:
            self._guard.shutdown()
        # 还原本地现场：关闭语音识别
        if self._enable_pub is not None and rclpy.ok():
            msg = Bool()
            msg.data = False
            self._enable_pub.publish(msg)


def _meaningful(text: str, min_chars: int) -> bool:
    """过滤空串、纯标点、过短的误识别。"""
    return len(re.sub(r"\W", "", text or "")) >= max(1, min_chars)


_PLACE_HINTS = ("想", "要", "去", "打", "玩", "练", "参观", "逛", "看", "找")
# 「带我去」「我要去」这种只有方向没有地名的半句，等下一句再合并
_GO_PREFIX_WORDS = ("带我去", "领我去", "我要去", "我想去", "我要到",
                    "我想到", "我要看", "我想看", "我要参观", "我想参观")
_PREFIX_WAIT_S = 2.5
_GO_WORDS = ("去", "到", "前往", "带我去", "领我去", "回", "走")
# 否定与疑问不能当成出发指令：「我不去厕所」「带我去吗」
_NEG_GO = ("不去", "不想去", "不用去", "别去", "不要去", "不打算去",
           "去不去", "去吗", "去呢", "去哪儿", "去哪里", "去哪个", "去哪好",
           "在哪", "哪里", "哪儿", "什么地方", "怎么走")
_QUESTION_TAIL = ("吗", "呢", "?", "？")
_YES_WORDS = ("需要", "好啊", "好的", "好呀", "可以", "行", "带我去", "去吧",
              "麻烦", "要")
_NO_WORDS = ("不用", "不需要", "不了", "算了", "暂时不", "先不", "别")
_DONE_WORDS = (
    "没事", "没有了", "没了", "不用了", "不用", "不需要",
    "没有别的", "没别的", "就这样", "结束", "回去", "回去吧",
    "我们走", "走吧", "走了", "可以走了", "该走了", "我得走了",
    "先走了", "就到这", "到这里", "到此为止", "再见", "拜拜", "辛苦了",
)


def _plain(text: str) -> str:
    return re.sub(r"[\s，。！？、,.!?~～]", "", text or "")


def _means_yes(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _YES_WORDS)


def _explicit_go(text: str) -> bool:
    plain = _plain(text)
    if any(word in plain for word in _NEG_GO):
        return False
    if str(text).strip().endswith(_QUESTION_TAIL):
        return False
    return any(word in plain for word in _GO_WORDS)


def _is_go_prefix(text: str) -> bool:
    """整句只有「带我去」这类方向词、还没说地名。"""
    plain = _plain(text).rstrip("吧啊呀哦嘛")
    return plain in _GO_PREFIX_WORDS


def _means_no(text: str) -> bool:
    plain = _plain(text)
    return any(word in plain for word in _NO_WORDS)


_STOP_WORDS = ("停下", "停一下", "停一停", "停止", "别动", "别走",
               "不要走", "站住", "刹车", "等一下", "等等我")


def _is_stop(plain: str) -> bool:
    """导航中的急停：必须是很短的句子，长句里出现的「停」不算。"""
    if plain in ("停", "停停", "停车"):
        return True
    return len(plain) <= 8 and any(word in plain for word in _STOP_WORDS)


def _wants_done(text: str) -> bool:
    """只在短句里认结束语：长句里的「回去」多半是随口一提，不能当指令。"""
    plain = _plain(text)
    if len(plain) > 15:
        return False
    return any(word in plain for word in _DONE_WORDS)


_HOME_WORDS = ("回去吧", "回吧", "回待命", "返回待命",
               "回待命点", "回去休息", "回乐知", "你回去", "先回去")


def _wants_go_home(text: str) -> bool:
    """明确让它回待命点的话；不依赖「到达后等待」状态，站着也能用。"""
    plain = _plain(text)
    if len(plain) > 15:
        return False
    return any(word in plain for word in _HOME_WORDS)


_CHANGE_WORDS = ("去别的", "去别处", "换个地方", "换一个地方", "改去别",
                 "不去这", "不去这里", "不想去", "别去这", "换个点位",
                 "改个地方", "去其他地方", "带我去其他")


def _wants_change_place(plain: str) -> bool:
    """没点名目的地，但明显是想改地方：先停下再问去哪。"""
    if len(plain) > 16:
        return False
    return any(word in plain for word in _CHANGE_WORDS)


_RESUME_WORDS = ("继续", "接着走", "接着", "继续走", "带我去吧", "可以走了")
_CALL_WORDS = ("机器人", "小静", "你好", "喂", "在吗", "在不在")
_ASK_WORDS = ("请问", "我想问", "问你", "这是哪", "什么地方", "在哪", "哪里",
              "怎么走", "这是什么", "干什么的")


def _wants_pause(plain: str) -> bool:
    """导航中有人明确叫住它：呼唤词或短句提问。路人闲聊不触发。"""
    if len(plain) > 24:
        return False
    if any(word in plain for word in ("机器人", "小静", "在吗", "在不在")):
        return True
    if plain in ("你好", "喂", "等一下"):
        return True
    return any(word in plain for word in _ASK_WORDS)


def _is_call_only(plain: str) -> bool:
    """整句只是「机器人」「停一下」这种呼叫，没有实际内容。"""
    rest = plain
    for word in _CALL_WORDS + _STOP_WORDS + ("停", "等等"):
        rest = rest.replace(word, "")
    # 「别走了」「停一下吧」只是语气词，不算有新内容
    return not rest.strip("了吧啊呀哦嘛")


def _wants_resume(plain: str) -> bool:
    return any(word in plain for word in _RESUME_WORDS)


def _echo_of(text: str, spoken: str) -> bool:
    """识别文本是不是自己刚播报内容的回声（长句才判，避免误伤短指令）。"""
    said = _plain(text)
    last = _plain(spoken)
    if not said or not last:
        return False
    if said == last:  # 短句回声（「好的」「我在」）长度不够也要拦
        return True
    if len(said) < 8 or len(last) < 8:
        return False
    shorter, longer = (said, last) if len(said) <= len(last) else (last, said)
    if shorter in longer and len(shorter) / len(longer) >= 0.5:
        return True
    # 播报时开麦后，识别结果可能和原句只差几个字，用相似度兜底
    return difflib.SequenceMatcher(None, said, last).ratio() >= 0.75


def _echo_fragment(text: str, spoken: str) -> bool:
    """短片段是否来自一句较长的播报（播报中被切开的回声）。"""
    said = _plain(text)
    last = _plain(spoken)
    return len(said) >= 4 and len(last) >= 12 and said in last


def _extract_json(text: str):
    match = re.search(r"\{.*\}", text or "", re.S)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError:
        return None


def _acquire_instance_lock():
    """只允许一个 tour_chat 实例。

    重复启动时两个节点都会听麦克风、都会播报，还会把对方的播报当用户输入，
    表现为一男一女两个声音自己聊起来。这里用文件锁直接拒绝第二个实例。
    """
    path = os.path.join("/tmp", "learn3_d_tour_chat.lock")
    handle = open(path, "w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


# 其他会控制同一底盘的导航程序（安装目录特征）
_OTHER_NAV_MARKERS = (
    "install/nav_pkg/lib/nav_pkg/tour_chat",
    "install/guide_voice/lib/guide_voice",
    "install/learn1_d/lib/learn1_d",
)


def _other_nav_processes() -> list:
    """扫描 /proc，找出可能抢占底盘的其他导航进程。"""
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit() or int(name) == os.getpid():
            continue
        try:
            with open(f"/proc/{name}/cmdline", "rb") as handle:
                cmd = handle.read().replace(b"\x00", b" ").decode(
                    "utf-8", "ignore")
        except OSError:
            continue
        if any(marker in cmd for marker in _OTHER_NAV_MARKERS):
            found.append(f"{name}: {cmd.strip()}")
    return found


def main(args=None) -> None:
    lock = _acquire_instance_lock()
    if lock is None:
        print("已有 tour_chat 在运行，本次启动已取消。"
              "重复实例会互相听到对方播报，出现男女声自己对话。",
              flush=True)
        return
    others = _other_nav_processes()
    if others:
        print("警告：检测到其他导航节点也在运行，会抢占同一个底盘：",
              flush=True)
        for line in others:
            print("  " + line, flush=True)
        print("建议先停止这些节点，再测试本项目。", flush=True)
    rclpy.init(args=args)
    node = TourChat()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown()
            executor.shutdown()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
