#!/usr/bin/env python3
"""daolankuozhan 地图导览导航的 ROS2 节点。

底盘导航只有 REST 接口，节点内部仍通过 robot_controller SDK 调 REST；
对外用 ROS 参数配置路线，启动后自动执行，Ctrl-C 取消动作。

用法：
    ros2 run learn3_d nav_route
    ros2 run learn3_d nav_route --ros-args \
        -p map:=消防 \
        -p start:=乐知教育 \
        -p pois:="核心力量跑酷,创想谷,厕所,跆拳道馆" \
        -p track:=track_line_1003 \
        -p reloc_poi:=乐知教育

导航能力被抽成 TourNavigator，tour_chat 语音节点也复用它。
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node

sys.path.insert(0, os.environ.get("ROBOT_CONTROLLER_SRC",
                                 "/home/nvidia/robot_controller/src"))
from robot_controller.chassis import KnewbotChassisNav  # noqa: E402

POLL_S = 0.5
OK_STATES = {"successed", "succeeded", "success", "finished", "done", "completed"}
BAD_STATES = {"failed", "failure", "error", "cancelled", "canceled",
              "stopped", "aborted", "rejected"}
RELOC_STATES = {1, 2, 3, 5}

OBSTACLE_STOP_DTC = 0x08020005
ARRIVE_TOL_M = 1.0
DEFAULT_TTS_SERVICE = "/intelligent_interaction/tts/play"
DEFAULT_AUDIO_ID = "audio_guide"
DEFAULT_POIS = "核心力量跑酷,创想谷,厕所,跆拳道馆"


def unwrap_list(rv, *keys):
    if isinstance(rv, list):
        return rv
    if isinstance(rv, dict):
        for key in keys:
            if isinstance(rv.get(key), list):
                return rv[key]
    return []


def verdict(status):
    if not isinstance(status, dict):
        return "running"
    if status.get("done") is True or status.get("is_finished") is True:
        return "ok"
    for key in ("status", "state", "action_state"):
        text = status.get(key)
        if isinstance(text, str):
            text = text.lower()
            if text in OK_STATES:
                return "ok"
            if text in BAD_STATES:
                return "fail"
    return "running"


def short_map(name):
    base = str(name or "").rsplit("/", 1)[-1]
    return base[:-4] if base.lower().endswith(".pcd") else base


def poi_pose(chassis, name, map_name=None):
    rv = chassis.get_pois(map_name=map_name or None, point_name=name)
    if isinstance(rv, dict) and (rv.get("name") or rv.get("point_name")):
        entries = [rv]
    else:
        entries = unwrap_list(rv, "pois", "poi", "data")
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        if str(entry.get("name") or entry.get("point_name")) != name:
            continue
        try:
            return (float(entry["x"]), float(entry["y"]), float(entry["yaw"]))
        except (KeyError, TypeError, ValueError):
            return None
    return None


def read_pois(chassis, map_name=None):
    rv = chassis.get_pois(map_name=map_name or None)
    result = {}
    for poi in unwrap_list(rv, "pois", "poi", "data"):
        if isinstance(poi, dict):
            name = poi.get("name") or poi.get("point_name")
            if name:
                result[str(name)] = poi
    return result


def read_tracks(chassis, map_name=None):
    rv = chassis.get_tracks(map_name=map_name or None)
    result = {}
    for track in unwrap_list(rv, "tracks", "track", "data"):
        if isinstance(track, dict) and track.get("track_name") is not None:
            result[str(track["track_name"])] = track.get("track_id")
    return result


class Speaker:
    """通过平台 TTS 服务播报（节点自带 service client）。

    注意：say() 内部用 spin_until_future_complete，只能在节点没有
    被 executor 自旋的线程里调用（nav_route 直接在主线程跑流程）。
    tour_chat 有 executor 自旋，使用自己的播报实现。
    """

    def __init__(self, node, audio_id=DEFAULT_AUDIO_ID, enabled=True):
        self._node = node
        self._log = node.get_logger()
        self._audio_id = audio_id
        self._enabled = enabled
        self._client = None
        if enabled:
            try:
                from interaction_msgs.srv import TtsService
            except Exception as exc:  # noqa: BLE001
                self._log.warn(f"语音播报不可用（需 source ~/xos/setup.bash）：{exc}")
                self._enabled = False
                return
            self._type = TtsService
            self._client = node.create_client(TtsService, DEFAULT_TTS_SERVICE)

    def say(self, text):
        if not self._enabled or not text:
            return False
        try:
            if not self._client.wait_for_service(timeout_sec=2.0):
                self._log.warn(f"TTS 服务 {DEFAULT_TTS_SERVICE} 未就绪，跳过播报")
                return False
            req = self._type.Request()
            req.cmd = "append"
            req.text = text
            req.type = "text"
            req.audio_id = self._audio_id
            req.extra = ""
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            response = future.result() if future.done() else None
            ok = bool(response is not None and response.success)
            self._log.info(f"语音播报「{text}」：{'成功' if ok else '失败'}")
            return ok
        except Exception as exc:  # noqa: BLE001
            self._log.warn(f"语音播报异常：{exc}")
            return False


class ObstacleNotifier:
    def __init__(self, log, speaker, text="请让一让", cooldown_s=15.0):
        self._log = log
        self._speaker = speaker
        self._text = text
        self._cooldown_s = cooldown_s
        self._last = 0.0
        self._blocked = False

    def check(self, chassis):
        try:
            health = chassis.get_robot_health()
        except Exception:  # noqa: BLE001
            return
        codes = {int(entry.id) for entry in (health.baseError or [])}
        if OBSTACLE_STOP_DTC not in codes:
            self._blocked = False
            return
        now = time.monotonic()
        if self._blocked and now - self._last < self._cooldown_s:
            return
        self._blocked = True
        self._last = now
        self._log.warn("检测到前方障碍物，底盘已停车")
        if self._speaker is not None:
            self._speaker.say(self._text)


class TourNavigator:
    """可复用的导览导航器：连接底盘、切换地图、按点位或轨道前往。

    只做阻塞式调用、不含 ROS 订阅，方便在节点工作线程里使用；
    speaker 只需提供 say(text) 方法。
    """

    def __init__(self, node, host="192.168.11.10", rest_port=9090,
                 map_name="消防", reloc_poi="", track="", timeout=180.0,
                 speaker=None, obstacle_text="请让一让"):
        self._log = node.get_logger()
        self._host = str(host)
        self._rest_port = int(rest_port)
        self._map_name = str(map_name or "")
        self._reloc_poi = str(reloc_poi or "")
        self._track = str(track or "")
        self._timeout = float(timeout)
        self._notifier = ObstacleNotifier(self._log, speaker, obstacle_text)
        self._chassis = None
        self._track_ids = {}
        self._ready = False
        self._cancelled = False

    @property
    def map_name(self):
        return self._map_name

    @property
    def ready(self):
        return self._ready

    def connect(self):
        self._chassis = KnewbotChassisNav(
            host=self._host, rest_port=self._rest_port)
        self._log.info(f"健康状态：{self._chassis.get_robot_health()}")
        return self

    def pose(self):
        return self._chassis.get_pose() if self._chassis else None

    def poi_names(self):
        """当前地图上的点位名（地点别名从这里派生）。"""
        return sorted(read_pois(self._chassis, self._map_name))

    def check_ready(self):
        """移动前健康自检：A/B 类故障时返回 (False, 一句提示)。

        维护模式（A 类）不拦的话，底盘会直接拒绝运动任务；
        读不到严重程度的老固件按阻断处理。
        """
        try:
            health = self._chassis.get_robot_health()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"读取底盘健康状态失败：{exc}")
            return False, "我读不到底盘状态，先不移动，请检查底盘连接"
        blocking = []
        for entry in getattr(health, "baseError", []) or []:
            severity = str(getattr(entry, "severity", "") or "").upper()
            if not severity or any(tag in severity for tag in
                                   ("FATAL", "ERROR", "A类", "B类")):
                blocking.append(entry)
        if not blocking:
            return True, ""
        detail = "；".join(
            " ".join(x for x in (str(getattr(e, "description", "")).strip(),
                                 str(getattr(e, "solution", "")).strip()) if x)
            for e in blocking)
        self._log.error(f"底盘健康检查未通过：{detail}")
        if any("维护模式" in str(getattr(e, "description", ""))
               for e in blocking):
            return False, "维护模式还开着，请先弹起维护按钮，退出手动模式后再叫我"
        return False, "底盘有故障，暂时不能移动，请先检查底盘告警"

    def _track_id(self, name):
        """按轨道名取 track_id；空名表示走点对点，返回 None。"""
        if not name:
            return None
        if name not in self._track_ids:
            self._track_ids[name] = read_tracks(
                self._chassis, self._map_name).get(name)
        track_id = self._track_ids[name]
        if track_id is None:
            raise RuntimeError(
                f"轨道不存在：{name}；当前可用轨道："
                f"{sorted(read_tracks(self._chassis, self._map_name)) or '无'}"
                "。可去掉 track 参数走点对点")
        return track_id

    # ------------------------------------------------------------ 导航
    def _wait_localization(self, timeout_s, pose):
        deadline = time.monotonic() + timeout_s
        attempts = 0
        next_try = 0.0
        last = None
        while time.monotonic() < deadline:
            status = (self._chassis.get_localization_status() or {}).get(
                "slam_status")
            if status != last:
                self._log.info(
                    f"定位状态 slam_status={status}（0=正常 5=等待重定位）")
                last = status
            if status == 0:
                return True
            now = time.monotonic()
            if status in RELOC_STATES and pose and attempts < 3 and now >= next_try:
                self._log.info(f"下发重定位 {attempts + 1}/3："
                               f"x={pose[0]:.2f} y={pose[1]:.2f} "
                               f"yaw={pose[2]:.2f}")
                try:
                    self._chassis.recover_localization(
                        x=pose[0], y=pose[1], yaw=pose[2])
                except Exception as exc:  # noqa: BLE001
                    self._log.warn(f"重定位接口返回异常：{exc}")
                attempts += 1
                next_try = now + 10.0
            time.sleep(POLL_S)
        return False

    def ensure_map(self, pose=None, force_reloc=False):
        """切换到目标地图并等定位就绪；重复调用只执行一次。"""
        if self._ready:
            return True
        map_name = self._map_name
        if pose is None and self._reloc_poi:
            pose = poi_pose(self._chassis, self._reloc_poi,
                            map_name=map_name)
            if pose:
                self._log.info(
                    f"重定位位姿取自 {self._reloc_poi}："
                    f"x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}")
            else:
                self._log.warn(
                    f"地图 {map_name} 上找不到点位 {self._reloc_poi}")
        current = str((self._chassis.get_current_map() or {}).get("map_name")
                      or "")
        target = short_map(map_name)
        if short_map(current) != target:
            self._log.info(f"切换地图：{short_map(current) or '无'} -> {target}")
            self._chassis.start_localization(slam_type=1, map_name=map_name)
        if force_reloc and pose:
            self._log.info(f"强制重定位：x={pose[0]:.2f} y={pose[1]:.2f} "
                           f"yaw={pose[2]:.2f}")
            try:
                self._chassis.recover_localization(
                    x=pose[0], y=pose[1], yaw=pose[2])
            except Exception as exc:  # noqa: BLE001
                self._log.warn(f"重定位接口返回异常：{exc}")
            pose = None
        if not self._wait_localization(60.0, pose):
            raise RuntimeError(
                "定位未就绪：把车放到已知点位后用 reloc_poi 或 pose 参数重试")
        self._log.info("定位就绪")
        self._ready = True
        return True

    def _wait_action(self, label, action_id, timeout_s):
        deadline = time.monotonic() + timeout_s
        last = None
        while time.monotonic() < deadline:
            self._notifier.check(self._chassis)
            status = self._chassis.get_action_status(action_id)
            if status != last:
                self._log.info(f"  {label}：{status}")
                last = status
            result = verdict(status)
            if result == "ok":
                return
            if result == "fail":
                pose = self._chassis.get_pose() or {}
                raise RuntimeError(
                    f"{label} 失败：{status}；当前位姿 x={pose.get('x')} "
                    f"y={pose.get('y')}（1m 内按到点处理）")
            time.sleep(POLL_S)
        self._chassis.cancel_action([action_id])
        raise TimeoutError(f"{label} 超时（{timeout_s}s），已请求取消")

    def _goto(self, target, track_id, timeout_s, map_name):
        if track_id is None:
            self._log.info(f"点对点前往 {target}")
            rv = self._chassis.move_to_poi(poi_name=target)
        else:
            self._log.info(f"沿轨道 {track_id} 前往 {target}")
            rv = self._chassis.move_to_locations_poi(
                track_id, target, uptrack_enabled=True)
        try:
            self._wait_action(target, str(rv.get("action_id")), timeout_s)
        except (RuntimeError, TimeoutError):
            if self._cancelled:  # 观众要求停车，不做「已在附近」的补偿
                raise
            pose = self._chassis.get_pose() or {}
            goal = poi_pose(self._chassis, target, map_name=map_name)
            if goal and pose.get("x") is not None:
                dist = math.hypot(pose["x"] - goal[0], pose["y"] - goal[1])
                if dist <= ARRIVE_TOL_M:
                    self._log.warn(
                        f"底盘回报失败，但已在 {target} 附近"
                        f"（偏差 {dist:.2f}m），按到点继续")
                    return
            raise
        pose = self._chassis.get_pose() or {}
        self._log.info(f"到达 {target}，当前位置 x={pose.get('x')} "
                       f"y={pose.get('y')} yaw={pose.get('yaw')}")

    def goto(self, name, track=None, timeout=None):
        """前往点位；track 留空用构造时的默认轨道，显式传""走点对点。"""
        if self._chassis is None:
            self.connect()
        self._cancelled = False
        self.ensure_map()
        track_name = self._track if track is None else str(track)
        track_id = self._track_id(track_name)
        timeout_s = self._timeout if timeout is None else float(timeout)
        self._chassis.set_aeb_control("On")
        self._goto(name, track_id, timeout_s, self._map_name)
        return True

    def cancel(self):
        """立即取消底盘当前动作，让阻塞中的 goto 抛异常返回。"""
        self._cancelled = True
        if self._chassis is not None:
            return self._chassis.cancel_action()
        return None

    def was_cancelled(self) -> bool:
        return self._cancelled

    def list_all(self):
        map_name = self._map_name
        current = str((self._chassis.get_current_map() or {}).get("map_name")
                      or "")
        query = map_name or current
        self._log.info(f"当前地图：{short_map(current) or '未知'}（{current or '-'}）")
        pois = read_pois(self._chassis, query)
        self._log.info(f"点位（{len(pois)} 个）：")
        for name in sorted(pois):
            p = pois[name]
            self._log.info(f"  {name:<10} x={p.get('x')} y={p.get('y')} "
                           f"yaw={p.get('yaw')}")
        tracks = read_tracks(self._chassis, query)
        self._log.info(f"轨道（{len(tracks)} 条）：")
        for name in sorted(tracks):
            self._log.info(f"  {name:<20} track_id={tracks[name]}")

    def shutdown(self):
        if self._chassis is not None:
            try:
                self._chassis.cancel_action()
            except Exception:  # noqa: BLE001
                pass


class NavRoute(Node):
    def __init__(self):
        super().__init__("nav_route")
        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("map", "消防")
        self.declare_parameter("start", "乐知教育")
        self.declare_parameter("pois", DEFAULT_POIS)
        self.declare_parameter("track", "track_line_1003")
        self.declare_parameter("return_poi", "")
        self.declare_parameter("return_track", "")
        self.declare_parameter("reloc_poi", "乐知教育")
        self.declare_parameter("pose", "")
        self.declare_parameter("force_reloc", False)
        self.declare_parameter("timeout", 180.0)
        self.declare_parameter("obstacle_text", "请让一让")
        self.declare_parameter("enable_voice", True)
        self._log = self.get_logger()
        self._nav = None
        self._speaker = None

    # ------------------------------------------------------------ 参数
    def _param(self, name):
        return self.get_parameter(name).value

    def _parse_pose(self):
        text = str(self._param("pose") or "").strip()
        if not text:
            return None
        parts = [p for p in text.split(",")]
        if len(parts) != 3:
            raise ValueError("pose 参数格式应为 x,y,yaw，例如 0.47,0.49,0.0")
        return tuple(float(p) for p in parts)

    def _goals(self):
        start = str(self._param("start") or "").strip()
        pois = [p.strip() for p in str(self._param("pois") or "").split(",")]
        return [g for g in [start] + pois if g]

    # ------------------------------------------------------------ 主流程
    def run(self):
        self._nav.ensure_map(pose=self._parse_pose(),
                             force_reloc=bool(self._param("force_reloc")))
        self._log.info(f"当前位姿：{self._nav.pose()}")
        self._nav.list_all()

        goals = self._goals()
        if not goals:
            raise RuntimeError("没有要去的点位（start 和 pois 都为空）")
        self._log.info("计划：" + " -> ".join(goals))

        track = str(self._param("track") or "")
        for name in goals:
            self._nav.goto(name, track=track)
        return_poi = str(self._param("return_poi") or "")
        if return_poi:
            self._nav.goto(return_poi,
                           track=str(self._param("return_track") or ""))
        self._log.info("路线执行完成")

    def shutdown(self):
        if self._nav is not None:
            self._nav.shutdown()

    def connect(self):
        self._speaker = Speaker(self, enabled=bool(self._param("enable_voice")))
        self._nav = TourNavigator(
            self,
            host=self._param("host"),
            rest_port=self._param("rest_port"),
            map_name=self._param("map"),
            reloc_poi=self._param("reloc_poi"),
            track=self._param("track"),
            timeout=self._param("timeout"),
            speaker=self._speaker,
            obstacle_text=self._param("obstacle_text"),
        )
        self._nav.connect()


def main(args=None):
    rclpy.init(args=args)
    node = NavRoute()
    exit_code = 0
    try:
        node.connect()
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("收到中断，取消所有动作")
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"导航失败：{exc}")
        exit_code = 1
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
