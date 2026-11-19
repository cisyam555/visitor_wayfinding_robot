"""底盘定点导航：按主线/支线逐点行驶，使用规划坐标及行进朝向。

每段调用原生 move_to(flags=0)，每次下发前要求定位连续稳定。
定位抖动优先等待自然恢复，每次 go 最多主动重定位一次，共用导航超时预算。
成功终态还必须通过 0.35m 到点位置校验。前向防撞随时可以取消任务。
中途恢复按主线段投影计算剩余必经点，不把前方最近点当成已经到达。
"""

from __future__ import annotations

import math
import os
import sys
import time
from collections import namedtuple

_ROBOT_CONTROLLER_SRC = os.environ.get(
    "ROBOT_CONTROLLER_SRC", "/home/nvidia/robot_controller/src"
)
if _ROBOT_CONTROLLER_SRC not in sys.path:
    sys.path.insert(0, _ROBOT_CONTROLLER_SRC)

from robot_controller.chassis import KnewbotChassisNav  # noqa: E402

# 一段直线：目标点名称、坐标、到达朝向（= 本段行进方向）
Leg = namedtuple("Leg", "name x y yaw")

_SUCCESS = {
    "successed", "succeeded", "success", "finished", "done", "completed",
}
_FAIL = {
    "failed", "failure", "error",
    "cancelled", "canceled", "stopped", "aborted", "rejected",
}
_POLL_S = 0.2
_NETWORK_FAILURE_LIMIT = 2
_REQUEST_TIMEOUT_CAP_S = 3.0


def _transport_error(exc) -> bool:
    # robot_controller SDK: 1002=request timeout, 3001=connection error.
    return (getattr(exc, "code", None) in (1002, 3001)
            or isinstance(exc, (TimeoutError, ConnectionError)))
# flags=0：SDK 定义为起步/到点时旋转；仍由底盘规划路径，不保证几何直线。
_POI_FLAGS = 0
# 到达一个点后让底盘停稳，再下发下一段
_LEG_SETTLE_S = 1.0
# 被判 failed 后，等多久重新下发 move_to
_REDISPATCH_SETTLE_S = 1.5
# 一段路线最多下发几次 move_to（底盘自己绕障，失败就重新发任务）
_LEG_ATTEMPTS = 3
# 导航中途停车后，在当前位置重定位最多等多久
_RECOVER_WAIT_S = 20.0
# 必须连续正常，不能把一次闪回正常视为恢复成功。
_LOCALIZATION_STABLE_S = 3.0
_LOCALIZATION_PASSIVE_WAIT_S = 8.0
_LAST_GOOD_POSE_MAX_AGE_S = 90.0
# 刚下发时先让底盘自己跑，过了这个时间才开始查前方障碍
_GUARD_GRACE_S = 1.5
# 连续多少次轮询都判到障碍才真正停车（防单帧误判）
_GUARD_CONFIRM = 3
# 底盘定位状态：0=正常；1=丢失 / 2=精度下降 / 3=异常 / 5=等待重定位
# 才允许主动下发重定位；6=重定位中 / 7=加载地图 / 8=等待传感器
# 是底盘自己在恢复，重复下发只会互相打断。
_RELOCALIZE_TRIGGER_STATUS = {1, 2, 3, 5}
_RELOCALIZE_BUSY_STATUS = {6, 7, 8}
# 4=空闲：定位服务没启动（常见于底盘重启后 SLAM 初始化失败）。
# 这个状态不会自行恢复，重定位接口也无从生效，只能提示人工处理。
_IDLE_STATUS = 4
# 定位置信度低（B 类 ERROR）：恢复等待入口可暂缓，运动下发前仍阻断。
_DTC_LOCALIZATION_LOW_CONFIDENCE = 0x03020003
# 定位空闲时重新加载地图、等待就绪的上限
_LOCALIZATION_START_WAIT_S = 20.0
# 两次自动启动定位的最小间隔，避免和平台自身的自动定位互相打断；
# 取 30s：失败后下一条指令仍有机会再试，不会一整天被冷却挡住。
_LOCALIZATION_RESTART_COOLDOWN_S = 30.0
# 重定位初值吸附半径：优先用附近的已知点位坐标
_POI_SNAP_M = 3.0
# 行驶中定位异常但车仍在前进时，允许持续多久再取消
_MOTION_BAD_CANCEL_S = 8.0
# 判定“仍在前进”的最小剩余距离缩减量
_BAD_PROGRESS_M = 0.30
# 定位异常但已到目标附近时，按到达处理的最大偏差
_ARRIVAL_FALLBACK_M = 1.0
# cancel 后等动作退出 running 的上限
_CANCEL_SETTLE_S = 2.0
# 短于这个长度的段不单独下发（已经站在点上）
_MIN_LEG_M = 0.15
_ARRIVAL_TOLERANCE_M = 0.35
# 上一段结束后定位仍在连续正常，这个时间内的观测可沿用到下一段，
# 不必每段重新等满 3 秒，避免到点后停 5 秒。
_LOCALIZATION_RECENT_S = 4.0
# 防撞取消后，要求前方连续清空这么久才重新下发，避免抖动式反复取消
_GUARD_CLEAR_HOLD_S = 1.0
# 等前方清空的上限，超时返回 blocked 交给上层
_GUARD_CLEAR_WAIT_S = 10.0


def _never_started(status) -> bool:
    """动作 failed 时判断是否「从未起步」。

    底盘在停稳/换任务的过渡期会直接拒绝新任务，返回 failed 且距离与
    耗时都是 0；真正跑了一段才失败的动作这些字段非 0，不能重试。
    """
    if not isinstance(status, dict):
        return False
    seen = False
    for key in ("distance_time", "distance_remaining", "progress"):
        if key not in status:
            continue
        seen = True
        try:
            if float(status[key]) != 0.0:
                return False
        except (TypeError, ValueError):
            return False
    return seen


def classify(status) -> str:
    if not isinstance(status, dict):
        return "running"
    for key in ("status", "state", "action_state"):
        value = status.get(key)
        if isinstance(value, str):
            text = value.lower()
            if text in _SUCCESS:
                return "success"
            if text in _FAIL:
                return "failure"
    # done 只说明任务结束，不保证执行成功。
    if status.get("success") is False:
        return "failure"
    return "running"


def _is_blocking_dtc(entry) -> bool:
    """DTC 是否必须阻止运动。

    底盘健康接口的 hasFatal/hasError 是聚合标志：一条 C 类告警
    （如「WiFi 未连接」）也会把 hasFatal 置 True。真正决定能不能动的
    是每条故障的 severity：A/B 类阻断，C 类只告警；没有 severity
    字段的老固件保持保守策略，视为阻断。
    """
    severity = str(getattr(entry, "severity", "") or "").strip().upper()
    if not severity:
        return True
    return any(tag in severity for tag in ("FATAL", "ERROR", "A类", "B类"))


def _dtc_id(entry):
    """DTC 数字 ID；取不到返回 None。"""
    try:
        return int(getattr(entry, "id", -1))
    except (TypeError, ValueError):
        return None


def _describe_dtc(entry) -> str:
    """单条 DTC 的日志文案：ID + 严重程度 + 描述 + 处理建议。"""
    try:
        dtc_id = f"0x{int(getattr(entry, 'id', 0)):08X}"
    except (TypeError, ValueError):
        dtc_id = "0x????????"
    severity = str(getattr(entry, "severity", "") or "").strip()
    desc = str(getattr(entry, "description", "") or "").strip()
    solution = str(getattr(entry, "solution", "") or "").strip()
    head = " ".join(x for x in (dtc_id, severity, desc) if x)
    return f"{head}（处理：{solution}）" if solution else head


class Navigator:
    def __init__(self, host: str, port: int, timeout: float, logger,
                 track_id: int = 0, max_speed: float = 0.0,
                 direct: bool = False) -> None:
        self._log = logger
        # 非 0 时走底盘原生轨道导航（moveto_locations_poi），支线仍走点对点
        self._track_id = int(track_id or 0)
        # 单点直达：不按主线拆段，一次 move_to 到目标，避免每站停顿
        self._direct = bool(direct)
        # 非 0 时下发底盘限速（m/s），0 表示不改底盘设置
        self._max_speed = float(max_speed or 0.0)
        self._speed_applied = None
        self._chassis = KnewbotChassisNav(
            host=host, rest_port=port,
            request_timeout=min(float(timeout), _REQUEST_TIMEOUT_CAP_S))
        # 点位只在第一次用到时读，之后缓存（地图运行中不会变）
        self._pois = None
        self._recover_attempted = False
        self._network_failed = False
        self._transport_failures = 0
        self._pending_cancel = False
        self._active_action_id = None
        self._last_good_pose = None
        # 最近一次「定位正常且位姿可信」的观测时间，及这段连续正常的起点
        self._last_ok_at = None
        self._ok_since = None
        self._health_message = "底盘状态异常，暂时不能移动"
        # 定点导航主线点位顺序与支线挂接关系，由 places.yaml 注入
        self._main_line = []
        self._branches = {}
        # 上次因前向防撞取消的时间；用于重发前要求前方持续清空
        self._last_guard_cancel_at = None
        # 上次自动重新启动定位的时间；带冷却，避免反复打断平台流程
        self._localization_restarted_at = None

    # ------------------------------------------------------------- 地图信息
    def _load_pois(self) -> dict:
        if self._pois is None:
            try:
                response = self._chassis.get_pois()
            except Exception as exc:  # noqa: BLE001 - 读不到下次再试
                self._log.warn(f"读取点位失败：{exc}")
                return {}
            rows = response.get("pois") if isinstance(response, dict) else response
            pois = {}
            for item in rows or []:
                if not isinstance(item, dict):
                    continue
                name = item.get("name") or item.get("point_name")
                if not name:
                    continue
                try:
                    pois[str(name)] = (
                        float(item.get("x", 0.0)),
                        float(item.get("y", 0.0)),
                        float(item.get("yaw", 0.0)),
                    )
                except (TypeError, ValueError):
                    continue
            if not pois:
                self._log.warn("点位列表为空，稍后重试")
                return {}
            self._pois = pois
        return self._pois

    def poi_names(self) -> list[str]:
        return list(self._load_pois())

    def _poi_pose(self, poi_name: str):
        """取点位位姿 (x, y, yaw)；地图上没有该点返回 None。"""
        return self._load_pois().get(poi_name)

    # ------------------------------------------------------------- 路线规划
    def set_route(self, main_line, branches) -> None:
        """注入主线点位顺序与支线挂接关系。"""
        self._main_line = [str(name) for name in (main_line or [])]
        self._branches = {
            str(name): str(attach)
            for name, attach in (branches or {}).items()
        }

    def plan_route(self, poi_name: str) -> list:
        """按主线顺序逐点规划路径，返回 [Leg(name, x, y, yaw), ...]。

        主线点位按物理顺序排列，用下标前缀的方式确定走法：当前位置落在
        主线的哪个点，目标在主线的哪个点，就按方向把中间每一个点都依次
        走一遍，不跳过。支线点位先从挂接点进入，并在挂接点原地转向后再
        进入支线。
        """
        target = self._poi_pose(poi_name)
        if target is None:
            return []
        xy = self._pose_xy(self._read_pose())
        if self._direct and xy is not None:
            yaw = math.atan2(target[1] - xy[1], target[0] - xy[0])
            return [Leg(poi_name, target[0], target[1], yaw)]
        names = self._route_names(xy, poi_name)
        if names is None:
            # 没配主线 / 点位不在路线上 / 读不到位姿：直接前往
            if xy is None:
                return [Leg(
                    poi_name, target[0], target[1], target[2])]
            yaw = math.atan2(target[1] - xy[1], target[0] - xy[0])
            return [Leg(poi_name, target[0], target[1], yaw)]

        points = []
        if xy is not None:
            points.append(("当前位置", xy))
        for name in names:
            pose = self._poi_pose(name)
            if pose is not None:
                points.append((name, (pose[0], pose[1])))
        if points[-1][0] != poi_name:
            points.append((poi_name, (target[0], target[1])))

        legs = []
        previous = points[0][1]
        for name, current in points[1:]:
            dx, dy = current[0] - previous[0], current[1] - previous[1]
            if math.hypot(dx, dy) < _MIN_LEG_M:
                continue
            legs.append(Leg(
                name, current[0], current[1], math.atan2(dy, dx)))
            previous = current
        return legs

    def _route_names(self, xy, poi_name: str):
        """按主线下标返回要经过的点位序列（不含当前位置）。

        返回 None 表示走不了这条路线（需要直接前往）；返回空列表表示
        按顺序走不需要再经过任何点（已经站在目标上）。
        """
        nodes = self._load_pois()
        chain = [name for name in self._main_line if name in nodes]
        if not chain or poi_name not in nodes or xy is None:
            return None
        index = {name: i for i, name in enumerate(chain)}
        branches = {
            name: attach for name, attach in self._branches.items()
            if name in nodes and attach in index
        }
        if poi_name in index:
            target_index, tail = index[poi_name], []
        elif poi_name in branches:
            target_index, tail = index[branches[poi_name]], [poi_name]
        else:
            return None

        # 当前位置在支线上时（离支线点比离挂接点更近），先从挂接点进入
        start_branch = None
        best_margin = 0.0
        for name, attach in branches.items():
            attach_distance = math.hypot(
                nodes[attach][0] - xy[0], nodes[attach][1] - xy[1])
            branch_distance = math.hypot(
                nodes[name][0] - xy[0], nodes[name][1] - xy[1])
            margin = attach_distance - branch_distance
            if margin > best_margin:
                start_branch, best_margin = name, margin
        if start_branch is not None:
            if start_branch == poi_name:
                return [poi_name]
            start_attach = branches[start_branch]
            return (
                [start_attach]
                + self._walk(chain, index[start_attach], target_index)
                + tail)

        # 将当前位置投影到主线段。最近点可能在前方，不能把它当成
        # 已经过的起点；否则恢复导航时会跳过拐角或必经点。
        if len(chain) == 1:
            return [chain[0]] + tail
        candidates = []
        for i in range(len(chain) - 1):
            ax, ay = nodes[chain[i]][:2]
            bx, by = nodes[chain[i + 1]][:2]
            dx, dy = bx - ax, by - ay
            length2 = dx * dx + dy * dy
            if length2 == 0:
                continue
            t = max(0.0, min(1.0,
                ((xy[0] - ax) * dx + (xy[1] - ay) * dy) / length2))
            candidates.append((math.hypot(
                xy[0] - ax - t * dx, xy[1] - ay - t * dy), i + t))
        if not candidates:
            return None
        position = min(candidates)[1]
        if target_index >= position:
            first = math.ceil(position)
            names = chain[first:target_index + 1]
        else:
            first = math.floor(position)
            names = [chain[i] for i in range(first, target_index - 1, -1)]
        return names + tail

    @staticmethod
    def _walk(chain, start: int, target: int) -> list:
        """按主线顺序从 start 下标走到 target 下标，返回途经点（不含起点）。"""
        if start == target:
            return []
        step = 1 if target > start else -1
        return [chain[i] for i in range(start + step, target + step, step)]

    # ------------------------------------------------------------- 位姿/定位
    def _read_pose(self):
        try:
            pose = self._chassis.get_pose() or {}
            self._transport_failures = 0
            if self._pose_confident(pose):
                self._last_good_pose = (time.monotonic(), dict(pose))
            return pose
        except Exception as exc:  # noqa: BLE001 - 单次读失败按不可用处理
            self._record_transport_failure(exc)
            self._log.warn(f"读取当前位姿失败：{exc}")
            return None

    @staticmethod
    def _pose_xy(pose):
        """从位姿字典取 (x, y)；缺失或非法返回 None。"""
        if not isinstance(pose, dict):
            return None
        try:
            x = float(pose["x"])
            y = float(pose["y"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return x, y

    @classmethod
    def _pose_confident(cls, pose) -> bool:
        if cls._pose_xy(pose) is None:
            return False
        try:
            return (math.isfinite(float(pose["yaw"]))
                    and math.isfinite(float(pose["confidence"]))
                    and float(pose["confidence"]) > 0.0)
        except (KeyError, TypeError, ValueError):
            return False

    def _record_transport_failure(self, exc):
        if _transport_error(exc):
            self._transport_failures += 1
            if self._transport_failures >= _NETWORK_FAILURE_LIMIT:
                self._network_failed = True

    def _slam_status(self) -> int:
        """定位状态：0=正常，1=丢失，2=降级，5=等待重定位，6=重定位中。"""
        try:
            status = self._chassis.get_localization_status()
            self._transport_failures = 0
            return int(status.get("slam_status", -1))
        except Exception as exc:  # noqa: BLE001 - 查询抖动当成未知
            self._record_transport_failure(exc)
            self._log.warn(f"读取定位状态失败：{exc}")
            return -1

    def _slam_status_stable(self, retries: int = 3) -> int:
        """读定位状态；单次查询失败（-1）时短暂重读，避免把抖动当成未知状态。"""
        slam = self._slam_status()
        for _ in range(retries - 1):
            if slam != -1:
                break
            time.sleep(0.3)
            slam = self._slam_status()
        return slam

    def localization_state(self) -> dict:
        """读取完整定位状态；失败返回空字典。"""
        try:
            status = self._chassis.get_localization_status()
        except Exception as exc:  # noqa: BLE001 - 仅用于诊断
            self._log.warn(f"读取定位状态失败：{exc}")
            return {}
        return status if isinstance(status, dict) else {}

    def localization_idle(self, state=None) -> bool:
        """定位服务是否处于空闲（未启动）。这个状态不会自行恢复。"""
        if state is None:
            state = self.localization_state()
        try:
            return int(state.get("slam_status", -1)) == _IDLE_STATUS
        except (AttributeError, TypeError, ValueError):
            return False

    def localization_detail(self, state=None) -> str:
        """定位状态的可读描述：状态 / 类型 / 当前地图 / 置信度。"""
        if state is None:
            state = self.localization_state()
        if not state:
            return "定位状态读取失败"
        slam = state.get("slam_status", "?")
        slam_type = state.get("slam_type", "?")
        try:
            map_name = self._chassis.get_current_map().get("map_name") or "未加载"
        except Exception as exc:  # noqa: BLE001 - 仅用于日志
            map_name = f"读取失败（{exc}）"
        try:
            confidence = (self._chassis.get_pose() or {}).get("confidence", "?")
        except Exception as exc:  # noqa: BLE001 - 仅用于日志
            confidence = f"读取失败（{exc}）"
        return (f"slam_status={slam} slam_type={slam_type} "
                f"地图={map_name} 置信度={confidence}")

    def current_map_short_name(self) -> str:
        """当前地图短名：start_localization 只接受名称，不带路径和扩展名。"""
        try:
            full = str((self._chassis.get_current_map() or {}).get("map_name") or "")
        except Exception as exc:  # noqa: BLE001 - 读不到就不自动启动
            self._log.warn(f"读取当前地图失败：{exc}")
            return ""
        base = full.rsplit("/", 1)[-1]
        return base.rsplit(".", 1)[0] if "." in base else base

    def _recovery_pose(self):
        """重定位初值：优先吸附 3m 内点位，其次近期可信位姿。

        定位置信度为 0 时 REST 位姿仍可能保留最后位置，但不一定可信；
        只有附近有点位可吸附、或刚丢失定位（近期位姿仍可信）才自动重定位。
        """
        pose = self._read_pose()
        xy = self._pose_xy(pose)
        if xy is not None:
            best = None
            for name, (px, py, pyaw) in (self._load_pois() or {}).items():
                distance = math.hypot(px - xy[0], py - xy[1])
                if distance <= _POI_SNAP_M and (best is None or distance < best[0]):
                    best = (distance, name, (px, py, pyaw))
            if best is not None:
                self._log.info(
                    f"重定位初值吸附点位 {best[1]}（相距 {best[0]:.2f}m）")
                return best[2]
        trusted = self._last_good_pose
        # 行驶中丢失（3/5）时，丢失前的可信位姿比冻结位姿可靠得多；
        # 冻结位姿会随轮速推算漂移（实测 2 米内），不能当重定位初值。
        max_age = (_LAST_GOOD_POSE_MAX_AGE_S
                   if self._slam_status() == _IDLE_STATUS else 3600.0)
        if (trusted is not None
                and time.monotonic() - trusted[0] <= max_age):
            p = trusted[1]
            return (p["x"], p["y"], p["yaw"])
        # 低置信度瞬间 REST 位姿会冻结在最后一帧，通常仍是准的；
        # 空闲(4)可能是重启后遗留的旧坐标，不能作为初值。
        if xy is not None and self._slam_status() in _RELOCALIZE_TRIGGER_STATUS:
            self._log.warn(
                f"没有可信历史位姿，退化使用当前冻结位姿："
                f"x={xy[0]:.2f} y={xy[1]:.2f} yaw={pose.get('yaw')}")
            return (xy[0], xy[1], float(pose.get("yaw") or 0.0))
        return None

    def restart_localization(self,
                             timeout_s: float = _LOCALIZATION_START_WAIT_S) -> bool:
        """定位空闲（4）时重新加载当前地图并启动激光定位。

        空闲说明 SLAM 服务没启动，此时 recover_localization 无效，必须先
        start_localization；状态走到 5（等待重定位）再用可信初值补一次。
        平台自身也有自动定位流程，这里带冷却，避免反复下发互相打断。
        """
        now = time.monotonic()
        if (self._localization_restarted_at is not None
                and now - self._localization_restarted_at
                < _LOCALIZATION_RESTART_COOLDOWN_S):
            self._log.warn("最近刚尝试过重新启动定位，本次不重复下发")
            return False
        map_name = self.current_map_short_name()
        if not map_name:
            self._log.error("当前地图为空，无法自动启动定位")
            return False
        self._localization_restarted_at = now
        try:
            self._chassis.start_localization(slam_type=1, map_name=map_name)
        except Exception as exc:  # noqa: BLE001 - 启动失败交给上层提示
            self._log.error(f"自动启动定位失败（地图={map_name}）：{exc}")
            return False
        self._log.warn(
            f"定位服务空闲，已重新加载地图 {map_name} 并启动激光定位")

        deadline = time.monotonic() + max(0.0, timeout_s)
        recovery_pose = None
        recovery_sent = False
        while time.monotonic() < deadline:
            state = self.localization_state()
            try:
                slam = int(state.get("slam_status", -1))
            except (TypeError, ValueError):
                slam = -1
            if slam == 0:
                if self._wait_slam_ok(
                        max(_LOCALIZATION_STABLE_S, deadline - time.monotonic())):
                    self._log.info("定位已自动恢复并连续稳定")
                    return True
                return False
            if slam in _RELOCALIZE_TRIGGER_STATUS and not recovery_sent:
                if recovery_pose is None:
                    recovery_pose = self._recovery_pose()
                if recovery_pose is None:
                    self._log.error("没有可信初始位姿，放弃自动重定位")
                    return False
                self._log.warn(
                    f"定位等待重定位（slam_status={slam}），按可信初值重定位："
                    f"x={recovery_pose[0]:.2f} y={recovery_pose[1]:.2f} "
                    f"yaw={recovery_pose[2]:.2f}")
                self._trigger_recover(*recovery_pose, timeout_s=timeout_s)
                recovery_sent = True
            time.sleep(0.5)
        self._log.error(
            f"自动启动定位后 {timeout_s:.0f}s 内仍未就绪："
            f"{self.localization_detail()}")
        return False

    def _trigger_recover(self, x: float, y: float, yaw: float,
                         timeout_s: float) -> bool:
        """触发重定位；接口 500 但任务已受理时也继续等定位状态。"""
        try:
            self._chassis.recover_localization(
                x=x, y=y, yaw=yaw, max_time_ms=int(timeout_s * 1000))
            return True
        except Exception as exc:  # noqa: BLE001
            # 底盘偶尔返回 500 Internal error，但响应里 accepted=true，
            # 实际重定位任务已经提交，这时不该直接当失败。
            if '"accepted":true' in str(exc).replace(" ", ""):
                self._log.warn(
                    f"重定位接口报错但任务已受理，继续等定位：{exc}")
                return True
            self._log.error(f"触发重定位失败：{exc}")
            return False

    def _wait_slam_ok(self, timeout_s: float) -> bool:
        """状态正常且位姿可信连续保持 3 秒；异常或查询失败重新计时。

        相邻导航段之间底盘一直在正常定位，上一段结束时的连续正常观测
        可以直接沿用到下一段，不用每段重新等满 3 秒。
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        stable_since = None
        if (self._last_ok_at is not None and self._ok_since is not None
                and time.monotonic() - self._last_ok_at
                <= _LOCALIZATION_RECENT_S):
            stable_since = self._ok_since
        while time.monotonic() < deadline:
            slam = self._slam_status()
            pose = self._read_pose() if slam == 0 else None
            if self._network_failed:
                return False
            now = time.monotonic()
            if now >= deadline:
                break
            if slam == 0 and self._pose_confident(pose):
                if stable_since is None:
                    stable_since = now
                self._last_ok_at = now
                self._ok_since = stable_since
                if now - stable_since >= _LOCALIZATION_STABLE_S:
                    return True
            else:
                stable_since = None
                self._last_ok_at = None
                self._ok_since = None
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
        return False

    def recover_here(self, timeout_s: float = _RECOVER_WAIT_S) -> bool:
        """最多主动恢复一次：优先吸附 3m 内点位，其次近期可信位姿。"""
        if timeout_s <= _LOCALIZATION_STABLE_S:
            return False
        slam = self._slam_status_stable()
        if slam == 0 or slam in _RELOCALIZE_BUSY_STATUS:
            return self._wait_slam_ok(timeout_s)
        if slam not in _RELOCALIZE_TRIGGER_STATUS or self._recover_attempted:
            return False
        pose = self._recovery_pose()
        if pose is None:
            self._log.error("没有可信初始位姿，拒绝盲目重定位")
            return False
        self._recover_attempted = True
        self._log.warn(
            f"仅尝试一次原地重定位：x={pose[0]:.2f} y={pose[1]:.2f} "
            f"yaw={pose[2]:.2f}")
        deadline = time.monotonic() + timeout_s
        if not self._trigger_recover(*pose, timeout_s=timeout_s):
            return False
        if self._wait_slam_ok(max(0.0, deadline - time.monotonic())):
            self._log.info("重定位后已连续稳定 3 秒")
            return True
        # 原地重定位在部分区段会一直卡在 3/0；实测重载地图能恢复，
        # 这里做一次强恢复，仍失败才交给上层终止。
        self._log.warn("原地重定位未收敛，尝试重载地图后按可信位姿重新匹配")
        return self.restart_localization(
            max(_LOCALIZATION_STABLE_S, min(timeout_s, _LOCALIZATION_START_WAIT_S)))

    def relocalize(self, poi_name: str, timeout_s: float = 20.0) -> bool:
        """启动时先等定位稳定；待命点名称不作为机器人已在该处的依据。"""
        if self._wait_slam_ok(min(_LOCALIZATION_PASSIVE_WAIT_S, timeout_s)):
            return True
        return self.recover_here(max(0.0, timeout_s - _LOCALIZATION_PASSIVE_WAIT_S))

    def check_ready(self) -> tuple[bool, str]:
        """导航前检查：定位可恢复 + 打开 AEB。

        可恢复状态（1/2/3/5）和底盘正在恢复（6/7/8）不在这里拦，
        交给 go() 按状态决定等待还是重定位；空闲（4）先自动重新加载
        当前地图启动定位，启动失败才拒绝。
        """
        if self._pending_cancel and not self._cancel_action():
            return False, "底盘通信未恢复，之前的任务还不能确认已取消，请先检查连接"
        self._network_failed = False
        self._transport_failures = 0
        try:
            status = self._chassis.get_localization_status()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"读取定位状态失败：{exc}")
            return False, ("我与底盘通信中断了，先不下发新任务，请检查底盘连接"
                           if _transport_error(exc) else "我这边读不到定位状态，先不移动了")
        slam = -1
        try:
            slam = int(status.get("slam_status", -1))
        except Exception:  # noqa: BLE001
            pass
        if slam == _IDLE_STATUS:
            # 空闲说明定位服务没启动，先尝试自动重新加载地图再判断。
            self._log.warn(
                f"定位服务处于空闲状态，尝试自动启动定位："
                f"{self.localization_detail(status)}")
            if self.restart_localization():
                slam = 0
            else:
                return False, (
                    "机器人定位服务没有启动，自动重启也没成功，"
                    "请联系工作人员检查底盘定位")
        if slam != 0:
            if (slam in _RELOCALIZE_TRIGGER_STATUS
                    or slam in _RELOCALIZE_BUSY_STATUS):
                self._log.warn(
                    f"定位未就绪（{self.localization_detail(status)}），"
                    "交给导航按状态恢复")
            else:
                self._log.error(
                    f"定位状态不可用（{self.localization_detail(status)}），{status}")
                return False, "我现在还没定位好，稍等一下再带你过去"
        if not self._check_health(allow_localization_recovery=True):
            return False, self._health_message
        self._set_aeb(True)
        return True, ""

    # ------------------------------------------------------------- 导航主流程
    def go(self, poi_name: str, timeout_s: float, guard=None) -> str:
        """定点导航到点位，返回 success / failure / blocked / timeout / localization / health / network。

        guard: 可选的前向防撞对象，判到障碍就取消本次导航并返回 blocked。
        整条路线按 places.yaml 的主线和支线拆成多段，逐段下发 move_to；
        中途被打断后由上层再次调用，按当前位姿重新规划剩余路段。
        """
        deadline = time.monotonic() + timeout_s
        self._recover_attempted = False
        self._network_failed = False
        self._transport_failures = 0
        if self._pending_cancel and not self._cancel_action():
            return "network"
        pois = self._load_pois()
        if not pois:
            # 点位读取偶发失败：短暂重试一次，别把一次抖动当成地图没有点位
            time.sleep(0.5)
            pois = self._load_pois()
        if poi_name not in pois:
            self._log.error(f"地图上没有点位 {poi_name}，无法导航")
            return "failure"

        # 出发前就有人 / 上次被防撞取消：必须等前方持续清空再下发。
        # 否则会先朝人走一段，或者障碍刚好闪开一帧就重发，
        # 变成反复取消/重发。
        if guard is not None:
            must_wait = (
                self._last_guard_cancel_at is not None or guard.blocked())
            if must_wait:
                if not self._wait_guard_clear(guard):
                    # 记下来，下次进 go 先要求连续清空，避免刚闪开就冲
                    self._last_guard_cancel_at = time.monotonic()
                    self._log.warn("前方持续有障碍，暂不下发导航")
                    return "blocked"
                self._last_guard_cancel_at = None

        if not self._check_health(allow_localization_recovery=True):
            return "network" if self._network_failed else "health"
        if not self._check_localization(deadline):
            return "network" if self._network_failed else "localization"
        if not self._check_health():
            return "network" if self._network_failed else "health"
        self._set_aeb(True)
        self._apply_speed()
        try:
            if self._track_id and poi_name not in self._branches:
                return self._navigate_by_track(poi_name, deadline, guard)
            legs = self.plan_route(poi_name)
            if not legs:
                self._log.info(f"当前位置已在 {poi_name} 附近，无需移动")
                return "success"
            self._log.info(
                f"定点导航到 {poi_name}：{len(legs)} 个导航段，"
                + " -> ".join(
                    f"{leg.name}({leg.x:.2f},{leg.y:.2f})" for leg in legs))
            for index, leg in enumerate(legs):
                result = self._navigate_leg(leg, deadline, guard)
                if result != "success":
                    return result
                if index + 1 < len(legs):
                    # 让底盘停稳再下发下一段，避免停稳过渡期被瞬拒
                    time.sleep(_LEG_SETTLE_S)
            return "success"
        finally:
            # 取消没有获确认时保持 AEB，不能假设底盘已经停下。
            if self._cancel_action():
                self._set_aeb(False)
            else:
                self._log.error("任务取消尚未确认，保留 AEB 并禁止新的导航任务")

    def rotate_by(self, angle_rad: float, timeout_s: float = 25.0) -> str:
        """原地转身；angle_rad 正为左转（逆时针），负为右转。"""
        if abs(angle_rad) < 1e-3:
            return "success"
        deadline = time.monotonic() + timeout_s
        if not self._check_localization(deadline):
            return "network" if self._network_failed else "localization"
        if not self._check_health():
            return "network" if self._network_failed else "health"
        self._set_aeb(True)
        try:
            try:
                response = self._chassis.move_by(
                    direction=max(-math.pi, min(math.pi, float(angle_rad))),
                    move_mode=1)
            except Exception as exc:  # noqa: BLE001 - 下发失败只回报
                self._record_transport_failure(exc)
                self._log.error(f"下发原地转身失败：{exc}")
                return "network" if _transport_error(exc) else "failure"
            action_id = (
                str(response.get("action_id") or "")
                if isinstance(response, dict) else "")
            if not action_id or response.get("success") is False:
                self._log.error(f"原地转身未返回 action_id：{response!r}")
                return "failure"
            self._active_action_id = action_id
            self._log.info(
                f"已下发原地转身 {math.degrees(angle_rad):+.0f}°，"
                f"action_id={action_id}")
            return self._run_action(action_id, "原地转身", deadline)
        finally:
            if self._cancel_action():
                self._set_aeb(False)

    def _navigate_by_track(self, poi_name: str, deadline: float, guard) -> str:
        """走底盘原生轨道导航：整条轨道一次下发，由底盘按轨道点行驶。"""
        target = (self._load_pois() or {}).get(poi_name)
        leg = (Leg(poi_name, target[0], target[1], target[2])
               if target else None)
        self._log.info(
            f"轨道导航到 {poi_name}：track_id={self._track_id}")
        try:
            result = self._chassis.move_to_locations_poi(
                track_id=self._track_id,
                poi_name=poi_name,
                uptrack_enabled=True,
            )
        except Exception as exc:  # noqa: BLE001 - 下发失败按普通失败处理
            self._log.error(
                f"轨道任务下发失败（track={self._track_id}）：{exc}")
            return "failure"
        action_id = str((result or {}).get("action_id") or "")
        if not action_id:
            self._log.error(f"轨道任务未返回 action_id：{result}")
            return "failure"
        self._log.info(f"已下发轨道任务，action_id={action_id}")
        return self._run_action(
            poi_name, action_id,
            max(0.0, deadline - time.monotonic()), guard, leg)

    def _navigate_leg(self, leg, deadline: float, guard) -> str:
        """逐段定位门控；定位未稳定时不靠拆短段或反复重发碰运气。"""
        for attempt in range(_LEG_ATTEMPTS):
            if time.monotonic() >= deadline:
                return "timeout"
            if attempt:
                self._log.warn(f"{leg.name} 重试前重新检查定位（{attempt+1}/{_LEG_ATTEMPTS}）")
                time.sleep(_REDISPATCH_SETTLE_S)
            if not self._check_health(allow_localization_recovery=True):
                return "network" if self._network_failed else "health"
            if not self._check_localization(deadline):
                return "network" if self._network_failed else "localization"
            # 定位门控通过仍需确认故障码已消除，不能带故障下发运动。
            if not self._check_health():
                return "network" if self._network_failed else "health"
            if time.monotonic() >= deadline:
                return "timeout"
            action_id = self._dispatch(leg)
            if action_id is None:
                if self._pending_cancel:
                    return "network"
                continue
            outcome = self._run_action(action_id, leg.name, deadline, guard, leg)
            if outcome in ("success", "blocked", "timeout", "localization", "network"):
                return outcome
        self._log.error(f"{leg.name} 连续 {_LEG_ATTEMPTS} 次导航任务失败")
        return "failure"

    def _check_localization(self, deadline=None) -> bool:
        """每次下发前连续稳定门控；先自然恢复，最多一次主动恢复。"""
        if deadline is None:
            deadline = time.monotonic() + _RECOVER_WAIT_S
        if self._slam_status() == _IDLE_STATUS:
            # 空闲不会自行恢复，先自动重新加载地图；失败才放弃本次导航。
            self._log.warn(
                f"定位服务处于空闲状态，尝试自动启动定位："
                f"{self.localization_detail()}")
            if not self.restart_localization():
                self._log.error("定位服务无法自动恢复，放弃本次导航")
                return False
        budget = min(_LOCALIZATION_PASSIVE_WAIT_S,
                     max(0.0, deadline - time.monotonic()))
        if self._wait_slam_ok(budget):
            return True
        if self._network_failed:
            return False
        slam = self._slam_status()
        self._log.warn(
            f"定位未连续稳定，暂停下发运动任务：{self.localization_detail()}")
        remaining = min(_RECOVER_WAIT_S, max(0.0, deadline - time.monotonic()))
        if remaining < _LOCALIZATION_STABLE_S:
            return False
        # 3/5 低置信度：先原地重定位；失败或没有近期可信位姿时，
        # 再重载地图重新匹配，避免“一次跌落之后永远被拒单”。
        if slam in _RELOCALIZE_TRIGGER_STATUS:
            if not self._recover_attempted and self.recover_here(remaining):
                return True
            self._log.warn("原地重定位未能恢复，尝试重载地图后重新匹配")
            return self.restart_localization(
                max(_LOCALIZATION_STABLE_S,
                    min(remaining, _LOCALIZATION_START_WAIT_S)))
        return self._wait_slam_ok(remaining)

    @property
    def health_message(self) -> str:
        return self._health_message

    def _check_health(self, allow_localization_recovery=False) -> bool:
        """只在恢复等待入口暂缓 0x03020003；运动下发前所有 A/B 故障仍阻断。"""
        try:
            health = self._chassis.get_robot_health()
        except Exception as exc:  # noqa: BLE001 - 读不到健康状态不阻断导航
            self._record_transport_failure(exc)
            self._health_message = "读不到底盘健康状态，请检查底盘连接后再叫我"
            self._log.error(f"读取底盘健康状态失败，暂停运动：{exc}")
            return False
        entries = list(getattr(health, "baseError", []) or [])
        blocking = [entry for entry in entries if _is_blocking_dtc(entry)]
        if allow_localization_recovery:
            deferred = [entry for entry in blocking
                        if _dtc_id(entry) == _DTC_LOCALIZATION_LOW_CONFIDENCE]
            if deferred:
                self._log.warn("定位置信度低，进入有时限的定位恢复等待；恢复前不下发运动")
                blocking = [entry for entry in blocking if entry not in deferred]
        warnings = [entry for entry in entries if not _is_blocking_dtc(entry)]
        if warnings:
            self._log.warn(
                "底盘告警（不影响运动，继续）："
                + "；".join(_describe_dtc(entry) for entry in warnings))
        if blocking:
            if any("维护模式" in str(getattr(e, "description", ""))
                   for e in blocking):
                self._health_message = (
                    "维护模式还开着，请先弹起维护按钮，退出手动模式后再叫我")
            elif any(_dtc_id(entry) == _DTC_LOCALIZATION_LOW_CONFIDENCE
                     for entry in blocking):
                self._health_message = (
                    "定位置信度异常（0x03020003），请工作人员检查地图定位")
            else:
                self._health_message = "底盘有故障，暂时不能移动，请先检查底盘告警"
            self._log.error(
                "底盘健康检查未通过："
                + "；".join(_describe_dtc(entry) for entry in blocking))
            return False
        if not entries and (
            getattr(health, "hasFatal", False)
            or getattr(health, "hasError", False)
        ):
            # 标志为真却没有任何 DTC 明细：无法判断严重程度，保守拒绝
            self._health_message = "底盘有故障，但读不到故障详情，请先检查底盘状态"
            self._log.error(
                "底盘健康检查未通过：未返回 DTC 详情"
                f"（hasFatal={getattr(health, 'hasFatal', None)}, "
                f"hasError={getattr(health, 'hasError', None)}）")
            return False
        return True

    def _run_action(self, action_id: str, label: str,
                    deadline: float, guard, leg=None) -> str:
        """轮询底盘动作到终态。

        返回 success / failure / timeout / blocked。
        failure 表示底盘导航任务失败，由调用方重新下发 move_to；
        blocked 表示前向防撞取消。
        """
        started = time.monotonic()
        blocked_hits = 0
        next_guard_log = 0.0
        last_log = 0.0
        localization_bad_since = None
        bad_ref_remaining = None
        query_failures = 0
        while True:
            now = time.monotonic()
            if now >= deadline:
                self._log.error(
                    f"导航到 {label} 超时，取消任务 {action_id}")
                self._cancel_and_settle(action_id)
                return "timeout"

            if guard is not None and now - started >= _GUARD_GRACE_S:
                current = guard.distance()
                elapsed = now - started
                if current is not None and elapsed >= next_guard_log:
                    self._log.info(
                        f"防撞监测：正前方 {current:.2f}m，"
                        f"无效像素 {guard.zero_ratio():.0%}（阈值 "
                        f"{guard.stop_distance():.2f}m）")
                    next_guard_log = elapsed + 2.0
                if guard.blocked():
                    blocked_hits += 1
                else:
                    blocked_hits = 0
                if blocked_hits >= _GUARD_CONFIRM:
                    distance = "未知" if current is None else f"{current:.2f}m"
                    self._log.warn(
                        f"前方 {distance} 有障碍，取消导航任务 {action_id}")
                    self._cancel_and_settle(action_id)
                    self._last_guard_cancel_at = time.monotonic()
                    return "blocked"

            try:
                status = self._chassis.get_action_status(action_id)
                query_failures = 0
            except Exception as exc:  # noqa: BLE001 - 查询抖动继续等
                query_failures += 1
                self._log.warn(f"查询导航状态失败（{query_failures}/{_NETWORK_FAILURE_LIMIT}）：{exc}")
                if query_failures >= _NETWORK_FAILURE_LIMIT:
                    self._network_failed = True
                    self._pending_cancel = True
                    self._active_action_id = action_id
                    self._log.error("连续查询失败，退出等待；任务状态未知，恢复通信后须先确认取消")
                    return "network"
                time.sleep(_POLL_S)
                continue
            state = classify(status)
            if state != "running":
                self._active_action_id = None
            if state == "success":
                # 低置信度期间 REST 位姿会冻结，先等可信新位姿再做位置校验。
                stable = self._wait_slam_ok(min(
                    _RECOVER_WAIT_S, max(0.0, deadline - time.monotonic())))
                pose = self._read_pose()
                xy = self._pose_xy(pose)
                confident = self._pose_confident(pose)
                at_target = (
                    leg is not None and xy is not None
                    and math.hypot(leg.x - xy[0], leg.y - xy[1])
                    <= _ARRIVAL_TOLERANCE_M)
                # 底盘定位会间歇抖动（0↔3），只要位姿可信且已在目标容差内，
                # 就不该因为凑不满“连续 3 秒正常”而把已到点判成失败。
                fallback_arrival = False
                if not stable and not (confident and at_target):
                    near_target = (
                        leg is not None and xy is not None
                        and math.hypot(leg.x - xy[0], leg.y - xy[1])
                        <= _ARRIVAL_FALLBACK_M)
                    if near_target:
                        fallback_arrival = True
                        self._log.warn(
                            f"{label} 定位异常，但距目标仅 "
                            f"{math.hypot(leg.x - xy[0], leg.y - xy[1]):.2f}m，"
                            f"按到达处理：{self.localization_detail()}")
                    else:
                        self._log.error(
                            f"{label} 动作结束，但定位未连续稳定，不能确认到达："
                            f"{self.localization_detail()}")
                        return "network" if self._network_failed else "localization"
                if leg is not None and not fallback_arrival:
                    if not confident:
                        return "localization"
                    if not at_target:
                        self._log.error(
                            f"{label} 动作报成功但未通过到点位置校验：{xy}")
                        return "failure"
                self._log.info(
                    f"到达 {label}，用时 {now - started:.1f}s")
                return "success"
            if state == "failure":
                elapsed = time.monotonic() - started
                note = "（从未起步）" if _never_started(status) else ""
                self._log.error(
                    f"导航到 {label} 失败，用时 {elapsed:.1f}s{note}：{status}")
                self._log.warn(
                    f"失败时 {self.localization_detail()}，位姿={self._read_pose()}")
                return "failure"

            if now - last_log >= 2.0:
                last_log = now
                pose = self._read_pose()
                slam = self._slam_status()
                xy = self._pose_xy(pose)
                remaining = None
                if xy is not None and leg is not None:
                    remaining = math.hypot(leg.x - xy[0], leg.y - xy[1])
                if slam != 0 or not self._pose_confident(pose):
                    self._last_ok_at = None
                    self._ok_since = None
                    if localization_bad_since is None:
                        localization_bad_since = now
                        bad_ref_remaining = remaining
                    elif now - localization_bad_since >= _MOTION_BAD_CANCEL_S:
                        # 底盘定位会短暂抖动（0↔3），只要车还在按预期靠近
                        # 目标就继续跑；单纯因为状态闪一下取消会打断正常任务。
                        progressed = (
                            remaining is not None
                            and bad_ref_remaining is not None
                            and bad_ref_remaining - remaining >= _BAD_PROGRESS_M)
                        if progressed:
                            self._log.info(
                                f"{label} 定位状态异常但仍在前进"
                                f"（剩余 {remaining:.2f}m），继续任务："
                                f"{self.localization_detail()}")
                            localization_bad_since = now
                            bad_ref_remaining = remaining
                        else:
                            self._log.warn(
                                f"{label} 行驶中定位持续异常且没有前进，"
                                f"取消本任务并等待稳定："
                                f"{self.localization_detail()}")
                            self._cancel_and_settle(action_id)
                            return "failure"
                else:
                    localization_bad_since = None
                    bad_ref_remaining = None
                    # 行驶中持续正常：记录连续正常的起点，供下一段直接复用
                    self._last_ok_at = now
                    if self._ok_since is None:
                        self._ok_since = now
                detail = f"，slam_status={slam}，置信度={pose.get('confidence') if isinstance(pose, dict) else None}"
                if xy is not None and leg is not None:
                    detail += (f"，位置({xy[0]:.2f},{xy[1]:.2f})，"
                              f"距目标 {math.hypot(leg.x-xy[0], leg.y-xy[1]):.2f}m")
                self._log.info(
                    f"{label} 运动进行中，剩余超时 "
                    f"{max(0.0, deadline - now):.0f}s{detail}")
            time.sleep(_POLL_S)

    # ------------------------------------------------------------- 运动下发
    def _dispatch(self, leg):
        """按规划坐标和朝向下发 move_to，返回 action_id；失败返回 None。"""
        try:
            # 显式使用规划坐标和行进朝向；POI 保存的 yaw 往往统一为 0，
            # 不能代表返程或支线的行进方向。
            response = self._chassis.move_to(
                leg.x, leg.y, yaw=leg.yaw, flags=_POI_FLAGS)
        except Exception as exc:  # noqa: BLE001
            # PUT 超时不等于底盘未接单，不能直接重新发送另一个任务。
            self._network_failed = True
            self._pending_cancel = True
            self._active_action_id = None
            self._log.error(f"下发 move_to({leg.name}) 结果不确定，禁止重发：{exc}")
            return None
        action_id = (
            str(response.get("action_id") or "")
            if isinstance(response, dict) else ""
        )
        if not action_id or response.get("success") is False:
            if not isinstance(response, dict) or response.get("success") is not False:
                self._pending_cancel = True
                self._network_failed = True
            self._log.error(
                f"导航到 {leg.name} 未返回 action_id：{response!r}")
            return None
        self._active_action_id = action_id
        self._log.info(
            f"已下发 move_to({leg.name}, x={leg.x:.2f}, y={leg.y:.2f}, "
            f"yaw={leg.yaw:.2f})，action_id={action_id}")
        return action_id

    def _wait_guard_clear(self, guard) -> bool:
        """要求前方连续清空 _GUARD_CLEAR_HOLD_S 才放行，避免抖动式反复取消。"""
        deadline = time.monotonic() + _GUARD_CLEAR_WAIT_S
        clear_since = None
        while time.monotonic() < deadline:
            if guard.blocked():
                clear_since = None
            else:
                if clear_since is None:
                    clear_since = time.monotonic()
                elif time.monotonic() - clear_since >= _GUARD_CLEAR_HOLD_S:
                    return True
            time.sleep(_POLL_S)
        return False

    # --------------------------------------------------------------- 收尾
    def _apply_speed(self) -> None:
        """按配置下发底盘限速；失败只告警，不影响导航。"""
        if self._max_speed <= 0 or self._max_speed == self._speed_applied:
            return
        try:
            self._chassis.set_speed_params(max_moving_speed=self._max_speed)
            self._speed_applied = self._max_speed
            self._log.info(f"底盘限速设为 {self._max_speed:.2f} m/s")
        except Exception as exc:  # noqa: BLE001 - 限速失败不阻断导航
            self._log.warn(f"设置底盘限速失败（继续）：{exc}")

    def _set_aeb(self, on: bool) -> None:
        try:
            self._chassis.set_aeb_control("On" if on else "Off")
        except Exception as exc:  # noqa: BLE001 - AEB 失败不阻塞主流程
            state = "开启" if on else "关闭"
            self._log.warn(f"AEB {state}失败（继续）：{exc}")

    def _cancel_and_settle(self, action_id: str) -> None:
        """取消动作并等它退出 running，再补一次取消。

        动作没退干净就下发新任务，底盘会在停稳过渡期以
        distance_time=0 / failed 直接拒绝，并容易带出定位降级。
        """
        self._cancel(action_id)
        deadline = time.monotonic() + _CANCEL_SETTLE_S
        while time.monotonic() < deadline:
            try:
                status = self._chassis.get_action_status(action_id)
            except Exception as exc:  # noqa: BLE001 - 查询失败就不再等
                self._log.warn(f"查询取消状态失败，停止等待：{exc}")
                break
            if classify(status) != "running":
                break
            time.sleep(_POLL_S)
        self._cancel(action_id)

    def _cancel(self, action_id=None) -> bool:
        try:
            response = self._chassis.cancel_action(
                [action_id] if action_id else None)
            if not isinstance(response, dict) or response.get("success") is not True:
                self._pending_cancel = True
                self._log.error(f"底盘未确认取消：{response}")
                return False
            return True
        except Exception as exc:  # noqa: BLE001
            self._pending_cancel = True
            self._log.error(f"取消导航未获确认（不能保证底盘已停）：{exc}")
            return False

    def cancel(self) -> bool:
        """外部请求停车：标记待取消并立即取消当前活动任务。"""
        self._pending_cancel = True
        return self._cancel_action()

    def pose(self):
        """当前位置（含定位置信度），读不到返回 None。"""
        try:
            return self._chassis.get_pose()
        except Exception:  # noqa: BLE001 - 查询失败不打断对话
            return None

    def _cancel_action(self) -> bool:
        """只清理本节点的活动任务；受理结果未知时须取消全部并取得确认。"""
        if self._active_action_id is None and not self._pending_cancel:
            return True
        action_id = self._active_action_id
        if not self._cancel(action_id):
            return False
        if action_id is not None:
            try:
                state = classify(self._chassis.get_action_status(action_id))
            except Exception as exc:  # noqa: BLE001
                self._pending_cancel = True
                self._log.error(f"取消后仍读不到任务终态：{exc}")
                return False
            if state == "running":
                self._pending_cancel = True
                return False
        self._pending_cancel = False
        self._active_action_id = None
        return True
