"""导航前向防撞：用头部深度相机检测正前方障碍。

底盘自带的 AEB 在自主导航模式下不生效（实测人站路上照样撞），
这里自己订阅深度图，取画面中央区域的最近距离，判到就取消导航，
等人离开后再重新下发同一个导航任务。
"""

from __future__ import annotations

import threading
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# ROI 内无效像素（贴得太近时深度输出 0）占比超过该值，视为前方有近处障碍
_ZERO_RATIO_BLOCK = 0.06


class FrontGuard:
    """只回答“正前方有没有东西”，不做识别、不发控制指令。"""

    def __init__(
        self,
        node,
        topic: str = "/ob_camera_head/depth/image_raw",
        stop_distance: float = 0.5,
        resume_distance: float = 0.8,
        stale_s: float = 1.5,
        turn_distance: float | None = None,
    ) -> None:
        self._log = node.get_logger()
        self._stop = float(stop_distance)
        # 迟滞：挡住按 stop 判，恢复要退到 resume 以外，避免在阈值附近反复停车
        self._resume = max(float(resume_distance), self._stop + 0.1)
        # 原地转向时用更近的阈值：此时正前方往往就是刚讲解完的展品或墙
        self._turn_stop = min(float(turn_distance), self._stop) \
            if turn_distance is not None else self._stop
        self._turning = False
        self._stale_s = float(stale_s)
        self._lock = threading.Lock()
        self._distance: float | None = None
        self._zero_ratio = 0.0
        self._stamp = 0.0
        # 深度图发布端是 best-effort，用传感器 QoS 才能收到
        node.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        try:
            if msg.encoding == "16UC1":
                img = np.frombuffer(msg.data, dtype=np.uint16)
                img = img.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                img = np.frombuffer(msg.data, dtype=np.float32)
                img = img.reshape(msg.height, msg.step // 4)[:, :msg.width]
            else:
                return
        except Exception:  # noqa: BLE001 - 单帧解析异常忽略
            return

        h, w = img.shape[:2]
        # 只看画面中央，避开地面和天花板
        roi = img[int(h * 0.25):int(h * 0.60), int(w * 0.35):int(w * 0.65)]
        valid = roi[np.isfinite(roi) & (roi > 0.05)]
        # 用低分位数而不是最小值，避免个别噪点误触发
        zero_ratio = float(np.count_nonzero(roi <= 0.05)) / float(roi.size)
        with self._lock:
            self._stamp = time.monotonic()
            self._zero_ratio = zero_ratio
            if valid.size == 0:
                # ROI 全部无效：多半是人贴到镜头前或镜头被挡，按最近距离处理，
                # 不能丢帧，否则恰恰在最危险时判定成“前方没东西”。
                self._distance = 0.0
            else:
                self._distance = float(np.percentile(valid, 5))

    def distance(self) -> float | None:
        """正前方最近距离（米）；没有新数据时返回 None。"""
        with self._lock:
            if (self._distance is None
                    or time.monotonic() - self._stamp > self._stale_s):
                return None
            return self._distance

    def zero_ratio(self) -> float:
        """ROI 内无效/过近像素占比（0~1）。"""
        with self._lock:
            return self._zero_ratio

    def stop_distance(self) -> float:
        """当前停车距离阈值（米）。"""
        return self._stop

    def set_turning(self, turning: bool) -> None:
        """切换“原地转向”模式：转向时正前方就是展品/墙，用更近的阈值。"""
        self._turning = bool(turning)

    @property
    def _stop_now(self) -> float:
        return self._turn_stop if self._turning else self._stop

    @property
    def _resume_now(self) -> float:
        base = self._turn_stop + 0.1 if self._turning else self._resume
        return max(base, self._stop_now + 0.1)

    def blocked(self) -> bool:
        """前方障碍是否近到必须停车；深度数据过期时不算挡住。"""
        distance = self.distance()
        if distance is None:
            return False
        if distance <= self._stop_now:
            return True
        # 贴得太近时深度会输出大片 0（无效值），这时 p5 反而看到远处背景
        return self.zero_ratio() >= _ZERO_RATIO_BLOCK

    def clear(self) -> bool:
        """前方是否已退到安全距离以外；数据过期时返回 False（不据此恢复）。"""
        distance = self.distance()
        if distance is None:
            return False
        return distance >= self._resume_now and self.zero_ratio() < _ZERO_RATIO_BLOCK


class GuardNode:
    """持有 ROS 节点并在后台线程 spin，主流程只读最近距离。"""

    def __init__(self, topic: str, stop_distance: float,
                 turn_distance: float | None = None) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        self._own_context = not rclpy.ok()
        if self._own_context:
            rclpy.init()
        self._node = rclpy.create_node("learn3_presence")
        self.guard = FrontGuard(
            self._node, topic, stop_distance,
            resume_distance=float(stop_distance) + 0.3,
            turn_distance=turn_distance)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        """后台 spin；Ctrl+C 关闭 context 时静默退出，不要打 traceback。"""
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - ExternalShutdownException 等关闭期异常
            pass

    def distance(self) -> float | None:
        """正前方最近距离（米）；没有新数据时返回 None。"""
        return self.guard.distance()

    def blocked(self) -> bool:
        """前方障碍是否近到必须停车。"""
        return self.guard.blocked()

    def clear(self) -> bool:
        """前方是否已退到安全距离以外。"""
        return self.guard.clear()

    def zero_ratio(self) -> float:
        """ROI 内无效/过近像素占比，供导航空闲时打印诊断。"""
        return self.guard.zero_ratio()

    def stop_distance(self) -> float:
        """当前停车阈值（米）。"""
        return self.guard.stop_distance()

    def set_turning(self, turning: bool) -> None:
        """转向时切到更近的阈值，直行时切回。"""
        self.guard.set_turning(turning)

    def shutdown(self) -> None:
        try:
            # executor.shutdown() 只是让 spin() 尽快返回，并不等它结束；
            # 必须 join 之后再销毁节点，否则会在 rmw 还在用节点时关底层 DDS 线程，
            # 进程会以 “terminate called without an active exception” 异常退出。
            self._executor.shutdown()
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
            self._node.destroy_node()
        finally:
            if self._own_context and self._rclpy.ok():
                self._rclpy.shutdown()
