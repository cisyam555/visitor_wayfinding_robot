"""机器人麦克风阵列 -> 本地语音识别（faster-whisper）。

平台只在调用 /lyre/audio_control 打开后才发布 /lyre/audio_stream。
实测 AudioFrame.sample_rate 字段不可信（写 16000，实际接近 96kHz），
所以这里按配置的实际采样率重采样，不读消息里的采样率。

识别线程：能量 VAD 断句 -> whisper 识别 -> 回调文本。
"""

from __future__ import annotations

import os
import queue
import re
import threading
import time
from collections import deque

import numpy as np
import rclpy

from lyre_msgs.msg import AudioFrame
from lyre_msgs.srv import AudioControl

# 10ms 一个分析块（16kHz）
_BLOCK = 160
# 识别结果里的常见幻觉，直接丢弃
_JUNK = ("谢谢观看", "请不吝点赞", "字幕由", "字幕提供", "明镜与点点")
# 单个分段的“不是语音”概率和平均对数概率阈值
_NO_SPEECH_PROB = 0.6
_MIN_AVG_LOGPROB = -1.2


def _is_hotword_hallucination(text: str, hotwords: str) -> bool:
    """噪声/静音时 Whisper 会把热词表整段吐出来。

    典型形态是「乐知院、核心力量跑酷、创想谷、厕所、跆拳道馆」这种
    多地点罗列，既没有动词也不是一句话；按“顿号分隔且命中多个热词”拦截。
    """
    terms = [item for item in re.split(r"[、,，\s]+", hotwords or "")
             if len(item) >= 2]
    if len(terms) < 3:
        return False
    plain = text or ""
    hits = {item for item in terms if item in plain}
    if len(hits) >= 3 and plain.count("、") >= 2:
        return True
    return len(hits) >= 5


class MicAsr:
    def __init__(
        self,
        node,
        on_text,
        *,
        mic_topic: str = "/lyre/audio_stream",
        control_service: str = "/lyre/audio_control",
        rate_in: int = 96000,
        rate_out: int = 16000,
        vad_threshold: float = 600.0,
        silence_s: float = 0.8,
        min_speech_s: float = 0.4,
        max_speech_s: float = 12.0,
        model_name: str = "small",
        compute_type: str = "int8",
        language: str = "zh",
        hotwords: str = "",
        initial_prompt: str = "",
        beam_size: int = 5,
    ) -> None:
        self._node = node
        self._log = node.get_logger()
        self._on_text = on_text
        self._rate_in = max(1, int(rate_in))
        self._rate_out = max(1, int(rate_out))
        self._threshold = float(vad_threshold)
        self._silence_blocks = max(1, int(silence_s * 100.0))
        self._min_samples = int(min_speech_s * self._rate_out)
        self._max_samples = int(max_speech_s * self._rate_out)
        self._model_name = model_name
        self._compute_type = compute_type
        self._language = language
        self._hotwords = (hotwords or "").strip()
        self._initial_prompt = (initial_prompt or "").strip()
        self._beam_size = max(1, int(beam_size))

        self._frames: queue.Queue = queue.Queue(maxsize=800)
        self._stop = threading.Event()
        self._muted = threading.Event()
        self._model = None
        self._model_failed = False
        self._stream_logged = False

        self._subscription = node.create_subscription(
            AudioFrame, mic_topic, self._on_frame, 100)
        self._client = node.create_client(AudioControl, control_service)
        self._control_service = control_service
        self._thread = threading.Thread(target=self._loop, daemon=True)

        # 提前在后台加载模型，避免第一句话等太久
        os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
        threading.Thread(target=self._load_model, daemon=True).start()

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> None:
        self._thread.start()
        threading.Thread(target=self._enable_stream, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()
        self._frames.put(None)
        self._thread.join(timeout=2.0)
        # 退出时 executor 已停，不等回调，只把关闭请求发出去
        self._set_stream(False, wait=False)

    def hold(self, muted: bool) -> None:
        """TTS 播报期间静音，避免机器人听见自己的声音。"""
        if muted:
            self._muted.set()
        else:
            self._drain()
            self._muted.clear()

    # ------------------------------------------------------------------ 音频入口
    def _on_frame(self, msg: AudioFrame) -> None:
        try:
            data = np.frombuffer(bytes(msg.data), dtype=np.int16)
            channels = int(getattr(msg, "channels", 1) or 1)
            if channels > 1 and data.size >= channels:
                # 多声道交错数据按均值下混，否则会被当成乱序单声道
                data = data.reshape(-1, channels).mean(axis=1).astype(np.int16)
            if not self._stream_logged:
                self._stream_logged = True
                self._log.info(
                    f"音频流：消息 {msg.sample_rate}Hz {channels}声道 "
                    f"{msg.bits_per_sample}bit，按 {self._rate_in}Hz 处理")
            self._frames.put_nowait(data)
        except queue.Full:
            pass

    def _drain(self) -> None:
        while True:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------------ 识别线程
    def _loop(self) -> None:
        pre_roll = deque(maxlen=max(1, int(0.3 * self._rate_out / _BLOCK)))
        speech: list[np.ndarray] = []
        speech_samples = 0
        silent_blocks = 0
        active = False

        while not self._stop.is_set():
            try:
                frame = self._frames.get(timeout=0.1)
            except queue.Empty:
                continue
            if frame is None:
                return
            if self._muted.is_set():
                pre_roll.clear()
                speech.clear()
                speech_samples = 0
                silent_blocks = 0
                active = False
                continue

            pcm = self._resample(frame)
            for start in range(0, len(pcm) - _BLOCK + 1, _BLOCK):
                block = pcm[start:start + _BLOCK]
                rms = float(np.sqrt(np.mean(block.astype(np.float32) ** 2)))
                if not active:
                    pre_roll.append(block)
                    if rms >= self._threshold:
                        active = True
                        speech = list(pre_roll)
                        speech_samples = sum(len(item) for item in speech)
                        silent_blocks = 0
                    continue

                speech.append(block)
                speech_samples += len(block)
                silent_blocks = 0 if rms >= self._threshold else silent_blocks + 1
                if (silent_blocks < self._silence_blocks
                        and speech_samples < self._max_samples):
                    continue

                audio = np.concatenate(speech)
                voiced = speech_samples - silent_blocks * _BLOCK
                pre_roll.clear()
                speech = []
                speech_samples = 0
                silent_blocks = 0
                active = False
                if voiced >= self._min_samples:
                    self._emit(audio)

    def _resample(self, pcm: np.ndarray) -> np.ndarray:
        if self._rate_in == self._rate_out:
            return pcm
        factor = self._rate_in / float(self._rate_out)
        if abs(factor - round(factor)) < 1e-6 and round(factor) > 1:
            step = int(round(factor))
            usable = len(pcm) - len(pcm) % step
            if usable <= 0:
                return pcm[:0]
            return pcm[:usable].reshape(-1, step).mean(axis=1).astype(np.int16)
        # 非整数倍：线性插值
        count = int(len(pcm) * self._rate_out / self._rate_in)
        if count <= 1:
            return pcm[:0]
        src = np.linspace(0, len(pcm) - 1, count)
        return np.interp(src, np.arange(len(pcm)), pcm).astype(np.int16)

    # -------------------------------------------------------------------- 识别
    def _load_model(self) -> None:
        # 配置的模型没下载好时退回已缓存的 small，别让麦克风识别整体失效
        names = [self._model_name]
        if self._model_name != "small":
            names.append("small")
        for name in names:
            try:
                from faster_whisper import WhisperModel
                started = time.monotonic()
                model = WhisperModel(
                    name, device="cpu", compute_type=self._compute_type)
                self._model = model
                self._log.info(
                    f"语音识别模型 {name} 已加载，"
                    f"用时 {time.monotonic() - started:.1f}s")
                return
            except Exception as exc:  # noqa: BLE001 - 模型不可用时退回小模型
                self._log.error(f"加载语音识别模型 {name} 失败：{exc}")
        self._model_failed = True

    def _emit(self, audio: np.ndarray) -> None:
        if self._model is None:
            if self._model_failed:
                return
            self._load_model()
            if self._model is None:
                return
        started = time.monotonic()
        try:
            # 能量 VAD 只能切出「有声音」的段，电机声、远处人声、喇叭尾音
            # 也会进来，Whisper 会把这些噪声编成「可以啊」「请用一下」之类的
            # 短句。再用内置 Silero VAD 过一遍，只把真正的人声交给模型。
            segments, _ = self._model.transcribe(
                audio.astype(np.float32) / 32768.0,
                language=self._language,
                beam_size=self._beam_size,
                vad_filter=True,
                vad_parameters={
                    "threshold": 0.5,
                    "min_speech_duration_ms": 200,
                    "min_silence_duration_ms": 400,
                    "speech_pad_ms": 200,
                },
                condition_on_previous_text=False,
                no_speech_threshold=_NO_SPEECH_PROB,
                log_prob_threshold=_MIN_AVG_LOGPROB,
                hotwords=self._hotwords or None,
                initial_prompt=self._initial_prompt or None,
            )
            # 分段级再过滤一次：静音/噪声段多半 no_speech_prob 很高
            kept = []
            for segment in segments:
                if getattr(segment, "no_speech_prob", 0.0) > _NO_SPEECH_PROB:
                    continue
                if getattr(segment, "avg_logprob", 0.0) < _MIN_AVG_LOGPROB:
                    continue
                kept.append(segment)
            text = "".join(segment.text for segment in kept).strip()
            conf = min((float(segment.avg_logprob) for segment in kept),
                       default=0.0)
        except Exception as exc:  # noqa: BLE001 - 单句识别失败不影响后续
            self._log.error(f"语音识别失败：{exc}")
            return
        cost = time.monotonic() - started
        if not text or any(word in text for word in _JUNK):
            return
        if _is_hotword_hallucination(text, self._hotwords):
            self._log.warn(f"丢弃热词幻觉：{text}")
            return
        self._log.info(f"麦克风识别：{text}（{cost:.1f}s，置信 {conf:.2f}）")
        self._on_text(text)

    # ---------------------------------------------------------------- 音频流开关
    def _enable_stream(self) -> None:
        self._set_stream(True)

    def _set_stream(self, enable: bool, wait: bool = True) -> None:
        if not rclpy.ok():
            return
        try:
            ready = self._client.wait_for_service(timeout_sec=5.0)
        except Exception:  # noqa: BLE001 - 关闭节点时 context 已失效
            return
        if not ready:
            self._log.warn(f"音频控制服务 {self._control_service} 未就绪")
            return
        request = AudioControl.Request()
        request.enable = enable
        request.audio_type = 1  # 原始采集音频
        future = self._client.call_async(request)
        if not wait:
            return
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3.0):
            self._log.warn(f"调用 {self._control_service}（enable={enable}）超时")
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - 开关失败只告警
            self._log.warn(f"调用音频控制服务失败：{exc}")
            return
        if response is not None and response.success:
            self._log.info(f"麦克风音频流已{'开启' if enable else '关闭'}")
        else:
            self._log.warn(f"音频流设置失败：{response}")
