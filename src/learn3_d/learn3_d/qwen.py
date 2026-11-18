"""本地千问（vLLM 的 OpenAI 兼容接口）最小客户端。

注意：该服务是 Qwen3 系列，必须显式关闭 thinking
（chat_template_kwargs.enable_thinking=False），否则 content 会是 null，
token 全被 reasoning 内容吃掉。
"""

from __future__ import annotations

import json
import re
import urllib.error
import urllib.request


class QwenClient:
    def __init__(
        self,
        base_url: str,
        model: str,
        api_key: str = "EMPTY",
        timeout: float = 30.0,
        system_prompt: str = "",
        history_turns: int = 6,
    ) -> None:
        self._url = base_url.rstrip("/") + "/chat/completions"
        self._model = model
        self._api_key = api_key
        self._timeout = timeout
        self._system_prompt = system_prompt
        self._history_turns = max(0, history_turns)
        self._turns: list[dict] = []
        self._context = ""

    def reset(self) -> None:
        self._turns.clear()

    def set_context(self, text: str) -> None:
        """更新临时上下文（如当前位置），下一条 chat 生效，不写入历史。"""
        self._context = str(text or "").strip()

    def _system_text(self) -> str:
        if not self._context:
            return self._system_prompt
        return f"{self._system_prompt}\n当前情况：{self._context}"

    def chat(self, text: str) -> str:
        """发一句话，返回模型的中文回答；失败抛 RuntimeError。"""
        messages = []
        system = self._system_text()
        if system:
            messages.append({"role": "system", "content": system})
        messages.extend(self._turns)
        messages.append({"role": "user", "content": text})

        reply = self._complete(messages)
        self._remember(text, reply)
        return reply

    def one_shot(self, prompt: str, system: str | None = None) -> str:
        """一次性请求，不写入对话历史（到达播报、意图解析等场景）。"""
        messages = []
        system_prompt = self._system_text() if system is None else system
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})
        return self._complete(messages)

    def _complete(self, messages: list[dict]) -> str:
        payload = {
            "model": self._model,
            "messages": messages,
            "temperature": 0.6,
            "max_tokens": 300,
            "chat_template_kwargs": {"enable_thinking": False},
        }
        request = urllib.request.Request(
            self._url,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self._api_key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as response:
                data = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"HTTP {exc.code}") from exc
        except Exception as exc:  # noqa: BLE001 - 网络/超时统一报错
            raise RuntimeError(str(exc)) from exc

        try:
            content = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError(f"响应格式异常：{data}") from exc
        if not content:
            raise RuntimeError("模型返回空内容")
        return _clean(content)

    def _remember(self, user_text: str, reply: str) -> None:
        self._turns.append({"role": "user", "content": user_text})
        self._turns.append({"role": "assistant", "content": reply})
        keep = self._history_turns * 2
        if keep <= 0:
            self._turns.clear()
        elif len(self._turns) > keep:
            self._turns = self._turns[-keep:]


def _clean(text: str) -> str:
    """去掉 markdown 痕迹，压成适合 TTS 的一两句话。"""
    text = re.sub(r"[*#`>]+", "", str(text))
    text = re.sub(r"\s+", " ", text).strip()
    text = re.sub(r"^(机器人|助手|小导|小静)[：:]\s*", "", text)
    return text[:200]
