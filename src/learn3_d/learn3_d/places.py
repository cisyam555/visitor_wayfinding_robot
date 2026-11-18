"""地点别名与待命点：把「带我去乐知教育」这类说法映射到底盘地图点位。

点位名直接是中文（消防地图：乐知教育、核心力量跑酷、创想谷、厕所、
跆拳道馆），中文别名全部来自 config/places.yaml。中转点只参与路线
规划，不会出现在可去地点清单里。
"""

from __future__ import annotations

import re

import yaml

_CN_DIGITS = {
    "一": "1", "二": "2", "三": "3", "四": "4", "五": "5",
    "六": "6", "七": "7", "八": "8", "九": "9", "十": "10",
}
_CN_NUMBERS = {str(value): key for key, value in _CN_DIGITS.items()}


class Place:
    def __init__(self, poi: str, spoken: str) -> None:
        self.poi = poi
        self.spoken = spoken

    def __repr__(self) -> str:
        return f"Place({self.poi!r}, {self.spoken!r})"


class PlaceBook:
    def __init__(self, poi_names, aliases, standby=None, main_line=None,
                 branches=None, transfer_points=None) -> None:
        self._known = {str(name) for name in poi_names}
        hidden = {str(name) for name in (transfer_points or [])}
        self._aliases: dict[str, str] = {}
        self._display: dict[str, str] = {}

        for poi in poi_names:
            if poi in hidden:
                continue
            number = _trailing_number(poi)
            self._display[poi] = f"{number}号点" if number else poi
            for alias in _auto_aliases(poi, number):
                self._aliases[_norm(alias)] = poi

        configured = {}
        for alias, target in (aliases or {}).items():
            # 别名可以配多个候选点位（不同地图的点位名不一样），取当前地图上有的
            targets = target if isinstance(target, (list, tuple)) else [target]
            poi = next(
                (str(item) for item in targets if str(item) in self._display),
                None,
            )
            if poi is None:
                continue
            self._aliases[_norm(alias)] = poi
            configured.setdefault(poi, str(alias))
        # 配了中文别名的点位，对外就用中文名（播报、给模型看的清单）
        self._display.update(configured)

        # 主线点位顺序：导航时按这个顺序逐点经过
        self._main_line: list[str] = []
        for name in main_line or []:
            poi = self._canonical(name)
            if poi is not None and poi not in self._main_line:
                self._main_line.append(poi)

        # 支线点位 -> 挂接的主线点位
        self._branches: dict[str, str] = {}
        for name, attach in (branches or {}).items():
            branch = self._canonical(name)
            hook = self._canonical(attach)
            if branch is not None and hook is not None and branch != hook:
                self._branches[branch] = hook

        self._standby = None
        if standby:
            self._standby = self.resolve(str(standby))

    def _canonical(self, name):
        """把路线里的名字解析成实际点位名；不在当前地图上返回 None。"""
        text = str(name)
        poi = self._aliases.get(_norm(text))
        if poi is not None:
            return poi
        return text if text in self._known else None

    @classmethod
    def load(cls, path: str, poi_names, logger) -> "PlaceBook":
        data = {}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:  # noqa: BLE001 - 配置缺失时退化成纯点位名
            logger.warn(f"读取地点配置 {path} 失败（只用点位名）：{exc}")
        book = cls(
            poi_names, data.get("aliases") or {}, data.get("standby"),
            data.get("main_line") or [], data.get("branches") or {},
            data.get("transfer_points") or [])
        logger.info(
            f"地点配置：{len(book._aliases)} 个别名，待命点="
            f"{book.standby.spoken if book.standby else '未配置'}，"
            f"主线点={len(book._main_line)} 个，支线={len(book._branches)} 个")
        return book

    @property
    def standby(self):
        return self._standby

    @property
    def main_line(self) -> list[str]:
        return list(self._main_line)

    @property
    def branches(self) -> dict[str, str]:
        return dict(self._branches)

    def describe(self) -> str:
        """给大模型看的地点清单：中文名（点位名）。"""
        return "、".join(
            f"{spoken}（{poi}）" for poi, spoken in self._display.items())

    def resolve(self, text: str):
        """从一句话里找出最长的匹配别名；找不到返回 None。"""
        normalized = _norm(text)
        best_alias = ""
        best_poi = None
        for alias, poi in self._aliases.items():
            if alias and alias in normalized and len(alias) > len(best_alias):
                best_alias, best_poi = alias, poi
        if best_poi is None:
            return None
        return Place(best_poi, self._display.get(best_poi, best_poi))


def _norm(text: str) -> str:
    text = re.sub(r"[\s，。！？、,.!?~～]", "", str(text or ""))
    return text.lower()


def _trailing_number(name: str) -> str:
    match = re.search(r"(\d+)\s*$", str(name))
    if match:
        return str(int(match.group(1)))
    for char, digit in _CN_DIGITS.items():
        if str(name).endswith(char):
            return digit
    return ""


def _auto_aliases(poi: str, number: str) -> list[str]:
    aliases = [str(poi)]
    if number:
        aliases += [
            f"{number}号点",
            f"{number}号位置",
            f"{number}号位",
            f"{number}号",
            f"点位{number}",
            f"第{number}个点",
        ]
        chinese = _CN_NUMBERS.get(number)
        if chinese:
            aliases += [
                f"{chinese}号点",
                f"{chinese}号",
                f"点位{chinese}",
                f"第{chinese}个点",
            ]
    return aliases
