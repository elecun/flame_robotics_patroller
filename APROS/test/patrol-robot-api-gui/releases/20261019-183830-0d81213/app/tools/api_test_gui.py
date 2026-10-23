"""Patrol Robot Service 공개 Python API 수동 시험 GUI(tkinter).

리모콘 없이 현장 Ubuntu에서 `PatrolRobotClient` 공개 API를 사람이 직접 눌러 시험한다.

- 시작 시 연결·제어권 획득·이동 명령을 자동 실행하지 않는다. 모든 요청은 버튼을 눌렀을 때만 보낸다.
- 서비스 시작·재시작, 설정 변경, sudo 작업은 하지 않는다.
- 모든 호출은 `*_async` API와 Future callback을 쓴다. callback은 Client thread에서 불리므로 결과를
  thread-safe queue에 넣고, GUI thread가 `root.after()` 주기로 꺼내 widget을 갱신한다.

    python tools/api_test_gui.py [--endpoint tcp://127.0.0.1:5590]

현장 사용 순서: 연결 → (상태 1초 주기 자동 갱신 확인) → 제어권 획득 → 프로그램 모드 → 장치 명령 →
STOP/비상정지 확인 → 제어권 반환 또는 수동 모드 → 종료. 요청 중인 버튼은 응답이 올 때까지 비활성화된다
(비상정지는 예외).

실제 장비에 연결된 서비스에 접속하면 버튼이 실제 장치를 움직인다.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
import tkinter as tk
from concurrent.futures import Future
from dataclasses import asdict, is_dataclass
from enum import Enum
from tkinter import messagebox, ttk
from typing import Any, Callable

from patrol_robot_client import CommandWaitTimeoutError, PatrolRobotClient, UnknownCommandError
from patrol_robot_common import CommandReceipt, CommandResult, CommandState, ControlMode, EmergencyStopResult
from patrol_robot_common.protocol import DEFAULT_ENDPOINT

PUMP_MS = 50                 # GUI thread가 결과 queue를 비우는 주기
DEFAULT_POLL_SEC = 1.0       # 상태 주기 조회 기본 간격
MIN_POLL_SEC = 0.2
DEFAULT_WAIT_SEC = 60.0      # 완료 대기 기본 제한시간
PTU_SPEED_RANGE = (0x00, 0x3F)
ERECTOR_SPEED_RANGE = (0, 255)


# ================================================================ 입력 검증


class InputError(ValueError):
    """GUI 입력값 오류. 메시지는 사용자에게 그대로 보여준다."""


def parse_float(label: str, text: str) -> float:
    """유한한 실수. 빈 값·NaN·inf는 거부한다."""
    text = text.strip()
    if not text:
        raise InputError(f"{label}: 값을 입력하세요")
    try:
        value = float(text)
    except ValueError:
        raise InputError(f"{label}: 숫자가 아닙니다 ({text!r})") from None
    if not math.isfinite(value):
        raise InputError(f"{label}: 유한한 숫자여야 합니다 ({text!r})")
    return value


def parse_positive_float(label: str, text: str) -> float:
    value = parse_float(label, text)
    if value <= 0:
        raise InputError(f"{label}: 0보다 커야 합니다 ({text!r})")
    return value


def parse_int_code(label: str, text: str, lo: int, hi: int) -> int:
    """정수 코드. 10진수 또는 0x 16진수를 받고 [lo, hi] 범위를 확인한다."""
    text = text.strip()
    if not text:
        raise InputError(f"{label}: 값을 입력하세요")
    try:
        value = int(text, 0)
    except ValueError:
        raise InputError(f"{label}: 정수가 아닙니다 ({text!r}, 10진수 또는 0x 16진수)") from None
    if not lo <= value <= hi:
        raise InputError(f"{label}: {lo}~{hi}(0x{lo:02X}~0x{hi:02X}) 범위여야 합니다 ({text!r})")
    return value


def parse_command_id(text: str) -> str:
    text = text.strip()
    if not text:
        raise InputError("command_id: 값을 선택하거나 입력하세요")
    return text


# ================================================================ 결과 표시


def to_plain(value: Any) -> Any:
    """dataclass·Enum을 JSON으로 표시할 수 있는 값으로 바꾼다."""
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value) and not isinstance(value, type):
        return to_plain(asdict(value))
    if isinstance(value, dict):
        return {str(k): to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(v) for v in value]
    return value


def _enum_text(value: Any) -> str:
    return "-" if value is None else str(value.value if isinstance(value, Enum) else value)


def describe_result(value: Any) -> str:
    """API 결과 한 줄 요약."""
    if isinstance(value, CommandReceipt):
        if value.accepted:
            text = f"접수 command_id={value.command_id}"
        else:
            text = f"거부 reason={_enum_text(value.reason)}"
        return text + (f" message={value.message!r}" if value.message else "")
    if isinstance(value, CommandResult):
        text = f"state={_enum_text(value.state)} command_id={value.command_id}"
        if value.reason is not None:
            text += f" reason={_enum_text(value.reason)}"
        return text + (f" message={value.message!r}" if value.message else "")
    if isinstance(value, EmergencyStopResult):
        # latched는 새 동작 차단, tx_result는 로컬 송신 결과일 뿐이다. 실제 정지는 상태의 motion으로 확인한다.
        parts = [f"latched={value.latched}(실제 정지 확인 아님)"]
        for name in ("ptu", "mast", "erector"):
            a = getattr(value, name)
            part = f"{name}: tx={_enum_text(a.tx_result)}"
            if a.reason is not None:
                part += f" reason={_enum_text(a.reason)}"
            if a.error_code:
                part += f" error={a.error_code}:{a.error_message or ''}"
            parts.append(part)
        return " | ".join(parts)
    if value is None:
        return "None"
    return json.dumps(to_plain(value), ensure_ascii=False, sort_keys=True)


def describe_error(exc: BaseException) -> str:
    text = f"{type(exc).__name__}: {exc}"
    if isinstance(exc, CommandWaitTimeoutError) and exc.last_result is not None:
        text += f" (마지막 상태: {describe_result(exc.last_result)})"
    return text


# ================================================================ 상태 대시보드(공개 상태 schema 1.x)

# ok=정상/준비(녹), warn=경고·확인 불가·오래됨(주황), bad=OFFLINE·고장·비상정지(적), none=값 없음·비활성(회), plain=중립
LEVEL_COLORS = {"ok": "#1a7f37", "warn": "#c25e00", "bad": "#c00000", "none": "#888888", "plain": "#000000"}
_LEVEL_RANK = ["none", "plain", "ok", "warn", "bad"]

SUPPORTED_SCHEMA_MAJOR = 1   # 이 GUI가 해석하는 공개 상태 schema major(고객 계약 03 §10)

# 논리 통신 경로(03 §4). mast_bus는 schema 1.1.0에서 추가된 마스트 전용 NPort Serial2 TCP 경로다.
LINK_IDS = ("digital_counter_bus", "actuator_bus", "bmu", "nport", "mast_bus")
LINK_LABELS = {"mast_bus": "mast_bus(마스트 전용)"}

READINESS_CAPS = {   # 장치별 readiness 항목 표시 순서(03 §8)
    "ptu": ("motion", "speed", "stop"),
    "mast": ("manual_motion", "target_motion", "stop"),
    "erector": ("motion", "speed", "stop"),
}

DASHBOARD_LAYOUT: list[tuple[str, list[tuple[str, str]]]] = [
    ("시스템", [("system.schema", "schema"), ("system.control_mode", "mode"), ("system.control_state", "state"),
              ("system.program_control", "program_ctl"), ("system.manual_authorization", "manual_auth"),
              ("system.health", "health"), ("system.emergency_stop_latched", "estop_latched"),
              ("active_commands", "진행 작업")]),
    ("링크·리모콘", [*((f"link.{k}", LINK_LABELS.get(k, k)) for k in LINK_IDS),
                 ("remote.nport_link", "nport_link"), ("remote.activity", "remote activity")]),
    ("PTU", [("ptu.response", "응답"), ("ptu.pan_deg", "pan_deg"), ("ptu.tilt_deg", "tilt_deg"),
             ("ptu.at_home", "at_home"), ("ptu.motion", "motion"), ("ptu.speed_code", "speed_code"),
             ("ptu.readiness", "준비")]),
    ("Mast", [("mast.response", "응답"), ("mast.link", "경로"), ("mast.height_mm", "height_mm"), ("mast.at_home", "at_home"),
              ("mast.motion", "motion"), ("mast.readiness", "준비")]),
    ("Erector", [("erector.response", "응답"), ("erector.position_mm", "position_mm"),
                 ("erector.at_home", "at_home"), ("erector.at_erect", "at_erect"),
                 ("erector.calibration_valid", "calibration"), ("erector.soft_limit_reached", "soft_limit"),
                 ("erector.motion", "motion"), ("erector.faults", "faults"), ("erector.readiness", "준비")]),
    ("Digital Counter", [("digital_counter.response", "응답"), ("digital_counter.height_mm", "height_mm")]),
    ("BMU", [("bmu.response", "응답"), ("bmu.voltage_v", "전압"), ("bmu.current_a", "전류"),
             ("bmu.soc_percent", "SOC"), ("bmu.soh_percent", "SOH"), ("bmu.temperature_c", "온도"),
             ("bmu.faults", "faults")]),
]

DASHBOARD_COLUMN = {"링크·리모콘": 1, "Mast": 1, "Digital Counter": 1, "BMU": 1}   # 나머지는 0열

# 알려진 열거값 → 수준. 표에 없는 값은 원문 + "알 수 없는 값"(warn)으로 표시하고 정상으로 단정하지 않는다.
_SYSTEM_ENUMS = {
    "control_mode": {"PROGRAM": "plain", "MANUAL": "plain"},
    "control_state": {"PROGRAM": "plain", "MANUAL_LOCKED": "plain", "MANUAL_ARMING": "plain",
                      "MANUAL_READY": "plain", "EMERGENCY_STOPPED": "bad"},
    "program_control": {"ACQUIRED": "plain", "NONE": "plain"},
    "manual_authorization": {"LOCKED": "plain", "ARMING": "plain", "READY": "plain"},
    "health": {"NORMAL": "ok", "DEGRADED": "warn", "EMERGENCY_STOPPED": "bad"},
}
_LINK_STATES = {"READY": "ok", "CONNECTING": "warn", "INITIALIZING": "warn", "DISCONNECTED": "bad"}
_NPORT_LINK = {"CONNECTED": "ok", "DISCONNECTED": "bad"}
_ACTIVITY = {"RECENT": "ok", "STALE": "plain", "UNKNOWN": "warn"}
_RESPONSE_STATES = {"ONLINE": ("ok", ""), "OFFLINE": ("bad", ""), "UNKNOWN": ("warn", ""),
                    "INITIALIZING": ("warn", ""), "UNSUPPORTED": ("plain", " (응답 기능 없음)"),
                    "DISABLED": ("none", " (비활성)")}
_MOTION = {"MOVING": "plain", "STOPPED": "plain", "INITIALIZING": "warn"}
_FAULT_CODES = {
    "erector": {"stall_or_direction_fault"},
    "bmu": {"over_voltage", "under_voltage", "charge_over_current", "discharge_over_current",
            "over_temperature", "under_temperature", "bmu_error", "fan_error"},
}


def _get(data: Any, *path: str) -> Any:
    for key in path:
        if not isinstance(data, dict):
            return None
        data = data.get(key)
    return data


def parse_schema_version(value: Any) -> tuple[int, int, int] | None:
    """'major.minor.patch' 문자열 → 정수 3개. 형식이 다르면 None(문자열 비교를 하지 않는다)."""
    if not isinstance(value, str):
        return None
    parts = value.split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        return None
    return int(parts[0]), int(parts[1]), int(parts[2])


def _enum(value: Any, known: dict[str, str]) -> tuple[str, str]:
    if value is None:
        return "-", "none"
    if isinstance(value, str) and value in known:
        return value, known[value]
    return f"{value} (알 수 없는 값)", "warn"


def _num(value: Any, unit: str | None = "") -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return str(value)
    text = f"{value:.2f}".rstrip("0").rstrip(".") if isinstance(value, float) else str(value)
    return f"{text} {unit or ''}".rstrip()


def _value_text(kind: str, value: Any, unit: Any) -> tuple[str, str]:
    """valid && fresh인 관측값의 표시."""
    if kind == "measure":
        return _num(value, unit), "ok"
    if kind == "state":
        return ("예" if value else "아니오") if isinstance(value, bool) else str(value), "plain"
    if kind == "motion":
        return _enum(value, _MOTION)
    return str(value), "plain"


def observation(obs: Any, kind: str = "measure", fault_codes: set[str] | None = None) -> tuple[str, str]:
    """Observation(03 §6.1) → (표시, 수준). 값 사용은 supported·valid·fresh가 모두 참일 때만이다."""
    if not isinstance(obs, dict):
        return "-", "none"
    supported = obs.get("supported")
    if supported is False:
        return "미지원", "none"
    valid, fresh = obs.get("valid") is True, obs.get("fresh") is True
    error = obs.get("error")
    err = f": {error}" if error else ""
    if supported is not True:
        return f"확인 불가(지원 미확인){err}", "warn"
    value = obs.get("value")
    if kind == "faults":
        if not (valid and fresh):
            return ("오래된 값: " + ", ".join(map(str, value)) if valid and isinstance(value, list) and value
                    else f"확인 불가{err}"), "warn"
        if not isinstance(value, list):
            return f"{value} (알 수 없는 형식)", "warn"
        if not value:
            return "없음", "ok"   # 빈 목록은 supported·valid·fresh일 때만 '활성 고장 없음'(03 §6.2)
        known = fault_codes or set()
        return ", ".join(c if c in known else f"{c}(알 수 없는 코드)" for c in map(str, value)), "bad"
    if valid and fresh:
        return _value_text(kind, value, obs.get("unit"))
    if valid:   # 마지막 유효값이 오래됨: 표시만 하고 제어 판단에 쓰지 않는다
        return f"{_value_text(kind, value, obs.get('unit'))[0]} (오래된 값)", "warn"
    if fresh:   # 장치가 최근 응답했지만 값이 무효
        return f"값 없음({error or '무효'})", "warn"
    return f"확인 불가{err}", "warn"


def _short_missing(items: Any) -> str:
    if not isinstance(items, list):
        return ""
    return ", ".join(str(m).split(".")[-1] for m in items)


def _response(dev: Any) -> tuple[str, str]:
    """response.state + configuration 요약."""
    if not isinstance(dev, dict):
        return "-", "none"
    state = _get(dev, "response", "state")
    if isinstance(state, str) and state in _RESPONSE_STATES:
        level, note = _RESPONSE_STATES[state]
        text = state + note
    else:
        text, level = _enum(state, {})
    cfg = dev.get("configuration") if isinstance(dev.get("configuration"), dict) else {}
    missing = _short_missing(cfg.get("missing"))
    if cfg.get("complete") is False:
        text += f" · 설정 누락: {missing or '?'}"
        level = max(level, "warn", key=_LEVEL_RANK.index)
    elif missing:
        text += f" · 선택 설정 누락: {missing}"
        level = max(level, "warn", key=_LEVEL_RANK.index)
    return text, level


def _readiness(dev: Any, caps: tuple[str, ...]) -> tuple[str, str]:
    ready = _get(dev, "readiness")
    if not isinstance(ready, dict) or not ready:
        return "-", "none"
    # 같은 결과의 항목은 묶어 한 번만 표시한다(예: motion·speed·stop ✗(emergency_stop_active)).
    groups: dict[str, list[str]] = {}
    level = "ok"
    for cap in caps:
        r = ready.get(cap)
        if not isinstance(r, dict):
            mark = "-"
            level = max(level, "warn", key=_LEVEL_RANK.index)
        elif r.get("supported") is False:
            mark = "미지원"
        elif r.get("available") is True:
            mark = "✓"
        else:
            reasons = [str(x) for x in (r.get("reasons") or [])]
            mark = f"✗({', '.join(reasons) or '사유 없음'})"
            bad = any("emergency" in x or "offline" in x for x in reasons)
            level = max(level, "bad" if bad else "warn", key=_LEVEL_RANK.index)
        groups.setdefault(mark, []).append(cap)
    return " · ".join(f"{'·'.join(names)} {mark}" for mark, names in groups.items()), level


def _device_link(dev: Any, links: dict) -> tuple[str, str]:
    """장치의 link 값으로 links 상태를 찾는다(03 §4, 경로 이름을 고정값으로 가정하지 않음).

    READY는 경로가 열려 있다는 뜻일 뿐 장치 응답은 아니다. 장치 응답은 '응답'(response.state)으로 본다.
    """
    link_id = _get(dev, "link")
    if not isinstance(link_id, str):
        return "-", "none"
    if link_id not in links:
        return f"{link_id}: 비활성(설정 꺼짐)", "none"
    state, level = _enum(_get(links, link_id, "state"), _LINK_STATES)
    return f"{link_id}: {state}", level


def _active_commands(value: Any) -> tuple[str, str]:
    if not isinstance(value, list):
        return "-", "none"
    if not value:
        return "없음", "plain"
    items = []
    for c in value:
        if isinstance(c, dict):
            phase = c.get("phase")
            items.append(f"{c.get('action', '?')}" + (f"({phase})" if phase else ""))
    return ", ".join(items) or "?", "plain"


def dashboard_values(status: Any) -> dict[str, tuple[str, str]]:
    """공개 상태 dict → {필드 key: (표시, 수준)}. 모르는 키는 무시하고, 형식이 달라도 예외를 내지 않는다.

    schema_version major가 지원 값과 다르면 상태를 해석하지 않고 버전 불일치만 표시한다(03 §10).
    """
    keys = [k for _, fields in DASHBOARD_LAYOUT for k, _ in fields]
    out: dict[str, tuple[str, str]] = {k: ("-", "none") for k in keys}
    s = status if isinstance(status, dict) else {}
    raw_version = s.get("schema_version")
    version = parse_schema_version(raw_version)
    if version is None or version[0] != SUPPORTED_SCHEMA_MAJOR:
        shown = "없음" if raw_version is None else str(raw_version)
        out["system.schema"] = (f"{shown} 해석 안 함(지원 major {SUPPORTED_SCHEMA_MAJOR})", "bad")
        return out
    out["system.schema"] = (str(raw_version), "ok")

    system = s.get("system") if isinstance(s.get("system"), dict) else {}
    for key, known in _SYSTEM_ENUMS.items():
        out[f"system.{key}"] = _enum(system.get(key), known)
    latched = system.get("emergency_stop_latched")
    out["system.emergency_stop_latched"] = (
        ("비상정지 중", "bad") if latched is True else ("해제", "ok") if latched is False else ("-", "none"))
    out["active_commands"] = _active_commands(s.get("active_commands"))

    links = s.get("links") if isinstance(s.get("links"), dict) else {}
    for link_id in LINK_IDS:
        if link_id in links:
            out[f"link.{link_id}"] = _enum(_get(links, link_id, "state"), _LINK_STATES)
        else:
            out[f"link.{link_id}"] = ("비활성(설정 꺼짐)", "none")   # 활성 경로만 키가 있다(03 §4)
    remote = s.get("remote") if isinstance(s.get("remote"), dict) else {}
    out["remote.nport_link"] = _enum(remote.get("nport_link"), _NPORT_LINK)
    activity, level = _enum(remote.get("activity"), _ACTIVITY)
    out["remote.activity"] = (activity + (" (최근 입력 없음)" if activity == "STALE" else ""), level)

    devices = s.get("devices") if isinstance(s.get("devices"), dict) else {}
    for role in ("ptu", "mast", "erector", "digital_counter", "bmu"):
        dev = devices.get(role)
        if not isinstance(dev, dict):
            continue
        out[f"{role}.response"] = _response(dev)
        disabled = _get(dev, "configuration", "enabled") is False
        role_keys = [k for k in keys if k.startswith(f"{role}.") and k != f"{role}.response"]
        if disabled:
            for k in role_keys:
                out[k] = ("비활성", "none")
            continue
        for k in role_keys:
            name = k.split(".", 1)[1]
            if name == "readiness":   # 서비스가 경로·카운터 조건을 조합한 값을 그대로 표시한다
                out[k] = _readiness(dev, READINESS_CAPS.get(role, ()))
            elif name == "link":
                out[k] = _device_link(dev, links)
            elif name == "motion":
                out[k] = observation(dev.get("motion"), "motion")
            elif name == "faults":
                out[k] = observation(dev.get("faults"), "faults", _FAULT_CODES.get(role))
            elif name == "speed_code":
                code = _get(dev, "details", "speed_code")
                out[k] = ("-", "none") if not isinstance(code, int) or isinstance(code, bool) else (
                    f"{code} (0x{code:02X}, 설정값)", "plain")
            elif name in (_get(dev, "states") or {}):
                out[k] = observation(_get(dev, "states", name), "state")
            else:
                out[k] = observation(_get(dev, "measurements", name), "measure")
    # 상태 의미에 따른 강조: 보정 무효·소프트 리밋 도달은 경고
    if out["erector.calibration_valid"][0] == "아니오":
        out["erector.calibration_valid"] = ("아니오(보정 무효)", "warn")
    if out["erector.soft_limit_reached"][0] == "예":
        out["erector.soft_limit_reached"] = ("예(소프트 리밋)", "warn")
    return out


# ================================================================ 명령 완료 의미(02 §4.2)

# 단순 명령의 completed는 서비스 처리 완료일 뿐 실제 이동·정지 완료가 아니다.
SIMPLE_ACTIONS = frozenset({
    "ptu.up", "ptu.down", "ptu.left", "ptu.right", "ptu.stop", "ptu.set_speed",
    "mast.extend", "mast.retract", "mast.stop",
    "erector.fold", "erector.unfold", "erector.stop", "erector.set_speed",
})
# 목표·Home·Erect/Stow·RESET의 completed는 정지 후 도착(허용오차 안) 확인이다.
OBSERVED_ACTIONS = frozenset({
    "ptu.home", "ptu.move_to_angle", "mast.home", "mast.move_to_height",
    "erector.erect", "erector.stow", "erector.move_to", "erector.move_by", "reset",
})


def completion_note(action: str | None, result: Any) -> str | None:
    """CommandResult가 completed일 때 그 의미 설명. 다른 상태면 None."""
    if not isinstance(result, CommandResult) or result.state is not CommandState.COMPLETED:
        return None
    if action in SIMPLE_ACTIONS:
        return "서비스 처리 완료 — 실제 장치 이동·정지 확인 아님(상태의 motion·위치로 확인)"
    if action in OBSERVED_ACTIONS:
        return "도착·완료 확인(정지 후 목표 허용오차 안)"
    return "명령 종류 미확인 — 단순 명령이면 서비스 처리 완료일 뿐 실제 동작 확인 아님"


# ================================================================ GUI


class ApiTestGui:
    """시험 GUI. `client_factory(endpoint)`는 연결된 Client를 반환한다(시험에서 mock 주입)."""

    def __init__(self, root: tk.Tk, *, endpoint: str = DEFAULT_ENDPOINT,
                 client_factory: Callable[[str], Any] | None = None) -> None:
        self.root = root
        self.client: Any = None
        self._client_factory = client_factory or (lambda ep: PatrolRobotClient(ep))
        self._events: queue.Queue = queue.Queue()   # Client thread → GUI thread
        self._connecting = False
        self._closed = False
        self._poll_job: str | None = None
        self._status_in_flight = False
        self._command_ids: list[str] = []
        self._command_actions: dict[str, str] = {}   # command_id → 접수한 명령(예: ptu.up, reset)
        self._buttons: dict[str, list[Any]] = {}   # 요청 key → 버튼(요청 중 비활성화)
        self._inflight: set[str] = set()

        root.title("Patrol Robot API 시험")
        root.protocol("WM_DELETE_WINDOW", self.on_window_close)

        self.endpoint_var = tk.StringVar(value=endpoint)
        self.conn_var = tk.StringVar(value="연결 안 됨")
        self.summary_var = tk.StringVar(value="-")
        self.poll_var = tk.BooleanVar(value=True)   # 연결되면 고정 주기로 상태 갱신
        self.poll_sec_var = tk.StringVar(value=str(DEFAULT_POLL_SEC))
        self.ptu_pan_var = tk.StringVar(value="0")
        self.ptu_tilt_var = tk.StringVar(value="0")
        self.ptu_speed_var = tk.StringVar(value="0x20")
        self.mast_height_var = tk.StringVar(value="")
        self.erector_pos_var = tk.StringVar(value="")
        self.erector_delta_var = tk.StringVar(value="")
        self.erector_speed_var = tk.StringVar(value="")
        self.command_id_var = tk.StringVar(value="")
        self.wait_sec_var = tk.StringVar(value=str(DEFAULT_WAIT_SEC))
        self.auto_wait_var = tk.BooleanVar(value=False)

        self._build()
        self.root.after(PUMP_MS, self._pump)

    # ---------------------------------------------------------------- 화면 구성

    def _build(self) -> None:
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)

        # 1행: 연결
        conn = ttk.Frame(root, padding=4)
        conn.grid(row=0, column=0, sticky="ew")
        ttk.Label(conn, text="Endpoint").pack(side="left")
        ttk.Entry(conn, textvariable=self.endpoint_var, width=28).pack(side="left", padx=4)
        self.connect_btn = ttk.Button(conn, text="연결", command=self.connect)
        self.connect_btn.pack(side="left")
        self.disconnect_btn = ttk.Button(conn, text="종료", command=self.disconnect)
        self.disconnect_btn.pack(side="left", padx=4)
        ttk.Label(conn, textvariable=self.conn_var).pack(side="left", padx=8)

        # 2행: 안전 — 항상 보이는 위치에 비상정지와 장치별 STOP
        safety = tk.Frame(root, padx=4, pady=4, bg="#5a1010")
        safety.grid(row=1, column=0, sticky="ew")
        big = {"font": ("TkDefaultFont", 12, "bold"), "padx": 10, "pady": 6}
        tk.Button(safety, text="비상정지", bg="#d00000", fg="white", activebackground="#ff2020",
                  command=self.emergency_stop, **big).pack(side="left", padx=4)
        self._reg("release_estop", tk.Button(safety, text="비상정지 해제", command=self.release_emergency_stop,
                                             **big)).pack(side="left", padx=4)
        for key, text, cmd in (("ptu.stop", "PTU STOP", lambda: self.device_cmd("ptu", "stop")),
                               ("mast.stop", "Mast STOP", lambda: self.device_cmd("mast", "stop")),
                               ("erector.stop", "Erector STOP", lambda: self.device_cmd("erector", "stop")),
                               (None, "3장치 STOP", self.stop_all)):
            self._reg(key, tk.Button(safety, text=text, bg="#ffcc00", command=cmd, **big)).pack(side="left", padx=4)

        # 3행: 제어권·모드·RESET
        ctl = ttk.Frame(root, padding=4)
        ctl.grid(row=2, column=0, sticky="ew")
        for key, text, cmd in (("acquire", "제어권 획득", self.acquire_control),
                               ("release", "제어권 반환", self.release_control),
                               ("mode.PROGRAM", "프로그램 모드", lambda: self.set_mode(ControlMode.PROGRAM)),
                               ("mode.MANUAL", "수동 모드", lambda: self.set_mode(ControlMode.MANUAL)),
                               ("reset", "전체 RESET", self.reset)):
            self._reg(key, ttk.Button(ctl, text=text, command=cmd)).pack(side="left", padx=2)

        # 4행: 좌(장치·명령) / 우(상태·로그)
        body = ttk.PanedWindow(root, orient="horizontal")
        body.grid(row=3, column=0, sticky="nsew", padx=4, pady=4)
        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=0)
        body.add(right, weight=1)

        self._build_ptu(left)
        self._build_mast(left)
        self._build_erector(left)
        self._build_command(left)
        self._build_log(left)
        self._build_status(right)

    def _entry_row(self, parent, row: int, label: str, var: tk.StringVar, button: str,
                   command: Callable[[], None], col: int = 0, key: str | None = None) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="e")
        ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=col + 1, padx=2, pady=1)
        self._reg(key, ttk.Button(parent, text=button, command=command)).grid(row=row, column=col + 2, sticky="w")

    def _build_ptu(self, parent) -> None:
        f = ttk.LabelFrame(parent, text="PTU", padding=4)
        f.pack(fill="x", pady=2)
        pad = ttk.Frame(f)
        pad.grid(row=0, column=0, rowspan=3, padx=4)
        self._reg("ptu.up", ttk.Button(pad, text="▲ 상", width=6,
                                        command=lambda: self.device_cmd("ptu", "up"))).grid(row=0, column=1)
        self._reg("ptu.left", ttk.Button(pad, text="◀ 좌", width=6,
                                        command=lambda: self.device_cmd("ptu", "left"))).grid(row=1, column=0)
        self._reg("ptu.stop", ttk.Button(pad, text="STOP", width=6,
                                        command=lambda: self.device_cmd("ptu", "stop"))).grid(row=1, column=1)
        self._reg("ptu.right", ttk.Button(pad, text="우 ▶", width=6,
                                        command=lambda: self.device_cmd("ptu", "right"))).grid(row=1, column=2)
        self._reg("ptu.down", ttk.Button(pad, text="▼ 하", width=6,
                                        command=lambda: self.device_cmd("ptu", "down"))).grid(row=2, column=1)
        self._reg("ptu.home", ttk.Button(f, text="Home", command=lambda: self.device_cmd("ptu", "home"))).grid(row=0, column=1, columnspan=2, sticky="w")
        ttk.Label(f, text="pan_deg").grid(row=1, column=1, sticky="e")
        ttk.Entry(f, textvariable=self.ptu_pan_var, width=8).grid(row=1, column=2, padx=2)
        ttk.Label(f, text="tilt_deg").grid(row=2, column=1, sticky="e")
        ttk.Entry(f, textvariable=self.ptu_tilt_var, width=8).grid(row=2, column=2, padx=2)
        self._reg("ptu.move_to_angle", ttk.Button(f, text="절대 각도 이동", command=self.ptu_move_to_angle)).grid(
            row=1, column=3, rowspan=2, sticky="w")
        self._entry_row(f, 3, "speed(0x00~0x3F)", self.ptu_speed_var, "속도 설정", self.ptu_set_speed, col=1,
                        key="ptu.set_speed")

    def _build_mast(self, parent) -> None:
        f = ttk.LabelFrame(parent, text="Mast", padding=4)
        f.pack(fill="x", pady=2)
        for i, (text, action) in enumerate((("Extend", "extend"), ("Retract", "retract"),
                                            ("STOP", "stop"), ("Home", "home"))):
            self._reg(f"mast.{action}", ttk.Button(f, text=text, command=lambda a=action: self.device_cmd("mast", a))
                      ).grid(row=0, column=i, padx=2)
        self._entry_row(f, 1, "height_mm", self.mast_height_var, "목표 높이 이동", self.mast_move_to_height,
                        key="mast.move_to_height")

    def _build_erector(self, parent) -> None:
        f = ttk.LabelFrame(parent, text="Erector", padding=4)
        f.pack(fill="x", pady=2)
        for i, (text, action) in enumerate((("Fold", "fold"), ("Unfold", "unfold"), ("Erect", "erect"),
                                            ("Stow", "stow"), ("STOP", "stop"))):
            self._reg(f"erector.{action}", ttk.Button(f, text=text, command=lambda a=action: self.device_cmd("erector", a))
                      ).grid(row=0, column=i, padx=2)
        g = ttk.Frame(f)
        g.grid(row=1, column=0, columnspan=5, sticky="w")
        self._entry_row(g, 0, "position_mm", self.erector_pos_var, "절대 위치 이동", self.erector_move_to, key="erector.move_to")
        self._entry_row(g, 1, "delta_mm", self.erector_delta_var, "상대 위치 이동", self.erector_move_by, key="erector.move_by")
        self._entry_row(g, 2, "speed_code(0~255)", self.erector_speed_var, "속도 설정", self.erector_set_speed, key="erector.set_speed")

    def _build_command(self, parent) -> None:
        f = ttk.LabelFrame(parent, text="명령 조회·완료 대기", padding=4)
        f.pack(fill="x", pady=2)
        ttk.Label(f, text="command_id").grid(row=0, column=0, sticky="e")
        self.command_combo = ttk.Combobox(f, textvariable=self.command_id_var, width=34, values=[])
        self.command_combo.grid(row=0, column=1, columnspan=3, sticky="w", padx=2)
        ttk.Button(f, text="상태 조회", command=self.get_command).grid(row=1, column=0, padx=2, pady=2)
        ttk.Label(f, text="대기(초)").grid(row=1, column=1, sticky="e")
        ttk.Entry(f, textvariable=self.wait_sec_var, width=6).grid(row=1, column=2, sticky="w")
        ttk.Button(f, text="완료 대기", command=self.wait_command).grid(row=1, column=3, sticky="w")
        ttk.Checkbutton(f, text="접수되면 자동으로 완료 대기", variable=self.auto_wait_var).grid(
            row=2, column=0, columnspan=4, sticky="w")

    def _build_status(self, parent) -> None:
        parent.rowconfigure(3, weight=1)
        parent.columnconfigure(0, weight=1)

        bar = ttk.Frame(parent)
        bar.grid(row=0, column=0, sticky="ew")
        ttk.Button(bar, text="상태 새로고침", command=self.refresh_status).pack(side="left")
        ttk.Checkbutton(bar, text="주기 조회", variable=self.poll_var,
                        command=self.on_poll_toggle).pack(side="left", padx=4)
        ttk.Label(bar, text="간격(초)").pack(side="left")
        ttk.Entry(bar, textvariable=self.poll_sec_var, width=5).pack(side="left")

        self._build_dashboard(parent).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 0))

        # 원문 JSON: 기본은 접고 버튼으로 펼친다.
        rbar = ttk.Frame(parent)
        rbar.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        self.raw_btn = ttk.Button(rbar, text="원문 JSON 보기", command=self.toggle_raw)
        self.raw_btn.pack(side="left")
        self.status_text = tk.Text(parent, height=14, wrap="none", font=("TkFixedFont", 9))
        self.status_text.configure(state="disabled")
        self.raw_visible = False

    def _build_log(self, parent) -> None:
        f = ttk.Frame(parent)
        f.pack(fill="both", expand=True, pady=(4, 0))
        f.rowconfigure(1, weight=1)
        f.columnconfigure(0, weight=1)
        lbar = ttk.Frame(f)
        lbar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(lbar, text="로그(시간순)").pack(side="left")
        ttk.Button(lbar, text="로그 지우기", command=self.clear_log).pack(side="right")

        self.log_text = tk.Text(f, height=8, width=60, wrap="word", font=("TkFixedFont", 9))
        self.log_text.grid(row=1, column=0, sticky="nsew")
        self.log_text.tag_configure("error", foreground="#c00000")
        self.log_text.tag_configure("reject", foreground="#b05000")
        self.log_text.tag_configure("ok", foreground="#006000")
        self.log_text.configure(state="disabled")
        scroll = ttk.Scrollbar(f, command=self.log_text.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)

    def _build_dashboard(self, parent) -> Any:
        """고정 필드 표. 라벨은 한 번 만들고 show_status에서 값·색만 바꾼다."""
        # 작은 화면(약 1000px 폭)에서도 잘리지 않게 두 열로 쌓는다.
        frame = ttk.Frame(parent)
        cols = [ttk.Frame(frame), ttk.Frame(frame)]
        for c, col in enumerate(cols):
            col.grid(row=0, column=c, sticky="nsew")
            frame.columnconfigure(c, weight=1)
        self.dash_labels: dict[str, tk.Label] = {}
        for title, fields in DASHBOARD_LAYOUT:
            box = ttk.LabelFrame(cols[DASHBOARD_COLUMN.get(title, 0)], text=title, padding=(2, 0))
            box.pack(fill="x", padx=2, pady=1)
            for r, (key, label) in enumerate(fields):
                ttk.Label(box, text=label).grid(row=r, column=0, sticky="nw")
                value = tk.Label(box, text="-", anchor="w", justify="left", wraplength=150,
                                 fg=LEVEL_COLORS["none"])
                value.grid(row=r, column=1, sticky="w", padx=(3, 0))
                self.dash_labels[key] = value
        return frame

    def toggle_raw(self) -> None:
        self.raw_visible = not self.raw_visible
        if self.raw_visible:
            self.status_text.grid(row=3, column=0, sticky="nsew")
            self.raw_btn.configure(text="원문 JSON 접기")
        else:
            self.status_text.grid_remove()
            self.raw_btn.configure(text="원문 JSON 보기")

    # ---------------------------------------------------------------- 로그·스레드 경계

    def log(self, text: str, tag: str = "") -> None:
        """GUI thread 전용."""
        stamp = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{stamp} {text}\n", tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def log_lines(self) -> list[str]:
        return self.log_text.get("1.0", "end").splitlines()

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def post(self, fn: Callable[[], None]) -> None:
        """아무 thread에서나 호출. GUI thread에서 fn을 실행하도록 넘긴다."""
        self._events.put(fn)

    def _pump(self) -> None:
        if self._closed:
            return
        self.process_events()
        self.root.after(PUMP_MS, self._pump)

    def process_events(self) -> None:
        while True:
            try:
                fn = self._events.get_nowait()
            except queue.Empty:
                return
            try:
                fn()
            except Exception as exc:  # GUI 처리 오류로 창이 닫히지 않게 한다
                self.log(f"GUI 처리 오류 {describe_error(exc)}", "error")

    def _watch(self, label: str, future: Future, on_result: Callable[[Any], None] | None = None,
               key: str | None = None, note: Callable[[Any], str | None] | None = None) -> None:
        """Future 완료를 GUI thread로 전달해 로그에 남기고 요청 중 표시를 푼다."""
        def done(f: Future) -> None:   # Client thread
            try:
                value = f.result()
            except BaseException as exc:
                self.post(lambda exc=exc: (self._end_request(key),
                                           self.log(f"{label} 예외 {describe_error(exc)}", "error")))
                return
            self.post(lambda: (self._end_request(key), self._on_success(label, value, on_result, note)))
        future.add_done_callback(done)

    def _reg(self, key: str | None, button: Any) -> Any:
        if key is not None:
            self._buttons.setdefault(key, []).append(button)
        return button

    def _set_buttons(self, key: str, state: str) -> None:
        for button in self._buttons.get(key, ()):
            try:
                button.configure(state=state)
            except tk.TclError:
                pass

    def _end_request(self, key: str | None) -> None:
        if key is not None and key in self._inflight:
            self._inflight.discard(key)
            self._set_buttons(key, "normal")

    def _on_success(self, label: str, value: Any, on_result, note=None) -> None:
        tag = "ok"
        if isinstance(value, CommandReceipt):
            tag = "ok" if value.accepted else "reject"
        elif isinstance(value, dict) and (value.get("granted") is False or value.get("accepted") is False):
            tag = "reject"
        text = describe_result(value)
        extra = note(value) if note is not None else None
        self.log(f"{label} → {text}" + (f" [{extra}]" if extra else ""), tag)
        if on_result is not None:
            on_result(value)

    def _call(self, label: str, start: Callable[[], Future],
              on_result: Callable[[Any], None] | None = None, key: str | None = None,
              note: Callable[[Any], str | None] | None = None) -> bool:
        """연결 확인 후 async API를 호출한다. 동기 예외(인자·종료 상태)도 로그로 남긴다.

        key가 있으면 응답이 올 때까지 같은 요청을 다시 보내지 않고 해당 버튼을 비활성화한다(중복 클릭 방지).
        """
        if self.client is None:
            self.log(f"{label} 실패: 서비스에 연결되지 않았습니다", "error")
            return False
        if key is not None and key in self._inflight:
            self.log(f"{label} 무시: 이전 요청 응답 대기 중")
            return False
        self.log(f"{label} 요청")
        try:
            future = start()
        except Exception as exc:
            self.log(f"{label} 호출 오류 {describe_error(exc)}", "error")
            return False
        if key is not None:
            self._inflight.add(key)
            self._set_buttons(key, "disabled")
        self._watch(label, future, on_result, key, note)
        return True

    def _input_error(self, exc: InputError) -> None:
        self.log(f"입력 오류 {exc}", "error")
        messagebox.showerror("입력 오류", str(exc), parent=self.root)

    # ---------------------------------------------------------------- 연결

    def connect(self) -> None:
        if self.client is not None or self._connecting:
            self.log("이미 연결됐거나 연결 중입니다")
            return
        endpoint = self.endpoint_var.get().strip()
        if not endpoint:
            self._input_error(InputError("Endpoint: 값을 입력하세요"))
            return
        self._connecting = True
        self.conn_var.set("연결 중…")
        self.log(f"연결 시도 {endpoint}")

        def work() -> None:   # Client 생성자는 HELLO 응답까지 blocking하므로 별도 thread
            try:
                client = self._client_factory(endpoint)
            except BaseException as exc:
                self.post(lambda exc=exc: self._on_connect_failed(endpoint, exc))
                return
            self.post(lambda: self._on_connected(endpoint, client))

        threading.Thread(target=work, name="api-gui-connect", daemon=True).start()

    def _on_connected(self, endpoint: str, client: Any) -> None:
        self._connecting = False
        if self._closed:
            threading.Thread(target=client.close, daemon=True).start()
            return
        self.client = client
        try:
            client.on_control_lost(lambda reason: self.post(
                lambda: self.log(f"제어권 상실 통지 reason={reason}", "error")))
            info = client.get_control_status()
        except Exception as exc:
            info = {}
            self.log(f"연결 정보 조회 오류 {describe_error(exc)}", "error")
        self.conn_var.set(f"연결됨 {endpoint}")
        self.log(f"연결됨 {endpoint} service_instance_id={info.get('service_instance_id')} "
                 f"client_instance_id={info.get('client_instance_id')}", "ok")
        self.refresh_status()
        if self.poll_var.get():
            self._schedule_poll()

    def _on_connect_failed(self, endpoint: str, exc: BaseException) -> None:
        self._connecting = False
        self.conn_var.set("연결 안 됨")
        self.log(f"연결 실패 {endpoint} {describe_error(exc)}", "error")

    def disconnect(self) -> None:
        client, self.client = self.client, None
        self._cancel_poll()
        self._status_in_flight = False
        for key in list(self._inflight):
            self._end_request(key)
        if client is None:
            self.log("연결돼 있지 않습니다")
            return
        self.conn_var.set("종료 중…")
        self.log("연결 종료(제어권이 있으면 반환 시도)")

        def work() -> None:   # close()는 제어권 반환을 최대 수 초 기다릴 수 있다
            try:
                client.close()
            except BaseException as exc:
                self.post(lambda exc=exc: self.log(f"종료 오류 {describe_error(exc)}", "error"))
            self.post(self._on_disconnected)

        threading.Thread(target=work, name="api-gui-close", daemon=True).start()

    def _on_disconnected(self) -> None:
        if self.client is None and not self._connecting:
            self.conn_var.set("연결 안 됨")
        self.log("연결 종료됨")

    def on_window_close(self) -> None:
        client, self.client = self.client, None
        self._closed = True
        self._cancel_poll()
        if client is not None:
            # 창은 바로 닫고 제어권 반환은 Client가 처리한다. 실패해도 서비스 heartbeat 만료가 같은 절차를 수행한다.
            t = threading.Thread(target=client.close, name="api-gui-close", daemon=True)
            t.start()
            t.join(timeout=4.0)
        self.root.destroy()

    # ---------------------------------------------------------------- 상태

    def refresh_status(self) -> None:
        if self.client is None:
            self.log("상태 조회 실패: 서비스에 연결되지 않았습니다", "error")
            return
        self._request_status(log_request=True)

    def _request_status(self, log_request: bool) -> None:
        if self.client is None or self._status_in_flight:
            return
        try:
            future = self.client.get_status_async()
        except Exception as exc:
            self.log(f"상태 조회 호출 오류 {describe_error(exc)}", "error")
            return
        self._status_in_flight = True
        if log_request:
            self.log("상태 조회 요청")

        def done(f: Future) -> None:   # Client thread
            try:
                value = f.result()
            except BaseException as exc:
                self.post(lambda exc=exc: self._on_status_error(exc))
                return
            self.post(lambda: self._on_status(value))

        future.add_done_callback(done)

    def _on_status(self, status: Any) -> None:
        self._status_in_flight = False
        self.show_status(status)

    def _on_status_error(self, exc: BaseException) -> None:
        self._status_in_flight = False
        self.log(f"상태 조회 예외 {describe_error(exc)}", "error")

    def show_status(self, status: Any) -> None:
        text = json.dumps(to_plain(status), ensure_ascii=False, indent=2, sort_keys=True)
        self.status_text.configure(state="normal")
        self.status_text.delete("1.0", "end")
        self.status_text.insert("end", text)
        self.status_text.configure(state="disabled")
        self.summary_var.set(summarize_status(status, self.client))
        self.root.title(f"Patrol Robot API 시험 — {self.summary_var.get()}")   # 요약은 창 제목에 표시
        for key, (value, level) in dashboard_values(status).items():
            label = self.dash_labels.get(key)
            if label is not None:
                label.configure(text=value, fg=LEVEL_COLORS[level],
                                font=("TkDefaultFont", 9, "bold") if level == "bad" else ("TkDefaultFont", 9))

    def on_poll_toggle(self) -> None:
        if self.poll_var.get():
            try:
                self._poll_interval()
            except InputError as exc:
                self.poll_var.set(False)
                self._input_error(exc)
                return
            self.log("상태 주기 조회 시작")
            self._schedule_poll()
        else:
            self._cancel_poll()
            self.log("상태 주기 조회 중지")

    def _poll_interval(self) -> float:
        sec = parse_positive_float("조회 간격(초)", self.poll_sec_var.get())
        if sec < MIN_POLL_SEC:
            raise InputError(f"조회 간격(초): {MIN_POLL_SEC} 이상이어야 합니다")
        return sec

    def _schedule_poll(self) -> None:
        self._cancel_poll()
        if self.client is None or not self.poll_var.get():
            return
        try:
            sec = self._poll_interval()
        except InputError as exc:
            self.poll_var.set(False)
            self.log(f"주기 조회 중지: {exc}", "error")
            return
        self._poll_job = self.root.after(int(sec * 1000), self._poll_tick)

    def _poll_tick(self) -> None:
        self._poll_job = None
        self._request_status(log_request=False)
        self._schedule_poll()

    def _cancel_poll(self) -> None:
        if self._poll_job is not None:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None

    # ---------------------------------------------------------------- 제어권·모드·안전

    def acquire_control(self) -> None:
        self._call("제어권 획득", lambda: self.client.acquire_control_async(), key="acquire")

    def release_control(self) -> None:
        self._call("제어권 반환", lambda: self.client.release_control_async(), key="release")

    def set_mode(self, mode: ControlMode) -> None:
        self._call(f"모드 변경 {mode.value}", lambda: self.client.set_control_mode_async(mode),
                   key=f"mode.{mode.value}")

    def emergency_stop(self) -> None:
        # 비상정지는 멱등이고 안전 우선이라 요청 중에도 버튼을 잠그지 않는다.
        self._call("비상정지", lambda: self.client.emergency_stop_async())

    def release_emergency_stop(self) -> None:
        self._call("비상정지 해제", lambda: self.client.release_emergency_stop_async(),
                   key="release_estop")

    def stop_all(self) -> None:
        for device in ("ptu", "mast", "erector"):   # 한 장치 호출이 실패해도 나머지를 시도한다
            self.device_cmd(device, "stop")

    def reset(self) -> None:
        if not messagebox.askyesno(
                "전체 RESET", "RESET은 Mast Home → PTU Home → Erector Stow 순서로 장치를 움직입니다.\n"
                "실행할까요?", parent=self.root):
            self.log("전체 RESET 취소(사용자)")
            return
        self._call("전체 RESET", lambda: self.client.reset_async(),
                   lambda v: self._on_receipt(v, "reset"), key="reset")

    # ---------------------------------------------------------------- 장치 명령

    def device_cmd(self, device: str, action: str, *args: Any) -> bool:
        """`client.<device>.<action>_async(*args)` 호출."""
        label = f"{device}.{action}" + (f"({', '.join(map(repr, args))})" if args else "")
        return self._call(label, lambda: getattr(getattr(self.client, device), f"{action}_async")(*args),
                          lambda v: self._on_receipt(v, f"{device}.{action}"), key=f"{device}.{action}")

    def _with_input(self, fn: Callable[[], None]) -> None:
        try:
            fn()
        except InputError as exc:
            self._input_error(exc)

    def ptu_move_to_angle(self) -> None:
        self._with_input(lambda: self.device_cmd(
            "ptu", "move_to_angle", parse_float("pan_deg", self.ptu_pan_var.get()),
            parse_float("tilt_deg", self.ptu_tilt_var.get())))

    def ptu_set_speed(self) -> None:
        self._with_input(lambda: self.device_cmd(
            "ptu", "set_speed", parse_int_code("PTU speed_code", self.ptu_speed_var.get(), *PTU_SPEED_RANGE)))

    def mast_move_to_height(self) -> None:
        self._with_input(lambda: self.device_cmd(
            "mast", "move_to_height", parse_float("height_mm", self.mast_height_var.get())))

    def erector_move_to(self) -> None:
        self._with_input(lambda: self.device_cmd(
            "erector", "move_to", parse_float("position_mm", self.erector_pos_var.get())))

    def erector_move_by(self) -> None:
        self._with_input(lambda: self.device_cmd(
            "erector", "move_by", parse_float("delta_mm", self.erector_delta_var.get())))

    def erector_set_speed(self) -> None:
        self._with_input(lambda: self.device_cmd(
            "erector", "set_speed",
            parse_int_code("Erector speed_code", self.erector_speed_var.get(), *ERECTOR_SPEED_RANGE)))

    # ---------------------------------------------------------------- command_id

    def _on_receipt(self, value: Any, action: str | None = None) -> None:
        if not isinstance(value, CommandReceipt) or not value.accepted or not value.command_id:
            return
        if action is not None:
            self._command_actions[value.command_id] = action
        self.remember_command_id(value.command_id)
        if self.auto_wait_var.get():
            self.wait_command(value.command_id)

    def remember_command_id(self, command_id: str) -> None:
        if command_id in self._command_ids:
            self._command_ids.remove(command_id)
        self._command_ids.insert(0, command_id)
        del self._command_ids[50:]
        self.command_combo.configure(values=self._command_ids)
        self.command_id_var.set(command_id)

    def get_command(self) -> None:
        try:
            cid = parse_command_id(self.command_id_var.get())
        except InputError as exc:
            self._input_error(exc)
            return
        self._call(f"명령 조회 {cid}", lambda: self.client.get_command_async(cid), note=self._note_for(cid))

    def wait_command(self, command_id: str | None = None) -> None:
        try:
            cid = parse_command_id(command_id if command_id is not None else self.command_id_var.get())
            timeout = parse_positive_float("대기(초)", self.wait_sec_var.get())
        except InputError as exc:
            self._input_error(exc)
            return
        self._call(f"완료 대기 {cid} (최대 {timeout:g}초)", lambda: self.client.wait_command_async(cid, timeout),
                   note=self._note_for(cid))

    def _note_for(self, command_id: str) -> Callable[[Any], str | None]:
        action = self._command_actions.get(command_id)
        return lambda result: completion_note(action, result)


def summarize_status(status: Any, client: Any = None) -> str:
    """창 제목용 상태 요약 한 줄. 필드가 없어도 실패하지 않는다."""
    if not isinstance(status, dict):
        return "-"
    version = parse_schema_version(status.get("schema_version"))
    if version is None or version[0] != SUPPORTED_SCHEMA_MAJOR:
        return f"schema {status.get('schema_version')} 해석 안 함"
    parts = [f"state={_get(status, 'system', 'control_state') or '?'}"]
    if client is not None:
        try:
            parts.append(f"has_control={client.has_control()}")
        except Exception:
            pass
    for role in ("ptu", "erector", "digital_counter"):
        parts.append(f"{role}={_get(status, 'devices', role, 'response', 'state') or '?'}")
    active = status.get("active_commands")
    if isinstance(active, list):
        parts.append(f"작업={len(active)}")
    return "  ".join(parts)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Patrol Robot API 시험 GUI")
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT, help=f"서비스 ZMQ endpoint (기본 {DEFAULT_ENDPOINT})")
    args = parser.parse_args(argv)
    root = tk.Tk()
    width = min(1400, root.winfo_screenwidth())
    height = min(900, root.winfo_screenheight() - 60)
    root.geometry(f"{width}x{height}+0+0")
    ApiTestGui(root, endpoint=args.endpoint)
    root.mainloop()


if __name__ == "__main__":
    main()
