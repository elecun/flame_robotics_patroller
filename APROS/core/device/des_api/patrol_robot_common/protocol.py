"""Python Client ↔ Patrol Robot Service ZMQ 메시지 규격. (04 §11, 06 §5)

전송: 클라이언트 DEALER ↔ 서비스 ROUTER. 메시지 본문은 UTF-8 JSON 한 프레임이다.

요청::

    {"protocol": "patrol-robot", "version": 1, "method": "...", "params": {...},
     "request_id": "...", "client_instance_id": "...", "lease_token": "..." | null,
     "expires_at": <unix time>}

응답::

    {"protocol": "patrol-robot", "version": 1, "request_id": "...",
     "service_instance_id": "...", "ok": true, "result": {...}}
    {"protocol": "patrol-robot", "version": 1, "request_id": "...",
     "service_instance_id": "...", "ok": false, "error": {"code": "...", "message": "..."}}

- 같은 고객사 호출의 자동 재시도는 같은 `request_id`를 쓴다. 서비스는 실행 중·완료된 요청을 기억해
  같은 요청을 다시 실행하지 않는다.
- `expires_at`이 지난 요청은 실행하지 않는다. 연결 복구나 서비스 재시작 뒤 늦게 전달된 과거 요청이
  장치를 움직이지 않게 하기 위해서다(04 §4).
- 이 규격은 직접 ZMQ 사용 문서화의 기반이며, 호환되지 않는 변경은 `version`을 올린다.
"""

from __future__ import annotations

import json
import math
from enum import Enum
from typing import Any, Mapping

PROTOCOL_NAME = "patrol-robot"
PROTOCOL_VERSION = 1
DEFAULT_ENDPOINT = "tcp://127.0.0.1:5590"
MAX_MESSAGE_BYTES = 1 << 20


class Method(str, Enum):
    HELLO = "hello"
    GET_STATUS = "get_status"
    GET_COMMAND = "get_command"
    ACQUIRE_CONTROL = "acquire_control"
    HEARTBEAT = "heartbeat"
    RELEASE_CONTROL = "release_control"
    SET_CONTROL_MODE = "set_control_mode"
    SUBMIT_COMMAND = "submit_command"
    RESET = "reset"
    EMERGENCY_STOP = "emergency_stop"
    RELEASE_EMERGENCY_STOP = "release_emergency_stop"


# 상태를 바꾸는 요청만 request_id 중복 제거 대상이다. 조회·heartbeat는 다시 실행해도 안전하다.
DEDUPED_METHODS = frozenset({
    Method.ACQUIRE_CONTROL,
    Method.RELEASE_CONTROL,
    Method.SET_CONTROL_MODE,
    Method.SUBMIT_COMMAND,
    Method.RESET,
    Method.EMERGENCY_STOP,
    Method.RELEASE_EMERGENCY_STOP,
})

# 다른 요청 처리에 막히지 않도록 별도 worker에서 처리하는 안전 요청
SAFETY_METHODS = frozenset({Method.EMERGENCY_STOP, Method.RELEASE_EMERGENCY_STOP})


class ApiErrorCode(str, Enum):
    PROTOCOL_VERSION_MISMATCH = "PROTOCOL_VERSION_MISMATCH"
    BAD_REQUEST = "BAD_REQUEST"
    UNKNOWN_METHOD = "UNKNOWN_METHOD"
    INVALID_PARAMS = "INVALID_PARAMS"
    EXPIRED = "EXPIRED"
    # 같은 (client_instance_id, request_id)로 다른 요청(method·params·lease_token)이 도착함
    REQUEST_ID_CONFLICT = "REQUEST_ID_CONFLICT"
    # 처리 대기 요청이 상한에 도달함. 비상정지는 이 제한을 받지 않는다.
    BUSY = "BUSY"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class MessageFormatError(ValueError):
    """메시지 프레임이나 envelope을 해석할 수 없음."""


def _dumps(obj: Mapping[str, Any]) -> bytes:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _reject_constant(name: str) -> Any:
    raise MessageFormatError(f"non-standard JSON constant {name} is not allowed")


def _loads(frame: bytes) -> dict[str, Any]:
    if len(frame) > MAX_MESSAGE_BYTES:
        raise MessageFormatError("message too large")
    try:
        # NaN·Infinity 같은 비표준 상수는 params 안쪽까지 모두 거부한다.
        data = json.loads(frame.decode("utf-8"), parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError) as exc:
        raise MessageFormatError(f"invalid JSON: {exc}") from None
    if not isinstance(data, dict):
        raise MessageFormatError("message must be a JSON object")
    return data


def _check_envelope(data: Mapping[str, Any]) -> None:
    if data.get("protocol") != PROTOCOL_NAME:
        raise MessageFormatError("unknown protocol")
    version = data.get("version")
    if not isinstance(version, int) or isinstance(version, bool):
        raise MessageFormatError("version must be int")


def encode_request(
    method: Method,
    params: Mapping[str, Any],
    *,
    request_id: str,
    client_instance_id: str,
    lease_token: str | None,
    expires_at: float,
) -> bytes:
    return _dumps({
        "protocol": PROTOCOL_NAME,
        "version": PROTOCOL_VERSION,
        "method": method.value,
        "params": dict(params),
        "request_id": request_id,
        "client_instance_id": client_instance_id,
        "lease_token": lease_token,
        "expires_at": expires_at,
    })


def decode_request(frame: bytes) -> dict[str, Any]:
    """envelope 형식만 검사한다. version 불일치 판단과 method·params 검증은 서버가 한다."""
    data = _loads(frame)
    _check_envelope(data)
    for key in ("request_id", "client_instance_id", "method"):
        if not isinstance(data.get(key), str) or not data[key]:
            raise MessageFormatError(f"{key} must be a non-empty string")
    if not isinstance(data.get("params"), dict):
        raise MessageFormatError("params must be an object")
    token = data.get("lease_token")
    if token is not None and not isinstance(token, str):
        raise MessageFormatError("lease_token must be a string or null")
    expires = data.get("expires_at")
    if not isinstance(expires, (int, float)) or isinstance(expires, bool) or not math.isfinite(expires):
        raise MessageFormatError("expires_at must be a finite number")
    return data


def request_fingerprint(method: str, params: Mapping[str, Any], lease_token: str | None) -> str:
    """같은 request_id 재시도인지 판정하는 요청 지문. expires_at은 재시도마다 같아 포함하지 않아도 된다."""
    return json.dumps([method, params, lease_token], sort_keys=True, ensure_ascii=False,
                      separators=(",", ":"), allow_nan=False)


def encode_response(
    request_id: str,
    service_instance_id: str,
    *,
    result: Any = None,
    error: tuple[ApiErrorCode, str] | None = None,
) -> bytes:
    body: dict[str, Any] = {
        "protocol": PROTOCOL_NAME,
        "version": PROTOCOL_VERSION,
        "request_id": request_id,
        "service_instance_id": service_instance_id,
    }
    if error is None:
        body.update(ok=True, result=result)
    else:
        code, message = error
        body.update(ok=False, error={"code": code.value, "message": message})
    return _dumps(body)


def decode_response(frame: bytes) -> dict[str, Any]:
    data = _loads(frame)
    _check_envelope(data)
    if not isinstance(data.get("request_id"), str):
        raise MessageFormatError("request_id must be a string")
    if not isinstance(data.get("service_instance_id"), str):
        raise MessageFormatError("service_instance_id must be a string")
    ok = data.get("ok")
    if not isinstance(ok, bool):
        raise MessageFormatError("ok must be bool")
    if not ok:
        error = data.get("error")
        if (
            not isinstance(error, dict)
            or not isinstance(error.get("code"), str)
            or not isinstance(error.get("message"), str)
        ):
            raise MessageFormatError("error must have string code and message")
    return data
