"""경계를 통과하는 직렬화 가능한 DTO.

DTO는 문자열, 숫자, boolean, enum, list, map만으로 구성한다(02 §7).
`to_dict()` 결과는 JSON으로 그대로 직렬화할 수 있어야 한다.

생성자와 `from_dict()`는 공개 schema 타입을 엄격히 검증하고 묵시적 변환을 하지 않는다.
예: 문자열 "false"를 boolean으로, 문자열 "7"을 정수로 바꾸지 않는다.
잘못된 타입은 `TypeError`, 잘못된 값·누락 키·정의되지 않은 enum 값은 `ValueError`다.
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Mapping, TypeVar

from core.device.des_api.patrol_robot_common.enums import (
    CommandOrigin,
    CommandState,
    DeviceId,
    RejectReason,
    TxResult,
)

E = TypeVar("E", bound=Enum)


def new_request_id() -> str:
    """고객사 메서드 호출 하나에 대응하는 요청 ID. 자동 재시도에서는 같은 값을 유지한다."""
    return uuid.uuid4().hex


def new_command_id() -> str:
    """서비스가 접수한 장시간 명령 ID."""
    return uuid.uuid4().hex


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


# --- 엄격한 타입 검사 ---------------------------------------------------------


def _require_bool(name: str, value: Any) -> bool:
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be bool, got {type(value).__name__}")
    return value


def _require_int(name: str, value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError(f"{name} must be int, got {type(value).__name__}")
    return value


def _require_str(name: str, value: Any, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be str, got {type(value).__name__}")
    if not allow_empty and not value:
        raise ValueError(f"{name} must not be empty")
    return value


def _optional_str(name: str, value: Any) -> str | None:
    return None if value is None else _require_str(name, value)


def _optional_int(name: str, value: Any) -> int | None:
    return None if value is None else _require_int(name, value)


def _require_enum(name: str, enum_type: type[E], value: Any) -> E:
    if not isinstance(value, enum_type):
        raise TypeError(f"{name} must be {enum_type.__name__}, got {type(value).__name__}")
    return value


def _optional_enum(name: str, enum_type: type[E], value: Any) -> E | None:
    return None if value is None else _require_enum(name, enum_type, value)


def _parse_enum(name: str, enum_type: type[E], value: Any) -> E:
    """직렬화된 enum 문자열을 해석한다. 정의된 문자열만 허용한다."""
    _require_str(name, value)
    try:
        return enum_type(value)
    except ValueError:
        raise ValueError(f"{name}: unknown {enum_type.__name__} value {value!r}") from None


def _parse_optional_enum(name: str, enum_type: type[E], value: Any) -> E | None:
    return None if value is None else _parse_enum(name, enum_type, value)


def _require_mapping(name: str, value: Any) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} must be a mapping, got {type(value).__name__}")
    return value


def _field(data: Mapping[str, Any], key: str) -> Any:
    """필수 키를 읽는다. 누락은 ValueError."""
    if key not in data:
        raise ValueError(f"missing field: {key}")
    return data[key]


# --- DTO -----------------------------------------------------------------------


@dataclass(frozen=True)
class CommandContext:
    """queue 경계를 넘을 때 함께 전달하는 명령 문맥.

    origin과 요청 식별자를 잃지 않도록 모든 내부 명령 DTO가 이 값을 포함한다.
    `lease_generation`은 PROGRAM_LEASE 명령에만 의미가 있다.
    """

    origin: CommandOrigin
    request_id: str | None = None
    command_id: str | None = None
    lease_generation: int | None = None
    client_instance_id: str | None = None

    def __post_init__(self) -> None:
        _require_enum("origin", CommandOrigin, self.origin)
        _optional_str("request_id", self.request_id)
        _optional_str("command_id", self.command_id)
        _optional_int("lease_generation", self.lease_generation)
        _optional_str("client_instance_id", self.client_instance_id)
        if self.origin is CommandOrigin.PROGRAM_LEASE and self.lease_generation is None:
            raise ValueError("PROGRAM_LEASE command requires lease_generation")

    def to_dict(self) -> dict[str, Any]:
        return {
            "origin": self.origin.value,
            "request_id": self.request_id,
            "command_id": self.command_id,
            "lease_generation": self.lease_generation,
            "client_instance_id": self.client_instance_id,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> CommandContext:
        _require_mapping("CommandContext", data)
        return cls(
            origin=_parse_enum("origin", CommandOrigin, _field(data, "origin")),
            request_id=_field(data, "request_id"),
            command_id=_field(data, "command_id"),
            lease_generation=_field(data, "lease_generation"),
            client_instance_id=_field(data, "client_instance_id"),
        )


@dataclass(frozen=True)
class CommandReceipt:
    """명령 접수 결과. 실제 이동 완료가 아니다. (04 §5)

    accepted이면 command_id가 있고 reason이 없다. 거부이면 command_id가 없고 reason이 있다.
    """

    accepted: bool
    command_id: str | None
    reason: RejectReason | None
    message: str

    def __post_init__(self) -> None:
        _require_bool("accepted", self.accepted)
        _optional_str("command_id", self.command_id)
        _optional_enum("reason", RejectReason, self.reason)
        _require_str("message", self.message, allow_empty=True)
        if self.accepted:
            if self.command_id is None:
                raise ValueError("accepted receipt requires command_id")
            if self.reason is not None:
                raise ValueError("accepted receipt must not have a reject reason")
        else:
            if self.command_id is not None:
                raise ValueError("rejected receipt must not have command_id")
            if self.reason is None:
                raise ValueError("rejected receipt requires a reason")

    @classmethod
    def accept(cls, command_id: str, message: str = "") -> CommandReceipt:
        return cls(accepted=True, command_id=command_id, reason=None, message=message)

    @classmethod
    def reject(cls, reason: RejectReason, message: str = "") -> CommandReceipt:
        return cls(accepted=False, command_id=None, reason=reason, message=message)

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "command_id": self.command_id,
            "reason": None if self.reason is None else self.reason.value,
            "message": self.message,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> CommandReceipt:
        _require_mapping("CommandReceipt", data)
        return cls(
            accepted=_field(data, "accepted"),
            command_id=_field(data, "command_id"),
            reason=_parse_optional_enum("reason", RejectReason, _field(data, "reason")),
            message=_field(data, "message"),
        )


@dataclass(frozen=True)
class CommandResult:
    """장시간 명령의 현재 또는 최종 상태."""

    command_id: str
    state: CommandState
    reason: RejectReason | None = None
    message: str = ""
    updated_at: str = field(default_factory=utc_now_iso)

    def __post_init__(self) -> None:
        _require_str("command_id", self.command_id)
        _require_enum("state", CommandState, self.state)
        _optional_enum("reason", RejectReason, self.reason)
        _require_str("message", self.message, allow_empty=True)
        _require_str("updated_at", self.updated_at)

    def to_dict(self) -> dict[str, Any]:
        return {
            "command_id": self.command_id,
            "state": self.state.value,
            "reason": None if self.reason is None else self.reason.value,
            "message": self.message,
            "updated_at": self.updated_at,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> CommandResult:
        _require_mapping("CommandResult", data)
        return cls(
            command_id=_field(data, "command_id"),
            state=_parse_enum("state", CommandState, _field(data, "state")),
            reason=_parse_optional_enum("reason", RejectReason, _field(data, "reason")),
            message=_field(data, "message"),
            updated_at=_field(data, "updated_at"),
        )


@dataclass(frozen=True)
class SensorValue:
    """값의 유효성(valid)과 표본의 최신성(fresh)을 따로 가지는 측정값. (01 §5)

    `valid`는 마지막 표본의 값이 사용 가능할 때 True다(값의 유효성).
    `fresh`는 마지막 표본이 현재 연결 세대에서 왔고 허용 시간이 지나지 않았을 때 True다(표본의 최신성).
    두 값은 독립이다. 장치가 값 무효를 보고한 최신 표본은 valid=False, fresh=True다.
    값을 사용할 때는 반드시 `usable_for_control`(valid and fresh)로 판단하고 fresh만 보지 않는다.
    valid 값은 bool이 아닌 유한한 숫자와 표본 시각이 있어야 하고, fresh 값은 표본 시각이 있어야 한다.
    """

    value: float | int | None
    valid: bool
    fresh: bool
    updated_at: str | None
    error: str | None = None

    def __post_init__(self) -> None:
        if self.value is not None:
            if not isinstance(self.value, (int, float)) or isinstance(self.value, bool):
                raise TypeError(f"value must be int or float, got {type(self.value).__name__}")
        _require_bool("valid", self.valid)
        _require_bool("fresh", self.fresh)
        _optional_str("updated_at", self.updated_at)
        _optional_str("error", self.error)
        if self.valid:
            if self.value is None:
                raise ValueError("valid SensorValue requires a value")
            if not math.isfinite(self.value):
                raise ValueError("valid SensorValue must be finite")
            if self.updated_at is None:
                raise ValueError("valid SensorValue requires updated_at")
        if self.fresh and self.updated_at is None:
            raise ValueError("fresh SensorValue requires updated_at")

    @property
    def usable_for_control(self) -> bool:
        """센서값이 필요한 제어 판단에 사용 가능한지. (01 §5)"""
        return self.valid and self.fresh

    @classmethod
    def unknown(cls, error: str | None = None) -> SensorValue:
        return cls(value=None, valid=False, fresh=False, updated_at=None, error=error)

    def to_dict(self) -> dict[str, Any]:
        return {
            "value": self.value,
            "valid": self.valid,
            "fresh": self.fresh,
            "updated_at": self.updated_at,
            "error": self.error,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> SensorValue:
        _require_mapping("SensorValue", data)
        return cls(
            value=_field(data, "value"),
            valid=_field(data, "valid"),
            fresh=_field(data, "fresh"),
            updated_at=_field(data, "updated_at"),
            error=_field(data, "error"),
        )


STATUS_SCHEMA_VERSION = 1
SUPPORTED_STATUS_SCHEMA_VERSIONS = frozenset({STATUS_SCHEMA_VERSION})


@dataclass(frozen=True)
class StatusEnvelope:
    """상태 snapshot·이벤트의 공통 envelope.

    `kind`는 payload 종류(예: "system_status"), `sequence`는 발행자 단위 단조 증가 번호다.
    payload 세부 필드는 이후 단계에서 정한다.
    """

    kind: str
    sequence: int
    payload: Mapping[str, Any]
    generated_at: str = field(default_factory=utc_now_iso)
    schema_version: int = STATUS_SCHEMA_VERSION

    def __post_init__(self) -> None:
        _require_str("kind", self.kind)
        _require_int("sequence", self.sequence)
        if self.sequence < 0:
            raise ValueError("sequence must be >= 0")
        _require_mapping("payload", self.payload)
        _require_str("generated_at", self.generated_at)
        _require_int("schema_version", self.schema_version)
        if self.schema_version not in SUPPORTED_STATUS_SCHEMA_VERSIONS:
            raise ValueError(f"unsupported schema_version: {self.schema_version}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "kind": self.kind,
            "sequence": self.sequence,
            "generated_at": self.generated_at,
            "payload": dict(self.payload),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> StatusEnvelope:
        _require_mapping("StatusEnvelope", data)
        payload = _require_mapping("payload", _field(data, "payload"))
        return cls(
            kind=_field(data, "kind"),
            sequence=_field(data, "sequence"),
            payload=dict(payload),
            generated_at=_field(data, "generated_at"),
            schema_version=_field(data, "schema_version"),
        )


@dataclass(frozen=True)
class StopAttempt:
    """장치 하나의 STOP 시도 결과. (04 §7)

    `tx_result`는 로컬 전송 결과다. 장치가 STOP을 받았거나 실제로 정지했다는 확인이 아니다.
    정지 여부는 장치 상태(motion, 위치 변화)로 확인한다.
    """

    device: DeviceId
    tx_result: TxResult
    reason: RejectReason | None = None
    error_code: str | None = None
    error_message: str | None = None

    def __post_init__(self) -> None:
        _require_enum("device", DeviceId, self.device)
        _require_enum("tx_result", TxResult, self.tx_result)
        _optional_enum("reason", RejectReason, self.reason)
        _optional_str("error_code", self.error_code)
        if self.error_message is not None:
            _require_str("error_message", self.error_message, allow_empty=True)

    def to_dict(self) -> dict[str, Any]:
        error = None
        if self.error_code is not None:
            error = {"code": self.error_code, "message": self.error_message or ""}
        return {
            "device": self.device.value,
            "tx_result": self.tx_result.value,
            "reason": None if self.reason is None else self.reason.value,
            "error": error,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> StopAttempt:
        _require_mapping("StopAttempt", data)
        error = _field(data, "error")
        if error is not None:
            _require_mapping("error", error)
        return cls(
            device=_parse_enum("device", DeviceId, _field(data, "device")),
            tx_result=_parse_enum("tx_result", TxResult, _field(data, "tx_result")),
            reason=_parse_optional_enum("reason", RejectReason, _field(data, "reason")),
            error_code=None if error is None else _require_str("error.code", _field(error, "code")),
            error_message=None if error is None else _require_str(
                "error.message", _field(error, "message"), allow_empty=True),
        )


@dataclass(frozen=True)
class EmergencyStopResult:
    """비상정지 결과. (04 §7)

    `latched=True`는 새 동작을 차단했다는 뜻이며 모든 장치의 실제 정지 확인이 아니다.
    """

    latched: bool
    ptu: StopAttempt
    mast: StopAttempt
    erector: StopAttempt

    def __post_init__(self) -> None:
        _require_bool("latched", self.latched)
        for name, device in (("ptu", DeviceId.PTU), ("mast", DeviceId.MAST),
                             ("erector", DeviceId.ERECTOR)):
            attempt = getattr(self, name)
            if not isinstance(attempt, StopAttempt):
                raise TypeError(f"{name} must be StopAttempt")
            if attempt.device is not device:
                raise ValueError(f"{name}.device must be {device.value}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "latched": self.latched,
            "ptu": self.ptu.to_dict(),
            "mast": self.mast.to_dict(),
            "erector": self.erector.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> EmergencyStopResult:
        _require_mapping("EmergencyStopResult", data)
        return cls(
            latched=_field(data, "latched"),
            ptu=StopAttempt.from_dict(_field(data, "ptu")),
            mast=StopAttempt.from_dict(_field(data, "mast")),
            erector=StopAttempt.from_dict(_field(data, "erector")),
        )
