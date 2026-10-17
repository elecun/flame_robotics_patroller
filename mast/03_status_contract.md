# 공개 상태 API 명세

- 이 문서는 고객사 앱 개발 기준입니다. 실행 가능한 패키지는 차주 방문하여 검증 후 제공 예정입니다.
- 대상: 고객사 애플리케이션 개발자
- 상태 형식 버전: `schema_version = "1.0.0"`(최초 고객사 공개 계약, `major.minor.patch`, 10절).
- 명령·결과 형식과 메서드는 [`02_api_reference.md`](02_api_reference.md)에서 설명한다.

## 1. 설계 원칙

1. 공통 필드의 이름, JSON 타입, 단위, 의미는 하드웨어 모델·펌웨어·통신 방식과 관계없이 고정한다. 장치 차이는 서비스 내부 adapter가 흡수한다.
2. 측정값·판정 상태·동작·고장은 처음부터 **관측 객체(Observation)**로 제공한다. 나중에 bool을 객체로 바꾸는 식의 타입 변경은 하지 않는다.
3. 확인할 수 없는 것은 `false`, `0`, `STOPPED`, 정상으로 바꾸지 않는다.
   - "알 수 없음"은 `null`이나 `valid=false`로 표현한다. `fresh`는 표본의 최신성만 뜻하며 값을 써도 된다는 뜻이 아니다(6.1절).
   - `false`는 "실제 거짓" 또는 "미지원"(`supported=false`)만 뜻한다.
4. **확장 원칙**
   - 같은 major 안에서는 키를 추가할 수 있다(minor 증가). 장치별 측정·상태·준비 항목, 고장 코드, 열거값도 추가될 수 있다.
   - 같은 major 안에서 기존 키의 제거와 타입·단위·의미 변경은 하지 않는다.
   - 앱은 **알 수 없는 키를 무시**하고, 알 수 없는 열거값·고장 코드는 원문을 표시하고 "알 수 없음"으로 다뤄야 한다. 정상으로 단정하지 않는다.
5. 키 제거나 의미 변경이 필요하면 major를 올린다. 앱은 자신이 지원하는 major가 아니면 상태를 해석하지 않는다(10절).
6. 내부 판정용 순번, 원시 비트 번호, 레지스터 주소 같은 구현 세부는 공개 계약에 포함하지 않는다.

## 2. 최상위 구조

```
{
  "schema_version": "1.0.0",                # str, major.minor.patch
  "generated_at": "2026-10-08T03:00:00.000+00:00",  # str, UTC ISO 8601
  "system":   { ... },                      # 3절
  "links":    { "<link_id>": {"state": str} },       # 4절
  "remote":   { ... },                      # 5절
  "devices":  { "<role_id>": Device },      # 6~8절
  "active_commands": [ {"command_id": str, "action": str, "phase": str|null} ]
}
```

- `role_id`는 장치 역할 이름이며 고정이다: `ptu`, `mast`, `erector`, `digital_counter`, `bmu`. 새 역할은 키 추가로 제공한다.
- `active_commands`는 진행 중인 **작업 명령**(목표 이동, Home, Erect/Stow, RESET)만 담는다. 연속 이동·STOP·속도 설정은 담지 않는다.

## 3. system

| 키 | 타입 | 값 / 의미 |
|---|---|---|
| control_mode | str | `PROGRAM`, `MANUAL` |
| control_state | str | `PROGRAM`, `MANUAL_LOCKED`, `MANUAL_ARMING`, `MANUAL_READY`, `EMERGENCY_STOPPED` |
| program_control | str | `ACQUIRED`, `NONE` — 서비스 기준 제어권 보유 여부(어느 Client인지는 아래 키) |
| lease_client_instance_id | str \| null | 제어권을 가진 Client 인스턴스 ID |
| manual_authorization | str | `LOCKED`, `ARMING`, `READY`(프로그램 모드에서는 `LOCKED`) |
| health | str | `NORMAL`, `DEGRADED`(NPort 미연결 또는 활성 통신 경로 중 READY 아님), `EMERGENCY_STOPPED` |
| emergency_stop_latched | bool | 비상정지 고정 여부 |

`health`는 통신 경로·비상정지 요약이다. 장치별 판단은 `devices`를 본다.

## 4. links

`links`에는 **설정에서 활성화된 통신 경로만** 나타난다. 비활성 경로는 키가 없다.

| 키 | 타입 | 값 |
|---|---|---|
| state | str | `DISCONNECTED`, `CONNECTING`, `INITIALIZING`, `READY` |

`READY`는 포트·소켓이 열려 있다는 뜻이다. 그 경로의 장치가 응답한다는 뜻은 아니다.
### 통신 경로 이름과 물리 포트

| link_id | 역할 | 물리 인터페이스 |
|---|---|---|
| digital_counter_bus | 디지털카운터 | RS485 |
| actuator_bus | PTU·마스트·Erector 공용 버스 | RS485 |
| bmu | BMU | RS232 |
| nport | 리모콘 NPort | TCP |

이 이름들은 장치 역할을 나타내는 논리적인 이름이다. 실제 Linux 포트 이름(`/dev/ttyS0` 등)과는 별개이며, 설치 시 설정 파일의 `device`에 실제 연결 포트를 지정한다. 장치의 `link`는 이 표의 `link_id`를 참조한다.

## 5. remote(리모콘·NPort)

| 키 | 타입 | 의미 |
|---|---|---|
| nport_link | str | `CONNECTED`, `DISCONNECTED` — NPort TCP 연결 |
| nport_connection_epoch | int | TCP 연결 세대 번호. 재연결마다 증가 |
| activity | str | `RECENT`, `STALE`, `UNKNOWN` — 최근 리모콘 패킷 수신 여부(표시 전용) |

`activity=STALE`은 최근 입력이 없다는 뜻이지 리모콘 단절 확정이 아니다.

## 6. Device 공통 구조

```
Device = {
  "configuration": {"enabled": bool, "complete": bool, "missing": [str]},
  "link":         str | null,
  "response":     {"supported": bool | null, "state": str},
  "measurements": { "<name>": Observation },
  "states":       { "<name>": Observation },
  "motion":       Observation,
  "faults":       Observation,
  "readiness":    { "<capability>": Readiness },
  "details":      { ... }
}
```

| 키 | 의미 |
|---|---|
| configuration.enabled | 설정에서 이 장치(통신 경로 포함)를 사용하도록 했는가 |
| configuration.complete | 역할 기본 기능에 필요한 정적 설정(주소, 레지스터 맵 등)이 모두 있는가. 통신 상태와 무관 |
| configuration.missing | 빠진 설정 항목 목록(문자열, 표시용이며 앱이 해석하지 않음). 선택 기능의 설정(예: 마스트 목표 허용오차)이 빠지면 `complete=true`이면서 `missing`에 나타나고, 해당 기능의 readiness가 `available=false`가 된다 |
| link | 이 장치가 쓰는 `links`의 키. 같은 버스를 공유하는 장치는 같은 값을 가진다 |
| response.supported | 장치가 상태 응답을 보내는 기능을 갖는가. `null` = 지원 여부 미확인 |
| response.state | `UNKNOWN`, `INITIALIZING`, `ONLINE`, `OFFLINE`, `UNSUPPORTED`, `DISABLED` |

- `response.state`는 서비스가 장치 응답을 근거로 판정한 값이다. 통신 경로가 `READY`라는 것만으로는 `ONLINE`이 되지 않는다.
- 비활성 장치는 `configuration.enabled=false`, `response.state="DISABLED"`이며, 모든 관측은 `value=null, valid=false, fresh=false`다.

**응답 상태 공통 규칙**(응답 기능이 있는 장치: PTU, Erector, digital_counter, BMU)

- **응답 근거**는 그 장치의 **정상 응답 형식**으로 받아 해석에 성공한 응답(6.1절의 표본)이다.
  - 정상 응답 형식: 디지털카운터는 현재값 읽기 응답, BMU는 상태 응답, Erector는 상태 응답, PTU는 위치 응답이다.
  - 정상 상태 응답 안에 장치가 값 무효나 고장을 담아 보낸 경우도 응답 근거다. 따라서 `ONLINE`은 "장치가 응답한다"는 뜻이며, 장비나 측정값이 정상이라는 뜻이 아니다.
  - 응답 근거가 아닌 것: 무응답, 체크섬·CRC 오류, 길이·형식 오류, 다른 장치 주소의 응답, 해석 실패. **Modbus 예외 응답과 BMU 오류 응답은 CRC·주소가 정상이어도 응답 근거가 아니다.**

state는 아래 순서로 처음 맞는 조건에 따라 정한다.

| 순서 | 조건 | state |
|---|---|---|
| 1 | `configuration.enabled=false` | `DISABLED` |
| 2 | 응답 기능이 없는 장치(마스트, 7.2절) | `UNSUPPORTED` |
| 3 | 기본 필수 설정이 빠져 조회하지 않음(빠진 항목은 `configuration.missing`, 사유는 readiness `reasons`) | `UNKNOWN` |
| 4 | 서비스 시작 이후 응답 근거가 한 번도 없음. 통신 경로 상태와 관계없다(경로가 끊겨 있어도 `UNKNOWN`). **PTU 예외**: 첫 위치 응답 전에는 `INITIALIZING`, 초기화 제한시간을 넘으면 `OFFLINE`(7.1절) | `UNKNOWN` |
| 5 | 현재 연결 세대에서 받은 마지막 응답 근거의 나이가 그 장치의 최신 한계(9절) 이내. **PTU 예외**: pan·tilt 두 축 위치가 각각 현재 세대에서 받은 최신 응답이어야 한다(한 축만 최신이면 `OFFLINE`) | `ONLINE` |
| 6 | 그 외: 응답 이력은 있지만 최신 한계를 넘음, 통신 경로가 끊긴 동안, 재연결 후 새 세대의 첫 응답 근거를 받기 전 | `OFFLINE` |

- 조회가 한 번 실패했다고 바로 `OFFLINE`이 되지는 않는다. 마지막 응답 근거의 나이가 최신 한계를 넘을 때 `OFFLINE`이 된다.
- 응답 상태와 측정값 유효성은 별개다. `ONLINE`이면서 측정값이 `valid=false`일 수 있다(예: Erector 위치 무효 보고).

### 6.0 configuration 판정 기준

`configuration.complete`는 **설정 파일만으로** 판정한다. 장치 기능 지원 여부(`supported`), 통신 상태, 응답 여부와 무관하다. 설정값 `false`·`0`도 "설정됨"이다. `null`만 미설정이다.

| 장치 | 기본 필수 설정(없으면 complete=false) | 선택 기능 설정(없으면 complete=true, missing에 표시, 해당 readiness만 불가) |
|---|---|---|
| ptu | `devices.ptu.address`(없으면 서비스가 시작되지 않음) | - |
| mast | `devices.mast.address` | target_motion: `lower_limit_mm`, `upper_limit_mm`, `target_tolerance_mm`(하나라도 null이면 목표 이동·Home 불가). 하강 목표 이동: `downward_stop_lead_mm` |
| erector | `devices.erector.address` | - |
| digital_counter | `slave_id`, `function_code`, `present_value_register`, `word_order`, `value_signed`(모두 non-null. `value_signed: false`도 설정값) | - |
| bmu | `devices.bmu.address` | - |

- `configuration.enabled`는 장치가 쓰는 통신 경로(`transports.<link>.enabled`)가 켜져 있는지다.
- 기본 필수 설정이 빠지면 그 장치는 조회·명령을 하지 않는다. 관측은 미수신(`valid=false`)이고, readiness는 `capability_unavailable`이다.

### 6.1 Observation

```
Observation = {
  "supported":  bool | null,     # 이 관측 기능이 있는가. null = 미확인
  "value":      number | bool | str | [str] | null,
  "valid":      bool,
  "fresh":      bool,
  "updated_at": str | null,      # 마지막 표본의 수신 시각(UTC ISO 8601)
  "source":     {"device": str, "method": str},
  "unit":       str | null,      # measurements에서 필수, 그 외 null
  "error":      str | null       # 마지막 실패 사유(표시용, 앱이 해석하지 않음)
}
```

**정의**

1. **표본**: 그 관측 항목을 담은 장치의 정상 응답 frame 하나로, 해석에 성공한 것이다.
   - 관측은 **자신을 담은 응답으로만** 갱신된다. 예를 들어 PTU pan 응답은 tilt의 `updated_at`·`fresh`를 바꾸지 않는다. 같은 응답에 담긴 관측(Erector 위치와 상태 플래그, BMU 측정값들)은 같은 표본을 공유한다.
   - 장치가 값 무효나 고장을 담아 보낸 정상 응답도 표본이다.
   - 무응답, 체크섬·CRC 오류, 길이·형식 오류, 다른 장치 주소의 응답, 해석 실패, Modbus 예외 응답, BMU 오류 응답은 표본이 아니다(통신 실패로 다룬다).
2. **valid**는 **값의 유효성**이다. 마지막 표본의 값을 사용할 수 있으면 `true`다.
3. **fresh**는 **표본의 최신성**이다. 마지막 표본이 현재 통신 세대에서 왔고 그 나이가 장치의 최신 한계(9절) 이내이면 `true`다. `valid`와 독립이다.

| 상황 | value | valid | fresh | updated_at | error |
|---|---|---|---|---|---|
| 유효한 값의 표본 수신 | 새 값 | true | 최신 한계 이내 동안 true | 표본 시각 | null |
| 장치가 값 무효를 보고한 표본 수신(예: Erector 위치 무효) | null | false | 최신 한계 이내 동안 true | 표본 시각 | 무효 사유 |
| 통신 실패(표본 아님) | 마지막 표본 값 보존 | 보존 | 마지막 표본 기준. 실패 즉시가 아니라 나이가 한계를 넘으면 false | 보존 | 실패 사유 |
| 통신 경로 단절, 재연결 후 새 세대 첫 표본 전 | 보존 | 보존 | false | 보존 | 경로 사유 |
| 서비스 시작 이후 표본을 한 번도 받지 못함 | null | false | false | null | 사유 또는 null |
| `supported=false` 또는 `null` | null | false | false | null | null 또는 사유 |

- 이전 통신 세대의 표본은 재연결 후 `fresh=true`가 되지 않는다.
- 값 무효 표본을 받으면 이전 값을 보존하지 않고 `value=null`로 둔다.

**판독 규칙(앱 계약)**: 제어 판단에는 `supported == true && valid && fresh`일 때의 `value`만 사용한다. 그 외는 "확인 불가"로 표시한다. **`fresh`만 보고 값을 사용하지 않는다.**

| valid | fresh | 뜻 | 앱 표시 |
|---|---|---|---|
| true | true | 최신 유효값 | 값 사용 가능 |
| true | false | 마지막 유효값이 오래됨 | "오래된 값"으로 표시 가능. 제어 판단에는 쓰지 않음 |
| false | true | 장치가 최근에 응답했지만 값이 무효 | 값 없음. `error`(무효 사유)를 표시 |
| false | false | 최근 표본 없음 또는 미수신 | 확인 불가 |

- `source.method`
  - `DEVICE_REPORTED`: 장치 응답값
  - `POSITION_INFERRED`: 위치 변화로 추정
  - `DERIVED`: 다른 관측과 설정으로 계산
  - `COMMAND_ASSUMED`: 보낸 명령으로 가정. 실제 장치 확인이 아니다.
  - 새 method가 추가될 수 있다.
- `source.device`: 근거를 낸 역할 ID. 예를 들어 마스트 높이는 `digital_counter`다.

**파생 관측(`DERIVED`, `POSITION_INFERRED`)**: 필요한 모든 원천 관측이 `valid && fresh`일 때만 계산한다. 원천 중 하나라도 그렇지 않으면 `value=null`, `valid=false`, `fresh=false`이고, `error`에 사유가 들어간다. 파생값은 표본이 아니므로, 원천이 `fresh=true`여도 계산할 수 없으면 `fresh=false`다. `updated_at`은 필요한 원천 중 가장 오래된 표본 시각이다.

### 6.2 measurements / states / motion / faults

| 블록 | value 타입 | 규칙 |
|---|---|---|
| measurements.<name> | number | `unit` 필수. 키 이름에 단위 접미사를 쓴다(`_mm`, `_deg`, `_v` 등) |
| states.<name> | bool | 판정 상태(at_home 등) |
| motion | str | `MOVING`, `STOPPED`, `INITIALIZING`. 판정할 수 없으면 `value=null, valid=false` |
| faults | [str] | 현재 활성 고장 코드 목록. **빈 목록은 `supported && valid && fresh`일 때만 "활성 고장 없음"** |

이동 기능이 없는 장치의 `motion`과, 고장 보고 기능이 없는 장치의 `faults`는 `supported=false`이다.

### 6.3 readiness

```
Readiness = {"supported": bool, "available": bool, "reasons": [str]}
```

- `supported`: 이 장치에 이 기능이 있는가
- `available`: 설정·통신·피드백·비상정지 조건상 지금 접수될 수 있는가
- `reasons`: `available=false`인 사유. 명령 거부 사유와 같은 문자열이다(`device_offline`, `device_initializing`, `not_ready`, `capability_unavailable`, `emergency_stop_active` 등).
- **readiness에는 호출자 권한(제어 모드, 제어권 보유)이 포함되지 않는다.** 권한은 `system.control_mode`, `system.program_control`, Client의 `has_control()`로 확인한다.
- `available=true`라도 접수가 보장되지 않는다. 명령 접수 시 다시 검사하며, 결과는 `CommandReceipt`로 확인한다.
- 장치마다 기능별 준비가 다르다(8절 표). 예를 들어 위치 피드백이 없어 이동은 막혀도 STOP은 시도할 수 있다.

## 7. 장치별 필드

### 7.1 ptu

| 경로 | 타입 / 단위 | 채우는 기준 |
|---|---|---|
| response | 객체 / 단위 없음 | supported=true. 첫 위치 응답 전 `INITIALIZING`(초기화 제한시간 초과 시 `OFFLINE`), 두 축 위치가 모두 fresh이면 `ONLINE`, 그 외 `OFFLINE`. 경로 비활성 시 `DISABLED` |
| measurements.pan_deg | number, `deg` | PTU 응답(DEVICE_REPORTED) |
| measurements.tilt_deg | number, `deg` | PTU 응답 |
| states.at_home | bool | DERIVED: 두 축이 0° 기준 허용오차 안. pan·tilt가 모두 valid·fresh일 때만 |
| motion | `MOVING`/`STOPPED` | POSITION_INFERRED: 두 축 위치 변화, 보낸 이동 명령, 축별 속도 정보로 추정. 장치가 정지를 보고하는 것은 아님 |
| faults | [str] / 단위 없음 | supported=false. PTU 고장 보고 없음 |
| details.speed_code | int | 연속 이동 속도 **설정·명령값**(실측 아님) |

### 7.2 mast

| 경로 | 타입 / 단위 | 채우는 기준 |
|---|---|---|
| response | 객체 / 단위 없음 | supported=false, state=`UNSUPPORTED`. 마스트는 응답 기능이 없다. 높이 응답으로 마스트를 ONLINE으로 만들지 않는다 |
| measurements.height_mm | number, `mm` | 디지털카운터가 보고한 높이를 그대로 사용한다(`source.method=DEVICE_REPORTED`, `source.device=digital_counter`). |
| states.at_home | bool | DERIVED: 높이 ≤ 운용 하한 + 목표 허용오차. 하한·허용오차 설정이 없으면 value=null, valid=false |
| motion | `MOVING`/`STOPPED` | POSITION_INFERRED: 카운터 높이 변화로 추정 |
| faults | [str] / 단위 없음 | supported=false. |

- 디지털카운터는 절대위치 센서가 아니다. 전원 투입 시 기준값이 실제 높이와 맞는지는 `valid`가 보장하지 않는다. 운용 절차로 확인해야 한다.
- 정지 판정은 피드백 변화로 추정하므로 실제 정지와 시간 차이가 있다.

### 7.3 erector

| 경로 | 타입 / 단위 | 채우는 기준 |
|---|---|---|
| response | 객체 / 단위 없음 | supported=true. 정상 상태 응답이 최신이면 `ONLINE`, 과거 응답만 있으면 `OFFLINE`, 응답 이력이 없으면 `UNKNOWN` |
| measurements.position_mm | number, `mm` | 펌웨어 보고 위치. 펌웨어가 위치 무효를 보고하면 value=null, valid=false이고, fresh·updated_at은 그 표본 기준(아래 예) |
| states.at_home | bool | 펌웨어 보고(Home = Stow 위치) |
| states.at_erect | bool | 펌웨어 보고 |
| states.calibration_valid | bool | 펌웨어 보고(위치 보정값 유효) |
| states.soft_limit_reached | bool | 펌웨어 보고(소프트 리밋으로 이동이 제한·정지됨) |
| motion | `MOVING`/`STOPPED` | 펌웨어 보고 |
| faults | [str] | 펌웨어 보고. 코드: `stall_or_direction_fault` |

- **위치 무효와 보정 무효의 구분**
  - `measurements.position_mm.valid=false`이고 `states.calibration_valid`가 `valid && fresh`이며 `value=false`이면, 원인은 보정 무효다.
  - `calibration_valid=true`인데 위치가 무효이면 센서 이상 등 다른 원인이다. `position_mm.error`를 표시한다.
  - 위치 무효를 보고한 최신 응답의 예(같은 응답에서 나온 `calibration_valid`도 같은 시각·최신성):

```json
"position_mm": {"supported": true, "value": null, "valid": false, "fresh": true,
                "updated_at": "2026-10-08T03:00:59.900+00:00",
                "source": {"device": "erector", "method": "DEVICE_REPORTED"},
                "unit": "mm", "error": "position invalid"},
"calibration_valid": {"supported": true, "value": false, "valid": true, "fresh": true,
                      "updated_at": "2026-10-08T03:00:59.900+00:00",
                      "source": {"device": "erector", "method": "DEVICE_REPORTED"},
                      "unit": null, "error": null}
```

  - 이때 `response.state`는 `ONLINE`이다. `error` 문자열은 표시용이며 앱이 해석하지 않는다.
- `stall_or_direction_fault`는 일반 스톨(이동 중 위치 변화 없음)과 방향 이상(명령과 반대로 이동)을 구분하지 못한다.
  - 방향 이상은 Erector 전원을 다시 켤 때까지 해제되지 않는다.
  - 펌웨어가 둘을 구분하게 되면 코드를 **추가**하고, 기존 코드는 "둘 중 하나"라는 의미로 계속 낸다.
- states·faults·motion은 모두 같은 상태 응답에서 나온다. 응답이 오래되면 함께 `fresh=false`가 된다.

### 7.4 digital_counter

| 경로 | 타입 / 단위 | 채우는 기준 |
|---|---|---|
| response | 객체 / 단위 없음 | supported=true. 6절 응답 상태 공통 규칙(`ONLINE`/`OFFLINE`/`UNKNOWN`/`DISABLED`). 레지스터 맵 미설정이면 `UNKNOWN` |
| measurements.height_mm | number, `mm` | Modbus 응답값 |
| motion | `null`(미지원) / 단위 없음 | `supported=false`. 디지털카운터 자체의 이동 상태는 제공하지 않는다. |
| faults | `null`(미지원) / 단위 없음 | `supported=false`. 디지털카운터의 장치 고장 코드 목록은 제공하지 않는다. |

`configuration.complete=false`이면(레지스터 맵 미설정) 조회하지 않고 `height_mm`는 미수신 상태다.

### 7.5 bmu

| 경로 | 타입 / 단위 | 채우는 기준 |
|---|---|---|
| response | 객체 / 단위 없음 | supported=true. 6절 응답 상태 공통 규칙(`ONLINE`/`OFFLINE`/`UNKNOWN`/`DISABLED`). 주소 미설정이면 `UNKNOWN` |
| measurements.voltage_v | number, `V` | BMU 응답 |
| measurements.current_a | number, `A` | (+) 충전, (−) 방전 |
| measurements.soc_percent | number, `%` | |
| measurements.soh_percent | number, `%` | |
| measurements.temperature_c | number, `°C` | |
| measurements.remaining_capacity_ah | number, `Ah` | |
| measurements.remaining_energy_wh | number, `Wh` | |
| measurements.charge_complete_min | number, `min` | 충전 완료 예상 시간 |
| measurements.discharge_complete_min | number, `min` | 방전 완료 예상 시간 |
| measurements.cycle_count | number, `count` | |
| motion | `MOVING`/`STOPPED` / 단위 없음 | supported=false. |
| faults | [str] | 코드: `over_voltage`, `under_voltage`, `charge_over_current`, `discharge_over_current`, `over_temperature`, `under_temperature`, `bmu_error`, `fan_error` |

BMU 측정값은 모두 같은 응답에서 나오므로 같은 `updated_at`, `valid`, `fresh`를 가진다.

## 8. readiness 항목과 현재 조건

명령 접수에는 아래 조건 외에 **프로그램 모드, 유효한 제어권, 비상정지 미고정**이 항상 필요하다(readiness에는 비상정지만 반영).

| 장치 | capability | 대응 메서드 | available=false가 되는 현재 조건 |
|---|---|---|---|
| ptu | motion | up/down/left/right, move_to_angle, home | 경로 비활성·미READY(`not_ready`), 초기화 중(`device_initializing`), 위치 미수신·오래됨(`device_offline`) |
| ptu | speed | set_speed | 경로 비활성·미READY |
| ptu | stop | stop | 경로 비활성. 경로가 READY가 아니어도 STOP은 접수하고 전송을 시도한다 |
| mast | manual_motion | extend, retract | 경로 비활성·미READY, 마스트 주소 미설정(`capability_unavailable`). **높이·카운터 조건과 무관** |
| mast | target_motion | move_to_height, home | manual_motion 조건 + 카운터 높이 미수신·오래됨(`device_offline`), 하한·상한·허용오차 미설정(`capability_unavailable`) |
| mast | stop | stop | 경로 비활성, 주소 미설정 |
| erector | motion | fold/unfold/erect/stow/move_to/move_by | 경로 비활성·미READY, 주소 미설정, 상태 응답 오래됨(`device_offline`) |
| erector | speed | set_speed | 경로 비활성·미READY, 주소 미설정 |
| erector | stop | stop | 경로 비활성, 주소 미설정 |

- 비상정지 고정 중에는 모든 항목에 `emergency_stop_active`가 들어간다. 이때 장치별 `stop()`도 거부된다. `emergency_stop()`은 항상 호출할 수 있다.
- 목표 범위(예: 마스트 목표 높이가 운용 범위 밖)는 readiness가 아니라 명령 접수 시 `out_of_range`로 거부된다.
- RESET 준비는 별도 항목이 없다. 접수 시 단계별로 검사한다.

## 9. 최신 한계(현재 시험 초기값)

`fresh`는 표본 나이가 아래 한계 이내일 때만 참이다. 아래 값은 **현장 검증이 끝난 값이 아니라 현재 시험 초기값**이며, 운영 설정에서 바뀔 수 있다.

- `fresh=false`는 "최근 표본이 없다"는 뜻이다. 시간이 지났다는 사실만으로 장치 무응답 원인이나 실제 정지를 확정하지 않는다.
- 같은 한계가 응답 상태(`ONLINE`/`OFFLINE`, 6절) 판정에도 쓰인다.
- PTU는 pan·tilt를 차례로 조회한다. 그래서 두 축 표본 시각이 다르고, at_home·motion은 두 축 중 오래된 시각을 기준으로 한다.
- PTU·마스트 motion은 장치 보고가 아니라 위치 변화로 추정한다. PTU는 보낸 이동 명령과 축별 속도 정보도 함께 사용한다.

| 장치 | 조회 주기 | 최신 한계 |
|---|---|---|
| ptu | 0.1 s | 0.5 s |
| erector | 0.2 s | 1.0 s |
| digital_counter(마스트 높이) | 0.1 s | 0.5 s |
| bmu | 0.5 s | 2.0 s |

## 10. 버전과 호환

`schema_version`은 `major.minor.patch` 형식의 문자열이다. 이 계약은 `"1.0.0"`이다.

| 성분 | 올리는 경우 |
|---|---|
| major | 비호환 변경: 기존 필드의 구조·이름 변경, 삭제, 타입·단위·의미 변경 |
| minor | 이전 앱이 그대로 쓸 수 있는 필드·열거값·고장 코드 추가 |
| patch | 의미를 바꾸지 않는 호환 계약 수정 |

- 문서 오탈자 수정만으로는 버전을 올리지 않는다.
- 앱은 `.`으로 나눈 **숫자 성분**으로 비교한다(문자열 비교 금지).
- **major가 앱이 지원하는 값과 같으면** 해석한다. 다르면 상태를 해석하거나 제어 판단에 쓰지 않고 버전 불일치를 표시한다.
- minor·patch가 앱이 아는 값보다 커도 해석한다. 모르는 필드는 무시하고, 모르는 열거값·고장 코드는 원문을 표시하고 확인 불가로 처리한다(정상으로 단정하지 않음).
- 명령 결과 형식(`CommandReceipt`, `CommandResult`, `EmergencyStopResult`)은 [`02_api_reference.md`](02_api_reference.md)에서 설명한다.

