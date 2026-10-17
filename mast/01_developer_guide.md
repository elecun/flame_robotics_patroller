# 고객사 애플리케이션 개발자 가이드

- 이 문서는 고객사 앱 개발 기준입니다. 실행 가능한 패키지는 차주 방문하여 검증 후 제공 예정입니다.
- 명령·제어권·비상정지 API와 상태 조회 방법을 설명한다.
- 대상: `patrol-robot` Python 패키지(Python 3.10 이상). 패키지 버전은 배포 시 안내한다. 함께 보는 문서: [`02_api_reference.md`](02_api_reference.md)(상세 API 명세), [`03_status_contract.md`](03_status_contract.md)(상태 명세), [`examples/`](examples/)

## 1. 구성과 책임

고객사 앱 → Python Client → Patrol Robot Service(통합제어) → 장치 통신 → 장치·Erector 펌웨어

| 구성 | 역할 |
|---|---|
| 고객사 앱 | 화면, 장치 간 운전 순서와 연동 조건(예: Erector 전개 전 마스트·PTU 구동 금지), 작업 시나리오, 오류 안내 |
| Python Client | 서비스 요청·응답, 자동 heartbeat, Future 기반 비동기 API, 명령 결과 조회 |
| 통합제어 서비스 | 제어권·모드·명령 관리, 장치 통신 독점, 상태 제공, STOP 시도 |
| Erector 펌웨어 | 모터 구동, 위치 센서·소프트 리밋 보호, 상태 보고 |

앱은 장치에 직접 serial/TCP 명령을 쓰지 않는다. 장치 간 연동 조건은 앱이 관리하는 것을 전제로 한다.

## 2. 설치와 서비스 등록

- 같은 제어 PC에서 서비스와 앱을 실행한다. 기본 API 주소는 `tcp://127.0.0.1:5590`이다.
- wheel 설치와 서비스 등록은 별도 절차다. 아래는 운영 PC의 시스템 서비스 등록 예시이며, 재부팅 후 로그인하지 않아도 시작한다.
- 배포 파일: wheel, 의존 패키지 wheelhouse, 장비에 맞춘 설정 파일 `patrol_robot.yaml`, `patrol-robot.system.service` 템플릿을 함께 제공한다.
- 아래 `/path/to/delivery`는 전달 파일을 풀어 놓은 폴더로 바꾼다. 서비스 실행 계정은 설치하는 사용자 계정이며 sudo 권한이 필요하다.

### 2.1 프로그램과 설정 설치

```bash
sudo apt install -y python3-venv
mkdir -p "$HOME/patrol-robot"
python3 -m venv "$HOME/patrol-robot/.venv"
"$HOME/patrol-robot/.venv/bin/python" -m pip install --no-index --find-links /path/to/delivery/wheelhouse patrol-robot

mkdir -p "$HOME/.config/patrol-robot"
cp /path/to/delivery/patrol_robot.yaml "$HOME/.config/patrol-robot/patrol_robot.yaml"
"$HOME/patrol-robot/.venv/bin/patrol-robot-check-config" "$HOME/.config/patrol-robot/patrol_robot.yaml"
```

실제 제어 PC는 RS485 포트 2개(디지털카운터, PTU·마스트·Erector 공용 버스)와 RS232 포트 1개(BMU)를 사용한다. 설정 파일(`patrol_robot.yaml`)의 `transports.digital_counter_bus.device`, `transports.actuator_bus.device`, `transports.bmu.device`를 각 물리 포트 이름에 맞춘다. `/dev/ttyS0` 같은 이름과 번호는 실제 장비에서 확인해야한다.

설정 검사에 성공한 뒤 다음 단계로 진행한다. 설정 파일의 포트 이름, 장치 주소, 통신 조건과 운용 제한은 연결할 장비에 맞아야 한다. wheelhouse에는 `patrol-robot` wheel과 의존 패키지를 포함한다. `python3-venv`는 사전에 설치하거나 Ubuntu 설치 패키지로 준비한다.

### 2.2 서비스 파일 설정·등록

운영용 템플릿의 실행 계정·설치 경로·설정 경로를 채워 등록한다. 아래 예시는 홈 경로에 공백이나 특수문자가 없는 계정을 기준으로 한다.

```bash
sed -e "s|<SERVICE_USER>|$(id -un)|g" \
    -e "s|<INSTALL_DIR>|$HOME/patrol-robot|g" \
    -e "s|<CONFIG_FILE>|$HOME/.config/patrol-robot/patrol_robot.yaml|g" \
    /path/to/delivery/patrol-robot.system.service > "$HOME/patrol-robot/patrol-robot.service"

sudo install -m 0644 "$HOME/patrol-robot/patrol-robot.service" /etc/systemd/system/patrol-robot.service
sudo systemctl daemon-reload
sudo systemctl enable --now patrol-robot.service
```

`enable --now`는 부팅 시 자동 실행을 등록하고 즉시 서비스를 시작한다. venv의 실행 파일 경로를 직접 사용하므로 `activate`는 필요 없다. 직렬 포트는 서비스 계정의 `dialout` 권한으로 접근한다. 사용자 서비스와 시스템 서비스가 동시에 실행되지 않도록 한 방식만 등록해야한다.

### 2.3 상태 확인·재시작·종료

```bash
sudo systemctl status patrol-robot.service
sudo journalctl -u patrol-robot.service -f -o cat

# 설정 파일을 변경한 뒤 적용
sudo systemctl restart patrol-robot.service

# 서비스 종료
sudo systemctl stop patrol-robot.service
```

서비스 파일 자체를 바꾼 경우에는 `sudo systemctl daemon-reload` 후 재시작한다. 서비스가 종료되어도 부팅 시 자동 실행 등록은 유지되며, 이를 해제하려면 `sudo systemctl disable --now patrol-robot.service`를 실행한다.

## 3. 첫 연결: 움직임 없이 상태 확인

```python
from patrol_robot_client import PatrolRobotClient

with PatrolRobotClient() as robot:          # 생성 시 서비스 응답을 확인(무응답이면 기본 약 6~7초 후 예외)
    status = robot.get_status(timeout=5)
    major = int(status["schema_version"].split(".")[0])
    if major != 1:                          # 지원하는 상태 형식(major 1)인지 확인(11절)
        raise RuntimeError(f"지원하지 않는 상태 형식: {status['schema_version']}")
    print(status["system"]["control_state"])
    height = status["devices"]["mast"]["measurements"]["height_mm"]
    if height["supported"] and height["valid"] and height["fresh"]:
        print("마스트 높이", height["value"], height["unit"])
    else:
        print("마스트 높이 확인 불가:", height["error"])
```

- 서비스가 응답하지 않으면 생성자가 `PatrolRobotServiceUnavailableError`를 낸다. `connect_timeout`(기본 3초)은 연결 확인 요청 한 번의 대기 한도이며 1회 재전송을 포함하면 약 2배가 걸린다. 정확한 대기 시간과 예외는 [`02_api_reference.md`](02_api_reference.md) 2.1절.
- 상태 조회에는 제어권이 필요 없다.
- GUI 앱은 Client 생성과 종료를 화면 스레드 밖에서 한다(8절).

## 4. 상태 읽기 규칙(요약)

상세 계약은 `03_status_contract.md`에 있다.

1. 관측 객체는 `supported == true && valid && fresh`일 때만 제어 판단에 쓴다.
   - `valid`는 값의 유효성, `fresh`는 표본의 최신성이다. 서로 독립이므로 **`fresh`만 보고 값을 쓰지 않는다.**
2. `valid=true, fresh=false`는 마지막 정상값이다. "오래된 값"으로 표시하고 판단에는 쓰지 않는다. `valid=false, fresh=true`는 장치가 최근 응답했지만 값이 무효라는 뜻이다(`value=null`, `error`에 사유).
3. `value=null`, `valid=false`, `state=UNKNOWN`/`UNSUPPORTED`는 "확인 불가"다. 0, 정지, 정상으로 바꾸지 않는다.
4. `faults.value`가 빈 목록이어도 `valid && fresh`가 아니면 "고장 없음"이 아니다.
5. 운전 준비는 `devices.<role>.readiness.<기능>.available`과 `reasons`로 표시한다. 제어 모드·제어권은 `system`과 `robot.has_control()`로 따로 확인한다.
6. 모르는 키는 무시하고, 모르는 열거값·고장 코드는 원문으로 표시한다.
7. 시각(`updated_at`)은 UTC다. 화면에서 현지 시각으로 바꾼다.

장치별 주의:
- **마스트**
  - 응답 기능이 없다(`response.state = "UNSUPPORTED"`).
  - 높이는 디지털카운터 값이다. 디지털카운터는 절대위치 센서가 아니므로 전원 투입 시 표시값과 실제 높이가 맞는지 운용 절차로 확인한다.
  - 정지는 높이 변화로 추정한다.
- **PTU**: 정지는 두 축 위치 변화로 추정하며 장치가 정지를 보고하지 않는다. 정지로 판단된 시간은 실제 정지한 시간과 차이가 있다.
- **Erector**
  - `position_mm.valid=false`이고 `calibration_valid`가 최신 `false`이면 보정 무효다. `calibration_valid=true`인데 위치가 무효이면 센서 등 다른 원인이므로 `error`를 표시한다.
  - 위치 무효를 보고한 최신 응답은 `position_mm`이 `value=null, valid=false, fresh=true`이고 `updated_at`이 그 응답 시각이다. 응답 상태는 `ONLINE`이다.
  - `stall_or_direction_fault`는 스톨과 방향 이상을 구분하지 못한다. 방향 이상은 Erector 전원을 다시 켤 때까지 남는다.
- **BMU**: 명령 API는 없다. `devices.bmu`의 측정·고장만 조회한다.
- **응답 상태(`response.state`)**
  - Erector·디지털카운터·BMU: 현재 연결에서 받은 정상 응답이 최신이면 `ONLINE`, 응답 이력은 있지만 최신 한계를 넘었거나 재연결 직후면 `OFFLINE`, 응답 이력이 없거나 기본 설정이 빠졌으면 `UNKNOWN`, 비활성이면 `DISABLED`다.
  - PTU: pan·tilt 두 축 위치가 모두 최신이어야 `ONLINE`이다. 첫 위치 응답 전에는 `INITIALIZING`이고, 초기화 제한시간을 넘으면 `OFFLINE`이다.
  - 조회 한 번 실패로 바로 `OFFLINE`이 되지는 않는다. `ONLINE`은 장치가 응답한다는 뜻이며 측정값이 유효하다는 뜻은 아니다.
  - 상세 조건과 우선순위는 `03_status_contract.md` 6절.

예시 JSON: `examples/status_normal.json`(정상), `status_link_lost.json`(통신 단절), `status_unconfigured.json`(설정 미완성), `status_disabled_unsupported.json`(비활성·미지원).

## 5. 프로그램 운전 순서

1. Client 생성
2. `acquire_control()` 결과의 `granted` 확인
3. `set_control_mode(ControlMode.PROGRAM)` 후 상태의 `system.control_mode == "PROGRAM"` 확인
4. 필요한 장치의 readiness와 측정값, 앱의 연동 조건 확인
5. 명령 호출 후 `CommandReceipt.accepted` 확인
6. 작업 명령이면 `wait_command()`로 최종 결과 확인
7. 종료 시 `release_control()` 결과를 확인하고 `close()` 호출

제어권 획득만으로 프로그램 모드가 되지는 않는다. `has_control()`은 Client가 토큰을 보유하는지만 알려준다.

아래 예제는 Erector를 실제로 움직인다. 주변 기구 상태를 확인한 뒤 시험한다.

```python
from patrol_robot_client import PatrolRobotClient
from patrol_robot_client.errors import CommandWaitTimeoutError
from patrol_robot_common import CommandState, ControlMode

with PatrolRobotClient() as robot:
    control = robot.acquire_control(timeout=5)
    if not control["granted"]:
        raise RuntimeError(f"제어권 획득 거부: {control}")

    mode = robot.set_control_mode(ControlMode.PROGRAM, timeout=5)
    status = robot.get_status(timeout=5)
    if status["system"]["control_mode"] != "PROGRAM":
        raise RuntimeError(f"프로그램 모드 전환 실패: {mode}")

    ready = status["devices"]["erector"]["readiness"]["motion"]
    if not ready["available"]:
        raise RuntimeError(f"Erector 운전 불가: {ready['reasons']}")

    receipt = robot.erector.erect(timeout=5)
    if not receipt.accepted:
        print("명령 거부:", receipt.reason, receipt.message)
    else:
        try:
            result = robot.wait_command(receipt.command_id, timeout=60)
            if result.state is CommandState.COMPLETED:
                print("Erect 도착 완료")
            else:
                print("명령 종료(성공 아님):", result.to_dict())
        except CommandWaitTimeoutError as exc:
            # 기다림 만료는 장치 정지를 뜻하지 않는다.
            print("완료 대기 만료, 마지막 결과:", exc.last_result)
            robot.erector.stop(timeout=5)
    robot.release_control(timeout=5)
```

- 60초는 앱의 대기 한도이며 장치 운전 시간 제한이 아니다.
- `with` 블록을 빠져나가면 제어권 반환과 연결 종료를 시도한다. 반환 결과를 화면에 보여야 하면 위처럼 `release_control()`을 직접 호출한다.

## 6. 명령 접수와 완료

명령 사용은 **접수 결과 확인**과 **명령 처리 상태·완료 확인**의 두 단계로 나뉜다. 각 단계의 동기·비동기 방식은 별도로 선택할 수 있다.

1. **접수 결과 확인**: 서비스가 명령을 접수했는지 확인한다. `erect()`는 접수 응답을 기다린 뒤 `CommandReceipt`를 반환하고, `erect_async()`는 즉시 Future를 반환한다. Future에서 접수 결과를 확인할 수 있다. 둘 다 실제 동작 완료까지 기다리지는 않는다.
2. **명령 처리 상태·완료 확인**: 접수된 명령의 `command_id`로 확인한다. `wait_command()`는 명령이 종료될 때까지 기다린 뒤 결과를 반환하고, `wait_command_async()`는 종료 결과를 받을 Future를 즉시 반환한다. `get_command_async()`는 호출 시점의 처리 상태를 한 번 조회하며, 같은 Future의 결과가 계속 갱신되지는 않는다. 명령 처리 완료와 실제 장치 동작의 관계는 아래 명령 종류별 완료 조건을 참고한다.

이 가이드의 운전 예제는 주로 동기 메서드를 사용한다. GUI에서 사용하는 비동기 방식은 8장을 참고한다. 필드·열거값·완료 조건의 전체 정의는 [`02_api_reference.md`](02_api_reference.md) 4절에 있다.

| CommandReceipt | 의미 |
|---|---|
| accepted | 접수 여부 |
| command_id | 접수된 명령 ID(거부 시 None) |
| reason | 거부 사유(`RejectReason`, 접수 시 None) |
| message | 보조 설명 |

`CommandState`는 **서비스의 명령 처리 진행 상태**다. 장치가 실제로 움직이거나 멈췄다는 보장이 아니다.

| CommandState | 의미 |
|---|---|
| ACCEPTED / RUNNING | 접수 / 처리 중 |
| COMPLETED | 그 명령의 처리 완료 조건 충족(**성공은 이것뿐**) |
| STOPPED | 정지로 종료 |
| LIMIT_REACHED | 리밋 조건으로 종료 |
| CANCELLED | 취소됨 |
| FAILED | 실행 실패 |

**명령 종류별 결과 확인**

| 종류 | 메서드 | 결과 확인 |
|---|---|---|
| 작업 명령 | ptu.home, ptu.move_to_angle, mast.home, mast.move_to_height, erector.erect/stow/move_to/move_by, reset | `wait_command()`로 최종 상태 확인. COMPLETED는 서비스가 목표 도착을 확인했다는 뜻 |
| 연속 이동 | ptu.up/down/left/right, mast.extend/retract, erector.fold/unfold | 서비스 송신 처리가 끝나면 COMPLETED. 실제 이동은 상태(motion, 위치)로 확인 |
| 정지·설정 | 각 장치 stop, set_speed | 서비스 처리가 끝나면 COMPLETED. 실제 정지는 상태로 확인. PTU set_speed는 장치로 보내지 않고 다음 방향 이동부터 적용 |

- 연속 이동·정지·속도 설정의 COMPLETED는 **서비스 처리 완료**만 뜻한다. 장치가 움직이기 시작했거나 멈췄다는 보장이 아니다. 이후 STOP이 와도 그 명령 기록은 COMPLETED로 남는다.
- `wait_command(command_id, timeout)`
  - 지정한 `timeout` 동안 명령 상태를 조회하고, 종료 상태가 오면 결과를 반환한다.
  - 끝나지 않으면 `CommandWaitTimeoutError`(`last_result`에 마지막 결과)가 난다.
  - 마감 직전에 보낸 조회의 응답까지 기다리므로 실제 종료는 `timeout`보다 늦을 수 있다(정상 진행 시 대략 +2 × `request_timeout`). 그 응답이 종료 상태면 마감 뒤에도 정상 결과를 반환한다.
  - 동기 호출은 `timeout + 3 × request_timeout` 동안 기다리며, 이를 넘으면 `concurrent.futures.TimeoutError`가 난다. 3배는 재시도 횟수가 아니고 보장된 실시간 상한도 아니다.
  - **대기 만료만으로 서비스가 명령을 STOP하거나 취소하지 않는다.**
  - 상세는 [`02_api_reference.md`](02_api_reference.md) 4.3절.
- Client는 응답이 없으면 같은 `request_id`로 한 번 다시 보낸다. 서비스는 같은 요청을 다시 실행하지 않는다. **응답을 못 받았다는 이유로 이동 메서드를 다시 호출하지 않는다.** 새 호출은 새 명령이다.

**작업 명령의 완료 기준**
- 마스트 목표 이동
  - 상승은 목표 −선행 거리, 하강은 목표 +선행 거리에서 STOP을 보낸다(현재 설정 50mm).
  - 정지 후 높이가 목표 허용오차 안이면 COMPLETED다.
  - 시작할 때 이미 목표 허용오차 안이면 움직이지 않고 바로 COMPLETED다.
  - 그렇지 않고 남은 거리가 선행 STOP 거리 이하이면 거부한다(`out_of_range`).
  - 50mm는 현재 설정의 선행 STOP 거리(`upward_stop_lead_mm`, `downward_stop_lead_mm`)다.
- 마스트 Home: 하한+허용오차까지 하강한 뒤 STOP한다(목표 이동과 다른 경로).
- Erector Erect/Stow: 펌웨어 정지 보고와 Erect/Home 도착 보고로 판정한다.
- Erector move_to/move_by
  - 펌웨어 정지 보고와 서비스 허용오차(현재 임시 ±5mm)로 판정한다.
  - stall이면 FAILED, 목표 밖에서 소프트 리밋이면 LIMIT_REACHED, 그 밖의 목표 밖 정지는 FAILED다.
  - 이미 목표에서 정지해 있으면 바로 COMPLETED다.
- RESET
  - Mast Home → PTU Home → Erector Stow를 차례로 실행하고, 이미 Home인 단계는 건너뛴다.
  - 한 단계가 실패하면 이후 단계로 진행하지 않는다.
  - 실행 중에 다시 요청하면 같은 `command_id`를 돌려준다.

## 7. 공개 API

**전체 메서드의 시그니처, 인자 범위, 반환값, 예외, 권한, timeout·재시도 규칙은 [`02_api_reference.md`](02_api_reference.md)에 있다.** 이 장은 요약이다.

대부분의 메서드에 `*_async()`가 있으며 즉시 `concurrent.futures.Future`를 반환한다. 동기 메서드의 `timeout`은 **호출 스레드의 대기 한도**다. 초과하면 `concurrent.futures.TimeoutError`가 나며 요청은 계속 처리될 수 있다.

### 공통(`robot`)

| 메서드 | 결과 |
|---|---|
| get_status() / get_status_async() | 상태 dict(공개 상태 API 명세 참고) |
| acquire_control() / release_control() | dict(`granted`, `lease_token`, `reason` 등) |
| has_control() / get_control_status() | Client 내부 제어권 정보 |
| on_control_lost(callback) | 제어권 상실 시 사유 문자열과 함께 호출(Client 스레드) |
| set_control_mode(ControlMode.PROGRAM / MANUAL) | 모드 전환 결과 dict |
| get_command_async(command_id) | CommandResult 또는 None(동기 메서드 없음) |
| wait_command(command_id, timeout) | 최종 CommandResult |
| emergency_stop() | EmergencyStopResult |
| release_emergency_stop() | 해제 결과 dict |
| reset() | CommandReceipt |
| close(timeout=3.0) | 제어권 반환 시도 후 종료 |

### PTU(`robot.ptu`)

| 메서드 | 인자·동작 |
|---|---|
| up(), down(), left(), right() / move_async(direction) | 연속 이동. direction은 "UP", "DOWN", "LEFT", "RIGHT". `move_async`는 비동기 전용(동기 `move()` 없음) |
| stop() | 정지 |
| home() | Home 이동(작업 명령) |
| move_to_angle(pan_deg, tilt_deg) | 각도 목표 이동(작업 명령) |
| set_speed(speed_code) | 정수 0x00~0x3F(0x00 정지, 0xFF 터보 미지원) |

Pan에는 확인된 하드웨어 이동 제한이 없다.

### 마스트(`robot.mast`)

| 메서드 | 인자·동작 |
|---|---|
| extend(), retract() | 연속 상승·하강. **높이 상·하한과 카운터 상태를 검사하지 않는다**(자동 리밋 보호 아님) |
| stop() | 정지 |
| home() | 하한 복귀(작업 명령) |
| move_to_height(height_mm) | 목표 높이 이동(작업 명령). 운용 범위 밖이면 `out_of_range` |

목표 이동·Home에는 디지털카운터 높이(valid·fresh)와 운용 하한·상한·허용오차 설정이 필요하다(readiness `target_motion`). 고객사 확인 운용 범위는 2900~9100mm다.

### Erector(`robot.erector`)

| 메서드 | 인자·동작 |
|---|---|
| fold(), unfold() | 연속 접기·펴기(STOP 또는 펌웨어 리밋까지) |
| erect(), stow() | 펌웨어 설정 위치로 이동(작업 명령) |
| stop() | 정지 |
| move_to(position_mm) | 절대 위치 이동(작업 명령) |
| move_by(delta_mm) | 상대 이동(작업 명령, 접수 시 위치 기준 절대 목표로 변환) |
| set_speed(speed_code) | 정수 0~255(0이면 구동 출력 0) |

## 8. GUI에서 비동기 사용

GUI 이벤트 스레드에서 동기 API나 `Future.result()`로 기다리면 화면이 멈춘다.

Future callback이 실행되는 스레드는 정해져 있지 않다. 응답 처리 중 완료되면 주로 Client 내부 스레드에서 실행되지만, 등록할 때 Future가 이미 완료돼 있으면 등록한 스레드에서 즉시 실행될 수 있다. 따라서 callback에서는 widget을 직접 바꾸지 말고, **GUI 갱신은 항상 GUI 스레드로 전달**한다(아래 queue 예시). 

별도로, 제어권 상실 알림을 등록하는 `on_control_lost(callback)`은 Future callback과 달리 항상 Client 내부 스레드에서 실행된다. 이 callback에서도 GUI 갱신은 GUI 스레드로 전달해야 한다.

```python
import queue

updates = queue.Queue()

def on_status(future):
    try:
        updates.put(("status", future.result()))
    except Exception as exc:
        updates.put(("error", repr(exc)))

robot.get_status_async().add_done_callback(on_status)
# GUI 타이머(Tkinter after, PySide QTimer)에서 updates를 꺼내 화면을 갱신한다.
```

이전 조회가 끝난 뒤 다음 조회를 예약해 요청이 쌓이지 않게 한다. 상태 이벤트 구독 공개 API는 없으므로 주기 조회를 쓴다.

## 9. 제어권·모드·장애 처리

- 프로그램 제어권은 한 Client만 가진다. Client가 heartbeat를 자동 전송하며(기본 1초), 서비스는 설정 시간(기본 3초) 동안 heartbeat가 없으면 제어권을 만료한다.
- 프로그램 모드에서 제어권이 반환되거나 만료되면 해당 제어권의 작업을 취소하고, 세 장치에 STOP을 시도한 뒤 수동 잠금(`MANUAL_LOCKED`)으로 전환한다. 수동 모드에서 만료되면 수동 동작을 유지한다.
- 수동 모드에서는 앱의 운전 명령이 `wrong_mode`로 거부된다.
- 현장 리모콘 STOP을 1초 안에 두 번 누르면 수동 READY가 된다.
  - **주의:** PTU 버튼을 놓을 때 오는 STOP도 수동 활성화 입력으로 계산될 수 있으므로, 버튼을 놓은 뒤 수동 활성화 상태를 확인해야 한다.
- 서비스 재시작이나 제어권 만료 뒤에는 상태를 조회하고 제어권을 다시 얻어 프로그램 모드를 선택한다. 중단된 이동은 자동으로 재개되지 않는다.

| 예외 / 거부 사유 | 앱 대응 |
|---|---|
| PatrolRobotServiceUnavailableError | 서비스 실행·주소 확인 |
| concurrent.futures.TimeoutError | 호출 대기 만료. 요청이 계속 처리될 수 있으므로 상태 확인 |
| RequestTimeoutError | Client 내부 요청 만료(1회 재시도 후). 실행 여부 불명, 상태 조회 후 판단. 이동 재호출 금지 |
| CommandWaitTimeoutError | 완료 대기 만료. 대기 시간 만료만으로 서비스가 명령을 자동으로 STOP하거나 취소하지는 않음 |
| ProtocolVersionError | Client·서비스 버전 확인 |
| UnknownCommandError | 서비스 재시작 등으로 명령 기록이 없음 |
| TypeError / ValueError | 인자 수정 |
| ResponseFormatError / ServiceRequestError | 오류 정보 보존, 로그·버전 확인 |
| wrong_mode / control_not_acquired / control_already_acquired / control_lost | 모드·제어권 확인 |
| emergency_stop_active | 비상정지 원인 확인 후 해제 |
| device_offline / device_initializing / not_ready / capability_unavailable | 통신·초기화·설정·피드백 확인(readiness의 reasons와 같은 문자열) |
| out_of_range / limit_reached | 목표값과 운용 제한 확인 |
| nport_link_unavailable / manual_locked | NPort 연결·수동 활성화 확인 |
| queue_full | 요청 과다. 간격을 두고 재시도 |

## 10. 정지와 안전 한계

- 장치별 `stop()`은 일반 운전 API로, 프로그램 모드와 제어권이 필요하다. 통신 경로가 READY가 아니어도 STOP 전송은 시도한다.
- `emergency_stop()`은 제어권·모드와 관계없이 세 장치에 STOP을 각각 시도하고 새 동작을 막는다. 다시 호출하면 비상정지 고정을 유지하면서 세 장치에 STOP을 다시 전송한다.
  - 결과의 `latched=True`는 서비스 차단 상태다.
  - 반환값 `EmergencyStopResult`에 있는 장치별(`ptu`, `mast`, `erector`) `tx_result`와 실패 사유(`reason`, `error_code`, `error_message`)를 확인해야 한다. 예: `result.mast.tx_result`. `TX_SUCCESS`는 로컬 송신 성공이며 장치 정지 확인이 아니다. 실제 정지는 이후 상태 조회의 장치 동작 상태로 확인해야 한다.
- 비상정지 고정 중에는 장치별 `stop()`도 거부된다. `release_emergency_stop()`은 고정만 풀고 이전 동작을 재개하지 않는다.
- 물리 비상정지 스위치가 앱으로 들어오는 구성이면, 앱이 스위치를 감지해 `emergency_stop()`을 호출해야 한다.
- 서비스·제어 PC가 멈추거나 장치 통신이 끊기면 새 STOP을 전달할 수 없다. 장치 자체 보호가 동작하기 전까지 움직임이 계속될 수 있다.
- Erector 하한(Home·Fold 방향) 보호는 펌웨어 위치 센서·소프트웨어에 의존한다. Unfold 방향은 하드리밋이 모터 전원을 직접 차단한다.

## 11. 상태 형식 버전과 향후 변경

- `schema_version`은 상태 응답의 구조・타입・의미를 식별하는 문자열이며 `major.minor.patch` 형식이다. 최초 공개 버전은 `"1.0.0"`이다.
  - **major**: 기존 필드의 이름 변경・삭제・타입 변경・단위 변경・의미 변경처럼 앱의 해석 방식을 바꿔야 하는 비호환 변경. 예: `height_mm` 이름 변경, 숫자를 객체로 변경, mm를 cm로 변경.
  - **minor**: 이전 앱이 그대로 쓸 수 있는 필드・열거값・고장 코드 추가.
  - **patch**: 의미를 바꾸지 않는 호환 계약 수정.
  - 문서 오탈자 수정만으로는 버전을 올리지 않는다.
- 앱은 버전을 `.`으로 나눈 **숫자 성분**으로 비교한다(문자열 비교 금지). **major가 앱이 지원하는 값과 같으면** 해석하고, 다르면 상태를 해석하거나 제어 판단에 쓰지 않고 버전 불일치를 표시한다.
- 앱은 **모르는 필드를 무시**해야 한다. 추가 필드가 있다는 이유로 응답 전체를 오류 처리하지 않는다.
- 모르는 열거값・고장 코드는 원문을 표시하고 확인 불가로 처리한다. 정상이나 정지로 단정하지 않는다.
