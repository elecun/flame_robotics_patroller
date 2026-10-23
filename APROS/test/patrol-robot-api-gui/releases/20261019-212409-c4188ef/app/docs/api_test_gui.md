# API 시험 GUI 사용법

`tools/api_test_gui.py`는 리모콘 없이 Patrol Robot Service의 공개 Python API(`PatrolRobotClient`)를
사람이 직접 눌러 시험하는 tkinter 도구다. 서비스·Client·설정은 바꾸지 않고 공개 API만 호출한다.

> 실제 장비에 연결된 서비스에 접속하면 버튼이 실제 장치를 움직인다. 시험 전에 주변 안전을 확인한다.

## 실행

서비스가 이미 실행 중이어야 한다. GUI는 서비스를 시작·재시작하거나 설정을 바꾸지 않는다.
실행 방법은 용도에 따라 두 가지다.

### 배포 설치본(다른 PC 운영·현장 시험)

배포 압축본을 만들어 대상 PC에 설치하고 `run_gui.sh`로 실행한다. 서비스는 들어 있지 않다.

```powershell
# Windows 개발 PC: 커밋된 GUI와 Client/common으로 압축본 생성(기본 출력 dist/api-gui)
powershell -File tools/api_gui_release/build_release.ps1 [-Ref HEAD] [-ClientRef <ref>] [-OutputDir <폴더>]
```

```bash
# 대상 Linux PC: 확인·설치·실행(상세는 압축본 안 INSTALL.md)
sha256sum -c patrol-robot-api-gui_<id>.tar.gz.sha256
tar -xzf patrol-robot-api-gui_<id>.tar.gz
bash patrol-robot-api-gui_<id>/install.sh [--install-dir DIR] [--activate]
~/patrol-robot-api-gui/run_gui.sh [--endpoint tcp://127.0.0.1:5590]
~/patrol-robot-api-gui/manage_releases.sh list|activate <id>|rollback|remove <id>
```

- builder는 선택한 commit의 파일만 담는다. HEAD로 빌드할 때 배포 대상에 커밋되지 않은 변경이 있으면
  빌드하지 않는다.
- 설치는 버전별 폴더(`releases/<id>`)와 전용 venv를 만들고 화면 없이(SSH) 진행할 수 있다. 첫 설치만
  자동 활성이고, 이후 버전은 `--activate` 또는 `manage_releases.sh activate`로 전환한다.
- 설치는 GUI 창을 열지 않고 서비스에 연결하지 않는다.

### 개발 실행(저장소 editable install, 개발·내부 시험용)

```bash
cd ~/patrol-robot-api-gui   # GUI 시험용 복사본(운영 경로 ~/patrol-robot와 별개)
.venv/bin/python tools/api_test_gui.py                       # 기본 tcp://127.0.0.1:5590
.venv/bin/python tools/api_test_gui.py --endpoint tcp://127.0.0.1:5691
```

필요 패키지는 Python 표준 `tkinter`(Ubuntu `python3-tk`)와 이 저장소의 editable install이다.

두 방법 모두 화면이 있는 세션(로컬 데스크톱 또는 X 전달)에서 실행한다.

## 시작 동작

- 창을 열어도 연결하지 않는다. `연결`을 눌러야 접속하고, 접속 직후에는 상태 조회 한 번만 보낸다.
- 제어권 획득, 모드 변경, 이동·Home·RESET은 버튼을 누를 때만 보낸다.
- 창을 닫거나 `종료`를 누르면 `PatrolRobotClient.close()`를 호출한다. 제어권이 있으면 Client가 반환을
  한 번 시도하고, 실패해도 서비스의 heartbeat 만료가 같은 절차를 수행한다.

## 현장 사용 순서

1. `연결` → 오른쪽 상태 대시보드가 1초마다 갱신되는지 확인(요약은 창 제목에도 표시)
2. `제어권 획득` → `프로그램 모드`
3. 장치 명령 시험(로그에서 접수/거부 사유 확인), STOP·비상정지·해제 확인
4. `제어권 반환` 또는 `수동 모드` → `종료`

## 화면 구성

| 영역 | 기능 | 호출 API |
|---|---|---|
| 상단 | Endpoint, 연결/종료 | `PatrolRobotClient(endpoint)`, `close()` |
| 빨간 띠(항상 표시) | 비상정지, 비상정지 해제, PTU/Mast/Erector STOP, 3장치 STOP | `emergency_stop_async`, `release_emergency_stop_async`, `<device>.stop_async` |
| 제어 줄 | 제어권 획득/반환, 프로그램/수동 모드, 전체 RESET(확인 대화상자) | `acquire_control_async`, `release_control_async`, `set_control_mode_async`, `reset_async` |
| PTU | 상·하·좌·우, STOP, Home, 절대 각도(pan/tilt deg), 속도 코드 | `ptu.up_async` … `ptu.move_to_angle_async`, `ptu.set_speed_async` |
| Mast | Extend, Retract, STOP, Home | `mast.extend_async`, `retract_async`, `stop_async`, `home_async` |
| Erector | Fold, Unfold, Erect, Stow, STOP, 절대/상대 위치(mm), 속도 코드 | `erector.*_async`, `move_to_async`, `move_by_async`, `set_speed_async` |
| 명령 조회 | 접수된 command_id 선택·입력, 상태 조회, 완료 대기(초), 접수 시 자동 완료 대기 | `get_command_async`, `wait_command_async` |
| 오른쪽 상태 | 새로고침, 주기 조회(간격 초, 최소 0.2), 고정 필드 대시보드(장치별 마지막 명령 포함), `원문 JSON 보기`(기본 접힘) | `get_status_async`, `get_command_async` |
| 왼쪽 아래 로그 | 요청·접수·거부 사유·command_id·예외·완료 결과를 시간순으로 표시 | — |

`3장치 STOP`은 PTU, Mast, Erector STOP을 각각 보내며 한 장치 호출이 실패해도 나머지를 시도한다. 장치
STOP은 일반 명령이라 제어권과 프로그램 모드가 필요하다. 제어권과 무관한 정지는 `비상정지`를 사용한다.

## 상태 대시보드(공개 상태 schema 1.x)

고객사 상태 계약(`docs/customer_api/03_status_contract.md`, `schema_version` 1.0.0·1.1.0)을 해석한다. 고정
필드의 값과 색만 제자리에서 갱신하고, 색만으로 판단하지 않도록 값 텍스트를 함께 표시한다.

- 시스템: schema, mode, state, program_ctl, manual_auth, health, estop_latched, 진행 작업(`active_commands` 목록)
- 링크·리모콘: `digital_counter_bus`, `actuator_bus`, `bmu`, `nport`, `mast_bus(마스트 전용)`(설정에서 끈
  경로는 키가 없어 "비활성(설정 꺼짐)"), `remote.nport_link`, `remote.activity`
  - `mast_bus`는 schema 1.1.0에서 추가된 마스트 전용 NPort Serial2 TCP 경로다. 1.0.0 서비스에는 키가 없어
    "비활성(설정 꺼짐)"으로 보이고, 마스트는 `actuator_bus`를 쓴다.
  - 링크 `READY`는 포트·소켓이 열려 있고 초기화가 끝났다는 뜻일 뿐 장치 응답이 아니다. `mast_bus` READY도
    실제 마스트의 수신·정지·연결을 보장하지 않는다. 장치 응답은 각 장치의 `응답`으로 본다.
- 장치별: 응답(`response.state`, 설정 누락은 `configuration.missing`으로 함께 표시), 측정값·판정 상태·motion·faults
  (Observation), 준비(`readiness`, 같은 결과의 항목은 묶어 표시. 호출자 권한은 포함하지 않음)
  - PTU: pan/tilt, at_home, motion, speed_code(설정값), 준비
  - Mast: 경로(`devices.mast.link`로 찾은 링크 상태, 예: `mast_bus: READY`), height_mm(디지털카운터 보고),
    at_home, motion, 준비. 응답 기능이 없어 응답은 `UNSUPPORTED`이며 경로가 READY여도 ONLINE으로 바꾸지 않는다.
    준비는 서비스가 경로와 카운터 조건을 조합해 반환한 값을 그대로 표시한다.
  - Erector: position_mm, at_home, at_erect, calibration, soft_limit, motion, faults, 준비
  - Digital Counter: 응답, height_mm
  - BMU: 응답, 전압·전류·SOC·SOH·온도, faults
- 상태 표가 창 높이보다 길면 오른쪽 스크롤바나 마우스 휠로 아래 상자를 본다.

### 장치별 마지막 명령

PTU·Mast·Erector 상자 하단에 이 GUI가 그 장치에 보낸 마지막 명령을 표시한다. 다른 Client나 리모콘이 보낸
명령은 표시하지 않는다. 디지털카운터·BMU에는 제어 명령이 없어 표시하지 않는다.

| 항목 | 내용 |
|---|---|
| 마지막 명령 | 명령명과 실제 인자(예: `절대 각도 이동 pan 30° tilt -10.5°`, `절대 위치 이동 120mm`, `속도 설정 0x10`, `STOP`) |
| 명령 상태 | 아래 표 |
| 요청 시각 | GUI가 요청을 시작한 로컬 시각(HH:MM:SS) |
| 사유 | 거부 사유·결과 메시지·오류 내용. 없으면 `-` |

| 명령 상태 | 뜻 |
|---|---|
| `요청 중(서비스 응답 대기)` | GUI가 요청을 보냈고 서비스 응답을 기다리는 중 |
| `접수됨 (accepted)` | 서비스가 접수(`CommandReceipt.accepted`) |
| `실행 중 (running)`, `완료 (completed)`, `중지 (stopped)`, `리밋 도달 (limit_reached)`, `취소 (cancelled)`, `실패 (failed)` | 서비스가 반환한 `CommandResult.state` |
| `거부` | 서비스가 접수를 거부. 사유에 `reason: message` |
| `호출 오류` | Client 호출 자체가 실패(요청을 보내지 못함) |
| `확인 불가` | 응답 오류, 결과 조회 실패, 서비스가 command_id를 모름(서비스 재시작 등) |
| `확인 불가(연결 해제)` | 결과를 받기 전에 연결을 끊음 |
| `확인 불가(서비스 재시작)` | 결과를 받기 전에 서비스가 재시작됨(`service_instance_id` 변경) |
| `… — 이전 연결 결과`, `… — 이전 서비스 결과` | 연결 해제·서비스 재시작 전에 받은 최종 결과(회색) |

- 입력 오류로 호출하지 않은 명령은 기록하지 않는다. `3장치 STOP`은 장치별 STOP 결과를 각 상자에 따로 표시하며,
  비상정지·RESET 같은 전역 명령 결과는 장치 상자에 표시하지 않는다.
- 접수 뒤에는 상태 조회 응답이 올 때마다 아직 끝나지 않은 마지막 명령의 결과를 장치마다 한 번 조회한다
  (`get_command_async`). 응답 전에는 다시 묻지 않고, 최종 상태·거부·확인 불가가 되면 조회를 끝낸다. 주기 조회를
  끄면 `상태 새로고침`을 누를 때만 갱신한다. 조회는 결과 확인만 하며 명령을 다시 보내지 않는다.
- 같은 command_id를 `완료 대기` 중이면 주기 조회로 묻지 않고 완료 대기 결과로 갱신한다. `상태 조회`·`완료 대기`
  결과도 그 장치의 마지막 명령과 command_id가 같으면 표시에 반영한다.
- 같은 장치에 새 명령을 보내면 이전 명령의 늦은 응답은 표시에 쓰지 않는다. 최종 상태를 받은 뒤 늦게 온 이전
  조회 응답·오류와, `실행 중`을 받은 뒤 늦게 온 `accepted`(완료 대기 시간 초과의 마지막 상태 포함)도 쓰지 않는다.
  다시 연결하면 이전 연결의 응답을 현재 결과로 쓰지 않는다.
- 연결을 유지한 채 서비스가 재시작되면 Client가 응답에서 갱신한 공개 `service_instance_id`(`get_control_status()`)
  변화로 알아채고, 로그에 `서비스 재시작 감지`를 남긴 뒤 이전 서비스의 마지막 명령을 위 표처럼 분리한다. 제어권이 없어
  제어권 상실 통지가 오지 않아도 같다. 이전 서비스 명령은 더 조회하지 않는다.
- 서비스가 `running`을 계속 반환하면 그대로 `실행 중`으로 표시한다. GUI가 시간·높이·motion으로 완료나 실패를
  만들어 내지 않는다.

판독 규칙:

- schema는 `.`으로 나눈 숫자 성분으로 읽는다. major가 1이 아니거나 형식이 다르면 상태를 해석하지 않고
  "해석 안 함"만 표시한다. minor·patch가 커도 해석한다.
- 관측값은 `supported && valid && fresh`일 때만 정상(녹색) 값으로 보여준다.

| supported | valid | fresh | 표시 |
|---|---|---|---|
| true | true | true | 값(녹색) |
| true | true | false | `값 (오래된 값)`(주황) — 제어 판단에 쓰지 않음 |
| true | false | true | `값 없음(오류 사유)`(주황) — 장치가 최근 응답했지만 값 무효 |
| true | false | false | `확인 불가[: 사유]`(주황) |
| false | - | - | `미지원`(회색) |
| null | - | - | `확인 불가(지원 미확인)`(주황) |

- faults의 빈 목록은 `supported && valid && fresh`일 때만 "없음"(녹색)이다. 활성 고장은 빨강, 모르는 코드는
  `코드(알 수 없는 코드)`로 표시한다.
- 모르는 열거값은 `값 (알 수 없는 값)`(주황)으로 표시하고 정상으로 단정하지 않는다. 모르는 키는 무시한다.
- `configuration.enabled=false`인 장치는 모든 항목을 "비활성"(회색)으로 표시한다.

| 색 | 의미 |
|---|---|
| 녹색 | 정상·준비(ONLINE, READY, CONNECTED, NORMAL, 유효한 최신 값, 준비 완료, 활성 고장 없음) |
| 주황 | 경고·확인 불가(UNKNOWN, INITIALIZING, DEGRADED, CONNECTING, 오래된 값, 값 무효, 설정 누락, 모르는 값, 준비 안 됨) |
| 빨강(굵게) | OFFLINE·DISCONNECTED·비상정지, 활성 고장, 비상정지·offline 사유의 준비 불가, schema 불일치 |
| 회색 | 값 없음(`-`), 미지원, 비활성 |
| 검정 | 중립 값(MANUAL, LOCKED, STOPPED, UNSUPPORTED 등) |

상태에 필드가 없거나 `None`이어도 `-`로 표시하고 GUI는 계속 동작한다.

## 입력 검증

API 호출 전에 GUI에서 먼저 확인하고, 잘못된 값은 오류 대화상자와 로그로 알린 뒤 호출하지 않는다.

- 각도·위치·대기 시간: 유한한 실수(빈 값, NaN, inf 거부). 대기 시간과 조회 간격은 0보다 커야 한다.
- PTU 속도 코드: 0x00~0x3F(10진수 또는 `0x` 16진수).
- Erector 속도 코드: 0~255.
- 목표 범위(각도·위치 상·하한 등)는 서비스 설정 기준으로 서비스가 판단하며 `OUT_OF_RANGE` 거부로 표시된다.

## 결과 해석

- `접수 command_id=…`는 명령이 접수됐다는 뜻이며 실제 완료가 아니다. 완료는 `완료 대기`로 확인한다.
- `거부 reason=…`은 `CommandReceipt` 거부 사유다(`wrong_mode`, `control_not_acquired` 등).
- 비상정지 결과는 `latched=True(실제 정지 확인 아님) | ptu: tx=… | mast: tx=… | erector: tx=…`로 표시한다.
  `tx=TX_SUCCESS`는 로컬 송신 성공일 뿐 장치 수신이나 정지 확인이 아니다. 실제 정지는 대시보드의 motion과
  위치 변화로 확인한다.
- 완료 대기·명령 조회 결과가 `state=completed`이면 접수한 명령 종류에 따라 의미를 덧붙인다.
  - 단순 명령(상·하·좌·우, Extend/Retract, Fold/Unfold, 장치별 STOP, 속도 설정):
    `[서비스 처리 완료 — 실제 장치 이동·정지 확인 아님(상태의 motion·위치로 확인)]`
  - PTU 절대 각도 이동·Home·Erect/Stow·Erector move_to/move_by·RESET: `[도착·완료 확인(정지 후 목표 허용오차 안)]`
  - 이 GUI에서 접수하지 않은 command_id를 직접 입력한 경우: `[명령 종류 미확인 — …]`
- `CommandWaitTimeoutError`는 제한시간 안에 끝나지 않았다는 뜻이며 마지막 상태를 함께 표시한다.

## 구현 메모

- 모든 호출은 `*_async` API를 쓴다. Future callback은 Client thread에서 실행되므로 결과를 thread-safe
  queue에 넣고, GUI thread가 `root.after()`(50 ms)로 꺼내 widget을 갱신한다.
- Client 생성자(HELLO 대기)와 `close()`는 blocking이므로 별도 thread에서 실행한다.
- 상태 주기 조회는 기본으로 켜져 있어 연결 후 1초마다 갱신하며, 이전 응답이 오기 전에 다시 보내지 않는다.
- 중복 클릭 방지: 요청을 보낸 버튼(같은 명령)은 응답이나 예외가 올 때까지 비활성화되고, 그 사이 같은 요청은
  로그에 `무시`로 남는다. 비상정지는 멱등이고 안전 우선이라 잠그지 않는다. 완료 대기·명령 조회도 잠그지 않는다.

## 시험

```bash
.venv/bin/python -m pytest tests/test_api_test_gui.py
```

입력 검증, 결과 표시(비상정지 결과 포함), 고객 예제 JSON 4종(`docs/customer_api/examples/`, 1.0.0)과
schema 1.1.0 `mast_bus` 시험 fixture(네 연결 상태·키 없음), schema·
valid/fresh·모르는 값 처리, completed 의미 구분, 장치별 마지막 명령 표시(결과 추적·늦은 응답 무시·연결 해제),
mock Client의 Future 결과 전달, 실제 Client와 모의 장치
서비스(`ServiceRig`) 연결 흐름을 확인한다. 고객 예제를 읽으므로 VM에 `docs/customer_api`도 동기화해야 한다
(`tools/sync_to_vm.sh`는 `docs/`를 복사하지 않는다). Tk 디스플레이가 없으면 GUI 시험은 skip된다. VM에서는 데스크톱 세션 디스플레이를 지정한다.

```bash
DISPLAY=:1 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority .venv/bin/python -m pytest tests/test_api_test_gui.py
```

배포 builder 시험은 어느 OS에서나 실행된다. 설치 시험은 Linux 전용이며 실제 venv를 만든다. 인터넷 대신 pyzmq 27.2.0 wheel 폴더를 지정한다.

```bash
PATROL_GUI_TEST_WHEELHOUSE=<wheel 폴더> [PATROL_GUI_TEST_PYTHON=python3] .venv/bin/python -m pytest tests/test_api_gui_release.py
```
