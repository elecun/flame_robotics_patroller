# 고객사 API 문서

이 문서는 고객사 앱 개발 기준입니다. 실행 가능한 패키지는 차주 방문하여 검증 후 제공 예정입니다.

| 문서 | 내용 |
|---|---|
| [01_developer_guide.md](01_developer_guide.md) | 먼저 읽을 문서: 구성, 설치와 서비스 등록, 첫 연결, 운전 순서, 명령 접수·완료, 비동기 사용, 제어권·장애 처리, 안전 한계 |
| [02_api_reference.md](02_api_reference.md) | Python Client 상세 API 명세: 생성자와 전체 메서드의 시그니처·인자·반환·예외·권한, timeout·재시도, 명령 상태와 완료 조건, 열거형 |
| [03_status_contract.md](03_status_contract.md) | 상태 API 명세: 공통 구조·타입, 장치별 필드와 채우는 기준, 운전 준비(readiness), 상태 형식 버전 규칙 |
| [examples/](examples/) | 상태 JSON 예시: 정상, 통신 단절, 설정 미완성, 비활성·미지원 |
