# 라즈봇 프로젝트

조 번호: 2조  ·  조원: (이름 대신 역할도 됨 — 조장 · 운전석 · 트랙·표지 · 측정 · 기록·발표)

## 우리가 보는 곳은 두 곳
- `specs/` — 우리가 쓴 **지시서** (무엇을 · 왜). 고칠 때도 여기부터.
- `results/` 와 `log.md` — 라즈봇이 **실제로 어떻게 움직였는지** (사진 · 측정 · 문제 기록)

나머지(`tools/` 점검 프로그램 · `missions/` 만들어진 프로그램)는 Codex가 정리합니다.
Codex는 `AGENTS.md`(작업 수칙)를 따릅니다.

## 실행
- 1단계 블랙박스: `cd /home/pi/raspbot` 후 `python3 missions/01_blackbox.py`
  - 바퀴를 움직이지 않고 10초 촬영. 2조·한국 시각·초음파 거리 표시.
  - `results/blackbox-날짜-시각/2jo-blackbox.mp4`에 저장. 같은 폴더에 거리 측정값과 검사 보고서 저장.
  - 최신 검증 영상: [10초 블랙박스](results/blackbox-20261104-131305-656931/2jo-blackbox.mp4). 휴대폰 재생 확인 대기.
  - 영상은 레포에 올리지 않고 안내된 조별 드라이브 폴더에 제출한다.
- 키보드 조종 + 사람 인식 정지: `python3 missions/guarded_teleop.py --drive`
  - 카메라에 사람(인식 점수 0.5 이상)이 인식되면 크기·거리와 관계없이 첫 인식부터 정지하고 이동 키를 차단한다.
  - w/s 앞뒤 · a/d 옆 · q/e 회전 · 스페이스 정지 · x 종료. 사람 미인식 상태가 1초 지속되면 잠금이 풀리고 다시 키를 눌러 움직인다.
  - 바퀴 없는 카메라 시험: `python3 missions/guarded_teleop.py --dry`.
  - 35m 거리 측정 기능은 없으며, 35m 사람 인식·실제 정지는 미검증. 카메라는 한 프로그램만 사용한다. `--dry` 시험은 x로 종료한 다음 `--drive`를 실행한다.
  - 기존 `python3 tools/teleop.py`는 사람 인식 정지 기능이 없는 일반 조종이다.

## 2단계 라인 따라가기 — 코드 준비 완료, 실제 주행 미검증

지시서: [02-라인.md](specs/02-라인.md). 기본 속도 60, 회전 안쪽 30·바깥쪽 60, 최대 10초.

- 가상 검사: `python3 missions/02_line.py` — 센서·바퀴에 접근하지 않는다.
- 정지 처리 검사: `python3 tools/test_line_logic.py` — 가짜 센서·모터로 검사한다.
- 실제 센서 값 확인: `python3 tools/line_monitor.py` — 바퀴 명령 없이 60초간 표시한다. 0은 검정, 1은 흰색. 검은 줄을 센서 한 칸씩 지나가게 하며 좌우 순서를 확인한다. Ctrl+C로 종료.
- 실제 주행: 센서 배치를 점검하고 바닥·사방 1m를 확보한 뒤 대화형 터미널에서 `python3 missions/02_line.py --drive`. 준비 질문에 `확인`을 입력하면 시작하며, 아무 키나 누르면 종료한다.
- 줄 놓침·네 센서 모두 검정·오류·10초 경과 때 정지한다. 실행 기록은 `results/line-날짜-시각.json`에 저장한다.
- 현재는 트랙 없이 코드만 준비한 상태다. 직선·곡선 통과 여부와 실제 정지는 아직 확인하지 않았다.

## 3~5단계 · 우리만의 기능 · 글 명령 — 코드 준비 완료, 실제 주행 미검증

숫자와 규칙은 Claude 초안이다. 조가 확인하고 지시서부터 고친다.

| 무엇 | 지시서 | 실행 |
|---|---|---|
| 장애물 정지 | [3-장애물.md](specs/3-장애물.md) | `python3 missions/3_obstacle.py --drive --session 4 --attempt 1` |
| YOLO | [4-욜로.md](specs/4-욜로.md) | `python3 tools/yolo.py` · `python3 missions/4_yolo.py --capture person` · `--eval` |
| 오토드라이브 | [5-오토드라이브.md](specs/5-오토드라이브.md) | `python3 missions/5_autodrive.py --drive --session 7 --attempt 1` |
| 보행자 긴급 제동 | [기능-보행자제동.md](specs/기능-보행자제동.md) | 오토드라이브에 포함. 끄기: `--no-feature` |
| 글 명령 | [글명령.md](specs/글명령.md) | `python3 missions/text_command.py --drive` |

- 바퀴 없이 시험: `--simulate` (기록표에 안 남음) · `python3 tools/test_autodrive.py`
- 기록표: `results/runs.csv` · 영상: `results/videos/` (레포에 안 올림) · 인쇄용 STOP 표지: `results/print/stop-sign.png`

## 실제 트랙 — 먼저 읽기

[트랙 지시서](specs/트랙-대구대.md) · 사진: `docs/track/`. 도로 전체가 검은색이라 **`06`, `02_line_follow`, `5_autodrive`는 트랙에서 실행하지 않는다**(차선을 보지 않고 직진할 수 있다). 트랙에서는 `07`을 출발점으로 쓴다.

## 3조에서 가져온 자율주행 코드 — 2조 실제 주행 미검증

출처: [raspbot-pbl-3team](https://github.com/jihooni217/raspbot-pbl-3team) `7e8a30d`. 규칙과 숫자는 3조 값이며 2조가 아직 확인하지 않았다. 기존 2조 프로그램은 그대로 있다.

| 무엇 | 지시서 | 실행 |
|---|---|---|
| 카메라 노란 차선 따라가기 | [07-camera-lane.md](specs/07-camera-lane.md) | 바퀴 없이: `python3 missions/07_camera_lane.py --check` · 주행: `python3 missions/07_camera_lane.py` |
| 검은 줄 + 사람·STOP 반응 | [06-drive-signs.md](specs/06-drive-signs.md) | `python3 missions/06_drive_signs.py --trial 시험이름` |
| 초음파 정지·재출발 | [04-obstacle.md](specs/04-obstacle.md) | `python3 missions/04_obstacle.py` |
| 검은 줄 따라가기 (06·07이 불러 씀) | [02-line-follow.md](specs/02-line-follow.md) | `python3 missions/02_line_follow.py --check-sensors` |
| 표지 인식 (06이 불러 씀) · 카메라·녹화 (05·06·07이 불러 씀) | [05-signs.md](specs/05-signs.md) · [03-video.md](specs/03-video.md) | `missions/05_signs.py` · `missions/03_video.py` |

- **주의:** `07`·`06`·`04`는 2조 프로그램과 달리 `--drive`나 「확인」 입력 없이 **실행하면 바로 바퀴가 돈다.** 바닥·사방 1m를 먼저 비운다. 아무 키나 누르면 멈춘다.
- `07`은 카메라 각도를 스스로 맞추지 않는다. 먼저 `--check`로 노란 선 두 줄이 잡히는지 본다.
- 바퀴 없이 시험: `python3 tools/test_camera_lane.py` · `test_drive_signs.py` · `test_obstacle.py` · `test_signs.py` · `test_line.py`

