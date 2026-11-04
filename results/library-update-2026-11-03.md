# 라이브러리 업데이트 확인 — 2026-11-03

정상 — 지정된 공유 폴더의 설치용 라이브러리와 현재 설치본이 같아 교체할 파일이 없습니다. 새 버전 설치는 하지 않았습니다.

## 확인한 자료
- 출처: https://drive.google.com/drive/folders/13kPGTN94561dEGRi_KxU-YzwxV51zEfn?hl=ko
- 6단계 → raspbot → tools → py_install.zip: 버전 0.0.2.
- 참고자료 → Yahboom-예제코드.zip: 교육용 예제 모음, 별도 Raspbot_Lib 배포 파일 없음.
- 이 판정은 지정된 공유 폴더 기준입니다. 제조사 전체 배포본이나 운영체제 전체 패키지의 최신 여부를 의미하지 않습니다.

## 비교·검증 결과
| 항목 | 결과 |
|---|---|
| 기존 tools/py_install.zip과 공유본 | 바이트 단위로 동일 |
| 실제 설치된 Raspbot_Lib/__init__.py | 공유본과 SHA-256 일치 |
| 실제 설치된 Raspbot_Lib/Raspbot_Lib.py | 공유본과 SHA-256 일치 |
| Raspbot_Lib·smbus·OpenCV 불러오기 | 성공, OpenCV 4.10.0 |
| 모터·서보·불빛·부저·초음파·센서 읽기 API | 존재 확인 |
| 기존 점검 프로그램 5개 문법 | 통과 |
| I2C 0x2b의 0x0a 레지스터 읽기 | [15] 응답 |

## 저장한 파일
- tools/py_install-incoming-2026-11-03.zip: 이번에 받은 설치 원본.
- tools/yahboom-examples-2026-11-03.zip: 이번에 받은 교육용 예제 원본.
- results/library-update-2026-11-03/installed-driver-backup.zip: 기존 설치 드라이버와 설치 ZIP 백업.
- results/library-update-2026-11-03/checksums.json: 내려받은 ZIP의 크기·SHA-256.
- 같은 결과 폴더의 HTML: 열람한 공유 폴더 목록 기록.

## 적용 내역과 확인 범위
- 설치된 파일 변경 없음: 공유본과 이미 동일하므로 재설치 생략.
- setup.py·pip·시스템 전체 업그레이드 실행 없음. smbus와 OpenCV는 이미 설치되어 프로젝트 설치 수칙에 따라 유지.
- 예제 프로그램 실행 없음. 바퀴·서보·불빛 동작 시험 없음.
- 실제 바퀴 동작은 이전 앞뒤 이동 시험에서 사용자 확인 대기 중입니다.
