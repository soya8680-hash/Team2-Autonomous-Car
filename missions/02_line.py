#!/usr/bin/env python3
# 지시서: specs/02-라인.md
"""2조 라인 따라가기. 기본 실행은 장치에 접근하지 않는 가상 검사."""
import argparse
import json
import select
import signal
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SPEED, INNER_SPEED, MAX_SECONDS = 60, 30, 10
STOP = (0, 0, 0, 0)


def decide(raw):
    if not isinstance(raw, int) or not 0 <= raw <= 255:
        raise ValueError('라인센서 값이 잘못됐습니다.')
    # 물리적 왼쪽 → 오른쪽: x2, x1, x3, x4. 검정은 0.
    black = tuple(not ((raw >> bit) & 1) for bit in (2, 3, 1, 0))
    if not any(black):
        return 'LOST_LINE', STOP
    if all(black):
        return 'ALL_BLACK', STOP
    offset = sum(weight for weight, hit in zip((-3, -1, 1, 3), black) if hit)
    if offset < 0:
        return 'LEFT', (INNER_SPEED, INNER_SPEED, SPEED, SPEED)
    if offset > 0:
        return 'RIGHT', (SPEED, SPEED, INNER_SPEED, INNER_SPEED)
    return 'STRAIGHT', (SPEED,) * 4


def drive(bot, speeds):
    for motor, speed in enumerate(speeds):
        if abs(speed) > SPEED:
            raise ValueError('확정한 속도 제한을 초과했습니다.')
        # 드라이버와 같은 명령. 통신 오류를 숨기지 않고 상위로 전달한다.
        bot._device.write_i2c_block_data(bot._addr, 0x01,
                                        [motor, int(speed < 0), abs(speed)])


def stop_all(bot):
    errors = []
    for motor in range(4):
        try:
            bot._device.write_i2c_block_data(bot._addr, 0x01, [motor, 0, 0])
        except OSError as exc:
            errors.append(f'바퀴 {motor}: {exc}')
    return errors


def run_loop(bot, key_pressed, report, clock=time.monotonic, sleep=time.sleep):
    started = clock()
    try:
        initial_errors = stop_all(bot)
        if initial_errors:
            raise OSError('시작 전 정지 통신 실패: ' + '; '.join(initial_errors))
        while True:
            elapsed = clock() - started
            if key_pressed():
                report['reason'] = 'KEY_STOP'
                break
            if elapsed >= MAX_SECONDS:
                report['reason'] = 'TIME_LIMIT'
                break
            data = bot._device.read_i2c_block_data(bot._addr, 0x0a, 1)
            if not data or len(data) != 1:
                raise OSError('라인센서 읽기에 실패했습니다.')
            state, speeds = decide(data[0])
            report['samples'].append({'elapsed': round(clock() - started, 4),
                                      'raw': data[0], 'state': state, 'speeds': speeds})
            # 센서 읽기 도중 들어온 키나 시간 초과도 이동 명령 전에 확인한다.
            if key_pressed():
                report['reason'] = 'KEY_STOP'
                break
            if clock() - started >= MAX_SECONDS:
                report['reason'] = 'TIME_LIMIT'
                break
            if speeds == STOP:
                report['reason'] = state
                break
            drive(bot, speeds)
            sleep(0.02)
    except BaseException as exc:
        report['reason'] = 'INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'ERROR'
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        report['stop_errors'] = stop_all(bot)
        report['elapsed_seconds'] = clock() - started


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='준비 확인 후 실제 주행')
    args = parser.parse_args()
    if not args.drive:
        print('가상 센서 검사 — 장치 접근·바퀴 명령 없음')
        for raw in range(16):
            state, speeds = decide(raw)
            print(f'센서 레지스터 {raw:04b}: {state:10s} {speeds}')
        return
    if not sys.stdin.isatty():
        raise RuntimeError('아무 키 정지를 위해 대화형 터미널에서 실행해야 합니다.')
    print('센서 배치를 먼저 확인한 뒤 실행하세요. 최대 10초, 아무 키나 누르면 정지합니다.')
    answer = input('라즈봇을 바닥에 내려놓고 사방 1m를 비웠나요? 준비됐다면 "확인" 입력: ')
    if answer.strip() != '확인':
        print('취소 — 바퀴 명령 없음')
        return
    from Raspbot_Lib import Raspbot
    report = {'team': '2조', 'mode': 'hardware', 'samples': []}
    terminal = termios.tcgetattr(sys.stdin)
    old_handler = signal.getsignal(signal.SIGTERM)
    bot = Raspbot()

    def interrupt(signum, frame):
        raise KeyboardInterrupt('종료 신호')

    def key_pressed():
        return bool(select.select([sys.stdin], [], [], 0)[0])

    try:
        signal.signal(signal.SIGTERM, interrupt)
        tty.setcbreak(sys.stdin.fileno())
        run_loop(bot, key_pressed, report)
    finally:
        # 루프 진입 전 실패했을 때도 네 바퀴 모두 정지를 시도한다.
        report['final_stop_errors'] = stop_all(bot)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        signal.signal(signal.SIGTERM, old_handler)
        bot._device.close()
        path = ROOT / 'results' / datetime.now(ZoneInfo('Asia/Seoul')).strftime('line-%Y%m%d-%H%M%S-%f.json')
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print('주행 종료: ' + report.get('reason', '시작 전 종료'))
        print('기록: ' + str(path))
        if report.get('stop_errors') or report.get('final_stop_errors'):
            print('이상 — 정지 통신 실패. 바퀴가 돌면 확장보드 전원 스위치를 꺼 주세요.')


if __name__ == '__main__':
    main()
