#!/usr/bin/env python3
# 지시서: specs/기능-원격조종-사람정지.md
"""터미널에 글자 명령을 쳐서 움직인다. 예: w 2 → 2초 앞으로. 카메라에 사람이 인식되면 거리와 무관하게 멈춘다.

  python3 missions/cmd_drive.py --dry    바퀴 없이 명령과 판단만
  python3 missions/cmd_drive.py --drive  실제 조종 (터미널에서 직접)
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from guarded_teleop import (BEEP_SECONDS, CLEAR_SECONDS, MAX_SPEED, MIN_SPEED, MOVES,
                            DryHardware, run_on_robot, session)

MAX_SECONDS = 5.0
SETTLE_SECONDS = 0.25
ALIASES = {'forward': 'w', 'back': 's', 'left': 'a', 'right': 'd', 'turnleft': 'q', 'turnright': 'e'}
HELP = ('w/s 앞뒤 · a/d 옆 · q/e 제자리 회전 (뒤에 초를 적음: w 2)\n'
        f'speed 80 ({MIN_SPEED}~{MAX_SPEED}) · stop · quit · 움직이는 중 아무 키 = 정지')


def parse(line):
    """글자 한 줄 → (종류, 값). 모르면 ('error', 이유). 움직이지 않는다."""
    parts = line.strip().lower().split()
    if not parts:
        return 'empty', None
    word = ALIASES.get(parts[0], parts[0])
    if word in ('quit', 'exit') and len(parts) == 1:
        return 'quit', None
    if word == 'stop' and len(parts) == 1:
        return 'stop', None
    if word == 'help' and len(parts) == 1:
        return 'help', None
    if word == 'speed' and len(parts) == 2:
        if parts[1].isdigit() and MIN_SPEED <= int(parts[1]) <= MAX_SPEED:
            return 'speed', int(parts[1])
        return 'error', f'속도는 {MIN_SPEED}~{MAX_SPEED} 숫자만 됩니다.'
    if word in MOVES and len(parts) <= 2:
        try:
            seconds = float(parts[1]) if len(parts) == 2 else 1.0
        except ValueError:
            return 'error', '시간은 숫자로 적어 주세요.'
        # nan 도 여기서 걸러진다
        if not 0 < seconds <= MAX_SECONDS:
            return 'error', f'시간은 0초 초과 {MAX_SECONDS:.0f}초 이하만 됩니다.'
        return 'move', (word, seconds)
    return 'error', '모르는 명령입니다. help 를 쳐 보세요.'


def settle(ctrl, vision, clock, sleep):
    """프롬프트에 있는 동안 못 본 인식 결과를 먼저 읽는다. 잠겨 있으면 풀릴 시간을 준다."""
    end = clock() + (CLEAR_SECONDS + SETTLE_SECONDS if ctrl.locked else SETTLE_SECONDS)
    while clock() < end:
        ctrl.update(*vision.latest(), clock())
        sleep(0.03)


def finish_beep(ctrl, sleep):
    """프롬프트로 돌아가기 전에 부저를 끈다."""
    if ctrl.beep_until is not None:
        sleep(BEEP_SECONDS)
        ctrl.hw.beep(False)
        ctrl.beep_until = None


def run_move(ctrl, vision, letter, seconds, key_pressed, clock=time.monotonic, sleep=time.sleep):
    """움직임 하나. DONE · KEY_STOP · 잠금 이유 중 하나를 돌려주고, 끝나면 언제나 바퀴 0."""
    try:
        settle(ctrl, vision, clock, sleep)
        if ctrl.locked:
            return ctrl.lock_reason
        started = clock()
        while True:
            now = clock()
            ctrl.update(*vision.latest(), now)
            if ctrl.locked:
                return ctrl.lock_reason
            if key_pressed():
                return 'KEY_STOP'
            if now - started >= seconds:
                return 'DONE'
            ctrl.key(letter, now)
            vision.set_status(f'CMD {letter} speed {ctrl.speed} t {now - started:.1f} person_h {ctrl.person_height}')
            sleep(0.05)
    finally:
        ctrl._stop()
        finish_beep(ctrl, sleep)
        vision.set_status(f'WAIT speed {ctrl.speed} {ctrl.lock_reason}')


def execute_move(ctrl, vision, letter, seconds):
    """움직이는 동안만 키 하나로 멈출 수 있게 터미널을 바꾼다."""
    terminal = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        return run_move(ctrl, vision, letter, seconds, lambda: bool(select.select([sys.stdin], [], [], 0)[0]))
    finally:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)


def shell(ctrl, vision):
    print(HELP)
    while True:
        try:
            line = input(f'[속도 {ctrl.speed}] > ')
        except EOFError:
            return
        kind, value = parse(line)
        if kind == 'quit':
            return
        if kind == 'help':
            print(HELP)
        elif kind == 'error':
            print('거절 —', value)
        elif kind == 'stop':
            ctrl._stop()
            print('정지')
        elif kind == 'speed':
            ctrl.speed = value
            print('속도', value)
        elif kind == 'move':
            result = execute_move(ctrl, vision, *value)
            print({'DONE': '끝', 'KEY_STOP': '키를 눌러 정지',
                   'PERSON_DETECTED': '사람이 인식되어 정지 · 잠금 (안 보이면 다시 치세요)',
                   'CAMERA_FAULT': '카메라가 멈춰서 정지 · 잠금'}.get(result, result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='실제 조종')
    parser.add_argument('--dry', action='store_true', help='바퀴 없이 명령과 판단만')
    args = parser.parse_args()
    if not args.drive and not args.dry:
        parser.print_help()
        return
    if not sys.stdin.isatty():
        raise SystemExit('터미널에서 직접 실행해 주세요.')
    if args.dry:
        session(DryHardware(), dry=True, body=shell, name='cmd_drive')
    else:
        run_on_robot(body=shell, name='cmd_drive')


if __name__ == '__main__':
    main()
