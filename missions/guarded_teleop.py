#!/usr/bin/env python3
# 지시서: specs/기능-원격조종-사람정지.md
"""키보드로 조종하되 앞에 사람이 가까우면 자동으로 멈추고 잠근다. 앞쪽 사람만 본다.

  python3 missions/guarded_teleop.py --dry    바퀴 없이 키와 판단만
  python3 missions/guarded_teleop.py --drive  실제 조종 (터미널에서 직접)
"""
import argparse
import curses
import json
import os
import signal
import sys
import termios
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from autodrive_core import Config, Perception, GREEN, RED
from autodrive_runtime import Hardware, Vision, check_video

ROOT = Path(__file__).resolve().parents[1]
CFG = Config(use_obstacle=False, use_feature=False, person_close_height=350)
MOVES = {'w': (1, 1, 1, 1), 's': (-1, -1, -1, -1), 'a': (-1, 1, 1, -1),
         'd': (1, -1, -1, 1), 'q': (-1, -1, 1, 1), 'e': (1, 1, -1, -1)}
START_SPEED, MIN_SPEED, MAX_SPEED, SPEED_STEP = 60, 40, 100, 20
AUTO_STOP = 0.6
CAMERA_FAULT = 1.0
CLEAR_SECONDS = 1.0
BEEP_SECONDS = 0.5


class DryHardware:
    """바퀴 · I2C 접근 없음."""

    def set_wheels(self, speeds):
        pass

    def stop_all(self):
        return []

    def light(self, color):
        pass

    def beep(self, on):
        pass

    def close(self):
        return []


class Controller:
    """키 입력과 사람 잠금. 잠금은 언제나 키보다 먼저 판단한다."""

    def __init__(self, hw):
        self.hw = hw
        self.perception = Perception(CFG)
        self.speed, self.moving, self.last_press = START_SPEED, None, 0.0
        self.locked, self.lock_reason, self.clear_since = False, '', None
        self.beep_until = None
        self.events = []
        self.person_height = 0

    def _stop(self):
        self.moving = None
        errors = self.hw.stop_all()
        if errors:
            raise OSError('정지 통신 실패: ' + '; '.join(errors))

    def update(self, seq, detections, det_t, frame_t, now):
        self.perception.update(seq, detections)
        heights = [d[2][3] for d in detections if d[0] == 'person' and d[1] >= CFG.min_score]
        self.person_height = max(heights, default=0)
        fault = now - min(det_t, frame_t) > CAMERA_FAULT
        danger = 'CAMERA_FAULT' if fault else 'PERSON_CLOSE' if self.perception.confirmed('close') else None
        if self.beep_until is not None and now >= self.beep_until:
            self.hw.beep(False)
            self.beep_until = None
        if danger:
            self.clear_since = None
            if not self.locked:
                self.locked, self.lock_reason = True, danger
                self._stop()
                self.hw.light(RED)
                self.hw.beep(True)
                self.beep_until = now + BEEP_SECONDS
                self.events.append({'t': now, 'lock': danger, 'person_height': self.person_height})
            return
        if self.locked:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= CLEAR_SECONDS:
                self.locked, self.lock_reason = False, ''
                self.hw.light(GREEN)
                self.events.append({'t': now, 'unlock': True})
            return
        if self.moving and now - self.last_press > AUTO_STOP:
            self._stop()

    def key(self, key, now):
        """x 를 누르면 False 를 돌려준다."""
        if key == 'x':
            return False
        if key == ' ':
            self._stop()
        elif key in '+=':
            self.speed = min(MAX_SPEED, self.speed + SPEED_STEP)
        elif key in '-_':
            self.speed = max(MIN_SPEED, self.speed - SPEED_STEP)
        elif key in MOVES and not self.locked:
            self.moving, self.last_press = key, now
            self.hw.set_wheels(tuple(sign * self.speed for sign in MOVES[key]))
        return True


def screen_loop(screen, ctrl, vision):
    curses.curs_set(0)
    screen.timeout(30)
    while True:
        screen.erase()
        screen.addstr(0, 0, 'Raspbot guarded teleop')
        screen.addstr(1, 0, 'w/s fwd/back   a/d side   q/e turn   space stop   +/- speed   x quit')
        screen.addstr(3, 0, f'speed {ctrl.speed:3d}   move {ctrl.moving or "stop"}   person_h {ctrl.person_height}')
        screen.addstr(4, 0, f'LOCKED: {ctrl.lock_reason}' if ctrl.locked else 'ready')
        ch = screen.getch()
        now = time.monotonic()
        # 사람 판단을 키보다 먼저 한다
        ctrl.update(*vision.latest(), now)
        vision.set_status(f'{"LOCK " + ctrl.lock_reason if ctrl.locked else "TELEOP"} '
                          f'speed {ctrl.speed} move {ctrl.moving or "stop"} person_h {ctrl.person_height}')
        if 0 <= ch < 256 and not ctrl.key(chr(ch).lower(), now):
            return


def session(hw, dry, body=None, name='guarded_teleop'):
    """장치 준비 · 조종 · 정리. body(ctrl, vision) 가 조종 방식이다 (기본은 키보드 화면)."""
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d-%H%M%S')
    prefix = 'dry-' if dry else ''
    video = ROOT / 'results' / 'videos' / f'{prefix}{name}-{stamp}.mp4'
    video.parent.mkdir(parents=True, exist_ok=True)
    report = {'program': name, 'dry': dry, 'person_stop_height': CFG.person_close_height,
              'video': str(video.relative_to(ROOT))}
    vision = Vision(video, True)
    ctrl = Controller(hw)
    try:
        vision.start()
        print('카메라와 인식 준비 중...', flush=True)
        if not vision.wait_ready():
            raise RuntimeError('카메라/YOLO 준비 실패: ' + str(vision.error))
        errors = hw.stop_all()
        if errors:
            raise OSError('시작 전 정지 통신 실패: ' + '; '.join(errors))
        hw.light(GREEN)
        if body is None:
            curses.wrapper(screen_loop, ctrl, vision)
        else:
            body(ctrl, vision)
        report['reason'] = 'QUIT'
    except BaseException as exc:
        report['reason'] = 'INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'ERROR'
        report['error'] = str(exc) or type(exc).__name__
        if not isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        report['stop_errors'] = hw.stop_all()
        report['stop_errors_final'] = hw.close()
        vision.stop()
        report['vision_error'] = vision.error
        report['events'] = ctrl.events
        if video.exists():
            report['video_check'] = check_video(video)
        folder = ROOT / 'results' / 'runs'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f'{prefix}{name}-{stamp}.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print('\n끝난 이유:', report.get('reason'))
        print('자동 잠금 횟수:', sum('lock' in e for e in ctrl.events))
        print('영상:', report['video'], report.get('video_check'))
        print('기록:', path.relative_to(ROOT))
        if report['stop_errors'] or report['stop_errors_final']:
            print('이상 — 정지 통신 실패. 바퀴가 돌면 확장보드 전원 스위치를 꺼 주세요.')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='실제 조종')
    parser.add_argument('--dry', action='store_true', help='바퀴 없이 키와 판단만')
    args = parser.parse_args()
    if not args.drive and not args.dry:
        parser.print_help()
        return
    if not sys.stdin.isatty():
        raise SystemExit('키보드 조종이라 VS Code 터미널에서 직접 실행해야 합니다.')
    if args.dry:
        session(DryHardware(), dry=True)
        return
    run_on_robot()


def run_on_robot(body=None, name='guarded_teleop'):
    """준비 확인을 받고 실제 장치로 실행한다. 터미널이 끊겨도 바퀴를 멈춘다."""
    print('앞쪽 사람만 보고 멈춥니다. 옆과 뒤의 사람, 벽, 의자에는 멈추지 않습니다.')
    if input('라즈봇을 바닥에 내려놓고 사방 1m를 비웠나요? 준비됐다면 "확인" 입력: ').strip() != '확인':
        print('취소 — 바퀴 명령 없음')
        return
    terminal = termios.tcgetattr(sys.stdin)
    old_handler = signal.getsignal(signal.SIGTERM)

    def interrupt(signum, frame):
        raise KeyboardInterrupt('종료 신호')

    try:
        # SSH 가 끊기면 SIGHUP 이 온다: 바퀴를 멈추고 끝낸다
        signal.signal(signal.SIGTERM, interrupt)
        signal.signal(signal.SIGHUP, interrupt)
        session(Hardware(use_ultrasonic=False), dry=False, body=body, name=name)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        signal.signal(signal.SIGTERM, old_handler)


if __name__ == '__main__':
    main()
