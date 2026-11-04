#!/usr/bin/env python3
# 지시서: specs/기능-자유주행-사람정지.md
"""15초 동안 앞으로 가다 사람이 가까우면 멈춘다. 사람 말고는 못 본다.

  python3 missions/free_drive.py --dry    바퀴 없이 카메라 · 인식 · 판단만
  python3 missions/free_drive.py --drive  실제 주행 (터미널에서 직접)
"""
import argparse
import json
import os
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from autodrive_core import Config, Perception, RED, GREEN, STOP
from autodrive_runtime import Hardware, Vision, check_video

ROOT = Path(__file__).resolve().parents[1]
SPEED, SECONDS, CAMERA_FAULT_SECONDS = 40, 15, 1.0
CFG = Config(use_obstacle=False, use_feature=False, person_close_height=300)


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


def run(hw, vision, key_pressed, report, clock=time.monotonic, sleep=time.sleep):
    """한 판. 어떻게 끝나든 finally 에서 네 바퀴를 0으로 만든다."""
    started = clock()
    perception = Perception(CFG)
    last_write = None
    try:
        errors = hw.stop_all()
        if errors:
            raise OSError('시작 전 정지 통신 실패: ' + '; '.join(errors))
        hw.light(GREEN)
        while True:
            now = clock()
            elapsed = now - started
            if key_pressed():
                report['reason'] = 'KEY_STOP'
                break
            if elapsed >= SECONDS:
                report['reason'] = 'TIME_LIMIT'
                break
            seq, detections, det_t, frame_t = vision.latest()
            if now - min(det_t, frame_t) > CAMERA_FAULT_SECONDS:
                report['reason'] = 'CAMERA_FAULT'
                break
            perception.update(seq, detections)
            heights = [d[2][3] for d in detections if d[0] == 'person' and d[1] >= CFG.min_score]
            report['max_person_height'] = max(heights + [report.get('max_person_height', 0)])
            if perception.confirmed('close'):
                report['reason'] = 'PERSON_CLOSE'
                report['stop_person_height'] = max(heights, default=None)
                hw.stop_all()
                hw.light(RED)
                hw.beep(True)
                sleep(0.5)
                hw.beep(False)
                break
            if key_pressed():
                report['reason'] = 'KEY_STOP'
                break
            if last_write is None or now - last_write >= 0.5:
                hw.set_wheels((SPEED,) * 4)
                last_write = now
            vision.set_status(f'DRIVE L{SPEED} R{SPEED} person_h {max(heights, default=0)} t {elapsed:.1f}')
            sleep(0.02)
    except BaseException as exc:
        report['reason'] = 'INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'ERROR'
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        report['stop_errors'] = hw.stop_all()
        report['elapsed_seconds'] = round(clock() - started, 3)
        vision.set_status(f'STOPPED {report.get("reason")}')


def session(hw, key_pressed, dry):
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d-%H%M%S')
    prefix = 'dry-' if dry else ''
    video = ROOT / 'results' / 'videos' / f'{prefix}free_drive-{stamp}.mp4'
    video.parent.mkdir(parents=True, exist_ok=True)
    report = {'program': 'free_drive', 'dry': dry, 'speed': SPEED, 'seconds': SECONDS,
              'person_stop_height': CFG.person_close_height, 'video': str(video.relative_to(ROOT))}
    vision = Vision(video, True)
    try:
        vision.start()
        print('카메라와 인식 준비 중...', flush=True)
        if not vision.wait_ready():
            raise RuntimeError('카메라/YOLO 준비 실패: ' + str(vision.error))
        print('출발 신호 — 삐', flush=True)
        hw.beep(True)
        time.sleep(0.3)
        hw.beep(False)
        time.sleep(0.7)
        run(hw, vision, key_pressed, report)
    finally:
        report['stop_errors_final'] = hw.close()
        vision.stop()
        report['vision_error'] = vision.error
        if video.exists():
            report['video_check'] = check_video(video)
        folder = ROOT / 'results' / 'runs'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f'{prefix}free_drive-{stamp}.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print('\n끝난 이유:', report.get('reason', '시작 전 종료'))
        print('시간(초):', report.get('elapsed_seconds'), '· 사람 상자 최대 높이:', report.get('max_person_height'))
        print('멈춘 순간 상자 높이:', report.get('stop_person_height'))
        print('영상:', report['video'], report.get('video_check'))
        print('기록:', path.relative_to(ROOT))
        if report.get('stop_errors') or report.get('stop_errors_final'):
            print('이상 — 정지 통신 실패. 바퀴가 돌면 확장보드 전원 스위치를 꺼 주세요.')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='실제 주행')
    parser.add_argument('--dry', action='store_true', help='바퀴 없이 판단만')
    args = parser.parse_args()
    if args.dry:
        session(DryHardware(), lambda: False, dry=True)
        return
    if not args.drive:
        parser.print_help()
        return
    if not sys.stdin.isatty():
        raise SystemExit('아무 키 정지를 위해 VS Code 터미널에서 직접 실행해야 합니다.')
    print(f'속도 {SPEED}로 최대 {SECONDS}초 직진합니다. 사람만 보고 멈춥니다 — 벽과 의자 앞에서는 멈추지 않습니다.')
    print('아무 키나 누르면 바로 멈춥니다.')
    if input('라즈봇을 바닥에 내려놓고 앞쪽과 사방 1m를 비웠나요? 준비됐다면 "확인" 입력: ').strip() != '확인':
        print('취소 — 바퀴 명령 없음')
        return
    terminal = termios.tcgetattr(sys.stdin)
    old_handler = signal.getsignal(signal.SIGTERM)

    def interrupt(signum, frame):
        raise KeyboardInterrupt('종료 신호')

    try:
        signal.signal(signal.SIGTERM, interrupt)
        tty.setcbreak(sys.stdin.fileno())
        session(Hardware(use_ultrasonic=False), lambda: bool(select.select([sys.stdin], [], [], 0)[0]), dry=False)
    finally:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        signal.signal(signal.SIGTERM, old_handler)


if __name__ == '__main__':
    main()
