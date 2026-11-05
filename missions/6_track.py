#!/usr/bin/env python3
# 지시서: specs/6-트랙주행.md
"""대구대 트랙 주행: 5단계 오토드라이브의 정지 규칙에 카메라 노란 차선 조향을 붙인다."""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from autodrive_core import Config
from autodrive_runtime import cli

DEFAULT_TILT = 5  # 초안 — specs/6-트랙주행.md 「카메라 각도」


def build_config(no_obstacle=False, tilt=DEFAULT_TILT, no_camera_block=False):
    if not 0 <= tilt <= 110:
        raise SystemExit('--tilt 는 0~110 이어야 합니다.')
    # 시간 제한 없음: 사용자가 「종료」를 칠 때까지. 사람은 한 번만 봐도 바로 멈춘다.
    return Config(use_camera_lane=True, use_obstacle=not no_obstacle, camera_tilt=tilt,
                  use_camera_block=not no_camera_block, person_close_confirm=1,
                  time_limit_seconds=math.inf)


def aim(cfg):
    """카메라 각도만 맞춘다. 바퀴 명령 없음."""
    from Raspbot_Lib import Raspbot
    bot = Raspbot()
    bot.Ctrl_Servo(1, cfg.camera_pan)
    bot.Ctrl_Servo(2, cfg.camera_tilt)
    print(f'카메라 각도: 좌우 {cfg.camera_pan} · 위아래 {cfg.camera_tilt}. 바퀴 명령 없음.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--no-obstacle', action='store_true')
    parser.add_argument('--tilt', type=int, default=DEFAULT_TILT)
    parser.add_argument('--aim', action='store_true')
    parser.add_argument('--no-camera-block', action='store_true')
    known, rest = parser.parse_known_args()
    config = build_config(known.no_obstacle, known.tilt, known.no_camera_block)
    if known.aim:
        aim(config)
    else:
        cli(config, '6_track', 'specs/6-트랙주행.md', rest)
