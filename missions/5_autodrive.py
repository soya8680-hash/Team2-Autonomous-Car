#!/usr/bin/env python3
# 지시서: specs/5-오토드라이브.md · specs/기능-보행자제동.md
"""5단계 오토드라이브: 한 번 실행으로 줄 1바퀴 · 장애물 정지 · 사람 · STOP 반응 · 영상과 기록 저장."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from autodrive_core import Config
from autodrive_runtime import cli

if __name__ == '__main__':
    argv = sys.argv[1:]
    # --no-feature: 우리만의 기능을 끄고 비교할 때
    feature = '--no-feature' not in argv
    cli(Config(use_feature=feature), '5_autodrive', 'specs/5-오토드라이브.md',
        [a for a in argv if a != '--no-feature'])
