#!/usr/bin/env python3
# 지시서: specs/3-장애물.md
"""3단계: 줄을 따라가다 장애물 앞에서 멈추고, 치우면 다시 간다. 블랙박스와 기록표를 남긴다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from autodrive_core import Config
from autodrive_runtime import cli

if __name__ == '__main__':
    cli(Config(use_yolo=False, use_feature=False), '3_obstacle', 'specs/3-장애물.md')
