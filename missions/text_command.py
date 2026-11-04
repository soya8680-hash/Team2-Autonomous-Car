#!/usr/bin/env python3
# 지시서: specs/글명령.md
"""글 명령: 입력한 문장을 시험한 동작 목록에서만 골라 실행한다. 목록에 없으면 거절.

지금은 AI 없이 문장을 정확히 맞춰 고른다 (8회차 AI 연결 전 단계). choose() 만 바꾸면 AI로 교체할 수 있다.
  python3 missions/text_command.py          문장 해석만 (장치 접근 없음)
  python3 missions/text_command.py --drive  실제 실행 (바퀴를 쓰는 동작 포함)
"""
import re
import sys
import time

SPEED, SECONDS = 60, 0.5
# 동작 이름 → 네 바퀴 (왼앞, 왼뒤, 오앞, 오뒤). AGENTS.md 메카넘 부호와 같다.
MOVES = {
    'forward': (1, 1, 1, 1), 'back': (-1, -1, -1, -1),
    'left': (-1, 1, 1, -1), 'right': (1, -1, -1, 1),
    'turn_left': (-1, -1, 1, 1), 'turn_right': (1, 1, -1, -1),
}
COLORS = {'red': 0, 'green': 1, 'blue': 2, 'yellow': 3}
PHRASES = {
    'forward': ['앞으로', '앞으로가', '앞으로가줘', '전진', '전진해', '전진해줘'],
    'back': ['뒤로', '뒤로가', '뒤로가줘', '후진', '후진해', '후진해줘'],
    'left': ['왼쪽으로', '왼쪽으로가', '왼쪽으로가줘', '옆으로왼쪽'],
    'right': ['오른쪽으로', '오른쪽으로가', '오른쪽으로가줘', '옆으로오른쪽'],
    'turn_left': ['왼쪽으로돌아', '왼쪽으로돌아줘', '제자리왼쪽', '좌회전'],
    'turn_right': ['오른쪽으로돌아', '오른쪽으로돌아줘', '제자리오른쪽', '우회전'],
    'beep': ['삐', '경적', '경적울려', '경적울려줘', '삐소리내줘'],
    'light_red': ['빨간불', '빨간불켜', '빨간불켜줘'],
    'light_green': ['초록불', '초록불켜', '초록불켜줘'],
    'light_blue': ['파란불', '파란불켜', '파란불켜줘'],
    'light_yellow': ['노란불', '노란불켜', '노란불켜줘'],
    'light_off': ['불꺼', '불꺼줘'],
    'stop': ['멈춰', '멈춰줘', '정지', '정지해', '정지해줘'],
}
LOOKUP = {phrase: action for action, phrases in PHRASES.items() for phrase in phrases}


def choose(text):
    """문장 → 동작 이름. 목록에 없으면 None (거절)."""
    return LOOKUP.get(re.sub(r'[\s.,!?~]+', '', text))


def perform(action, bot, sleep=time.sleep):
    """동작 하나 실행. bot 은 Hardware 와 같은 모양. 바퀴는 오류가 나도 0으로 돌아온다."""
    if action in MOVES:
        try:
            bot.set_wheels(tuple(SPEED * sign for sign in MOVES[action]))
            sleep(SECONDS)
        finally:
            errors = bot.stop_all()
        if errors:
            raise OSError('정지 통신 실패: ' + '; '.join(errors))
    elif action == 'stop':
        errors = bot.stop_all()
        if errors:
            raise OSError('정지 통신 실패: ' + '; '.join(errors))
    elif action == 'beep':
        try:
            bot.beep(True)
            sleep(0.3)
        finally:
            bot.beep(False)
    elif action == 'light_off':
        bot.light(None)
    elif action.startswith('light_'):
        bot.light(COLORS[action.removeprefix('light_')])
    else:
        raise ValueError(f'목록에 없는 동작: {action}')


def main():
    drive = sys.argv[1:] == ['--drive']
    if sys.argv[1:] and not drive:
        raise SystemExit('옵션은 --drive 만 있습니다.')
    bot = None
    if drive:
        if not sys.stdin.isatty():
            raise SystemExit('VS Code 터미널에서 직접 실행해 주세요.')
        if input('라즈봇을 바닥에 내려놓고 사방 1m를 비웠나요? 준비됐다면 "확인" 입력: ').strip() != '확인':
            print('취소 — 바퀴 명령 없음')
            return
        sys.path.insert(0, __file__.rsplit('/missions/', 1)[0] + '/tools')
        from autodrive_runtime import Hardware
        bot = Hardware(use_ultrasonic=False)
    print('문장을 입력하세요. 끝내려면 빈 줄 또는 Ctrl+D.' + ('' if drive else ' (해석만 합니다)'))
    try:
        while True:
            try:
                text = input('> ').strip()
            except EOFError:
                break
            if not text:
                break
            action = choose(text)
            if action is None:
                print('거절 — 시험한 동작 목록에 없습니다.')
                continue
            print('동작:', action)
            if bot is not None:
                perform(action, bot)
    finally:
        if bot is not None:
            bot.close()


if __name__ == '__main__':
    main()
