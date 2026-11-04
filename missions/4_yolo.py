#!/usr/bin/env python3
# 지시서: specs/4-욜로.md
"""4단계: YOLO로 사람 · STOP 표지 찾기. 바퀴는 --drive 때만 돈다.

  python3 missions/4_yolo.py                   10초 동안 본 것 출력 (바퀴 없음)
  python3 missions/4_yolo.py --capture person  Enter 칠 때마다 사진 저장 (person / stop / none)
  python3 missions/4_yolo.py --eval            모은 사진에서 맞힌 개수 세기
  python3 missions/4_yolo.py --drive --session 6 --attempt 1   달리면서 반응
"""
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from autodrive_core import Config

ROOT = Path(__file__).resolve().parents[1]
PHOTOS = ROOT / 'results' / 'yolo-photos'
TZ = ZoneInfo('Asia/Seoul')
TARGET = {'person': 'person', 'stop': 'stop sign'}
CFG = Config(use_obstacle=False, use_feature=False)


def hits(found, min_score=CFG.min_score):
    return {name for name, score, _ in found if score >= min_score}


def score_photo(label, names):
    """정답 판정: person/stop 폴더는 그 물체를 찾으면, none 폴더는 둘 다 없으면 맞힌 것."""
    if label == 'none':
        return not (names & set(TARGET.values()))
    return TARGET[label] in names


def evaluate(folder, detector, read_image):
    rows = []
    for label in ('person', 'stop', 'none'):
        for path in sorted((folder / label).glob('*')):
            if path.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
                continue
            image = read_image(str(path))
            if image is None:
                continue
            found = detector(image)
            top = max((d[2][3] for d in found if d[0] == TARGET.get(label)), default=0)
            rows.append({'label': label, 'file': path.name, 'hit': score_photo(label, hits(found)),
                         'box_height': top, 'found': ' '.join(f'{n}:{s:.2f}' for n, s, _ in found)})
    return rows


def watch(seconds=10):
    from yolo import Detector, open_camera
    det, cap = Detector(), open_camera()
    try:
        end = time.monotonic() + seconds
        print(f'{seconds}초 동안 본 것을 출력합니다. 바퀴는 움직이지 않습니다.')
        while time.monotonic() < end:
            ok, frame = cap.read()
            if ok:
                print([(n, round(s, 2), f'h={b[3]}') for n, s, b in det(frame)] or '없음', flush=True)
    finally:
        cap.release()


def capture(label):
    import cv2
    from yolo import open_camera
    if label not in ('person', 'stop', 'none'):
        raise SystemExit('사진 종류는 person, stop, none 중 하나입니다.')
    folder = PHOTOS / label
    folder.mkdir(parents=True, exist_ok=True)
    cap = open_camera()
    try:
        for _ in range(10):
            cap.read()
        while input('Enter = 한 장 찍기, q = 끝: ').strip().lower() != 'q':
            for _ in range(3):  # 묵은 화면을 버린다
                ok, frame = cap.read()
            if not ok:
                raise SystemExit('카메라를 읽지 못했습니다.')
            path = folder / (datetime.now(TZ).strftime('%H%M%S-%f') + '.jpg')
            cv2.imwrite(str(path), frame)
            print('저장:', path.relative_to(ROOT), '· 지금까지', len(list(folder.glob('*.jpg'))), '장')
    finally:
        cap.release()


def run_eval():
    import cv2
    from yolo import Detector
    rows = evaluate(PHOTOS, Detector(), cv2.imread)
    if not rows:
        raise SystemExit(f'사진이 없습니다. {PHOTOS.relative_to(ROOT)}/person · stop · none 에 넣어 주세요.')
    for label in ('person', 'stop', 'none'):
        part = [r for r in rows if r['label'] == label]
        print(f'{label}: {sum(r["hit"] for r in part)} / {len(part)}')
    print(f'합계: {sum(r["hit"] for r in rows)} / {len(rows)}')
    out = ROOT / 'results' / datetime.now(TZ).strftime('yolo-eval-%Y%m%d-%H%M%S.csv')
    with open(out, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print('자세한 표:', out.relative_to(ROOT))


def main():
    argv = sys.argv[1:]
    if not argv:
        watch()
    elif argv[0] == '--capture' and len(argv) == 2:
        capture(argv[1])
    elif argv == ['--eval']:
        run_eval()
    else:
        from autodrive_runtime import cli
        cli(CFG, '4_yolo', 'specs/4-욜로.md', argv)


if __name__ == '__main__':
    main()
