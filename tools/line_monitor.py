#!/usr/bin/env python3
"""바퀴 명령 없이 라인센서만 60초 동안 표시·기록한다."""
import json
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def main():
    import smbus
    records = []
    bus = smbus.SMBus(1)
    start = time.monotonic()
    print('바퀴는 움직이지 않습니다. 0=검정, 1=흰색. Ctrl+C로 종료.', flush=True)
    print('왼쪽부터 x2 · x1 · x3 · x4 순서로 가정합니다. 실제 위치를 확인하세요.', flush=True)
    try:
        while time.monotonic() - start < 60:
            raw = bus.read_i2c_block_data(0x2b, 0x0a, 1)[0]
            bits = [(raw >> bit) & 1 for bit in (2, 3, 1, 0)]
            records.append({'elapsed': round(time.monotonic() - start, 3),
                            'raw': raw, 'assumed_left_to_right': bits})
            print('왼쪽부터: ' + '  '.join(map(str, bits)), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('점검 종료')
    finally:
        bus.close()
        path = Path(__file__).resolve().parents[1] / 'results' / datetime.now(
            ZoneInfo('Asia/Seoul')).strftime('line-monitor-%Y%m%d-%H%M%S-%f.json')
        path.write_text(json.dumps(records, ensure_ascii=False, indent=2) + '\n')
        print('기록: ' + str(path))


if __name__ == '__main__':
    main()
