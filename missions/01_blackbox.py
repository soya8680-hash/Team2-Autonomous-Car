#!/usr/bin/env python3
# 지시서: specs/01-블랙박스.md
"""2조 정지 상태 블랙박스: 시각·초음파 거리를 넣어 10초 MP4 저장."""
import json
import subprocess
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from Raspbot_Lib import Raspbot

ROOT = Path(__file__).resolve().parents[1]
TZ = ZoneInfo('Asia/Seoul')


def main():
    folder = ROOT / 'results' / datetime.now(TZ).strftime('blackbox-%Y%m%d-%H%M%S-%f')
    folder.mkdir(parents=True)
    raw, output = folder / 'capture.avi', folder / '2jo-blackbox.mp4'
    font = ImageFont.truetype(str(ROOT / 'tools/fonts/NanumGothic-Regular.ttf'), 24)
    bot, cap, writer = None, None, None
    samples = []
    metrics = {'team': '2조', 'target_seconds': 10, 'fps': 20, 'status': 'failed'}
    try:
        bot = Raspbot()
        # 직접 버스 호출: 드라이버가 삼키는 I2C 오류도 반드시 예외로 받는다.
        bot._device.write_i2c_block_data(bot._addr, 0x07, [1])
        time.sleep(1)
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(5):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('카메라 읽기 실패: USB 연결을 확인하세요.')
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*'MJPG'), 20, (width, height))
        if not writer.isOpened():
            raise RuntimeError('영상 저장기를 열 수 없습니다.')
        print('10초 촬영 시작 — 바퀴는 움직이지 않습니다.', flush=True)
        start, written = time.monotonic(), 0
        while written < 200:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('촬영 도중 카메라 연결이 끊겼습니다.')
            elapsed = time.monotonic() - start
            try:
                high = bot._device.read_i2c_block_data(bot._addr, 0x1b, 1)[0]
                low = bot._device.read_i2c_block_data(bot._addr, 0x1a, 1)[0]
                distance = (high << 8 | low) / 10
                label = f'{distance:.1f} cm' if distance > 0 else '측정 불가'
                sensor_error = None
            except OSError as exc:
                distance, label, sensor_error = None, '센서 오류', str(exc)
            stamp = datetime.now(TZ).isoformat(timespec='milliseconds')
            samples.append({'elapsed_seconds': round(elapsed, 4), 'time': stamp,
                            'distance_cm': distance, 'sensor_error': sensor_error})
            panel = Image.new('RGB', (width, 78), (0, 0, 0))
            draw = ImageDraw.Draw(panel)
            draw.text((10, 2), '2조 | ' + stamp[:19].replace('T', ' ') + ' KST', font=font, fill='white')
            draw.text((10, 38), '거리: ' + label, font=font, fill='white')
            frame[:78] = cv2.cvtColor(np.asarray(panel), cv2.COLOR_RGB2BGR)
            # 늦어진 프레임 구간은 복제해 실제 시간과 재생 시간이 맞도록 한다.
            target = min(200, int((time.monotonic() - start) * 20) + 1)
            while written < target:
                writer.write(frame)
                written += 1
            if written < 200:
                time.sleep(max(0, start + written / 20 - time.monotonic()))
        time.sleep(max(0, start + 10 - time.monotonic()))
        metrics.update(capture_seconds=time.monotonic() - start, frames=written,
                       camera_samples=len(samples), resolution=[width, height])
        writer.release()
        writer = None
        cap.release()
        cap = None
        subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(raw),
                        '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)], check=True)
        metrics['probe'] = json.loads(subprocess.check_output(
            ['ffprobe', '-v', 'error', '-show_entries',
             'format=duration:stream=codec_name,pix_fmt,width,height,nb_frames',
             '-of', 'json', str(output)], text=True))
        metrics['status'] = 'recorded'
        raw.unlink()
        print('저장: ' + str(output), flush=True)
        print('재생 길이: ' + metrics['probe']['format']['duration'] + '초', flush=True)
    except BaseException as exc:
        metrics['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        if writer is not None:
            writer.release()
        if cap is not None:
            cap.release()
        if bot is not None:
            try:
                bot._device.write_i2c_block_data(bot._addr, 0x07, [0])
            except OSError as exc:
                metrics['cleanup_error'] = str(exc)
            finally:
                bot._device.close()
        (folder / 'measurements.json').write_text(json.dumps(samples, ensure_ascii=False, indent=2) + '\n')
        (folder / 'report.json').write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
