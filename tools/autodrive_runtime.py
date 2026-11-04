#!/usr/bin/env python3
# 지시서: specs/3-장애물.md · specs/4-욜로.md · specs/5-오토드라이브.md
"""오토드라이브 실행부: 센서 읽기 · 바퀴 · 블랙박스 · 기록표.

  (옵션 없음)   설명만 출력. 장치 접근 없음
  --simulate   가짜 센서로 한 판. 카메라·YOLO·녹화는 진짜, 바퀴·I2C 접근 없음. 기록표에 안 남김
  --drive      실제 주행. 대화형 터미널에서 「확인」을 입력해야 시작
"""
import argparse
import csv
import json
import os
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from autodrive_core import (DistanceFilter, Obs, Perception, StateMachine, STOP,
                            MAX_WHEEL_SPEED, line_black)

ROOT = Path(__file__).resolve().parents[1]
TZ = ZoneInfo('Asia/Seoul')
RUNS_CSV = ROOT / 'results' / 'runs.csv'
RUNS_HEADER = ['회차', '코드 버전', '시도', '완주', '이탈', '시간', '손댄 횟수', '메모']
VIDEO_FPS = 15
LOOP_SECONDS = 0.02
ULTRASONIC_EVERY = 3


class Hardware:
    """라즈봇 장치. 드라이버가 숨기는 I2C 오류도 예외로 받도록 버스를 직접 쓴다."""

    def __init__(self, use_ultrasonic):
        from Raspbot_Lib import Raspbot
        self.bot = Raspbot()
        self.dev, self.addr = self.bot._device, self.bot._addr
        self.use_ultrasonic = use_ultrasonic
        # 카메라 각도 — specs/4-욜로.md (드라이버 함수: 서보는 정지 안전과 무관)
        self.bot.Ctrl_Servo(1, 90)
        self.bot.Ctrl_Servo(2, 27)
        if use_ultrasonic:
            self.dev.write_i2c_block_data(self.addr, 0x07, [1])
            time.sleep(1)

    def read_line(self):
        return self.dev.read_i2c_block_data(self.addr, 0x0a, 1)[0]

    def read_distance_mm(self):
        high = self.dev.read_i2c_block_data(self.addr, 0x1b, 1)[0]
        low = self.dev.read_i2c_block_data(self.addr, 0x1a, 1)[0]
        return high << 8 | low

    def set_wheels(self, speeds):
        for motor, speed in enumerate(speeds):
            if abs(speed) > MAX_WHEEL_SPEED:
                raise ValueError('바퀴 속도 150 초과')
            self.dev.write_i2c_block_data(self.addr, 0x01, [motor, int(speed < 0), abs(speed)])

    def stop_all(self):
        errors = []
        for motor in range(4):
            try:
                self.dev.write_i2c_block_data(self.addr, 0x01, [motor, 0, 0])
            except OSError as exc:
                errors.append(f'바퀴 {motor}: {exc}')
        return errors

    def light(self, color):
        self.dev.write_i2c_block_data(self.addr, 0x03, [0, 0] if color is None else [1, color])

    def beep(self, on):
        self.dev.write_i2c_block_data(self.addr, 0x06, [1 if on else 0])

    def close(self):
        """정리 도중 하나가 실패해도 나머지를 끝까지 시도한다."""
        errors = self.stop_all()
        steps = [lambda: self.beep(False), lambda: self.light(None)]
        if self.use_ultrasonic:
            steps.append(lambda: self.dev.write_i2c_block_data(self.addr, 0x07, [0]))
        for step in steps:
            try:
                step()
            except OSError as exc:
                errors.append(str(exc))
        self.dev.close()
        return errors


class SimHardware:
    """정해진 줄거리대로 값을 돌려주는 가짜 장치. 바퀴 명령은 기록만 한다."""
    CENTER, LEFT, LOST, BLACK = 0b0101, 0b1011, 0b1111, 0b0000

    def __init__(self, clock=time.monotonic):
        self.clock, self.t0 = clock, clock()
        self.wheel_log = []
        self.stops = 0

    def _t(self):
        return self.clock() - self.t0

    def read_line(self):
        t = self._t()
        if t >= 27:
            return self.BLACK
        if 22 <= t < 23:
            return self.LEFT
        if 23.5 <= t < 23.7:
            return self.LOST
        return self.CENTER

    def read_distance_mm(self):
        t = self._t()
        if 6 <= t < 8:
            return 100
        if 8 <= t < 10:
            return 300
        return 1000

    def set_wheels(self, speeds):
        self.wheel_log.append((round(self._t(), 3), tuple(speeds)))

    def stop_all(self):
        self.stops += 1
        self.wheel_log.append((round(self._t(), 3), STOP))
        return []

    def light(self, color):
        pass

    def beep(self, on):
        pass

    def close(self):
        return self.stop_all()


def sim_detections(t):
    """--simulate 전용 가짜 YOLO 결과 (상자 높이 포함)."""
    if 12 <= t < 13:
        return [('stop sign', 0.9, (300, 100, 60, 60))]
    if 18 <= t < 20:
        return [('person', 0.9, (200, 100, 150, 300))]
    return []


class Vision:
    """카메라 · 녹화 스레드와 YOLO 스레드. 주행 쪽은 latest() 로 읽기만 한다."""

    def __init__(self, path, use_yolo, fake_detections=None):
        self.path, self.use_yolo, self.fake_detections = str(path), use_yolo, fake_detections
        self.lock = threading.Lock()
        self.new_frame = threading.Event()
        self.stopping = threading.Event()
        self.frame, self.frame_t = None, 0.0
        self.seq, self.detections, self.det_t = 0, [], 0.0
        self.status = 'START'
        self.error = None
        self.start_t = None
        self.frames_written = 0
        self.threads = []

    def start(self):
        self.threads = [threading.Thread(target=self._camera_loop, daemon=True)]
        if self.use_yolo:
            self.threads.append(threading.Thread(target=self._detect_loop, daemon=True))
        for thread in self.threads:
            thread.start()

    def wait_ready(self, timeout=30):
        end = time.monotonic() + timeout
        while time.monotonic() < end and self.error is None:
            with self.lock:
                ready = self.frame is not None and (not self.use_yolo or self.seq > 0)
            if ready:
                return True
            time.sleep(0.05)
        return False

    def latest(self):
        with self.lock:
            return self.seq, list(self.detections), self.det_t, self.frame_t

    def set_status(self, text):
        with self.lock:
            self.status = text

    def stop(self):
        self.stopping.set()
        self.new_frame.set()
        for thread in self.threads:
            thread.join(timeout=10)

    def _camera_loop(self):
        import cv2
        from yolo import draw, open_camera
        cap, writer = None, None
        try:
            cap = open_camera()
            for _ in range(10):  # 켜자마자 몇 장은 어둡다
                ok, frame = cap.read()
            if not ok:
                raise RuntimeError('카메라를 읽지 못했습니다. USB 선을 확인하세요.')
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'avc1'), VIDEO_FPS, (w, h))
            if not writer.isOpened():
                raise RuntimeError('영상 저장기를 열 수 없습니다.')
            self.start_t = time.monotonic()
            while not self.stopping.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('촬영 도중 카메라 연결이 끊겼습니다.')
                now = time.monotonic()
                with self.lock:
                    self.frame, self.frame_t = frame, now
                    detections, status = list(self.detections), self.status
                self.new_frame.set()
                shown = draw(frame.copy(), detections)
                # 영상 위 글자는 영어와 숫자만 (OpenCV 는 한글을 못 쓴다)
                stamp = datetime.now(TZ).strftime('%Y-%m-%d %H:%M:%S')
                cv2.rectangle(shown, (0, 0), (w, 54), (0, 0, 0), -1)
                cv2.putText(shown, f'TEAM2 {stamp}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(shown, status, (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                # 저장 장수를 실제 시간에 맞춘다 (빨리 감기처럼 나오지 않게)
                target = int((time.monotonic() - self.start_t) * VIDEO_FPS) + 1
                while self.frames_written < target:
                    writer.write(shown)
                    self.frames_written += 1
        except Exception as exc:
            self.error = str(exc) or type(exc).__name__
        finally:
            if writer is not None:
                writer.release()
            if cap is not None:
                cap.release()

    def _detect_loop(self):
        from yolo import Detector
        try:
            detector = Detector()
            while not self.stopping.is_set():
                if not self.new_frame.wait(timeout=1):
                    continue
                self.new_frame.clear()
                with self.lock:
                    frame = self.frame
                if frame is None or self.stopping.is_set():
                    continue
                found = detector(frame)
                if self.fake_detections is not None:
                    found = self.fake_detections(time.monotonic() - self.sim_t0) if self.sim_t0 else []
                with self.lock:
                    self.detections, self.det_t = found, time.monotonic()
                    self.seq += 1
        except Exception as exc:
            self.error = str(exc) or type(exc).__name__

    sim_t0 = None


def run(cfg, hw, vision, key_pressed, report, clock=time.monotonic, sleep=time.sleep):
    """한 판. 어떤 이유로 끝나든 finally 에서 네 바퀴를 0으로 만든다."""
    started = None
    sm = None
    try:
        errors = hw.stop_all()
        if errors:
            raise OSError('시작 전 정지 통신 실패: ' + '; '.join(errors))
        started = clock()
        sm = StateMachine(cfg, started)
        distance = DistanceFilter(cfg.distance_fault_seconds)
        perception = Perception(cfg)
        current, last_write, shown_light, beep_until, tick = None, 0.0, 'unset', None, 0
        max_gap, previous = 0.0, None
        while True:
            now = clock()
            if previous is not None:
                max_gap = max(max_gap, now - previous)
            previous = now
            if key_pressed():
                report['reason'] = 'KEY_STOP'
                break
            raw = hw.read_line()
            if cfg.use_obstacle and tick % ULTRASONIC_EVERY == 0:
                distance.add(hw.read_distance_mm(), now)
            tick += 1
            seq, detections, det_t, frame_t = vision.latest()
            perception.update(seq, detections)
            age = now - min(frame_t, det_t) if cfg.use_yolo else now - frame_t
            action = sm.step(Obs(t=now, line_raw=raw, distance_cm=distance.value(),
                                 distance_fault=distance.fault(now),
                                 stop_sign=perception.confirmed('stop'),
                                 person_close=perception.confirmed('close'),
                                 person_far=perception.confirmed('far'),
                                 vision_age=age))
            # 눌린 키는 이동 명령을 보내기 직전에 한 번 더 본다
            if key_pressed():
                report['reason'] = 'KEY_STOP'
                break
            if action.speeds != current or now - last_write >= 0.5:
                hw.set_wheels(action.speeds)
                current, last_write = action.speeds, now
            if action.light != shown_light:
                hw.light(action.light)
                shown_light = action.light
            if action.beep_seconds > 0:
                hw.beep(True)
                beep_until = now + action.beep_seconds
            elif beep_until is not None and now >= beep_until:
                hw.beep(False)
                beep_until = None
            bits = ''.join('B' if b else '-' for b in line_black(raw))
            dist = distance.value()
            names = ','.join(sorted({d[0] for d in detections})) or 'none'
            vision.set_status(f'{action.state} L{action.speeds[0]} R{action.speeds[2]} line {bits} '
                              f'{"--" if dist is None else f"{dist:.0f}cm"} see {names}')
            if action.done:
                report['reason'] = action.reason
                if action.beep_seconds > 0:
                    sleep(action.beep_seconds)  # 바퀴는 이미 0
                break
            sleep(LOOP_SECONDS)
    except BaseException as exc:
        report['reason'] = 'INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'ERROR'
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        report['stop_errors'] = hw.stop_all()
        if started is not None:
            report['elapsed_seconds'] = round(clock() - started, 3)
        if sm is not None:
            report['off_line_count'] = sm.off_line_count
            report['transitions'] = sm.transitions
            report['final_state'] = sm.state
            report['max_loop_gap_ms'] = round(max_gap * 1000, 1)


def code_version():
    try:
        head = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', 'status', '--porcelain', '--', 'missions', 'tools'], cwd=ROOT,
                               capture_output=True, text=True, check=True).stdout.strip()
        return head + ('-dirty' if dirty else '')
    except (OSError, subprocess.CalledProcessError):
        return 'no-git'


def append_run(path, row):
    new = not path.exists() or path.stat().st_size == 0
    with open(path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if new:
            writer.writerow(RUNS_HEADER)
        writer.writerow(row)


def check_video(path):
    import cv2
    cap = cv2.VideoCapture(str(path))
    frames, fps = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    return {'frames': frames, 'fps': fps, 'seconds': round(frames / fps, 2) if fps else 0}


def execute(cfg, program, hw, key_pressed, fake_detections=None):
    """장치 준비 · 한 판 · 정리. 기록 파일을 남기고 report 를 돌려준다."""
    stamp = datetime.now(TZ).strftime('%Y%m%d-%H%M%S')
    prefix = 'sim-' if fake_detections is not None else ''
    video = ROOT / 'results' / 'videos' / f'{prefix}{program}-{stamp}.mp4'
    video.parent.mkdir(parents=True, exist_ok=True)
    report = {'team': '2조', 'program': program, 'simulated': fake_detections is not None,
              'video': str(video.relative_to(ROOT)), 'config': cfg.__dict__, 'code_version': code_version()}
    vision = Vision(video, cfg.use_yolo, fake_detections)
    try:
        vision.start()
        print('카메라와 인식 준비 중...', flush=True)
        if not vision.wait_ready():
            raise RuntimeError('카메라/YOLO 준비 실패: ' + str(vision.error))
        raw = hw.read_line()
        print(f'출발 전 센서: 라인 {raw:04b}', flush=True)
        if not any(line_black(raw)):
            print('주의 — 줄이 센서 아래에 없습니다. 이대로면 0.5초 뒤 비상 정지합니다.')
        if cfg.use_obstacle:
            mm = hw.read_distance_mm()
            print(f'출발 전 초음파: {mm / 10:.1f} cm', flush=True)
            if 0 < mm / 10 <= cfg.obstacle_stop_cm:
                print('주의 — 앞이 막혀 있다고 읽힙니다. 치울 때까지 출발하지 않습니다.')
        print('출발 신호 — 삐', flush=True)
        hw.beep(True)
        time.sleep(0.3)
        hw.beep(False)
        time.sleep(0.7)
        if fake_detections is not None:
            # 모의 줄거리는 출발 순간을 0초로 맞춘다
            vision.sim_t0 = hw.t0 = time.monotonic()
        run(cfg, hw, vision, key_pressed, report)
    finally:
        report['stop_errors_final'] = hw.close()
        vision.stop()
        report['vision_error'] = vision.error
        if video.exists():
            report['video_check'] = check_video(video)
        folder = ROOT / 'results' / 'runs'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f'{prefix}{program}-{stamp}.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str) + '\n', encoding='utf-8')
        report['report_path'] = str(path.relative_to(ROOT))
    return report


def print_summary(report):
    print('\n끝난 이유:', report.get('reason', '시작 전 종료'))
    print('시간(초):', report.get('elapsed_seconds'), '· 이탈:', report.get('off_line_count'))
    print('영상:', report['video'], report.get('video_check'))
    print('기록:', report.get('report_path'))
    if report.get('stop_errors') or report.get('stop_errors_final'):
        print('이상 — 정지 통신 실패. 바퀴가 돌면 확장보드 전원 스위치를 꺼 주세요.')
    if report.get('vision_error'):
        print('이상 — 카메라:', report['vision_error'])


def ask_record(report, session, attempt):
    """완주와 손댄 횟수는 사람만 안다. 눈으로 본 대로 입력받아 기록표에 한 줄 남긴다."""
    finished = report.get('reason') == 'FINISH_LINE'
    print(f'\n프로그램 판단: {"결승선 도착" if finished else "도착 못 함"}. 실제로 본 대로 입력하세요.')
    done = input('완주했나요? (예/아니오): ').strip() or '아니오'
    touches = input('사람이 손댄 횟수 (숫자): ').strip() or '?'
    note = input('메모 (조명 · 배터리 · 특이 사항): ').strip()
    note = f'{note} [{report.get("reason")}]'.strip()
    append_run(RUNS_CSV, [session, report['code_version'], attempt, done,
                          report.get('off_line_count'), report.get('elapsed_seconds'), touches, note])
    print('기록표에 저장:', RUNS_CSV.relative_to(ROOT))


def cli(cfg, program, spec, argv=None):
    parser = argparse.ArgumentParser(description=f'{program} — 지시서 {spec}')
    parser.add_argument('--drive', action='store_true', help='실제 주행 (대화형 터미널 필요)')
    parser.add_argument('--simulate', action='store_true', help='가짜 센서로 한 판. 바퀴·I2C 접근 없음')
    parser.add_argument('--session', default='', help='회차 (예: 4)')
    parser.add_argument('--attempt', default='', help='시도 번호 1~5')
    args = parser.parse_args(argv)
    if args.simulate:
        report = execute(cfg, program, SimHardware(), lambda: False,
                         fake_detections=sim_detections if cfg.use_yolo else (lambda t: []))
        print_summary(report)
        return report
    if not args.drive:
        print(f'{program} — 지시서 {spec}\n'
              f'  가짜 센서 시험:  python3 missions/{program}.py --simulate\n'
              f'  실제 주행:       python3 missions/{program}.py --drive --session 회차 --attempt 시도')
        return None
    if not sys.stdin.isatty():
        raise SystemExit('아무 키 정지를 위해 VS Code 터미널에서 직접 실행해야 합니다.')
    if not args.session or not args.attempt:
        raise SystemExit('기록표에 남기려면 --session 회차 --attempt 시도 를 함께 적어 주세요.')
    print('아무 키나 누르면 바로 멈춥니다. 처음에는 항상 천천히, 손이 닿는 곳에서 지켜보세요.')
    if input('라즈봇을 바닥에 내려놓고 사방 1m를 비웠나요? 준비됐다면 "확인" 입력: ').strip() != '확인':
        print('취소 — 바퀴 명령 없음')
        return None
    terminal = termios.tcgetattr(sys.stdin)
    old_handler = signal.getsignal(signal.SIGTERM)

    def interrupt(signum, frame):
        raise KeyboardInterrupt('종료 신호')

    def key_pressed():
        return bool(select.select([sys.stdin], [], [], 0)[0])

    report = None
    try:
        signal.signal(signal.SIGTERM, interrupt)
        tty.setcbreak(sys.stdin.fileno())
        report = execute(cfg, program, Hardware(cfg.use_obstacle), key_pressed)
    finally:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        signal.signal(signal.SIGTERM, old_handler)
    print_summary(report)
    ask_record(report, args.session, args.attempt)
    return report
