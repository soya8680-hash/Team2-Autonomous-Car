#!/usr/bin/env python3
# 지시서: specs/6-트랙주행.md
"""트랙 주행 시험. 가짜 값만 쓰고 장치에 접근하지 않는다."""
import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from autodrive_core import (Config, EMERGENCY, FOLLOW, Lane, LaneTracker, Obs, StateMachine, STOP,
                            STOP_SIGN, WAIT_OBSTACLE, WAIT_PERSON, LANE_NONE, LANE_SEARCH, LANE_TRACK,
                            LANE_WAIT, lane_speeds)
import autodrive_runtime as rt


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'missions' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def image(left=None, right=None):
    frame = np.zeros((480, 640, 3), np.uint8)
    for x in (left, right):
        if x is not None:
            frame[120:300, x - 3:x + 4] = (0, 220, 240)  # BGR 노랑
    return frame


CFG = Config(use_camera_lane=True, use_feature=False)
TRACK = Lane(LANE_TRACK, (40, 40, 30, 30), 332.0)
SEARCH = Lane(LANE_SEARCH, (60, 60, 0, 0))


def obs(t, lane=TRACK, dist=100.0, **kw):
    return Obs(t=t, line_raw=0b1111, distance_cm=dist, lane=lane, **kw)


class FindLane(unittest.TestCase):
    def test_finds_both_one_none(self):
        self.assertEqual(rt.find_lane(image(160, 480)), (160.0, 480.0))
        self.assertEqual(rt.find_lane(image(160)), (160.0, None))
        self.assertEqual(rt.find_lane(image(None, 480)), (None, 480.0))
        self.assertEqual(rt.find_lane(image()), (None, None))

    def test_ignores_outside_band_and_other_colors(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[:120, 100:120] = frame[300:, 100:120] = (0, 220, 240)
        frame[120:300, 500:520] = (255, 255, 255)   # 흰 섬
        frame[120:300, 200:220] = (200, 100, 0)     # 파란 구간
        self.assertEqual(rt.find_lane(frame), (None, None))

    def test_needs_50_points(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[120:169, 100] = (0, 220, 240)
        self.assertIsNone(rt.find_lane(frame)[0])
        frame[169, 100] = (0, 220, 240)
        self.assertEqual(rt.find_lane(frame)[0], 100.0)

    def test_bad_size_rejected(self):
        with self.assertRaises(ValueError):
            rt.find_lane(np.zeros((720, 1280, 3), np.uint8))

    def test_same_answer_as_mission_07(self):
        lane07 = load('lane07', '07_camera_lane.py')
        for left, right in ((160, 480), (100, None), (None, 500), (None, None)):
            frame = image(left, right)
            found = lane07.find_lines(frame)
            self.assertEqual(rt.find_lane(frame), (found['left'], found['right']))
        for center in (0, 100, 300, 320, 345, 640):
            self.assertEqual(lane_speeds(center), lane07.wheel_speeds(center))


class Tracker(unittest.TestCase):
    def test_center_and_direction(self):
        self.assertEqual(lane_speeds(320), (35,) * 4)
        self.assertGreater(lane_speeds(400)[0], lane_speeds(400)[2])  # 오른쪽으로 가려면 왼쪽 바퀴가 빠르다
        for center in (-500, 1500):
            self.assertTrue(all(0 <= s <= 80 for s in lane_speeds(center)))

    def test_one_line_needs_known_width(self):
        t = LaneTracker()
        self.assertEqual(t.update(100, None).kind, LANE_WAIT)
        self.assertEqual(t.update(100, 500).center, 300)
        self.assertEqual(t.update(100, None).center, 300)
        self.assertEqual(t.update(None, 500).center, 300)

    def test_lost_uses_last_turn(self):
        t = LaneTracker()
        self.assertEqual(t.update(None, None).kind, LANE_NONE)
        t.update(200, 600)  # 중심 400: 우회전
        self.assertEqual(t.update(None, None).speeds, (60, 60, 0, 0))
        t.update(100, 500)  # 중심 300: 좌회전
        self.assertEqual(t.update(None, None).speeds, (0, 0, 60, 60))
        t.update(120, 520)  # 똑바로 가도 방향은 남는다
        self.assertEqual(t.update(None, None).speeds, (0, 0, 60, 60))


class States(unittest.TestCase):
    def setUp(self):
        self.sm = StateMachine(CFG, 0.0)

    def test_follows_lane_and_ignores_all_black_road(self):
        for t in (0.1, 6.0, 6.1, 6.2, 6.3):  # 5_autodrive 라면 결승선으로 보는 조건
            a = self.sm.step(Obs(t=t, line_raw=0b0000, distance_cm=100.0, lane=TRACK))
            self.assertEqual((a.state, a.speeds), (FOLLOW, TRACK.speeds))

    def test_obstacle_person_stop_override_lane(self):
        self.assertEqual(self.sm.step(obs(0.1, dist=15.0)).state, WAIT_OBSTACLE)
        self.assertEqual(self.sm.step(obs(0.2, dist=18.0)).speeds, STOP)
        self.assertEqual(self.sm.step(obs(0.3, dist=20.0)).speeds, TRACK.speeds)
        self.assertEqual(self.sm.step(obs(0.4, person_close=True)).state, WAIT_PERSON)
        self.assertEqual(self.sm.step(obs(0.5)).speeds, STOP)
        self.assertEqual(self.sm.step(obs(1.6)).speeds, TRACK.speeds)
        a = self.sm.step(obs(1.7, stop_sign=True))
        self.assertEqual((a.state, a.speeds), (STOP_SIGN, STOP))
        self.assertEqual(self.sm.step(obs(4.6, stop_sign=True)).speeds, STOP)
        self.assertEqual(self.sm.step(obs(4.8, stop_sign=True)).speeds, TRACK.speeds)

    def test_no_lines_from_start_is_emergency(self):
        a = self.sm.step(obs(0.1, Lane(LANE_NONE)))
        self.assertEqual((a.state, a.speeds, a.reason), (EMERGENCY, STOP, 'LOST_LINE'))

    def test_width_wait_stops_without_ending(self):
        a = self.sm.step(obs(0.1, Lane(LANE_WAIT)))
        self.assertEqual((a.state, a.speeds, a.done), (FOLLOW, STOP, False))

    def test_search_one_second_then_emergency(self):
        self.assertEqual(self.sm.step(obs(1.0, SEARCH)).speeds, SEARCH.speeds)
        self.assertEqual(self.sm.step(obs(1.99, SEARCH)).speeds, SEARCH.speeds)
        a = self.sm.step(obs(2.0, SEARCH))
        self.assertEqual((a.state, a.speeds, a.reason), (EMERGENCY, STOP, 'LOST_LINE'))
        self.assertEqual(self.sm.step(obs(2.1)).speeds, STOP)  # 다시 안 달린다

    def test_seeing_line_resets_search(self):
        self.sm.step(obs(1.0, SEARCH))
        self.sm.step(obs(1.9))
        self.assertEqual(self.sm.step(obs(2.5, SEARCH)).speeds, SEARCH.speeds)

    def test_time_stopped_is_not_lost_time(self):
        self.sm.step(obs(1.0, SEARCH))
        self.sm.step(obs(1.5, SEARCH, dist=10.0))
        self.sm.step(obs(9.0, SEARCH, dist=10.0))
        self.assertEqual(self.sm.step(obs(9.1, SEARCH, dist=30.0)).speeds, SEARCH.speeds)
        self.assertEqual(self.sm.step(obs(10.1, SEARCH, dist=30.0)).state, EMERGENCY)

    def test_stale_frame_and_time_limit_stop(self):
        self.assertEqual(self.sm.step(obs(0.1)).state, FOLLOW)
        a = self.sm.step(obs(0.2, lane_age=0.51))
        self.assertEqual((a.state, a.reason), (EMERGENCY, 'CAMERA_FAULT'))
        a = StateMachine(CFG, 0.0).step(obs(180.0))
        self.assertEqual((a.state, a.reason), (EMERGENCY, 'TIME_LIMIT'))

    def test_old_autodrive_unchanged(self):
        sm = StateMachine(Config(), 0.0)
        self.assertEqual(sm.step(Obs(t=0.1, line_raw=0b0101, distance_cm=100.0)).speeds, (60,) * 4)
        self.assertEqual(Config().camera_tilt, 40)


class FakeVision:
    def __init__(self, frames):
        self.frames, self.n, self.t = frames, 0, 0.0

    def latest(self):
        return 0, [], self.t, self.t

    def latest_frame(self):
        frame = self.frames[min(self.n, len(self.frames) - 1)]
        self.n += 1
        return frame, self.n, self.t

    def set_status(self, text):
        self.status = text


class FakeHw:
    def __init__(self):
        self.wheels, self.stops = [], 0

    def read_line(self):
        raise AssertionError('트랙 주행은 라인센서를 읽지 않는다')

    def read_distance_mm(self):
        return 1000

    def set_wheels(self, speeds):
        self.wheels.append(tuple(speeds))

    def stop_all(self):
        self.stops += 1
        return []

    def light(self, color):
        pass

    def beep(self, on):
        pass


class RunLoop(unittest.TestCase):
    def drive(self, frames, keys, cfg=CFG):
        hw, vision, clock = FakeHw(), FakeVision(frames), [0.0]

        def tick():
            clock[0] += 0.02
            vision.t = clock[0]
            return clock[0]

        report = {}
        rt.run(cfg, hw, vision, keys, report, clock=tick, sleep=lambda s: None)
        return hw, report

    def test_steers_and_stops_on_key(self):
        presses = iter([False] * 20 + [True] * 5)
        hw, report = self.drive([image(200, 600)], lambda: next(presses))  # 중심 400: 우회전
        self.assertEqual(hw.wheels[0], lane_speeds(400))
        self.assertEqual(report['reason'], 'KEY_STOP')
        self.assertGreaterEqual(hw.stops, 2)  # 시작 전 + 끝날 때

    def test_no_lines_never_moves(self):
        hw, report = self.drive([image()], lambda: False)
        self.assertEqual(report['reason'], 'LOST_LINE')
        self.assertTrue(all(s == STOP for s in hw.wheels))

    def test_lost_after_tracking_ends_with_stop(self):
        hw, report = self.drive([image(200, 600)] * 5 + [image()], lambda: False)
        self.assertIn((60, 60, 0, 0), hw.wheels)
        self.assertEqual(report['reason'], 'LOST_LINE')
        self.assertEqual(hw.wheels[-1], STOP)

    def test_error_still_stops(self):
        hw = FakeHw()
        vision = FakeVision([np.zeros((10, 10, 3), np.uint8)])
        with self.assertRaises(ValueError):
            rt.run(CFG, hw, vision, lambda: False, {}, sleep=lambda s: None)
        self.assertGreaterEqual(hw.stops, 2)


class Setup(unittest.TestCase):
    def test_servo_angles_sent(self):
        fake = Mock()
        with patch.dict(sys.modules, {'Raspbot_Lib': fake}):
            rt.Hardware(False, 90, 5)
            fake.Raspbot.return_value.Ctrl_Servo.assert_any_call(2, 5)
            fake.Raspbot.return_value.Ctrl_Servo.reset_mock()
            rt.Hardware(False)
            fake.Raspbot.return_value.Ctrl_Servo.assert_any_call(2, 40)

    def test_aim_moves_camera_only(self):
        fake = Mock()
        with patch.dict(sys.modules, {'Raspbot_Lib': fake}):
            module = load('track6aim', '6_track.py')
            module.aim(module.build_config(tilt=10))
        bot = fake.Raspbot.return_value
        bot.Ctrl_Servo.assert_any_call(2, 10)
        bot.Ctrl_Muto.assert_not_called()

    def test_mission_config(self):
        cfg = load('track6', '6_track.py').build_config()
        self.assertTrue(cfg.use_camera_lane)
        self.assertFalse(cfg.use_feature)
        self.assertTrue(cfg.use_obstacle)
        self.assertEqual(cfg.camera_tilt, 5)
        self.assertFalse(load('track6b', '6_track.py').build_config(no_obstacle=True).use_obstacle)
        with self.assertRaises(SystemExit):
            load('track6c', '6_track.py').build_config(tilt=200)


if __name__ == '__main__':
    unittest.main()
