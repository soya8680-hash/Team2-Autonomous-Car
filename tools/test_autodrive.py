#!/usr/bin/env python3
# 지시서: specs/5-오토드라이브.md
"""상태표 · 정지 처리 · 글 명령 · 사진 채점 시험. 가짜 값만 쓰고 장치에 접근하지 않는다."""
import csv
import importlib.util
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import autodrive_core as core
from autodrive_core import (Config, DistanceFilter, Obs, Perception, StateMachine, STOP,
                            FOLLOW, SLOW, STOP_SIGN, WAIT_PERSON, WAIT_OBSTACLE, FINISHED, EMERGENCY)
import autodrive_runtime as rt


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'missions' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CENTER, LEFT, RIGHT, LOST, BLACK = 0b0101, 0b1011, 0b1110, 0b1111, 0b0000


def obs(t, line=CENTER, dist=100.0, **kw):
    return Obs(t=t, line_raw=line, distance_cm=dist, **kw)


class StateTable(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.sm = StateMachine(self.cfg, 0.0)

    def test_line_rules_match_stage2(self):
        line = load('line2', '02_line.py')
        names = {'LOST_LINE': 'LOST', 'ALL_BLACK': 'ALL_BLACK', 'LEFT': 'LEFT',
                 'RIGHT': 'RIGHT', 'STRAIGHT': 'STRAIGHT'}
        for raw in range(16):
            state, speeds = line.decide(raw)
            self.assertEqual(core.line_direction(raw), names[state], raw)
            if state not in ('LOST_LINE', 'ALL_BLACK'):
                self.assertEqual(core.wheel_speeds(names[state], 60, 30), speeds, raw)

    def test_follow_and_turn(self):
        self.assertEqual(self.sm.step(obs(0.1)).speeds, (60,) * 4)
        self.assertEqual(self.sm.step(obs(0.2, LEFT)).speeds, (30, 30, 60, 60))
        self.assertEqual(self.sm.step(obs(0.3, RIGHT)).speeds, (60, 60, 30, 30))

    def test_obstacle_stops_and_resumes_with_gap(self):
        self.sm.step(obs(0.1))
        a = self.sm.step(obs(0.2, dist=15.0))
        self.assertEqual((a.state, a.speeds), (WAIT_OBSTACLE, STOP))
        self.assertGreater(a.beep_seconds, 0)
        # 15~20cm 사이에서는 계속 기다린다 (떨림 방지)
        self.assertEqual(self.sm.step(obs(0.3, dist=18.0)).speeds, STOP)
        self.assertEqual(self.sm.step(obs(0.4, dist=20.0)).state, SLOW)
        self.assertNotEqual(self.sm.step(obs(0.5, dist=100.0)).speeds, STOP)

    def test_stop_sign_waits_then_ignores_same_sign(self):
        a = self.sm.step(obs(1.0, stop_sign=True))
        self.assertEqual((a.state, a.speeds), (STOP_SIGN, STOP))
        self.assertEqual(self.sm.step(obs(3.9, stop_sign=True)).speeds, STOP)
        a = self.sm.step(obs(4.0, stop_sign=True))
        self.assertEqual(a.state, FOLLOW)
        self.assertNotEqual(a.speeds, STOP)
        self.assertEqual(self.sm.step(obs(8.9, stop_sign=True)).state, FOLLOW)
        self.assertEqual(self.sm.step(obs(9.0, stop_sign=True)).state, STOP_SIGN)

    def test_person_waits_until_gone_one_second(self):
        self.assertEqual(self.sm.step(obs(1.0, person_close=True)).state, WAIT_PERSON)
        self.assertEqual(self.sm.step(obs(2.0)).state, WAIT_PERSON)
        # 사라졌다가 다시 보이면 1초를 처음부터 다시 잰다
        self.assertEqual(self.sm.step(obs(2.5, person_close=True)).state, WAIT_PERSON)
        self.assertEqual(self.sm.step(obs(3.0)).state, WAIT_PERSON)
        self.assertEqual(self.sm.step(obs(3.9)).state, WAIT_PERSON)
        self.assertEqual(self.sm.step(obs(4.0)).state, FOLLOW)

    def test_obstacle_has_priority_over_stop_sign(self):
        self.assertEqual(self.sm.step(obs(1.0, dist=5.0, stop_sign=True, person_close=True)).state, WAIT_OBSTACLE)

    def test_feature_slows_for_near_things(self):
        a = self.sm.step(obs(1.0, dist=40.0))
        self.assertEqual((a.state, a.speeds), (SLOW, (45,) * 4))
        a = self.sm.step(obs(1.1, person_far=True))
        self.assertEqual(a.state, SLOW)
        off = StateMachine(replace(self.cfg, use_feature=False), 0.0)
        self.assertEqual(off.step(obs(1.0, dist=40.0)).speeds, (60,) * 4)

    def test_lost_line_grace_then_emergency_and_count(self):
        self.sm.step(obs(0.1, LEFT))
        a = self.sm.step(obs(0.2, LOST))
        self.assertEqual(a.speeds, (30, 30, 60, 60))
        self.sm.step(obs(0.35, LOST))
        self.assertEqual(self.sm.off_line_count, 1)
        self.assertEqual(self.sm.step(obs(0.4)).state, FOLLOW)
        self.assertEqual(self.sm.step(obs(1.0, LOST)).state, FOLLOW)
        a = self.sm.step(obs(1.6, LOST))
        self.assertEqual((a.state, a.speeds, a.done, a.reason), (EMERGENCY, STOP, True, 'LOST_LINE'))
        self.assertEqual(self.sm.off_line_count, 2)

    def test_no_line_at_start_never_moves(self):
        self.assertEqual(self.sm.step(obs(0.1, LOST)).speeds, STOP)
        self.assertEqual(self.sm.step(obs(0.7, LOST)).state, EMERGENCY)

    def test_finish_only_after_min_lap_and_steady_black(self):
        self.assertNotEqual(self.sm.step(obs(1.0, BLACK)).speeds, STOP)  # 출발선은 지나간다
        self.sm.step(obs(4.9, CENTER))
        self.assertEqual(self.sm.step(obs(5.0, BLACK)).state, FOLLOW)
        self.assertEqual(self.sm.step(obs(5.02, BLACK)).state, FOLLOW)
        a = self.sm.step(obs(5.04, BLACK))
        self.assertEqual((a.state, a.speeds, a.done, a.reason), (FINISHED, STOP, True, 'FINISH_LINE'))
        self.assertEqual(self.sm.step(obs(5.1)).speeds, STOP)  # 끝나면 다시 달리지 않는다

    def test_faults_stop_from_any_state(self):
        for kw, reason in (({'vision_age': 2.1}, 'CAMERA_FAULT'), ({'distance_fault': True}, 'ULTRASONIC_FAULT')):
            sm = StateMachine(self.cfg, 0.0)
            sm.step(obs(1.0, person_close=True))
            a = sm.step(obs(1.1, person_close=True, **kw))
            self.assertEqual((a.state, a.reason, a.speeds), (EMERGENCY, reason, STOP))
        a = self.sm.step(obs(180.0))
        self.assertEqual((a.reason, a.speeds), ('TIME_LIMIT', STOP))

    def test_every_config_speed_within_limit(self):
        c = self.cfg
        self.assertTrue(all(0 < s <= core.MAX_WHEEL_SPEED for s in (c.speed, c.inner_speed, c.slow_speed, c.slow_inner_speed)))
        sm = StateMachine(replace(c, speed=151), 0.0)
        with self.assertRaises(ValueError):
            sm.step(obs(1.0))


class Sensors(unittest.TestCase):
    def test_distance_median_ignores_spike_and_zero(self):
        f = DistanceFilter(1.0)
        for mm, t in ((1000, 0.0), (0, 0.1), (100, 0.2), (1000, 0.3)):
            f.add(mm, t)
        self.assertEqual(f.value(), 100.0)
        self.assertFalse(f.fault(1.3))
        self.assertTrue(f.fault(1.4))

    def test_distance_never_valid_is_fault(self):
        f = DistanceFilter(1.0)
        f.add(0, 0.0)
        self.assertIsNone(f.value())
        self.assertTrue(f.fault(1.1))

    def test_perception_needs_two_new_results(self):
        p = Perception(Config())
        stop = [('stop sign', 0.9, (0, 0, 50, 50))]
        p.update(1, stop)
        self.assertFalse(p.confirmed('stop'))
        p.update(1, stop)  # 같은 결과를 다시 읽어도 세지 않는다
        self.assertFalse(p.confirmed('stop'))
        p.update(2, stop)
        self.assertTrue(p.confirmed('stop'))
        p.update(3, [])
        self.assertFalse(p.confirmed('stop'))

    def test_perception_thresholds(self):
        p = Perception(Config())
        for seq in (1, 2):
            p.update(seq, [('person', 0.9, (0, 0, 50, 100)), ('stop sign', 0.4, (0, 0, 90, 90)), ('car', 0.9, (0, 0, 90, 400))])
        self.assertTrue(p.confirmed('far'))
        self.assertFalse(p.confirmed('close'))
        self.assertFalse(p.confirmed('stop'))


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeVision:
    def __init__(self, clock):
        self.clock, self.seq, self.frozen = clock, 0, False
        self.statuses = []

    def latest(self):
        if not self.frozen:
            self.seq += 1
            self.t = self.clock()
        return self.seq, [], self.t, self.t

    def set_status(self, text):
        self.statuses.append(text)


class FakeBot:
    def __init__(self, line=CENTER, mm=1000, fail_at=None):
        self.line, self.mm, self.fail_at = line, mm, fail_at
        self.reads, self.wheels, self.stop_calls, self.beeps = 0, [], 0, []

    def read_line(self):
        self.reads += 1
        if self.fail_at and self.reads >= self.fail_at:
            raise OSError('가짜 I2C 오류')
        return self.line(self.reads) if callable(self.line) else self.line

    def read_distance_mm(self):
        return self.mm

    def set_wheels(self, speeds):
        self.wheels.append(tuple(speeds))

    def stop_all(self):
        self.stop_calls += 1
        self.wheels.append(STOP)
        return []

    def light(self, color):
        pass

    def beep(self, on):
        self.beeps.append(on)


class RunLoop(unittest.TestCase):
    def play(self, bot, key=lambda: False, cfg=None):
        clock = Clock()
        report = {}
        rt.run(cfg or Config(), bot, FakeVision(clock), key, report, clock=clock, sleep=clock.sleep)
        return report

    def test_any_key_stops_and_last_command_is_zero(self):
        presses = iter([False] * 20 + [True] * 2)
        bot = FakeBot()
        report = self.play(bot, key=lambda: next(presses))
        self.assertEqual(report['reason'], 'KEY_STOP')
        self.assertEqual(bot.wheels[-1], STOP)
        self.assertIn((60,) * 4, bot.wheels)

    def test_key_before_first_move_sends_no_move(self):
        bot = FakeBot()
        self.play(bot, key=lambda: True)
        self.assertTrue(all(w == STOP for w in bot.wheels))

    def test_error_during_drive_still_stops(self):
        bot = FakeBot(fail_at=10)
        report = {}
        clock = Clock()
        with self.assertRaises(OSError):
            rt.run(Config(), bot, FakeVision(clock), lambda: False, report, clock=clock, sleep=clock.sleep)
        self.assertEqual(report['reason'], 'ERROR')
        self.assertEqual(bot.wheels[-1], STOP)

    def test_ctrl_c_still_stops(self):
        presses = iter([False] * 6)

        def key():
            try:
                return next(presses)
            except StopIteration:
                raise KeyboardInterrupt

        bot = FakeBot()
        clock = Clock()
        report = {}
        with self.assertRaises(KeyboardInterrupt):
            rt.run(Config(), bot, FakeVision(clock), key, report, clock=clock, sleep=clock.sleep)
        self.assertEqual(report['reason'], 'INTERRUPTED')
        self.assertEqual(bot.wheels[-1], STOP)

    def test_lost_line_emergency(self):
        bot = FakeBot(line=lambda n: CENTER if n < 10 else LOST)
        report = self.play(bot)
        self.assertEqual(report['reason'], 'LOST_LINE')
        self.assertEqual(report['off_line_count'], 1)
        self.assertEqual(bot.wheels[-1], STOP)

    def test_camera_freeze_stops(self):
        clock = Clock()
        vision = FakeVision(clock)
        bot = FakeBot()
        original = bot.read_line

        def freeze():
            if bot.reads == 10:
                vision.frozen = True
            return original()

        bot.read_line = freeze
        report = {}
        rt.run(Config(), bot, vision, lambda: False, report, clock=clock, sleep=clock.sleep)
        self.assertEqual(report['reason'], 'CAMERA_FAULT')
        self.assertEqual(bot.wheels[-1], STOP)

    def test_stuck_near_ultrasonic_never_moves(self):
        # 지금 실물 초음파가 3.4cm 로 고정된 경우: 움직이지 않고 시간 제한으로 끝나야 한다
        bot = FakeBot(mm=34)
        report = self.play(bot, cfg=replace(Config(), time_limit_seconds=5))
        self.assertEqual(report['reason'], 'TIME_LIMIT')
        moves = [w for w in bot.wheels if w != STOP]
        self.assertLessEqual(len(set(moves)), 1)
        self.assertLess(len(moves), 3)  # 첫 초음파 값이 필터에 들어가기 전 한순간만 허용되는지 확인

    def test_finish_line(self):
        bot = FakeBot(line=lambda n: BLACK if n > 300 else CENTER)
        report = self.play(bot)
        self.assertEqual(report['reason'], 'FINISH_LINE')
        self.assertEqual(report['final_state'], FINISHED)
        self.assertEqual(bot.wheels[-1], STOP)


class FreeDrive(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fd = load('free_drive', 'free_drive.py')

    def play(self, vision=None, key=lambda: False, bot=None):
        clock = Clock()
        bot = bot or FakeBot()
        vision = vision(clock) if vision else FakeVision(clock)
        report = {}
        self.fd.run(bot, vision, key, report, clock=clock, sleep=clock.sleep)
        return bot, report

    def test_ends_at_15_seconds_and_stops(self):
        bot, report = self.play()
        self.assertEqual(report['reason'], 'TIME_LIMIT')
        self.assertAlmostEqual(report['elapsed_seconds'], 15, delta=0.1)
        self.assertEqual(bot.wheels[-1], STOP)
        self.assertEqual({w for w in bot.wheels}, {(40,) * 4, STOP})

    def test_close_person_stops_and_never_moves_again(self):
        class Person(FakeVision):
            def latest(self):
                seq, _, t, f = super().latest()
                near = seq > 100
                return seq, [('person', 0.9, (0, 0, 100, 350))] if near else [], t, f

        bot, report = self.play(vision=Person)
        self.assertEqual(report['reason'], 'PERSON_CLOSE')
        self.assertEqual(report['stop_person_height'], 350)
        self.assertLess(report['elapsed_seconds'], 3.5)
        self.assertEqual(bot.wheels[-1], STOP)
        self.assertEqual(bot.wheels[-2], STOP)
        self.assertEqual(bot.beeps, [True, False])

    def test_far_person_keeps_going(self):
        class Far(FakeVision):
            def latest(self):
                seq, _, t, f = super().latest()
                return seq, [('person', 0.9, (0, 0, 100, 299))], t, f

        _, report = self.play(vision=Far)
        self.assertEqual(report['reason'], 'TIME_LIMIT')

    def test_camera_freeze_stops(self):
        class Frozen(FakeVision):
            def latest(self):
                self.frozen = self.seq >= 50
                return super().latest()

        bot, report = self.play(vision=Frozen)
        self.assertEqual(report['reason'], 'CAMERA_FAULT')
        self.assertEqual(bot.wheels[-1], STOP)

    def test_key_first_never_moves(self):
        bot, report = self.play(key=lambda: True)
        self.assertEqual(report['reason'], 'KEY_STOP')
        self.assertTrue(all(w == STOP for w in bot.wheels))

    def test_error_still_stops(self):
        class Broken(FakeBot):
            def set_wheels(self, speeds):
                super().set_wheels(speeds)
                raise OSError('가짜 I2C 오류')

        bot = Broken()
        with self.assertRaises(OSError):
            self.play(bot=bot)
        self.assertEqual(bot.wheels[-1], STOP)


class GuardedTeleop(unittest.TestCase):
    PERSON = [('person', 0.9, (0, 0, 100, 400))]

    @classmethod
    def setUpClass(cls):
        cls.gt = load('guarded_teleop', 'guarded_teleop.py')

    def setUp(self):
        self.bot = FakeBot()
        self.c = self.gt.Controller(self.bot)
        self.seq = 0

    def see(self, now, detections=(), det_t=None):
        self.seq += 1
        t = now if det_t is None else det_t
        self.c.update(self.seq, list(detections), t, t, now)

    def test_moves_and_auto_stop(self):
        self.see(1.0)
        self.c.key('w', 1.0)
        self.assertEqual(self.bot.wheels[-1], (60,) * 4)
        self.see(1.5)
        self.assertEqual(self.c.moving, 'w')
        self.see(1.7)
        self.assertEqual(self.bot.wheels[-1], STOP)

    def test_close_person_stops_and_blocks_all_moves(self):
        self.c.key('w', 1.0)
        self.see(1.0, self.PERSON)
        self.assertEqual(self.bot.wheels[-1], (60,) * 4)  # 한 번은 잘못 본 것일 수 있다
        self.see(1.1, self.PERSON)
        self.assertTrue(self.c.locked)
        self.assertEqual(self.bot.wheels[-1], STOP)
        self.assertEqual(self.bot.beeps, [True])
        count = len(self.bot.wheels)
        for key in self.gt.MOVES:
            self.c.key(key, 1.2)
        self.assertEqual(len(self.bot.wheels), count)

    def test_unlock_after_one_clear_second_without_moving(self):
        self.see(1.0, self.PERSON)
        self.see(1.1, self.PERSON)
        self.see(1.2)
        self.see(2.1)
        self.assertTrue(self.c.locked)
        self.see(2.2)
        self.assertFalse(self.c.locked)
        self.assertEqual(self.c.moving, None)
        self.assertTrue(all(w == STOP for w in self.bot.wheels))
        self.c.key('s', 2.3)
        self.assertEqual(self.bot.wheels[-1], (-60,) * 4)

    def test_reappearing_person_restarts_clear_time(self):
        self.see(1.0, self.PERSON)
        self.see(1.1, self.PERSON)
        self.see(1.5)
        self.see(1.9, self.PERSON)
        self.see(2.0, self.PERSON)
        self.see(2.6)
        self.assertTrue(self.c.locked)

    def test_far_person_does_not_lock(self):
        far = [('person', 0.9, (0, 0, 100, 349))]
        self.see(1.0, far)
        self.see(1.1, far)
        self.assertFalse(self.c.locked)

    def test_camera_freeze_locks(self):
        self.c.key('w', 1.0)
        self.see(2.1, det_t=1.0)
        self.assertTrue(self.c.locked)
        self.assertEqual(self.c.lock_reason, 'CAMERA_FAULT')
        self.assertEqual(self.bot.wheels[-1], STOP)

    def test_beep_turns_off(self):
        self.see(1.0, self.PERSON)
        self.see(1.1, self.PERSON)
        self.see(1.7, self.PERSON)
        self.assertEqual(self.bot.beeps, [True, False])

    def test_speed_limits_and_quit(self):
        for _ in range(10):
            self.c.key('+', 1.0)
        self.assertEqual(self.c.speed, 100)
        self.assertLessEqual(self.c.speed, core.MAX_WHEEL_SPEED)
        for _ in range(10):
            self.c.key('-', 1.0)
        self.assertEqual(self.c.speed, 40)
        self.assertFalse(self.c.key('x', 1.0))


class Records(unittest.TestCase):
    def test_runs_csv_header_once(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'runs.csv'
            rt.append_run(path, [4, 'abc', 1, '예', 0, 12.3, 0, '메모'])
            rt.append_run(path, [4, 'abc', 2, '아니오', 1, 5.0, 1, ''])
            rows = list(csv.reader(open(path, encoding='utf-8')))
            self.assertEqual(rows[0], ['회차', '코드 버전', '시도', '완주', '이탈', '시간', '손댄 횟수', '메모'])
            self.assertEqual(len(rows), 3)


class TextCommand(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tc = load('text_command', 'text_command.py')

    def test_allowed(self):
        self.assertEqual(self.tc.choose('앞으로 가 줘!'), 'forward')
        self.assertEqual(self.tc.choose('멈춰'), 'stop')
        self.assertEqual(self.tc.choose('빨간 불 켜 줘'), 'light_red')

    def test_refused(self):
        for text in ('최고 속도로 앞으로 가', '앞으로 10미터', '벽에 박아', '라즈봇 꺼 줘', '', 'rm -rf /'):
            self.assertIsNone(self.tc.choose(text), text)

    def test_all_phrases_run_and_wheels_end_zero(self):
        for action in self.tc.PHRASES:
            bot = FakeBot()
            self.tc.perform(action, bot, sleep=lambda s: None)
            if action in self.tc.MOVES:
                self.assertEqual(bot.wheels[-1], STOP)
                self.assertTrue(all(abs(s) <= core.MAX_WHEEL_SPEED for w in bot.wheels for s in w))

    def test_wheels_stop_even_if_sleep_interrupted(self):
        bot = FakeBot()

        def boom(seconds):
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            self.tc.perform('forward', bot, sleep=boom)
        self.assertEqual(bot.wheels[-1], STOP)


class PhotoScoring(unittest.TestCase):
    def test_score(self):
        yolo = load('yolo4', '4_yolo.py')
        self.assertTrue(yolo.score_photo('person', {'person'}))
        self.assertFalse(yolo.score_photo('stop', {'person'}))
        self.assertTrue(yolo.score_photo('none', {'chair'}))
        self.assertFalse(yolo.score_photo('none', {'person'}))


if __name__ == '__main__':
    unittest.main(verbosity=1)
