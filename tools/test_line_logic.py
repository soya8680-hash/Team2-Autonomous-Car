#!/usr/bin/env python3
"""장치에 접근하지 않고 라인 주행 판단과 정지 처리를 검사한다."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'missions/02_line.py'
spec = importlib.util.spec_from_file_location('line', path)
line = importlib.util.module_from_spec(spec)
spec.loader.exec_module(line)


class FakeBot:
    _addr = 0x2b

    def __init__(self, values=(), fail_motor=None, read_error=None):
        self._device = self
        self.values = iter(values)
        self.writes = []
        self.fail_motor = fail_motor
        self.read_error = read_error

    def write_i2c_block_data(self, address, register, data):
        self.writes.append(tuple(data))
        if data[0] == self.fail_motor:
            raise OSError('simulated bus error')

    def read_i2c_block_data(self, address, register, length):
        if self.read_error:
            raise self.read_error
        return [next(self.values)]


class LineTests(unittest.TestCase):
    def test_all_sensor_patterns(self):
        expected = ['ALL_BLACK', 'LEFT', 'LEFT', 'LEFT', 'RIGHT', 'LEFT',
                    'RIGHT', 'LEFT', 'RIGHT', 'LEFT', 'RIGHT', 'LEFT',
                    'RIGHT', 'RIGHT', 'RIGHT', 'LOST_LINE']
        # 가운데 두 센서만 검정(raw=0101): x1=x3=0 → 가운데.
        expected[5] = 'STRAIGHT'
        # 바깥 두 센서만 검정(raw=1010): 좌우 대칭 → 가운데.
        expected[10] = 'STRAIGHT'
        for raw, state in enumerate(expected):
            with self.subTest(raw=raw):
                actual, speeds = line.decide(raw)
                self.assertEqual(actual, state)
                self.assertLessEqual(max(map(abs, speeds)), 60)

    def test_stops_on_lost_line(self):
        bot, report = FakeBot([5, 15]), {'samples': []}
        line.run_loop(bot, lambda: False, report, sleep=lambda _: None)
        self.assertEqual(report['reason'], 'LOST_LINE')
        self.assertEqual(bot.writes[-4:], [(i, 0, 0) for i in range(4)])

    def test_key_stops_before_movement(self):
        bot, report = FakeBot(), {'samples': []}
        line.run_loop(bot, lambda: True, report)
        self.assertEqual(report['reason'], 'KEY_STOP')
        self.assertTrue(all(data[2] == 0 for data in bot.writes))

    def test_sensor_error_and_interrupt_stop_all(self):
        for error in (OSError('sensor disconnected'), KeyboardInterrupt()):
            bot, report = FakeBot(read_error=error), {'samples': []}
            with self.assertRaises(type(error)):
                line.run_loop(bot, lambda: False, report)
            self.assertEqual(bot.writes[-4:], [(i, 0, 0) for i in range(4)])

    def test_one_failed_stop_does_not_skip_other_wheels(self):
        bot = FakeBot(fail_motor=1)
        self.assertEqual(len(line.stop_all(bot)), 1)
        self.assertEqual(bot.writes, [(i, 0, 0) for i in range(4)])

    def test_timeout_after_sensor_read_prevents_movement(self):
        moments = iter([0, 0, 11, 11, 11])
        bot, report = FakeBot([5]), {'samples': []}
        line.run_loop(bot, lambda: False, report, clock=lambda: next(moments))
        self.assertEqual(report['reason'], 'TIME_LIMIT')
        self.assertTrue(all(data[2] == 0 for data in bot.writes))

    def test_partial_motor_write_error_stops_all(self):
        bot, report = FakeBot([5]), {'samples': []}
        write = bot.write_i2c_block_data

        def fail_when_moving(address, register, data):
            write(address, register, data)
            if data[0] == 1 and data[2] > 0:
                raise OSError('motor write failed')

        bot.write_i2c_block_data = fail_when_moving
        with self.assertRaises(OSError):
            line.run_loop(bot, lambda: False, report)
        self.assertEqual(bot.writes[-4:], [(i, 0, 0) for i in range(4)])


if __name__ == '__main__':
    unittest.main(verbosity=2)
