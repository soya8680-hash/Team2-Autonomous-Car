#!/usr/bin/env python3
# 지시서: specs/3-장애물.md · specs/4-욜로.md · specs/5-오토드라이브.md · specs/기능-보행자제동.md · specs/6-트랙주행.md
"""오토드라이브의 두뇌(상태표). 장치에 접근하지 않아서 가짜 값으로 시험할 수 있다.

상태표를 바꾸려면 지시서를 먼저 고치고, 그다음 아래 Config 숫자를 맞춘다.
"""
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Optional

MAX_WHEEL_SPEED = 150
STOP = (0, 0, 0, 0)

# 상태 이름
FOLLOW = 'FOLLOW'
SLOW = 'SLOW'
STOP_SIGN = 'STOP_SIGN'
WAIT_PERSON = 'WAIT_PERSON'
WAIT_OBSTACLE = 'WAIT_OBSTACLE'
FINISHED = 'FINISHED'
EMERGENCY = 'EMERGENCY'

# 불빛 색 번호 (드라이버 기준)
RED, GREEN, BLUE, YELLOW = 0, 1, 2, 3


@dataclass(frozen=True)
class Config:
    # 어떤 기능을 켤지 (단계별 프로그램이 다르게 고른다)
    use_obstacle: bool = True
    use_yolo: bool = True
    use_feature: bool = True      # 우리만의 기능: 보행자 긴급 제동
    # 줄 따라가기 — specs/02-라인.md 에서 조가 확정한 값
    speed: int = 60
    inner_speed: int = 30
    # 이하 Claude 초안 — 조 확인 필요 (지시서 참고)
    slow_speed: int = 45
    slow_inner_speed: int = 25
    lost_line_seconds: float = 0.5
    min_lap_seconds: float = 5.0
    finish_reads: int = 3
    time_limit_seconds: float = 180.0
    # 초음파
    obstacle_stop_cm: float = 15.0
    obstacle_resume_cm: float = 20.0
    slow_cm: float = 40.0
    distance_fault_seconds: float = 1.0
    # YOLO (상자 높이는 480픽셀 화면 기준)
    min_score: float = 0.5
    stop_sign_min_height: int = 40
    person_close_height: int = 200
    person_far_height: int = 80
    confirm_count: int = 2
    stop_sign_seconds: float = 3.0
    stop_sign_ignore_seconds: float = 5.0
    person_clear_seconds: float = 1.0
    vision_fault_seconds: float = 2.0
    # 카메라 각도 — specs/기능-자유주행-사람정지.md
    camera_pan: int = 90
    camera_tilt: int = 40
    # 트랙 주행 — specs/6-트랙주행.md (켜면 라인센서 대신 카메라 차선으로 조향)
    use_camera_lane: bool = False
    lane_lost_seconds: float = 1.0
    lane_frame_fault_seconds: float = 0.5


@dataclass
class Obs:
    """주행 반복 한 번에서 본 것."""
    t: float
    line_raw: int
    distance_cm: Optional[float] = None
    distance_fault: bool = False
    stop_sign: bool = False
    person_close: bool = False
    person_far: bool = False
    vision_age: float = 0.0
    lane: Optional['Lane'] = None     # 트랙 주행일 때만
    lane_age: float = 0.0


@dataclass
class Action:
    state: str
    speeds: tuple
    light: Optional[int]
    beep_seconds: float = 0.0
    done: bool = False
    reason: str = ''


def line_black(raw):
    """(왼쪽 → 오른쪽) 칸마다 검정인지. 실제 배치 x2 · x1 · x3 · x4, 검정은 0."""
    if not isinstance(raw, int) or not 0 <= raw <= 255:
        raise ValueError('라인센서 값이 잘못됐습니다.')
    return tuple(not ((raw >> bit) & 1) for bit in (2, 3, 1, 0))


def line_direction(raw):
    """LOST / ALL_BLACK / LEFT / RIGHT / STRAIGHT. 규칙은 specs/02-라인.md 와 같다."""
    black = line_black(raw)
    if not any(black):
        return 'LOST'
    if all(black):
        return 'ALL_BLACK'
    offset = sum(w for w, hit in zip((-3, -1, 1, 3), black) if hit)
    return 'LEFT' if offset < 0 else 'RIGHT' if offset > 0 else 'STRAIGHT'


def wheel_speeds(direction, fast, inner):
    if direction == 'LEFT':
        return (inner, inner, fast, fast)
    if direction == 'RIGHT':
        return (fast, fast, inner, inner)
    return (fast,) * 4


# 차선 값 — specs/6-트랙주행.md
LANE_BASE_SPEED = 35
LANE_K = 0.4
LANE_MAX_SPEED = 80
LANE_SEARCH_OUTER = 60
LANE_TRACK, LANE_WAIT, LANE_SEARCH, LANE_NONE = 'TRACK', 'WAIT', 'SEARCH', 'NONE'


@dataclass(frozen=True)
class Lane:
    kind: str
    speeds: tuple = STOP
    center: Optional[float] = None


def lane_speeds(center):
    correction = LANE_K * (center - 320)
    left = max(0, min(LANE_MAX_SPEED, round(LANE_BASE_SPEED + correction)))
    right = max(0, min(LANE_MAX_SPEED, round(LANE_BASE_SPEED - correction)))
    return (left, left, right, right)


class LaneTracker:
    """왼쪽·오른쪽 선 위치(없으면 None)를 바퀴 값으로 바꾼다. 시간과 정지 판단은 상태표가 한다."""

    def __init__(self):
        self.width = None
        self.last_turn = None

    def update(self, left, right):
        if left is None and right is None:
            if self.last_turn is None:
                return Lane(LANE_NONE)
            outer = LANE_SEARCH_OUTER
            speeds = (outer, outer, 0, 0) if self.last_turn == 'RIGHT' else (0, 0, outer, outer)
            return Lane(LANE_SEARCH, speeds)
        if left is not None and right is not None:
            self.width = right - left
            center = (left + right) / 2
        elif self.width is None:
            return Lane(LANE_WAIT)
        elif left is not None:
            center = left + self.width / 2
        else:
            center = right - self.width / 2
        speeds = lane_speeds(center)
        if speeds[0] != speeds[2]:
            self.last_turn = 'RIGHT' if speeds[0] > speeds[2] else 'LEFT'
        return Lane(LANE_TRACK, speeds, center)


class DistanceFilter:
    """초음파 값 튐을 줄인다: 최근 3개의 가운데 값. 0 이하는 측정 실패로 본다."""

    def __init__(self, fault_seconds):
        self.values = deque(maxlen=3)
        self.fault_seconds = fault_seconds
        self.last_valid = None
        self.started = None

    def add(self, mm, t):
        if self.started is None:
            self.started = t
        if mm > 0:
            self.values.append(mm / 10)
            self.last_valid = t

    def value(self):
        return median(self.values) if self.values else None

    def fault(self, t):
        if self.started is None:
            return False
        return t - (self.last_valid if self.last_valid is not None else self.started) > self.fault_seconds


class Perception:
    """YOLO 결과를 상태표가 쓰는 예/아니오로 바꾼다. 연속으로 봐야 「봤다」로 친다."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.seq = -1
        self.counts = {'stop': 0, 'close': 0, 'far': 0}

    def update(self, seq, detections):
        if seq == self.seq:
            return
        self.seq = seq
        c = self.cfg
        height = {'person': 0, 'stop sign': 0}
        for name, score, (_, _, _, h) in detections:
            if name in height and score >= c.min_score:
                height[name] = max(height[name], h)
        now = {'stop': height['stop sign'] >= c.stop_sign_min_height,
               'close': height['person'] >= c.person_close_height,
               'far': height['person'] >= c.person_far_height}
        for key, hit in now.items():
            self.counts[key] = self.counts[key] + 1 if hit else 0

    def confirmed(self, key):
        return self.counts[key] >= self.cfg.confirm_count


LIGHT = {FOLLOW: GREEN, SLOW: YELLOW, STOP_SIGN: RED, WAIT_PERSON: RED,
         WAIT_OBSTACLE: BLUE, FINISHED: None, EMERGENCY: RED}
BEEP = {SLOW: 0.0, STOP_SIGN: 0.2, WAIT_PERSON: 0.5, WAIT_OBSTACLE: 0.2,
        FINISHED: 0.2, EMERGENCY: 1.0, FOLLOW: 0.0}


class StateMachine:
    """상태표. specs/5-오토드라이브.md 의 표와 한 줄씩 맞춰 둔다."""

    def __init__(self, cfg, start_t):
        self.cfg, self.start_t = cfg, start_t
        self.state = FOLLOW
        self.last_speeds = None
        self.lost_since = None
        self.lost_counted = False
        self.off_line_count = 0
        self.black_reads = 0
        self.stop_until = 0.0
        self.ignore_stop_until = 0.0
        self.person_clear_since = None
        self.transitions = []

    def _action(self, state, speeds, t, reason=''):
        beep = 0.0
        if state != self.state:
            self.transitions.append({'t': round(t - self.start_t, 3), 'from': self.state,
                                     'to': state, 'reason': reason})
            beep = BEEP[state]
        self.state = state
        for s in speeds:
            if abs(s) > MAX_WHEEL_SPEED:
                raise ValueError('바퀴 속도 150 초과')
        return Action(state, speeds, LIGHT[state], beep, state in (FINISHED, EMERGENCY), reason)

    def _hold(self, state, t, reason):
        self.lost_since, self.lost_counted = None, False
        return self._action(state, STOP, t, reason)

    def step(self, o):
        c = self.cfg
        if self.state in (FINISHED, EMERGENCY):
            return self._action(self.state, STOP, o.t, 'already done')

        # 1) 안전 고장 — 어느 상태에서든 먼저 본다
        if o.t - self.start_t >= c.time_limit_seconds:
            return self._action(EMERGENCY, STOP, o.t, 'TIME_LIMIT')
        if o.vision_age > c.vision_fault_seconds:
            return self._action(EMERGENCY, STOP, o.t, 'CAMERA_FAULT')
        if c.use_camera_lane and o.lane_age > c.lane_frame_fault_seconds:
            return self._action(EMERGENCY, STOP, o.t, 'CAMERA_FAULT')
        if c.use_obstacle and o.distance_fault:
            return self._action(EMERGENCY, STOP, o.t, 'ULTRASONIC_FAULT')

        near = c.use_obstacle and o.distance_cm is not None and o.distance_cm <= c.obstacle_stop_cm
        clear = (not c.use_obstacle) or (o.distance_cm is not None and o.distance_cm >= c.obstacle_resume_cm)
        person_close = c.use_yolo and o.person_close

        # 2) 멈춰서 기다리는 상태
        if self.state == STOP_SIGN:
            if o.t < self.stop_until:
                return self._hold(STOP_SIGN, o.t, 'stop sign wait')
            self.ignore_stop_until = o.t + c.stop_sign_ignore_seconds
        elif self.state == WAIT_OBSTACLE:
            if not clear:
                return self._hold(WAIT_OBSTACLE, o.t, 'obstacle')
        elif self.state == WAIT_PERSON:
            if person_close:
                self.person_clear_since = None
                return self._hold(WAIT_PERSON, o.t, 'person')
            if self.person_clear_since is None:
                self.person_clear_since = o.t
            if o.t - self.person_clear_since < c.person_clear_seconds:
                return self._hold(WAIT_PERSON, o.t, 'person clearing')

        # 3) 달리다가 멈출 일
        if near:
            return self._hold(WAIT_OBSTACLE, o.t, f'distance {o.distance_cm:.1f}cm')
        if person_close:
            self.person_clear_since = None
            return self._hold(WAIT_PERSON, o.t, 'person close')
        if c.use_yolo and o.stop_sign and o.t >= self.ignore_stop_until:
            self.stop_until = o.t + c.stop_sign_seconds
            return self._hold(STOP_SIGN, o.t, 'stop sign')

        if c.use_camera_lane:
            return self._lane_step(o)

        # 4) 줄 따라가기
        direction = line_direction(o.line_raw)
        if direction == 'ALL_BLACK':
            self.black_reads += 1
            if o.t - self.start_t >= c.min_lap_seconds and self.black_reads >= c.finish_reads:
                return self._action(FINISHED, STOP, o.t, 'FINISH_LINE')
        else:
            self.black_reads = 0

        if direction == 'LOST':
            if self.lost_since is None:
                self.lost_since = o.t
            lost_for = o.t - self.lost_since
            if lost_for >= 0.1 and not self.lost_counted:
                self.off_line_count += 1
                self.lost_counted = True
            if lost_for > c.lost_line_seconds:
                return self._action(EMERGENCY, STOP, o.t, 'LOST_LINE')
            # 잠깐 놓치면 직전 동작을 유지한다. 출발 직후면 움직이지 않는다.
            speeds = self.last_speeds or STOP
            return self._action(self.state if self.state in (FOLLOW, SLOW) else FOLLOW, speeds, o.t, 'line lost')
        self.lost_since, self.lost_counted = None, False

        # 5) 우리만의 기능: 사람이나 물체가 조금 멀리 있으면 미리 줄인다
        slow = c.use_feature and (o.person_far or (o.distance_cm is not None and o.distance_cm <= c.slow_cm))
        fast, inner = (c.slow_speed, c.slow_inner_speed) if slow else (c.speed, c.inner_speed)
        self.last_speeds = wheel_speeds('STRAIGHT' if direction == 'ALL_BLACK' else direction, fast, inner)
        return self._action(SLOW if slow else FOLLOW, self.last_speeds, o.t, direction)

    def _lane_step(self, o):
        """트랙 주행의 조향. 멈춰 있던 시간은 _hold 가 lost_since 를 지워 세지 않는다."""
        c = self.cfg
        lane = o.lane if o.lane is not None else Lane(LANE_WAIT)
        if lane.kind == LANE_NONE:
            return self._action(EMERGENCY, STOP, o.t, 'LOST_LINE')
        if lane.kind == LANE_SEARCH:
            if self.lost_since is None:
                self.lost_since = o.t
            lost_for = o.t - self.lost_since
            if lost_for >= 0.1 and not self.lost_counted:
                self.off_line_count += 1
                self.lost_counted = True
            if lost_for >= c.lane_lost_seconds:
                return self._action(EMERGENCY, STOP, o.t, 'LOST_LINE')
            return self._action(FOLLOW, lane.speeds, o.t, 'lane search')
        self.lost_since, self.lost_counted = None, False
        if lane.kind == LANE_WAIT:
            return self._action(FOLLOW, STOP, o.t, 'lane width wait')
        return self._action(FOLLOW, lane.speeds, o.t, 'lane')

