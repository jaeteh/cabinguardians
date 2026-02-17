from dataclasses import dataclass
from enum import Enum, auto
import time
from collections import deque
from typing import Optional
import numpy as np

'''
MAKE SURE YOU INSTALL NUMPY
'''

'''
logic notes:
confidence level: 0..1 indicating how sure the system is that there is a kid
ts = timestamp
Optional means variable is allowed to have a real value or None
'''

# global states
class State(Enum):
    IDLE = auto()
    MONITOR = auto()
    SUSPECT = auto()
    CONFIRMED = auto()
    ALERTING = auto()

### NEED TO DEVELOP ALGOS FOR ALL VARIABLES IN DATACLASSES ###
# consult the sensor datasheets for outputs

# data class for thermal array
# 8x8 grid array
@dataclass
class ThermalReading:
    # use AMG88XX library to read data, calibrate, write to array
    temp_array = np.array([
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        []
    ])
    ts: float = time.time()
    t_avg: float = np.mean(temp_array) # avg temp across array
    t_max: float = np.max(temp_array) # highest temp in array
    hotspot_area: float # 0..1, percent of image occupied by hottest region
    hotspot_count: float = 0
    for t in temp_array:
        if t >= t_max * 0.9: # tune this
            hotspot_count += 1.0
    hotspot_area = hotspot_count / 64.0
    blob_score: float    # 0..1, percent estimate of how human-like the heat blob is

# data class for CO2 sensor
@dataclass
class CO2Reading:
    ts: float = time.time()
    ppm: float = # ppm concentration
    ppm_rate_per_min: float # rate of rise/fall of ppm concentration

# data class for motion sensor
@dataclass
class MotionReading:
    ts: float = time.time()
    motion: bool # is there motion? T/F
    confidence: float = 1.0  # 0..1 if available
    # starts at 1.0 and changed later by logic
    # helps grade how meaningful a motion detected is

@dataclass
# OBD
class OBDReading:
    # how to read CAN -> UART -> C for Arduino
    # if the engine is on/off
    # how long has it been since front & rear doors were opened/closed
    # general internal cabin temperature

# clamping function, makes sure the logic stays between 0 and 1
def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))

# hot car detector object
class ChildHotCarDetector:
    # tunables (start points, adjust with real data)
    HOT_CABIN_C = 35.0 # lower temp limit for what is considered a hot cabin
    HOTSPOT_HUMAN_MIN_C = 33.0 # lower temp limit for IR array
    CO2_HIGH_PPM = 1200.0 # lower limit for dangerous CO2 concentration
    CO2_RISE_PPM_PER_MIN = 100.0 # lower limit for rate of CO2 rise
    MOTION_RECENT_SEC = 20.0 # motion was last seen N seconds ago, if last motion time is less than this, it is still relevant

    SUSPECT_ENTER_CONF = 0.55 # minimum confidence level to enter suspect state
    CONFIRM_CONF = 0.78 # min conf level to confirm
    CLEAR_CONF = 0.35 # max conf level to clear state and return suspect -> monitor or alert -> idle

    SUSPECT_HOLD_SEC = 20.0 # how long conf level has to stay above suspect threshold before entering suspect state
    CONFIRM_HOLD_SEC = 30.0 # how long conf level has to stay above confirm threshold before confirm state
    CLEAR_HOLD_SEC = 60.0 # how long conf level has to stay below clear threshold before leaving activated state

    SENSOR_TIMEOUT_SEC = 5.0 # max allowed time between sensor readings before signaling potential sensor stale or failure
    # stale = stops reporting real time data even though appears to be working

    # weights (must sum <= ~1.2 then clamp)
    # how much each sensor contributes to overall confidence level, relativity matters
    WT = 0.50
    WC = 0.35
    WM = 0.25


    def __init__(self):
        self.state = State.IDLE # start with idle state
        self.state_enter_ts = time.time() # time that a state was entered

        # initialize sensor readings
        self.thermal: Optional[ThermalReading] = None
        self.co2: Optional[CO2Reading] = None
        self.motion: Optional[MotionReading] = None

        # initialize timestamps
        self.last_motion_ts: float = -1e9
        self.last_valid_thermal_ts: float = -1e9
        self.last_valid_co2_ts: float = -1e9
        self.last_valid_motion_ts: float = -1e9

        # initialize timestamp calculators
        self._conf_above_since: Optional[float] = None
        self._conf_below_since: Optional[float] = None

        # T/F is system in alert state?
        self.alerting = False

    def _heat_risk(self) -> float:
        if not self.thermal: # if there is no thermal reading stored, return 0 heat risk
            return 0.0
        # heat risk - weighted avg of t_avg + max
        t_avg = self.thermal.t_avg
        t_max = self.thermal.t_max
        # map 25C..45C to 0..1
        # change range -> change formulas
        # 25C = start of meaningful heat concern
        # 30C = starting reference point for when hotspots matter (moderate risk)
        # 45C = lower limit of extremely dangerous risk
        # divide by size of temp window to convert to dimensionless temp
        risk_avg = clamp((t_avg - 25.0) / 20.0) # dimensionless temp
        risk_max = clamp((t_max - 30.0) / 20.0)
        # weighted average of avg temp + max temp, can change weights if needed
        return clamp(0.6 * risk_avg + 0.4 * risk_max)

    def _occupant_score(self, now: float) -> float:
        # reset occupant score
        score = 0.0

        # thermal evidence
        if self.thermal:
            # reset thermal evidence score
            thermal_evidence = 0.0
            # add all indicators of hotspot to thermal evidence score, with weights
            if self.thermal.t_max >= self.HOTSPOT_HUMAN_MIN_C:
                thermal_evidence += 0.5
            thermal_evidence += 0.3 * clamp(self.thermal.hotspot_area)
            thermal_evidence += 0.2 * clamp(self.thermal.blob_score)
            # add thermal evidence score to total occupant score
            score += self.WT * clamp(thermal_evidence)

        # CO2 evidence
        if self.co2:
            # reset CO2 evidence score
            co2_evidence = 0.0
            # add all indicators of CO2 levels to CO2 score
            # weights do not add up to 1 because either can indicate risk
            if self.co2.ppm >= self.CO2_HIGH_PPM:
                co2_evidence += 0.7
            if self.co2.ppm_rate_per_min >= self.CO2_RISE_PPM_PER_MIN:
                co2_evidence += 0.6
            # add CO2 evidence score to total occupant score
            score += self.WC * clamp(co2_evidence)

        # motion evidence
        # T/F is there recent motion? compare most recent motion time to upper limit
        motion_recent = (now - self.last_motion_ts) <= self.MOTION_RECENT_SEC
        if motion_recent:
            # compute mconf level, currently can be only 0, 1, or direct confidence reading put into ingest_motion
            # check if mmWave module outputs detection confidence, signal strength, energy, or presence probability
            # if not, develop algo for detecting consecutive motion frames, reflection strength, small vs large motion, persistence of motion, filtering noise
            mconf = 1.0 if not self.motion else clamp(self.motion.confidence)
            score += self.WM * mconf

        # OBD evidence added to occupant score 

        return clamp(score)

    # T/F has the sensor reported data within the time limit for new readings?
    def _sensor_ok(self, last_ts: float, now: float) -> bool:
        return (now - last_ts) <= self.SENSOR_TIMEOUT_SEC

    # stores thermal reading and timestamp
    def ingest_thermal(self, r: ThermalReading):
        self.thermal = r
        self.last_valid_thermal_ts = r.ts

    # stores CO2 reading and timestamp
    def ingest_co2(self, r: CO2Reading):
        self.co2 = r
        self.last_valid_co2_ts = r.ts

    # stores motion reading and timestamp
    def ingest_motion(self, r: MotionReading):
        self.motion = r
        self.last_valid_motion_ts = r.ts
        if r.motion:
            self.last_motion_ts = r.ts

    def _transition(self, new_state: State, now: float):
        if new_state != self.state:
            # changes system state to current state
            self.state = new_state
            # records when new state started
            self.state_enter_ts = now
            # resets persistence timers
            self._conf_above_since = None
            self._conf_below_since = None

    # main method for process
    def update(self, now: Optional[float] = None):
        now = time.time() if now is None else now # find current timestamp

        # (optional) take count of working sensors
        thermal_ok = self._sensor_ok(self.last_valid_thermal_ts, now)
        co2_ok = self._sensor_ok(self.last_valid_co2_ts, now)
        motion_ok = self._sensor_ok(self.last_valid_motion_ts, now)

        # calculate total confidence score
        heat = self._heat_risk()
        occ = self._occupant_score(now)
        confidence = clamp(heat * occ)

        # THINK ABOUT THIS AND CHANGE HOW MUCH TO REDUCE CONF LEVEL BY
        # if too many sensors missing, reduce confidence
        ok_count = sum([thermal_ok, co2_ok, motion_ok])
        if ok_count <= 1:
            confidence *= 0.5

        # timers for persistence
        def held_above(thresh: float, hold_sec: float) -> bool:
            if confidence >= thresh:
                # if no previous high conf exists, set to now
                if self._conf_above_since is None:
                    self._conf_above_since = now
                # if previous high conf exists, calculate amount of time that high conf has been held
                # return true and transition to higher state
                return (now - self._conf_above_since) >= hold_sec
            else:
                # if conf below threshold, reset the timer
                self._conf_above_since = None
                return False

        def held_below(thresh: float, hold_sec: float) -> bool:
            if confidence <= thresh:
                if self._conf_below_since is None:
                    self._conf_below_since = now
                return (now - self._conf_below_since) >= hold_sec
            else:
                self._conf_below_since = None
                return False

        # state machine
        if self.state == State.IDLE:
            self.alerting = False
            if heat > 0.4:
                self._transition(State.MONITOR, now)

        elif self.state == State.MONITOR:
            self.alerting = False
            if held_above(self.SUSPECT_ENTER_CONF, self.SUSPECT_HOLD_SEC):
                self._transition(State.SUSPECT, now)
            elif heat < 0.2 and held_below(0.2, 10.0):
                self._transition(State.IDLE, now)

        elif self.state == State.SUSPECT:
            self.alerting = False
            if held_above(self.CONFIRM_CONF, self.CONFIRM_HOLD_SEC):
                self._transition(State.CONFIRMED, now)
            elif held_below(self.CLEAR_CONF, self.CLEAR_HOLD_SEC):
                self._transition(State.MONITOR, now)

        elif self.state == State.CONFIRMED:
            # immediate to alerting
            self.alerting = True
            self._transition(State.ALERTING, now)

        elif self.state == State.ALERTING:
            self.alerting = True
            # clear conditions
            if heat < 0.2 and held_below(self.CLEAR_CONF, self.CLEAR_HOLD_SEC):
                self.alerting = False
                self._transition(State.IDLE, now)

        return {
            "state": self.state.name,
            "heat_risk": heat,
            "occupant_score": occ,
            "confidence": confidence,
            "thermal_ok": thermal_ok,
            "co2_ok": co2_ok,
            "motion_ok": motion_ok,
            "alerting": self.alerting
        }


# example test with fake data
if __name__ == "__main__":
    d = ChildHotCarDetector()
    t0 = time.time()

    for i in range(0, 120):
        now = t0 + i

        # fake thermal ramp
        d.ingest_thermal(ThermalReading(
            ts=now, t_avg=30 + 0.05*i, t_max=33 + 0.08*i,
            hotspot_area=0.4, blob_score=0.6
        ))

        # fake CO2 rising
        d.ingest_co2(CO2Reading(ts=now, ppm=800 + 6*i, ppm_rate_per_min=120))

        # fake motion bursts
        d.ingest_motion(MotionReading(ts=now, motion=(i % 15 == 0), confidence=0.9))

        out = d.update(now)
        if i % 10 == 0:
            print(i, out)




