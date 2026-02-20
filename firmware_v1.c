#include <Arduino.h>
#include <math.h>

/*
logic notes:
confidence level: 0..1 indicating how sure the system is that there is a kid
ts = timestamp
Optional means variable is allowed to have a real value or None
*/

// global states
typedef enum {
  STATE_IDLE = 0,
  STATE_MONITOR,
  STATE_SUSPECT,
  STATE_CONFIRMED,
  STATE_ALERTING
} State;

static const char* state_name(State s) {
  switch (s) {
    case STATE_IDLE: return "IDLE";
    case STATE_MONITOR: return "MONITOR";
    case STATE_SUSPECT: return "SUSPECT";
    case STATE_CONFIRMED: return "CONFIRMED";
    case STATE_ALERTING: return "ALERTING";
    default: return "UNKNOWN";
  }
}

### NEED TO DEVELOP ALGOS FOR ALL VARIABLES IN DATACLASSES ###
/* consult the sensor datasheets for outputs */

// data class for thermal array
// 8x8 grid array
typedef struct {
  // use AMG88XX library to read data, calibrate, write to array
  float temp_array[8][8];

  float ts;
  float t_avg; // avg temp across array
  float t_max; // highest temp in array
  float hotspot_area; // 0..1, percent of image occupied by hottest region
  float blob_score;   // 0..1, percent estimate of how human-like the heat blob is
} ThermalReading;

// data class for CO2 sensor
typedef struct {
  float ts;
  int th; // high level output time
  int to; // cycle output time (1004 ms +/- 5%)
  float ppm; // ppm concentration
  float ppm_rate_per_min; // rate of rise/fall of ppm concentration
} CO2Reading;

// data class for motion sensor
typedef struct {
  float ts;
  bool motion; // is there motion? T/F
  float confidence;  // 0..1 if available
  // starts at 1.0 and changed later by logic
  // helps grade how meaningful a motion detected is
} MotionReading;

typedef struct {
} OBDReading;

@dataclass
// OBD
/* class OBDReading:
    # how to read CAN -> UART -> C for Arduino
    # if the engine is on/off
    # how long has it been since front & rear doors were opened/closed
    # general internal cabin temperature
*/

// clamping function, makes sure the logic stays between 0 and 1
static float clampf(float x, float lo = 0.0f, float hi = 1.0f) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// hot car detector object
class ChildHotCarDetector {
public:
  // tunables (start points, adjust with real data)
  const float HOT_CABIN_C = 35.0f; // lower temp limit for what is considered a hot cabin
  const float HOTSPOT_HUMAN_MIN_C = 33.0f; // lower temp limit for IR array
  const float CO2_HIGH_PPM = 1200.0f; // lower limit for dangerous CO2 concentration
  const float CO2_RISE_PPM_PER_MIN = 100.0f; // lower limit for rate of CO2 rise
  const float MOTION_RECENT_SEC = 20.0f; // motion was last seen N seconds ago, if last motion time is less than this, it is still relevant

  const float SUSPECT_ENTER_CONF = 0.55f; // minimum confidence level to enter suspect state
  const float CONFIRM_CONF = 0.78f; // min conf level to confirm
  const float CLEAR_CONF = 0.35f; // max conf level to clear state and return suspect -> monitor or alert -> idle

  const float SUSPECT_HOLD_SEC = 20.0f; // how long conf level has to stay above suspect threshold before entering suspect state
  const float CONFIRM_HOLD_SEC = 30.0f; // how long conf level has to stay above confirm threshold before confirm state
  const float CLEAR_HOLD_SEC = 60.0f; // how long conf level has to stay below clear threshold before leaving activated state

  const float SENSOR_TIMEOUT_SEC = 5.0f; // max allowed time between sensor readings before signaling potential sensor stale or failure
  // stale = stops reporting real time data even though appears to be working

  // weights (must sum <= ~1.2 then clamp)
  // how much each sensor contributes to overall confidence level, relativity matters
  const float WT = 0.50f;
  const float WC = 0.35f;
  const float WM = 0.25f;

  ChildHotCarDetector() {
    this->state = STATE_IDLE; // start with idle state
    this->state_enter_ts = now_s(); // time that a state was entered

    // initialize sensor readings
    this->has_thermal = false;
    this->has_co2 = false;
    this->has_motion = false;

    // initialize timestamps
    this->last_motion_ts = -1e9f;
    this->last_valid_thermal_ts = -1e9f;
    this->last_valid_co2_ts = -1e9f;
    this->last_valid_motion_ts = -1e9f;

    // initialize timestamp calculators
    this->_conf_above_since_set = false;
    this->_conf_below_since_set = false;
    this->_conf_above_since = 0.0f;
    this->_conf_below_since = 0.0f;

    // T/F is system in alert state?
    this->alerting = false;
  }

  // stores thermal reading and timestamp
  void ingest_thermal(const ThermalReading &r) {
    this->thermal = r;
    this->has_thermal = true;
    this->last_valid_thermal_ts = r.ts;
  }

  // stores CO2 reading and timestamp
  void ingest_co2(const CO2Reading &r) {
    this->co2 = r;
    this->has_co2 = true;
    this->last_valid_co2_ts = r.ts;
  }

  // stores motion reading and timestamp
  void ingest_motion(const MotionReading &r) {
    this->motion = r;
    this->has_motion = true;
    this->last_valid_motion_ts = r.ts;
    if (r.motion) {
      this->last_motion_ts = r.ts;
    }
  }

  typedef struct {
    const char* state;
    float heat_risk;
    float occupant_score;
    float confidence;
    bool thermal_ok;
    bool co2_ok;
    bool motion_ok;
    bool alerting;
  } Output;

  // main method for process
  Output update(float now_in = NAN) {
    float now = isnan(now_in) ? now_s() : now_in; // find current timestamp

    // (optional) take count of working sensors
    bool thermal_ok = _sensor_ok(this->last_valid_thermal_ts, now);
    bool co2_ok = _sensor_ok(this->last_valid_co2_ts, now);
    bool motion_ok = _sensor_ok(this->last_valid_motion_ts, now);

    // calculate total confidence score
    float heat = _heat_risk();
    float occ = _occupant_score(now);
    float confidence = clampf(heat * occ);

    // THINK ABOUT THIS AND CHANGE HOW MUCH TO REDUCE CONF LEVEL BY
    // if too many sensors missing, reduce confidence
    int ok_count = (thermal_ok ? 1 : 0) + (co2_ok ? 1 : 0) + (motion_ok ? 1 : 0);
    if (ok_count <= 1) {
      confidence *= 0.5f;
    }

    // timers for persistence
    auto held_above = [&](float thresh, float hold_sec) -> bool {
      if (confidence >= thresh) {
        // if no previous high conf exists, set to now
        if (!this->_conf_above_since_set) {
          this->_conf_above_since = now;
          this->_conf_above_since_set = true;
        }
        // if previous high conf exists, calculate amount of time that high conf has been held
        // return true and transition to higher state
        return (now - this->_conf_above_since) >= hold_sec;
      } else {
        // if conf below threshold, reset the timer
        this->_conf_above_since_set = false;
        return false;
      }
    };

    auto held_below = [&](float thresh, float hold_sec) -> bool {
      if (confidence <= thresh) {
        if (!this->_conf_below_since_set) {
          this->_conf_below_since = now;
          this->_conf_below_since_set = true;
        }
        return (now - this->_conf_below_since) >= hold_sec;
      } else {
        this->_conf_below_since_set = false;
        return false;
      }
    };

    // state machine
    if (this->state == STATE_IDLE) {
      this->alerting = false;
      if (heat > 0.4f) {
        _transition(STATE_MONITOR, now);
      }

    } else if (this->state == STATE_MONITOR) {
      this->alerting = false;
      if (held_above(SUSPECT_ENTER_CONF, SUSPECT_HOLD_SEC)) {
        _transition(STATE_SUSPECT, now);
      } else if (heat < 0.2f && held_below(0.2f, 10.0f)) {
        _transition(STATE_IDLE, now);
      }

    } else if (this->state == STATE_SUSPECT) {
      this->alerting = false;
      if (held_above(CONFIRM_CONF, CONFIRM_HOLD_SEC)) {
        _transition(STATE_CONFIRMED, now);
      } else if (held_below(CLEAR_CONF, CLEAR_HOLD_SEC)) {
        _transition(STATE_MONITOR, now);
      }

    } else if (this->state == STATE_CONFIRMED) {
      // immediate to alerting
      this->alerting = true;
      _transition(STATE_ALERTING, now);

    } else if (this->state == STATE_ALERTING) {
      this->alerting = true;
      // clear conditions
      if (heat < 0.2f && held_below(CLEAR_CONF, CLEAR_HOLD_SEC)) {
        this->alerting = false;
        _transition(STATE_IDLE, now);
      }
    }

    Output out;
    out.state = state_name(this->state);
    out.heat_risk = heat;
    out.occupant_score = occ;
    out.confidence = confidence;
    out.thermal_ok = thermal_ok;
    out.co2_ok = co2_ok;
    out.motion_ok = motion_ok;
    out.alerting = this->alerting;
    return out;
  }

private:
  State state;
  float state_enter_ts;

  // initialize sensor readings
  ThermalReading thermal;
  CO2Reading co2;
  MotionReading motion;
  bool has_thermal;
  bool has_co2;
  bool has_motion;

  // initialize timestamps
  float last_motion_ts;
  float last_valid_thermal_ts;
  float last_valid_co2_ts;
  float last_valid_motion_ts;

  // initialize timestamp calculators
  bool _conf_above_since_set;
  bool _conf_below_since_set;
  float _conf_above_since;
  float _conf_below_since;

  // T/F is system in alert state?
  bool alerting;

  static float now_s() {
    return (float)millis() / 1000.0f;
  }

  float _heat_risk() {
    if (!this->has_thermal) { // if there is no thermal reading stored, return 0 heat risk
      return 0.0f;
    }
    // heat risk - weighted avg of t_avg + max
    float t_avg = this->thermal.t_avg;
    float t_max = this->thermal.t_max;
    // map 25C..45C to 0..1
    // change range -> change formulas
    // 25C = start of meaningful heat concern
    // 30C = starting reference point for when hotspots matter (moderate risk)
    // 45C = lower limit of extremely dangerous risk
    // divide by size of temp window to convert to dimensionless temp
    float risk_avg = clampf((t_avg - 25.0f) / 20.0f); // dimensionless temp
    float risk_max = clampf((t_max - 30.0f) / 20.0f);
    // weighted average of avg temp + max temp, can change weights if needed
    return clampf(0.6f * risk_avg + 0.4f * risk_max);
  }

  float _occupant_score(float now) {
    // reset occupant score
    float score = 0.0f;

    // thermal evidence
    if (this->has_thermal) {
      // reset thermal evidence score
      float thermal_evidence = 0.0f;
      // add all indicators of hotspot to thermal evidence score, with weights
      if (this->thermal.t_max >= this->HOTSPOT_HUMAN_MIN_C) {
        thermal_evidence += 0.5f;
      }
      thermal_evidence += 0.3f * clampf(this->thermal.hotspot_area);
      thermal_evidence += 0.2f * clampf(this->thermal.blob_score);
      // add thermal evidence score to total occupant score
      score += this->WT * clampf(thermal_evidence);
    }

    // CO2 evidence
    if (this->has_co2) {
      // reset CO2 evidence score
      float co2_evidence = 0.0f;
      // add all indicators of CO2 levels to CO2 score
      // weights do not add up to 1 because either can indicate risk
      if (this->co2.ppm >= this->CO2_HIGH_PPM) {
        co2_evidence += 0.7f;
      }
      if (this->co2.ppm_rate_per_min >= this->CO2_RISE_PPM_PER_MIN) {
        co2_evidence += 0.6f;
      }
      // add CO2 evidence score to total occupant score
      score += this->WC * clampf(co2_evidence);
    }

    // motion evidence
    // T/F is there recent motion? compare most recent motion time to upper limit
    bool motion_recent = (now - this->last_motion_ts) <= this->MOTION_RECENT_SEC;
    if (motion_recent) {
      // compute mconf level, currently can be only 0, 1, or direct confidence reading put into ingest_motion
      // check if mmWave module outputs detection confidence, signal strength, energy, or presence probability
      // if not, develop algo for detecting consecutive motion frames, reflection strength, small vs large motion, persistence of motion, filtering noise
      float mconf = (!this->has_motion) ? 1.0f : clampf(this->motion.confidence);
      score += this->WM * mconf;
    }

    // OBD evidence added to occupant score

    return clampf(score);
  }

  // T/F has the sensor reported data within the time limit for new readings?
  bool _sensor_ok(float last_ts, float now) {
    return (now - last_ts) <= this->SENSOR_TIMEOUT_SEC;
  }

  void _transition(State new_state, float now) {
    if (new_state != this->state) {
      // changes system state to current state
      this->state = new_state;
      // records when new state started
      this->state_enter_ts = now;
      // resets persistence timers
      this->_conf_above_since_set = false;
      this->_conf_below_since_set = false;
    }
  }
};

// Helper: compute avg/max + hotspot_area similarly to the Python class-side logic (done at ingest time here)
static void compute_thermal_derived(ThermalReading &r) {
  float sum = 0.0f;
  float mx = -1e9f;
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      float t = r.temp_array[i][j];
      sum += t;
      if (t > mx) mx = t;
    }
  }
  r.t_avg = sum / 64.0f;
  r.t_max = mx;

  float hotspot_count = 0.0f;
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      float t = r.temp_array[i][j];
      if (t >= r.t_max * 0.9f) { // tune this
        hotspot_count += 1.0f;
      }
    }
  }
  r.hotspot_area = hotspot_count / 64.0f;
}

static void compute_co2_derived(CO2Reading &r) {
  // ppm: float = 2000 * (th - 2) / (to - 4) # ppm concentration
  // NOTE: guard against divide-by-zero
  int denom = (r.to - 4);
  if (denom == 0) denom = 1;
  r.ppm = 2000.0f * (float)(r.th - 2) / (float)denom;
}

// example test with fake data
ChildHotCarDetector d;
float t0 = 0.0f;

void setup() {
  Serial.begin(9600); // it should be used with an arduino and serial rate 9600
  while (!Serial) { /* wait on some boards */ }

  t0 = (float)millis() / 1000.0f;
}

void loop() {
  static int i = 0;
  if (i >= 120) {
    delay(1000);
    return;
  }

  float now = t0 + (float)i;

  // fake thermal ramp
  ThermalReading tr;
  tr.ts = now;
  tr.blob_score = 0.6f;
  // Fill temp_array with something plausible; keep the ramp primarily in derived t_avg/t_max via assigned values below.
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      tr.temp_array[r][c] = 30.0f + 0.05f * (float)i;
    }
  }
  // create a "hotspot" cell to push max upward
  tr.temp_array[3][3] = 33.0f + 0.08f * (float)i;

  compute_thermal_derived(tr);
  d.ingest_thermal(tr);

  // fake CO2 rising
  CO2Reading cr;
  cr.ts = now;
  cr.ppm = 800.0f + 6.0f * (float)i;
  cr.ppm_rate_per_min = 120.0f;
  d.ingest_co2(cr);

  // fake motion bursts
  MotionReading mr;
  mr.ts = now;
  mr.motion = (i % 15 == 0);
  mr.confidence = 0.9f;
  d.ingest_motion(mr);

  ChildHotCarDetector::Output out = d.update(now);
  if (i % 10 == 0) {
    Serial.print(i);
    Serial.print(" {");
    Serial.print("\"state\":\""); Serial.print(out.state); Serial.print("\",");
    Serial.print("\"heat_risk\":"); Serial.print(out.heat_risk, 4); Serial.print(",");
    Serial.print("\"occupant_score\":"); Serial.print(out.occupant_score, 4); Serial.print(",");
    Serial.print("\"confidence\":"); Serial.print(out.confidence, 4); Serial.print(",");
    Serial.print("\"thermal_ok\":"); Serial.print(out.thermal_ok ? "true" : "false"); Serial.print(",");
    Serial.print("\"co2_ok\":"); Serial.print(out.co2_ok ? "true" : "false"); Serial.print(",");
    Serial.print("\"motion_ok\":"); Serial.print(out.motion_ok ? "true" : "false"); Serial.print(",");
    Serial.print("\"alerting\":"); Serial.print(out.alerting ? "true" : "false");
    Serial.println("}");
  }

  i++;
  delay(1000);
}
