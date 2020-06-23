from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from cereal import car

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

# kph
FIRST_PRESS_TIME = 1
LONG_PRESS_TIME = 75

V_CRUISE_MAX = 177
V_CRUISE_MIN = 0.6 * CV.MPH_TO_KPH
V_CRUISE_LONG_PRESS_DELTA_MPH = 5
V_CRUISE_LONG_PRESS_DELTA_KPH = 10
V_CRUISE_ENABLE_MIN = 5 * CV.MPH_TO_KPH


class MPC_COST_LAT:
  PATH = 1.0
  LANE = 3.0
  HEADING = 1.0
  STEER_RATE = 1.0


class MPC_COST_LONG:
  TTC = 5.0
  DISTANCE = 0.1
  ACCELERATION = 10.0
  JERK = 20.0


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def update_v_cruise(v_cruise_kph, v_ego, gas_pressed, buttonEvents, enabled, metric):
  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  global ButtonCnt, LongPressed, ButtonPrev, PrevDisable, CurrentVspeed, PrevGaspressed
  if enabled:
    if ButtonCnt:
      ButtonCnt += 1
    for b in buttonEvents:
      if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or
                                          b.type == ButtonType.decelCruise):
        ButtonCnt = FIRST_PRESS_TIME
        ButtonPrev = b.type
      elif not b.pressed:
        LongPressed = False
        ButtonCnt = 0

    CurrentVspeed = clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)
    CurrentVspeed = CurrentVspeed if metric else (CurrentVspeed * CV.KPH_TO_MPH)
    CurrentVspeed = int(round(CurrentVspeed))

    v_cruise = v_cruise_kph if metric else int(round(v_cruise_kph * CV.KPH_TO_MPH))

    if ButtonCnt > LONG_PRESS_TIME:
      LongPressed = True
      V_CRUISE_DELTA = V_CRUISE_LONG_PRESS_DELTA_KPH if metric else V_CRUISE_LONG_PRESS_DELTA_MPH
      if ButtonPrev == ButtonType.accelCruise:
        v_cruise += V_CRUISE_DELTA - v_cruise % V_CRUISE_DELTA
      elif ButtonPrev == ButtonType.decelCruise:
        v_cruise -= V_CRUISE_DELTA - -v_cruise % V_CRUISE_DELTA
      ButtonCnt = FIRST_PRESS_TIME
    elif ButtonCnt == FIRST_PRESS_TIME and not LongPressed and not PrevDisable:
      if ButtonPrev == ButtonType.accelCruise:
        v_cruise = CurrentVspeed if gas_pressed and not PrevGaspressed and (v_cruise < CurrentVspeed) else (v_cruise + 1)
        PrevGaspressed = gas_pressed
      elif ButtonPrev == ButtonType.decelCruise:
        v_cruise = CurrentVspeed if gas_pressed and not PrevGaspressed else (v_cruise - 1)
        PrevGaspressed = gas_pressed
    elif not gas_pressed:
      PrevGaspressed = False

    v_cruise_min = V_CRUISE_MIN if metric else V_CRUISE_MIN * CV.KPH_TO_MPH
    v_cruise_max = V_CRUISE_MAX if metric else V_CRUISE_MAX * CV.KPH_TO_MPH

    v_cruise = clip(v_cruise, v_cruise_min, v_cruise_max)
    v_cruise_kph = v_cruise if metric else v_cruise * CV.MPH_TO_KPH

    v_cruise_kph = int(round(v_cruise_kph))
    PrevDisable = False
  else:
    PrevDisable = True
  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type == ButtonType.accelCruise and v_cruise_last < 250:
      return v_cruise_last
  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))
