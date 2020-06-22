from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, \
                                             create_scc11, create_scc12, create_mdps12, \
                                             create_scc13, create_scc14
from selfdrive.car.hyundai.interface import GearShifter
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longcontrol import LongCtrlState

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel Hard limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 3.  # 1.5 m/s2
ACCEL_MIN = -8.  # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

CONTROL1_BP = [0.15, 0.1, 0.08, 0.05, 0.]
CONTROL1_A = [0.2, 0.2, 0.22, 0.1, 0.]
CONTROL2_BP = [0.15, 0.1, 0.08, 0.05, 0.0]
CONTROL2_A = [0.8, 0.5, 0.3, 0.3, 0.25]
CONTROL3_BP = [0.15, 0.3]
CONTROL3_A = [1.0, 2.0]

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart, button_on):
  sys_warning = (visual_alert == VisualAlert.steerRequired)
  if sys_warning:
      sys_warning = 4 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 3

  # initialize to no lane visible
  sys_state = 1
  if not button_on:
    sys_state = 0
  if left_lane and right_lane or sys_warning:  #HUD alert only display when LKAS status is active
    if enabled or sys_warning:
      sys_state = 3
    else:
      sys_state = 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.accel_steady = 0
    self.apply_steer_last = 0
    self.steer_rate_limited = False
    self.lkas11_cnt = 0
    self.scc12_cnt = 0
    self.resume_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 0
    self.turning_signal_timer = 0
    self.lkas_button_on = True
    self.longcontrol = True #TODO: make auto
    self.fs_error = False
    self.update_live = False
    self.scc_live = not CP.radarOffCan
    self.lead_visible = False
    self.lead_debounce = 0
    self.apply_accel_last = 0
    self.spas_accel = 0
    self.op_spas_brake_state = 0
    self.op_spas_state = -1
    self.phasecount = 0

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart, set_speed, lead_visible):

    # *** compute control surfaces ***

    # gas and brake
    apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    # Steering Torque
    new_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    # temporarily disable steering when LKAS button off 
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90. and self.lkas_button_on

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 60 * CV.KPH_TO_MS and self.car_fingerprint == CAR.HYUNDAI_GENESIS and not CS.mdps_bus:
      lkas_active = 0

    # Disable steering while turning blinker on and speed below 60 kph
    if CS.out.leftBlinker or CS.out.rightBlinker:
      if self.car_fingerprint not in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_H]:
        self.turning_signal_timer = 100  # Disable for 1.0 Seconds after blinker turned off
      elif CS.left_blinker_flash or CS.right_blinker_flash: # Optima has blinker flash signal only
        self.turning_signal_timer = 100
    if self.turning_indicator_alert: # set and clear by interface
      lkas_active = 0
    if self.turning_signal_timer > 0:
      self.turning_signal_timer -= 1

    if not lkas_active:
      apply_steer = 0

    self.apply_accel_last = apply_accel
    self.apply_steer_last = apply_steer
  
    if self.update_live or (CS.lkas11["CF_Lkas_FusionState"] == 0):
       self.fs_error = CS.lkas11["CF_Lkas_FusionState"]
       self.update_live = True

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(lkas_active, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart,
                        self.lkas_button_on)

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph  else 60
    if clu11_speed > enabled_speed or not lkas_active or CS.out.gearShifter == GearShifter.reverse:
      enabled_speed = clu11_speed

    if CS.is_set_speed_in_mph:
      set_speed *= CV.MS_TO_MPH
    else:
      set_speed *= CV.MS_TO_KPH

    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"]
      self.scc12_cnt = CS.scc12["CR_VSM_Alive"] + 1 if not CS.no_radar else 0
      self.prev_scc_cnt = CS.scc11["AliveCounterACC"]
      self.scc_update_frame = frame

    # check if SCC on bus 0 is live
    if frame % 7 == 0 and not CS.no_radar:
      if CS.scc11["AliveCounterACC"] == self.prev_scc_cnt:
        if frame - self.scc_update_frame > 20 and self.scc_live:
          self.scc_live = False
      else:
        self.scc_live = True
        self.prev_scc_cnt = CS.scc11["AliveCounterACC"]
        self.scc_update_frame = frame

    self.prev_scc_cnt = CS.scc11["AliveCounterACC"]

    self.lkas11_cnt = (self.lkas11_cnt + 1) % 0x10
    self.scc12_cnt %= 0xF

    can_sends = []
    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.fs_error, 0))

    if CS.mdps_bus or CS.scc_bus == 1: # send lkas11 bus 1 if mdps or scc is on bus 1
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.fs_error, 1))
    if frame % 2 and CS.mdps_bus == 1: # send clu11 to mdps if it is not on bus 0
      can_sends.append(create_clu11(self.packer, frame, CS.mdps_bus, CS.clu11, Buttons.NONE, enabled_speed))

    if pcm_cancel_cmd and self.longcontrol:
      can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.CANCEL, clu11_speed))
    elif CS.mdps_bus: # send mdps12 to LKAS to prevent LKAS error if no cancel cmd
      can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    if lead_visible:
      self.lead_visible = True
      self.lead_debounce = 50
    elif self.lead_debounce > 0:
      self.lead_debounce -= 1
    else:
      self.lead_visible = lead_visible

    self.acc_paused = True if (CS.out.brakePressed or CS.out.gasPressed or CS.out.brakeHold) else False

    self.acc_standstill = True if (LongCtrlState.stopping and CS.out.standstill) else False

    self.prev_spas_accel = self.spas_accel

    # todo add all parking type enumeration below
    # reverse parking left - 18
    # parallel parking right - 19
    # parking exit left - 40
    if CS.spas_on:
      if (self.op_spas_state == -1):
        print('SPAS ON = ')
      self.op_spas_state = 0  # SPAS enabled

    if self.op_spas_state == 0 and \
       ((CS.prev_spas_hmi_state != 18 and CS.spas_hmi_state == 18) or
       (CS.prev_spas_hmi_state != 19 and CS.spas_hmi_state == 19)):
    #   (CS.prev_spas_hmi_state != 40 and CS.spas_hmi_state == 40)):
      self.op_spas_state = 1  # space found
      self.op_spas_brake_state = 12
      self.phasecount = 0
      print('SPACE FOUND')
      print('Phase = 0')
      print('XXX################%%####################')
      print('XXXX###########%%%%%%##################')
      print('XXX#############%%####################')

    if self.op_spas_state == 1 and CS.prev_spas_hmi_state != CS.spas_hmi_state and CS.spas_hmi_state == 23:
      self.op_spas_state = 2  # move in Reverse after space found
      self.op_spas_brake_state = 11
      self.phasecount = 1
      print('1st Phase Movement')
      print('Phase = 1')
      print('################%%####################')
      print('##############%%%%%%##################')
      print('################%%####################')
    elif self.op_spas_state == 2 and CS.prev_spas_hmi_state == 23 and CS.spas_hmi_state == 25 \
          and not CS.out.gearShifter == GearShifter.drive:
      self.op_spas_brake_state = 13  # shift to Drive
      self.phasecount = 1
      print('Brake for Phase 1')
      print('####!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!####')
      print('###########!!!%%%%%%!!!###############')
      print('###############!%%!###################')
    elif self.op_spas_state == 2 and CS.spas_hmi_state == 25 \
           and CS.out.gearShifter == GearShifter.drive:
      self.op_spas_state = 3   # move in Drive
      self.op_spas_brake_state = 11
      self.phasecount = 2
      print('Phase = 2, Drive')
      print('################%%####################')
      print('##############%%%%%%##################')
      print('################%%####################')
    elif self.op_spas_state > 2 and CS.prev_spas_hmi_state == 25 and CS.spas_hmi_state == 26 \
          and not CS.out.gearShifter == GearShifter.reverse:
      self.op_spas_brake_state = 13  # shift to Reverse
      print('Brake for Phase =', self.phasecount)
      print('####!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!####')
      print('###########!!!%%%%%%!!!###############')
      print('###############!%%!###################')
    elif self.op_spas_state > 2 and CS.spas_hmi_state == 26 \
           and CS.out.gearShifter == GearShifter.reverse:
      self.op_spas_state = 4  # move to Reverse
      self.op_spas_brake_state = 11
      self.phasecount += 1
      print('Reverse, Phase = ', self.phasecount)
      print('################%%####################')
      print('##############%%%%%%##################')
      print('################%%####################')
    elif self.op_spas_state > 2 and CS.prev_spas_hmi_state == 26 and CS.spas_hmi_state == 25 \
          and not CS.out.gearShifter == GearShifter.drive:
      self.op_spas_brake_state = 13  # shift to Drive
      print('Brake for Phase =', self.phasecount)
      print('####!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!####')
      print('###########!!!%%%%%%!!!###############')
      print('###############!%%!###################')
    elif self.op_spas_state > 2 and CS.spas_hmi_state == 25 \
           and CS.out.gearShifter == GearShifter.drive:
      self.op_spas_state = 5   # move in Drive
      self.op_spas_brake_state = 11
      self.phasecount += 1
      print('Drive, Phase = ', self.phasecount)
      print('################%%####################')
      print('##############%%%%%%##################')
      print('################%%####################')

    if CS.out.gearShifter == GearShifter.drive:
      if CS.front_sensor_state > 2 and self.op_spas_brake_state < 13:
        self.op_spas_brake_state = 13
        print('Brake for Front Sensor =', CS.front_sensor_state)
        print('##$$$$$$$$$$$##!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!##$$$$$$$$$$$##')
        print('***#######$$$##!!!%%%%%%!!!##$$$###########*****************')
        print('@@@@@###############!%%!###################@@@@@@@@@@@@@@@@@')
      elif CS.front_sensor_state == 2 and self.op_spas_brake_state == 11:
        self.op_spas_brake_state = 12
        print('Brake for Front Sensor =', CS.front_sensor_state)
        print('##$$$$$$$$$$$##!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!##$$$$$$$$$$$##')
        print('***#######$$$##!!!%%%%%%!!!##$$$###########*****************')
        print('@@@@@###############!%%!###################@@@@@@@@@@@@@@@@@')

    if CS.out.gearShifter == GearShifter.reverse:
      if CS.rear_sensor_state > 2 and self.op_spas_brake_state < 13:
        self.op_spas_brake_state = 13
        print('Brake for Front Sensor =', CS.rear_sensor_state)
        print('##$$$$$$$$$$$##!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!##$$$$$$$$$$$##')
        print('***#######$$$##!!!%%%%%%!!!##$$$###########*****************')
        print('@@@@@###############!%%!###################@@@@@@@@@@@@@@@@@')
      elif CS.rear_sensor_state == 2 and self.op_spas_brake_state == 11:
        self.op_spas_brake_state = 12
        print('Brake for Front Sensor =', CS.rear_sensor_state)
        print('##$$$$$$$$$$$##!!!!!!!!!!!!%%!!!!!!!!!!!!!!!!##$$$$$$$$$$$##')
        print('***#######$$$##!!!%%%%%%!!!##$$$###########*****************')
        print('@@@@@###############!%%!###################@@@@@@@@@@@@@@@@@')

    if self.op_spas_brake_state == 13:
      self.spas_accel = -interp(CS.out.vEgo, CONTROL3_BP, CONTROL3_A)
    elif self.op_spas_brake_state == 12:
      self.spas_accel = -interp((CS.out.vEgo - 0.25), CONTROL2_BP, CONTROL2_A)
    elif self.op_spas_brake_state == 11:
      self.spas_accel = -interp((CS.out.vEgo - 0.4), CONTROL1_BP, CONTROL1_A)

    if not CS.spas_on or CS.out.vEgo > 2. or CS.out.gearShifter == GearShifter.park:
      self.op_spas_state = -1  # no control
      self.op_spas_brake_state = 0
      self.spas_accel = 0
      self.prev_spas_accel  = 0

    if self.spas_accel > self.prev_spas_accel:
      self.spas_accel = self.prev_spas_accel + 0.03

    # send scc to car if longcontrol enabled and SCC not on bus 0 or ont live
    if self.longcontrol and (CS.scc_bus or not self.scc_live) and frame % 2 == 0: 
      can_sends.append(create_scc12(self.packer, apply_accel, enabled,
                                    self.acc_standstill, self.acc_paused,
                                    self.op_spas_brake_state, self.spas_accel,
                                    self.scc12_cnt, CS.scc12))

      can_sends.append(create_scc11(self.packer, frame, enabled,
                                    set_speed, self.lead_visible,
                                    CS.out.standstill, CS.scc11))

      #if CS.has_scc13 and frame % 20 == 0:
      #  can_sends.append(create_scc13(self.packer, CS.scc13))
      #if CS.has_scc14:
      #  can_sends.append(create_scc14(self.packer, enabled, CS.scc14))
      #self.scc12_cnt += 1

    if CS.out.cruiseState.standstill and not self.longcontrol:
      # run only first time when the car stopped
      if self.last_lead_distance == 0:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
          self.resume_cnt = 0
    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0  

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.SONATA_H, CAR.SANTA_FE]:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    return can_sends
