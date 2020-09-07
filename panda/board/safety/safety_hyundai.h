const int HYUNDAI_MAX_STEER = 384;             // like stock
const int HYUNDAI_MAX_RT_DELTA = 112;          // max delta torque allowed for real time checks
const uint32_t HYUNDAI_RT_INTERVAL = 250000;   // 250ms between real time checks
const int HYUNDAI_MAX_RATE_UP = 3;
const int HYUNDAI_MAX_RATE_DOWN = 6;
const int HYUNDAI_DRIVER_TORQUE_ALLOWANCE = 50;
const int HYUNDAI_DRIVER_TORQUE_FACTOR = 2;
const int HYUNDAI_STANDSTILL_THRSLD = 30;  // ~1kph
bool hyundai_has_scc = false;
int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int OP_EMS_live = 0;
int hyundai_mdps_bus = -1;
bool hyundai_LCAN_on_bus1 = false;
bool hyundai_forward_bus1 = false;
const CanMsg HYUNDAI_TX_MSGS[] = {
  {832, 0, 8}, {832, 1, 8}, // LKAS11 Bus 0, 1
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4}, // CLU11 Bus 0, 1, 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {593, 2, 8},  // MDPS12, Bus 2
  {1056, 0, 8}, //   SCC11,  Bus 0
  {1057, 0, 8}, //   SCC12,  Bus 0
  {1290, 0, 8}, //   SCC13,  Bus 0
  {905, 0, 8},  //   SCC14,  Bus 0
  {1186, 0, 8},  //   4a2SCC, Bus 0
  {790, 1, 8}, // EMS11, Bus 1
  {912, 0, 7}, {912,1, 7}, // SPAS11, Bus 0, 1
  {1268, 0, 8}, {1268,1, 8}, // SPAS12, Bus 0, 1
 };

// TODO: missing checksum for wheel speeds message,worst failure case is
//       wheel speeds stuck at 0 and we don't disengage on brake press
AddrCheckStruct hyundai_rx_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U}}},
  {.msg = {{902, 0, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 10000U}}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_RX_CHECK_LEN = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]);

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_rx_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{902, 0, 8, .expected_timestep = 20000U}}},
  {.msg = {{916, 0, 8, .expected_timestep = 20000U}}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_LEGACY_RX_CHECK_LEN = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]);

bool hyundai_legacy = false;

static uint8_t hyundai_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == 608) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3;
  } else if (addr == 902) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == 916) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
  } else if (addr == 1057) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint8_t hyundai_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 608) {
    chksum = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == 916) {
    chksum = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == 1057) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

static uint8_t hyundai_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum = 0;
  // same algorithm, but checksum is in a different place
  for (int i = 0; i < 8; i++) {
    uint8_t b = GET_BYTE(to_push, i);
    if (((addr == 608) && (i == 7)) || ((addr == 916) && (i == 6)) || ((addr == 1057) && (i == 7))) {
      b &= (addr == 1057) ? 0x0FU : 0xF0U; // remove checksum
    }
    chksum += (b % 16U) + (b / 16U);
  }
  return (16U - (chksum %  16U)) % 16U;
}

static int hyundai_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid;
  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);
  if (hyundai_legacy) {
    valid = addr_safety_check(to_push, hyundai_legacy_rx_checks, HYUNDAI_LEGACY_RX_CHECK_LEN,
                              hyundai_get_checksum, hyundai_compute_checksum,
                              hyundai_get_counter);

  } else {
    valid = addr_safety_check(to_push, hyundai_rx_checks, HYUNDAI_RX_CHECK_LEN,
                              hyundai_get_checksum, hyundai_compute_checksum,
                              hyundai_get_counter);
  }

  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 1296 || addr == 524)) {
    if (hyundai_forward_bus1 || !hyundai_LCAN_on_bus1) {
      hyundai_LCAN_on_bus1 = true;
      hyundai_forward_bus1 = false;
    }
  }
  if (bus == 1 && hyundai_LCAN_on_bus1) {valid = false;}

  // check MDPS on Bus
  if ((addr == 593 || addr == 897) && hyundai_mdps_bus != bus) {
    if (bus == 0){
      hyundai_mdps_bus = bus;
      if (!hyundai_forward_bus1 && board_has_obd()) {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
    } else if (bus == 1 && !hyundai_LCAN_on_bus1) {
      hyundai_mdps_bus = bus;
      hyundai_forward_bus1 = true;
    } 
  }
  // check if we have a SCC on Bus1 and LCAN not on the bus
  if (bus == 1 && addr == 1057 && !hyundai_LCAN_on_bus1) {
    if (!hyundai_forward_bus1) {
      hyundai_forward_bus1 = true;
    }
  }

  if (valid) {
    if (addr == 593 && bus == hyundai_mdps_bus) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 1057 && OP_SCC_live) { // for cars with long control
      hyundai_has_scc = true;
      car_SCC_live = 50;
      // 2 bits: 13-14
      int cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    if (addr == 1056 && !OP_SCC_live) { // for cars without long control
      hyundai_has_scc = true;
      // 2 bits: 13-14
      int cruise_engaged = GET_BYTES_04(to_push) & 0x1; // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    // cruise control for car without SCC
    if (addr == 608 && bus == 0 && !hyundai_has_scc && !OP_SCC_live) {
      // bit 25
      int cruise_engaged = (GET_BYTES_04(to_push) >> 25 & 0x1); // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    // engage for Cruise control disabled car
    if (addr == 1265 && bus == 0 && OP_SCC_live && !car_SCC_live) {
      // first byte
      int cruise_button = (GET_BYTES_04(to_push) & 0x7);
      // enable on both accel and decel buttons falling edge
      if (!cruise_button && (cruise_engaged_prev == 1 || cruise_engaged_prev == 2)) {
        controls_allowed = 1;
      }
      // disable on cancel rising edge
      if (cruise_button == 4) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_button;
    }
    // exit controls on rising edge of gas press for cars with long control
    if (addr == 608 && OP_SCC_live && bus == 0) {
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0;
    }
    if (addr == 881 && hyundai_legacy && OP_SCC_live && bus == 0) {
      gas_pressed = (((GET_BYTE(to_push, 4) & 0x7F) << 1) | GET_BYTE(to_push, 3) >> 7) != 0;
    }
    // sample wheel speed, averaging opposite corners
    if (addr == 902 && bus == 0) {
      int hyundai_speed = GET_BYTES_04(to_push) & 0x3FFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 16) & 0x3FFF;  // RL
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }
    // exit controls on rising edge of brake press for cars with long control
    if (addr == 916 && OP_SCC_live && bus == 0) {
      brake_pressed = (GET_BYTE(to_push, 6) >> 7) != 0;
    }

    generic_rx_checks((addr == 832 && bus == 0));
  }
  return valid;
}

static int hyundai_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, HYUNDAI_TX_MSGS, sizeof(HYUNDAI_TX_MSGS)/sizeof(HYUNDAI_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // LKA STEER: safety check
  if (addr == 832) {
    OP_LKAS_live = 20;
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (!controls_allowed) { // a reset worsen the issue of Panda blocking some valid LKAS messages
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 1265 && !controls_allowed && (bus != hyundai_mdps_bus && hyundai_mdps_bus == 1)) { 
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  if (addr == 593) {OP_MDPS_live = 20;}
  if (addr == 1265 && bus == 1) {OP_CLU_live = 20;} // only count mesage created for MDPS
  if (addr == 1057) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;}}
  if (addr == 790) {OP_EMS_live = 20;}

  // 1 allows the message through
  return tx;
}

static int hyundai_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  int fwd_to_bus1 = -1;
  if (hyundai_forward_bus1){fwd_to_bus1 = 1;}

  // forward cam to ccan and viceversa, except lkas cmd
  if (!relay_malfunction) {
    if (bus_num == 0) {
      if (!OP_CLU_live || addr != 1265 || hyundai_mdps_bus == 0) {
        if (!OP_MDPS_live || addr != 593) {
          if (!OP_EMS_live || addr != 790) {
            bus_fwd = hyundai_forward_bus1 ? 12 : 2;
          } else {
            bus_fwd = 2;  // EON create EMS11 for MDPS
            OP_EMS_live -= 1;
          }
        } else {
          bus_fwd = fwd_to_bus1;  // EON create MDPS for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = 2; // EON create CLU12 for MDPS
        OP_CLU_live -= 1;
      }
    }
    if (bus_num == 1 && hyundai_forward_bus1) {
      if (!OP_MDPS_live || addr != 593) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 20;
        } else {
          bus_fwd = 2;  // EON create SCC11 SCC12 SCC13 SCC14 for Car
          OP_SCC_live -= 1;
        }
      } else {
        bus_fwd = 0;  // EON create MDPS for LKAS
        OP_MDPS_live -= 1;
      }
    }
    if (bus_num == 2) {
      if (!OP_LKAS_live || (addr != 832 && addr != 1157)) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = hyundai_forward_bus1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // EON create SCC12 for Car
          OP_SCC_live -= 1;
        }
      } else if (hyundai_mdps_bus == 0) {
        bus_fwd = fwd_to_bus1; // EON create LKAS and LFA for Car
        OP_LKAS_live -= 1; 
      } else {
        OP_LKAS_live -= 1; // EON create LKAS and LFA for Car and MDPS
      }
    }
  } else {
    if (bus_num == 0) {
      bus_fwd = fwd_to_bus1;
    }
    if (bus_num == 1 && hyundai_forward_bus1) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static void hyundai_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = false;

  if (board_has_obd()) {
    current_board->set_can_mode(CAN_MODE_OBD_CAN2);
    }
}

static void hyundai_legacy_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = true;

  if (board_has_obd()) {
    current_board->set_can_mode(CAN_MODE_OBD_CAN2);
    }
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
  .addr_check = hyundai_rx_checks,
  .addr_check_len = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]),
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
  .addr_check = hyundai_legacy_rx_checks,
  .addr_check_len = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]),
};
