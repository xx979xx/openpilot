const int HYUNDAI_MAX_STEER = 409;             // like stock
const int HYUNDAI_MAX_RT_DELTA = 200;          // max delta torque allowed for real time checks
const uint32_t HYUNDAI_RT_INTERVAL = 250000;   // 250ms between real time checks
const int HYUNDAI_MAX_RATE_UP = 4;
const int HYUNDAI_MAX_RATE_DOWN = 7;
const int HYUNDAI_DRIVER_TORQUE_ALLOWANCE = 50;
const int HYUNDAI_DRIVER_TORQUE_FACTOR = 2;
const int HYUNDAI_STANDSTILL_THRSLD = 30;  // ~1kph
const CanMsg HYUNDAI_TX_MSGS[] = {
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 0, 4}, // CLU11 Bus 0
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {832, 1, 8},{1265, 1, 4}, {1265, 2, 4}, {593, 2, 8}, {1057, 0, 8}, {790, 1, 8}, {912, 0, 7}, {912,1, 7}, {1268, 0, 8}, {1268,1, 8},
  {1056, 0, 8}, //   SCC11,  Bus 0
  {1057, 0, 8}, //   SCC12,  Bus 0
  {1290, 0, 8}, //   SCC13,  Bus 0
  {905, 0, 8},  //   SCC14,  Bus 0
  {1186, 0, 8}  //   4a2SCC, Bus 0
 };

// TODO: missing checksum for wheel speeds message,worst failure case is
//       wheel speeds stuck at 0 and we don't disengage on brake press
// TODO: refactor addr check to cleanly re-enable commented out checks for cars that have them
AddrCheckStruct hyundai_rx_checks[] = {
  //{.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U}}},
  // TODO: older hyundai models don't populate the counter bits in 902
  //{.msg = {{902, 0, 8, .max_counter = 15U,  .expected_timestep = 10000U}}},
  //{.msg = {{902, 0, 8, .max_counter = 0U,  .expected_timestep = 10000U}}},
  //{.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}}},
  //{.msg = {{916, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 10000U}}},
 // {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_RX_CHECK_LEN = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]);

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_rx_checks[] = {
//  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
//           {881, 0, 8, .expected_timestep = 10000U}}},
//  {.msg = {{902, 0, 8, .expected_timestep = 10000U}}},
//  {.msg = {{916, 0, 8, .expected_timestep = 10000U}}},
//  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_LEGACY_RX_CHECK_LEN = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]);

bool hyundai_legacy = false;

bool hyundai_has_scc = false;
int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int hyundai_mdps_bus = 0;
bool hyundai_LCAN_on_bus1 = false;
bool hyundai_forward_bus1 = false;


static int hyundai_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid;
  if (0) {
    valid = addr_safety_check(to_push, hyundai_legacy_rx_checks, HYUNDAI_LEGACY_RX_CHECK_LEN,
                              NULL, NULL,
                              NULL);

  } else {
    valid = addr_safety_check(to_push, hyundai_rx_checks, HYUNDAI_RX_CHECK_LEN,
                              NULL, NULL,
                              NULL);
  }


  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  if (valid && ((bus == 0) || (bus == 1) || (bus == 2))) {
    // check if we have a LCAN on Bus1
    if ((bus == 1) && (addr == 1296 || addr == 524)) {
      if (hyundai_forward_bus1 || !hyundai_LCAN_on_bus1) {
        hyundai_LCAN_on_bus1 = true;
        hyundai_forward_bus1 = false;
      }
    }
    // check if we have a MDPS on Bus1 and LCAN not on the bus
    if ((bus == 1) && (addr == 593 || addr == 897) && (!hyundai_LCAN_on_bus1)) {
       if ((hyundai_mdps_bus != bus) || (!hyundai_forward_bus1)) {
        hyundai_mdps_bus = bus;
        hyundai_forward_bus1 = true;
      }
    }
    // check if we have a SCC on Bus1 and LCAN not on the bus
    if ((bus == 1) && (addr == 1057) && (!hyundai_LCAN_on_bus1)) {
      if (!hyundai_forward_bus1) {
        hyundai_forward_bus1 = true;
      }
    }
    if ((addr == 593) && (bus == hyundai_mdps_bus)) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 1056 && (bus != 1 || !hyundai_LCAN_on_bus1)) {
      hyundai_has_scc = true;
      car_SCC_live = 50;
      int cruise_engaged;
      if (0) { // for cars with long control
        cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3; // 2 bits: 13-14
      } else if (1) { // for cars without long control
        cruise_engaged = GET_BYTES_04(to_push) & 0x1; // ACC main_on signal
      }
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    gas_pressed = false;
    // sample subaru wheel speed, averaging opposite corners
    if ((addr == 902) && (bus == 0)) {
      int hyundai_speed = GET_BYTES_04(to_push) & 0x3FFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 16) & 0x3FFF;  // RL
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }


    brake_pressed = false;
    if(bus == 0){
      generic_rx_checks((addr == 832));
    }
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
  if ((addr == 1265) && (!controls_allowed) && ((bus != hyundai_mdps_bus) || (!hyundai_mdps_bus))) { 
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  if (addr == 593) {OP_MDPS_live = 20;}
  if ((addr == 1265) && bus == 1) {OP_CLU_live = 20;} // only count mesage to mdps
  if (addr == 1057) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;}}

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
      if (!OP_CLU_live || addr != 1265 || !hyundai_mdps_bus) {
        if (!OP_MDPS_live || addr != 593) {
          bus_fwd = hyundai_forward_bus1 ? 12 : 2;
        } else {
          bus_fwd = fwd_to_bus1;  // EON create MDPS for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = 2; // EON create CLU11 for MDPS
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
      if (addr != 832 || !OP_LKAS_live) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = hyundai_forward_bus1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // EON create SCC12 for Car
          OP_SCC_live -= 1;
        }
      } else if (!hyundai_mdps_bus) {
        bus_fwd = fwd_to_bus1; // EON create LKAS for Car
        OP_LKAS_live -= 1; 
      } else {
        OP_LKAS_live -= 1; // EON create LKAS for Car and MDPS
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
}

static void hyundai_legacy_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = true;
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
