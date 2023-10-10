// Functions related to the operation of the program and debugging go here.


void read_initialize_eeprom(void) {
  uint16_t stored_eeprom_checksum = EEPROM.read(0) << 8 | EEPROM.read(1);
  uint16_t calculated_eeprom_checksum = eeprom_crc();

  if (stored_eeprom_checksum != calculated_eeprom_checksum){
    sprintf(serial_debug_string, "EEPROM data corrupted. Loaded checksum: 0x%X, calculated: 0x%X. Restoring defaults.",
            stored_eeprom_checksum, calculated_eeprom_checksum);
    serial_log(serial_debug_string, 1);

    EEPROM.update(2, mdrive_dsc);                                                                           
    EEPROM.update(3, mdrive_power);
    EEPROM.update(4, mdrive_edc);
    EEPROM.update(5, mdrive_svt);
    EEPROM.update(6, 0xF1);
    EEPROM.update(7, 0xF1);
    EEPROM.update(8, 0xF1);
    EEPROM.update(9, 0xF1);
    EEPROM.update(10, 0xF1);
    EEPROM.update(11, 0xF1);
    EEPROM.update(12, 0);
    EEPROM.update(13, 1);
    EEPROM.update(14, 1);
    EEPROM.update(15, 0xFF);
    EEPROM.update(16, 0x10);
    EEPROM.update(17, 0);
    EEPROM.update(18, 0);
    EEPROM.update(19, 0);
    EEPROM.update(20, 0);
    EEPROM.update(21, 0);
    EEPROM.update(22, 0);
    EEPROM.update(23, 1);
    EEPROM.update(24, 1);
    EEPROM.update(25, 1);
    EEPROM.update(26, 0);
    EEPROM.update(27, 0);
    EEPROM.update(28, 0);

    update_eeprom_checksum();
  } else {
    mdrive_dsc = EEPROM.read(2);
    mdrive_power = EEPROM.read(3);
    mdrive_edc = EEPROM.read(4);
    mdrive_svt = EEPROM.read(5);
    dme_ckm[0][0] = EEPROM.read(6);
    dme_ckm[1][0] = EEPROM.read(7);
    dme_ckm[2][0] = EEPROM.read(8);
    #if EDC_CKM_FIX
      edc_ckm[0] = EEPROM.read(9);
      edc_ckm[1] = EEPROM.read(10);
      edc_ckm[2] = EEPROM.read(11);
    #endif
    #if AUTO_MIRROR_FOLD
      unfold_with_door_open = EEPROM.read(12) == 1 ? true : false;
    #endif
    #if IMMOBILIZER_SEQ
      immobilizer_released = EEPROM.read(13) == 1 ? true : false;
      immobilizer_persist = EEPROM.read(14) == 1 ? true : false;
      if (!immobilizer_persist) {
        immobilizer_released = true;                                                                                                // Deactivate immobilizer at boot if this value is set.
      }
    #endif
    max_cpu_temp = EEPROM.read(15);
    if (max_cpu_temp == 0xFF) {
      max_cpu_temp = -40.0;
    }
    #if DOOR_VOLUME
      peristent_volume = EEPROM.read(16);
      if (peristent_volume > 0x33) {
        peristent_volume = 0x15;
      } else if (peristent_volume == 0) {
        peristent_volume = 0x10;
      }
    #endif
    visual_signal_ckm[0] = EEPROM.read(17) == 1 ? true : false;
    visual_signal_ckm[1] = EEPROM.read(18) == 1 ? true : false;
    visual_signal_ckm[2] = EEPROM.read(19) == 1 ? true : false;
    #if COMFORT_EXIT
      auto_seat_ckm[0] = EEPROM.read(20) == 1 ? true : false;
      auto_seat_ckm[1] = EEPROM.read(21) == 1 ? true : false;
      auto_seat_ckm[2] = EEPROM.read(22) == 1 ? true : false;
    #endif
    #if DIM_DRL
      drl_ckm[0] = EEPROM.read(23) == 1 ? true : false;
      drl_ckm[1] = EEPROM.read(24) == 1 ? true : false;
      drl_ckm[2] = EEPROM.read(25) == 1 ? true : false;
    #endif
    serial_commands_unlocked = EEPROM.read(26) == 1 ? true : false;
    doors_locked = EEPROM.read(27) == 1 ? true : false;
    doors_alarmed = EEPROM.read(28) == 1 ? true : false;
    serial_log("Loaded data from EEPROM.", 2);
  }
}


uint16_t eeprom_crc(void) {
  teensy_eeprom_crc.restart();
  for (uint8_t i = 2; i < 29; i++) {
    teensy_eeprom_crc.add(EEPROM.read(i));
  }
  return teensy_eeprom_crc.calc();
}


void update_data_in_eeprom(void) {                                                                                                  // EEPROM lifetime approx. 100k writes. Always update, never write()!
  if (!key_guest_profile) {
    EEPROM.update(2, mdrive_dsc);
    EEPROM.update(3, mdrive_power);
    EEPROM.update(4, mdrive_edc);
    EEPROM.update(5, mdrive_svt);
  }
  EEPROM.update(6, dme_ckm[0][0]);
  EEPROM.update(7, dme_ckm[1][0]);
  EEPROM.update(8, dme_ckm[2][0]);
  #if EDC_CKM_FIX
    EEPROM.update(9, edc_ckm[0]);
    EEPROM.update(10, edc_ckm[1]);
    EEPROM.update(11, edc_ckm[2]);
  #endif
  #if AUTO_MIRROR_FOLD
    EEPROM.update(12, unfold_with_door_open);
  #endif
  EEPROM.update(15, round(max_cpu_temp));
  #if DOOR_VOLUME
    EEPROM.update(16, peristent_volume);
  #endif
  EEPROM.update(17, visual_signal_ckm[0]);
  EEPROM.update(18, visual_signal_ckm[1]);
  EEPROM.update(19, visual_signal_ckm[2]);
  #if COMFORT_EXIT
    EEPROM.update(20, auto_seat_ckm[0]);
    EEPROM.update(21, auto_seat_ckm[1]);
    EEPROM.update(22, auto_seat_ckm[2]);
  #endif
  #if DIM_DRL
    EEPROM.update(23, drl_ckm[0]);
    EEPROM.update(24, drl_ckm[1]);
    EEPROM.update(25, drl_ckm[2]);
  #endif
  EEPROM.update(27, doors_locked);
  EEPROM.update(28, doors_alarmed);
  update_eeprom_checksum();
  serial_log("Saved data to EEPROM.", 2);
}


void update_eeprom_checksum(void) {
  uint16_t new_eeprom_checksum = eeprom_crc();
  EEPROM.update(0, new_eeprom_checksum >> 8);
  EEPROM.update(1, new_eeprom_checksum & 0xFF);
}


void initialize_watchdog(void) {
  WDT_timings_t config;
  config.trigger = 15;
  config.callback = wdt_callback;
  config.timeout = 18;                                                                                                              // If the watchdog timer is not reset within 15s, re-start the program.
  wdt.begin(config);
  serial_log("WDT initialized.", 2);
}


void wdt_callback(void) {
  serial_log("Watchdog not fed. Program will reset in 3s!", 0);
  update_data_in_eeprom();
}


void print_current_state(Stream &status_serial) {
  sprintf(serial_debug_string, " ================================ Operation ===============================\r\n"
          " Vehicle PTCAN: %s\r\n"
          " Terminal R: %s, Ignition: %s\r\n"
          " Engine: %s, RPM: %d\r\n"
          " Indicated speed: %d, Real speed: %d %s",
          vehicle_awake ? "Active" : "Standby", terminal_r ? "ON" : "OFF",
          ignition ? "ON" : "OFF", engine_running ? "ON" : "OFF", RPM / 4,
          indicated_speed, real_speed, speed_mph ? "MPH" : "KPH");
  status_serial.println(serial_debug_string);
  #if HDC
    sprintf(serial_debug_string, " Cruise Control: %s", cruise_control_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  if (vehicle_awake) {
    sprintf(serial_debug_string, " Voltage: %.2f V", battery_voltage);
    status_serial.println(serial_debug_string);
  } else {
    status_serial.println(" Voltage: Unknown");
  }
  if (ignition) {
    if (dsc_program_status == 0) {
      status_serial.println(" DSC: Fully ON");
    } else if (dsc_program_status == 1) {
      status_serial.println(" DSC: DTC/MDM mode");
    } else {
      status_serial.println(" DSC: Fully OFF");
    }
  } else {
    status_serial.println(" DSC: Asleep");
  }
  sprintf(serial_debug_string, " Clutch: %s\r\n Car is: %s",
          clutch_pressed ? "Depressed" : "Released",
          vehicle_moving ? "Moving" : "Stationary");
  status_serial.println(serial_debug_string);
  #if F_NIVI
    if (vehicle_direction == 8) {
      status_serial.println(" Direction: None");
    } else if (vehicle_direction == 9) {
      status_serial.println(" Direction: Forward");
    } else if (vehicle_direction == 0xA) {
      status_serial.println(" Direction: Backward");
    } else {
      status_serial.println(" Direction: Unknown");
    }
    sprintf(serial_debug_string, " Sine Tilt: %.1f, FXX-Converted: %.1f deg\r\n"
            " Longitudinal acceleration: %.2f g, FXX-Converted: %.2f m/s^2\r\n"
            " Yaw rate FXX-Converted: %.2f deg/s",
            sine_tilt_angle, f_vehicle_angle * 0.05 - 64.0, 
            e_longitudinal_acceleration * 0.10197162129779283, longitudinal_acceleration * 0.002 - 65.0, 
            yaw_rate * 0.005 - 163.83);
    status_serial.println(serial_debug_string);
  #endif
  #if F_NIVI || MIRROR_UNDIM
    sprintf(serial_debug_string, " Outside brightness: 0x%X, %s.", rls_brightness, 
            rls_time_of_day == 0 ? "Daytime" : rls_time_of_day == 1 ? "Twilight" : "Darkness");
    status_serial.println(serial_debug_string);
  #endif
  #if IMMOBILIZER_SEQ
    sprintf(serial_debug_string, " ================================ Immobilizer ===============================\r\n"
            " Immobilizer: %s, Persistent setting: %s\r\n"
            " Key detected: %s",
            immobilizer_released ? "OFF" : "Active", EEPROM.read(14) == 1 ? "Active" : "OFF",
            key_valid ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  #endif
  sprintf(serial_debug_string, " ================================= MDrive =================================\r\n"
          " Active: %s", mdrive_status ? "YES" : "NO");
  status_serial.println(serial_debug_string);
  status_serial.print(" Settings: DSC-");
  if (mdrive_dsc == 3) {
    status_serial.print("Unchanged");
  } else if (mdrive_dsc == 7) {
    status_serial.print("OFF");
  } else if (mdrive_dsc == 0x13) {
    status_serial.print("DTC/MDM");
  } else if (mdrive_dsc == 0xB) {
    status_serial.print("ON");
  }
  status_serial.print("  POWER-");
  if (mdrive_power == 0) {
    status_serial.print("Unchanged");
  } else if (mdrive_power == 0x10) {
    status_serial.print("Normal");
  } else if (mdrive_power == 0x20) {
    status_serial.print("Sport");
  } else if (mdrive_power == 0x30) {
    status_serial.print("Sport+");
  }
  status_serial.print("  EDC-");
  if (mdrive_edc == 0x20) {
    status_serial.print("Unchanged");
  } else if (mdrive_edc == 0x21) {
    status_serial.print("Comfort");
  } else if (mdrive_edc == 0x22) {
    status_serial.print("Normal");
  } else if (mdrive_edc == 0x2A) {
    status_serial.print("Sport");
  }
  status_serial.print("  SVT-");
  if (mdrive_svt == 0xE9) {
    status_serial.print("Normal");
  } else if (mdrive_svt == 0xF1) {
    status_serial.print("Sport");
  }
  status_serial.println();
  sprintf(serial_debug_string, " Console POWER: %s\r\n"
          " POWER CKM: %s\r\n"
          " Key profile number: %d\r\n"
          " ============================== Body ===============================",
          console_power_mode ? "ON" : "OFF",
          dme_ckm[cas_key_number][0] == 0xF1 ? "Normal" : "Sport",
          cas_key_number + 1);
  status_serial.println(serial_debug_string);

  status_serial.println();
  #if RTC
    time_t t = now();
    uint8_t rtc_hours = hour(t);
    uint8_t rtc_minutes = minute(t);
    uint8_t rtc_seconds = second(t);
    uint8_t rtc_day = day(t);
    uint8_t rtc_month = month(t);
    uint16_t rtc_year = year(t);
    sprintf(serial_debug_string, " RTC: %s%d:%s%d:%s%d %s%d/%s%d/%d %s", 
            rtc_hours > 9 ? "" : "0", rtc_hours, rtc_minutes > 9 ? "" : "0", rtc_minutes, 
            rtc_seconds > 9 ? "" : "0", rtc_seconds, 
            rtc_day > 9 ? "" : "0", rtc_day, rtc_month > 9 ? "" : "0", rtc_month, rtc_year,
            rtc_valid ? "Valid" : "Invalid");
    status_serial.println(serial_debug_string);
  #endif
  sprintf(serial_debug_string, " Reverse gear: %s", reverse_gear_status ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  #if PDC_AUTO_OFF
    sprintf(serial_debug_string, " Handbrake: %s", handbrake_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  #if MSA_RVC
    sprintf(serial_debug_string, " PDC: %s", pdc_bus_status > 0x80 ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  if (ambient_temperature_real != 87.5) {
      sprintf(serial_debug_string, " Ambient temp: %.1f °C", ambient_temperature_real);
    } else {
      sprintf(serial_debug_string, " Ambient temp: Unknown");
    }
  status_serial.println(serial_debug_string);
  #if AUTO_SEAT_HEATING
    sprintf(serial_debug_string, " Driver's seat heating: %s", driver_seat_heating_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  #if AUTO_SEAT_HEATING_PASS
    sprintf(serial_debug_string, " Passenger's seat heating: %s, occupied: %s, seatbelt fastened: %s", 
            passenger_seat_heating_status ? "ON" : "OFF", bitRead(passenger_seat_status, 0) ? "YES" : "NO",
            bitRead(passenger_seat_status, 3) ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  #endif
  #if HOOD_OPEN_GONG
    sprintf(serial_debug_string, " Hood: %s", last_hood_status ? "Open" : "Closed");
    status_serial.println(serial_debug_string);
  #endif
  sprintf(serial_debug_string, " Door locks: %s, alarm: %s", doors_locked ? "ON" : "OFF", doors_alarmed ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  #if DOOR_VOLUME
    #if RHD
      sprintf(serial_debug_string, " Passenger's door: %s, Driver's door: %s", 
              left_door_open ? "Open" : "Closed", right_door_open ? "Open" : "Closed");
    #else
      sprintf(serial_debug_string, " Driver's's door: %s, Passenger's door: %s", 
              left_door_open ? "Open" : "Closed", right_door_open ? "Open" : "Closed");
    #endif
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " iDrive volume: 0x%X", peristent_volume);
    status_serial.println(serial_debug_string);
  #endif
  #if F_VSW01
    sprintf(serial_debug_string, " VSW switch input: %d", vsw_current_input);
    status_serial.println(serial_debug_string);
  #endif
  #if EXHAUST_FLAP_CONTROL
    sprintf(serial_debug_string, " Exhaust flap: %s", exhaust_flap_open ? "Open" : "Closed");
    status_serial.println(serial_debug_string);
  #endif
  #if FRONT_FOG_LED_INDICATOR
    sprintf(serial_debug_string, " Front fogs: %s", front_fog_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
   #if FRONT_FOG_CORNER
    sprintf(serial_debug_string, " Dipped beam: %s", dipped_beam_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
    #if FRONT_FOG_CORNER_AHL_SYNC
      sprintf(serial_debug_string, " AHL: %s", ahl_active ? "ON" : "OFF");
      status_serial.println(serial_debug_string);
    #endif
  #endif
  sprintf(serial_debug_string, " Indicators: %s", indicators_on ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  #if DIM_DRL
    sprintf(serial_debug_string, " DRL: %s", drl_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  #if FTM_INDICATOR
    if (ignition) {
      sprintf(serial_debug_string, " FTM indicator: %s", ftm_indicator_status ? "ON" : "OFF");
      status_serial.println(serial_debug_string);
    } else {
      status_serial.println(" FTM indicator: Inactive");
    }
  #endif

  int8_t max_stored_temp = EEPROM.read(15);
  sprintf(serial_debug_string, " ================================= Debug ==================================\r\n"
          " CPU speed: %ld, MHz CPU temperature: %.2f, °C Max recorded: %.2f °C", 
          F_CPU_ACTUAL / 1000000, cpu_temp, max_cpu_temp > max_stored_temp ? max_cpu_temp : max_stored_temp);
  status_serial.println(serial_debug_string);
  if (clock_mode == 0) {
    status_serial.println(" Clock mode: STANDARD_CLOCK");
  } else if (clock_mode == 1) {
    status_serial.println(" Clock mode: MILD_UNDERCLOCK");
  } else if (clock_mode == 2) {
    status_serial.println(" Clock mode: MEDIUM_UNDERCLOCK");
  } else if (clock_mode == 3) {
    status_serial.println(" Clock mode: HIGH_UNDERCLOCK");
  } else if (clock_mode == 4) {
    status_serial.println(" Clock mode: MAX_UNDERCLOCK");
  } else {
    status_serial.println(" Clock mode: STARTUP_CLOCK");
  }
  unsigned long loop_calc = micros() - loop_timer;
  if (loop_calc > max_loop_timer) {
    max_loop_timer = loop_calc;
  }
  if (max_loop_timer > 1000) {
    sprintf(serial_debug_string, " Max loop execution time: %.2f ms", max_loop_timer / 1000.0);
  } else {
    sprintf(serial_debug_string, " Max loop execution time: %ld μs", max_loop_timer);
  }
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " KCAN errors: %ld, PTCAN errors: %ld, DCAN errors: %ld\r\n"
          " ==========================================================================",
          kcan_error_counter, ptcan_error_counter, dcan_error_counter);
  status_serial.println(serial_debug_string);
  status_serial.println();
  debug_print_timer = 0;
}


void serial_log(const char message[], int8_t level) {
  #if DEBUG_MODE
    if (!clearing_dtcs) {
      if (level <= LOGLEVEL) {
        Serial.println(message);
      }
    }
    if (millis() <= 10000) {                                                                                                        // Store early messages that won't be on the Serial monitor.
      char buffer[128];
      if (micros() < 1000) {
        sprintf(buffer, " %ld μs: %s\r\n", micros(), message);
      } else if (millis() < 100) {
        sprintf(buffer, " %.2f ms: %s\r\n", micros() / 1000.0, message);
      } else {
        sprintf(buffer, " %ld ms: %s\r\n", millis(), message);
      }
      strcat(boot_debug_string, buffer);
    }
  #endif
}


void reset_ignition_variables(void) {                                                                                               // Ignition OFF. Set variables to pre-ignition state.
  dsc_program_status = 0;
  if (mdrive_status) {
    toggle_mdrive_message_active();
  }
  engine_running = false;
  RPM = 0;
  ignore_m_press = ignore_m_hold = false;
  mdrive_power_active = restore_console_power_mode = false;
  m_mfl_held_count = 0;
  uif_read = false;
  console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;                                                           // When cycling ignition, restore this to its CKM value.
  edc_mismatch_check_counter = 0;
  edc_ckm_txq.flush();
  dsc_txq.flush();
  seat_heating_dr_txq.flush();
  seat_heating_pas_txq.flush();
  reverse_beep_sent = pdc_too_close = false;
  reverse_beep_resend_timer = 0;
  pdc_tone_on = false;
  ihk_extra_buttons_cc_txq.flush();
  hdc_txq.flush();
  pdc_buttons_txq.flush();
  exhaust_flap_sport = false;
  #if !QUIET_START
    actuate_exhaust_solenoid(LOW);
  #endif
  lc_cc_active = mdm_with_lc = false;
  shiftlights_segments_active = engine_coolant_warmed_up = false;
  ignore_shiftlights_off_counter = 0;
  startup_animation_active = false;
  last_var_rpm_can = 0;
  START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
  MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
  MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
  digitalWrite(POWER_LED_PIN, LOW);
  #if FRONT_FOG_LED_INDICATOR
    digitalWrite(FOG_LED_PIN, LOW);
  #endif
  #if AUTO_STEERING_HEATER
    if (transistor_active) {
      digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);
      transistor_active = false;
    }
  #endif
  front_fog_status = false;
  #if FRONT_FOG_CORNER
    fog_corner_left_txq.flush();
    fog_corner_right_txq.flush();
    if (((millis() - last_fog_action_timer) < 15000) || (left_fog_on || right_fog_on)) {                                            // This is a limitation of controlling lights via 6F1. Whatever state is set with
      m = {front_left_fog_off_buf, millis()};                                                                                       // job STEUERN_LAMPEN_PWM persists for 15s.
      fog_corner_left_txq.push(&m);
      m = {front_right_fog_off_buf, millis() + 100};
      fog_corner_right_txq.push(&m);
    }
    dipped_beam_status = left_fog_on = right_fog_on = indicators_on = false;
  #endif
  ahl_active = flc_active = false;
  #if DIM_DRL
    dim_drl_txq.flush();
    if (((millis() - last_drl_action_timer) < 15000) || (left_dimmed || right_dimmed)) {
      m = {left_drl_dim_off, millis()};
      dim_drl_txq.push(&m);
      m = {right_drl_dim_off, millis() + 100};
      dim_drl_txq.push(&m);
    }
    drl_status = left_dimmed = right_dimmed = false;
  #endif
  ftm_indicator_status = false;
  #if FRM_AHL_MODE
    kcan_write_msg(frm_ckm_ahl_komfort_buf);
  #endif
  cruise_control_status = hdc_button_pressed = hdc_requested = hdc_active = false;
  msa_button_pressed = false;
  msa_fake_status_counter = 0;
  pdc_bus_status = 0x80;
  pdc_button_pressed = pdc_with_rvc_requested = false;
  rvc_tow_view_by_module = rvc_tow_view_by_driver = false;
  #if IMMOBILIZER_SEQ
    reset_key_cc();
  #endif
  reverse_gear_status = false;
  sine_angle_requested = false;
}


void reset_sleep_variables(void) {
  driver_sent_seat_heating_request = false;                                                                                         // Reset the seat heating request now that the car's asleep.
  driver_seat_heating_status = false;
  passenger_sent_seat_heating_request = false;
  passenger_seat_status = 0;
  passenger_seat_heating_status = false;
  sent_steering_heating_request = false;
  volume_reduced = false;                                                                                                           // In case the car falls asleep with the door open.
  volume_restore_offset = 0;
  initial_volume_set = false;
  key_guest_profile = false;
  idrive_txq.flush();
  kombi_needle_txq.flush();
  vehicle_moving = false;
  wiper_txq.flush();
  wash_message_counter = stalk_down_message_counter = stalk_down_last_press_time = 0;
  wipe_scheduled = intermittent_wipe_active = false, auto_wipe_active = false;
  frm_mirror_status_requested = false;
  frm_ahl_flc_status_requested = false;
  fold_lock_button_pressed = fold_unlock_button_pressed = false;
  lock_button_pressed = false;
  lock_button_pressed_counter = 0;
  mirror_status_retry = 0;
  mirror_fold_txq.flush();
  last_lock_status_can = 0;
  vsw_initialized = false;
  vsw_current_input = 0;
  vsw_switch_counter = 0xF1;
  asd_initialized = false;
  comfort_exit_done = false;
  full_indicator = false;
  #if IMMOBILIZER_SEQ
    if (immobilizer_persist) {
      if (immobilizer_released) {
        activate_immobilizer();
      }
    }
  #endif
  hazards_flash_txq.flush();
  update_data_in_eeprom();
  kcan_retry_counter = ptcan_retry_counter = dcan_retry_counter = 0;
  kcan_resend_txq.flush();
  ptcan_resend_txq.flush();
  dcan_resend_txq.flush();
}
