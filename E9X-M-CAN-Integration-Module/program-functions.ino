// Functions related to the operation of the program and debugging go here.


void read_initialize_eeprom(void) {
  stored_eeprom_checksum = EEPROM.read(0) << 8 | EEPROM.read(1);
  calculated_eeprom_checksum = eeprom_crc();

  if (stored_eeprom_checksum != calculated_eeprom_checksum){
    serial_log("EEPROM data corrupted. Restoring defaults.");

    // Defaults for when EEPROM is not initialized  
    mdrive_dsc = 0x13;
    mdrive_power = 0x30;
    mdrive_edc = 0x2A;
    mdrive_svt = 0xF1;
    dme_ckm[0][0] = 0xF1;
    dme_ckm[1][0] = 0xF1;
    dme_ckm[2][0] = 0xF1;
    #if EDC_CKM_FIX
      edc_ckm[0] = 0xF1;
      edc_ckm[1] = 0xF1;
      edc_ckm[2] = 0xF1;
    #endif

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
    EEPROM.update(15, 0);
    EEPROM.update(16, 0x10);
    EEPROM.update(17, 0);
    update_eeprom_checksum();
  } else {
    mdrive_dsc = EEPROM.read(2);
    mdrive_power = EEPROM.read(3);
    mdrive_edc = EEPROM.read(4);
    mdrive_svt = EEPROM.read(5);
    mdrive_message[1] = mdrive_dsc - 2;                                                                                             // Difference between iDrive settting and MDrive CAN message (OFF) is always 2.
                                                                                                                                    // 1 unchanged, 5 OFF, 0x11 MDM, 9 On
    mdrive_message[2] = mdrive_power;                                                                                               // Copy POWER as is.
    mdrive_message[3] = mdrive_edc;                                                                                                 // Copy EDC as is.
    if (mdrive_svt == 0xE9) {
      mdrive_message[4] = 0x41;                                                                                                     // SVT normal, MDrive OFF.
    } else if (mdrive_svt == 0xF1) {
      mdrive_message[4] = 0x81;                                                                                                     // SVT sport, MDrive OFF.
    }

    dme_ckm[0][0] = EEPROM.read(6);
    dme_ckm[1][0] = EEPROM.read(7);
    dme_ckm[2][0] = EEPROM.read(8);
    #if EDC_CKM_FIX
      edc_ckm[0] = EEPROM.read(9);
      edc_ckm[1] = EEPROM.read(10);
      edc_ckm[2] = EEPROM.read(11);
    #endif
    #if UNFOLD_WITH_DOOR
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
      max_cpu_temp = 0;
    }
    #if DOOR_VOLUME
      peristent_volume = EEPROM.read(16);
      if (peristent_volume > 0x33) {
        peristent_volume = 0x15;
      } else if (peristent_volume == 0) {
        peristent_volume = 0x10;
      }
    #endif
    #if INDICATE_TRUNK_OPENED
      visual_signal_ckm = EEPROM.read(17) == 1 ? true : false;
    #endif
    #if COMFORT_EXIT
      auto_seat_ckm = EEPROM.read(18) == 1 ? true : false;
    #endif
    #if DIM_DRL
      drl_ckm = EEPROM.read(19) == 1 ? true : false;
    #endif
    serial_log("Loaded data from EEPROM.");
  }
}


uint16_t eeprom_crc(void) {
  teensy_eeprom_crc.restart();
  for (uint8_t i = 2; i < 20; i++) {
    teensy_eeprom_crc.add(EEPROM.read(i));
  }
  return teensy_eeprom_crc.calc();
}


void update_data_in_eeprom(void) {
  EEPROM.update(2, mdrive_dsc);                                                                                                     // EEPROM lifetime approx. 100k writes. Always update, never write()!                                                                                          
  EEPROM.update(3, mdrive_power);
  EEPROM.update(4, mdrive_edc);
  EEPROM.update(5, mdrive_svt);
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
  #if INDICATE_TRUNK_OPENED
    EEPROM.update(17, visual_signal_ckm);
  #endif
  #if COMFORT_EXIT
    EEPROM.update(18, auto_seat_ckm);
  #endif
  #if DIM_DRL
    EEPROM.update(19, drl_ckm);
  #endif
  update_eeprom_checksum();
  serial_log("Saved data to EEPROM.");
}


void update_eeprom_checksum(void) {
  uint16_t new_eeprom_checksum = eeprom_crc();
  EEPROM.update(0, new_eeprom_checksum >> 8);
  EEPROM.update(1, new_eeprom_checksum & 0xFF);
}


void initialize_watchdog(void) {
  WDT_timings_t config;
  config.trigger = wdt_timeout_sec;
  config.callback = wdt_callback;
  config.timeout = wdt_timeout_sec + 3;                                                                                             // If the watchdog timer is not reset within 10s, re-start the program.
  wdt.begin(config);
}


void wdt_callback(void) {
  serial_log("Watchdog not fed. Program will reset in 3s!");
  update_data_in_eeprom();
}


#if DEBUG_MODE
void print_current_state(Stream &status_serial) {
  status_serial.println("========================= Operation ========================");
  sprintf(serial_debug_string, " Vehicle PTCAN: %s", vehicle_awake ? "Active" : "Standby");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Terminal R: %s Ignition: %s", terminal_r ? "ON" : "OFF", ignition ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Engine: %s RPM: %d", engine_running ? "ON" : "OFF", RPM / 4);
  status_serial.println(serial_debug_string);
  #if HDC || IMMOBILIZER_SEQ || FRONT_FOG_CORNER || F_NIVI
    sprintf(serial_debug_string, " Indicated speed: %d %s", indicated_speed, speed_mph ? "MPH" : "KPH");
    status_serial.println(serial_debug_string);
  #endif
  #if F_NIVI
    sprintf(serial_debug_string, " Real speed: %d %s", real_speed, speed_mph ? "MPH" : "KPH");
    status_serial.println(serial_debug_string);
  #endif
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
  sprintf(serial_debug_string, " Clutch: %s", clutch_pressed ? "Depressed" : "Released");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Car is: %s", vehicle_moving ? "Moving" : "Stationary");
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
    sprintf(serial_debug_string, " Sine Tilt: %.1f, FXX-Converted: %.1f deg", sine_tilt_angle, f_vehicle_angle * 0.05 - 64.0);
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Longitudinal acceleration: %.2f g, FXX-Converted: %.2f m/s^2", 
            e_longitudinal_acceleration * 0.10197162129779283, longitudinal_acceleration * 0.002 - 65.0);
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Yaw rate FXX-Converted: %.2f deg/s", yaw_rate * 0.005 - 163.83);
    status_serial.println(serial_debug_string);
  #endif
  #if F_NIVI || MIRROR_UNDIM
    sprintf(serial_debug_string, " Outside brightness: 0x%X, %s.", rls_brightness, 
            rls_time_of_day == 0 ? "Daytime" : rls_time_of_day == 1 ? "Twilight" : "Darkness");
    status_serial.println(serial_debug_string);
  #endif
  #if IMMOBILIZER_SEQ
    status_serial.println("========================= Immobilizer ========================");
    sprintf(serial_debug_string, " Immobilizer: %s, Persistent setting: %s", 
            immobilizer_released ? "OFF" : "Active", EEPROM.read(14) == 1 ? "Active" : "OFF");
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Key detected: %s", key_valid ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  #endif
  status_serial.println("========================== MDrive ==========================");
  sprintf(serial_debug_string, " Active: %s", mdrive_status ? "YES" : "NO");
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
  sprintf(serial_debug_string, " Console POWER: %s", console_power_mode ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " POWER CKM: %s", dme_ckm[cas_key_number][0] == 0xF1 ? "Normal" : "Sport");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Key profile number: %d", cas_key_number + 1);
  status_serial.println(serial_debug_string);

  status_serial.println("======================= Body ========================");
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
    sprintf(serial_debug_string, " Passenger's seat heating: %s", passenger_seat_heating_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Passenger's seat occupied: %s, seatbelt fastened: %s",
            passenger_seat_status >= 8 ? "YES" : "NO", passenger_seat_status & 1 ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  #endif
  #if HOOD_OPEN_GONG
    sprintf(serial_debug_string, " Hood: %s", last_hood_status ? "Open" : "Closed");
    status_serial.println(serial_debug_string);
  #endif
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

  status_serial.println("========================== Debug ===========================");
  uint8_t max_stored_temp = EEPROM.read(15);
  sprintf(serial_debug_string, "CPU speed: %ld, MHz CPU temperature: %.2f, °C Max recorded: %.2f °C", 
          F_CPU_ACTUAL / 1000000, tempmonGetTemp(), max_cpu_temp > max_stored_temp ? max_cpu_temp : max_stored_temp);
  status_serial.println(serial_debug_string);
  unsigned long loop_calc = micros() - loop_timer;
  if (loop_calc > max_loop_timer) {
    max_loop_timer = loop_calc;
  }
  if (max_loop_timer > 1000) {
    sprintf(serial_debug_string, " Max loop execution time: %ld mSeconds", max_loop_timer / 1000);
  } else {
    sprintf(serial_debug_string, " Max loop execution time: %ld μSeconds", max_loop_timer);
  }
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " KCAN errors: %ld, PTCAN errors: %ld, DCAN errors: %ld", kcan_error_counter, ptcan_error_counter, dcan_error_counter);
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " EEPROM CRC: %X -> %s at boot", 
          stored_eeprom_checksum, calculated_eeprom_checksum == stored_eeprom_checksum ? "Matched" : "Mismatched");
  status_serial.println(serial_debug_string);
  status_serial.println("============================================================");
  debug_print_timer = 0;
}
#endif


void serial_log(const char message[]) {
  #if DEBUG_MODE
  if (!clearing_dtcs) {
    Serial.println(message);
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
  rvc_dipped = false;
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
  idrive_txq.flush();
  kombi_needle_txq.flush();
  vehicle_moving = false;
  wiper_txq.flush();
  wash_message_counter = 0;
  wipe_scheduled = false;
  frm_mirror_status_requested = false;
  frm_ahl_flc_status_requested = false;
  lock_button_pressed  = unlock_button_pressed = false;
  mirror_status_retry = 0;
  mirror_fold_txq.flush();
  last_lock_status_can = 0;
  vsw_initialized = false;
  vsw_current_input = 0;
  vsw_switch_counter = 0xF1;
  asd_initialized = false;
  comfort_exit_done = false;
  #if IMMOBILIZER_SEQ
    if (immobilizer_persist) {
      if (immobilizer_released) {
        activate_immobilizer();
      }
    }
  #endif
  update_data_in_eeprom();
  kcan_retry_counter = ptcan_retry_counter = dcan_retry_counter = 0;
  kcan_resend_txq.flush();
  ptcan_resend_txq.flush();
  dcan_resend_txq.flush();
}
