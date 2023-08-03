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
    #if CKM
      dme_ckm[0][0] = 0xF1;
      dme_ckm[1][0] = 0xF1;
      dme_ckm[2][0] = 0xF1;
    #endif
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

    #if CKM
      dme_ckm[0][0] = EEPROM.read(6);
      dme_ckm[1][0] = EEPROM.read(7);
      dme_ckm[2][0] = EEPROM.read(8);
    #endif
    #if EDC_CKM_FIX
      edc_ckm[0] = EEPROM.read(9);
      edc_ckm[1] = EEPROM.read(10);
      edc_ckm[2] = EEPROM.read(11);
    #endif
    #if UNFOLD_WITH_DOOR
      unfold_with_door_open = EEPROM.read(12) == 1 ? true : false;
    #endif
    #if ANTI_THEFT_SEQ
      anti_theft_released = EEPROM.read(13) == 1 ? true : false;
      anti_theft_persist = EEPROM.read(14) == 1 ? true : false;
      if (!anti_theft_persist) {
        anti_theft_released = true;                                                                                                 // Deactivate anti-theft at boot if this value is set.
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
    serial_log("Loaded data from EEPROM.");
  }
}


uint16_t eeprom_crc(void) {
  teensy_eep_crc.restart();
  for (uint8_t i = 2; i < 18; i++) {
    teensy_eep_crc.add(EEPROM.read(i));
  }
  return teensy_eep_crc.calc();
}


void update_data_in_eeprom(void) {
  EEPROM.update(2, mdrive_dsc);                                                                                                     // EEPROM lifetime approx. 100k writes. Always update, never write()!                                                                                          
  EEPROM.update(3, mdrive_power);
  EEPROM.update(4, mdrive_edc);
  EEPROM.update(5, mdrive_svt);
  #if CKM
    EEPROM.update(6, dme_ckm[0][0]);
    EEPROM.update(7, dme_ckm[1][0]);
    EEPROM.update(8, dme_ckm[2][0]);
  #endif 
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
  status_serial.println("============= Operation ============");
  sprintf(serial_debug_string, " Vehicle PTCAN: %s", vehicle_awake ? "Active" : "Standby");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Terminal R: %s", terminal_r ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Ignition: %s", ignition ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Engine: %s", engine_running ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " RPM: %d", RPM / 4);
  status_serial.println(serial_debug_string);
  #if HDC || ANTI_THEFT_SEQ || FRONT_FOG_CORNER || F_NIVI
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
  
  sprintf(serial_debug_string, " Clutch: %s", clutch_pressed ? "Pressed" : "Released");
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
    sprintf(serial_debug_string, " Sine Tilt: %.1f, Converted: %.1f deg", sine_tilt_angle, f_vehicle_angle * 0.05 - 64.0);
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Outside brightness: 0x%X %s.", rls_brightness, 
            rls_time_of_day == 0 ? "Daytime" : rls_time_of_day == 1 ? "Twilight" : "Darkness");
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Longitudinal acceleration: %.2f g, %.2f m/s^2", e_long_acceleration * 0.10197162129779283, longitudinal_acceleration * 0.002 - 65.0);
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Yaw rate: %.2f deg/s", (yaw_rate - 163.84) * 0.005);
    status_serial.println(serial_debug_string);
  #endif
  #if ANTI_THEFT_SEQ
    status_serial.println("============= Antitheft ============");
    sprintf(serial_debug_string, " Anti-theft: %s", anti_theft_released ? "OFF" : "Active");
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Anti-theft persistent setting: %s", EEPROM.read(14) == 1 ? "Active" : "OFF");
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Key detected: %s", key_valid ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  #endif
  status_serial.println("============== MDrive ==============");
  if (ignition) {
    sprintf(serial_debug_string, " Active: %s", mdrive_status ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  } else {
    status_serial.println(" Inactive");
  }
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
  #if CKM
    sprintf(serial_debug_string, " POWER CKM: %s", dme_ckm[cas_key_number][0] == 0xF1 ? "Normal" : "Sport");
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " Key profile number: %d", cas_key_number + 1);
    status_serial.println(serial_debug_string);
  #endif

  status_serial.println("=========== Body ============");
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
  #if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR || FRONT_FOG_CORNER
    sprintf(serial_debug_string, " Reverse gear: %s", reverse_gear_status ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  #if MSA_RVC
    sprintf(serial_debug_string, " PDC: %s", pdc_status > 0x80 ? "ON" : "OFF");
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
    #if AUTO_SEAT_HEATING_PASS
      sprintf(serial_debug_string, " Passenger's seat heating: %s", passenger_seat_heating_status ? "ON" : "OFF");
      status_serial.println(serial_debug_string);
      sprintf(serial_debug_string, " Passenger's seat occupied: %s", (passenger_seat_status >= 8) ? "YES" : "NO");
      status_serial.println(serial_debug_string);
      sprintf(serial_debug_string, " Passenger's seatbelt fastened: %s", passenger_seat_status & 1 ? "YES" : "NO");
      status_serial.println(serial_debug_string);
    #endif
  #endif
  #if DOOR_VOLUME
    #if RHD
      sprintf(serial_debug_string, " Passenger's door: %s", left_door_open ? "Open" : "Closed");
    #else
      sprintf(serial_debug_string, " Driver's's door: %s", left_door_open ? "Open" : "Closed");
    #endif
    status_serial.println(serial_debug_string);
    #if RHD
      sprintf(serial_debug_string, " Driver's door: %s", right_door_open ? "Open" : "Closed");
    #else
      sprintf(serial_debug_string, " Passenger's door: %s", right_door_open ? "Open" : "Closed");
    #endif
    status_serial.println(serial_debug_string);
    sprintf(serial_debug_string, " iDrive volume: 0x%X", peristent_volume);
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
  #endif
  #if DIM_DRL || FRONT_FOG_CORNER
    sprintf(serial_debug_string, " Indicators: %s", indicators_on ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
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

  status_serial.println("============== Debug ===============");
  sprintf(serial_debug_string, " CPU temperature: %.2f °C", tempmonGetTemp());
  status_serial.println(serial_debug_string);
  uint8_t max_stored_temp = EEPROM.read(15);
  sprintf(serial_debug_string, " Max recorded CPU temperature: %.2f °C", max_cpu_temp > max_stored_temp ? max_cpu_temp : max_stored_temp);
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " CPU speed: %ld MHz", F_CPU_ACTUAL / 1000000);
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
  sprintf(serial_debug_string, " KCAN errors: %ld PTCAN errors: %ld DCAN errors: %ld", kcan_error_counter, ptcan_error_counter, dcan_error_counter);
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " EEPROM CRC: %X -> %s at boot", stored_eeprom_checksum, calculated_eeprom_checksum == stored_eeprom_checksum ? "Matched" : "Mismatched");
  status_serial.println(serial_debug_string);
  status_serial.println("====================================");
  debug_print_timer = 0;
}
#endif


void serial_log(const char message[]) {
  #if DEBUG_MODE
    Serial.println(message);
  #endif
}


void reset_runtime_variables(void) {                                                                                                // Ignition OFF. Set variables to pre-ignition state.
  dsc_program_status = 0;
  if (mdrive_status) {
    toggle_mdrive_message_active();
  }
  engine_running = false;
  engine_runtime = 0;
  RPM = 0;
  ignore_m_press = ignore_m_hold = false;
  mdrive_power_active = restore_console_power_mode = false;
  mfl_pressed_count = 0;
  #if SERVOTRONIC_SVT70
    uif_read = false;
  #endif
  #if CKM
    console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;                                                         // When cycling ignition, restore this to its CKM value.
  #endif
  #if EDC_CKM_FIX
    edc_mismatch_check_counter = 0;
    edc_ckm_txq.flush();
  #endif
  dsc_txq.flush();
  #if AUTO_SEAT_HEATING
    seat_heating_dr_txq.flush();
    #if AUTO_SEAT_HEATING_PASS
      seat_heating_pas_txq.flush();
    #endif
  #endif
  #if REVERSE_BEEP
    pdc_beep_txq.flush();
    pdc_beep_sent = false;
  #endif
  #if HDC || FAKE_MSA
    ihk_extra_buttons_cc_txq.flush();
  #endif
  #if EXHAUST_FLAP_CONTROL
    exhaust_flap_sport = false;
    #if !QUIET_START
      actuate_exhaust_solenoid(LOW);
    #endif
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    lc_cc_active = mdm_with_lc = false;
  #endif
  #if CONTROL_SHIFTLIGHTS
    shiftlights_segments_active = engine_coolant_warmed_up = false;
    ignore_shiftlights_off_counter = 0;
    last_var_rpm_can = 0;
    START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
    MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
    MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
  #endif
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
  #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER
    front_fog_status = false;
  #endif
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
  #if FTM_INDICATOR
    ftm_indicator_status = false;
  #endif
  #if FRM_HEADLIGHT_MODE
    kcan_write_msg(frm_ckm_ahl_komfort_buf);
  #endif
  #if HDC
    cruise_control_status = hdc_button_pressed = hdc_requested = hdc_active = false;
  #endif
  #if FAKE_MSA
    msa_button_pressed = false;
    msa_fake_status_counter = 0;
  #endif
  #if MSA_RVC
    pdc_status = 0x80;
    pdc_button_pressed = pdc_with_rvc_requested = false;
  #endif
  #if ANTI_THEFT_SEQ
    reset_key_cc();
  #endif
  #if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR || FRONT_FOG_CORNER || MSA_RVC || F_NIVI
    reverse_gear_status = false;
  #endif
  #if F_NIVI
    sine_angle_requested = false;
  #endif
}


void reset_sleep_variables(void) {
  #if AUTO_SEAT_HEATING
    driver_sent_seat_heating_request = false;                                                                                       // Reset the seat heating request now that the car's asleep.
    driver_seat_heating_status = false;
    #if AUTO_SEAT_HEATING_PASS
      passenger_sent_seat_heating_request = false;
      passenger_seat_status = 0;
      passenger_seat_heating_status = false;
    #endif
    #if AUTO_STEERING_HEATER
      sent_steering_heating_request = false;
    #endif
  #endif
  #if DOOR_VOLUME
    volume_reduced = false;                                                                                                         // In case the car falls asleep with the door open.
    volume_restore_offset = 0;
    initial_volume_set = false;
    idrive_txq.flush();
  #endif
  #if NEEDLE_SWEEP
    kombi_needle_txq.flush();
  #endif
  vehicle_moving = false;
  #if WIPE_AFTER_WASH
    wiper_txq.flush();
    wash_message_counter = 0;
    wipe_scheduled = false;
  #endif
  #if AUTO_MIRROR_FOLD
    frm_status_requested = false;
    lock_button_pressed  = unlock_button_pressed = false;
    mirror_fold_txq.flush();
  #endif
  #if AUTO_MIRROR_FOLD || INDICATE_TRUNK_OPENED
    last_lock_status_can = 0;
  #endif
  #if F_VSW01
    vsw_initialized = false;
  #endif
  #if ANTI_THEFT_SEQ
    if (anti_theft_persist) {
      activate_anti_theft();
    }
  #endif
  update_data_in_eeprom();
  kcan_retry_counter = ptcan_retry_counter = dcan_retry_counter = 0;
  kcan_resend_txq.flush();
  ptcan_resend_txq.flush();
  dcan_resend_txq.flush();
}
