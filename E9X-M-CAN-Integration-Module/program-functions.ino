// Functions related to the operation of the program and debugging go here.


void read_initialize_eeprom(void) {
  uint16_t stored_eeprom_checksum = EEPROM.read(0) << 8 | EEPROM.read(1);
  uint16_t calculated_eeprom_checksum = eeprom_crc();

  if (stored_eeprom_checksum != calculated_eeprom_checksum){
    sprintf(serial_debug_string, "EEPROM data corrupted. Stored checksum: 0x%X, calculated checksum: 0x%X. Restoring defaults.",
            stored_eeprom_checksum, calculated_eeprom_checksum);
    serial_log(serial_debug_string, 1);

    EEPROM.update(2, mdrive_dsc[0]);                                                                                                // MDrive
    EEPROM.update(3, mdrive_power[0]);
    EEPROM.update(4, mdrive_edc[0]);
    EEPROM.update(5, mdrive_svt[0]);
    EEPROM.update(6, 0xFF);                                                                                                         // Blank for future use
    EEPROM.update(7, 0xFF);
    EEPROM.update(8, mdrive_dsc[1]);
    EEPROM.update(9, mdrive_power[1]);
    EEPROM.update(10, mdrive_edc[1]);
    EEPROM.update(11, mdrive_svt[1]);   
    EEPROM.update(12, 0xFF);
    EEPROM.update(13, 0xFF);
    EEPROM.update(14, mdrive_dsc[2]);
    EEPROM.update(15, mdrive_power[2]);
    EEPROM.update(16, mdrive_edc[2]);
    EEPROM.update(17, mdrive_svt[2]);
    EEPROM.update(18, 0xFF);
    EEPROM.update(19, 0xFF);
    EEPROM.update(20, 0xF1);                                                                                                        // DME CKM
    EEPROM.update(21, 0xF1);
    EEPROM.update(22, 0xF1);
    EEPROM.update(23, 1);                                                                                                           // Torque / Power units
    EEPROM.update(24, 1);
    EEPROM.update(25, 1);
    EEPROM.update(26, 1);
    EEPROM.update(27, 1);
    EEPROM.update(28, 1);
    EEPROM.update(29, 0);                                                                                                           // Un-fold mirrors when unlocking
    EEPROM.update(30, 1);                                                                                                           // Immobilizer
    EEPROM.update(31, 1);
    EEPROM.update(32, 0xFF);                                                                                                        // CPU temp
    EEPROM.update(33, 0x10);                                                                                                        // CIC persistent volume
    EEPROM.update(34, 0);                                                                                                           // Visual acknowledge CKM
    EEPROM.update(35, 0);
    EEPROM.update(36, 0);
    EEPROM.update(37, 0);                                                                                                           // Automatic seat position CKM
    EEPROM.update(38, 0);
    EEPROM.update(39, 0);
    EEPROM.update(40, 1);                                                                                                           // DRL CKM
    EEPROM.update(41, 1);
    EEPROM.update(42, 1);
    EEPROM.update(43, 0);
    EEPROM.update(44, 0);                                                                                                           // Door locks
    EEPROM.update(45, 0);

    update_eeprom_checksum();
  } else {
    mdrive_dsc[0] = EEPROM.read(2);
    mdrive_power[0] = EEPROM.read(3);
    mdrive_edc[0] = EEPROM.read(4);
    mdrive_svt[0] = EEPROM.read(5);
    mdrive_dsc[1] = EEPROM.read(8);
    mdrive_power[1] = EEPROM.read(9);
    mdrive_edc[1] = EEPROM.read(10);
    mdrive_svt[1] = EEPROM.read(11);
    mdrive_dsc[2] = EEPROM.read(14);
    mdrive_power[2] = EEPROM.read(15);
    mdrive_edc[2] = EEPROM.read(16);
    mdrive_svt[2] = EEPROM.read(17);
    #if !F_NBT                                                                                                                      // NBT does not have M-Key settings.
      dme_ckm[0][0] = EEPROM.read(20);
      dme_ckm[1][0] = EEPROM.read(21);
      dme_ckm[2][0] = EEPROM.read(22);
    #endif
    #if F_NBT
      torque_unit[0] = EEPROM.read(23);
      power_unit[0] = EEPROM.read(24);
      torque_unit[0] = EEPROM.read(25);
      power_unit[0] = EEPROM.read(26);
      torque_unit[0] = EEPROM.read(27);
      power_unit[0] = EEPROM.read(28);
    #endif
    #if AUTO_MIRROR_FOLD
      unfold_with_door_open = EEPROM.read(29) == 1 ? true : false;
    #endif
    #if IMMOBILIZER_SEQ
      immobilizer_released = EEPROM.read(30) == 1 ? true : false;
      immobilizer_persist = EEPROM.read(31) == 1 ? true : false;
      if (!immobilizer_persist) {
        immobilizer_released = true;                                                                                                // Deactivate immobilizer at boot if this value is set.
      }
    #endif
    max_cpu_temp = EEPROM.read(32);
    if (max_cpu_temp == 0xFF) {
      max_cpu_temp = -40.0;
    }
    #if DOOR_VOLUME && !F_NBT
      peristent_volume = EEPROM.read(33);
      if (peristent_volume > 0x33) {
        peristent_volume = 0x15;
      } else if (peristent_volume == 0) {
        peristent_volume = 0x10;
      }
    #endif
    visual_signal_ckm[0] = EEPROM.read(34) == 1 ? true : false;
    visual_signal_ckm[1] = EEPROM.read(35) == 1 ? true : false;
    visual_signal_ckm[2] = EEPROM.read(36) == 1 ? true : false;
    #if COMFORT_EXIT
      auto_seat_ckm[0] = EEPROM.read(37) == 1 ? true : false;
      auto_seat_ckm[1] = EEPROM.read(38) == 1 ? true : false;
      auto_seat_ckm[2] = EEPROM.read(39) == 1 ? true : false;
    #endif
    #if DIM_DRL
      drl_ckm[0] = EEPROM.read(40) == 1 ? true : false;
      drl_ckm[1] = EEPROM.read(41) == 1 ? true : false;
      drl_ckm[2] = EEPROM.read(42) == 1 ? true : false;
    #endif
    serial_commands_unlocked = EEPROM.read(43) == 1 ? true : false;
    doors_locked = EEPROM.read(44) == 1 ? true : false;
    doors_alarmed = EEPROM.read(45) == 1 ? true : false;
    serial_log("Loaded data from EEPROM.", 2);
  }
}


uint16_t eeprom_crc(void) {
  teensy_eeprom_crc.restart();
  for (uint8_t i = 2; i < 46; i++) {
    teensy_eeprom_crc.add(EEPROM.read(i));
  }
  return teensy_eeprom_crc.calc();
}


void update_data_in_eeprom(void) {                                                                                                  // EEPROM lifetime approx. 100k writes. Always update, never write()!
  if (!key_guest_profile) {
    EEPROM.update(2, mdrive_dsc[0]);
    EEPROM.update(3, mdrive_power[0]);
    EEPROM.update(4, mdrive_edc[0]);
    EEPROM.update(5, mdrive_svt[0]);
    EEPROM.update(8, mdrive_dsc[1]);
    EEPROM.update(9, mdrive_power[1]);
    EEPROM.update(10, mdrive_edc[1]);
    EEPROM.update(11, mdrive_svt[1]);
    EEPROM.update(14, mdrive_dsc[2]);
    EEPROM.update(15, mdrive_power[2]);
    EEPROM.update(16, mdrive_edc[2]);
    EEPROM.update(17, mdrive_svt[2]);
  }
  #if !F_NBT
    EEPROM.update(20, dme_ckm[0][0]);
    EEPROM.update(21, dme_ckm[1][0]);
    EEPROM.update(22, dme_ckm[2][0]);
  #endif
  #if F_NBT
    EEPROM.update(23, torque_unit[0]);
    EEPROM.update(24, power_unit[0]);
    EEPROM.update(25, torque_unit[1]);
    EEPROM.update(26, power_unit[1]);
    EEPROM.update(27, torque_unit[2]);
    EEPROM.update(28, power_unit[2]);
  #endif
  #if AUTO_MIRROR_FOLD
    EEPROM.update(29, unfold_with_door_open);
  #endif
  EEPROM.update(32, round(max_cpu_temp));
  #if DOOR_VOLUME && !F_NBT
    EEPROM.update(33, peristent_volume);
  #endif
  EEPROM.update(34, visual_signal_ckm[0]);
  EEPROM.update(35, visual_signal_ckm[1]);
  EEPROM.update(36, visual_signal_ckm[2]);
  #if COMFORT_EXIT
    EEPROM.update(37, auto_seat_ckm[0]);
    EEPROM.update(38, auto_seat_ckm[1]);
    EEPROM.update(39, auto_seat_ckm[2]);
  #endif
  #if DIM_DRL
    EEPROM.update(40, drl_ckm[0]);
    EEPROM.update(41, drl_ckm[1]);
    EEPROM.update(42, drl_ckm[2]);
  #endif
  EEPROM.update(44, doors_locked);
  EEPROM.update(45, doors_alarmed);
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
  config.trigger = 20;
  config.callback = wdt_callback;
  config.timeout = 23;                                                                                                              // If the watchdog timer is not reset, re-start the program.
  wdt.begin(config);
  serial_log("WDT initialized.", 2);
}


void wdt_callback(void) {
  serial_log("Watchdog not fed. Program will reset in 3s!", 0);
  update_data_in_eeprom();
}


void print_current_state(Stream &status_serial) {
  sprintf(serial_debug_string, "\r\n ================================ Operation ===============================\r\n"
          " Vehicle PTCAN: %s\r\n"
          " Terminal R: %s, Ignition: %s\r\n"
          " Engine: %s, RPM: %d, Torque: %.1f Nm, idling: %s\r\n"
          " Coolant temperature: %d °C, Oil temperature %d °C\r\n"
          " Indicated speed: %d, Real speed: %d %s",
          vehicle_awake ? "ON" : "OFF", terminal_r ? "ON" : "OFF", ignition ? "ON" : "OFF",
          engine_running ? "ON" : "OFF", RPM / 4, engine_torque, engine_idling ? "YES" : "NO",
          engine_coolant_temperature - 48, engine_oil_temperature - 48,
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
    if (e_vehicle_direction == 0) {
      status_serial.println(" Direction: None");
    } else if (e_vehicle_direction == 1) {
      status_serial.println(" Direction: Forward");
    } else if (e_vehicle_direction == 2) {
      status_serial.println(" Direction: Backward");
    } else {
      status_serial.println(" Direction: Unknown");
    }
    sprintf(serial_debug_string, " Sine Tilt: %.1f, FXX-Converted: %.1f deg\r\n"
            " Longitudinal acceleration: %.2f g, FXX-Converted: %.2f m/s^2\r\n"
            " Lateral acceleration: %.2f g, FXX-Converted: %.2f m/s^2\r\n"
            " Yaw rate FXX-Converted: %.2f deg/s",
            sine_tilt_angle, f_vehicle_angle * 0.05 - 64.0, 
            e_longitudinal_acceleration * 0.10197162129779283, longitudinal_acceleration * 0.002 - 65.0,
            e_lateral_acceleration * 0.10197162129779283, lateral_acceleration * 0.002 - 65.0,
            yaw_rate * 0.005 - 163.84);
    status_serial.println(serial_debug_string);
  #endif
  #if F_NBT || F_NIVI || MIRROR_UNDIM
    sprintf(serial_debug_string, " Outside brightness: 0x%X, %s.", rls_brightness, 
            rls_time_of_day == 0 ? "Daytime" : rls_time_of_day == 1 ? "Twilight" : "Darkness");
    status_serial.println(serial_debug_string);
  #endif
  #if IMMOBILIZER_SEQ
    sprintf(serial_debug_string, " ================================ Immobilizer ===============================\r\n"
            " Immobilizer: %s, Persistent setting: %s, Key detected: %s",
            immobilizer_released ? "OFF" : "Active", EEPROM.read(31) == 1 ? "Active" : "OFF",
            key_valid ? "YES" : "NO");
    status_serial.println(serial_debug_string);
  #endif
  sprintf(serial_debug_string, " ================================= MDrive =================================\r\n"
          " Active: %s", mdrive_status ? "YES" : "NO");
  status_serial.println(serial_debug_string);
  status_serial.print(" Settings: DSC-");
  if (mdrive_dsc[cas_key_number] == 3) {
    status_serial.print("Unchanged");
  } else if (mdrive_dsc[cas_key_number] == 7) {
    status_serial.print("OFF");
  } else if (mdrive_dsc[cas_key_number] == 0x13) {
    status_serial.print("DTC/MDM");
  } else if (mdrive_dsc[cas_key_number] == 0xB) {
    status_serial.print("ON");
  }
  status_serial.print("  POWER-");
  if (mdrive_power[cas_key_number] == 0) {
    status_serial.print("Unchanged");
  } else if (mdrive_power[cas_key_number] == 0x10) {
    status_serial.print("Normal");
  } else if (mdrive_power[cas_key_number] == 0x20) {
    status_serial.print("Sport");
  } else if (mdrive_power[cas_key_number] == 0x30) {
    status_serial.print("Sport+");
  }
  status_serial.print("  EDC-");
  if (mdrive_edc[cas_key_number] == 0x20) {
    status_serial.print("Unchanged");
  } else if (mdrive_edc[cas_key_number] == 0x21) {
    status_serial.print("Comfort");
  } else if (mdrive_edc[cas_key_number] == 0x22) {
    status_serial.print("Normal");
  } else if (mdrive_edc[cas_key_number] == 0x2A) {
    status_serial.print("Sport");
  }
  status_serial.print("  SVT-");
  if (mdrive_svt[cas_key_number] == 0xE9) {
    status_serial.print("Normal");
  } else if (mdrive_svt[cas_key_number] == 0xF1) {
    status_serial.print("Sport");
  } else if (mdrive_svt[cas_key_number] == 0xF2) {
    status_serial.print("Sport+");
  }
  status_serial.println();
  sprintf(serial_debug_string, " Console POWER: %s\r\n"
          #if !F_NBT
            " POWER CKM: %s\r\n"
          #endif
          " Key profile number: %d\r\n"
          " ============================== Body ===============================",
          console_power_mode ? "ON" : "OFF",
          #if !F_NBT
            dme_ckm[cas_key_number][0] == 0xF1 ? "Normal" : "Sport",
          #endif
          cas_key_number + 1);
  status_serial.println(serial_debug_string);
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
    sprintf(serial_debug_string, " VSW switch current input: %d", vsw_current_input);
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

  int8_t max_stored_temp = EEPROM.read(32);
  sprintf(serial_debug_string, " ================================= Debug ==================================\r\n"
          " CPU speed: %ld, MHz CPU temperature: %.2f °C, Max recorded: %.2f °C", 
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
  #if F_NBT
    sprintf(serial_debug_string, " KCAN errors: %ld, KCAN2 errors: %ld, PTCAN errors: %ld, DCAN errors: %ld\r\n"
            " ==========================================================================",
            kcan_error_counter, kcan2_error_counter, ptcan_error_counter, dcan_error_counter);
  #else
    sprintf(serial_debug_string, " KCAN errors: %ld, PTCAN errors: %ld, DCAN errors: %ld\r\n"
            " ==========================================================================",
            kcan_error_counter, ptcan_error_counter, dcan_error_counter);
  #endif
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
  #if F_NBT
    kcan2_write_msg(f_oil_level_measuring_buf);                                                                                     // Needed to reset the oil level display when switching ingition OFF
    f_oil_level_timer = 0;
    f_oil_level_measuring_timer = 8000;
  #endif
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
  vsw_current_input = 0;
  vsw_switch_counter = 0xF1;
  asd_initialized = false;
  comfort_exit_done = false;
  full_indicator = false;
  f_pdc_request = 1;
  #if IMMOBILIZER_SEQ
    if (immobilizer_persist) {
      if (immobilizer_released) {
        activate_immobilizer();
      }
    }
  #endif
  zbe_action_counter = zbe_rotation[2] = zbe_rotation[3] = 0;
  faceplate_volume = 0;
  gong_active = false;
  faceplate_buttons_txq.flush();
  nbt_cc_txq.flush();
  hazards_flash_txq.flush();
  #if F_NBT
    FACEPLATE_UART.end();                                                                                                           // Close serial connection.
  #endif
  driving_mode = 0;
  update_data_in_eeprom();
  kcan_retry_counter = ptcan_retry_counter = dcan_retry_counter = 0;
  kcan_resend_txq.flush();
  ptcan_resend_txq.flush();
  dcan_resend_txq.flush();
}
