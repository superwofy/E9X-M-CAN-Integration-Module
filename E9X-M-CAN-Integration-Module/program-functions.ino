// Functions related to the operation of the program and debugging go here.


void read_initialize_eeprom(void) {
  uint16_t stored_eeprom_checksum = EEPROM.read(0) << 8 | EEPROM.read(1);
  uint16_t calculated_eeprom_checksum = calculate_eeprom_crc();

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
    EEPROM.update(29, 0);                                                                                                           // Un-fold exterior mirrors when unlocking
    EEPROM.update(30, 1);                                                                                                           // Immobilizer
    EEPROM.update(31, 1);
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
    EEPROM.update(46, 9);                                                                                                           // Pressure units and date format.
    EEPROM.update(47, 9);
    EEPROM.update(48, 9);
    EEPROM.update(49, 4);                                                                                                           // Loglevel
    EEPROM.update(50, 5);                                                                                                           // IHKA auto states
    EEPROM.update(51, 3);
    EEPROM.update(52, 0);
    EEPROM.update(53, 0);

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
    #if !F_NBTE                                                                                                                      // NBT does not have M-Key settings.
      dme_ckm[0][0] = EEPROM.read(20);
      dme_ckm[1][0] = EEPROM.read(21);
      dme_ckm[2][0] = EEPROM.read(22);
    #else
      torque_unit[0] = EEPROM.read(23);
      power_unit[0] = EEPROM.read(24);
      torque_unit[1] = EEPROM.read(25);
      power_unit[1] = EEPROM.read(26);
      torque_unit[2] = EEPROM.read(27);
      power_unit[2] = EEPROM.read(28);
      pressure_unit_date_format[0] = EEPROM.read(46);
      pressure_unit_date_format[1] = EEPROM.read(47);
      pressure_unit_date_format[2] = EEPROM.read(48);
    #endif
    unfold_with_door_open = EEPROM.read(29) == 1 ? true : false;
    immobilizer_released = EEPROM.read(30) == 1 ? true : false;
    immobilizer_persist = EEPROM.read(31) == 1 ? true : false;
    if (!immobilizer_persist) {
      immobilizer_released = true;                                                                                                  // Deactivate immobilizer at boot if this value is set.
    }
    #if DOOR_VOLUME && !F_NBTE
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
    auto_seat_ckm[0] = EEPROM.read(37) == 1 ? true : false;
    auto_seat_ckm[1] = EEPROM.read(38) == 1 ? true : false;
    auto_seat_ckm[2] = EEPROM.read(39) == 1 ? true : false;
    drl_ckm[0] = EEPROM.read(40) == 1 ? true : false;
    drl_ckm[1] = EEPROM.read(41) == 1 ? true : false;
    drl_ckm[2] = EEPROM.read(42) == 1 ? true : false;
    serial_commands_unlocked = EEPROM.read(43) == 1 ? true : false;
    doors_locked = EEPROM.read(44) == 1 ? true : false;
    doors_alarmed = EEPROM.read(45) == 1 ? true : false;
    LOGLEVEL = constrain(EEPROM.read(49), 0, 4);
    ihka_auto_fan_speed = EEPROM.read(50);
    ihka_auto_fan_state = EEPROM.read(51);
    ihka_recirc_state = EEPROM.read(52);
    ihka_auto_distr_state = EEPROM.read(53);
    serial_log("Loaded data from EEPROM.", 2);
  }
}


uint16_t calculate_eeprom_crc(void) {
  teensy_eeprom_crc.restart();
  for (uint8_t i = 2; i < 54; i++) {
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
  #if !F_NBTE
    EEPROM.update(20, dme_ckm[0][0]);
    EEPROM.update(21, dme_ckm[1][0]);
    EEPROM.update(22, dme_ckm[2][0]);
  #else
    EEPROM.update(23, torque_unit[0]);
    EEPROM.update(24, power_unit[0]);
    EEPROM.update(25, torque_unit[1]);
    EEPROM.update(26, power_unit[1]);
    EEPROM.update(27, torque_unit[2]);
    EEPROM.update(28, power_unit[2]);
    EEPROM.update(46, pressure_unit_date_format[0]);
    EEPROM.update(47, pressure_unit_date_format[1]);
    EEPROM.update(48, pressure_unit_date_format[2]);
  #endif
  EEPROM.update(29, unfold_with_door_open);
  #if DOOR_VOLUME && !F_NBTE
    EEPROM.update(33, peristent_volume);
  #endif
  EEPROM.update(34, visual_signal_ckm[0]);
  EEPROM.update(35, visual_signal_ckm[1]);
  EEPROM.update(36, visual_signal_ckm[2]);
  EEPROM.update(37, auto_seat_ckm[0]);
  EEPROM.update(38, auto_seat_ckm[1]);
  EEPROM.update(39, auto_seat_ckm[2]);
  EEPROM.update(40, drl_ckm[0]);
  EEPROM.update(41, drl_ckm[1]);
  EEPROM.update(42, drl_ckm[2]);
  EEPROM.update(44, doors_locked);
  EEPROM.update(45, doors_alarmed);
  EEPROM.update(49, LOGLEVEL);
  EEPROM.update(50, ihka_auto_fan_speed);
  EEPROM.update(51, ihka_auto_fan_state);
  EEPROM.update(52, ihka_recirc_state);
  EEPROM.update(53, ihka_auto_distr_state);
  update_eeprom_checksum();
}


void update_eeprom_checksum(void) {
  uint16_t new_eeprom_checksum = calculate_eeprom_crc();
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
          " Engine: %s, RPM: %d, Torque: %.1f Nm, idling: %s, EML: %s\r\n"
          " Coolant temperature: %d °C, Oil temperature %d °C\r\n"
          " Indicated speed: %.1f %s, Real speed: %.1f KPH",
          vehicle_awake ? "ON" : "OFF", terminal_r ? "ON" : "OFF", ignition ? "ON" : "OFF",
          engine_running == 2 ? "ON" : "OFF", RPM / 4, engine_torque, engine_idling ? "YES" : "NO",
          eml_light ? "ON" : "OFF",
          engine_coolant_temperature - 48, engine_oil_temperature - 48,
          indicated_speed, speed_mph ? "MPH" : "KPH", real_speed);
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
      status_serial.print(" DSC: Fully ON");
    } else if (dsc_program_status == 1) {
      status_serial.print(" DSC: Fully OFF");
    } else if (dsc_program_status == 4) {
      status_serial.print(" DSC: DTC/MDM mode");
    } else {
      status_serial.print(" DSC: Unknown/Error");
    }
    sprintf(serial_debug_string, ", Intervention: %d", dsc_intervention);
    status_serial.println(serial_debug_string);
  } else {
    status_serial.println(" DSC: Asleep");
  }
  sprintf(serial_debug_string, " Clutch: %s\r\n Car is: %s",
          clutch_pressed ? "Depressed" : "Released",
          vehicle_moving ? "Moving" : "Stationary");
  status_serial.println(serial_debug_string);
  
  if (e_vehicle_direction == 0) {
    status_serial.println(" Direction: None");
  } else if (e_vehicle_direction == 1) {
    status_serial.println(" Direction: Forward");
  } else if (e_vehicle_direction == 2) {
    status_serial.println(" Direction: Backward");
  } else {
    status_serial.println(" Direction: Unknown");
  }
   
  if (ignition) {
    #if F_NIVI
    sprintf(serial_debug_string, " Sine pitch angle: %.1f, FXX-Converted: %.1f degrees\r\n"
            " Sine roll angle: %.1f, FXX-Converted: %.1f degrees\r\n",
            sine_pitch_angle, (f_vehicle_pitch_angle - 0x2000) * 0.05 - 64.0,
            sine_roll_angle, (f_vehicle_roll_angle - 0x2000) * 0.05 - 64.0);
    #endif
    #if F_NBTE
      sprintf(serial_debug_string, " Yaw rate FXX-Converted: %.2f degrees/s\r\n",
          f_yaw_rate * 0.005 - 163.84);
    #endif
    status_serial.println(serial_debug_string);
  }
  #if F_NIVI || F_NBTE
    sprintf(serial_debug_string, " Longitudinal acceleration: %.2f g, FXX-Converted: %.2f m/s^2\r\n"
            " Lateral acceleration: %.2f g, FXX-Converted: %.2f m/s^2",          
            e_longitudinal_acceleration * 0.10197162129779283, longitudinal_acceleration * 0.002 - 65.0,
            e_lateral_acceleration * 0.10197162129779283, lateral_acceleration * 0.002 - 65.0);
    status_serial.println(serial_debug_string);
  #endif
  #if F_NBTE || F_NIVI || MIRROR_UNDIM
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
          #if !F_NBTE
            " POWER CKM: %s\r\n"
          #endif
          " Key profile number: %d\r\n"
          " ============================== Body ===============================",
          console_power_mode ? "ON" : "OFF",
          #if !F_NBTE
            dme_ckm[cas_key_number][0] == 0xF1 ? "Normal" : "Sport",
          #endif
          cas_key_number + 1);
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Reverse gear: %s", reverse_gear_status ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " Handbrake: %s", handbrake_status ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  sprintf(serial_debug_string, " PDC: %s", pdc_bus_status > 0x80 ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
  if (ambient_temperature_real != -255.0) {
    sprintf(serial_debug_string, " Ambient temp: %.1f °C", ambient_temperature_real);
  } else {
    sprintf(serial_debug_string, " Ambient temp: Unknown");
  }
  status_serial.println(serial_debug_string);
  if (ambient_temperature_real != -255.0) {
    sprintf(serial_debug_string, " Interior temp: %.1f °C", interior_temperature);
  } else {
    sprintf(serial_debug_string, " Interior temp: Unknown");
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
    sprintf(serial_debug_string, " Left door: %s, Right door: %s", 
            left_door_open ? "Open" : "Closed", right_door_open ? "Open" : "Closed");
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
    sprintf(serial_debug_string, " AHL: %s", ahl_active ? "ON" : "OFF");
    status_serial.println(serial_debug_string);
  #endif
  sprintf(serial_debug_string, " HBA: %s", hba_status == 0xE ? "ON" : "OFF");
  status_serial.println(serial_debug_string);
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

  sprintf(serial_debug_string, " ================================= Debug ==================================\r\n"
          " CPU speed: %ld, MHz CPU temperature: %.2f °C, Max: %.2f °C", 
          F_CPU_ACTUAL / 1000000, cpu_temp, max_cpu_temp);
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
  if (max_loop_timer >= 1000) {
    sprintf(serial_debug_string, " Max loop execution time: %.2f ms", max_loop_timer / 1000.0);
  } else {
    sprintf(serial_debug_string, " Max loop execution time: %ld μs", max_loop_timer);
  }
  status_serial.println(serial_debug_string);
  #if F_NBTE
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
}


void serial_log(const char message[], uint8_t level) {
  #if DEBUG_MODE
    if (millis() <= 10000) {                                                                                                        // Store early messages that won't be on the Serial monitor.
      char buffer[128];
      if (micros() < 1000) {
        snprintf(buffer, sizeof(buffer), " %ld μs: %s\r\n", micros(), message);
      } else if (millis() < 100) {
        snprintf(buffer, sizeof(buffer), " %.2f ms: %s\r\n", micros() / 1000.0, message);
      } else {
        snprintf(buffer, sizeof(buffer), " %ld ms: %s\r\n", millis(), message);
      }
      if (strlen(boot_debug_string) + strlen(buffer) < sizeof(boot_debug_string)) {
        strcat(boot_debug_string, buffer);
      }
    }
    if (!clearing_dtcs) {
      if (level <= LOGLEVEL) {
        if (Serial.dtr()) {
          Serial.println(message);
        }
      }
    }
  #endif
}


void reset_ignition_variables(void) {                                                                                               // Ignition OFF. Set variables to pre-ignition state.
  dsc_program_status = dsc_intervention = 0;
  if (mdrive_status) {
    toggle_mdrive_message_active();
  }
  RPM = 0;
  eml_light = 0;
  ignore_m_press = ignore_m_hold = false;
  mdrive_power_active = restore_console_power_mode = false;
  mdrive_settings_requested = false;
  m_mfl_held_count = 0;
  uif_read = false;
  console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;                                                           // When cycling ignition, restore this to its CKM value.
  edc_mismatch_check_counter = 0;
  dsc_txq.flush();
  dsc_mode_change_disable = false;
  f_driving_dynamics_ignore = 3;
  seat_heating_dr_txq.flush();
  seat_heating_pas_txq.flush();
  reverse_beep_sent = pdc_too_close = false;
  reverse_beep_resend_timer = 0;
  pdc_tone_on = false;
  ihk_extra_buttons_cc_txq.flush();
  hdc_txq.flush();
  pdc_buttons_txq.flush();
  exhaust_flap_sport = false;
  fzd_ec_dimming = 0;
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
    if (steering_heater_transistor_active) {
      digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);
      steering_heater_transistor_active = false;
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
  pdc_bus_status = 0x80;
  rvc_tow_view_by_module = rvc_tow_view_by_driver = false;
  dme_boost_requested = false;
  #if IMMOBILIZER_SEQ
    reset_key_cc();
  #endif
  reverse_gear_status = false;
  sine_pitch_angle_requested = sine_roll_angle_requested = false;
  trsvc_cc_gong = false;
  #if F_NBTE
    send_f_pdc_function_status(true);                                                                                               // If PDC was active, update NBT with the OFF status.
    #if F_VSW01 && F_VSW01_MANUAL
      vsw_switch_input(4);
    #endif
  #endif
  nbt_network_management_next_neighbour = 0x6D;                                                                                     // Driver's seat module.
  nbt_network_management_timer = 3000;
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
  wash_message_counter = wiper_stalk_down_message_counter = wiper_stalk_down_last_press_time = 0;
  indicator_stalk_pushed_message_counter = 0;
  wipe_scheduled = intermittent_wipe_active = false, auto_wipe_active = false;
  frm_mirror_status_requested = false;
  frm_ahl_flc_status_requested = false;
  fold_lock_button_pressed = false;
  lock_button_pressed = false;
  lock_button_pressed_counter = 0;
  mirror_status_retry = 0;
  mirror_fold_txq.flush();
  last_lock_status_can = 0;
  car_locked_indicator_counter = 0;
  vsw_current_input = 0;
  vsw_switch_counter = 0xF1;
  asd_initialized = asd_rad_on_initialized = false;
  szl_full_indicator = false;
  f_pdc_request = 1;
  svt70_sport_plus = false;
  #if IMMOBILIZER_SEQ
    if (immobilizer_persist) {
      if (immobilizer_released) {
        activate_immobilizer();
      }
    }
  #endif
  alarm_led_message_timer = 100000;
  zbe_action_counter = zbe_rotation[2] = zbe_rotation[3] = 0;
  faceplate_volume = 0;
  gong_active = false;
  faceplate_eject_pressed = faceplate_power_mute_pressed = faceplate_hu_reboot = false;
  requested_hu_off_t1 = requested_hu_off_t2 = nbt_network_management_initialized = false;
  nbt_bus_sleep = nbt_active_after_terminal_r = false;
  nbt_network_management_next_neighbour = 0x6D;                                                                                     // Driver's seat module.
  comfort_exit_ready = false;                                                                                                       // Do not execute comfort exit if car fell asleep.
  faceplate_buttons_txq.flush();
  radon_txq.flush();
  nbt_cc_txq.flush();
  hazards_flash_txq.flush();
  #if F_NBTE
    FACEPLATE_UART.end();                                                                                                           // Close serial connection.
  #endif
  faceplate_reset_counter = 0;
  idrive_run_timer = 0;
  driving_mode = 0;
  update_data_in_eeprom();
  kcan_retry_counter = ptcan_retry_counter = dcan_retry_counter = 0;
  kcan_resend_txq.flush();
  ptcan_resend_txq.flush();
  dcan_resend_txq.flush();
  diag_transmit = diag_timeout_active = true;
  nbt_network_management_timer = 3000;
  low_battery_cc_active = false;
  frm_consumer_shutdown = true;                                                                                                     // When going to sleep the FRM may not set this if the car wasn't awake long enough.
}
