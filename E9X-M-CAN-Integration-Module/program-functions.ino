// Functions related to the operation of the program and debugging go here.


void initialize_timers() {
  #if DEBUG_MODE
    power_button_debounce_timer = dsc_off_button_debounce_timer = mdrive_message_timer = vehicle_awake_timer 
      = debug_print_timer = loop_timer = millis();
  #else
    power_button_debounce_timer = dsc_off_button_debounce_timer = mdrive_message_timer = vehicle_awake_timer = millis();
  #endif
  #if FRONT_FOG_CORNER
    corner_timer = vehicle_awake_timer;
  #endif
}


void initialize_watchdog() {
  WDT_timings_t config;
  #if DEBUG_MODE
    config.trigger = 15;
    config.callback = wdt_callback;
    config.timeout = 20;
  #else
    config.timeout = 10;                                                                                                            // If the watchdog timer is not reset within 10s, re-start the program.
  #endif
  wdt.begin(config);
}


#if DEBUG_MODE
void wdt_callback() {
  serial_log("Watchdog not fed. Program will reset in 5s.");
}
#endif


#if DEBUG_MODE && CDC2_STATUS_INTERFACE == 2                                                                                        // Check if Dual Serial is set
void print_current_state() {
  SerialUSB1.println("=========== Operation ==========");
  sprintf(serial_debug_string, " Vehicle PTCAN: %s", vehicle_awake ? "active" : "standby");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Terminal R: %s", terminal_r ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Ignition: %s", ignition ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Engine: %s", engine_running ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " RPM: %d", RPM / 4);
  SerialUSB1.println(serial_debug_string);
  #if HDC
    sprintf(serial_debug_string, " Speed: %d", vehicle_speed);
    SerialUSB1.println(serial_debug_string);
    sprintf(serial_debug_string, " Cruise Control: %s", cruise_control_status ? "ON" : "OFF");
    SerialUSB1.println(serial_debug_string);
  #endif
  if (vehicle_awake) {
    sprintf(serial_debug_string, " Voltage: %.2f V", battery_voltage);
    SerialUSB1.println(serial_debug_string);
  } else {
    SerialUSB1.println(" Voltage: Unknown");
  }
  if (ignition) {
    if (dsc_program_status == 0) {
      SerialUSB1.println(" DSC: Fully ON");
    } else if (dsc_program_status == 1) {
      SerialUSB1.println(" DSC: DTC/MDM mode");
    } else {
      SerialUSB1.println(" DSC: Fully OFF");
    }
  } else {
    SerialUSB1.println(" DSC: Asleep");
  }
  #if LAUNCH_CONTROL_INDICATOR
    sprintf(serial_debug_string, " Clutch: %s", clutch_pressed ? "Pressed" : "Released");
    SerialUSB1.println(serial_debug_string);
    sprintf(serial_debug_string, " Car is: %s", vehicle_moving ? "Moving" : "Stationary");
    SerialUSB1.println(serial_debug_string);
  #endif

  SerialUSB1.println("============ MDrive ============");
  if (ignition) {
    sprintf(serial_debug_string, " Active: %s", mdrive_status ? "YES" : "NO");
    SerialUSB1.println(serial_debug_string);
  } else {
    SerialUSB1.println(" Inactive");
  }
  SerialUSB1.print(" Settings: DSC-");
  switch (mdrive_dsc) {
    case 3:
      SerialUSB1.print("Unchanged"); break;
    case 7:
      SerialUSB1.print("OFF"); break;
    case 0x13:
      SerialUSB1.print("DTC/MDM"); break;
    case 0xB:
      SerialUSB1.print("ON"); break;
  }
  SerialUSB1.print("  POWER-");
  switch (mdrive_power) {
    case 0:
      SerialUSB1.print("Unchanged"); break;
    case 0x10:
      SerialUSB1.print("Normal"); break;
    case 0x20:
      SerialUSB1.print("Sport"); break;
    case 0x30:
      SerialUSB1.print("Sport+"); break;
  }
  SerialUSB1.print("  EDC-");
  switch (mdrive_edc) {
    case 0x20:
      SerialUSB1.print("Unchanged"); break;
    case 0x21:
      SerialUSB1.print("Comfort"); break;
    case 0x22:
      SerialUSB1.print("Normal"); break;
    case 0x2A:
      SerialUSB1.print("Sport"); break;
  }
  SerialUSB1.print("  SVT-");
  switch (mdrive_svt) {
    case 0xE9:
      SerialUSB1.print("Normal"); break;
    case 0xF1:
      SerialUSB1.print("Sport"); break;
  }
  SerialUSB1.println();
  sprintf(serial_debug_string, " POWER CKM: %s", dme_ckm[cas_key_number][0] == 0xF1 ? "Normal" : "Sport");
  SerialUSB1.println(serial_debug_string);

  SerialUSB1.println("========= Body ==========");
  #if RTC
    time_t t = now();
    uint8_t rtc_hours = hour(t);
    uint8_t rtc_minutes = minute(t);
    uint8_t rtc_seconds = second(t);
    uint8_t rtc_day = day(t);
    uint8_t rtc_month = month(t);
    uint16_t rtc_year = year(t);
    sprintf(serial_debug_string, " RTC: %s%d:%s%d:%s%d %s%d/%s%d/%d", 
            rtc_hours > 9 ? "" : "0", rtc_hours, rtc_minutes > 9 ? "" : "0", rtc_minutes, 
            rtc_seconds > 9 ? "" : "0", rtc_seconds, 
            rtc_day > 9 ? "" : "0", rtc_day, rtc_month > 9 ? "" : "0", rtc_month, rtc_year);
    SerialUSB1.println(serial_debug_string);
  #endif
  #if AUTO_SEAT_HEATING
    if (ambient_temperature_can != 255) {
      sprintf(serial_debug_string, " Ambient temp: %.1f °C", (ambient_temperature_can - 80) / 2.0);
    } else {
      sprintf(serial_debug_string, " Ambient temp: Unknown");
    }
    SerialUSB1.println(serial_debug_string);
    #if REVERSE_BEEP
      sprintf(serial_debug_string, " Reverse gear: %s", pdc_beep_sent ? "ON" : "OFF");
      SerialUSB1.println(serial_debug_string);
    #endif
    sprintf(serial_debug_string, " Driver's seat heating: %s", driver_seat_heating_status ? "ON" : "OFF");
    SerialUSB1.println(serial_debug_string);
    #if AUTO_SEAT_HEATING_PASS
      sprintf(serial_debug_string, " Passenger's seat heating: %s", passenger_seat_heating_status ? "ON" : "OFF");
      SerialUSB1.println(serial_debug_string);
      sprintf(serial_debug_string, " Passenger's seat occupied: %s", (passenger_seat_status >= 8) ? "YES" : "NO");
      SerialUSB1.println(serial_debug_string);
      sprintf(serial_debug_string, " Passenger's seatbelt fastened: %s", passenger_seat_status & 1 ? "YES" : "NO");
      SerialUSB1.println(serial_debug_string);
    #endif
  #endif
  #if DOOR_VOLUME
    #if RHD
      sprintf(serial_debug_string, " Passenger's door: %s", left_door_open ? "Open" : "Closed");
    #else
      sprintf(serial_debug_string, " Driver's's door: %s", left_door_open ? "Open" : "Closed");
    #endif
    SerialUSB1.println(serial_debug_string);
    #if RHD
      sprintf(serial_debug_string, " Driver's door: %s", right_door_open ? "Open" : "Closed");
    #else
      sprintf(serial_debug_string, " Passenger's door: %s", right_door_open ? "Open" : "Closed");
    #endif
    SerialUSB1.println(serial_debug_string);
  #endif
  #if EXHAUST_FLAP_CONTROL
    sprintf(serial_debug_string, " Exhaust flap: %s", exhaust_flap_open ? "Open" : "Closed");
    SerialUSB1.println(serial_debug_string);
  #endif
  #if FRONT_FOG_LED_INDICATOR
    sprintf(serial_debug_string, " Front fogs: %s", front_fog_status ? "ON" : "OFF");
    SerialUSB1.println(serial_debug_string);
  #endif
   #if FRONT_FOG_CORNER
    sprintf(serial_debug_string, " Dipped beam: %s", dipped_beam_status ? "ON" : "OFF");
    SerialUSB1.println(serial_debug_string);
  #endif
  #if DIM_DRL
    sprintf(serial_debug_string, " DRL: %s", drl_status ? "ON" : "OFF");
    SerialUSB1.println(serial_debug_string);
  #endif
  #if FTM_INDICATOR
    if (ignition) {
      sprintf(serial_debug_string, " FTM indicator: %s", ftm_indicator_status ? "ON" : "OFF");
      SerialUSB1.println(serial_debug_string);
    } else {
      SerialUSB1.println(" FTM indicator: Inactive");
    }
  #endif
  sprintf(serial_debug_string, " DCAN fix for SVT: %s", diagnose_svt ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Forwarded requests from DCAN (>): %d", dcan_forwarded_count);
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Forwarded responses from PTCAN (<): %d", ptcan_forwarded_count);
  SerialUSB1.println(serial_debug_string);

  SerialUSB1.println("============ Debug =============");
  sprintf(serial_debug_string, " CPU temperature: %.2f °C", tempmonGetTemp());
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " CPU speed: %ld MHz", F_CPU_ACTUAL / 1000000);
  SerialUSB1.println(serial_debug_string);
  unsigned long loop_calc = micros() - loop_timer;
  if (loop_calc > max_loop_timer) {
    max_loop_timer = loop_calc;
  }
  if (max_loop_timer > 1000) {
    sprintf(serial_debug_string, " Max loop execution time: %ld mSeconds", max_loop_timer / 1000);
  } else {
    sprintf(serial_debug_string, " Max loop execution time: %ld μSeconds", max_loop_timer);
  }
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " KCAN errors: %ld PTCAN errors: %ld DCAN errors: %ld", kcan_error_counter, ptcan_error_counter, dcan_error_counter);
  SerialUSB1.println(serial_debug_string);
  SerialUSB1.println("================================");
  debug_print_timer = millis();
}
#endif


void serial_log(const char message[]) {
  #if DEBUG_MODE
    Serial.println(message);
  #endif
}


void reset_runtime_variables() {                                                                                                    // Ignition OFF. Set variables to pre-ignition state.
  dsc_program_status = 0;
  if (mdrive_status) {
    toggle_mdrive_message_active();
  }
  engine_running = false;
  RPM = 0;
  ignore_m_press = ignore_m_hold = false;
  mdrive_power_active = restore_console_power_mode = false;
  mfl_pressed_count = 0;
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
    kcan_cc_txq.flush();
  #endif
  #if SERVOTRONIC_SVT70
    diagnose_svt = false;
  #endif
  #if EXHAUST_FLAP_CONTROL
    exhaust_flap_sport = false;
    #if !QUIET_START
      actuate_exhaust_solenoid(LOW);
    #endif
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    lc_cc_active = mdm_with_lc = clutch_pressed = false;
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
  #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER
    front_fog_status = false;
  #endif
  #if FRONT_FOG_CORNER
    fog_corner_txq.flush();
    if (left_fog_on) {
      delayed_can_tx_msg m = {front_left_fog_off_buf, millis() + 100};
      fog_corner_txq.push(&m);
    }
    if (right_fog_on) {
      delayed_can_tx_msg m = {front_right_fog_off_buf, millis() + 100};
      fog_corner_txq.push(&m);
    }
    dipped_beam_status = left_fog_on = right_fog_on = false;
    frm_lamp_status_requested = false;
  #endif
  #if DIM_DRL
    if (left_dimmed) {                                                                                                              // Send OFF now to make sure DRLs don't stay ON.
      kcan_write_msg(left_drl_dim_off);
    }
    if (right_dimmed) {
      kcan_write_msg(right_drl_dim_off);
    }
    drl_status = left_dimmed = right_dimmed = false;
  #endif
  #if FTM_INDICATOR
    ftm_indicator_status = false;
  #endif
  #if HDC
    vehicle_speed = 0;
    cruise_control_status = hdc_button_pressed = hdc_requested = hdc_active = false;
  #endif
  #if HDC || LAUNCH_CONTROL_INDICATOR
    vehicle_moving = false;
  #endif
  #if FAKE_MSA
    msa_button_pressed = false;
    msa_fake_status_counter = 0;
  #endif
}


void reset_sleep_variables() {
  #if AUTO_SEAT_HEATING
    driver_sent_seat_heating_request = false;                                                                                       // Reset the seat heating request now that the car's asleep.
    driver_seat_heating_status = false;
    #if AUTO_SEAT_HEATING_PASS
      passenger_sent_seat_heating_request = false;
      passenger_seat_status = 0;
      passenger_seat_heating_status = false;
    #endif
  #endif
  #if CKM
    dme_ckm_sent = false;
    dme_ckm_tx_queue.flush();
  #endif
  #if DOOR_VOLUME
    volume_reduced = false;                                                                                                         // In case the car falls asleep with the door open.
    volume_requested = false;
    volume_restore_offset = 0;
    default_volume_sent = false;
    idrive_txq.flush();
  #endif
  update_settings_in_eeprom();
  #if AUTO_MIRROR_FOLD
    frm_status_requested = false;
    lock_button_pressed  = unlock_button_pressed = false;
    last_lock_status_can = 0;
  #endif
  mdrive_settings_updated = false;
}
