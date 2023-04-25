#if DEBUG_MODE && CDC2_STATUS_INTERFACE == 2                                                                                        // Check if Dual Serial is set
void print_current_state()
{
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
  sprintf(serial_debug_string, " POWER CKM: %s", dme_ckm[0] == 0xF1 ? "Normal" : "Sport");
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
    sprintf(serial_debug_string, " Passenger's seat heating: %s", passenger_seat_heating_status ? "ON" : "OFF");
    SerialUSB1.println(serial_debug_string);
    sprintf(serial_debug_string, " Passenger's seat occupied: %s", (passenger_seat_status >= 8) ? "YES" : "NO");
    SerialUSB1.println(serial_debug_string);
    sprintf(serial_debug_string, " Passenger's seatbelt fastened: %s", passenger_seat_status & 1 ? "YES" : "NO");
    SerialUSB1.println(serial_debug_string);
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
  #if FRONT_FOG_INDICATOR
    sprintf(serial_debug_string, " Front fogs: %s", front_fog_status ? "ON" : "OFF");
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


void reset_runtime_variables()                                                                                                      // Ignition off. Set variables to original state and commit MDrive settings.
{
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
    console_power_mode = dme_ckm[0] == 0xF1 ? false : true;                                                                         // When cycling ignition, restore this to its CKM value.
  #endif
  dsc_txq.flush();
  #if AUTO_SEAT_HEATING
    seat_heating_dr_txq.flush();
    seat_heating_pas_txq.flush();
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
  #if FRONT_FOG_INDICATOR
    front_fog_status = false;
    last_light_status = 0;
    digitalWrite(FOG_LED_PIN, LOW);
  #endif
  #if DIM_DRL
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
  update_settings_in_eeprom();
  mdrive_settings_updated = false;
  if (deactivate_ptcan_temporariliy) {
    temp_reactivate_ptcan();
  }
}


void reset_sleep_variables()
{
  #if AUTO_SEAT_HEATING
    driver_sent_seat_heating_request = false;                                                                                       // Reset the seat heating request now that the car's asleep.
    passenger_sent_seat_heating_request = false;
    passenger_seat_status = 0;
    driver_seat_heating_status = false;
    passenger_seat_heating_status = false;
  #endif
  #if DOOR_VOLUME
    volume_reduced = false;                                                                                                         // In case the car falls asleep with the door open.
    volume_requested = false;
    volume_restore_offset = 0;
    default_volume_sent = false;
  #endif
}


void cache_can_message_buffers()                                                                                                    // Put all static the buffers in memory during setup().
{
  uint8_t dsc_on[] = {0xCF, 0xE3}, dsc_mdm_dtc[] = {0xCF, 0xF3}, dsc_off[] = {0xCF, 0xE7};
  dsc_on_buf = makeMsgBuf(0x398, 2, dsc_on);
  dsc_mdm_dtc_buf = makeMsgBuf(0x398, 2, dsc_mdm_dtc);
  dsc_off_buf = makeMsgBuf(0x398, 2, dsc_off);
  uint8_t idrive_mdrive_settings_a[] = {0x63, 0x10, 0xA, 0x31, 0x52, 0, 0, 6};
  uint8_t idrive_mdrive_settings_b[] = {0x63, 0x21, 0x5C, 0, 0, 0, 0, 0};
  idrive_mdrive_settings_a_buf = makeMsgBuf(0x6F1, 8, idrive_mdrive_settings_a);
  idrive_mdrive_settings_b_buf = makeMsgBuf(0x6F1, 8, idrive_mdrive_settings_b);
  uint8_t cc_gong[] = {0x60, 3, 0x31, 0x22, 2, 0, 0, 0};
  cc_gong_buf = makeMsgBuf(0x6F1, 8, cc_gong);
  #if SERVOTRONIC_SVT70
    uint8_t servotronic_cc_on[] = {0x40, 0x46, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
    servotronic_cc_on_buf = makeMsgBuf(0x58E, 8, servotronic_cc_on);
  #endif
  #if FTM_INDICATOR
    uint8_t ftm_indicator_flash[] = {0x40, 0x50, 1, 0x69, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t ftm_indicator_off[] = {0x40, 0x50, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF};
    ftm_indicator_flash_buf = makeMsgBuf(0x5A0, 8, ftm_indicator_flash);
    ftm_indicator_off_buf = makeMsgBuf(0x5A0, 8, ftm_indicator_off);
  #endif
  #if REVERSE_BEEP
    #if RHD 
      uint8_t pdc_beep[] = {0, 0, 0, 1};                                                                                            // Front right beep.
    #else
      uint8_t pdc_beep[] = {0, 0, 1, 0};                                                                                            // Front left beep.
    #endif
    uint8_t pdc_quiet[] = {0, 0, 0, 0};
    pdc_beep_buf = makeMsgBuf(0x1C6, 4, pdc_beep);
    pdc_quiet_buf = makeMsgBuf(0x1C6, 4, pdc_quiet);
  #endif
  #if DIM_DRL
    uint8_t left_drl_off[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0};
    uint8_t left_drl_dim[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x16};
    uint8_t left_drl_bright[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x64};
    uint8_t right_drl_off[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0};
    uint8_t right_drl_dim[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0x16};
    uint8_t right_drl_bright[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0x64};
    left_drl_dim_off = makeMsgBuf(0x6F1, 8, left_drl_off);
    left_drl_dim_buf = makeMsgBuf(0x6F1, 8, left_drl_dim);
    left_drl_bright_buf = makeMsgBuf(0x6F1, 8, left_drl_bright);
    right_drl_dim_off = makeMsgBuf(0x6F1, 8, right_drl_off);
    right_drl_dim_buf = makeMsgBuf(0x6F1, 8, right_drl_dim);
    right_drl_bright_buf = makeMsgBuf(0x6F1, 8, right_drl_bright);
  #endif
  #if F_ZBE_WAKE
    uint8_t f_wakeup[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                                         // Network management KOMBI - F-series.
    f_wakeup_buf = makeMsgBuf(0x560, 8, f_wakeup);
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    uint8_t lc_cc_on[] = {0x40, 0xBE, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t lc_cc_off[] = {0x40, 0xBE, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    lc_cc_on_buf = makeMsgBuf(0x598, 8, lc_cc_on);
    lc_cc_off_buf = makeMsgBuf(0x598, 8, lc_cc_off);
  #endif
  #if AUTO_SEAT_HEATING
    uint8_t seat_heating_button_pressed[] = {0xFD, 0xFF};
    uint8_t seat_heating_button_released[] = {0xFC, 0xFF};
    seat_heating_button_pressed_dr_buf = makeMsgBuf(0x1E7, 2, seat_heating_button_pressed);
    seat_heating_button_released_dr_buf = makeMsgBuf(0x1E7, 2, seat_heating_button_released);
    seat_heating_button_pressed_pas_buf = makeMsgBuf(0x1E8, 2, seat_heating_button_pressed);
    seat_heating_button_released_pas_buf = makeMsgBuf(0x1E8, 2, seat_heating_button_released);
  #endif
  #if RTC
    uint8_t set_time_cc[] = {0x40, 0xA7, 0, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    set_time_cc_buf = makeMsgBuf(0x5E0, 8, set_time_cc);;
  #endif
  #if CONTROL_SHIFTLIGHTS
    uint8_t shiftlights_start[] = {0x86, 0x3E};
    uint8_t shiftlights_mid_buildup[] = {0xF6, 0};
    uint8_t shiftlights_startup_buildup[] = {0x56, 0};                                                                              // Faster sequential buildup. First byte 0-0xF (0xF - slowest).
    uint8_t shiftlights_max_flash[] = {0xA, 0};
    uint8_t shiftlights_off[] = {5, 0};
    shiftlights_start_buf = makeMsgBuf(0x206, 2, shiftlights_start);
    shiftlights_mid_buildup_buf = makeMsgBuf(0x206, 2, shiftlights_mid_buildup);
    shiftlights_startup_buildup_buf = makeMsgBuf(0x206, 2, shiftlights_startup_buildup);
    shiftlights_max_flash_buf = makeMsgBuf(0x206, 2, shiftlights_max_flash);
    shiftlights_off_buf = makeMsgBuf(0x206, 2, shiftlights_off);
  #endif
  #if NEEDLE_SWEEP
    uint8_t speedo_needle_sweep[] = {0x60, 5, 0x30, 0x20, 6, 0x12, 0x11, 0};                                                        // Set to 325 KM/h
    uint8_t speedo_needle_release[] = {0x60, 3, 0x30, 0x20, 0, 0, 0, 0};
    uint8_t tacho_needle_sweep[] = {0x60, 5, 0x30, 0x21, 6, 0x12, 0x3D, 0};                                                         // Set to 8000 RPM
    uint8_t tacho_needle_release[] = {0x60, 3, 0x30, 0x21, 0, 0, 0, 0};
    uint8_t fuel_needle_sweep[] = {0x60, 5, 0x30, 0x22, 6, 7, 0x4E, 0};                                                             // Set to 100%
    uint8_t fuel_needle_release[] = {0x60, 3, 0x30, 0x22, 0, 0, 0, 0};
    uint8_t oil_needle_sweep[] = {0x60, 5, 0x30, 0x23, 6, 7, 0x12, 0};                                                              // Set to 150 C
    uint8_t oil_needle_release[] = {0x60, 3, 0x30, 0x23, 0, 0, 0, 0};
    speedo_needle_sweep_buf = makeMsgBuf(0x6F1, 8, speedo_needle_sweep);
    speedo_needle_release_buf = makeMsgBuf(0x6F1, 8, speedo_needle_release);
    tacho_needle_sweep_buf = makeMsgBuf(0x6F1, 8, tacho_needle_sweep);
    tacho_needle_release_buf = makeMsgBuf(0x6F1, 8, tacho_needle_release);
    fuel_needle_sweep_buf = makeMsgBuf(0x6F1, 8, fuel_needle_sweep);
    fuel_needle_release_buf = makeMsgBuf(0x6F1, 8, fuel_needle_release);
    oil_needle_sweep_buf = makeMsgBuf(0x6F1, 8, oil_needle_sweep);
    oil_needle_release_buf = makeMsgBuf(0x6F1, 8, oil_needle_release);
  #endif
  #if DOOR_VOLUME
    uint8_t vol_request[] = {0x63, 3, 0x31, 0x24, 0, 0, 0, 0}; 
    uint8_t default_vol_set[] = {0x63, 2, 0x31, 0x25, 0, 0, 0, 0};
    vol_request_buf = makeMsgBuf(0x6F1, 8, vol_request);
    default_vol_set_buf = makeMsgBuf(0x6F1, 8, default_vol_set);
  #endif
  #if HDC
    uint8_t set_hdc_cruise_control[] = {0, 0xFB, 8, 0xFC};
    uint8_t cancel_hdc_cruise_control[] = {0, 4, 0x10, 0xFC};
    uint8_t hdc_cc_activated_on[] = {0x40, 0x4B, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_unavailable_on[] = {0x40, 0x4D, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_deactivated_on[] = {0x40, 0x4C, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_activated_off[] = {0x40, 0x4B, 1, 0, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_unavailable_off[] = {0x40, 0x4D, 1, 0, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_deactivated_off[] = {0x40, 0x4C, 1, 0, 0xFF, 0xFF, 0xFF, 0xAB};
    set_hdc_cruise_control_buf = makeMsgBuf(0x194, 4, set_hdc_cruise_control);
    cancel_hdc_cruise_control_buf = makeMsgBuf(0x194, 4, cancel_hdc_cruise_control);
    hdc_cc_activated_on_buf = makeMsgBuf(0x5A9, 8, hdc_cc_activated_on);
    hdc_cc_unavailable_on_buf = makeMsgBuf(0x5A9, 8, hdc_cc_unavailable_on);
    hdc_cc_deactivated_on_buf = makeMsgBuf(0x5A9, 8, hdc_cc_deactivated_on);
    hdc_cc_activated_off_buf = makeMsgBuf(0x5A9, 8, hdc_cc_activated_off);
    hdc_cc_unavailable_off_buf = makeMsgBuf(0x5A9, 8, hdc_cc_unavailable_off);
    hdc_cc_deactivated_off_buf = makeMsgBuf(0x5A9, 8, hdc_cc_deactivated_off);
  #endif
  #if FAKE_MSA
    uint8_t msa_deactivated_cc_on[] = {0x40, 0xC2, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t msa_deactivated_cc_off[] = {0x40, 0xC2, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t msa_fake_status[] = {0xFF, 0xFF};
    msa_deactivated_cc_on_buf = makeMsgBuf(0x592, 8, msa_deactivated_cc_on);
    msa_deactivated_cc_off_buf = makeMsgBuf(0x592, 8, msa_deactivated_cc_off);
    msa_fake_status_buf = makeMsgBuf(0x308, 2, msa_fake_status);
  #endif
}


CAN_message_t makeMsgBuf(uint16_t txID, uint8_t txLen, uint8_t* txBuf) 
{
  CAN_message_t tx_msg;
  tx_msg.id = txID;
  tx_msg.len = txLen;
  for (uint8_t i = 0; i < txLen; i++) {
      tx_msg.buf[i] = txBuf[i];
  }
  return tx_msg;
}


void kcan_write_msg(const CAN_message_t &msg) 
{
  if (msg.id == 0x6F1 && !diag_transmit) {
    return;
  }
  #if DEBUG_MODE
  uint8_t result;
  result = KCAN.write(msg);
  if (result != 1) {
    sprintf(serial_debug_string, "KCAN write failed for ID: %lx with error %d.", msg.id, result);
    serial_log(serial_debug_string);
    kcan_error_counter++;
  }
  #else
    KCAN.write(msg);
  #endif
}


void ptcan_write_msg(const CAN_message_t &msg) 
{
  #if DEBUG_MODE
  uint8_t result;
  result = PTCAN.write(msg);
  if (result != 1) {
    sprintf(serial_debug_string, "PTCAN write failed for ID: %lx with error %d.", msg.id, result);
    serial_log(serial_debug_string);
    ptcan_error_counter++;
  }
  #else
    PTCAN.write(msg);
  #endif
}


#if SERVOTRONIC_SVT70
void dcan_write_msg(const CAN_message_t &msg) 
{
  #if DEBUG_MODE
  uint8_t result;
  result = DCAN.write(msg);
  if (result != 1) {
    sprintf(serial_debug_string, "DCAN write failed for ID: %lx with error %d.", msg.id, result);
    serial_log(serial_debug_string);
    dcan_error_counter++;
  }
  #else
    DCAN.write(msg);
  #endif
}
#endif
