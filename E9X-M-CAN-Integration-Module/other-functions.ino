#if DEBUG_MODE
void debug_can_message(uint16_t canid, uint8_t len, uint8_t* message)
{   
    sprintf(serial_debug_string, "0x%.2X: ", canid);
    Serial.print(serial_debug_string);
    for (uint8_t i = 0; i<len; i++) {
      sprintf(serial_debug_string, " 0x%.2X", message[i]);
      Serial.print(serial_debug_string);
    }
    Serial.println();
}


void print_current_state()
{
  SerialUSB1.write(27);       // ESC command
  SerialUSB1.print("[2J");    // clear screen command
  SerialUSB1.write(27);
  SerialUSB1.print("[H");     // cursor to home command

  SerialUSB1.println("=========== Operation ==========");
  sprintf(serial_debug_string, " Vehicle PTCAN: %s", vehicle_awake ? "active" : "standby");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Ignition: %s", ignition ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Engine: %s", engine_running ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  if (engine_running) {
    sprintf(serial_debug_string, " RPM: %ld", RPM / 4);
    SerialUSB1.println(serial_debug_string);
  }
  if (vehicle_awake) {
    sprintf(serial_debug_string, " Voltage: %.2f V", battery_voltage);
    SerialUSB1.println(serial_debug_string);
  }
  if (ignition) {
    if (dsc_program_status == 0) {
      SerialUSB1.println(" DSC: Fully ON");
    } else if (dsc_program_status == 1) {
      SerialUSB1.println(" DSC: DTC/MDM mode");
    } else {
      SerialUSB1.println(" DSC: Fully OFF");
    }
  }
  #if LAUNCH_CONTROL_INDICATOR
    sprintf(serial_debug_string, " Clutch: %s", clutch_pressed ? "Pressed" : "Released");
    SerialUSB1.println(serial_debug_string);
    if (ignition) {
      sprintf(serial_debug_string, " Car is: %s", vehicle_moving ? "Moving" : "Stationary");
      SerialUSB1.println(serial_debug_string);
    }
  #endif

  SerialUSB1.println("============ MDrive ============");
  if (ignition) {
    sprintf(serial_debug_string, " Active: %s", mdrive_status ? "YES" : "NO");
    SerialUSB1.println(serial_debug_string);
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
  SerialUSB1.print(" POWER-");
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
  SerialUSB1.print(" EDC-");
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
  SerialUSB1.print(" SVT-");
  switch (mdrive_svt) {
    case 0xE9:
      SerialUSB1.print("Normal"); break;
    case 0xF1:
      SerialUSB1.print("Sport"); break;
  }
  SerialUSB1.println();

  SerialUSB1.println("========= Body ==========");
  #if RTC
    time_t t = now();
    uint8_t rtc_hours = hour(t);
    uint8_t rtc_minutes = minute(t);
    uint8_t rtc_seconds = second(t);
    uint8_t rtc_day = day(t);
    uint8_t rtc_month = month(t);
    uint16_t rtc_year = year(t);
    sprintf(serial_debug_string, " RTC: %d:%d:%d %d/%d/%d", rtc_hours, rtc_minutes, rtc_seconds, rtc_day, rtc_month, rtc_year);
    SerialUSB1.println(serial_debug_string);
  #endif
  #if AUTO_SEAT_HEATING
    if (ambient_temperature_can != 255) {
      sprintf(serial_debug_string, " Ambient temp: %d °C", (ambient_temperature_can - 80) / 2);
    } else {
      sprintf(serial_debug_string, " Ambient temp: Unknown");
    }
    SerialUSB1.println(serial_debug_string);
    #if REVERSE_BEEP
      if (ignition) {
        sprintf(serial_debug_string, " Reverse gear: %s", pdc_beep_sent ? "ON" : "OFF");
        SerialUSB1.println(serial_debug_string);
      }
    #endif
    if (ignition) {
      sprintf(serial_debug_string, " Driver's seat heating: %s", driver_seat_heating_status ? "ON" : "OFF");
      SerialUSB1.println(serial_debug_string);
      sprintf(serial_debug_string, " Passenger's seat heating: %s", passenger_seat_heating_status ? "ON" : "OFF");
      SerialUSB1.println(serial_debug_string);
    }
    sprintf(serial_debug_string, " Passenger's seat occupied: %s", (passenger_seat_status >= 8) ? "YES" : "NO");
    SerialUSB1.println(serial_debug_string);
    sprintf(serial_debug_string, " Passenger's seatbelt fastened: %s", passenger_seat_status & 1 ? "YES" : "NO");
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
  #if FTM_INDICATOR
    if (ignition) {
      sprintf(serial_debug_string, " FTM indicator: %s", ftm_indicator_status ? "ON" : "OFF");
      SerialUSB1.println(serial_debug_string);
    }
  #endif
  sprintf(serial_debug_string, " DCAN fix for SVT: %s", diagnose_svt ? "ON" : "OFF");
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Forwarded requests from DCAN (>): %lu", dcan_forwarded_count);
  SerialUSB1.println(serial_debug_string);
  sprintf(serial_debug_string, " Forwarded responses from PTCAN (<): %lu", ptcan_forwarded_count);
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
  SerialUSB1.println("================================");
  debug_print_timer = millis();
}
#endif  


void reset_runtime_variables()                                                                                                      // Ignition off. Set variables to original state and commit MDrive settings.
{
  dsc_program_status = 0;
  if (mdrive_status) {
    toggle_mdrive_message_active();
  }
  engine_running = false;
  RPM = 0;
  ignore_m_press = false;
  mdrive_power_active = restore_console_power_mode = false;
  sending_dsc_off = send_dsc_off_from_mdm = false;
  sending_dsc_off_counter = 0;
  dtcTx.flush();                                                                                                                    // Empty these queues in case something was left over.
  dscTx.flush();
  #if AUTO_SEAT_HEATING
    seatHeatingTx.flush();
  #endif
  #if REVERSE_BEEP
    pdcBeepTx.flush();
    pdc_beep_sent = false;
  #endif
  #if SERVOTRONIC_SVT70
    digitalWrite(DCAN_STBY_PIN, HIGH);
    diagnose_svt = false;
  #endif
  #if EXHAUST_FLAP_CONTROL
    exhaust_flap_sport = false;
    #if QUIET_START                                                                                                                // Keep the flap closed until the next start.
      actuate_exhaust_solenoid(HIGH);                                                                                              // Run this write in case the engine was shut down in sport mode.
    #else
      actuate_exhaust_solenoid(LOW);
    #endif
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    lc_cc_active = mdm_with_lc = clutch_pressed = vehicle_moving = false;
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
  #if FTM_INDICATOR
    ftm_indicator_status = false;
  #endif
  update_settings_in_eeprom();
  mdrive_settings_updated = false;
}


void cache_can_message_buffers()                                                                                                    // Put all static the buffers in memory during setup().
{
  dtc_button_pressed_buf = makeMsgBuf(0x316, 2, dtc_button_pressed);
  dtc_button_released_buf = makeMsgBuf(0x316, 2, dtc_button_released);
  dsc_off_fake_cc_status_buf = makeMsgBuf(0x5A9, 8, dsc_off_fake_cc_status);
  mdm_fake_cc_status_buf = makeMsgBuf(0x5A9, 8, mdm_fake_cc_status);
  mdm_fake_cc_status_off_buf = makeMsgBuf(0x5A9, 8, mdm_fake_cc_status_off);
  #if SERVOTRONIC_SVT70
    servotronic_cc_on_buf = makeMsgBuf(0x58E, 8, servotronic_cc_on);
  #endif
  #if FTM_INDICATOR
    ftm_indicator_flash_buf = makeMsgBuf(0x5A0, 8, ftm_indicator_flash);
    ftm_indicator_off_buf = makeMsgBuf(0x5A0, 8, ftm_indicator_off);
  #endif
  #if REVERSE_BEEP
    pdc_beep_buf = makeMsgBuf(0x1C6, 4, pdc_beep);
    pdc_quiet_buf = makeMsgBuf(0x1C6, 4, pdc_quiet);
  #endif
  #if F_ZBE_WAKE
    f_wakeup_buf = makeMsgBuf(0x560, 8, f_wakeup);
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    lc_cc_on_buf = makeMsgBuf(0x598, 8, lc_cc_on);
    lc_cc_off_buf = makeMsgBuf(0x598, 8, lc_cc_off);
  #endif
  #if CONTROL_SHIFTLIGHTS
    shiftlights_start_buf = makeMsgBuf(0x206, 2, shiftlights_start);
    shiftlights_mid_buildup_buf = makeMsgBuf(0x206, 2, shiftlights_mid_buildup);
    shiftlights_startup_buildup_buf = makeMsgBuf(0x206, 2, shiftlights_startup_buildup);
    shiftlights_max_flash_buf = makeMsgBuf(0x206, 2, shiftlights_max_flash);
    shiftlights_off_buf = makeMsgBuf(0x206, 2, shiftlights_off);
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
