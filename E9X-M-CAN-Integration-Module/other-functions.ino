#if FRONT_FOG_INDICATOR
void evaluate_fog_status()
{
  if (ptrxBuf[0] != last_light_status) {
    if ((ptrxBuf[0] & 32) == 32) {                                                                                                  // Check the third bit of the first byte represented in binary for front fog status.
      front_fog_status = true;
      digitalWrite(FOG_LED_PIN, HIGH);
      #if DEBUG_MODE
        Serial.println(F("Front fogs on. Turned on FOG LED"));
      #endif
    } else {
      front_fog_status = false;
      digitalWrite(FOG_LED_PIN, LOW);
      #if DEBUG_MODE
        Serial.println(F("Front fogs off. Turned off FOG LED"));
      #endif
    }
    last_light_status = ptrxBuf[0];
  }
}
#endif


#if AUTO_SEAT_HEATING
void send_seat_heating_request()
{
  delay(10);
  KCAN.sendMsgBuf(0x1E7, 2, seat_heating_button_pressed);
  delay(20);
  KCAN.sendMsgBuf(0x1E7, 2, seat_heating_button_released);
  delay(20);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent dr seat heating request at ambient %dC, treshold %dC.\n", 
           (ambient_temperature_can - 80) / 2, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2);
    Serial.print(serial_debug_string);
  #endif
  sent_seat_heating_request = true;
}
#endif


#if F_ZBE_WAKE
void send_zbe_wakeup()
{
  if ((millis() - zbe_wakeup_last_sent) >= 1500) {
    KCAN.sendMsgBuf(0x560, 8, f_wakeup);
    zbe_wakeup_last_sent = millis();
    #if DEBUG_MODE
      Serial.println(F("Sent F-ZBE wake-up message."));
    #endif
  }
}
#endif


#if EXHAUST_FLAP_CONTROL
void evaluate_exhaust_flap_position()
{
  if (!exhaust_flap_sport) {                                                                                                        // Exhaust is in quiet mode.
    if ((millis() - exhaust_flap_action_timer) >= 1500) {                                                                           // Avoid vacuum drain, oscillation and apply startup delay.
      if (RPM >= EXHAUST_FLAP_QUIET_RPM) {                                                                                          // Open at defined rpm setpoint.
        if (!exhaust_flap_open) {
          digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
          exhaust_flap_action_timer = millis();
          exhaust_flap_open = true;
          #if DEBUG_MODE
            Serial.println(F("Exhaust flap opened at RPM setpoint."));
          #endif
        }
      } else {
        if (exhaust_flap_open) {
          digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, HIGH);
          exhaust_flap_action_timer = millis();
          exhaust_flap_open = false;
          #if DEBUG_MODE
            Serial.println(F("Exhaust flap closed."));
          #endif
        }
      }
    }
  } else {                                                                                                                          // Flap always open in sport mode.
    if ((millis() - exhaust_flap_action_timer) >= 500) {
      if (!exhaust_flap_open) {
        digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
        exhaust_flap_action_timer = millis();
        exhaust_flap_open = true;
        #if DEBUG_MODE
          Serial.println(F("Opened exhaust flap with MDrive."));
        #endif
      }
    }
  }
}
#endif


void initialize_timers()
{
  #if F_ZBE_WAKE
    power_button_debounce_timer = dsc_off_button_debounce_timer = mdrive_message_timer 
    = vehicle_awake_timer = zbe_wakeup_last_sent = millis();
  #else
    power_button_debounce_timer = dsc_off_button_debounce_timer = mdrive_message_timer 
    = vehicle_awake_timer = millis();
  #endif
}


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
#endif  


void reset_runtime_variables()                                                                                                      // Ignition off. Set variables to original state and commit MDrive settings.
{
  dsc_program_last_status_can = 0xEA;
  dsc_program_status = 0;
  if (mdrive_status) {
    toggle_mdrive_message_active();
  }
  engine_running = false;
  RPM = 0;
  ignore_m_press = false;
  mdrive_power_active = console_power_mode = restore_console_power_mode = false;
  sending_dsc_off = send_second_dtc_press = send_dsc_off_from_mdm = false;
  sending_dsc_off_counter = 0;
  #if EXHAUST_FLAP_CONTROL
    exhaust_flap_sport = false;
    digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
    exhaust_flap_open = true;
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    lc_cc_active = clutch_pressed = vehicle_moving = false;
  #endif
  #if CONTROL_SHIFTLIGHTS
    shiftlights_segments_active = engine_warmed_up = false;
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
  if (mdrive_settings_change) {
    update_mdrive_settings_in_eeprom();
    mdrive_settings_change = false;
  }
}
