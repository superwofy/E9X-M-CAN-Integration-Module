#if FRONT_FOG_INDICATOR
void evaluate_fog_status()
{
  if (prxBuf[0] != last_light_status) {
    if ((prxBuf[0] & 32) == 32) {                                                                                            // Check the third bit of the first byte represented in binary for front fog status.
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
    last_light_status = prxBuf[0];
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
    sprintf(serial_debug_string, "Sent dr seat heating request at ambient %dC, treshold %dC\n", (ambient_temperature_can - 80) / 2, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2);
    Serial.print(serial_debug_string);
  #endif
  sent_seat_heating_request = true;
}
#endif


#if F_ZBE_WAKE
void send_zbe_wakeup()
{
  if ((millis() - zbe_wakeup_last_sent) >= 1000) {
    KCAN.sendMsgBuf(0x560, 8, f_wakeup);
    zbe_wakeup_last_sent = millis();
    #if DEBUG_MODE
      Serial.println(F("Sent F-ZBE wake-up message"));
    #endif
  }
}
#endif


#if EXHAUST_FLAP_WITH_M_BUTTON
void evaluate_exhaust_flap_position()
{
  if (!exhaust_flap_sport) {                                                                                                        // Exhaust is in quiet mode
    if ((millis() - exhaust_flap_action_timer) >= 1000) {                                                                           // Avoid vacuum drain, oscillation and apply startup delay
      if (RPM >= EXHAUST_FLAP_QUIET_RPM) {                                                                                          // Open at defined rpm setpoint
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
  } else {                                                                                                                          // Flap always open in sport mode
    if ((millis() - exhaust_flap_action_timer) >= 200) {
      if (!exhaust_flap_open) {
        digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
        exhaust_flap_action_timer = millis();
        exhaust_flap_open = true;
        #if DEBUG_MODE
          Serial.println(F("Opened exhaust flap with MDrive/POWER."));
        #endif
      }
    }
  }
}
#endif