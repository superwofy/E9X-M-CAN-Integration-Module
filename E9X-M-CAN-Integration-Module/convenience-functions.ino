#if FRONT_FOG_INDICATOR
void evaluate_fog_status()
{
  if (k_msg.buf[0] != last_light_status) {
    if ((k_msg.buf[0] & 32) == 32) {                                                                                                  // Check the third bit of the first byte represented in binary for front fog status.
      front_fog_status = true;
      digitalWrite(FOG_LED_PIN, HIGH);
      #if DEBUG_MODE
        Serial.println("Front fogs on. Turned on FOG LED");
      #endif
    } else {
      front_fog_status = false;
      digitalWrite(FOG_LED_PIN, LOW);
      #if DEBUG_MODE
        Serial.println("Front fogs off. Turned off FOG LED");
      #endif
    }
    last_light_status = k_msg.buf[0];
  }
}
#endif


#if AUTO_SEAT_HEATING
void send_seat_heating_request(bool driver)
{
  unsigned long timeNow = millis();
  uint16_t canId;
  if (driver) {
    canId = 0x1E7;
    driver_sent_seat_heating_request = true;
  } else {
    canId = 0x1E8;
    passenger_sent_seat_heating_request = true;
  }
  delayedCanTxMsg m = {makeMsgBuf(canId, 2, seat_heating_button_pressed, 1), timeNow};
  seatHeatingTx.push(&m);
  m = {makeMsgBuf(canId, 2, seat_heating_button_released, 1), timeNow + 50};
  seatHeatingTx.push(&m);
  m = {makeMsgBuf(canId, 2, seat_heating_button_released, 1), timeNow + 100};
  seatHeatingTx.push(&m);
  #if DEBUG_MODE
    if (driver) {
      sprintf(serial_debug_string, "Sent driver's seat heating request at ambient %dC, treshold %dC.", 
           (ambient_temperature_can - 80) / 2, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2);
    } else {
      sprintf(serial_debug_string, "Sent passenger's seat heating request at ambient %dC, treshold %dC.", 
           (ambient_temperature_can - 80) / 2, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2);
    }
    Serial.println(serial_debug_string);
  #endif
}


void check_seatheating_queue() 
{
  if (!seatHeatingTx.isEmpty()) {
    delayedCanTxMsg delayedTx;
    seatHeatingTx.peek(&delayedTx);
    if (millis() >= delayedTx.transmitTime) {
      KCAN.write(delayedTx.txMsg);
      seatHeatingTx.drop();
    }
  }
}
#endif


#if F_ZBE_WAKE
void send_zbe_wakeup()
{
  KCAN.write(makeMsgBuf(0x560, 8, f_wakeup, 0));
  #if DEBUG_MODE
    Serial.println("Sent F-ZBE wake-up message.");
  #endif
}


void send_zbe_acknowledge()
{
  zbe_response[2] = k_msg.buf[7];
  KCAN.write(makeMsgBuf(0x277, 4, zbe_response, 0));
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent ZBE response to CIC with counter: 0x%X", k_msg.buf[7]);
    Serial.println(serial_debug_string);
  #endif
}
#endif


#if EXHAUST_FLAP_CONTROL
void change_exhaust_flap_position()
{
  if (engine_running) {
    if (!exhaust_flap_sport && !lc_cc_active) {                                                                                     // Exhaust is in quiet mode. Open with LC.
      if ((millis() - exhaust_flap_action_timer) >= 1500) {                                                                         // Avoid vacuum drain, oscillation and apply startup delay.
        if (RPM >= EXHAUST_FLAP_QUIET_RPM) {                                                                                        // Open at defined rpm setpoint.
          if (!exhaust_flap_open) {
            digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
            exhaust_flap_action_timer = millis();
            exhaust_flap_open = true;
            #if DEBUG_MODE
              Serial.println("Exhaust flap opened at RPM setpoint.");
            #endif
          }
        } else {
          if (exhaust_flap_open) {
            digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, HIGH);
            exhaust_flap_action_timer = millis();
            exhaust_flap_open = false;
            #if DEBUG_MODE
              Serial.println("Exhaust flap closed.");
            #endif
          }
        }
      }
    } else {                                                                                                                        // Flap always open in sport mode.
      if ((millis() - exhaust_flap_action_timer) >= 500) {
        if (!exhaust_flap_open) {
          digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
          exhaust_flap_action_timer = millis();
          exhaust_flap_open = true;
          #if DEBUG_MODE
            Serial.println("Opened exhaust flap with MDrive.");
          #endif
        }
      }
    }
  }
}
#endif


void evaluate_battery_engine() 
{
  #if DEBUG_MODE
    battery_voltage = (((k_msg.buf[1] - 240 ) * 256.0) + k_msg.buf[0]) / 68.0;
  #endif
  engine_running = !k_msg.buf[2] ? true : false;
}


void svt_kcan_cc_notification()
{
  if (pt_msg.buf[1] == 0x49 && pt_msg.buf[2] == 0) {                                                                                // Change from CC-ID 73 (EPS Inoperative) to CC-ID 70 (Servotronic).
    pt_msg.buf[1] = 0x46;
  }
  KCAN.write(pt_msg);                                                                                                               // Forward the SVT error status to KCAN.
}


void dcan_to_ptcan()
{
  PTCAN.write(d_msg);
}


void ptcan_to_dcan()
{
  DCAN.write(d_msg);
}