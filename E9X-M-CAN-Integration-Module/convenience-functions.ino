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
  KCAN.write(makeMsgBuf(canId, 2, seat_heating_button_pressed));
  CAN_message_t released = makeMsgBuf(canId, 2, seat_heating_button_released);
  delayedCanTxMsg m = {released, timeNow + 100};
  seatHeatingTx.push(&m);
  m = {released, timeNow + 250};
  seatHeatingTx.push(&m);
  m = {released, timeNow + 400};
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
  KCAN.write(f_wakeup_buf);
  #if DEBUG_MODE
    Serial.println("Sent F-ZBE wake-up message.");
  #endif
}


void send_zbe_acknowledge()
{
  zbe_response[2] = k_msg.buf[7];
  KCAN.write(makeMsgBuf(0x277, 4, zbe_response));
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent ZBE response to CIC with counter: 0x%X", k_msg.buf[7]);
    Serial.println(serial_debug_string);
  #endif
}
#endif


#if EXHAUST_FLAP_CONTROL
void control_exhaust_flap_user()
{
  if (engine_running) {
    #if LAUNCH_CONTROL_INDICATOR
    if (exhaust_flap_sport || lc_cc_active) {                                                                                       // Exhaust is in quiet mode. Open with LC.
    #else
    if (exhaust_flap_sport) {                                                                                                       // Flap always open in sport mode.
    #endif                                                                                                                      
      if ((millis() - exhaust_flap_action_timer) >= 500) {
        if (!exhaust_flap_open) {
          actuate_exhaust_solenoid(LOW);
          #if DEBUG_MODE
            if (lc_cc_active) {
              Serial.println("Opened exhaust flap with Launch Control.");
            } else {
              Serial.println("Opened exhaust flap with MDrive.");
            }
          #endif
        }
      }
    }
  }
}


void control_exhaust_flap_rpm()
{
  if (engine_running) {
    #if LAUNCH_CONTROL_INDICATOR
    if (!exhaust_flap_sport && !lc_cc_active) {                                                                                     // Only execute these checks if the flap is not user-controlled
    #else
    if (!exhaust_flap_sport) {
    #endif
      if ((millis() - exhaust_flap_action_timer) >= exhaust_flap_action_interval) {                                                 // Avoid vacuum drain, oscillation and apply startup delay.
        if (RPM >= EXHAUST_FLAP_QUIET_RPM) {                                                                                        // Open at defined rpm setpoint.
          if (!exhaust_flap_open) {
            actuate_exhaust_solenoid(LOW);
            #if DEBUG_MODE
              Serial.println("Exhaust flap opened at RPM setpoint.");
            #endif
          }
        } else {
          if (exhaust_flap_open) {
            actuate_exhaust_solenoid(HIGH);
            #if DEBUG_MODE
              Serial.println("Exhaust flap closed.");
            #endif
          }
        }
      }
    }
  }
}

void actuate_exhaust_solenoid(bool activate)
{
  digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, activate);
  exhaust_flap_action_timer = millis();
  exhaust_flap_open = !activate;                                                                                                    // Flap position is the inverse of solenoid state. When active, the flap is closed.
}
#endif


#if REVERSE_BEEP
void evaluate_pdc_beep()
{
  if (k_msg.buf[0] == 0xFE) {
    if (!pdc_beep_sent) {
      unsigned long timeNow = millis();
      delayedCanTxMsg m = {pdc_beep_buf, timeNow};
      pdcBeepTx.push(&m);
      m = {pdc_quiet_buf, timeNow + 150};
      pdcBeepTx.push(&m);
      pdc_beep_sent = true;
      #if DEBUG_MODE
        Serial.println("Sending PDC beep.");
      #endif
    }
  } else {
    if (pdc_beep_sent) {
      pdc_beep_sent = false;
    }
  }
}


void check_pdc_queue() 
{
  if (!pdcBeepTx.isEmpty()) {
    delayedCanTxMsg delayedTx;
    pdcBeepTx.peek(&delayedTx);
    if (millis() >= delayedTx.transmitTime) {
      KCAN.write(delayedTx.txMsg);
      pdcBeepTx.drop();
    }
  }
}
#endif


void evaluate_battery_engine() 
{
  #if DEBUG_MODE
    battery_voltage = (((k_msg.buf[1] - 240 ) * 256.0) + k_msg.buf[0]) / 68.0;
  #endif

  // 0x89: terminal R / off, 0x80 Terminal 15, 9 engine turning off.
  if (k_msg.buf[2] == 9 || k_msg.buf[2] == 0x89 || k_msg.buf[2] == 0x80) {                                                          // Use the charge status byte to determine if the engine is running.
    engine_running = false;
  } else {
    engine_running = true;
  }
}


#if SERVOTRONIC_SVT70
void send_svt_kcan_cc_notification()
{
  if (pt_msg.buf[1] == 0x49 && pt_msg.buf[2] == 0) {                                                                                // Change from CC-ID 73 (EPS Inoperative) to CC-ID 70 (Servotronic).
    pt_msg.buf[1] = 0x46;
  }
  KCAN.write(pt_msg);                                                                                                               // Forward the SVT error status to KCAN.
}


void dcan_to_ptcan()
{
  PTCAN.write(d_msg);
  #if DEBUG_MODE
    dcan_forwarded_count++;
  #endif
}


void ptcan_to_dcan()
{
  DCAN.write(pt_msg);
  #if DEBUG_MODE
    ptcan_forwarded_count++;
  #endif
}
#endif


void send_dme_ckm()
{
  byte dme_ckm[] = {0xF2, 0xFF};
  KCAN.write(makeMsgBuf(0x3A9, 2, dme_ckm));                                                                                        // This is sent by the DME to populate the M Key iDrive section
  #if DEBUG_MODE
    Serial.println("Sent dummy DME POWER CKM.");
  #endif
}


void check_console_buttons()
{
  if (!digitalRead(POWER_BUTTON_PIN)) {
    if ((millis() - power_button_debounce_timer) >= power_debounce_time_ms) {                                                       // POWER console button should only change throttle mapping.
      power_button_debounce_timer = millis();
      if (!console_power_mode) {
        if (!mdrive_power_active) {
          console_power_mode = true;
          #if DEBUG_MODE
            Serial.println("Console: POWER mode ON.");
          #endif 
        } else {
          mdrive_power_active = false;                                                                                              // If POWER button was pressed while MDrive POWER is active, disable POWER.
          #if DEBUG_MODE
            Serial.println("Deactivated MDrive POWER with console button press.");
          #endif
        }
      } else {
        #if DEBUG_MODE
          Serial.println("Console: POWER mode OFF.");
        #endif
        console_power_mode = false;
        if (mdrive_power_active) {
          mdrive_power_active = false;                                                                                              // If POWER button was pressed while MDrive POWER is active, disable POWER.
          #if DEBUG_MODE
            Serial.println("Deactivated MDrive POWER with console button press.");
          #endif
        }
      }
    }
  } 
  
  if (!digitalRead(DSC_BUTTON_PIN)) {
    if (dsc_program_status == 0) {
      if (!holding_dsc_off_console) {
        holding_dsc_off_console = true;
        dsc_off_button_hold_timer = millis();
      } else {
        if ((millis() - dsc_off_button_hold_timer) >= dsc_hold_time_ms) {                                                           // DSC OFF sequence should only be sent after user holds button for a configured time
          #if DEBUG_MODE
            if (!sending_dsc_off) {
              Serial.println("Console: DSC OFF button held. Sending DSC OFF.");
            }
          #endif
          send_dsc_off_sequence();
          dsc_off_button_debounce_timer = millis();
        }
      }      
    } else {
      if ((millis() - dsc_off_button_debounce_timer) >= dsc_debounce_time_ms) {                                                     // A quick tap re-enables everything
        #if DEBUG_MODE
          if (!sending_dsc_off) {
            Serial.println("Console: DSC button tapped. Re-enabling DSC normal program.");
          }
        #endif
        dsc_off_button_debounce_timer = millis();
        send_dtc_button_press(false);
      }
    }
  } else {
    holding_dsc_off_console = false;
  }
}
