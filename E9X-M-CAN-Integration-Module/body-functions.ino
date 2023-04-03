void evaluate_ignition_status()
{
  if (k_msg.buf[0] == 0x45) {
    if (!ignition) {
      ignition = true;
      scale_mcu_speed();
      #if SERVOTRONIC_SVT70
        if (!digitalRead(POWER_BUTTON_PIN)) {                                                                                       // If POWER button is being held when turning on ignition, allow SVT diagnosis.
          diagnose_svt = true;
          KCAN.write(servotronic_cc_on_buf);                                                                                        // Indicate that diagnosing is now possible.
        }
      #endif
      #if DEBUG_MODE
        Serial.println("Ignition ON.");
      #endif
    }
  } else {
    if (ignition) {
      if (k_msg.buf[0] != 5) {                                                                                                      // 5 is sent in CA cars when the key is not detected, ignore.
        ignition = false;
        reset_runtime_variables();
        scale_mcu_speed();                                                                                                          // Now that the ignition is off, underclock the MCU
        #if DEBUG_MODE
          sprintf(serial_debug_string, "(%X) Ignition OFF. Reset values.", k_msg.buf[0]);
          Serial.println(serial_debug_string);
        #endif
      }
    }
  }

  if (!vehicle_awake) {
    vehicle_awake = true;    
    toggle_transceiver_standby();                                                                                                   // Re-activate the transceivers.                                                                                         
    #if DEBUG_MODE
      Serial.println("Vehicle Awake.");
    #endif
  }

  vehicle_awake_timer = millis();  
}


#if FRONT_FOG_INDICATOR
void evaluate_fog_status()
{
  if (k_msg.buf[0] != last_light_status) {
    if ((k_msg.buf[0] & 32) == 32) {                                                                                                  // Check the third bit of the first byte represented in binary for front fog status.
      if (!front_fog_status) {
        front_fog_status = true;
        digitalWrite(FOG_LED_PIN, HIGH);
        #if DEBUG_MODE
          Serial.println("Front fogs on. Turned on FOG LED");
        #endif
      }
    } else {
      if (front_fog_status) {
        front_fog_status = false;
        digitalWrite(FOG_LED_PIN, LOW);
        #if DEBUG_MODE
          Serial.println("Front fogs off. Turned off FOG LED");
        #endif
      }
    }
    last_light_status = k_msg.buf[0];
  }
}
#endif


#if AUTO_SEAT_HEATING
void evaluate_seat_heating_status()
{
  if (k_msg.id == 0x22A) {                                                                                                          // Passenger's seat heating status message is only sent with ignition on.
    passenger_seat_heating_status = !k_msg.buf[0] ? false : true;
  } else {
    if (!k_msg.buf[0]) {                                                                                                            // Check if seat heating is already on.
      if (!driver_sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) {
        send_seat_heating_request(true);
      }
    } else {
      driver_sent_seat_heating_request = true;                                                                                      // Seat heating already on. No need to request anymore.
    }
    driver_seat_heating_status = !k_msg.buf[0] ? false : true;
  }
}


void evaluate_passenger_seat_status()
{
  passenger_seat_status = k_msg.buf[1];
  if (ignition) {
    if (!passenger_seat_heating_status) {                                                                                         // Check if seat heating is already on.
      //This will be ignored if already on and cycling ignition. Press message will be ignored by IHK anyway.
      if (!passenger_sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) { 
        if (passenger_seat_status == 9) {                                                                                         // Passenger sitting and their seatbelt is on
          send_seat_heating_request(false);                                                                                       // Execute heating request here so we don't have to wait 15s for the next 0x22A.
        }
      }
    } else {
      passenger_sent_seat_heating_request = true;                                                                                 // Seat heating already on. No need to request anymore.
    }
  }
}


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
  delayed_can_tx_msg m = {released, timeNow + 100};
  seat_heating_tx.push(&m);
  m = {released, timeNow + 250};
  seat_heating_tx.push(&m);
  m = {released, timeNow + 400};
  seat_heating_tx.push(&m);
  #if DEBUG_MODE
    if (driver) {
      sprintf(serial_debug_string, "Sent driver's seat heating request at ambient %.1fC, treshold %.1fC.", 
           (ambient_temperature_can - 80) / 2.0, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2.0);
    } else {
      sprintf(serial_debug_string, "Sent passenger's seat heating request at ambient %.1fC, treshold %.1fC.", 
           (ambient_temperature_can - 80) / 2.0, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2.0);
    }
    Serial.println(serial_debug_string);
  #endif
}


void check_seatheating_queue() 
{
  if (!seat_heating_tx.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    seat_heating_tx.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      KCAN.write(delayed_tx.tx_msg);
      seat_heating_tx.drop();
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
    if (exhaust_flap_sport) {                                                                                                       // Flap always open in sport mode.
      if ((millis() - exhaust_flap_action_timer) >= 500) {
        if (!exhaust_flap_open) {
          actuate_exhaust_solenoid(LOW);
          #if DEBUG_MODE
            Serial.println("Opened exhaust flap with MDrive.");
          #endif
        }
      }
    }
  }
}


void control_exhaust_flap_rpm()
{
  if (engine_running) {
    if (!exhaust_flap_sport) {
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
      delayed_can_tx_msg m = {pdc_beep_buf, timeNow};
      pdc_beep_tx.push(&m);
      m = {pdc_quiet_buf, timeNow + 150};
      pdc_beep_tx.push(&m);
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
  if (!pdc_beep_tx.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    pdc_beep_tx.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      KCAN.write(delayed_tx.tx_msg);
      pdc_beep_tx.drop();
    }
  }
}
#endif


void evaluate_engine_rpm()
{
  RPM = ((uint32_t)k_msg.buf[5] << 8) | (uint32_t)k_msg.buf[4];
  if (!engine_running && (RPM > 2000)) {
    engine_running = true;
    #if CONTROL_SHIFTLIGHTS
      startup_animation();                                                                                                          // Show off shift light segments during engine startup (>500rpm).
    #endif
    #if EXHAUST_FLAP_CONTROL
      exhaust_flap_action_timer = millis();                                                                                         // Start tracking the exhaust flap.
    #endif
    #if DEBUG_MODE
      Serial.println("Engine started.");
    #endif
  } else if (engine_running && (RPM < 200)) {                                                                                       // Less than 50 RPM. Engine stalled or was stopped.
    engine_running = false;
    #if DEBUG_MODE
      Serial.println("Engine stopped.");
    #endif
  }
}


#if DEBUG_MODE
void evaluate_battery_voltage() 
{
  battery_voltage = (((k_msg.buf[1] - 240 ) * 256.0) + k_msg.buf[0]) / 68.0;
}
#endif


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
  if (!deactivate_ptcan_temporariliy) {
    PTCAN.write(d_msg);
    #if DEBUG_MODE
      dcan_forwarded_count++;
    #endif
  }
}


void ptcan_to_dcan()
{
  DCAN.write(pt_msg);
  #if DEBUG_MODE
    ptcan_forwarded_count++;
  #endif
}
#endif


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
    if (dsc_program_status != 2) {
      if (!holding_dsc_off_console) {
        holding_dsc_off_console = true;
        dsc_off_button_hold_timer = millis();
      } else {
        if ((millis() - dsc_off_button_hold_timer) >= dsc_hold_time_ms) {                                                           // DSC OFF sequence should only be sent after user holds button for a configured time
          if ((millis() - dsc_off_button_debounce_timer) >= dsc_debounce_time_ms) {
            #if DEBUG_MODE
              Serial.println("Console: DSC OFF button held. Sending DSC OFF.");
            #endif
            send_dsc_mode(2);
            dsc_off_button_debounce_timer = millis();
          }
        }
      }      
    } else {
      if ((millis() - dsc_off_button_debounce_timer) >= dsc_debounce_time_ms) {                                                     // A quick tap re-enables everything
        #if DEBUG_MODE
          Serial.println("Console: DSC button tapped. Sending DSC ON.");
        #endif
        dsc_off_button_debounce_timer = millis();
        send_dsc_mode(0);
      }
    }
  } else {
    holding_dsc_off_console = false;
  }
}


#if RTC
void update_idrive_time_from_rtc()
{
  #if DEBUG_MODE
    Serial.println("Vehicle date/time not set. Setting from RTC.");
  #endif
  time_t t = now();
  uint8_t rtc_hours = hour(t);
  uint8_t rtc_minutes = minute(t);
  uint8_t rtc_seconds = second(t);
  uint8_t rtc_day = day(t);
  uint8_t rtc_month = month(t);
  uint16_t rtc_year = year(t);
  uint8_t date_time_can[] = {rtc_hours, rtc_minutes, rtc_seconds, 
                            rtc_day, uint8_t((rtc_month << 4) | 0xF), uint8_t(rtc_year & 0xFF), uint8_t(rtc_year >> 8), 0xF2};
  KCAN.write(makeMsgBuf(0x39E, 8, date_time_can));
}
#endif


#if DOOR_VOLUME
void evaluate_door_status()
{
  if (k_msg.id == 0xE2) {
    if (k_msg.buf[3] == 0xFD) {
      if (!left_door_open) {
        left_door_open = true;
        #if DEBUG_MODE
          #if RHD
            Serial.println("Passenger's door open.");
          #else
            Serial.println("Driver's door open.");
          #endif
        #endif
        if (!right_door_open) {
          send_volume_request();
        }
      }
    } else if (k_msg.buf[3] == 0xFC) {
      if (left_door_open) {
        left_door_open = false;
        #if DEBUG_MODE
          #if RHD
            Serial.println("Passenger's door closed.");
          #else
            Serial.println("Driver's door closed.");
          #endif
        #endif
        if (!right_door_open) {
          send_volume_request();
        }
      }
    }
  } else if (k_msg.id == 0xEA) {
    if (k_msg.buf[3] == 0xFD) {
      if (!right_door_open) {
        right_door_open = true;
        #if DEBUG_MODE
          #if RHD
            Serial.println("Driver's's door opened.");
          #else
            Serial.println("Passenger's door opened.");
          #endif
        #endif
        if (!left_door_open) {
          send_volume_request();
        }
      }
    } else if (k_msg.buf[3] == 0xFC) {
      if (right_door_open) {
        right_door_open = false;
        #if DEBUG_MODE
          #if RHD
            Serial.println("Driver's's door closed.");
          #else
            Serial.println("Passenger's door closed.");
          #endif
        #endif
        if (!left_door_open) {
          send_volume_request();
        }
      }
    }
  }
}


void send_volume_request()
{
  volume_requested = true;
  KCAN.write(vol_request_buf);
}


void evaluate_audio_volume()
{
  if (volume_requested) {                                                                                                           // Make sure that the module is the one that requested this result.
    if (k_msg.buf[3] == 0x24) {                                                                                                     // status_volumeaudio response.
      uint8_t audio_volume = k_msg.buf[4];
      if (audio_volume > 0) {
        #if DEBUG_MODE
          sprintf(serial_debug_string, "Received audio volume: 0x%X", audio_volume);
          Serial.println(serial_debug_string);
        #endif
        uint8_t volume_change[] = {0x63, 4, 0x31, 0x23, 0, 0, 0, 0};
        if (!volume_reduced) {
          if (left_door_open || right_door_open) {
            volume_restore_offset = (audio_volume % 2) == 0 ? 0 : 1;                                                                  // Volumes adjusted from faceplate go up by 1 while MFL goes up by 2.
            volume_change[4] = floor(audio_volume / 2);                                                                               // Reduce volume to 50%.
            if ((volume_change[4] + volume_restore_offset) > 0x20) {                                                                  // Don't blow the speakers out if something went wrong...
              volume_change[4] = 0x20;
              volume_restore_offset = 0;
            }
            KCAN.write(makeMsgBuf(0x6F1, 8, volume_change));
            volume_reduced = true;
            volume_changed_to = volume_change[4];                                                                                     // Save this value to compare when door is closed back.
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Reducing audio volume with door open to: 0x%X", volume_changed_to);
              Serial.println(serial_debug_string);
            #endif
          }
        } else {
          if (!left_door_open && !right_door_open) {
            if (volume_reduced) {
              if (audio_volume == volume_changed_to) {
                volume_change[4] = audio_volume * 2 + volume_restore_offset;
                if (volume_change[4] > 0x33) {
                  volume_change[4] = 0x33;                                                                                            // Set a nanny in case the code goes wrong. 0x33 is pretty loud...
                }
                delayed_can_tx_msg m;
                if (ignition && !engine_running) {
                  m = {makeMsgBuf(0x6F1, 8, volume_change), millis() + 500};                                                          // Need this delay when Ignition is on and the warning gong is on.
                  audio_volume_tx.push(&m);
                  m = {makeMsgBuf(0x6F1, 8, volume_change), millis() + 1000};                                                         // Send more in case we get lucky and the gong shuts up early...
                  audio_volume_tx.push(&m);
                  m = {makeMsgBuf(0x6F1, 8, volume_change), millis() + 1700};
                  audio_volume_tx.push(&m);
                } else {
                  m = {makeMsgBuf(0x6F1, 8, volume_change), millis() + 200};
                  audio_volume_tx.push(&m);
                  m = {makeMsgBuf(0x6F1, 8, volume_change), millis() + 500};
                  audio_volume_tx.push(&m);
                }
                #if DEBUG_MODE
                  sprintf(serial_debug_string, "Restoring audio volume with door closed. to: 0x%X", volume_change[4]);
                  Serial.println(serial_debug_string);
                #endif
              }
              volume_reduced = false;
            }
          }
        }
      }
    }
    volume_requested = false;
  }
}


void check_audio_queue() 
{
  if (!audio_volume_tx.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    audio_volume_tx.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      if (vehicle_awake) {
        KCAN.write(delayed_tx.tx_msg);
        #if DEBUG_MODE
          Serial.println("Sent volume change job to iDrive.");
        #endif
      }
      audio_volume_tx.drop();
    }
  }
}
#endif
