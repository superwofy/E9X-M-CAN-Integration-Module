// General body functions dealing with interior electronics go here.


void evaluate_terminal_clutch_keyno_status(void) {
  vehicle_awake_timer = 0;
  if (!vehicle_awake) {
    vehicle_awake = true;    
    serial_log("Vehicle Awake.", 0);
    toggle_transceiver_standby(false);                                                                                              // Re-activate the transceivers.                                                                                         
    vehicle_awakened_timer = 0;
  }

  bool ignition_ = ignition, terminal_r_ = terminal_r;

  // These must be checked since error states such as 2 or 3 can be set. E.g when PGS is being reset (Byte0 = 0x27).
  terminal_r = (k_msg.buf[0] & 0b11) == 1 ? true : false;                                                                           // 0 OFF, 1 ON, 3 INVALID_SIGNAL ??
  ignition = ((k_msg.buf[0] & 0b1100) >> 2) == 1 ? true : false;                                                                    // 0 IGK_OFF, 1 IGK_ON, 2 INVALID_SIGNAL
  terminal_50 = ((k_msg.buf[0] & 0b110000) >> 4) == 1 ? true : false;
  key_valid = ((k_msg.buf[0] & 0b11000000) >> 6) == 1 ? true : false;                                                               // Set to 0 when PGS module loses key.

  #if LAUNCH_CONTROL_INDICATOR
  if (ignition) {
    bool clutch_pressed_can = (k_msg.buf[2] & 0b11000000) >> 6;
    if (clutch_pressed != clutch_pressed_can) {
      if (clutch_pressed_can) {
        clutch_pressed = true;
        serial_log("Clutch pedal depressed.", 2);
      } else {
        clutch_pressed = false;
        serial_log("Clutch pedal released.", 2);
      }
    }
  }
  #endif

  #if F_ZBE_WAKE || F_VSW01 || F_NIVI                                                                                               // Translate 0x130 to 0x12F for F-series modules.
    uint8_t f_terminal_status[] = {0, 0, 0, 0xFF, 0, 0, 0x3F, 0xFF};
    f_terminal_status[2] = 0xF << 4;                                                                                                // Set ST_KL
    if (terminal_50) {
      f_terminal_status[1] = 0xB;
      f_terminal_status[2] |= 0xD;
    } else if (ignition) {
      f_terminal_status[1] = 5;
      f_terminal_status[2] |= 0xA;
    } else if (terminal_r) {
      f_terminal_status[1] = 2;
      f_terminal_status[2] |= 8;
    } else {
      f_terminal_status[1] = 1;
      f_terminal_status[2] |= 1;
    }
    f_terminal_status[1] = f_terminal_status[1] << 4 | f_terminal_status_alive_counter;                                             // Combine ST_VEH_CON and ALIV_COU_KL
    f_terminal_status_alive_counter == 0xF ? f_terminal_status_alive_counter = 0 : f_terminal_status_alive_counter++;
    f_terminal_status[4] = (0xF << 4) | key_valid ? 3 : 1;                                                                          // Set ST_KL_KEY_VLD

    f_terminal_status_crc.restart();
    for (uint8_t i = 1; i < 8; i++) {
      f_terminal_status_crc.add(f_terminal_status[i]);
    }
    f_terminal_status[0] = f_terminal_status_crc.calc();

    CAN_message_t f_terminal_status_buf = make_msg_buf(0x12F, 8, f_terminal_status);
    if (!frm_consumer_shutdown) {
      #if F_ZBE_WAKE || F_VSW01
        kcan_write_msg(f_terminal_status_buf);
      #endif
      #if F_NIVI
        ptcan_write_msg(f_terminal_status_buf);
      #endif
    }
  #endif

  #if FAKE_MSA || MSA_RVC
    if (ignition) {
      if (msa_fake_status_counter == 5){
        kcan_write_msg(msa_fake_status_buf);                                                                                        // Send this message every 500ms to keep the IHKA module happy.
        msa_fake_status_counter = 0;
      }
      msa_fake_status_counter++;
    }
  #endif

  if (ignition && !ignition_) {                                                                                                     // Ignition changed from OFF to ON.
    scale_cpu_speed();
    #if USB_DISABLE
      activate_usb(100);                                                                                                            // If this fails to run, the program button will need to be pressed to recover.
    #endif
    serial_log("Ignition ON.", 2);

    #if DIM_DRL                                                                                                                     // These resets are required if quickly (less than 15s) switching igniton 
      if (((millis() - last_drl_action_timer) < 15000)) {
        // RESET - TODO
        last_drl_action_timer = 0;
      }
    #endif
    #if FRONT_FOG_CORNER
      if (((millis() - last_fog_action_timer) < 15000)) {
        // RESET - TODO
        last_fog_action_timer = 0;
      }
    #endif
  } else if (!ignition && ignition_) {
    reset_ignition_variables();
    scale_cpu_speed();                                                                                                              // Now that the ignition is OFF, underclock the MCU
    serial_log("Ignition OFF. Reset values.", 2);
  }

  if (terminal_r && !terminal_r_) {
    serial_log("Terminal R ON.", 2);
    #if RTC
      if (!rtc_valid) {
        kcan_write_msg(set_time_cc_buf);                                                                                            // Warn that the time needs to be updated by the user.
      }
    #endif
    #if FRM_AHL_MODE
      kcan_write_msg(frm_ckm_ahl_komfort_buf);                                                                                      // Make sure we're in comfort mode on startup.
    #endif
    #if COMFORT_EXIT
      comfort_exit_ready = false;
    #endif
  } else if (!terminal_r && terminal_r_) {
    serial_log("Terminal R OFF.", 2);
    #if COMFORT_EXIT
      comfort_exit_ready = true;
    #endif
    #if INTERMITTENT_WIPERS
      intermittent_wipe_active = false;
    #endif
  }
}


void evaluate_frm_consumer_shutdown(void) {
  // The FRM sends this request after the vehicle has been sleeping for a predefined time.
  // This can be observed when the car wakes up and turns off the interior lights. It then goes to deep sleep.
  // OR
  // If the car is locked, FC is sent immediately.
  // OR
  // If the car is half-woken with the remote trunk button or lock button while locked, FC is sent immediately.
  if (k_msg.buf[0] == 0xFC) {
    if (!frm_consumer_shutdown) {
      frm_consumer_shutdown = true;
      serial_log("FRM requested consumers OFF.", 2);
      scale_cpu_speed();                                                                                                            // Reduce power consumption in this state.
      toggle_transceiver_standby(frm_consumer_shutdown);                                                                            // KCAN is all that's needed to resume operation later.
    }
  } else if (k_msg.buf[0] == 0xFD) {
    if (frm_consumer_shutdown) {
      frm_consumer_shutdown = false;
      serial_log("FRM requested consumers back ON.", 2);
      toggle_transceiver_standby(frm_consumer_shutdown);
    }
  }
}


void evaluate_seat_heating_status(void) {
  if (k_msg.id == 0x232) {
    driver_seat_heating_status = !k_msg.buf[0] ? false : true;
    if (!driver_seat_heating_status) {                                                                                              // Check if seat heating is already ON.
      if (!driver_sent_seat_heating_request) { 
        if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_HIGH) {
          send_seat_heating_request_dr(false);
        } else if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_MEDIUM) {
          send_seat_heating_request_dr(true);
        }
      }
    } else {
      driver_sent_seat_heating_request = true;                                                                                      // Seat heating already ON. No need to request anymore.
    }
  } 
  #if AUTO_SEAT_HEATING_PASS
  else {                                                                                                                            // Passenger's seat heating status message is only sent with ignition ON.
    passenger_seat_heating_status = !k_msg.buf[0] ? false : true;
    if (!passenger_seat_heating_status) {
      if (!passenger_sent_seat_heating_request) {
        if (bitRead(passenger_seat_status, 0) && bitRead(passenger_seat_status, 3)) {                                               // Occupied and belted.
          if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_HIGH) {
            send_seat_heating_request_pas(false);
          } else if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_MEDIUM) {
            send_seat_heating_request_pas(true);
          }
        }
      }
    } else {
      passenger_sent_seat_heating_request = true;
    }
  }
  #endif
}


void send_seat_heating_request_dr(bool medium) {
  unsigned long time_now = millis();
  kcan_write_msg(seat_heating_button_pressed_dr_buf);
  driver_sent_seat_heating_request = true;
  m = {seat_heating_button_released_dr_buf, time_now + 100};
  seat_heating_dr_txq.push(&m);
  m = {seat_heating_button_released_dr_buf, time_now + 200};
  seat_heating_dr_txq.push(&m);
  m = {seat_heating_button_released_dr_buf, time_now + 400};
  seat_heating_dr_txq.push(&m);
  if (medium) {
    m = {seat_heating_button_pressed_dr_buf, time_now + 600};
    seat_heating_dr_txq.push(&m);
    m = {seat_heating_button_released_dr_buf, time_now + 700};
    seat_heating_dr_txq.push(&m);
    m = {seat_heating_button_released_dr_buf, time_now + 900};
    seat_heating_dr_txq.push(&m);
    m = {seat_heating_button_released_dr_buf, time_now + 1100};
    seat_heating_dr_txq.push(&m);
  }
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent [%s] driver's seat heating request at ambient temp: %.1fC.",
            medium ? "medium" : "high", ambient_temperature_real);
    serial_log(serial_debug_string, 2);
  #endif
}


void check_seatheating_queue(void) {
  if (!seat_heating_dr_txq.isEmpty()) {
    seat_heating_dr_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      seat_heating_dr_txq.drop();
    }
  }
  if (!seat_heating_pas_txq.isEmpty()) {
    seat_heating_pas_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      seat_heating_pas_txq.drop();
    }
  }
}


void evaluate_passenger_seat_status(void) {
  passenger_seat_status = k_msg.buf[1];
  if (ignition) {
    if (!passenger_seat_heating_status) {                                                                                           // Check if seat heating is already ON.
      //This will be ignored if already ON and cycling ignition. Press message will be ignored by IHK anyway.
      if (!passenger_sent_seat_heating_request) {
        if (bitRead(passenger_seat_status, 0) && bitRead(passenger_seat_status, 3)) {                                               // Occupied and belted.
          if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_HIGH) {                                                        // Execute heating requests here so we don't have to wait 15s for the next 0x22A.
            send_seat_heating_request_pas(false);
          } else if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_MEDIUM) {
            send_seat_heating_request_pas(true);
          }
        }
      }
    } else {
      passenger_sent_seat_heating_request = true;                                                                                   // Seat heating already ON. No need to request anymore.
    }
  }
}


void send_seat_heating_request_pas(bool medium) {
  unsigned long time_now = millis();
  kcan_write_msg(seat_heating_button_pressed_pas_buf);
  passenger_sent_seat_heating_request = true;
  m = {seat_heating_button_released_pas_buf, time_now + 100};
  seat_heating_pas_txq.push(&m);
  m = {seat_heating_button_released_pas_buf, time_now + 200};
  seat_heating_pas_txq.push(&m);
  m = {seat_heating_button_released_pas_buf, time_now + 400};
  seat_heating_pas_txq.push(&m);
  if (medium) {
    m = {seat_heating_button_pressed_pas_buf, time_now + 600};
    seat_heating_pas_txq.push(&m);
    m = {seat_heating_button_released_pas_buf, time_now + 700};
    seat_heating_pas_txq.push(&m);
    m = {seat_heating_button_released_pas_buf, time_now + 900};
    seat_heating_pas_txq.push(&m);
    m = {seat_heating_button_released_pas_buf, time_now + 1100};
    seat_heating_pas_txq.push(&m);
  }
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent [%s] passenger's seat heating request at ambient temp: %.1fC.",
            medium ? "medium" : "high", ambient_temperature_real);
    serial_log(serial_debug_string, 2);
  #endif
}


void evaluate_steering_heating_request(void) {
  // Wait for supply voltage to stabilize.
  // Also limit the time the request can be sent after as the driver may already have eanbled the heater.
  // The temperature may also have dropped below the threshold while the driver already enabled the heater.
  // There is no way to track the steering heater via CAN.

  if (engine_runtime >= AUTO_HEATING_START_DELAY && engine_runtime <= 8000) {                                                               
    if (!sent_steering_heating_request) {
      if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD_HIGH) {
        digitalWrite(STEERING_HEATER_SWITCH_PIN, HIGH);
        serial_log("Activated steering wheel heating.", 2);
        sent_steering_heating_request = transistor_active = true;
        transistor_active_timer = 0;
      } else {
        sent_steering_heating_request = true;                                                                                       // If the conditions aren't right, cancel activation for this wake cycle.
      }
    } else {
      if (transistor_active) {
        if (transistor_active_timer >= 200) {
          digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);                                                                            // Release control of the switch so that the driver can now operate it.
          transistor_active = false;
        }
      }
    }
  }
}


void send_f_wakeup(void) {
  kcan_write_msg(f_kombi_network_mgmt_buf);
  #if F_NIVI
    if (ignition) {
      ptcan_write_msg(f_kombi_network_mgmt_buf);
    }
  #endif
    //serial_log("Sent FXX wake-up message.", 2);
}


void send_f_vehicle_mode(void) {
  if (f_vehicle_mode_timer >= 5000) {
    uint8_t f_vehicle_mode[] = {0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0};
    if (battery_voltage < 12.00 && battery_voltage > 11.60) {
      f_vehicle_mode[2] = 0xF1;                                                                                                     // Energy OK.
    } else if (battery_voltage < 11.60 && battery_voltage > 11.20) {
      f_vehicle_mode[2] = 0xF2;                                                                                                     // Energy shortage.
    } else if (battery_voltage < 11.20) {
      f_vehicle_mode[2] = 0xF3;                                                                                                     // Energy severe shortage.
    } else {
      f_vehicle_mode[2] = 0xF0;                                                                                                     // Energy good.
    }
    CAN_message_t f_vehicle_mode_buf = make_msg_buf(0x3A0, 8, f_vehicle_mode);
    kcan_write_msg(f_vehicle_mode_buf);
    #if F_NIVI
      if (ignition) {                                                                                                               // NiVi is powered by Terminal 15.
        ptcan_write_msg(f_vehicle_mode_buf);
      }
    #endif

    f_vehicle_mode_timer = 0;
  }
}


void send_zbe_acknowledge(void) {
  uint8_t zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  zbe_response[2] = k_msg.buf[7];
  kcan_write_msg(make_msg_buf(0x277, 4, zbe_response));
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent ZBE response to CIC with counter: 0x%X", k_msg.buf[7]);
    serial_log(serial_debug_string, 2);
  #endif
}


void evaluate_battery_voltage(void) {
  battery_voltage = (((pt_msg.buf[1] - 240 ) * 256.0) + pt_msg.buf[0]) / 68.0;
}


void check_power_led_state(void) {
  if (power_led_delayed_off_action) {
    if (millis() >= power_led_delayed_off_action_time) {
      digitalWrite(POWER_LED_PIN, LOW);
      power_led_delayed_off_action = false;
    }
  }
}


void check_console_buttons(void) {
  #if IMMOBILIZER_SEQ
  if (immobilizer_released) {
    if (!digitalRead(POWER_BUTTON_PIN) && !digitalRead(DSC_BUTTON_PIN)) {
      if (!holding_both_console) {
        both_console_buttons_timer = 0;
        holding_both_console = true;
      } else {
        if (both_console_buttons_timer >= 10000) {                                                                                  // Hold both buttons for more than 10s.
          kcan_write_msg(cic_button_sound_buf);                                                                                     // Acknowledge Immobilizer persist ON-OFF with Gong.
          immobilizer_persist = !immobilizer_persist;
          EEPROM.update(14, immobilizer_persist);
          update_eeprom_checksum();
          #if DEBUG_MODE
            sprintf(serial_debug_string, "Anti theft now persistently: %s.", immobilizer_persist ? "ON" : "OFF");
            serial_log(serial_debug_string, 0);
          #endif
          both_console_buttons_timer = 0;                                                                                           // Reset to prevent multiple activations.
        }
      }
      return;                                                                                                                       // Prevent nuisance mode changes while holding both buttons.
    } else {
      holding_both_console = false;
    }
  }
  #endif

  if (!digitalRead(POWER_BUTTON_PIN)) {
    if (power_button_debounce_timer >= power_debounce_time_ms) {                                                                    // POWER console button should only change throttle mapping.
      power_button_debounce_timer = 0;
      if (!console_power_mode) {
        if (!mdrive_power_active) {
          console_power_mode = true;
          serial_log("Console: POWER mode ON.", 2);
        } else {
          mdrive_power_active = false;                                                                                              // If POWER button was pressed while MDrive POWER is active, disable POWER.
          serial_log("Deactivated MDrive POWER with console button press.", 2);
        }
      } else {
        serial_log("Console: POWER mode OFF.", 2);
        console_power_mode = false;
        if (mdrive_power_active) {
          mdrive_power_active = false;                                                                                              // If POWER button was pressed while MDrive POWER is active, disable POWER.
          serial_log("Deactivated MDrive POWER with console button press.", 2);
        }
      }
      send_power_mode();
    }
  } 
  
  if (!digitalRead(DSC_BUTTON_PIN)) {
    if (!holding_dsc_off_console) {
      holding_dsc_off_console = true;
      dsc_off_button_hold_timer = 0;
    }
    
    if (dsc_off_button_hold_timer >= dsc_hold_time_ms) {                                                                            // DSC OFF sequence should only be sent after user holds button for a configured time.
      if (dsc_off_button_debounce_timer >= dsc_debounce_time_ms) {
        if (dsc_program_status != 2) {                                                                                              // If the button is held after DSC OFF, no more messages are sent until release.
          serial_log("Console: DSC OFF button held.", 2);
          send_dsc_mode(2);
        }
        dsc_off_button_debounce_timer = 0;
      }
    }      
  } else {                                                                                                                          // A quick tap re-enables everything.
    if (holding_dsc_off_console) {
      if (dsc_off_button_debounce_timer >= dsc_debounce_time_ms) {
        if (dsc_program_status != 0) {
          serial_log("Console: DSC button tapped.", 2);
          send_dsc_mode(0);
        }
        dsc_off_button_debounce_timer = 0;
      }
      holding_dsc_off_console = false;
    }
  }
}


#if RTC
void update_car_time_from_rtc(void) {
  if (rtc_valid) {                                                                                                                  // Make sure time in RTC is actually valid before forcing it.
    serial_log("Vehicle date/time not set. Setting from RTC.", 2);
    time_t t = now();
    uint8_t rtc_hours = hour(t);
    uint8_t rtc_minutes = minute(t);
    uint8_t rtc_seconds = second(t);
    uint8_t rtc_day = day(t);
    uint8_t rtc_month = month(t);
    uint16_t rtc_year = year(t);
    uint8_t date_time_can[] = {rtc_hours, rtc_minutes, rtc_seconds, 
                              rtc_day, uint8_t((rtc_month << 4) | 0xF), uint8_t(rtc_year & 0xFF), uint8_t(rtc_year >> 8), 0xF2};
    kcan_write_msg(make_msg_buf(0x39E, 8, date_time_can));
    kcan_write_msg(set_time_cc_off_buf);                                                                                            // Now that the time is fixed, cancel the CC.
  } else {
    serial_log("Teensy RTC invalid. Cannot set car's clock.", 1);
  }
}
#endif


void send_volume_request_periodic(void) {
  if (terminal_r) {
    if (idrive_txq.isEmpty() && initial_volume_set && !volume_reduced && volume_request_periodic_timer >= 3000) {
      if (diag_transmit) {
        kcan_write_msg(vol_request_buf);
        volume_request_periodic_timer = 0;
      }
    }
  }
}


void send_volume_request_door(void) {
  if (terminal_r) {
    if (diag_transmit) {
      if (!idrive_txq.isEmpty()) {                                                                                                  // If there are pending volume change actions we need to wait for them to complete.                                                                                           
        m = {vol_request_buf, millis() + 900};
        idrive_txq.push(&m);
      } else {
        if (volume_request_door_timer >= 300) {                                                                                     // Basic debounce to account for door not fully shut.
          kcan_write_msg(vol_request_buf);
          volume_request_door_timer = volume_request_periodic_timer = 0;
          serial_log("Requested volume from iDrive with door status change.", 2);
        }
      }
    }
  }
}


void evaluate_audio_volume(void) {
  if (k_msg.buf[0] == 0xF1 && k_msg.buf[3] == 0x24) {                                                                               // status_volumeaudio response.
    if (k_msg.buf[4] > 0) {
      uint8_t volume_change[] = {0x63, 4, 0x31, 0x23, 0, 0, 0, 0};
      if (!volume_reduced) {
        if (peristent_volume != k_msg.buf[4]) {
          peristent_volume = k_msg.buf[4];
          #if DEBUG_MODE
            sprintf(serial_debug_string, "Received new audio volume: 0x%X.", k_msg.buf[4]);
            serial_log(serial_debug_string, 3);
          #endif
        }
        if (pdc_tone_on) {                                                                                                          // If PDC beeps are active, volume change has no effect.
          return;
        }
        if (k_msg.buf[4] >= 5) {                                                                                                    // Don't reduce if already very low.
          if (left_door_open || right_door_open) {
            volume_change[4] = floor(k_msg.buf[4] * 0.75);                                                                          // Softer decrease.
            if (volume_change[4] > 0x33) {
              volume_change[4] = 0x33;
            }
            unsigned long time_now = millis();
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 200};
            idrive_txq.push(&m);
            volume_restore_offset = (k_msg.buf[4] % 2) == 0 ? 0 : 1;                                                                // Volumes adjusted from faceplate go up by 1 while MFL goes up by 2.
            volume_change[4] = floor(k_msg.buf[4] / 2);                                                                             // Reduce volume to 50%.
            if ((volume_change[4] + volume_restore_offset) > 0x33) {                                                                // Don't blow the speakers out if something went wrong...
              volume_change[4] = 0x33;
              volume_restore_offset = 0;
            }
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 600};
            idrive_txq.push(&m);
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 900};
            idrive_txq.push(&m);
            volume_reduced = true;
            volume_changed_to = volume_change[4];                                                                                   // Save this value to compare when door is closed back.
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Reducing audio volume with door open to: 0x%X.", volume_changed_to);
              serial_log(serial_debug_string, 3);
            #endif
          }
        }
      } else {
        peristent_volume = k_msg.buf[4] * 2 + volume_restore_offset;
        if (!left_door_open && !right_door_open) {
          if (k_msg.buf[4] == volume_changed_to) {
            volume_change[4] = floor(k_msg.buf[4] * 1.5);                                                                           // Softer increase.
            if (volume_change[4] > 0x33) {
              volume_change[4] = 0x33;
            }
            unsigned long time_now = millis();
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 200};
            idrive_txq.push(&m);
            volume_change[4] = k_msg.buf[4] * 2 + volume_restore_offset;
            if (volume_change[4] > 0x33) {
              volume_change[4] = 0x33;                                                                                              // Set a nanny in case the code goes wrong. 0x33 is pretty loud...
            }
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 600};
            idrive_txq.push(&m);
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 900};                                                            // Make sure the restore is received.
            idrive_txq.push(&m);
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Restoring audio volume with door closed. to: 0x%X.", volume_change[4]);
              serial_log(serial_debug_string, 3);
            #endif
          } else {
            peristent_volume = k_msg.buf[4];                                                                                        // User changed volume while door was opened.
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Volume changed by user while door was open to: 0x%X.", k_msg.buf[4]);
              serial_log(serial_debug_string, 3);
            #endif
          }
          volume_reduced = false;
        }
      }
    } else {
      #if RTC
        check_rtc_night_volume();
      #endif
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));
      initial_volume_set = true;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Sent saved initial volume (0x%X) to iDrive after receiving volume 0.", peristent_volume);     // 0 means that the vol knob wasn't used / initial job was not sent since iDrive boot.
        serial_log(serial_debug_string, 2);
      #endif
    }
  }
}


void disable_door_open_ignition_on_cc(void) {
  if (k_msg.buf[1] == 0x4F && k_msg.buf[2] == 1 && k_msg.buf[3] == 0x29) {
    kcan_write_msg(door_open_cc_off_buf);
  }
}


void check_idrive_queue(void) {
  if (!idrive_txq.isEmpty()) {
    if (diag_transmit) {
      idrive_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        if (vehicle_awake) {
          kcan_write_msg(delayed_tx.tx_msg);
          #if DOOR_VOLUME
            if (delayed_tx.tx_msg.buf[3] == 0x23) {
              serial_log("Sent volume change job to iDrive.", 2);
            }
          #endif
        }
        idrive_txq.drop();
      }
    } else {
      idrive_txq.flush();
    }
  }
}


void check_idrive_alive_monitor(void) {
  if (terminal_r) {
    if (idrive_alive_timer >= 4000) {                                                                                               // This message should be received every 2s.
      if (!idrive_died) {
        idrive_died = true;
        serial_log("iDrive booting/rebooting.", 2);
        initial_volume_set = false;
        vsw_initialized = false;
        asd_initialized = false;
      }
    } else {
      if (idrive_died) {                                                                                                            // It's back.
        idrive_died = false;
      }
    }
  }
}


void send_initial_volume(void) {
  if (k_msg.buf[7] >= 3) {                                                                                                          // 0x273 has been transmitted X times according to the counter.
    if (!initial_volume_set && diag_transmit) {
      #if RTC
        check_rtc_night_volume();
      #endif
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));                                                                  // Set iDrive volume to last volume before sleep. This must run before any set volumes.
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Sent saved sleep volume (0x%X) to iDrive after boot/reboot.", peristent_volume);
        serial_log(serial_debug_string, 2);
      #endif
      initial_volume_set = true;
    }
  }
}


void check_rtc_night_volume(void) {
  time_t t = now();                                                                                                                 // Reduce the startup volume at night time.
  uint8_t rtc_hours = hour(t);
  if (rtc_valid && (rtc_hours >= 21 || rtc_hours <= 6)) {
    peristent_volume = 8;
  }
}


void send_nivi_button_press(void) {
  ptcan_write_msg(nivi_button_pressed_buf);
  ptcan_write_msg(nivi_button_pressed_buf);
  ptcan_write_msg(nivi_button_released_buf);
  ptcan_write_msg(nivi_button_released_buf);
  serial_log("Sent NiVi button press.", 2);
}


void vsw_switch_input(uint8_t input) {
  uint8_t vsw_switch_position[] = {input, 0, 0, 0, 0, 0, 0, vsw_switch_counter};
  kcan_write_msg(make_msg_buf(0x2FB, 8, vsw_switch_position));
  vsw_current_input = input;
  vsw_switch_counter == 0xFE ? vsw_switch_counter = 0xF1 : vsw_switch_counter++;
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent VSW/%d (%s) request.", input, vsw_positions[input]);
    serial_log(serial_debug_string, 3);
  #endif
}


void initialize_vsw(void) {
  if (!vsw_initialized) {
    vsw_switch_input(1);
    vsw_initialized = true;
  }
}


// void request_idrive_menu(void) {
//   kcan_write_msg(idrive_menu_request_a_buf);
// }


// void evaluate_idrive_menu(void) {
  
// }


void initialize_asd(void) {
  if (!asd_initialized) {
    if (diag_transmit) {
      kcan_write_msg(mute_asd_buf);
      serial_log("Muted ASD on init.", 2);
      asd_initialized = true;
    }
  }
}


void evaluate_dr_seat_ckm(void) {
  if (k_msg.buf[0] == 0xFC) {
    if (!auto_seat_ckm[cas_key_number]) {
      serial_log("Automatic seat position CKM ON.", 2);
      auto_seat_ckm[cas_key_number] = true;
      eeprom_unsaved = true;
    }
  } else {
    if (auto_seat_ckm[cas_key_number]) {
      serial_log("Automatic seat position CKM OFF.", 2);
      auto_seat_ckm[cas_key_number] = false;
      eeprom_unsaved = true;
    }
  }
}


void evaluate_comfort_exit(void) {
  if (comfort_exit_ready && auto_seat_ckm[cas_key_number]) {
    kcan_write_msg(dr_seat_move_back_buf);
    comfort_exit_done = true;
    comfort_exit_ready = false;
    serial_log("Moved driver's seat back for comfort exit.", 2);
  } else {
    comfort_exit_ready = comfort_exit_done = false;
  }
}


void store_rvc_settings_cic(void) {
  if (k_msg.buf[0] == 0xE6) {                                                                                                       // Camera OFF.
    rvc_settings[0] = 0xE5;                                                                                                         // Store Camera ON, Two view OFF instead.
  } else {
    rvc_settings[0] = k_msg.buf[0];
  }

  for (uint8_t i = 1; i < 4; i++) {
    rvc_settings[i] = k_msg.buf[i];
  }

  if (!rvc_tow_view_by_driver && !rvc_tow_view_by_module && bitRead(k_msg.buf[0], 3) && pdc_bus_status == 0xA5) {                   // If the driver changed this setting, do not interfere during this cycle.
    rvc_tow_view_by_driver = true;
    serial_log("Driver changed RVC tow view manually.", 3);
  }
}


void store_rvc_settings_trsvc(void) {
  rvc_settings[1] = k_msg.buf[1];                                                                                                   // These values are the same as sent by CIC or TRSVC.
  rvc_settings[2] = k_msg.buf[2];
  if (k_msg.buf[3] == 1) {                                                                                                          // Parking lines OFF, Obstacle marking OFF, Tow view OFF.
    rvc_settings[3] = 0xE0;
  } else if (k_msg.buf[3] == 9) {                                                                                                   // Parking lines OFF, Obstacle marking ON, Tow view OFF.
    rvc_settings[3] = 0xE1;
  } else if (k_msg.buf[3] == 0x39) {                                                                                                // Parking lines ON, Obstacle marking ON, Tow view OFF.
    rvc_settings[3] = 0xE7;
  }
}


void evaluate_indicator_stalk(void) {
  if (k_msg.buf[0] == 1 || k_msg.buf[0] == 4) {
    full_indicator = false;
  } else if (k_msg.buf[0] == 2 || k_msg.buf[0] == 8) {
    if (!full_indicator) {
      full_indicator = true;
      serial_log("Indicator stalk pushed fully.", 3);
      undim_mirrors_with_indicators();
    }
  }
}


void evaluate_power_down_response(void) {
  if (power_down_requested) {
    if (diag_transmit) {
      if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10) {
        kcan_write_msg(power_down_cmd_b_buf);
      } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x21) {                                                                    // Ignore.
      } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x22) {                                                                    // Ignore.
      } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x23) {
        kcan_write_msg(power_down_cmd_c_buf);
      } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 3 && k_msg.buf[2] == 0x71 && k_msg.buf[3] == 5) {
        serial_log("Power-down command sent successfully. Car will assume deep sleep in ~15s.", 0);
        power_down_requested = false;
      } else {
        power_down_requested = false;
        serial_log("Power-down command aborted due to error.", 0);
      }
    } else {
      power_down_requested = false;
      serial_log("Power-down command aborted due to OBD tool presence.", 0);
    }
  }
}


void evaluate_wiper_stalk_status(void) {
  if (terminal_r) {
    if (pt_msg.buf[0] == 0x10) {
      #if WIPE_AFTER_WASH
        if (wash_message_counter >= 2 || wipe_scheduled) {
          serial_log("Washing cycle started.", 2);
          wiper_txq.flush();
          uint8_t single_wipe[] = {8, intermittent_setting_can};
          m = {make_msg_buf(0x2A6, 2, single_wipe), millis() + 7000};
          wiper_txq.push(&m);
          wipe_scheduled = true;                                                                                                    // If a quick pull is detected after the main one, re-schedule the wipe.
          wash_message_counter = 0;
        } else {
          wash_message_counter++;
        }
      #endif
      #if INTERMITTENT_WIPERS
        stalk_down_message_counter = 0;
      #endif
    } 

    else if (pt_msg.buf[0] == 0) {                                                                                                  // Wiping completely OFF.
      #if WIPE_AFTER_WASH
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        stalk_down_message_counter = 0;
      #endif
    }
    
    else if (pt_msg.buf[0] == 1) {                                                                                                  // AUTO button pressed.
      #if WIPE_AFTER_WASH
        abort_wipe_after_wash();
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        if (intermittent_wipe_active) {
          disable_intermittent_wipers();
        }
        stalk_down_message_counter = 0;
      #endif
    }

    else if (pt_msg.buf[0] == 2 || pt_msg.buf[0] == 3) {                                                                            // Stalk pushed up once / twice.
      #if WIPE_AFTER_WASH
        abort_wipe_after_wash();
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        if (intermittent_wipe_active) {
          disable_intermittent_wipers();
        }
        stalk_down_message_counter = 0;
      #endif
    }

    else if (pt_msg.buf[0] == 8) {                                                                                                  // Stalk pushed down. 9 = stalk pushed down with AUTO ON.
      #if WIPE_AFTER_WASH
        abort_wipe_after_wash();
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        if (millis() - stalk_down_last_press_time >= 1100) {                                                                        // If more than 1100ms passed, stalk must have been released.
          stalk_down_message_counter = 0;
        }

        stalk_down_last_press_time = millis();
        stalk_down_message_counter++;

        intermittent_wipe_timer = 0;

        if (stalk_down_message_counter >= 4) {
          if (!intermittent_wipe_active) {
            activate_intermittent_wipers();
            intermittent_wipe_timer = intermittent_intervals[intermittent_setting];
          } else {
            disable_intermittent_wipers();
          }
          stalk_down_message_counter = 0;
        }
      #endif
    }

    #if INTERMITTENT_WIPERS || WIPE_AFTER_WASH
      if (pt_msg.buf[1] != intermittent_setting_can) {                                                                              // Position of the speed wheel changed.
        intermittent_setting_can = pt_msg.buf[1];
        uint8_t new_intermittent_setting = 0;
        new_intermittent_setting = intermittent_setting_can - 0xF8;
        #if DEBUG_MODE && INTERMITTENT_WIPERS
          if (intermittent_wipe_active) {
            sprintf(serial_debug_string, "New wiper speed setting %d%s.", new_intermittent_setting + 1,
                    !vehicle_moving ? " [not moving]" : "");
            serial_log(serial_debug_string, 3);
          }
        #endif
        stalk_down_message_counter = 0;
        if (new_intermittent_setting > intermittent_setting) {                                                                      // If wheel moved up, wipe immediately. Else, wait until next interval.
          intermittent_wipe_timer = intermittent_intervals[intermittent_setting] + 100;
          if (!vehicle_moving) {
            intermittent_wipe_timer += intermittent_intervals_offset_stopped[intermittent_setting];
          }
        } else {
          intermittent_wipe_timer = 0;
        }
        intermittent_setting = new_intermittent_setting;
      }
    #endif
  }
}
