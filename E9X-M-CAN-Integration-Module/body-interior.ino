// General body functions dealing with interior electronics go here.


void evaluate_terminal_clutch_keyno_status(void) {
  vehicle_awake_timer = 0;
  if (!vehicle_awake) {
    vehicle_awake = true;    
    toggle_transceiver_standby();                                                                                                   // Re-activate the transceivers.                                                                                         
    serial_log("Vehicle Awake.");
    vehicle_awakened_time = 0;
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
        serial_log("Clutch pedal pressed.");
      } else {
        clutch_pressed = false;
        serial_log("Clutch pedal released.");
      }
    }
  }
  #endif

  #if CKM || EDC_CKM_FIX
    check_key_changed();
  #endif

  #if F_ZBE_WAKE || F_VSW01 || F_NIVI                                                                                               // Translate 0x130 to 0x12F for F-series modules.
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

    f_terminal_status_buf = make_msg_buf(0x12F, 8, f_terminal_status);
    #if F_ZBE_WAKE || F_VSW01
      kcan_write_msg(f_terminal_status_buf);
    #endif
    #if F_NIVI
      ptcan_write_msg(f_terminal_status_buf);
    #endif
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
    scale_mcu_speed();
    #if USB_DISABLE
      activate_usb();                                                                                                               // If this fails to run, the program button will need to be pressed to recover.
    #endif
    serial_log("Ignition ON.");

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
    reset_runtime_variables();
    scale_mcu_speed();                                                                                                              // Now that the ignition is OFF, underclock the MCU
    #if DEBUG_MODE
      serial_log("Ignition OFF. Reset values.");
    #endif
  }

  if (terminal_r && !terminal_r_) {
    serial_log("Terminal R ON.");
    #if RTC
      if (!rtc_valid) {
        kcan_write_msg(set_time_cc_buf);                                                                                            // Warn that the time needs to be updated by the user.
      }
    #endif
  } else if (!terminal_r && terminal_r_) {
    serial_log("Terminal R OFF.");
  }
}


#if AUTO_SEAT_HEATING
void evaluate_seat_heating_status(void) {
  if (k_msg.id == 0x232) {
    driver_seat_heating_status = !k_msg.buf[0] ? false : true;
    if (!driver_seat_heating_status) {                                                                                              // Check if seat heating is already ON.
      if (!driver_sent_seat_heating_request && (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD)) {
        send_seat_heating_request_dr();
      }
    } else {
      driver_sent_seat_heating_request = true;                                                                                      // Seat heating already ON. No need to request anymore.
    }
  } 
  #if AUTO_SEAT_HEATING_PASS
  else {                                                                                                                            // Passenger's seat heating status message is only sent with ignition ON.
    passenger_seat_heating_status = !k_msg.buf[0] ? false : true;
    if (!passenger_sent_seat_heating_request && (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD)) {
      if (passenger_seat_status == 9) {
        send_seat_heating_request_pas();
      }
    }
  }
  #endif
}


void send_seat_heating_request_dr(void) {
  time_now = millis();
  kcan_write_msg(seat_heating_button_pressed_dr_buf);
  driver_sent_seat_heating_request = true;
  m = {seat_heating_button_released_dr_buf, time_now + 100};
  seat_heating_dr_txq.push(&m);
  m = {seat_heating_button_released_dr_buf, time_now + 250};
  seat_heating_dr_txq.push(&m);
  m = {seat_heating_button_released_dr_buf, time_now + 400};
  seat_heating_dr_txq.push(&m);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent driver's seat heating request at ambient %.1fC, treshold %.1fC.", 
          ambient_temperature_real, AUTO_SEAT_HEATING_TRESHOLD);
    serial_log(serial_debug_string);
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
  #if AUTO_SEAT_HEATING_PASS
    if (!seat_heating_pas_txq.isEmpty()) {
      seat_heating_pas_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        seat_heating_pas_txq.drop();
      }
    }
  #endif
}
#endif


#if AUTO_SEAT_HEATING_PASS
void evaluate_passenger_seat_status(void) {
  passenger_seat_status = k_msg.buf[1];
  if (ignition) {
    if (!passenger_seat_heating_status) {                                                                                           // Check if seat heating is already ON.
      //This will be ignored if already ON and cycling ignition. Press message will be ignored by IHK anyway.
      if (!passenger_sent_seat_heating_request && (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD)) { 
        if (passenger_seat_status == 9) {                                                                                           // Passenger sitting and their seatbelt is ON
          send_seat_heating_request_pas();                                                                                          // Execute heating request here so we don't have to wait 15s for the next 0x22A.
        }
      }
    } else {
      passenger_sent_seat_heating_request = true;                                                                                   // Seat heating already ON. No need to request anymore.
    }
  }
}


void send_seat_heating_request_pas(void) {
  time_now = millis();
  kcan_write_msg(seat_heating_button_pressed_pas_buf);
  passenger_sent_seat_heating_request = true;
  m = {seat_heating_button_released_pas_buf, time_now + 100};
  seat_heating_pas_txq.push(&m);
  m = {seat_heating_button_released_pas_buf, time_now + 250};
  seat_heating_pas_txq.push(&m);
  m = {seat_heating_button_released_pas_buf, time_now + 400};
  seat_heating_pas_txq.push(&m);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent passenger's seat heating request at ambient %.1fC, treshold %.1fC.", 
          ambient_temperature_real, AUTO_SEAT_HEATING_TRESHOLD);
    serial_log(serial_debug_string);
  #endif
}
#endif


#if AUTO_STEERING_HEATER
void evaluate_steering_heating_request(void) {
  if (engine_running && engine_runtime >= AUTO_HEATING_START_DELAY) {                                                               // Wait for supply voltage to stabilize.
    if (!sent_steering_heating_request) {
      if (ambient_temperature_real <= AUTO_SEAT_HEATING_TRESHOLD) {
        digitalWrite(STEERING_HEATER_SWITCH_PIN, HIGH);
        serial_log("Activated steering wheel heating.");
        sent_steering_heating_request = transistor_active = true;
        transistor_active_timer = 0;
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
#endif


#if F_ZBE_WAKE || F_VSW01 || F_NIVI
void send_f_wakeup(void) {
  #if F_ZBE_WAKE || F_VSW01
    kcan_write_msg(f_kombi_network_mgmt_buf);
  #endif
  #if F_NIVI
    ptcan_write_msg(f_kombi_network_mgmt_buf);
  #endif
    //serial_log("Sent FXX wake-up message.");
}


void send_f_vehicle_mode(void) {
  if (f_vehicle_mode_timer >= 5000) {
    if (battery_voltage < 12.00 && battery_voltage > 11.60) {
      f_vehicle_mode[2] = 0xF1;                                                                                                     // Energy OK.
    } else if (battery_voltage < 11.60 && battery_voltage > 11.20) {
      f_vehicle_mode[2] = 0xF2;                                                                                                     // Energy shortage.
    } else if (battery_voltage < 11.20) {
      f_vehicle_mode[2] = 0xF3;                                                                                                     // Energy severe shortage.
    } else {
      f_vehicle_mode[2] = 0xF0;                                                                                                     // Energy good.
    }
    f_vehicle_mode_buf = make_msg_buf(0x3A0, 8, f_vehicle_mode);
    #if F_ZBE_WAKE || F_VSW01
      kcan_write_msg(f_vehicle_mode_buf);
    #endif
    #if F_NIVI
      ptcan_write_msg(f_vehicle_mode_buf);
    #endif

    f_vehicle_mode_timer = 0;
  }
}
#endif


#if F_ZBE_WAKE
void send_zbe_acknowledge(void) {
  zbe_response[2] = k_msg.buf[7];
  kcan_write_msg(make_msg_buf(0x277, 4, zbe_response));
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent ZBE response to CIC with counter: 0x%X", k_msg.buf[7]);
    serial_log(serial_debug_string);
  #endif
}
#endif


void evaluate_battery_voltage(void) {
  battery_voltage = (((pt_msg.buf[1] - 240 ) * 256.0) + pt_msg.buf[0]) / 68.0;
}


void check_console_buttons(void) {
  #if ANTI_THEFT_SEQ
  if (anti_theft_released) {
    if (!digitalRead(POWER_BUTTON_PIN) && !digitalRead(DSC_BUTTON_PIN)) {
      if (!holding_both_console) {
        both_console_buttons_timer = 0;
        holding_both_console = true;
      } else {
        if (both_console_buttons_timer >= 10000) {                                                                                  // Hold both buttons for more than 10s.
          if (diag_transmit) {
            kcan_write_msg(cc_gong_buf);                                                                                            // Acknowledge anti-theft persist ON-OFF
          }
          anti_theft_persist = !anti_theft_persist;
          EEPROM.update(15, anti_theft_persist);
          update_eeprom_checksum();
          #if DEBUG_MODE
            sprintf(serial_debug_string, "Anti theft persistently: %s.", anti_theft_persist ? "ON" : "OFF");
            serial_log(serial_debug_string);
          #endif
          both_console_buttons_timer = 0;                                                                                           // Reset to prevent multiple activations.
        }
      }
      return;
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
          serial_log("Console: POWER mode ON.");
        } else {
          mdrive_power_active = false;                                                                                              // If POWER button was pressed while MDrive POWER is active, disable POWER.
          serial_log("Deactivated MDrive POWER with console button press.");
        }
      } else {
        serial_log("Console: POWER mode OFF.");
        console_power_mode = false;
        if (mdrive_power_active) {
          mdrive_power_active = false;                                                                                              // If POWER button was pressed while MDrive POWER is active, disable POWER.
          serial_log("Deactivated MDrive POWER with console button press.");
        }
      }
      send_power_mode();
    }
  } 
  
  if (!digitalRead(DSC_BUTTON_PIN)) {
    if (!holding_dsc_off_console) {
      holding_dsc_off_console = true;
      dsc_off_button_hold_timer = 0;
    } else {
      if (dsc_off_button_hold_timer >= dsc_hold_time_ms) {                                                                          // DSC OFF sequence should only be sent after user holds button for a configured time
        if (dsc_off_button_debounce_timer >= dsc_debounce_time_ms) {
          if (dsc_program_status != 2) {
            serial_log("Console: DSC OFF button held.");
            send_dsc_mode(2);
            dsc_off_button_debounce_timer = 0;
            holding_dsc_off_console = false;
          }
        }
      }
    }      
  } else {                                                                                                                          // A quick tap re-enables everything
    if (holding_dsc_off_console) {
      if (dsc_off_button_debounce_timer >= dsc_debounce_time_ms) {
        if (dsc_program_status != 0) {
          serial_log("Console: DSC button tapped.");
          dsc_off_button_debounce_timer = 0;
          send_dsc_mode(0);
        }
      }
      holding_dsc_off_console = false;
    }
  }
}


#if RTC
void update_car_time_from_rtc(void) {
  if (rtc_valid) {                                                                                                                  // Make sure time in RTC is actually valid before forcing it.
    serial_log("Vehicle date/time not set. Setting from RTC.");
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
    kcan_write_msg(set_time_cc_off_buf);                                                                                            // Now that the time is fixed, cancel the CC>
  } else {
    serial_log("Teensy RTC invalid. Cannot set car's clock.");
  }
}
#endif


#if DOOR_VOLUME
void send_volume_request_periodic(void) {
  if (volume_request_timer >= 3000) {
    if (diag_transmit) {
      kcan_write_msg(vol_request_buf);
    }
    volume_request_timer = 0;
  }
}


void send_volume_request_door(void) {
  if (diag_transmit) {
    m = {vol_request_buf, millis() + 10};
    idrive_txq.push(&m);
    serial_log("Requesting volume from iDrive with door status change.");
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
            serial_log(serial_debug_string);
          #endif
        }
        if (left_door_open || right_door_open) {
          volume_change[4] = floor(k_msg.buf[4] * 0.75);                                                                            // Softer decrease.
          if (volume_change[4] > 0x33) {
            volume_change[4] = 0x33;
          }
          time_now = millis();
          m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 200};
          idrive_txq.push(&m);
          volume_restore_offset = (k_msg.buf[4] % 2) == 0 ? 0 : 1;                                                                  // Volumes adjusted from faceplate go up by 1 while MFL goes up by 2.
          volume_change[4] = floor(k_msg.buf[4] / 2);                                                                               // Reduce volume to 50%.
          if ((volume_change[4] + volume_restore_offset) > 0x33) {                                                                  // Don't blow the speakers out if something went wrong...
            volume_change[4] = 0x33;
            volume_restore_offset = 0;
          }
          m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 600};
          idrive_txq.push(&m);
          m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 900};
          idrive_txq.push(&m);
          volume_reduced = true;
          volume_changed_to = volume_change[4];                                                                                     // Save this value to compare when door is closed back.
          #if DEBUG_MODE
            sprintf(serial_debug_string, "Reducing audio volume with door open to: 0x%X.", volume_changed_to);
            serial_log(serial_debug_string);
          #endif
        }
      } else {
        peristent_volume = k_msg.buf[4] * 2 + volume_restore_offset;
        if (!left_door_open && !right_door_open) {
          if (k_msg.buf[4] == volume_changed_to) {
            volume_change[4] = floor(k_msg.buf[4] * 1.5);                                                                           // Softer increase.
            if (volume_change[4] > 0x33) {
              volume_change[4] = 0x33;
            }
            time_now = millis();
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
              serial_log(serial_debug_string);
            #endif
          } else {
            peristent_volume = k_msg.buf[4];                                                                                        // User changed volume while door was opened.
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Volume changed by user while door was open to: 0x%X.", k_msg.buf[4]);
              serial_log(serial_debug_string);
            #endif
          }
          volume_reduced = false;
        }
      }
    } else {
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));
      initial_volume_set = true;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Sent saved initial volume (0x%X) to iDrive after receiving volume 0.", peristent_volume);     // 0 means that the vol knob wasn't used / initial job was not sent since iDrive boot.
        serial_log(serial_debug_string);
      #endif
    }
  }
}


void disable_door_ignition_cc(void) {
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
          if (delayed_tx.tx_msg.buf[3] == 0x23) {
            serial_log("Sent volume change job to iDrive.");
          }
        }
        idrive_txq.drop();
      }
    } else {
      idrive_txq.flush();
    }
  }
}
#endif


#if CKM || DOOR_VOLUME || REVERSE_BEEP || F_VSW01
void check_idrive_alive_monitor(void) {
  if (terminal_r) {
    if (idrive_alive_timer >= 4000) {                                                                                               // This message should be received every 2s.
      if (!idrive_died) {
        idrive_died = true;
        serial_log("iDrive booting/rebooting.");
        #if DOOR_VOLUME
          initial_volume_set = false;
        #endif
        #if F_VSW01
          vsw_initialized = false;
        #endif
      }
    } else {
      if (idrive_died) {                                                                                                            // It's back.
        idrive_died = false;
        #if F_VSW01
          vsw_initialized = true;
        #endif
      }
    }
  }
}


void update_idrive_alive_timer(void) {
  idrive_alive_timer = 0;

  #if DOOR_VOLUME
  if (k_msg.buf[7] >= 3) {                                                                                                          // 0x273 has been transmitted X times according to the counter.
    if (!initial_volume_set && diag_transmit) {
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));                                                                  // Set iDrive volume to last volume before sleep. This must run before any set volumes.
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Sent saved sleep volume (0x%X) to iDrive after boot/reboot.", peristent_volume);
        serial_log(serial_debug_string);
      #endif
      initial_volume_set = true;
    }
  }
  #endif

  #if F_VSW01
    if (!vsw_initialized) {
      vsw_switch_input(1);
      vsw_initialized = true;
    }
  #endif
}
#endif


#if F_VSW01
void vsw_switch_input(uint8_t input) {
  kcan_write_msg(vsw_switch_buf[input]);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent VSW/%d request.", input);
    serial_log(serial_debug_string);
  #endif
}


// void request_idrive_menu(void) {
//   kcan_write_msg(idrive_menu_request_buf);
// }


// void evaluate_idrive_menu(void) {
  
// }
#endif
