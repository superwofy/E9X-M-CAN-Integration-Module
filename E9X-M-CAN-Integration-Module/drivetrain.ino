// Engine/driveline functions go here.


void evaluate_engine_rpm(void) {
  RPM = ((uint16_t)k_msg.buf[5] << 8) | (uint16_t)k_msg.buf[4];
  if (RPM > 2000) {
    if (!engine_running) {
      engine_runtime = 0;
      engine_running = true;
      #if CONTROL_SHIFTLIGHTS
        shiftlight_startup_animation();                                                                                             // Show off shift light segments during engine startup (>500rpm).
      #endif
      #if NEEDLE_SWEEP
        needle_sweep_animation();
      #endif
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_action_timer = 0;                                                                                              // Start tracking the exhaust flap.
      #endif
      serial_log("Engine started.");
      #if IMMOBILIZER_SEQ_ALARM
        if (!immobilizer_released) {
          alarm_after_engine_stall = true;
          serial_log("Immobilizer still active. Alarm will sound after stall.");      
        }
      #endif
    }
  } else if (RPM < 200) {                                                                                                           // Less than 50 RPM. Engine stalled or was stopped.
    if (engine_running) {
      engine_running = false;
      serial_log("Engine stopped.");
      #if IMMOBILIZER_SEQ_ALARM
        trip_alarm_after_stall();
      #endif
    }
  }
}


void toggle_mdrive_message_active(void) {
  if (mdrive_status) {                                                                                                              // Turn OFF MDrive.
    serial_log("Status MDrive OFF.");
    mdrive_message[1] -= 1;                                                                                                         // Decrement bytes 1 (6MT, DSC mode) and 4 (SVT) to deactivate.
    mdrive_message[4] -= 0x10;
    mdrive_status = mdrive_power_active = false;
    #if FRM_AHL_MODE
      kcan_write_msg(frm_ckm_ahl_komfort_buf);
      serial_log("Set AHL back to comfort mode.");
    #endif
    #if ASD
      kcan_write_msg(mute_asd_buf);
      serial_log("Muted ASD.");
    #endif
    if (mdrive_power == 0x30) {
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = false;
      #endif
    } else if (mdrive_power == 0x10) {
      console_power_mode = restore_console_power_mode;
    }                                                                                                                               // Else, POWER unchanged
  } else {                                                                                                                          // Turn ON MDrive.
    serial_log("Status MDrive ON.");
    if (mdrive_power == 0x20) {                                                                                                     // POWER in Sport.
      mdrive_power_active = true;
    } else if (mdrive_power == 0x30) {                                                                                              // POWER Sport+.
      #if ASD
        kcan_write_msg(demute_asd_buf);
        serial_log("De-muted ASD.");
      #endif
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = true;                                                                                                  // Exhaust flap always open in Sport+
      #endif
      mdrive_power_active = true;
    } else if (mdrive_power == 0x10) {
      restore_console_power_mode = console_power_mode;                                                                              // We'll need to return to its original state when MDrive is turned OFF.
      console_power_mode = false;                                                                                                   // Turn OFF POWER from console too.
    }                                                                                                                               // Else, POWER unchanged.

    mdrive_message[1] += 1;
    mdrive_message[4] += 0x10;
    mdrive_status = true;

    #if FRM_AHL_MODE
      kcan_write_msg(frm_ckm_ahl_sport_buf);
      serial_log("Set AHL to sport mode.");
    #endif
  }
}


void toggle_mdrive_dsc_mode(void) {
  if (mdrive_status) {
    if (mdrive_dsc == 7) {                                                                                                          // DSC OFF requested.
      send_dsc_mode(2);
    } else if (mdrive_dsc == 0x13) {                                                                                                // DSC MDM (DTC in non-M) requested.
      send_dsc_mode(1);
    } else if (mdrive_dsc == 0xB) {                                                                                                 // DSC ON requested.
      send_dsc_mode(0);
    }
  } else {
    if (mdrive_dsc == 0x13 || mdrive_dsc == 7) {                                                                                    // If MDrive was set to change DSC, restore back to DSC ON.
      if (dsc_program_status != 0) {
        send_dsc_mode(0);
      }
    }
  }
}


void evaluate_m_mfl_button_press(void) {
  if (pt_msg.buf[1] == 0x4C) {                                                                                                      // M button is pressed.
    if (!ignore_m_press) {
      ignore_m_press = true;                                                                                                        // Ignore further pressed messages until the button is released.
      #if IMMOBILIZER_SEQ
      if (immobilizer_released && ignition) {                                                                                       // Disable normal M button function when used for immobilizer.
      #endif
        toggle_mdrive_message_active();
        send_mdrive_message();
        toggle_mdrive_dsc_mode();
      #if IMMOBILIZER_SEQ
      }
      #endif
    }
    if (m_mfl_held_count > 10 && !ignore_m_hold) {                                                                                  // Each count is about 100ms
      #if IMMOBILIZER_SEQ
      if (immobilizer_released) {
      #endif
      if (ignition) {
        show_mdrive_settings_screen();
      }
      #if IMMOBILIZER_SEQ
      }
      #endif
    } else {
      m_mfl_held_count++;
    }
  } else if (pt_msg.buf[1] == 0xC && pt_msg.buf[0] == 0xC0 && ignore_m_press) {                                                     // Button is released.
    ignore_m_press = ignore_m_hold = false;
    m_mfl_held_count = 0;

    #if IMMOBILIZER_SEQ
    uint8_t activate_release_counter = IMMOBILIZER_SEQ_NUMBER - 1;
    if (!immobilizer_released) {
      #if IMMOBILIZER_SEQ_ALARM
        activate_release_counter = alarm_active ? IMMOBILIZER_SEQ_ALARM_NUMBER - 1 : IMMOBILIZER_SEQ_NUMBER - 1;
      #endif

      if (immobilizer_pressed_release_count < activate_release_counter) {
        immobilizer_pressed_release_count++;
      } else {
        release_immobilizer();
      }
    } else {                                                                                                                        // Allow re-activation before sleep mode.
      if (immobilizer_activate_release_timer >= 3000) {
        if (!terminal_r && immobilizer_persist) {
          if (immobilizer_pressed_activate_count < activate_release_counter) {
            immobilizer_pressed_activate_count++;
          } else {
            activate_immobilizer();
            play_cc_gong();
          }
        }
      }
    }
    #endif
  }
}


void show_mdrive_settings_screen(void) {
  if (diag_transmit) {
    #if IMMOBILIZER_SEQ
    if (immobilizer_released) {
    #endif
      serial_log("Steering wheel M button held. Showing settings screen.");
      kcan_write_msg(idrive_mdrive_settings_a_buf);                                                                                 // Send steuern_menu job to iDrive.
      kcan_write_msg(idrive_mdrive_settings_b_buf);
    #if IMMOBILIZER_SEQ
    }
    #endif
    ignore_m_hold = true;
  }
}


void send_mdrive_message(void) {
  mdrive_message[0] += 10;
  if (mdrive_message[0] > 0xEF) {                                                                                                   // Alive(first half of byte) must be between 0..E.
    mdrive_message[0] = 0;
  }
  can_checksum_update(0x399, 6, mdrive_message);                                                                                    // Recalculate checksum. No module seems to check this - MDSC?
  ptcan_write_msg(make_msg_buf(0x399, 6, mdrive_message));                                                                          // Send to PT-CAN like the DME would. EDC will receive. KOMBI will receive on KCAN through JBE.
  mdrive_message_timer = 0;                                                                   
}


void send_mdrive_alive_message(uint16_t interval) {
  if (terminal_r) {
    if (mdrive_message_timer >= interval) {                                                                                         // Time MDrive alive message outside of CAN loops. Original cycle time is 10s (idle).                                                                     
      if (ignition) {
        serial_log("Sending Ignition ON MDrive alive message.");
      } else {
        serial_log("Sending Vehicle Awake MDrive alive message.");
      }
      send_mdrive_message();
    }
  }
}


void update_mdrive_message_settings(void) {
  if (k_msg.buf[4] == 0xEC || k_msg.buf[4] == 0xF4 || k_msg.buf[4] == 0xE4) {                                                       // Reset requested.
    reset_mdrive_settings();
  } else if ((k_msg.buf[4] == 0xE0 || k_msg.buf[4] == 0xE1)) {                                                                      // Ignore E0/E1 (Invalid).
  } else {
    //Decode settings
    mdrive_dsc = k_msg.buf[0];                                                                                                      // 3 unchanged, 7 OFF, 0x13 MDM, 0xB ON.
    mdrive_power = k_msg.buf[1];                                                                                                    // 0 unchanged, 0x10 normal, 0x20 sport, 0x30 sport+.
    mdrive_edc = k_msg.buf[2];                                                                                                      // 0x20(Unchanged), 0x21(Comfort) 0x22(Normal) 0x2A(Sport).
    mdrive_svt = k_msg.buf[4];                                                                                                      // 0xE9 Normal, 0xF1 Sport, 0xEC/0xF4/0xE4 Reset. E0/E1-invalid?
    
    mdrive_message[1] = mdrive_dsc - 2 + mdrive_status;                                                                             // DSC message is 2 less than iDrive setting. 1 is added if MDrive is ON.
    mdrive_message[2] = mdrive_power;                                                                                               // Copy POWER as is.
    mdrive_message[3] = mdrive_edc;                                                                                                 // Copy EDC as is.
    if (mdrive_svt == 0xE9) {
      if (mdrive_status == 0) {
        mdrive_message[4] = 0x41;                                                                                                   // SVT normal, MDrive OFF.
      } else {
        mdrive_message[4] = 0x51;                                                                                                   // SVT normal, MDrive ON.
      }
    } else if (mdrive_svt == 0xF1) {
      if (mdrive_status == 0) {
        mdrive_message[4] = 0x81;                                                                                                   // SVT sport, MDrive OFF.
      } else {
        mdrive_message[4] = 0x91;                                                                                                   // SVT sport, MDrive ON.
      }
    }
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received iDrive settings: DSC 0x%X POWER 0x%X EDC 0x%X SVT 0x%X.", 
          mdrive_dsc, mdrive_power, mdrive_edc, mdrive_svt);
      serial_log(serial_debug_string);
    #endif
  }
  send_mdrive_message();
}


void reset_mdrive_settings(void) {
  mdrive_dsc = 3;                                                                                                                   // Unchanged
  mdrive_message[1] = 1;
  mdrive_power = 0;                                                                                                                 // Unchanged
  mdrive_message[2] = mdrive_power;
  mdrive_edc = 0x20;                                                                                                                // Unchanged
  mdrive_message[3] = mdrive_edc;
  mdrive_svt = 0xE9;                                                                                                                // Normal
  mdrive_message[4] = 0x41;
  dme_ckm[cas_key_number][0] = 0xF1;                                                                                                // Normal
  #if EDC_CKM_FIX
    edc_ckm[cas_key_number] = 0xF1;                                                                                                 // Comfort
  #endif
  serial_log("Reset MDrive settings.");
}


// Written by amg6975
// https://www.spoolstreet.com/threads/MDrive-and-mdm-in-non-m-cars.7155/post-107037
void can_checksum_update(uint16_t canid, uint8_t len,  uint8_t *message) {
  message[0] &= 0xF0;                                                                                                               // Remove checksum from byte.
  // Add up all bytes and the CAN ID
  uint16_t checksum = canid;
  for (uint8_t i = 0; i < len; i++) {
    checksum += message[i];
  }                                 
  checksum = (checksum & 0x00FF) + (checksum >> 8); //add upper and lower Bytes
  checksum &= 0x00FF; //throw away anything in upper Byte
  checksum = (checksum & 0x000F) + (checksum >> 4); //add first and second nibble
  checksum &= 0x000F; //throw away anything in upper nibble

  message[0] += checksum;                                                                                                           // Add the checksum back to Byte0.
}


void send_power_mode(void) {
  power_mode_only_dme_veh_mode[0] += 0x10;                                                                                          // Increase alive counter.
  if (power_mode_only_dme_veh_mode[0] > 0xEF) {                                                                                     // Alive(first half of byte) must be between 0..E.
    power_mode_only_dme_veh_mode[0] = 0;
  }

  if (console_power_mode || mdrive_power_active) {                                                                                  // Activate sport throttle mapping if POWER from console ON or Sport/Sport+ selected in MDrive (active).
    power_mode_only_dme_veh_mode[1] = 0xF2;                                                                                         // Sport
    digitalWrite(POWER_LED_PIN, HIGH);
    power_led_delayed_off_action = false;
  } else {
    power_mode_only_dme_veh_mode[1] = 0xF1;                                                                                         // Normal
    power_led_delayed_off_action = true;
    power_led_delayed_off_action_time = millis() + 105;
  }

  can_checksum_update(DME_FAKE_VEH_MODE_CANID, 2, power_mode_only_dme_veh_mode);
  ptcan_write_msg(make_msg_buf(DME_FAKE_VEH_MODE_CANID, 2, power_mode_only_dme_veh_mode));
}


void send_dme_power_ckm(void) {
  kcan_write_msg(make_msg_buf(0x3A9, 2, dme_ckm[cas_key_number]));                                                                  // This is sent by the DME to populate the M Key iDrive section
  serial_log("Sent DME POWER CKM.");
}


void update_dme_power_ckm(void) {
  dme_ckm[cas_key_number][0] = k_msg.buf[0];
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Received new POWER CKM setting: %s for key number %d", 
            k_msg.buf[0] == 0xF1 ? "Normal" : "Sport", cas_key_number);
    serial_log(serial_debug_string);
  #endif
  send_dme_power_ckm();                                                                                                             // Acknowledge settings received from iDrive;
}


void update_edc_ckm(void) {
  edc_ckm[cas_key_number] = k_msg.buf[0];
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Received new EDC CKM setting: %s for key %d.", k_msg.buf[0] == 0xF1 ? "Comfort" : 
                                  k_msg.buf[0] == 0xF2 ? "Normal" : "Sport", cas_key_number);
    serial_log(serial_debug_string);
  #endif
}


void evaluate_edc_ckm_mismatch(void) {
  if (edc_mismatch_check_counter < 2) {
    if (pt_msg.buf[1] != edc_ckm[cas_key_number]) {
      serial_log("EDC EEPROM CKM setting match the current value. Correcting.");
      uint8_t edc_state = pt_msg.buf[1] == 0xFA ? 3 : pt_msg.buf[1] - 0xF0;                                                           // Normalize these values for easier comparison.
      uint8_t edc_memory = edc_ckm[cas_key_number] == 0xFA ? 3 : edc_ckm[cas_key_number] - 0xF0;
      if ((edc_memory == 1 && edc_state == 2) || (edc_memory == 2 && edc_state == 3) || (edc_memory == 3 && edc_state == 1)) {
        time_now = millis();
        kcan_write_msg(edc_button_press_buf);
        m = {edc_button_press_buf, time_now + 1200};
        edc_ckm_txq.push(&m);
      } else {
        kcan_write_msg(edc_button_press_buf);
      }
    }
    edc_mismatch_check_counter++;
  }
}


void check_edc_ckm_queue(void) {
  if (!edc_ckm_txq.isEmpty()) {
    edc_ckm_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      edc_ckm_txq.drop();
    }
  }
}


void check_immobilizer_status(void) {
  if (vehicle_awakened_time >= 2000) {                                                                                              // Delay ensures that time passed after Teensy (re)started to receive messages.
    if (!immobilizer_released) {
      uint16_t theft_max_speed = 20;
      if (speed_mph) {
        theft_max_speed = 12;
      }

      #if IMMOBILIZER_SEQ_ALARM
        if (vehicle_awakened_time >= 10000) {                                                                                       // Delay so we don't interfere with normal DWA behavior indicating alarm faults. Also when doing 30G reset.
          if (!terminal_r && !alarm_led && !frm_consumer_shutdown && !lock_led) {
            kcan_write_msg(alarm_led_on_buf);                                                                                       // Visual indicator when driver just got in and did not activate anything / car woke. Timeout 120s.
            alarm_led = true;                                                                                                       // Sending this multiple times keeps the car awake.
            serial_log("Sent DWA LED ON with Terminal R off.");
            led_message_counter = 60;                                                                                               // Make sure we're ready once Terminal R cycles.
          }
        }
      #endif

      // This could be temporarily bypassed if the car is started and driven very very quickly?
      // It will reactivate once indicated_speed <= theft_max_speed. Think Speed (1994)...
      if (!vehicle_moving || indicated_speed <= theft_max_speed) {                                                                  // Make sure we don't cut this OFF while the car is in (quick) motion!!!
        if (immobilizer_timer >= immobilizer_send_interval) {
          if (terminal_r) {
            if (engine_running) {                                                                                                   // This ensures LPFP can still prime when unlocking, opening door, Terminal R, Ignition ON etc.
              ptcan_write_msg(ekp_pwm_off_buf);
              serial_log("EKP is disabled.");
            }

            // Visual indicators with Terminal R, 15.
            kcan_write_msg(key_cc_on_buf);                                                                                          // Keep sending this message so that CC is ON until disabled.
            #if IMMOBILIZER_SEQ_ALARM
              if (led_message_counter > 56) {                                                                                       // Send LED message every 112s to keep it ON.
                kcan_write_msg(alarm_led_on_buf);
                alarm_led = true;
                led_message_counter = 0;
              } else {
                led_message_counter++;
              }
            #endif
          }
          immobilizer_timer = 0;
          immobilizer_send_interval = 2000;
        }
      }
    }
  }
  if (!ekp_txq.isEmpty()) {
    ekp_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      ptcan_write_msg(delayed_tx.tx_msg);
      ekp_txq.drop();
    }
  }
  if (!immobilizer_txq.isEmpty()) {
    immobilizer_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      immobilizer_txq.drop();
    }
  }
  #if IMMOBILIZER_SEQ_ALARM
    if (!alarm_siren_txq.isEmpty()) {
      alarm_siren_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        alarm_siren_txq.drop();
      }
    }
  #endif
}


void reset_key_cc(void) {
  kcan_write_msg(key_cc_off_buf);
}


void activate_immobilizer(void) {
  immobilizer_released = false;
  immobilizer_pressed_release_count = 0;
  immobilizer_txq.flush();
  ekp_txq.flush();
  EEPROM.update(13, immobilizer_released);
  update_eeprom_checksum();                                                                                                         // Update now in case we lose power before sleep.
  #if IMMOBILIZER_SEQ_ALARM
    alarm_led = false;
  #endif
  immobilizer_activate_release_timer = 0;
  serial_log("Immobilizer activated.");
}


void release_immobilizer(void) {
  immobilizer_released = true;
  immobilizer_pressed_activate_count = 0;
  EEPROM.update(13, immobilizer_released);                                                                                          // Save to EEPROM directly in case program crashes.
  update_eeprom_checksum();
  ptcan_write_msg(ekp_return_to_normal_buf);                                                                                        // KWP To EKP.
  serial_log("Immobilizer released. EKP control restored to DME.");
  time_now = millis();
  #if IMMOBILIZER_SEQ_ALARM
    if (alarm_active) {
      m = {alarm_siren_off_buf, time_now};                                                                                          // KWP to DWA.
      immobilizer_txq.push(&m);
      alarm_after_engine_stall = alarm_active = false;
      alarm_siren_txq.flush();
      serial_log("Deactivated alarm.");
    }
  #endif
  m = {key_cc_off_buf, time_now + 50};                                                                                              // CC to KCAN.
  immobilizer_txq.push(&m);
  #if IMMOBILIZER_SEQ_ALARM
    m = {alarm_led_off_buf, time_now + 100};                                                                                        // KWP to DWA.
    immobilizer_txq.push(&m);
    m = {alarm_led_off_buf, time_now + 500};
    immobilizer_txq.push(&m);
    alarm_led = false;
  #endif
  if (terminal_r) {
    m = {start_cc_on_buf, time_now + 450};                                                                                          // CC to KCAN.
    immobilizer_txq.push(&m);
    serial_log("Sent start ready CC.");
  }
  if (!terminal_r) {
    play_cc_gong();                                                                                                                 // KWP to KOMBI.
  }
  m = {ekp_return_to_normal_buf, time_now + 500};                                                                                   // KWP To EKP. Make sure these messages are received.
  ekp_txq.push(&m);
  m = {ekp_return_to_normal_buf, time_now + 800};                                                                                   // KWP To EKP.
  ekp_txq.push(&m);
  m = {start_cc_off_buf, time_now + 1000};                                                                                          // CC to KCAN
  immobilizer_txq.push(&m);
  immobilizer_activate_release_timer = 0;
}


void trip_alarm_after_stall(void) {
  if (alarm_after_engine_stall) {
    serial_log("Alarm siren and hazards ON.");
    alarm_active = true;
    time_now = millis();
    for (uint8_t i = 0; i < 10; i++) {                                                                                              // Alarm test job times out after 30s. Make it blast for 5 min.
      m = {alarm_siren_on_buf, time_now + 30000 * i};
      alarm_siren_txq.push(&m);
    }
  }
}


void send_f_powertrain_2_status(void) {
  if (f_data_powertrain_2_timer >= 1000) {

    f_data_powertrain_2[1] = 1 << 4 | f_data_powertrain_2_alive_counter;                                                            // Combine ST_ECU_DT_PT_2 (0001 - State of normal operation) and ALIV_DT_PT_2.
    f_data_powertrain_2_alive_counter == 0xE ? f_data_powertrain_2_alive_counter = 0 
                                             : f_data_powertrain_2_alive_counter++;
    if (engine_running) {
      f_data_powertrain_2[2] = 2;
      if (RPM > 4800) {
        f_data_powertrain_2[3] = 0;                                                                                                 // Engine is idling.
      } else {
        f_data_powertrain_2[3] = 1 << 6;                                                                                            // Not idling.
      }
    } else {
      f_data_powertrain_2[2] = 0x83;
      f_data_powertrain_2[3] = 1 << 6;                                                                                              // Not idling.
    }

    f_data_powertrain_2[3] |= 1 << 4;                                                                                               // Engine start allowed.
    if (clutch_pressed) {
      f_data_powertrain_2[3] |= 1 << 2;
    }
    if (engine_running) {
      f_data_powertrain_2[3] |= 1;
    }

    #if CONTROL_SHIFTLIGHTS
      if (engine_coolant_warmed_up) {
        f_data_powertrain_2[4] = 0x80;                                                                                              // Engine coolant temperature (C): temperature in hex - 48.
        f_data_powertrain_2[5] = 0x80;                                                                                              // Engine oil temperature (C): temperature in hex - 48.
      } else {
        f_data_powertrain_2[4] = 0x3A;                                                                                              // 10 degrees.
        f_data_powertrain_2[5] = 0x3A;
      }
    #else
      f_data_powertrain_2[4] = 0x80;                                                                                                // Hardcode if this status is not available.
      f_data_powertrain_2[5] = 0x80;
    #endif

    f_data_powertrain_2[6] = 0 | 2 << 4;                                                                                            // Engine start allowed.
    if (reverse_gear_status) {
      f_data_powertrain_2[6] |= 2;                                                                                                  // Gearbox in R.
    } else {
      f_data_powertrain_2[6] |= 1;                                                                                                  // Gearbox in N.
    }

    f_data_powertrain_2_crc.restart();
    for (uint8_t i = 1; i < 8; i++) {
      f_data_powertrain_2_crc.add(f_data_powertrain_2[i]);
    }
    f_data_powertrain_2[0] = f_data_powertrain_2_crc.calc();
    f_data_powertrain_2_buf = make_msg_buf(0x3F9, 8, f_data_powertrain_2);
    ptcan_write_msg(f_data_powertrain_2_buf);
    f_data_powertrain_2_timer = 0;
  }
}
