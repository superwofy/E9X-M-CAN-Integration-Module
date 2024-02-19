// Engine/driveline functions go here.


void evaluate_engine_status(void) {
  RPM = ((uint16_t)k_msg.buf[5] << 8) | (uint16_t)k_msg.buf[4];
  uint8_t idle_status = (k_msg.buf[6] & 0xF) >> 2;                                                                                  // Get the first two bits of the second half of this byte.
  if (idle_status == 0) {
    engine_idling = true;
  } else {
    engine_idling = false;
  }
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
      serial_log("Engine started.", 2);
      #if IMMOBILIZER_SEQ
        enable_alarm_after_stall();
      #endif
    }
  } else if (RPM < 200) {                                                                                                           // Less than 50 RPM. Engine stalled or was stopped.
    if (engine_running) {
      engine_running = false;
      serial_log("Engine stopped.", 2);
      #if IMMOBILIZER_SEQ
        execute_alarm_after_stall();
      #endif
    }
  }
}


void toggle_mdrive_message_active(void) {
  if (mdrive_status) {                                                                                                              // Turn OFF MDrive.
    serial_log("Status MDrive OFF.", 2);
    mdrive_message_bn2000[1] -= 1;                                                                                                  // Decrement bytes 1 (6MT, DSC mode) and 4 (SVT) to deactivate.
    mdrive_message_bn2000[4] -= 0x10;
    mdrive_status = mdrive_power_active = false;
    #if FRM_AHL_MODE
      kcan_write_msg(frm_ckm_ahl_komfort_buf);
      serial_log("Set AHL back to comfort mode.", 2);
    #endif
    #if ASD
      if (diag_transmit) {
        kcan_write_msg(mute_asd_buf);
        serial_log("Muted ASD.", 2);
      }
    #endif
    if (mdrive_power[cas_key_number] == 0x30) {
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = false;
      #endif
    } else if (mdrive_power[cas_key_number] == 0x10) {
      console_power_mode = restore_console_power_mode;
    }                                                                                                                               // Else, POWER unchanged
  } else {                                                                                                                          // Turn ON MDrive.
    serial_log("Status MDrive ON.", 2);
    if (mdrive_power[cas_key_number] == 0x20) {                                                                                     // POWER in Sport.
      mdrive_power_active = true;
    } else if (mdrive_power[cas_key_number] == 0x30) {                                                                              // POWER Sport+.
      #if ASD
        if (diag_transmit) {
          kcan_write_msg(demute_asd_buf);
          serial_log("De-muted ASD.", 2);
        }
      #endif
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = true;                                                                                                  // Exhaust flap always open in Sport+
      #endif
      mdrive_power_active = true;
    } else if (mdrive_power[cas_key_number] == 0x10) {
      restore_console_power_mode = console_power_mode;                                                                              // We'll need to return to its original state when MDrive is turned OFF.
      console_power_mode = false;                                                                                                   // Turn OFF POWER from console too.
    }                                                                                                                               // Else, POWER unchanged.

    #if FRM_AHL_MODE
    if (mdrive_svt[cas_key_number] >= 0xF1) {                                                                                       // Headlights will move faster if Servotronic is set to Sport.
      kcan_write_msg(frm_ckm_ahl_sport_buf);
      serial_log("Set AHL to sport mode.", 2);
    }
    #endif

    mdrive_message_bn2000[1] += 1;
    mdrive_message_bn2000[4] += 0x10;
    mdrive_status = true;
  }

  #if F_NBT
    f_driving_dynamics_timer = 1001;
  #endif
}


void toggle_mdrive_dsc_mode(void) {
  if (mdrive_status) {
    if (mdrive_dsc[cas_key_number] == 7) {                                                                                          // DSC OFF requested.
      send_dsc_mode(2);
    } else if (mdrive_dsc[cas_key_number] == 0x13) {                                                                                // DSC MDM (DTC in non-M) requested.
      send_dsc_mode(1);
    } else if (mdrive_dsc[cas_key_number] == 0xB) {                                                                                 // DSC ON requested.
      send_dsc_mode(0);
    }
  } else {
    if (mdrive_dsc[cas_key_number] == 0x13 || mdrive_dsc[cas_key_number] == 7) {                                                    // If MDrive was set to change DSC, restore back to DSC ON.
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
    #if !F_NBT
      if (m_mfl_held_count > 10 && !ignore_m_hold) {                                                                                // Each count is about 100ms
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
    #endif
  } else if (pt_msg.buf[1] == 0xC && pt_msg.buf[0] == 0xC0 && ignore_m_press) {                                                     // Button is released.
    ignore_m_press = ignore_m_hold = false;
    m_mfl_held_count = 0;

    #if IMMOBILIZER_SEQ
    uint8_t activate_release_counter = IMMOBILIZER_SEQ_NUMBER - 1;
    if (!immobilizer_released) {
      activate_release_counter = alarm_active ? IMMOBILIZER_SEQ_ALARM_NUMBER - 1 : IMMOBILIZER_SEQ_NUMBER - 1;

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
            play_cc_gong(1);
          }
        }
      }
    }
    #endif
  }
}


void update_mdrive_can_message(void) {
  mdrive_message_bn2000[1] = mdrive_dsc[cas_key_number] - 2;                                                                        // Difference between iDrive settting and BN2000 CAN message is always 2
                                                                                                                                    // DSC: 1 unchanged, 5 OFF, 0x11 MDM, 9 On
  mdrive_message_bn2000[2] = mdrive_power[cas_key_number];                                                                          // Copy POWER as is.
  mdrive_message_bn2000[3] = mdrive_edc[cas_key_number];                                                                            // Copy EDC as is.
  if (mdrive_svt[cas_key_number] == 0xE9) {
    mdrive_message_bn2000[4] = 0x41;                                                                                                // SVT normal, MDrive OFF.
  } else if (mdrive_svt[cas_key_number] == 0xF1 || mdrive_svt[cas_key_number] == 0xF2) {
    mdrive_message_bn2000[4] = 0x81;                                                                                                // SVT sport (or Sport+ from NBT), MDrive OFF.
  }

  #if F_NBT
    if (mdrive_dsc[cas_key_number] == 7) {                                                                                          // DSC OFF.
      mdrive_message_bn2010[1] = 1;
    } else if (mdrive_dsc[cas_key_number] == 0x13) {                                                                                // DSC MDM (DTC in non-M).
      mdrive_message_bn2010[1] = 3;
    } else if (mdrive_dsc[cas_key_number] == 0xB) {                                                                                 // DSC ON.
      mdrive_message_bn2010[1] = 2;
    }

    if (mdrive_power[cas_key_number] == 0 || mdrive_power[cas_key_number] == 0x10) {
      mdrive_message_bn2010[2] = 1 << 4;                                                                                            // Efficient
    } else if (mdrive_power[cas_key_number] == 0x20) {
      mdrive_message_bn2010[2] = 2 << 4;                                                                                            // Sport                                                                                           
    } else if (mdrive_power[cas_key_number] == 0x30) {
      mdrive_message_bn2010[2] = 3 << 4;                                                                                            // Sport+
    }

    if (mdrive_svt[cas_key_number] == 0xE9) {
      mdrive_message_bn2010[3] = 1 << 4;                                                                                            // Comfort
    } else if (mdrive_svt[cas_key_number] == 0xF1) {
      mdrive_message_bn2010[3] = 2 << 4;                                                                                            // Sport
    } else if (mdrive_svt[cas_key_number] == 0xF2) {
      mdrive_message_bn2010[3] = 3 << 4;                                                                                            // Sport+
    }

    if (mdrive_edc[cas_key_number] == 0x20 || mdrive_edc[cas_key_number] == 0x21) {
      mdrive_message_bn2010[3] |= 1;                                                                                                // Comfort                         
    } else if (mdrive_edc[cas_key_number] == 0x22) {
      mdrive_message_bn2010[3] |= 2;                                                                                                // Sport
    } else if (mdrive_edc[cas_key_number] == 0x2A) {
      mdrive_message_bn2010[3] |= 3;                                                                                                // Sport+
    }
  #endif

  if (mdrive_status) {
    mdrive_message_bn2000[1] += 1;
    mdrive_message_bn2000[4] += 0x10;
  }
}


void show_mdrive_settings_screen(void) {
  if (diag_transmit) {
    #if IMMOBILIZER_SEQ
    if (immobilizer_released) {
    #endif
      serial_log("Steering wheel M button held. Showing settings screen.", 2);
      kcan_write_msg(idrive_mdrive_settings_menu_a_buf);                                                                            // Send steuern_menu job to iDrive.
      kcan_write_msg(idrive_mdrive_settings_menu_b_buf);
      if (!idrive_died) {
        unsigned long time_now = millis();
        m = {idrive_button_sound_buf, time_now + 500};
        idrive_txq.push(&m);
      }
    #if IMMOBILIZER_SEQ
    }
    #endif
    ignore_m_hold = true;
  }
}


void send_mdrive_message(void) {
  mdrive_message_bn2000[0] += 10;
  if (mdrive_message_bn2000[0] > 0xEF) {                                                                                            // Alive(first half of byte) must be between 0..E.
    mdrive_message_bn2000[0] = 0;
  }
  can_checksum_update(0x399, 6, mdrive_message_bn2000);                                                                             // Recalculate checksum. No module seems to check this - MDSC?
  ptcan_write_msg(make_msg_buf(0x399, 6, mdrive_message_bn2000));                                                                   // Send to PT-CAN like the DME would. EDC will receive. KOMBI will receive on KCAN through JBE.
  mdrive_message_timer = 0;
  #if F_NBT
    mdrive_message_bn2010[0] = f_mdrive_alive_counter;
    kcan2_write_msg(make_msg_buf(0x42E, 8, mdrive_message_bn2010));
    f_mdrive_alive_counter == 0xE ? f_mdrive_alive_counter = 0 
                                  : f_mdrive_alive_counter++;
  #endif
}


void send_mdrive_alive_message(uint16_t interval) {
  if (terminal_r) {
    if (mdrive_message_timer >= interval) {                                                                                         // Time MDrive alive message outside of CAN loops. Original cycle time is 10s (idle).                                                                     
      if (ignition) {
        serial_log("Sending Ignition ON MDrive alive message.", 2);
      } else {
        serial_log("Sending Vehicle Awake MDrive alive message.", 2);
      }
      send_mdrive_message();
    }
  }
}


void execute_mdrive_settings_changed_actions() {
  if (mdrive_status) {                                                                                                              // Perform actions that are impacted by changing settings while MDrive is active.
    if (mdrive_power[cas_key_number] == 0x20) {                                                                                     // POWER in Sport.
      mdrive_power_active = true;
    } else if (mdrive_power[cas_key_number] == 0x30) {                                                                              // POWER Sport+.
      #if ASD
        if (diag_transmit) {
          kcan_write_msg(demute_asd_buf);
          serial_log("De-muted ASD.", 2);
        }
      #endif
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = true;                                                                                                  // Exhaust flap always open in Sport+
      #endif
      mdrive_power_active = true;
    } else if (mdrive_power[cas_key_number] == 0x10) {
      restore_console_power_mode = console_power_mode;                                                                              // We'll need to return to its original state when MDrive is turned OFF.
      console_power_mode = false;                                                                                                   // Turn OFF POWER from console too.
      mdrive_power_active = false;
      #if ASD
        if (diag_transmit) {
          kcan_write_msg(mute_asd_buf);
          serial_log("Muted ASD.", 3);
        }
      #endif
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = false;
      #endif
    }
    toggle_mdrive_dsc_mode();                                                                                                       // Change DSC to the new state.

    if (mdrive_svt[cas_key_number] == 0xE9) {
      #if FRM_AHL_MODE
        kcan_write_msg(frm_ckm_ahl_komfort_buf);
        serial_log("Set AHL to comfort mode with SVT setting of Normal.", 3);
      #endif
    } else if (mdrive_svt[cas_key_number] >= 0xF1) {
      #if FRM_AHL_MODE
        kcan_write_msg(frm_ckm_ahl_sport_buf);
        serial_log("Set AHL to sport mode with SVT setting of Sport.", 3);
      #endif
    }
  }
}


void update_mdrive_message_settings_nbt(void) {
  if (k_msg.buf[0] == 0 && k_msg.buf[4] == 0xE0) {                                                                                  // Reset requested.
    reset_mdrive_settings();
  } else if (k_msg.buf[0] == 0xF0) {                                                                                                // Ignore this ping and send the status message.
  } else {
    if (k_msg.buf[0] != 0) {                                                                                                        // DSC settings were updated.
      if (k_msg.buf[0] == 1) {                                                                                                      // DSC OFF.
        mdrive_dsc[cas_key_number] = 7;
      } else if (k_msg.buf[0] == 2) {                                                                                               // DSC ON.
        mdrive_dsc[cas_key_number] = 0xB;
      } else if (k_msg.buf[0] == 3) {                                                                                               // DSC MDM.
        mdrive_dsc[cas_key_number] = 0x13;
      }
    }

    else if(k_msg.buf[1] != 0) {                                                                                                    // Engine settings were updated.
      mdrive_power[cas_key_number] = (k_msg.buf[1] >> 4) * 0x10;
    }

    else if ((k_msg.buf[2] & 0xF) != 0) {                                                                                           // Chassis settings were updated.
      if ((k_msg.buf[2] & 0xF) == 1) {                                                                                              // Comfort.
        mdrive_edc[cas_key_number] = 0x21;
      } else if ((k_msg.buf[2] & 0xF) == 2) {                                                                                       // Sport.
        mdrive_edc[cas_key_number] = 0x22;
      } else if ((k_msg.buf[2] & 0xF) == 3) {                                                                                       // Sport+.
        mdrive_edc[cas_key_number] = 0x2A;
      }
    }

    else if (((k_msg.buf[2] >> 4) & 0x0F) != 0) {                                                                                   // Steering settings were updated.
      if (((k_msg.buf[2] >> 4) & 0x0F) == 1) {                                                                                      // Comfort.
        mdrive_svt[cas_key_number] = 0xE9;
      } else if ((k_msg.buf[2] >> 4) == 2) {                                                                                        // Sport.
        mdrive_svt[cas_key_number] = 0xF1;
      } else if ((k_msg.buf[2] >> 4) == 3) {                                                                                        // Sport+.
        mdrive_svt[cas_key_number] = 0xF2;
      }
    }

    eeprom_unsaved = true;

    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received iDrive settings: DSC 0x%X POWER 0x%X EDC 0x%X SVT 0x%X.", 
          mdrive_dsc[cas_key_number], mdrive_power[cas_key_number], mdrive_edc[cas_key_number], mdrive_svt[cas_key_number]);
      serial_log(serial_debug_string, 3);
    #endif

    update_mdrive_can_message();
    execute_mdrive_settings_changed_actions();
  }
  send_mdrive_message();
}


void reset_mdrive_settings(void) {
  #if F_NBT                                                                                                                         // NBT does not have "Unchanged" settings.
    mdrive_dsc[cas_key_number] = 0xB;                                                                                               // DSC ON
    mdrive_power[cas_key_number] = 0x10;                                                                                            // Normal
    mdrive_edc[cas_key_number] = 0x21;                                                                                              // Comfort
    mdrive_svt[cas_key_number] = 0xE9;                                                                                              // Normal
  #else
    mdrive_dsc[cas_key_number] = 3;                                                                                                 // Unchanged
    mdrive_power[cas_key_number] = 0;                                                                                               // Unchanged
    mdrive_edc[cas_key_number] = 0x20;                                                                                              // Unchanged
    mdrive_svt[cas_key_number] = 0xE9;                                                                                              // Normal
    dme_ckm[cas_key_number][0] = 0xF1;                                                                                              // Normal
  #endif
  serial_log("Reset MDrive settings to defaults.", 3);
  update_mdrive_can_message();
  execute_mdrive_settings_changed_actions();
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
  if (!frm_consumer_shutdown) {
    ptcan_write_msg(make_msg_buf(0x3A9, 2, dme_ckm[cas_key_number]));                                                               // This is sent by the DME to populate the M Key iDrive section
    serial_log("Sent DME POWER CKM.", 3);
  }
}


void update_dme_power_ckm(void) {
  dme_ckm[cas_key_number][0] = k_msg.buf[0];
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Received new POWER CKM setting: %s for key number %d", 
            k_msg.buf[0] == 0xF1 ? "Normal" : "Sport", cas_key_number);
    serial_log(serial_debug_string, 3);
  #endif
  send_dme_power_ckm();                                                                                                             // Acknowledge settings received from iDrive;
  eeprom_unsaved = true;
}


void check_immobilizer_status(void) {
  if (vehicle_awakened_timer >= 2000) {                                                                                             // Delay ensures that time passed after Teensy (re)started to receive messages.
    if (!immobilizer_released) {
      if (vehicle_awakened_timer >= 10000) {                                                                                        // Delay so we don't interfere with normal DWA behavior indicating alarm faults. Also when doing 30G reset.

        // Check frm_consumer_shutdown so that we don't activate when half-waking. Open trunk, lock button while locked...
        if (!terminal_r && !alarm_led_active && !frm_consumer_shutdown && !alarm_led_disable_on_lock) {
          kcan_write_msg(alarm_led_on_buf);                                                                                         // Visual indicator when driver just got in and did not activate anything / car woke. Timeout 120s.
          alarm_led_active = true;                                                                                                  // Sending this multiple times keeps the car awake.
          serial_log("Sent DWA LED ON with Terminal R off.", 2);
          led_message_counter = 60;                                                                                                 // Make sure we're ready once Terminal R cycles.
        }
      }

      // This could be temporarily bypassed if the car is started and driven very very quickly?
      // It will reactivate once indicated_speed <= immobilizer_max_speed. Think Speed (1994)...
      if (!vehicle_moving || (indicated_speed <= immobilizer_max_speed)) {                                                          // Make sure we don't cut this OFF while the car is in (quick) motion!!!
        if (immobilizer_timer >= immobilizer_send_interval) {
          if (terminal_r) {
            if (engine_running && engine_runtime >= 2000) {                                                                         // This ensures LPFP can still prime when unlocking, opening door, Terminal R, Ignition ON etc.
              ptcan_write_msg(ekp_pwm_off_buf);
              serial_log("EKP is disabled.", 0);
            }

            // Visual indicators with Terminal R, 15.
            kcan_write_msg(key_cc_on_buf);                                                                                          // Keep sending this message so that CC is ON until disabled.
            #if F_NBT
              kcan2_write_msg(key_cc_on_buf);
            #endif
            if (led_message_counter > 56) {                                                                                         // Send LED message every 112s to keep it ON.
              kcan_write_msg(alarm_led_on_buf);
              alarm_led_active = true;
              led_message_counter = 0;
            } else {
              led_message_counter++;
            }
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
  if (!alarm_led_txq.isEmpty()) {
    alarm_led_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      alarm_led_txq.drop();
    }
  }
  if (!alarm_warnings_txq.isEmpty()) {
    alarm_warnings_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      #if F_NBT
        kcan2_write_msg(delayed_tx.tx_msg);
      #endif
      alarm_warnings_txq.drop();
    }
  }
  if (!alarm_siren_txq.isEmpty()) {
    alarm_siren_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      alarm_siren_txq.drop();
    }
  }
}


void reset_key_cc(void) {
  kcan_write_msg(key_cc_off_buf);
  #if F_NBT
    kcan2_write_msg(key_cc_off_buf);
  #endif
}


void activate_immobilizer(void) {
  immobilizer_released = false;
  immobilizer_pressed_release_count = 0;
  alarm_led_txq.flush();
  ekp_txq.flush();
  EEPROM.update(30, immobilizer_released);
  update_eeprom_checksum();                                                                                                         // Update now in case we lose power before sleep.
  alarm_led_active = false;
  immobilizer_activate_release_timer = 0;
  serial_log("Immobilizer activated.", 2);
}


void release_immobilizer(void) {
  immobilizer_released = true;
  immobilizer_pressed_activate_count = 0;
  EEPROM.update(30, immobilizer_released);                                                                                          // Save to EEPROM directly in case program crashes.
  update_eeprom_checksum();
  ptcan_write_msg(ekp_return_to_normal_buf);
  unsigned long time_now = millis();
  ekp_txq.flush();
  m = {ekp_return_to_normal_buf, time_now + 100};                                                                                   // Make sure these messages are received.
  ekp_txq.push(&m);
  m = {ekp_return_to_normal_buf, time_now + 300};
  ekp_txq.push(&m);
  serial_log("Immobilizer released. EKP control restored to DME.", 0);
  if (alarm_active) {
    kcan_write_msg(alarm_siren_return_control_buf);
    alarm_siren_txq.flush();                                                                                                        // Clear pending siren requests.
    m = {alarm_siren_return_control_buf, time_now + 100};
    alarm_siren_txq.push(&m);
    m = {alarm_siren_return_control_buf, time_now + 200};
    alarm_siren_txq.push(&m);
    alarm_after_engine_stall = alarm_active = false;
    serial_log("Alarm OFF.", 0);
  }
  kcan_write_msg(alarm_led_return_control_buf);
  alarm_led_txq.flush();
  m = {alarm_led_return_control_buf, time_now + 200};                                                                               // Delay 100ms since we already sent a 6F1 to DWA.
  alarm_led_txq.push(&m);
  m = {alarm_led_return_control_buf, time_now + 300};
  alarm_led_txq.push(&m);
  alarm_led_active = false;
  kcan_write_msg(key_cc_off_buf);
  #if F_NBT
    kcan2_write_msg(key_cc_off_buf);
  #endif
  if (terminal_r) {
    m = {start_cc_on_buf, time_now + 500};
    alarm_warnings_txq.push(&m);
    m = {start_cc_off_buf, time_now + 1000};
    alarm_warnings_txq.push(&m);
    serial_log("Sent start ready CC.", 2);
  } else {
    play_cc_gong(1);
  }
  immobilizer_activate_release_timer = 0;
}


void enable_alarm_after_stall(void) {
  if (!immobilizer_released) {
    alarm_after_engine_stall = true;
    serial_log("Immobilizer still active. Alarm will sound after engine stalls.", 2);    
    kcan_write_msg(key_cc_on_buf);
    #if F_NBT
      kcan2_write_msg(key_cc_on_buf);
    #endif
    if (diag_transmit) {
      kcan_write_msg(alarm_led_on_buf);
      unsigned long time_now = millis();
      m = {cc_single_gong_buf, time_now + 1500};                                                                                    // Audible alert of impending stall.
      alarm_warnings_txq.push(&m);
    }
  }
}


void execute_alarm_after_stall(void) {
  if (alarm_after_engine_stall) {
    alarm_led_txq.flush();                                                                                                          // Clear pending gongs.
    unsigned long time_now = millis();
    m = {alarm_siren_on_buf, time_now};
    alarm_siren_txq.push(&m);
    m = {alarm_siren_on_buf, time_now + 500};                                                                                       // Make sure the alarm request is received.
    alarm_siren_txq.push(&m);
    for (uint8_t i = 1; i < 10; i++) {                                                                                              // Alarm test job times out after 30s. Make it blast for 5 min.
      m = {alarm_siren_on_buf, time_now + 29900 * i};
      alarm_siren_txq.push(&m);
    }
    serial_log("Alarm siren and hazards ON.", 0);
    #if F_NBT
      send_cc_message_text("Immobilizer activated!        ", 0);
    #endif
    alarm_active = true;
  }
}


void send_f_powertrain_2_status(void) {
  if (f_data_powertrain_2_timer >= 1000) {
    uint8_t f_data_powertrain_2[] = {0, 0, 0, 0, 0, 0, 0, 0x8C};                                                                    // Byte7 max rpm: 50 * 140(0x8C) = 7000.

    f_data_powertrain_2[1] = 1 << 4 | f_data_powertrain_2_alive_counter;                                                            // Combine ST_ECU_DT_PT_2 (0001 - State of normal operation) and ALIV_DT_PT_2.
    f_data_powertrain_2_alive_counter == 0xE ? f_data_powertrain_2_alive_counter = 0 
                                             : f_data_powertrain_2_alive_counter++;
                                             
    if (engine_running) {
      f_data_powertrain_2[2] = 0x82;                                                                                                // Engine running.
      if (engine_idling) {
        f_data_powertrain_2[3] = 0;
      } else {
        f_data_powertrain_2[3] = 1 << 6;                                                                                            // Not idling.
      }
    } else {
      f_data_powertrain_2[2] = 0x80;                                                                                                // Engine OFF.
      f_data_powertrain_2[3] = 1 << 6;                                                                                              // Not idling.
    }

    f_data_powertrain_2[3] |= 1 << 4;                                                                                               // Engine start allowed.
    if (clutch_pressed) {
      f_data_powertrain_2[3] |= 1 << 2;
    }
    if (engine_running) {
      f_data_powertrain_2[3] |= 1;
    }

    f_data_powertrain_2[4] = engine_coolant_temperature;                                                                            // Engine coolant temperature (C): value - 48.
    f_data_powertrain_2[5] = engine_oil_temperature;                                                                                // Engine oil temperature (C): value - 48.

    f_data_powertrain_2[6] = 0x2F;                                                                                                  // Engine start allowed, unused (6MT).

    f_data_powertrain_2_crc.restart();
    for (uint8_t i = 1; i < 8; i++) {
      f_data_powertrain_2_crc.add(f_data_powertrain_2[i]);
    }
    f_data_powertrain_2[0] = f_data_powertrain_2_crc.calc();
    
    CAN_message_t f_data_powertrain_2_buf = make_msg_buf(0x3F9, 8, f_data_powertrain_2);
    #if F_NIVI
      ptcan_write_msg(f_data_powertrain_2_buf);
    #endif
    #if F_NBT
      kcan2_write_msg(f_data_powertrain_2_buf);
    #endif
    f_data_powertrain_2_timer = 0;
  }
}


void send_f_torque_1(void) {
  if (f_torque_1_timer >= 100) {
    uint8_t f_torque_1[] = {0, 0, 0, 0, 0, 0, 0, 0xF2};                                                                             // Byte7 QU_AVL_RPM_ENG_CRSH hardcoded to 2 - Signal valid.

    f_torque_1[1] = 0xF << 4 | f_torque_1_alive_counter;                                                                            // Combine FREI (0xF - unused) and ALIV_TORQ_CRSH_1.
    f_torque_1_alive_counter == 0xE ? f_torque_1_alive_counter = 0 
                                  : f_torque_1_alive_counter++;
    
    uint16_t raw_torque = (k_msg.buf[2] << 4 | ((k_msg.buf[1] >> 4) & 0x0F));                                                       // Extract E torque value to big endian.
    
    // Engine torque
    f_torque_1[2] = raw_torque & 0xFF;                                                                                              // LE encoded.
    f_torque_1[3] = (raw_torque >> 8) << 4;

    // Gearbox torque
    f_torque_1[3] = f_torque_1[3] | (raw_torque & 0xF);
    f_torque_1[4] = raw_torque >> 4;

    // Engine RPM (raw, not scaled by 0.25)
    f_torque_1[5] = RPM & 0xFF;                                                                                                     // Transmit in LE.
    f_torque_1[6] = RPM >> 8;

    f_torque_1_crc.restart();
    for (uint8_t i = 1; i < 8; i++) {
      f_torque_1_crc.add(f_torque_1[i]);
    }
    f_torque_1[0] = f_torque_1_crc.calc();

    CAN_message_t f_torque_1_buf = make_msg_buf(0xA5, 8, f_torque_1);
    kcan2_write_msg(f_torque_1_buf);
    f_torque_1_timer = 0;
  }
}


void send_f_driving_dynamics_switch_nbt(void) {
  if (f_driving_dynamics_timer >= 1001) {
    uint8_t f_driving_dynamics[] = {0xFF, 0xFF, 0, 0, 0, 0, 0xC0};                                                                  // Inspired very loosely by 272.4.8...
    uint8_t new_driving_mode = driving_mode;

    // Not needed for KCAN
    // f_driving_dynamics[1] = 0xF << 4 | f_driving_dynamics_alive_counter;
    // f_driving_dynamics_alive_counter == 0xE ? f_driving_dynamics_alive_counter = 0 
    //                                         : f_driving_dynamics_alive_counter++;

    if (pdc_bus_status != 0xA4 && pdc_bus_status != 0xA5) {                                                                         // The popup interferes with the reverse camera.
      if (mdrive_status && mdrive_power[cas_key_number] == 0x30 && mdrive_dsc[cas_key_number] == 7 
          && mdrive_edc[cas_key_number] == 0x2A && mdrive_svt[cas_key_number] >= 0xF1) {                                            // Sportiest settings.
        f_driving_dynamics[4] = 6;
        new_driving_mode = 2;
      } else if (mdrive_status && mdrive_power[cas_key_number] == 0x30 && mdrive_dsc[cas_key_number] == 0x13 
                  && mdrive_edc[cas_key_number] == 0x2A && mdrive_svt[cas_key_number] >= 0xF1) {                                    // Sportiest settings but with MDM.
        f_driving_dynamics[4] = 5;
        new_driving_mode = 1;
      } else {
        f_driving_dynamics[4] = 0x13;                                                                                               // Blank box with COMFORT title. Used to indicate DSC back ON.
        new_driving_mode = 0;
      }
    }

    // Not needed for KCAN
    // f_driving_dynamics_crc.restart();
    // for (uint8_t i = 1; i < 7; i++) {
    //   f_driving_dynamics_crc.add(f_driving_dynamics[i]);
    // }
    // f_driving_dynamics[0] = f_driving_dynamics_crc.calc();

    CAN_message_t f_driving_dynamics_buf = make_msg_buf(0x3A7, 7, f_driving_dynamics);
    kcan2_write_msg(f_driving_dynamics_buf);
    f_driving_dynamics_timer = 0;

    if (new_driving_mode != driving_mode) {                                                                                         // Simulate a quick left movement of the ZBE to disimiss COMFORT popup.
      if (new_driving_mode == 0) {
        uint8_t zbe_east[] = {0xE1, 0xFD, 0, 0x81, 0xDD, 1};
        kcan2_write_msg(make_msg_buf(0x267, 6, zbe_east));
        zbe_east[3] = 0;
        kcan2_write_msg(make_msg_buf(0x267, 6, zbe_east));
      }
      driving_mode = new_driving_mode;
    }
  }
}


void send_f_driving_dynamics_switch_evo(void) {
  if (f_driving_dynamics_timer >= 1001) {
    uint8_t f_driving_dynamics[] = {0xFF, 0xFF, 0, 0, 0, 0, 0xC0};

    if (dsc_program_status == 1) {
      f_driving_dynamics[4] = 1;
    } else if (dsc_program_status == 2) {
      f_driving_dynamics[4] = 6;
    }

    CAN_message_t f_driving_dynamics_buf = make_msg_buf(0x3A7, 7, f_driving_dynamics);
    kcan2_write_msg(f_driving_dynamics_buf);
    f_driving_dynamics_timer = 0;
  }
}


void send_f_oil_level(void) {                                                                                                       // Used with OELSTAND_OENS.
  if (f_oil_level_timer >= 501) {
    uint8_t f_oil_level[4] = {0, 0xF0, 4, 0xC0};
    switch(e_oil_level) {
      case 0xC: {                                                                                                                   // Below MIN
        f_oil_level[1] = 0;
        break;
      }
      case 0x19: {                                                                                                                  // MIN
        f_oil_level[1] = 0x10;
        break;
      }
      case 0x26: {                                                                                                                  // Between MIN and OK
        f_oil_level[1] = 0x20;
        break;
      }
      case 0x35: {                                                                                                                  // OK
        f_oil_level[1] = 0x30;
        break;
      }
      case 0x45: {                                                                                                                  // Between OK and MAX
        f_oil_level[1] = 0x40;
        break;
      }
      case 0x55: {                                                                                                                  // State MAX
        f_oil_level[1] = 0x50;
        break;
      }
      case 0x5F: {                                                                                                                  // Overfilled
        f_oil_level[1] = 0x70;
        break;
      }
      case 0x78: {                                                                                                                  // No measurement possible
        f_oil_level[2] = 0x40;
        break;
      }
      case 0x79: {                                                                                                                  // Measurement in progress
        break;
      }
      case 0x7A: {                                                                                                                  // Oil Level Check OK (Oil level sufficient to start engine).
        f_oil_level[1] = 0x50;
        break;
      }
      case 0x7B: {                                                                                                                  // Measurement OK
        f_oil_level[1] = 0x50;
        break;
      }
      case 0x7C: {                                                                                                                  // Ignition OFF, no measurement possible
        f_oil_level[0] = 0xFF;
        f_oil_level[2] = 0xFF;
        break;
      }
      case 0x7D: {                                                                                                                  // Engine oil level OK, precise measurement in progress.
        break;
      }
      case 0x7E: {                                                                                                                  // Engine oil level OK
        f_oil_level[1] = 0x50;
        break;
      }
      case 0x7F: {                                                                                                                  // Measurement in progress
        break;
      }
      case 0x80: {                                                                                                                  // Accurate measurement in progress. Measuring time: 1min
        break;
      }
      case 0xFF: {                                                                                                                  // signal invalid
        f_oil_level[2] = 0x40;
        break;
      }
    }
    kcan2_write_msg(make_msg_buf(0x435, 4, f_oil_level));
    f_oil_level_timer = 0;
  }
  if (ignition) {
    if (f_oil_level_measuring_timer >= 60000) {                                                                                     // Periodically and when cycling ignition, re-set the level.
      kcan2_write_msg(f_oil_level_measuring_buf);
      f_oil_level_measuring_timer = 0;
      f_oil_level_timer = 501;
    }
  }
}
