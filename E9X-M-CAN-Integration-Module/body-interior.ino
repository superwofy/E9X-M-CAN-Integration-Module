// General body functions dealing with interior electronics go here.


void check_vehicle_awake(void) {
  vehicle_awake_timer = 0;
  if (!vehicle_awake) {
    vehicle_awake = true;
    serial_log("Vehicle Awake.", 2);
    vehicle_awakened_timer = 0;
    kl30g_cutoff_imminent = false;
    requested_hu_off_t2 = false;
    hu_bn2000_nm_timer = 1000;

    // Keep this fairly low - it delays the FRM 30G reset.
    hu_bn2000_bus_sleep_ready_timer = HU_ENT_MODE_TIMEOUT / 2;

    activate_optional_transceivers();
    #if F_NBTE
      FACEPLATE_UART.begin(38400, SERIAL_8E1);
    #endif
    alarm_led_message_timer = 100000;
  }
}


void evaluate_terminal_clutch_keyno_status(void) {
  uint8_t f_terminal_status[] = {0, 0, 0x80, 0xFF, 0, 1, 0x3F, 0xFF};                                                               // Byte2: ST_KL. Spec says it should be 0xF but all traces have 0x8...
  bool terminal_r_ = terminal_r, ignition_ = ignition, terminal_50_ = terminal_50;

  // These must be checked since error states such as 2 or 3 can be set. E.g when PGS is being reset (Byte0 = 0x27).
  terminal_r = (k_msg.buf[0] & 0b11) == 1 ? true : false;                                                                           // 0 OFF, 1 ON, 3 INVALID_SIGNAL ??
  ignition = ((k_msg.buf[0] & 0b1100) >> 2) == 1 ? true : false;                                                                    // 0 IGK_OFF, 1 IGK_ON, 2 INVALID_SIGNAL
  terminal_50 = ((k_msg.buf[0] & 0b110000) >> 4) == 1 ? true : false;
  key_valid = ((k_msg.buf[0] & 0b11000000) >> 6) == 1 ? true : false;                                                               // Set to 0 when PGS module loses key.

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

  #if F_VSW01 || F_NIVI || F_NBTE                                                                                                   // Translate 0x130 to 0x12F for F-series modules. 6MC2DL0B pg. 1316.
    if (terminal_50) {
      f_terminal_status[1] = 0xB << 4;                                                                                              // VSM_STM_STATE_ENG_START
      f_terminal_status[2] = 0x8D;                                                                                                  // KL50_ON
    } else if (ignition) {
      if (engine_running == 2) {
        if (vehicle_moving) {
          f_terminal_status[1] = 8 << 4;                                                                                            // VSM_STM_STATE_DRIVE
        } else {
          f_terminal_status[1] = 7 << 4;                                                                                            // VSM_STM_STATE_ENG_IDLE
        }
        f_terminal_status[5] = 5;                                                                                                   // ST_STCD_PENG: Starting condition of traction fulfilled, CTR_ENG_STOP: Default
      } else {
        f_terminal_status[1] = 5 << 4;                                                                                              // VSM_STM_STATE_IGNITION
      }
      f_terminal_status[2] = 0x8A;                                                                                                  // KL15_ON
    } else if (terminal_r) {
      f_terminal_status[1] = 2 << 4;
      f_terminal_status[2] = 0x88;                                                                                                  // KLR_ON
    } else {
      if (doors_alarmed) {
        f_terminal_status[1] = 1 << 4;                                                                                              // Driver not present. VSM_STM_STATE_STANDBY
      } else {
        f_terminal_status[1] = 2 << 4;                                                                                              // VSM_STM_STATE_BASICOP
      }
      f_terminal_status[2] = 0x86;                                                                                                  // KL30B_ON / 30G for BN2000
    }

    f_terminal_status[1] = f_terminal_status[1] | f_terminal_status_alive_counter;                                                  // Combine ST_VEH_CON and ALIV_COU_KL
    f_terminal_status_alive_counter == 0xF ? f_terminal_status_alive_counter = 0 : f_terminal_status_alive_counter++;
    f_terminal_status[4] = (0xF << 4) | key_valid ? 3 : 1;                                                                          // Set ST_KL_KEY_VLD
  #endif

  if (terminal_r && !terminal_r_) {                                                                                                 // Terminal R changed from OFF to ON.
    serial_log("Terminal R ON.", 2);
    comfort_exit_ready = false;
    f_terminal_status[2] = 0x87;
    requested_hu_off_t2 = false;
    kl30g_cutoff_imminent = false;
    activate_optional_transceivers();                                                                                               // Re-activate transceivers with Terminal R in case the FRM message was missed.
    car_locked_indicator_counter = 0;
    custom_info_cc_timer = 3000;
    #if FRM_AHL_MODE
      kcan_write_msg(frm_ckm_ahl_komfort_buf);                                                                                      // Make sure we're in comfort mode on startup.
    #endif
    #if F_NBTE_CCC_ZBE
      kcan_write_msg(ccc_zbe_wake_buf);                                                                                             // ZBE1 will now transmit data on 0x1B8.
    #endif
    #if F_NBTE
      send_nbt_sport_displays_data(false);                                                                                          // Initialize the sport display scale.
    #endif
  } else if (!terminal_r && terminal_r_) {
    serial_log("Terminal R OFF.", 2);
    comfort_exit_ready = true;
    intermittent_wipe_active = false;
    f_terminal_status[2] = 0x87;
    hu_bn2000_bus_sleep_ready_timer = 0;                                                                                            // Will allow the network to sleep unless the driver presses the faceplate button before timeout.
    custom_info_cc_timer = 3000;
  }

  if (ignition && !ignition_) {                                                                                                     // Ignition changed from OFF to ON.
    scale_cpu_speed();
    serial_log("Ignition ON.", 2);
    #if F_NBTE
      send_nbt_sport_displays_data(false);
    #endif
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
    f_terminal_status[2] = 0x89;                                                                                                    // KL15_change
    hu_bn2000_nm_next_neighbour = 0x64;                                                                                             // PDC controller.
    hu_bn2000_nm_timer = 3000;
    custom_info_cc_timer = 3000;
    trsvc_watchdog_timer = 0;
  } else if (!ignition && ignition_) {
    reset_ignition_variables();
    scale_cpu_speed();                                                                                                              // Now that the ignition is OFF, underclock the MCU
    serial_log("Ignition OFF.", 2);
    f_terminal_status[2] = 0x89;                                                                                                    // KL15_change
    custom_info_cc_timer = 3000;
  }

  if (terminal_50 && !terminal_50_) {
    f_terminal_status[1] = 9 << 4;                                                                                                  // Impending start of engine (VSM_STM_STATE_ENG_START_PRE).
    f_terminal_status[2] = 0x8C;                                                                                                    // KL50_change
    serial_log("Terminal 50 (starter) ON", 2);
  } else if (!terminal_50 && terminal_50) {
    f_terminal_status[2] = 0x8C;                                                                                                    // KL50_change
    serial_log("Terminal 50 (starter) OFF.", 2);
  }

  #if F_VSW01 || F_NIVI || F_NBTE
    f_terminal_status_crc.restart();
    for (uint8_t i = 1; i < 8; i++) {
      f_terminal_status_crc.add(f_terminal_status[i]);
    }
    f_terminal_status[0] = f_terminal_status_crc.calc();

    CAN_message_t f_terminal_status_buf = make_msg_buf(0x12F, 8, f_terminal_status);
    #if F_VSW01
      kcan_write_msg(f_terminal_status_buf);
    #endif
    #if F_NBTE || F_NIVI
      kcan2_write_msg(f_terminal_status_buf);
      
      uint8_t f_vehicle_status[] = {0, 0, 0, 0, 0, 0, 0xE3, 0xFF};
      if (terminal_r) {
        f_vehicle_status[1] = 0xA;
        f_vehicle_status[2] = 2;
        f_vehicle_status[3] = 0x12;
        f_vehicle_status[4] = 1;
        f_vehicle_status[6] = 0x2A;
      }
      f_vehicle_status[1] = f_vehicle_status[1] << 4 | f_vehicle_status_alive_counter;
      f_vehicle_status_alive_counter == 0xE ? f_vehicle_status_alive_counter = 0 
                                            : f_vehicle_status_alive_counter++;
      f_vehicle_status_crc.restart();
      for (uint8_t i = 1; i < 8; i++) {
        f_vehicle_status_crc.add(f_vehicle_status[i]);
      }
      f_vehicle_status[0] = f_vehicle_status_crc.calc();
      kcan2_write_msg(make_msg_buf(0x3C, 8, f_vehicle_status));
    #endif
  #endif
}


void evaluate_frm_consumer_shutdown(void) {
  // The FRM sends this request after the vehicle has been sleeping for a predefined time (NACHLAUF_KLEMME_VA).
  // This can be observed when the car wakes up and turns off the interior lights. It then goes to deep sleep except:
    // A special case is when the iDrive is still awake after pressing the power/mute button and its NM is keeping the car awake.
    // In that context, the consumers OFF message is sent
  // OR
  // If the car is locked, FC is sent immediately.
  // OR
  // If the car is half-woken with the remote trunk button or lock button while locked, FC is sent immediately.

  if (k_msg.buf[0] == 0xFC) {
    if (!frm_consumer_shutdown) {
      frm_consumer_shutdown = true;
      scale_cpu_speed();                                                                                                            // Reduce power consumption in this state.
      if (vehicle_awakened_timer <= 2000) {
        serial_log("FRM woke KCAN to reset 30G.", 2);
      } else {
        serial_log("FRM requested optional consumers OFF.", 2);
      }
    }
    #if F_NBTE
      if (!ignition) {
        kcan2_write_msg(fzm_sleep_buf);                                                                                             // Indicate sleep readiness to KCAN2 modules.
      }
    #endif
  } else if (k_msg.buf[0] == 0xFD) {
    if (frm_consumer_shutdown) {
      frm_consumer_shutdown = false;
      serial_log("FRM requested optional consumers back ON.", 2);
      activate_optional_transceivers();
      #if F_VSW01 && F_VSW01_MANUAL
        vsw_switch_input(4);
      #endif
      requested_hu_off_t2 = false;
    }
    #if F_NBTE
      if (!ignition) {                                                                                                              // 3A5 is only sent with KL15_OFF.
        if (!hu_bn2000_bus_sleep_active) {
          kcan2_write_msg(fzm_wake_buf);                                                                                            // Without 3A5 wake AMPT goes to sleep almost immediately after KLR_OFF.
        } else {
          kcan2_write_msg(fzm_sleep_buf);                                                                                           // If BN2000(OSEK) HU NM set to sleep, indicate sleep readiness to KCAN2 modules.
        }
      }
    #endif
  } 
}


void evaluate_seat_heating_status(void) {
  if (terminal_r) {
    if (k_msg.id == 0x232) {
      driver_seat_heating_status = !k_msg.buf[0] ? false : true;
      if (!driver_seat_heating_status) {                                                                                            // Check if seat heating is already ON.
        if (!driver_sent_seat_heating_request) { 
          if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_HIGH) {
            send_seat_heating_request_driver(false);
          } else if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_MEDIUM) {
            send_seat_heating_request_driver(true);
          }
        }
      } else {
        driver_sent_seat_heating_request = true;                                                                                    // Seat heating already ON. No need to request anymore.
      }
    } 
    #if AUTO_SEAT_HEATING_PASS
    else {                                                                                                                          // Passenger's seat heating status message is only sent with ignition ON.
      passenger_seat_heating_status = !k_msg.buf[0] ? false : true;
      if (!passenger_seat_heating_status) {
        if (!passenger_sent_seat_heating_request) {
          if (passenger_seat_status == 9) {                                                                                         // Occupied and belted.
            if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_HIGH) {
              send_seat_heating_request_pas(false);
            } else if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_MEDIUM) {
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
}


void send_seat_heating_request_driver(bool medium) {
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
    m = {seat_heating_button_pressed_dr_buf, time_now + 700};
    seat_heating_dr_txq.push(&m);
    m = {seat_heating_button_released_dr_buf, time_now + 800};
    seat_heating_dr_txq.push(&m);
    m = {seat_heating_button_released_dr_buf, time_now + 1000};
    seat_heating_dr_txq.push(&m);
    m = {seat_heating_button_released_dr_buf, time_now + 1200};
    seat_heating_dr_txq.push(&m);
  }
  sprintf(serial_debug_string, "Sent [%s] driver's seat heating request at %s temp: %.1fC.",
          medium ? "medium" : "high",
          interior_temperature > ambient_temperature_real ? "interior" : "ambient",
          interior_temperature);
  serial_log(serial_debug_string, 2);
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
  if (k_msg.buf[0] != 0xFD) {                                                                                                       // Until byte0 stabilizes at 0xFC, this status is not to be trusted.
    if (ignition) {
      if (!passenger_seat_heating_status) {                                                                                         // Check if seat heating is already ON.
        //This will be ignored if already ON and cycling ignition. Press message will be ignored by IHK anyway.
        if (!passenger_sent_seat_heating_request) {
          if (bitRead(passenger_seat_status, 0) && bitRead(passenger_seat_status, 3)) {                                             // Occupied and belted.
            if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_HIGH) {                          // Execute heating requests here so we don't have to wait 15s for the next 0x22A.
              send_seat_heating_request_pas(false);
            } else if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_MEDIUM) {
              send_seat_heating_request_pas(true);
            }
          }
        }
      } else {
        passenger_sent_seat_heating_request = true;                                                                                 // Seat heating already ON. No need to request anymore.
      }
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
    m = {seat_heating_button_pressed_pas_buf, time_now + 700};
    seat_heating_pas_txq.push(&m);
    m = {seat_heating_button_released_pas_buf, time_now + 800};
    seat_heating_pas_txq.push(&m);
    m = {seat_heating_button_released_pas_buf, time_now + 1000};
    seat_heating_pas_txq.push(&m);
    m = {seat_heating_button_released_pas_buf, time_now + 1200};
    seat_heating_pas_txq.push(&m);
  }
  sprintf(serial_debug_string, "Sent [%s] passenger's seat heating request at %s temp: %.1fC.",
          medium ? "medium" : "high",
          interior_temperature > ambient_temperature_real ? "interior" : "ambient",
          interior_temperature);
  serial_log(serial_debug_string, 2);
}


void evaluate_steering_heating_request(void) {
  // Wait for supply voltage to stabilize.
  // Also limit the time the request can be sent after as the driver may already have eanbled the heater.
  // The temperature may also have dropped below the threshold while the driver already enabled the heater.
  // There is no way to track the steering heater via CAN.

  if (engine_run_timer >= AUTO_HEATING_START_DELAY && engine_run_timer <= 10000) {                                                               
    if (!sent_steering_heating_request) {
      if (max(ambient_temperature_real, interior_temperature) <= AUTO_SEAT_HEATING_THRESHOLD_HIGH) {
        digitalWrite(STEERING_HEATER_SWITCH_PIN, HIGH);
        serial_log("Activated steering wheel heating.", 2);
        sent_steering_heating_request = steering_heater_transistor_active = true;
        steering_heater_transistor_active_timer = 0;
      } else {
        sent_steering_heating_request = true;                                                                                       // If the conditions aren't right, cancel activation for this wake cycle.
      }
    } else {
      if (steering_heater_transistor_active) {
        if (steering_heater_transistor_active_timer >= 400) {
          digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);                                                                            // Release control of the switch so that the driver can now operate it.
          steering_heater_transistor_active = false;
        }
      }
    }
  }
}


void send_f_energy_condition(void) {                                                                                                // 6MC2DL0B pg. 1532.
  if (f_energy_condition_timer >= 5000) {
    uint8_t f_energy_condition[] = {0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};                                                // Energy good.

    if (low_battery_cc_active) {
      f_energy_condition[2] = 0xF3;
    } else if (battery_voltage <= 12.4 && battery_voltage > 12.2) {
      f_energy_condition[2] = 0xF1;                                                                                                 // Energy OK (80% to 50% SoC).
    } else if (battery_voltage <= 12.2 && battery_voltage > 11.6) {
      f_energy_condition[2] = 0xF2;                                                                                                 // Energy shortage (50% to 20% SoC).
    } else if (battery_voltage <= 11.6) {
      f_energy_condition[2] = 0xF3;                                                                                                 // Energy severe shortage (less than 20% SoC).
    }

    CAN_message_t f_energy_condition_buf = make_msg_buf(0x3A0, 8, f_energy_condition);
    kcan_write_msg(f_energy_condition_buf);
    #if F_NBTE || F_NIVI
      kcan2_write_msg(f_energy_condition_buf);
    #endif

    f_energy_condition_timer = 0;
  }
}


void evaluate_battery_voltage(void) {
  battery_voltage = ((((k_msg.buf[1] & 0xF) << 8) | k_msg.buf[0]) * 15) / 1000.0;
}


void check_console_buttons(void) {
  #if IMMOBILIZER_SEQ
    if (immobilizer_released) {
      if (!digitalRead(POWER_BUTTON_PIN) && !digitalRead(DSC_BUTTON_PIN)) {
        if (!holding_both_console) {
          both_console_buttons_timer = 0;
          holding_both_console = true;
        } else {
          if (both_console_buttons_timer >= 10000) {                                                                                // Hold both buttons for more than 10s.
            #if F_NBTE
              kcan2_write_msg(idrive_horn_sound_buf);
            #else
              kcan_write_msg(idrive_button_sound_buf);                                                                              // Acknowledge Immobilizer persist ON-OFF with Gong.
            #endif
            immobilizer_persist = !immobilizer_persist;
            EEPROM.update(31, immobilizer_persist);
            update_eeprom_checksum();
            sprintf(serial_debug_string, "Immobilizer now persistently: %s.", immobilizer_persist ? "ON" : "OFF");
            serial_log(serial_debug_string, 2);
            #if F_NBTE
              if (!immobilizer_persist) {
                send_cc_message("Immobilizer deactivated persistently.", true, 5000);
              } else {
                send_cc_message("Immobilizer activated persistently.", true, 5000);
              }
            #endif
            both_console_buttons_timer = 0;                                                                                         // Reset to prevent multiple activations.
          }
        }
        dsc_off_button_hold_timer = 0;
        return;                                                                                                                     // Prevent nuisance mode changes while holding both buttons.
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

  #if !MDSC_ZB                                                                                                                      // For M3 DSC the switch must be wired directly to pin 41 of X18303.
    if (!digitalRead(DSC_BUTTON_PIN)) {
      if (!holding_dsc_off_console) {
        holding_dsc_off_console = true;
        dsc_off_button_hold_timer = 0;
      }
      
      if (dsc_off_button_hold_timer >= dsc_hold_time_ms) {                                                                          // DSC OFF sequence should only be sent after user holds button for a configured time.
        if (dsc_off_button_debounce_timer >= dsc_debounce_time_ms) {
          if (dsc_program_status != 1 && !dsc_mode_change_disable) {                                                                // If the button is held after DSC OFF, no more messages are sent until release.
            serial_log("Console: DSC OFF requested.", 2);
            if (!send_dsc_mode(1)) {
              serial_log("Console: Failed to change DSC mode during stabilisation.", 1);
              #if F_NBTE
                send_cc_message("DSC mode change refused while stabilising.", true, 3000);
              #endif
            }
          }
          dsc_off_button_debounce_timer = 0;
        }
      }

      if (dsc_off_button_hold_timer >= 15000) {                                                                                     // Emulate BMW's safety feature when a purse/other object rests on the DSC OFF button.
        if (!dsc_mode_change_disable) {
          send_dsc_mode(0);
          serial_log("Console: DSC OFF button obstruction detected! Cycle ignition to reset.", 1);
          #if F_NBTE
            kcan2_write_msg(cc_single_gong_buf);
            send_cc_message("DSC OFF button obstruction detected!", true, 5000);
          #endif
          dsc_mode_change_disable = true;
        }
      }
    } else {                                                                                                                        // A quick tap re-enables everything.
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
  #endif
}


void send_nivi_button_press(void) {
  kcan2_write_msg(nivi_button_pressed_buf);
  kcan2_write_msg(nivi_button_pressed_buf);
  kcan2_write_msg(nivi_button_released_buf);
  kcan2_write_msg(nivi_button_released_buf);
  serial_log("Sent NiVi button press.", 2);
}


void evaluate_dr_seat_ckm(void) {
  #if F_NBTE
  if (k_msg.buf[0] == 0xFA) {
  #else
  if (k_msg.buf[0] == 0xFC) {
  #endif
    if (!auto_seat_ckm[cas_key_number]) {
      serial_log("Automatic seat position CKM ON.", 2);
      auto_seat_ckm[cas_key_number] = true;
    }
  } else {
    if (auto_seat_ckm[cas_key_number]) {
      serial_log("Automatic seat position CKM OFF.", 2);
      auto_seat_ckm[cas_key_number] = false;
    }
  }
}


void evaluate_comfort_exit(void) {
  if (comfort_exit_ready && auto_seat_ckm[cas_key_number]) {
    kcan_write_msg(dr_seat_move_back_buf);
    comfort_exit_ready = false;
    serial_log("Moved driver's seat back for comfort exit.", 2);
  }

  // If the door is opened and conditions are not satisfied, disable if it is re-opened and conditions are satsisfied.
  comfort_exit_ready = false;
}


void store_rvc_settings_idrive(void) {
  if (ignition) {
    if (k_msg.buf[0] == 0xE6) {                                                                                                     // Camera OFF.
      rvc_settings[0] = 0xE5;                                                                                                       // Store Camera ON, Tow view OFF instead.
    } else {
      rvc_settings[0] = k_msg.buf[0];
    }

    for (uint8_t i = 1; i < 4; i++) {
      rvc_settings[i] = k_msg.buf[i];
    }

    if (!rvc_tow_view_by_driver && !rvc_tow_view_by_module && bitRead(k_msg.buf[0], 3) && pdc_bus_status == 0xA5) {                 // If the driver changed this setting, do not interfere during this cycle.
      rvc_tow_view_by_driver = true;
      serial_log("Driver changed RVC tow view manually.", 2);
    }
  }
}


void store_rvc_settings_trsvc(void) {
  if (ignition) {
    rvc_settings[1] = k_msg.buf[1];                                                                                                 // These values are the same as sent by iDrive or TRSVC.
    rvc_settings[2] = k_msg.buf[2];
    if (k_msg.buf[3] == 1) {                                                                                                        // Parking lines OFF, Obstacle marking OFF, Tow view OFF.
      rvc_settings[3] = 0xE0;
    } else if (k_msg.buf[3] == 9) {                                                                                                 // Parking lines OFF, Obstacle marking ON, Tow view OFF.
      rvc_settings[3] = 0xE1;
    } else if (k_msg.buf[3] == 0x39) {                                                                                              // Parking lines ON, Obstacle marking ON, Tow view OFF.
      rvc_settings[3] = 0xE7;
    }
  }
}


void evaluate_indicator_stalk(void) {
  #if MIRROR_UNDIM
    if (ignition) {
      if (k_msg.buf[0] == 1 || k_msg.buf[0] == 4) {
        szl_full_indicator = false;
      } else if (k_msg.buf[0] == 2 || k_msg.buf[0] == 8) {
        if (!szl_full_indicator) {
          szl_full_indicator = true;
          serial_log("Indicator stalk pushed fully.", 2);
          undim_mirrors_with_indicators();
        }
      }
    }
  #endif
  #if F_NBTE
    if (terminal_r) {
      if (k_msg.buf[0] == 0x10) {                                                                                                   // Stalk pushed away.
        indicator_stalk_pushed_message_counter++;
        if (indicator_stalk_pushed_message_counter >= 45) {                                                                         // About 5 seconds.
          if (hba_status == 0xD) {
            kcan_write_msg(idrive_bn2000_hba_on_buf);
            kcan2_write_msg(idrive_horn_sound_buf);
            send_cc_message("High beam assistant ON.", true, 3000);
          } else {
            kcan_write_msg(idrive_bn2000_hba_off_buf);
            kcan2_write_msg(idrive_horn_sound_buf);
            send_cc_message("High beam assistant OFF.", true, 3000);
          }
          indicator_stalk_pushed_message_counter = 0;
        }
      } else {
        indicator_stalk_pushed_message_counter = 0;
      }
    }
  #endif
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
        serial_log("Power-down command sent successfully. Car will kill KL30G and assume deep sleep in <15s.", 0);
        power_down_requested = false;
      } else {
        power_down_requested = false;
        serial_log("Power-down command aborted due to error.", 0);
      }
    } else {
      power_down_requested = false;
      serial_log("Power-down command rejected due to OBD tool presence.", 0);
    }
  }
}


void evaluate_wiper_stalk_status(void) {
  if (terminal_r) {
    if (k_msg.buf[0] == 0x10) {
      #if HEADLIGHT_WASHING
        track_wash_wipe_cycle = true;
      #endif
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
        wiper_stalk_down_message_counter = 0;
      #endif
    } 

    else if (k_msg.buf[0] == 0) {                                                                                                   // Wiping completely OFF (stak released).
      #if HEADLIGHT_WASHING
        if (track_wash_wipe_cycle) {
          // NOTE: counter will increase if spraying with ignition OFF.
          track_wash_wipe_cycle = false;
          wash_wipe_cycles = (wash_wipe_cycles + 1) % 0x100;                                                                        // With the stalk released, count this as a cycle.
          control_headlight_washers();
        }
      #endif
      #if WIPE_AFTER_WASH
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        wiper_stalk_down_message_counter = 0;
      #endif
    }
    
    else if (k_msg.buf[0] == 1) {                                                                                                   // AUTO button pressed.
      #if WIPE_AFTER_WASH
        abort_wipe_after_wash();
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        if (intermittent_wipe_active) {
          disable_intermittent_wipers();
        }
        wiper_stalk_down_message_counter = 0;
      #endif
    }

    else if (k_msg.buf[0] == 2 || k_msg.buf[0] == 3) {                                                                              // Stalk pushed up once / twice.
      #if WIPE_AFTER_WASH
        abort_wipe_after_wash();
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        if (intermittent_wipe_active) {
          disable_intermittent_wipers();
        }
        wiper_stalk_down_message_counter = 0;
      #endif
    }

    else if (k_msg.buf[0] == 8) {                                                                                                   // Stalk pushed down. 9 = stalk pushed down with AUTO ON.
      #if WIPE_AFTER_WASH
        abort_wipe_after_wash();
        wash_message_counter = 0;
      #endif
      #if INTERMITTENT_WIPERS
        if (millis() - wiper_stalk_down_last_press_time >= 1100) {                                                                  // If more than 1100ms passed, stalk must have been released.
          wiper_stalk_down_message_counter = 0;
        }

        wiper_stalk_down_last_press_time = millis();
        wiper_stalk_down_message_counter++;

        intermittent_wipe_timer = 0;

        if (wiper_stalk_down_message_counter >= 4) {
          if (!intermittent_wipe_active) {
            activate_intermittent_wipers();
            intermittent_wipe_timer = intermittent_intervals[intermittent_setting];
          } else {
            disable_intermittent_wipers();
          }
          wiper_stalk_down_message_counter = 0;
        }
      #endif
    }

    #if INTERMITTENT_WIPERS || WIPE_AFTER_WASH
      if (pt_msg.buf[1] != intermittent_setting_can) {                                                                              // Position of the wiper speed thumbwheel changed.
        intermittent_setting_can = pt_msg.buf[1];
        uint8_t new_intermittent_setting = 0;
        new_intermittent_setting = intermittent_setting_can - 0xF8;
        #if INTERMITTENT_WIPERS
          if (intermittent_wipe_active) {
            sprintf(serial_debug_string, "New wiper speed setting %d%s.", new_intermittent_setting + 1,
                    !vehicle_moving ? " [not moving]" : "");
            serial_log(serial_debug_string, 3);
          }
        #endif
        wiper_stalk_down_message_counter = 0;
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


void send_climate_popup_acknowledge(void) {
  if (ignition) {
    uint8_t climate_popup_acknowledge[] = {k_msg.buf[0], (uint8_t)((k_msg.buf[1] + 1) % 256)};
    kcan_write_msg(make_msg_buf(0x339, 2, climate_popup_acknowledge));                                                              // This reply is required to allow cycling through AUTO blower modes.
  }
}


void evaluate_ihka_auto_ckm(void) {
  uint8_t new_speed = k_msg.buf[0] >> 4;
  if (ihka_auto_fan_speed != new_speed) {
    if (ignition) {
      if (new_speed == 6) {
        send_cc_message("Fan speed: AUTO Low intensity.", true, 4000);
        serial_log("Fan speed CKM AUTO Medium.", 2);
      } else if (new_speed == 5) {
        send_cc_message("Fan speed: AUTO Medium intensity.", true, 4000);
        serial_log("Fan speed CKM AUTO Medium.", 2);
      } else if (new_speed == 9) {
        send_cc_message("Fan speed: AUTO High intensity.", true, 4000);
        serial_log("Fan speed CKM AUTO High.", 2);
      }
    }
    ihka_auto_fan_speed = new_speed;
  }
}


void evaluate_ihka_auto_state(void) {
  if (ignition) {
    uint8_t new_fan_state = k_msg.buf[6] >> 4, 
            new_distr_state = k_msg.buf[4] >> 4;

    if ((ihka_auto_fan_state != new_fan_state) || (ihka_auto_distr_state != new_distr_state)) {
      String climate_message = "";

      if (new_distr_state == 2) {
        climate_message += "Air distribution AUTO";
      } else {
        climate_message += "Air distribution manual";
      }

      if (new_fan_state == 7) {
        if (ihka_auto_fan_speed == 6) {
          climate_message += ", fan speed AUTO Low.";
        } else if (ihka_auto_fan_speed == 5) {
          climate_message += ", fan speed AUTO Mid.";
        } else if (ihka_auto_fan_speed == 9) {
          climate_message += ", fan speed AUTO High.";
        }
      } else {
        climate_message += ", fan speed manual.";
      }

      serial_log(climate_message.c_str(), 3);
      send_cc_message(climate_message.c_str(), true, 2000);
      ihka_auto_fan_state = new_fan_state;
      ihka_auto_distr_state = new_distr_state;
    }
  }
}


void evaluate_ihka_recirculation(void) {
  if (ignition) {
    uint8_t new_state = 0;
    bitWrite(new_state, 0, bitRead(k_msg.buf[0], 4));
    bitWrite(new_state, 1, bitRead(k_msg.buf[0], 5));
    if (ihka_recirc_state != new_state) {
      if (new_state == 0) {
        serial_log("IHKA recirculation OFF.", 2);
        send_cc_message("Air recirculation: OFF.", true, 4000);
      } else if (new_state == 1) {
        serial_log("IHKA recirculation AUTO.", 2);
        send_cc_message("Air recirculation: AUTO.", true, 4000);
      } else if (new_state == 2) {
        serial_log("IHKA recirculation ON.", 2);
        send_cc_message("Air recirculation: ON.", true, 4000);
      }
      ihka_recirc_state = new_state;
    }
  }
}


void evaluate_temperature_unit(void) {
  bitWrite(temperature_unit, 0, bitRead(k_msg.buf[1], 4));
  bitWrite(temperature_unit, 1, bitRead(k_msg.buf[1], 5));
}


void evaluate_terminal_followup(void) {
  if (!terminal_r) {
    if (!(k_msg.buf[0] == 0xFE && k_msg.buf[1] == 0xFF)) {

      terminal30g_followup_time = ((k_msg.buf[1] & 0xF) << 8) | k_msg.buf[0];
      if (terminal30g_followup_time > 0) {
        kl30g_cutoff_imminent = false;
      }

      if (kl30g_cutoff_imminent) {
        serial_log("30G relay will be cut very shortly!.", 2);
      } else {
        sprintf(serial_debug_string, "30G relay will be cut off in %d seconds.", terminal30g_followup_time * 10 + 10);
        serial_log(serial_debug_string, 2);
      }

      if (terminal30g_followup_time <= 0x37 && terminal30g_followup_time > 0x1E) {                                                  // 9.5 minutes.
        // With CIC, the CID is switched off occasionally at 600s and 280s remaining.
        // If the power button is pressed, operation continues until 30G is OFF and the CIC is *forcefully* killed.

        // NBTE will display a warning about sleep depending on the setting of SLEEPDELAY_CLAMP30B_MIN.
        // I.e it will appear at max_terminal30g_followup_time - SLEEPDELAY_CLAMP30B_MIN. E.g 0x5A (900s) - 0x1E (300s) = 600s
        #if F_NBTE
          if (kcan2_mode == MCP_NORMAL && !requested_hu_off_t1 && hu_ent_mode) {
            serial_log("Sent HU OFF at timer1.", 2);
            kcan2_write_msg(dme_request_consumers_off_buf);
            requested_hu_off_t1 = true;
          }
        #endif
      } else if (terminal30g_followup_time == 8) {                                                                                  // 90 second warning.
        #if F_NBTE
          send_cc_message("iDrive switching off in 30s to save battery!", true, 5000);
        #endif
      } else if (terminal30g_followup_time <= 5 && terminal30g_followup_time > 0) {
        #if F_NBTE
          if (kcan2_mode == MCP_NORMAL) {
            if (!requested_hu_off_t2) {
              serial_log("30G cutoff imminent (<60s) Sent HU OFF at timer2.", 2);
              requested_hu_off_t2 = true;                                                                                           // Allow the HU to shut down more gracefully. User requests to wake up will be ignored.
            }
          }
        #endif
      } else if (terminal30g_followup_time == 0) {
        if (!kl30g_cutoff_imminent) {
          serial_log("Received critical 30G timer message. Deep sleep in less than 20s.", 1);
          update_data_in_eeprom();
          kl30g_cutoff_imminent = true;
        }
      }
    }
  }
}


void evaluate_interior_temperature(void) {
  interior_temperature = (k_msg.buf[3] / 6.0);
}


void evaluate_date_time(void) {
  if (k_msg.buf[0] != 0xFD && k_msg.buf[7] != 0xFC) {                                                                               // Invalid time / not set.
    date_time_valid = true;
  } else {
    date_time_valid = false;
  }
  t_hours = k_msg.buf[0];
  t_minutes = k_msg.buf[1];
  t_seconds = k_msg.buf[2];
  d_day = k_msg.buf[3];
  d_month = k_msg.buf[4] >> 4;
  d_year = (k_msg.buf[6] << 8) | k_msg.buf[5];
}


void modify_vehicle_vin(void) {
  #if F_NBTE_CPS_VIN
    if (k_msg.id == 0x380) {                                                                                                        // Set VIN to match HU donor.
      if (donor_vin_initialized) {
        for (uint8_t i = 0; i < k_msg.len; i++) {
          k_msg.buf[i] = DONOR_VIN[i];
        }
      } else {
        return;
      }
    }
  #elif F_KCAN2_VIN
    if (k_msg.id == 0x380) {                                                                                                        // Set KCAN2 VIN to hard-coded value.
      for (uint8_t i = 0; i < k_msg.len; i++) {
        k_msg.buf[i] = KCAN2_VIN[i];
      }
    }
  #endif

  kcan2_write_msg(k_msg);
}
