// Functions relating to the headunit go here.


void send_zbe_acknowledge(void) {
  uint8_t zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  zbe_response[2] = k_msg.buf[7];
  #if F_NBT_CCC_ZBE
    kcan2_write_msg(make_msg_buf(0x277, 4, zbe_response));
    kcan_write_msg(ccc_zbe_wake_buf);                                                                                               // Reset the ZBE counter upon HU's request.
  #else
    kcan_write_msg(make_msg_buf(0x277, 4, zbe_response));
  #endif
}


void send_volume_request_door(void) {
  if (terminal_r) {
    if (diag_transmit) {
      if (!idrive_txq.isEmpty()) {                                                                                                  // If there are pending volume change actions we need to wait for them to complete.                                                                                           
        m = {vol_request_buf, millis() + 900};
        idrive_txq.push(&m);
      } else {
        if (volume_request_door_timer >= 300) {                                                                                     // Basic debounce to account for door not fully shut.
          #if F_NBT
            kcan2_write_msg(vol_request_buf);
          #else
            kcan_write_msg(vol_request_buf);
          #endif
          volume_request_door_timer = volume_request_periodic_timer = 0;
          serial_log("Requested volume from iDrive with door status change.", 2);
        }
      }
    }
  }
}


void evaluate_audio_volume_nbt(void) {
  if (k_msg.buf[0] == 0xF1 && k_msg.buf[4] == 0xA0 && k_msg.buf[5] == 0x39) {                                                      // status_volumeaudio response.
    if (k_msg.buf[6] > 0) {                                                                                                        // Otherwise, probably muted.
      uint8_t volume_change[] = {0x63, 6, 0x31, 1, 0xA0, 0x36, 0, 0};
      if (!volume_reduced) {
        if (peristent_volume != k_msg.buf[6]) {
          peristent_volume = k_msg.buf[6];
          #if DEBUG_MODE
            sprintf(serial_debug_string, "Received new audio volume: 0x%X.", k_msg.buf[6]);
            serial_log(serial_debug_string, 3);
          #endif
        }
        if (pdc_tone_on || gong_active) {                                                                                           // If PDC beeps are active, volume change has no effect.
          return;
        }
        if (k_msg.buf[6] >= 5) {                                                                                                    // Don't reduce if already very low.
          if (left_door_open || right_door_open) {
            volume_change[6] = floor(k_msg.buf[6] * 0.75);                                                                          // Softer decrease.
            if (volume_change[6] > 0x33) {
              volume_change[6] = 0x33;
            }
            unsigned long time_now = millis();
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 200};
            idrive_txq.push(&m);
            volume_restore_offset = (k_msg.buf[6] % 2) == 0 ? 0 : 1;                                                                // Volumes adjusted from faceplate go up by 1 while MFL goes up by 2.
            volume_change[6] = floor(k_msg.buf[6] / 2);                                                                             // Reduce volume to 50%.
            if ((volume_change[6] + volume_restore_offset) > 0x33) {                                                                // Don't blow the speakers out if something went wrong...
              volume_change[6] = 0x33;
              volume_restore_offset = 0;
            }
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 600};
            idrive_txq.push(&m);
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 900};
            idrive_txq.push(&m);
            volume_reduced = true;
            volume_changed_to = volume_change[6];                                                                                   // Save this value to compare when door is closed back.
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Reducing audio volume with door open to: 0x%X.", volume_changed_to);
              serial_log(serial_debug_string, 3);
            #endif
          }
        }
      } else {
        peristent_volume = k_msg.buf[6] * 2 + volume_restore_offset;
        if (!left_door_open && !right_door_open) {
          if (k_msg.buf[6] == volume_changed_to) {
            volume_change[6] = floor(k_msg.buf[6] * 1.5);                                                                           // Softer increase.
            if (volume_change[6] > 0x33) {
              volume_change[6] = 0x33;
            }
            unsigned long time_now = millis();
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 200};
            idrive_txq.push(&m);
            volume_change[6] = k_msg.buf[6] * 2 + volume_restore_offset;
            if (volume_change[6] > 0x33) {
              volume_change[6] = 0x33;                                                                                              // Set a nanny in case the code goes wrong. 0x33 is pretty loud...
            }
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 600};
            idrive_txq.push(&m);
            m = {make_msg_buf(0x6F1, 8, volume_change), time_now + 900};                                                            // Make sure the restore is received.
            idrive_txq.push(&m);
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Restoring audio volume with door closed. to: 0x%X.", volume_change[6]);
              serial_log(serial_debug_string, 3);
            #endif
          } else {
            peristent_volume = k_msg.buf[6];                                                                                        // User changed volume while door was opened.
            #if DEBUG_MODE
              sprintf(serial_debug_string, "Volume changed by user while door was open to: 0x%X.", k_msg.buf[6]);
              serial_log(serial_debug_string, 3);
            #endif
          }
          volume_reduced = false;
        }
      }
    }
  }
}


void check_idrive_queue(void) {
  if (!idrive_txq.isEmpty()) {
    if (diag_transmit) {
      idrive_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        if (vehicle_awake) {
          #if F_NBT
            kcan2_write_msg(delayed_tx.tx_msg);
            if (delayed_tx.tx_msg.buf[4] == 0xA0 && delayed_tx.tx_msg.buf[5] == 0x36) {
              serial_log("Sent volume change job to iDrive.", 2);
            }
          #else
            kcan_write_msg(delayed_tx.tx_msg);
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
    if (idrive_alive_timer >= 3000) {                                                                                               // This message should be received every 1-2s.
      if (!idrive_died) {
        idrive_died = true;
        serial_log("iDrive booting/rebooting.", 2);
        initial_volume_set = false;
        asd_initialized = false;
      }
    } else {
      if (idrive_died) {                                                                                                            // It's back.
        idrive_died = false;
      }
    }
  }
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



void evaluate_vsw_position_request() {
  vsw_current_input = k_msg.buf[0];
  #if DEBUG_MODE
    sprintf(serial_debug_string, "NBT sent VSW/%d (%s) request.", vsw_current_input, vsw_positions[vsw_current_input]);
    serial_log(serial_debug_string, 3);
  #endif
}


void send_nbt_vin_request(void) {
  if (!donor_vin_initialized) {
    if (nbt_vin_request_timer >= 3000) {
      kcan2_write_msg(nbt_vin_request_buf);
      nbt_vin_request_timer = 0;
      requested_donor_vin = true;
    }
  }
}


void evaluate_nbt_vin_response(void) {
  if (requested_donor_vin) {
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10 && k_msg.buf[4] == 0xF1 && k_msg.buf[5] == 0x90) {                             // Received the first part of the VIN response. Send acknowledge
      uint8_t vin_continue_response[] = {0x63, 0x30, 0x08, 0x14, k_msg.buf[4], k_msg.buf[5], k_msg.buf[6], k_msg.buf[7]};
      kcan2_write_msg(make_msg_buf(0x6F1, 8, vin_continue_response));
      receiving_donor_vin = true;
    } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x22 && receiving_donor_vin) {
      DONOR_VIN[0] = k_msg.buf[4];
      DONOR_VIN[1] = k_msg.buf[5];
      DONOR_VIN[2] = k_msg.buf[6];
      DONOR_VIN[3] = k_msg.buf[7];
    } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x23 && receiving_donor_vin) {
      DONOR_VIN[4] = k_msg.buf[2];
      DONOR_VIN[5] = k_msg.buf[3];
      DONOR_VIN[6] = k_msg.buf[4];
      receiving_donor_vin = false;
      requested_donor_vin = false;
      donor_vin_initialized = true;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Received NBT donor VIN: %c%c%c%c%c%c%c.", DONOR_VIN[0], DONOR_VIN[1], DONOR_VIN[2],
                                      DONOR_VIN[3], DONOR_VIN[4], DONOR_VIN[5], DONOR_VIN[6]);
        serial_log(serial_debug_string, 2);
      #endif
    }
  }
}


void evaluate_faceplate_buttons(void) {
  if (!digitalRead(FACEPLATE_EJECT_PIN)) {
    if (!faceplate_eject_pressed) {
      if (faceplate_eject_debounce_timer >= 1000) {
        faceplate_eject_pressed = true;
        faceplate_eject_pressed_timer = 0;
      }
    }
  } else {
    if (faceplate_eject_pressed) {
      if (faceplate_eject_pressed_timer >= 20) {                                                                                    // These inputs are very noisy. Make sure they're actually pressed.
        unsigned long time_now = millis();
        m = {faceplate_eject_buf, time_now};
        faceplate_buttons_txq.push(&m);
        time_now += 200;
        m = {faceplate_a1_released_buf, time_now};
        faceplate_buttons_txq.push(&m);
        serial_log("Faceplate eject button pressed.", 0);
        faceplate_eject_debounce_timer = 0;
      }
      faceplate_eject_pressed = false;
    }
  }
  
  if (!digitalRead(FACEPLATE_POWER_MUTE_PIN)) {
    if (!faceplate_power_mute_pressed) {
      if (faceplate_power_mute_debounce_timer >= 150) {
        faceplate_power_mute_pressed = true;
        faceplate_power_mute_pressed_timer = 0;
      }
    }
  } else {
    if (faceplate_power_mute_pressed) {
      if (faceplate_power_mute_pressed_timer >= 20) {
        unsigned long time_now = millis();
        m = {faceplate_power_mute_buf, time_now};
        faceplate_buttons_txq.push(&m);
        time_now += 100;
        m = {faceplate_a1_released_buf, time_now};
        faceplate_buttons_txq.push(&m);
        serial_log("Faceplate power/mute button pressed.", 0);
        faceplate_power_mute_debounce_timer = 0;
      }
      faceplate_power_mute_pressed = false;
    }
  }
}


void check_faceplate_buttons_queue(void) {
  if (!faceplate_buttons_txq.isEmpty()) {
    faceplate_buttons_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan2_write_msg(delayed_tx.tx_msg);
      faceplate_buttons_txq.drop();
    }
  }
}


void evaluate_faceplate_uart(void) {
  #if F_NBT
    uint8_t faceplate_status_message[7] = {0};

    if (vehicle_awakened_timer >= 500) {
      if (FACEPLATE_UART.available() == 7) {                                                                                        // Wait for the full message to come in. Cycle time 1s.
        for (uint8_t i = 0; i < 7; i++) {
          faceplate_status_message[i] = FACEPLATE_UART.read();
        }

        if (faceplate_status_message[0] != 0x55) {                                                                                  // Faceplate is in some error state which should be ignored.
          FACEPLATE_UART.clear();
          return;
        }

        if (faceplate_status_message[2] == 0x80) {                                                                                  // Seeking right
          kcan2_write_msg(faceplate_seek_right_buf);
        } else if (faceplate_status_message[2] == 0x40) {                                                                           // Seeking left
          kcan2_write_msg(faceplate_seek_left_buf);
        } else {                                                                                                                    // Released
          kcan2_write_msg(faceplate_a3_released_buf);
        }

        uint16_t faceplate_new_volume = (faceplate_status_message[6] << 8) | faceplate_status_message[4];
        if (faceplate_new_volume != faceplate_volume) {
            int16_t volume_difference = faceplate_new_volume - faceplate_volume;

            if (volume_difference > 0 && volume_difference <= 0x7FFF) {                                                             // Clockwise rotation
                kcan2_write_msg(faceplate_volume_decrease_buf);
            } else if (volume_difference < 0 && volume_difference >= -0x7FFF) {                                                     // Counterclockwise rotation
                kcan2_write_msg(faceplate_volume_increase_buf);
            } else {                                                                                                                // Handle wraparound
                if (volume_difference > 0) {
                    kcan2_write_msg(faceplate_volume_decrease_buf);
                } else {
                    kcan2_write_msg(faceplate_volume_increase_buf);
                }
            }

            faceplate_volume = faceplate_new_volume;
        } else {
            kcan2_write_msg(faceplate_f1_released_buf);
        }

        if (faceplate_status_message[3] == 0x80) {
          if (faceplate_status_message[5] == 0x80) {
            kcan2_write_msg(faceplate_button1_press_buf);
          } else {
            kcan2_write_msg(faceplate_button1_hover_buf);
          }
        } else if (faceplate_status_message[3] == 0x40) {
          if (faceplate_status_message[5] == 0x40) {
            kcan2_write_msg(faceplate_button2_press_buf);
          } else {
            kcan2_write_msg(faceplate_button2_hover_buf);
          }
        } else if (faceplate_status_message[3] == 0x20) {
          if (faceplate_status_message[5] == 0x20) {
            kcan2_write_msg(faceplate_button3_press_buf);
          } else {
            kcan2_write_msg(faceplate_button3_hover_buf);
          }
        } else if (faceplate_status_message[3] == 0x10) {
          if (faceplate_status_message[5] == 0x10) {
            kcan2_write_msg(faceplate_button4_press_buf);
          } else {
            kcan2_write_msg(faceplate_button4_hover_buf);
          }
        } else if (faceplate_status_message[3] == 8) {
          if (faceplate_status_message[5] == 8) {
            kcan2_write_msg(faceplate_button5_press_buf);
          } else {
            kcan2_write_msg(faceplate_button5_hover_buf);
          }
        } else if (faceplate_status_message[3] == 4) {
          if (faceplate_status_message[5] == 4) {
            kcan2_write_msg(faceplate_button6_press_buf);
          } else {
            kcan2_write_msg(faceplate_button6_hover_buf);
          }
        } else if (faceplate_status_message[3] == 2) {
          if (faceplate_status_message[5] == 2) {
            kcan2_write_msg(faceplate_button7_press_buf);
          } else {
            kcan2_write_msg(faceplate_button7_hover_buf);
          }
        } else if (faceplate_status_message[3] == 1) {
          if (faceplate_status_message[5] == 1) {
            kcan2_write_msg(faceplate_button8_press_buf);
          } else {
            kcan2_write_msg(faceplate_button8_hover_buf);
          }
        } else {
          kcan2_write_msg(faceplate_a2_released_buf);
        }
      }
    } else {
      FACEPLATE_UART.clear();                                                                                                       // This data is garbage. Clear the RX buffer.
    }
  #endif
}


void convert_zbe1_message(void) {
  #if F_NBT_CCC_ZBE
    zbe_action_counter = (zbe_action_counter + 1) % 0x100;                                                                          // Increase counter and reset when past 0xFF.
    zbe_buttons[2] = zbe_action_counter;

    uint16_t current_position = k_msg.buf[3] << 8 | k_msg.buf[2];                                                                   // Correct ZBE counter starting at 1 instead of 0.
    current_position = (current_position - 1) % 0x10000;
    k_msg.buf[2] = current_position & 0xFF;
    k_msg.buf[3] = current_position >> 8;

    if (zbe_rotation[3] != (k_msg.buf[2])) {
      zbe_rotation[2] = zbe_action_counter;
      zbe_rotation[3] = k_msg.buf[2];
      zbe_rotation[4] = (0x80 + k_msg.buf[3]) % 0x100;
      kcan2_write_msg(make_msg_buf(0x264, 6, zbe_rotation));
      return;
    }

    if (k_msg.buf[1] == 0xC4 || k_msg.buf[1] == 0xC5) {                                                                             // MENU
      zbe_buttons[3] = 1;
      zbe_buttons[4] = 0xC0;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    } else if (k_msg.buf[0] == 0 && k_msg.buf[1] == 0xC0) {                                                                         // Joystick north
      zbe_buttons[3] = 0x11;
      zbe_buttons[4] = 0xDD;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    } else if (k_msg.buf[0] == 4 && k_msg.buf[1] == 0xC0) {                                                                         // Joystick south
      zbe_buttons[3] = 0x41;
      zbe_buttons[4] = 0xDD;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    } else if (k_msg.buf[0] == 6 && k_msg.buf[1] == 0xC0) {                                                                         // Joystick east
      zbe_buttons[3] = 0x81;
      zbe_buttons[4] = 0xDD;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    } else if (k_msg.buf[0] == 2 && k_msg.buf[1] == 0xC0) {                                                                         // Joystick west
      zbe_buttons[3] = 0x21;
      zbe_buttons[4] = 0xDD;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    } else if (k_msg.buf[1] == 0xC1) {                                                                                              // Joystick pressed
      zbe_buttons[3] = 1;
      zbe_buttons[4] = 0xDE;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    } else {                                                                                                                        // Released
      zbe_buttons[3] = 0;
      kcan2_write_msg(make_msg_buf(0x267, 6, zbe_buttons));
    }
  #endif
}


void evaluate_cc_gong_status(void) {
  if (k_msg.buf[0] != 0xF0) {
    gong_active = true;
  } else {
    gong_active = false;
  }
}


void evaluate_idrive_units(void) {
  uint8_t new_torque_unit = k_msg.buf[4] >> 4, new_power_unit = k_msg.buf[4] & 0xF, new_pressure_unit = k_msg.buf[3] & 0xF;
  if (new_torque_unit > 0) {
    torque_unit[cas_key_number] = new_torque_unit;
  }
  if (new_power_unit > 0) {
    power_unit[cas_key_number] = new_power_unit;
  }
  if (new_pressure_unit > 0) {
    pressure_unit[cas_key_number] = new_pressure_unit;
  }
  #if DEBUG_MODE
      sprintf(serial_debug_string, "Received iDrive units: %s/%s/%s.", 
              torque_unit[cas_key_number] == 1 ? "Nm" : (torque_unit[cas_key_number] == 2 ? "lb-ft" : (torque_unit[cas_key_number] == 3 ? "Kg-m" : "-")), 
              power_unit[cas_key_number] == 1 ? "kW" :  (power_unit[cas_key_number] == 2 ? "hp" : "-"),
              pressure_unit[cas_key_number] == 1 ? "bar" : (pressure_unit[cas_key_number] == 2 ? "kPa" : (pressure_unit[cas_key_number] == 3 ? "psi" : "-")));
      serial_log(serial_debug_string, 3);
  #endif
  convert_f_units(true);
  send_nbt_sport_displays_data(false);                                                                                              // Update the scale for the new units.
}


void convert_f_units(bool idrive_units_only) {
  #if F_NBT
    if (!idrive_units_only) {
      f_units[0] = k_msg.buf[0];
      f_units[1] = k_msg.buf[1];
      f_units[2] = k_msg.buf[2];
    }
    f_units[3] = (k_msg.buf[3] & 0xF0) | pressure_unit[cas_key_number];
    f_units[4] = (torque_unit[cas_key_number] << 4) | power_unit[cas_key_number];                                                   // The units message is mostly the same with the addition of units in Byte4.
    kcan2_write_msg(make_msg_buf(0x2F7, 6, f_units));
  #endif
}


void send_cc_message_text(const char input[], uint16_t dismiss_time) {

  #if F_NBT_EVO6
    return;       // Disable for now.
  #endif

  nbt_cc_txq.flush();                                                                                                               // Clear any pending dimiss messages.
  uint8_t input_length = strlen(input);
  uint8_t padded_length = input_length + (3 - (input_length % 3)) % 3;
  char padded_input[padded_length];
  memset(padded_input, ' ', padded_length);
  strncpy(padded_input, input, input_length);
  
  uint8_t cc_message_chunk_counter = 0xDF;
  for (uint8_t i = 0; i < (padded_length / 3); i++) {
    cc_message_chunk_counter = (cc_message_chunk_counter + 1) % 256;
    #if F_NBT_EVO6
      uint8_t cc_message_text[] = {0x46, 3, 0x32, 0xF0, cc_message_chunk_counter,
                                  padded_input[3 * i], padded_input[(3 * i) + 1], padded_input[(3 * i) + 2]};
    #else
      uint8_t cc_message_text[] = {0x46, 3, 0x72, 0xF0, cc_message_chunk_counter,
                                  padded_input[3 * i], padded_input[(3 * i) + 1], padded_input[(3 * i) + 2]};
    #endif
    kcan2_write_msg(make_msg_buf(0x338, 8, cc_message_text));
  }
  
  cc_message_chunk_counter = (cc_message_chunk_counter + 3) % 256;
  #if F_NBT_EVO6
    uint8_t cc_message_text_end[] = {0x46, 3, 0x32, 0xF0, cc_message_chunk_counter, 0x20, 0x20, 0x20};
  #else
    uint8_t cc_message_text_end[] = {0x46, 3, 0x72, 0xF0, cc_message_chunk_counter, 0x20, 0x20, 0x20};
  #endif
  kcan2_write_msg(make_msg_buf(0x338, 8, cc_message_text_end));
  cc_message_text_end[4] = (cc_message_chunk_counter + 2) % 256;
  kcan2_write_msg(make_msg_buf(0x338, 8, cc_message_text_end));
  if (dismiss_time > 0) {
    unsigned long time_now = millis();
    m = {custom_cc_dismiss_buf, time_now + dismiss_time};
    nbt_cc_txq.push(&m);
  }
}


void check_nbt_cc_queue(void) {
  if (!nbt_cc_txq.isEmpty()) {
    nbt_cc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan2_write_msg(delayed_tx.tx_msg);
      nbt_cc_txq.drop();
    }
  }
}
