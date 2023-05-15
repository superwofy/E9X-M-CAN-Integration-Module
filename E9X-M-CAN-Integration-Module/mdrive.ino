void read_settings_from_eeprom() {
  mdrive_dsc = EEPROM.read(1);
  mdrive_power = EEPROM.read(2);
  mdrive_edc = EEPROM.read(3);
  mdrive_svt = EEPROM.read(4);

  // Defaults for when EEPROM is not initialized  
  if (mdrive_dsc == 0xFF || mdrive_power == 0xFF || mdrive_edc == 0xFF || mdrive_svt == 0xFF) {
    mdrive_dsc = 0x13;
    mdrive_power = 0x30;
    mdrive_edc = 0x2A;
    mdrive_svt = 0xF1;
  }

  #if CKM
    dme_ckm[0][0] = EEPROM.read(5);
    dme_ckm[1][0] = EEPROM.read(6);
    dme_ckm[2][0] = EEPROM.read(7);

    // Defaults for when EEPROM is not initialized  
    if (dme_ckm[0][0] == 0xFF || dme_ckm[1][0] == 0xFF || dme_ckm[2][0] == 0xFF) {
      dme_ckm[0][0] = 0xF1;
      dme_ckm[1][0] = 0xF1;
      dme_ckm[2][0] = 0xF1;
    }
  #endif
  #if EDC_CKM_FIX
    edc_ckm[0] = EEPROM.read(8);
    edc_ckm[1] = EEPROM.read(9);
    edc_ckm[2] = EEPROM.read(10);

    // Defaults for when EEPROM is not initialized  
    if (edc_ckm[0] == 0xFF || edc_ckm[1] == 0xFF || edc_ckm[2] == 0xFF) {
      edc_ckm[0] = 0xF1;
      edc_ckm[1] = 0xF1;
      edc_ckm[2] = 0xF1;
    }
  #endif

  serial_log("Loaded MDrive settings from EEPROM.");
  mdrive_message[1] = mdrive_dsc - 2;                                                                                               // Difference between iDrive settting and MDrive CAN message (OFF) is always 2.
                                                                                                                                    // 1 unchanged, 5 OFF, 0x11 MDM, 9 On
  mdrive_message[2] = mdrive_power;                                                                                                 // Copy POWER as is.
  mdrive_message[3] = mdrive_edc;                                                                                                   // Copy EDC as is.
  if (mdrive_svt == 0xE9) {
      mdrive_message[4] = 0x41;                                                                                                     // SVT normal, MDrive OFF.
  } else if (mdrive_svt == 0xF1) {
      mdrive_message[4] = 0x81;                                                                                                     // SVT sport, MDrive OFF.
  } 
}


void update_settings_in_eeprom() {
  if (mdrive_settings_updated) {
    EEPROM.update(1, mdrive_dsc);                                                                                                   // EEPROM lifetime approx. 100k writes. Always update, never write()!                                                                                          
    EEPROM.update(2, mdrive_power);
    EEPROM.update(3, mdrive_edc);
    EEPROM.update(4, mdrive_svt);
    #if CKM
      EEPROM.update(5, dme_ckm[0][0]);
      EEPROM.update(6, dme_ckm[1][0]);
      EEPROM.update(7, dme_ckm[2][0]);
    #endif 
    #if EDC_CKM_FIX
      EEPROM.update(8, edc_ckm[0]);
      EEPROM.update(9, edc_ckm[1]);
      EEPROM.update(10, edc_ckm[2]);
    #endif                                                                                          
    serial_log("Saved M settings to EEPROM.");
  }
}


void toggle_mdrive_message_active() {
  if (mdrive_status) {                                                                                                              // Turn OFF MDrive.
    serial_log("Status MDrive OFF.");
    mdrive_message[1] -= 1;                                                                                                         // Decrement bytes 1 (6MT, DSC mode) and 4 (SVT) to deactivate.
    mdrive_message[4] -= 0x10;
    mdrive_status = mdrive_power_active = false;
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
  }
}


void toggle_mdrive_dsc_mode() {
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


void evaluate_m_mfl_button_press() {
  if (pt_msg.buf[1] == 0x4C) {                                                                                                      // M button is pressed.
    if (!ignore_m_press) {
      ignore_m_press = true;                                                                                                        // Ignore further pressed messages until the button is released.
      toggle_mdrive_message_active();
      send_mdrive_message();
      toggle_mdrive_dsc_mode();
    }
    if (mfl_pressed_count > 10 && !ignore_m_hold) {                                                                                 // Each count is about 100ms
      show_idrive_mdrive_settings_screen();
    } else {
      mfl_pressed_count++;
    }
  } else if (pt_msg.buf[1] == 0xC && pt_msg.buf[0] == 0xC0 && ignore_m_press) {                                                     // Button is released.
    ignore_m_press = ignore_m_hold = false;    
    mfl_pressed_count = 0;
  }
}


void show_idrive_mdrive_settings_screen() {
  if (diag_transmit) {
    serial_log("Steering wheel M button held. Showing settings screen.");
    kcan_write_msg(idrive_mdrive_settings_a_buf);                                                                                   // Send steuern_menu job to iDrive.
    kcan_write_msg(idrive_mdrive_settings_b_buf);
    ignore_m_hold = true;
  }
}


void send_mdrive_message() {
  mdrive_message[0] += 10;
  if (mdrive_message[0] > 0xEF) {                                                                                                   // Alive(first half of byte) must be between 0..E.
    mdrive_message[0] = 0;
  }
  // Deactivated because no module actually checks this. Perhaps MDSC would?
//  can_checksum_update(0x399, 6, mdrive_message);                                                                                  // Recalculate checksum.
  ptcan_write_msg(makeMsgBuf(0x399, 6, mdrive_message));                                                                            // Send to PT-CAN like the DME would. EDC will receive. KOMBI will receive on KCAN through JBE.
  mdrive_message_timer = millis();                                                                   
}


void send_mdrive_alive_message(uint16_t interval) {
  if ((millis() - mdrive_message_timer) >= interval) {                                                                              // Time MDrive alive message outside of CAN loops. Original cycle time is 10s (idle).                                                                     
    if (ignition) {
      serial_log("Sending Ignition ON MDrive alive message.");
    } else {
      serial_log("Sending Vehicle Awake MDrive alive message.");
    }
    send_mdrive_message();
  }
}


void update_mdrive_message_settings() {
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
    mdrive_settings_updated = true;
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received iDrive settings: DSC 0x%X POWER 0x%X EDC 0x%X SVT 0x%X.", 
          mdrive_dsc, mdrive_power, mdrive_edc, mdrive_svt);
      serial_log(serial_debug_string);
    #endif
  }
  send_mdrive_message();
}


void reset_mdrive_settings() {
  mdrive_dsc = 3;                                                                                                                   // Unchanged
  mdrive_message[1] = 1;
  mdrive_power = 0;                                                                                                                 // Unchanged
  mdrive_message[2] = mdrive_power;
  mdrive_edc = 0x20;                                                                                                                // Unchanged
  mdrive_message[3] = mdrive_edc;
  mdrive_svt = 0xE9;                                                                                                                // Normal
  mdrive_message[4] = 0x41;
  #if CKM
    dme_ckm[cas_key_number][0] = 0xF1;                                                                                              // Normal
  #endif
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


void send_power_mode() {
  power_mode_only_dme_veh_mode[0] += 0x10;                                                                                          // Increase alive counter.
  if (power_mode_only_dme_veh_mode[0] > 0xEF) {                                                                                     // Alive(first half of byte) must be between 0..E.
    power_mode_only_dme_veh_mode[0] = 0;
  }

  if (console_power_mode || mdrive_power_active) {                                                                                  // Activate sport throttle mapping if POWER from console ON or Sport/Sport+ selected in MDrive (active).
    power_mode_only_dme_veh_mode[1] = 0xF2;                                                                                         // Sport
    digitalWrite(POWER_LED_PIN, HIGH);
  } else {
    power_mode_only_dme_veh_mode[1] = 0xF1;                                                                                         // Normal
    digitalWrite(POWER_LED_PIN, LOW);
  }

  can_checksum_update(DME_FAKE_VEH_MODE_CANID, 2, power_mode_only_dme_veh_mode);
  ptcan_write_msg(makeMsgBuf(DME_FAKE_VEH_MODE_CANID, 2, power_mode_only_dme_veh_mode));
}


#if CKM
void send_dme_power_ckm() {
  kcan_write_msg(makeMsgBuf(0x3A9, 2, dme_ckm[cas_key_number]));                                                                    // This is sent by the DME to populate the M Key iDrive section
  serial_log("Sent DME POWER CKM.");
  dme_ckm_sent = true;
}


void check_ckm_queue() {
  if (!dme_ckm_tx_queue.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    dme_ckm_tx_queue.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      dme_ckm_tx_queue.drop();
    }
  }
}


void update_dme_power_ckm() {
  dme_ckm[cas_key_number][0] = k_msg.buf[0];
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Received new POWER CKM setting: %s for key number %d", 
            k_msg.buf[0] == 0xF1 ? "Normal" : "Sport", cas_key_number);
    serial_log(serial_debug_string);
  #endif
  mdrive_settings_updated = true;
  send_dme_power_ckm();                                                                                                             // Acknowledge settings received from iDrive;
}
#endif


#if CKM || EDC_CKM_FIX
void evaluate_key_number() {
  if (k_msg.buf[0] / 11 != cas_key_number) {
    cas_key_number = k_msg.buf[0] / 11;                                                                                             // Key 1 = 0, Key 2 = 11, Key 3 = 22...
    if (cas_key_number > 2) {
      serial_log("Received wrong key personalisation number. Assuming default");
      cas_key_number = 3;                                                                                                           // Default.
    }
    #if DEBUG_MODE
    else {
      sprintf(serial_debug_string, "Received key number: %d.", cas_key_number + 1);
      serial_log(serial_debug_string);
    }
    #endif
    #if CKM
      console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;
    #endif
  }
}
#endif


#if EDC_CKM_FIX
void update_edc_ckm() {
  edc_ckm[cas_key_number] = k_msg.buf[0];
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Received new EDC CKM setting: %s for key %d.", k_msg.buf[0] == 0xF1 ? "Comfort" : 
                                  k_msg.buf[0] == 0xF2 ? "Normal" : "Sport", cas_key_number);
    serial_log(serial_debug_string);
  #endif
  mdrive_settings_updated = true;
}


void evaluate_edc_ckm_mismatch() {
  if (edc_mismatch_check_counter < 2) {
    if (pt_msg.buf[1] != edc_ckm[cas_key_number]) {
      serial_log("EDC EEPROM CKM setting match the current value. Correcting.");
      uint8_t edc_state = pt_msg.buf[1] == 0xFA ? 3 : pt_msg.buf[1] - 0xF0;                                                           // Normalize these values for easier comparison.
      uint8_t edc_memory = edc_ckm[cas_key_number] == 0xFA ? 3 : edc_ckm[cas_key_number] - 0xF0;
      if ((edc_memory == 1 && edc_state == 2) || (edc_memory == 2 && edc_state == 3) || (edc_memory == 3 && edc_state == 1)) {
        unsigned long timeNow = millis();
        kcan_write_msg(edc_button_press_buf);
        delayed_can_tx_msg m = {edc_button_press_buf, timeNow + 1200};
        edc_ckm_txq.push(&m);
      } else {
        kcan_write_msg(edc_button_press_buf);
      }
    }
    edc_mismatch_check_counter++;
  }
}


void check_edc_ckm_queue() {
  if (!edc_ckm_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    edc_ckm_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      edc_ckm_txq.drop();
    }
  }
}
#endif
