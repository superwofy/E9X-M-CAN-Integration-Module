// Functions that are no longer tested go here. I probably no longer use the necessary hardware anymore.


void set_kcan_filters(uint8_t *filter_set_ok_counter, uint8_t *filter_position_counter) {
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0xAA, STD);                                                // RPM, throttle pos:                                           Cycle time 100ms (KCAN).
  *filter_position_counter = *filter_position_counter + 1; 
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0xEA, STD);                                                // Driver's door status.
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x130, STD);                                               // Key/ignition status:                                         Cycle time 100ms.
  *filter_position_counter = *filter_position_counter + 1;
  #if HDC
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x193, STD);                                             // Kombi cruise control status:                                 Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if FAKE_MSA || MSA_RVC
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x195, STD);                                             // MSA button status sent by IHKA:                              Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x1AA, STD);                                               // iDrive ErgoCommander (rear entertainment?):                  Sent at boot time and when cycling Terminal R.
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x1B4, STD);                                               // Kombi status (indicated speed, handbrake):                   Cycle time 100ms (terminal R ON).
  *filter_position_counter = *filter_position_counter + 1;
  #if REVERSE_BEEP || DOOR_VOLUME
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x1C6, STD);                                             // PDC acoustic message
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if MIRROR_UNDIM
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x1EE, STD);                                             // Indicator stalk status from FRM (KCAN only).
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x1F6, STD);                                               // Indicator status:                                            Cycle time 1s. Sent when changed.
  *filter_position_counter = *filter_position_counter + 1;
  #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER || DIM_DRL
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x21A, STD);                                             // FRM Light status:                                            Cycle time 5s (idle). Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if AUTO_SEAT_HEATING_PASS
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x22A, STD);                                             // Passenger's seat heating status:                             Cycle time 10s (idle), 150ms (change).
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if AUTO_SEAT_HEATING
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x232, STD);                                             // Driver's seat heating status:                                Cycle time 10s (idle), 150ms (change).
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x23A, STD);                                               // Remote button and number sent by CAS:                        Sent 3x when changed.
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x273, STD);                                               // iDrive status and ZBE challenge:                             Sent when iDrive is idle or a button is pressed on the ZBE.
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x2CA, STD);                                               // Ambient temperature:                                         Cycle time 1s.
  *filter_position_counter = *filter_position_counter + 1;                                                                         
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x2F7, STD);                                               // Units from KOMBI:                                            Sent 3x on Terminal R. Sent when changed.
  *filter_position_counter = *filter_position_counter + 1;
  #if AUTO_SEAT_HEATING_PASS
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x2FA, STD);                                             // Seat occupancy and belt status:                              Cycle time 5s.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if DOOR_VOLUME || AUTO_MIRROR_FOLD || IMMOBILIZER_SEQ || HOOD_OPEN_GONG
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x2FC, STD);                                             // Door, hood status sent by CAS:                               Cycle time 5s. Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if F_NIVI || MIRROR_UNDIM || FRONT_FOG_CORNER
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x314, STD);                                             // RLS light status:                                            Cycle time 3s. Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if MSA_RVC
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x317, STD);                                             // Monitor PDC button status:                                   Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if HDC
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x31A, STD);                                             // HDC button status sent by IHKA:                              Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if PDC_AUTO_OFF
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x34F, STD);                                             // Handbrake status sent by JBBFE:                              Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if AUTO_TOW_VIEW_RVC
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x36D, STD);                                             // Distance status sent by PDC:                                 Sent when active.
    *filter_position_counter = *filter_position_counter + 1;
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x38F, STD);                                             // Camera settings sent by iDrive:                              Sent when activating camera and when changed.
    *filter_position_counter = *filter_position_counter + 1;
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x39B, STD);                                             // Camera settings status/acknowledge sent by TRSVC:            Sent when activating ignition and when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3A8, STD);                                               // M Key (CKM) POWER setting from iDrive:                       Sent when changed.
  *filter_position_counter = *filter_position_counter + 1;
  #if MSA_RVC || PDC_AUTO_OFF || AUTO_TOW_VIEW_RVC
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3AF, STD);                                             // PDC bus status:                                              Cycle time 2s (idle). Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3B0, STD);                                               // Reverse gear status:                                         Cycle time 1s (idle).
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3BD, STD);                                               // FRM consumer shutdown:                                       Cycle time 5s (idle). Sent when changed.
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3CA, STD);                                               // iDrive MDrive settings:                                      Sent when changed.
  *filter_position_counter = *filter_position_counter + 1;
  *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3D7, STD);                                               // Door lock CKM settings from DWA:                             Sent when changed.
  *filter_position_counter = *filter_position_counter + 1;
  #if COMFORT_EXIT
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3DB, STD);                                             // Seat CKM settings:                                           Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if DIM_DRL
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x3DD, STD);                                             // Lights CKM settings:                                         Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if F_ZBE_KCAN1 || F_VSW01 || F_NIVI
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x4E0, STD);                                             // Kombi Network management:                                    Sent when Kombi is ON, cycle time 500ms.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if DOOR_VOLUME
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x5C0, STD);                                             // CAS CC notifications:                                        Sent when changed.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if DEBUG_MODE
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x640, STD);                                             // CAS diagnostic responses:                                    Sent when response is requested.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if F_VSW01
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x648, STD);                                             // VSW diagnostic responses:                                    Sent when response is requested.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if F_NIVI
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x650, STD);                                             // SINE diagnostic responses:                                   Sent when response is requested.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if DOOR_VOLUME || F_VSW01
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x663, STD);                                             // iDrive diagnostic responses:                                 Sent when response is requested.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  #if AUTO_MIRROR_FOLD || FRONT_FOG_CORNER
    *filter_set_ok_counter += KCAN.setFIFOFilter(*filter_position_counter, 0x672, STD);                                             // FRM diagnostic responses:                                    Sent when response is requested.
    *filter_position_counter = *filter_position_counter + 1;
  #endif
  KCAN.setFIFOFilter(REJECT_ALL, *filter_position_counter);                                                                         // Reject unfiltered messages.
}


void update_mdrive_message_settings_cic(void) {
  if (k_msg.buf[4] == 0xEC || k_msg.buf[4] == 0xF4 || k_msg.buf[4] == 0xE4) {                                                       // Reset requested.
    reset_mdrive_settings();
  } else if ((k_msg.buf[4] == 0xE0 || k_msg.buf[4] == 0xE1)) {                                                                      // Ignore E0/E1 (Invalid).
  } else {
    //Decode settings
    mdrive_dsc[cas_key_number] = k_msg.buf[0];                                                                                      // 3 unchanged, 7 OFF, 0x13 MDM, 0xB ON.
    mdrive_power[cas_key_number] = k_msg.buf[1];                                                                                    // 0 unchanged, 0x10 normal, 0x20 sport, 0x30 sport+.
    mdrive_edc[cas_key_number] = k_msg.buf[2];                                                                                      // 0x20(Unchanged), 0x21(Comfort) 0x22(Normal) 0x2A(Sport).
    mdrive_svt[cas_key_number] = k_msg.buf[4];                                                                                      // 0xE9 Normal, 0xF1 Sport, 0xEC/0xF4/0xE4 Reset. E0/E1-invalid?

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


void evaluate_audio_volume_cic(void) {
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
        if (pdc_tone_on || gong_active) {                                                                                           // If PDC beeps are active, volume change has no effect.
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
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      #if F_NBT
        kcan2_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));
      #else
        kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));
      #endif
      initial_volume_set = true;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Sent saved initial volume (0x%X) to iDrive after receiving volume 0.", peristent_volume);     // 0 means that the vol knob wasn't used / initial job was not sent since iDrive boot.
        serial_log(serial_debug_string, 2);
      #endif
    }
  }
}


void send_initial_volume(void) {
  if (k_msg.buf[7] >= 3) {                                                                                                          // 0x273 has been transmitted X times according to the counter.
    if (!initial_volume_set && diag_transmit) {
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      #if F_NBT
        kcan2_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));
      #else
        kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));                                                                // Set iDrive volume to last volume before sleep. This must run before any set volumes.
      #endif
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Sent saved sleep volume (0x%X) to iDrive after boot/reboot.", peristent_volume);
        serial_log(serial_debug_string, 2);
      #endif
      initial_volume_set = true;
    }
  }
}
