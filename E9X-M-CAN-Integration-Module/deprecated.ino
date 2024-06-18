// Functions that are no longer tested go here. I probably no longer use the necessary hardware anymore.


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


void send_initial_volume_cic(void) {
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


void send_f_driving_dynamics_switch_nbt(void) {
  if (f_driving_dynamics_timer >= 1001) {
    uint8_t f_driving_dynamics[] = {0xFF, 0xFF, 0, 0, 0, 0, 0xC0};                                                                  // Inspired very loosely by 272.4.8...
    uint8_t new_driving_mode = driving_mode;

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
