// Functions that are no longer tested go here. I probably no longer use the necessary hardware anymore.


void update_mdrive_message_settings_cic(void) {
  if (ignition) {
    if (k_msg.buf[4] == 0xEC || k_msg.buf[4] == 0xF4 || k_msg.buf[4] == 0xE4) {                                                     // Reset requested.
      reset_mdrive_settings();
    } else if ((k_msg.buf[4] == 0xE0 || k_msg.buf[4] == 0xE1)) {                                                                    // Ignore E0/E1 (Invalid).
    } else {
      //Decode settings
      mdrive_dsc[cas_key_number] = k_msg.buf[0];                                                                                    // 3 unchanged, 7 OFF, 0x13 MDM, 0xB ON.
      mdrive_power[cas_key_number] = k_msg.buf[1];                                                                                  // 0 unchanged, 0x10 normal, 0x20 sport, 0x30 sport+.
      mdrive_edc[cas_key_number] = k_msg.buf[2];                                                                                    // 0x20(Unchanged), 0x21(Comfort) 0x22(Normal) 0x2A(Sport).
      mdrive_svt[cas_key_number] = k_msg.buf[4];                                                                                    // 0xE9 Normal, 0xF1 Sport, 0xEC/0xF4/0xE4 Reset. E0/E1-invalid?

      sprintf(serial_debug_string, "Received iDrive settings: DSC 0x%X POWER 0x%X EDC 0x%X SVT 0x%X.", 
          mdrive_dsc[cas_key_number], mdrive_power[cas_key_number], mdrive_edc[cas_key_number], mdrive_svt[cas_key_number]);
      serial_log(serial_debug_string, 3);
      
      update_mdrive_settings_can_message();
      execute_mdrive_settings_changed_actions();
    }
    mdrive_message_bn2000_timer = 10000;
  }
}


void evaluate_audio_volume_cic(void) {
  if (k_msg.buf[0] == 0xF1 && k_msg.buf[3] == 0x24) {                                                                               // status_volumeaudio response.
    if (k_msg.buf[4] > 0) {
      uint8_t volume_change[] = {0x63, 4, 0x31, 0x23, 0, 0, 0, 0};
      if (!volume_reduced) {
        if (peristent_volume != k_msg.buf[4]) {
          peristent_volume = k_msg.buf[4];
          sprintf(serial_debug_string, "Received new audio volume: 0x%X.", k_msg.buf[4]);
          serial_log(serial_debug_string, 3);
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
            sprintf(serial_debug_string, "Reducing audio volume with door open to: 0x%X.", volume_changed_to);
            serial_log(serial_debug_string, 3);
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
            sprintf(serial_debug_string, "Restoring audio volume with door closed. to: 0x%X.", volume_change[4]);
            serial_log(serial_debug_string, 3);
          } else {
            peristent_volume = k_msg.buf[4];                                                                                        // User changed volume while door was opened.
            sprintf(serial_debug_string, "Volume changed by user while door was open to: 0x%X.", k_msg.buf[4]);
            serial_log(serial_debug_string, 3);
          }
          volume_reduced = false;
        }
      }
    } else {
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));
      initial_volume_set = true;
      sprintf(serial_debug_string, "Sent saved initial volume (0x%X) to iDrive after receiving volume 0.", peristent_volume);       // 0 means that the vol knob wasn't used / initial job was not sent since iDrive boot.
      serial_log(serial_debug_string, 2);
    }
  }
}


void send_initial_volume_cic(void) {
  if (k_msg.buf[7] >= 3) {                                                                                                          // 0x273 has been transmitted X times according to the counter.
    if (!initial_volume_set && diag_transmit) {
      uint8_t restore_last_volume[] = {0x63, 4, 0x31, 0x23, peristent_volume, 0, 0, 0};
      kcan_write_msg(make_msg_buf(0x6F1, 8, restore_last_volume));                                                                  // Set iDrive volume to last volume before sleep. This must run before any set volumes.
      sprintf(serial_debug_string, "Sent saved sleep volume (0x%X) to iDrive after boot/reboot.", peristent_volume);
      serial_log(serial_debug_string, 2);
      initial_volume_set = true;
    }
  }
}


void send_dme_power_ckm(void) {
  if (!frm_consumer_shutdown) {
    ptcan_write_msg(make_msg_buf(0x3A9, 2, dme_ckm[cas_key_number]));                                                               // This is sent by the DME to populate the M Key iDrive section
    serial_log("Sent DME POWER CKM.", 3);
  }
}


void update_dme_power_ckm(void) {
  dme_ckm[cas_key_number][0] = k_msg.buf[0];
  sprintf(serial_debug_string, "Received new POWER CKM setting: %s for key number %d", 
          k_msg.buf[0] == 0xF1 ? "Normal" : "Sport", cas_key_number);
  serial_log(serial_debug_string, 3);
  send_dme_power_ckm();                                                                                                             // Acknowledge settings received from iDrive;
}


// void send_f_oil_level(void) {                                                                                                       // Used with OELSTAND_OENS. 266.0.4 - 6MC2DL0B pg. 1453.
//   if (f_oil_level_timer >= 501) {
//     uint8_t f_oil_level[4] = {0, 0xF0, 4, 0xC0};
//     switch(e_oil_level) {
//       case 0xC: {                                                                                                                   // Below MIN
//         f_oil_level[1] = 0;
//         break;
//       }
//       case 0x19: {                                                                                                                  // MIN
//         f_oil_level[1] = 0x10;
//         break;
//       }
//       case 0x26: {                                                                                                                  // Between MIN and OK
//         f_oil_level[1] = 0x20;
//         break;
//       }
//       case 0x35: {                                                                                                                  // OK
//         f_oil_level[1] = 0x30;
//         break;
//       }
//       case 0x45: {                                                                                                                  // Between OK and MAX
//         f_oil_level[1] = 0x40;
//         break;
//       }
//       case 0x55: {                                                                                                                  // State MAX
//         f_oil_level[1] = 0x50;
//         break;
//       }
//       case 0x5F: {                                                                                                                  // Overfilled
//         f_oil_level[1] = 0x70;
//         break;
//       }
//       case 0x78: {                                                                                                                  // No measurement possible
//         f_oil_level[2] = 0x40;
//         break;
//       }
//       case 0x79: {                                                                                                                  // Measurement in progress
//         break;
//       }
//       case 0x7A: {                                                                                                                  // Oil Level Check OK (Oil level sufficient to start engine).
//         f_oil_level[1] = 0x50;
//         break;
//       }
//       case 0x7B: {                                                                                                                  // Measurement OK
//         f_oil_level[1] = 0x50;
//         break;
//       }
//       case 0x7C: {                                                                                                                  // Ignition OFF, no measurement possible
//         f_oil_level[0] = 0xFF;
//         f_oil_level[2] = 0xFF;
//         break;
//       }
//       case 0x7D: {                                                                                                                  // Engine oil level OK, precise measurement in progress.
//         break;
//       }
//       case 0x7E: {                                                                                                                  // Engine oil level OK
//         f_oil_level[1] = 0x50;
//         break;
//       }
//       case 0x7F: {                                                                                                                  // Measurement in progress
//         break;
//       }
//       case 0x80: {                                                                                                                  // Accurate measurement in progress. Measuring time: 1min
//         break;
//       }
//       case 0xFF: {                                                                                                                  // signal invalid
//         f_oil_level[2] = 0x40;
//         break;
//       }
//     }
//     kcan2_write_msg(make_msg_buf(0x435, 4, f_oil_level));
//     f_oil_level_timer = 0;
//   }
//   if (ignition) {
//     if (f_oil_level_measuring_timer >= 60000) {                                                                                     // Periodically and when cycling ignition, re-set the level.
//       kcan2_write_msg(f_oil_level_measuring_buf);
//       f_oil_level_measuring_timer = 0;
//       f_oil_level_timer = 501;
//     }
//   }
// }
