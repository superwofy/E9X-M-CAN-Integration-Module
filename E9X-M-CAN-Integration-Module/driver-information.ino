// Functions that generate warnings or information for the driver go here.


void shiftlight_startup_animation(void) {
  activate_shiftlight_segments(shiftlights_startup_buildup_buf);
  startup_animation_active = true;
  serial_log("Showing shift light on engine startup.", 3);
  #if NEEDLE_SWEEP
    ignore_shiftlights_off_counter = 7;
  #else
    ignore_shiftlights_off_counter = 4;                                                                                             // Skip a few OFF cycles to allow segments to light up.
  #endif
}


void evaluate_shiftlight_display(void) {
  if (START_UPSHIFT_WARN_RPM_ <= RPM && RPM <= MID_UPSHIFT_WARN_RPM_) {                                                             // First yellow segment.                                                              
    activate_shiftlight_segments(shiftlights_start_buf);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying first warning at RPM: %d", RPM / 4);
      serial_log(serial_debug_string, 3);
    #endif                     
  } else if (MID_UPSHIFT_WARN_RPM_ <= RPM && RPM <= MAX_UPSHIFT_WARN_RPM_) {                                                        // Buildup from second yellow segment to reds.
    activate_shiftlight_segments(shiftlights_mid_buildup_buf);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying increasing warning at RPM: %d", RPM / 4);
      serial_log(serial_debug_string, 3);
    #endif
  } else if (MAX_UPSHIFT_WARN_RPM_ <= RPM) {                                                                                        // Flash all segments.
    activate_shiftlight_segments(shiftlights_max_flash_buf);
    if (GONG_UPSHIFT_WARN_RPM_ <= RPM) {
      play_cc_gong(1);                                                                                                              // Send an audible warning.
    }
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Flash max warning at RPM: %d", RPM / 4);
      serial_log(serial_debug_string, 2);
    #endif
  } else {                                                                                                                          // RPM dropped. Disable lights.
    if (shiftlights_segments_active) {
      if (ignore_shiftlights_off_counter == 0) {
        if (startup_animation_active) {
          activate_shiftlight_segments(shiftlights_max_flash_buf);
          #if NEEDLE_SWEEP
            ignore_shiftlights_off_counter = 10;
          #else
            ignore_shiftlights_off_counter = 5;
          #endif
          startup_animation_active = false;
        } else {
          deactivate_shiftlights();
        }
      } else {
        ignore_shiftlights_off_counter--;
      }
    }
  }
}


void deactivate_shiftlights(void) {
  kcan_write_msg(shiftlights_off_buf);                                                                            
  shiftlights_segments_active = false;
  serial_log("Deactivated shiftlights segments", 3);
}


void activate_shiftlight_segments(CAN_message_t message) {
  kcan_write_msg(message);                                                               
  shiftlights_segments_active = true;
}


void evaluate_update_shiftlight_sync(void) {
  if (!engine_coolant_warmed_up) {
    if (pt_msg.buf[0] != last_var_rpm_can) {
      var_redline_position = ((pt_msg.buf[0] * 0x32) + VAR_REDLINE_OFFSET_RPM) * 4;                                                 // This is where the variable redline actually starts on the KOMBI (x4).
      START_UPSHIFT_WARN_RPM_ = var_redline_position;                                                                              
      MID_UPSHIFT_WARN_RPM_ = var_redline_position + 1600;                                                                          // +400 RPM
      MAX_UPSHIFT_WARN_RPM_ = var_redline_position + 2400;                                                                          // +600 RPM
      GONG_UPSHIFT_WARN_RPM_ = var_redline_position + 2800;                                                                         // +700 RPM
      if (pt_msg.buf[0] == 0x88) {                                                                                                  // DME is sending 6800 RPM.
        engine_coolant_warmed_up = true;
      }
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Set shiftlight RPMs to %d %d %d %d. Variable redline is at %d.", 
                (START_UPSHIFT_WARN_RPM_ / 4), (MID_UPSHIFT_WARN_RPM_ / 4), 
                (MAX_UPSHIFT_WARN_RPM_ / 4), (GONG_UPSHIFT_WARN_RPM_ / 4), (var_redline_position / 4));
        serial_log(serial_debug_string, 3);
      #endif
      last_var_rpm_can = pt_msg.buf[0];
    }
  } else {
    if (START_UPSHIFT_WARN_RPM_ != START_UPSHIFT_WARN_RPM) {
      START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;                                                                             // Return shiftlight RPMs to default setpoints.
      MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
      MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
      GONG_UPSHIFT_WARN_RPM_ = GONG_UPSHIFT_WARN_RPM;
      serial_log("Engine coolant warmed up. Shiftlight setpoints reset to default.", 3);
    }
  }
}


void check_kombi_needle_queue(void) {
  if (!kombi_needle_txq.isEmpty()) {
    if (diag_transmit) {
      kombi_needle_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        kombi_needle_txq.drop();
      }
    } else {
      kombi_needle_txq.flush();
    }
  }
}


void needle_sweep_animation(void) {
  if (diag_transmit) {
    unsigned long time_now = millis();
    m = {speedo_needle_max_buf, time_now + 100};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_max_buf, time_now + 200};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_max_buf, time_now + 300};
    kombi_needle_txq.push(&m);
    m = {oil_needle_max_buf, time_now + 400};
    kombi_needle_txq.push(&m);

    m = {tacho_needle_min_buf, time_now + 1000};
    kombi_needle_txq.push(&m);
    m = {speedo_needle_min_buf, time_now + 1100};
    kombi_needle_txq.push(&m);
    m = {oil_needle_min_buf, time_now + 1200};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_min_buf, time_now + 1300};
    kombi_needle_txq.push(&m);

    m = {speedo_needle_release_buf, time_now + 1900};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_release_buf, time_now + 2000};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_release_buf, time_now + 2100};
    kombi_needle_txq.push(&m);
    m = {oil_needle_release_buf, time_now + 2200};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_release_buf, time_now + 2700};                                                                                // Send the release twice to make sure needles now work.
    kombi_needle_txq.push(&m);
    m = {speedo_needle_release_buf, time_now + 2800};
    kombi_needle_txq.push(&m);
    m = {oil_needle_release_buf, time_now + 2900};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_release_buf, time_now + 3000};
    kombi_needle_txq.push(&m);
    serial_log("Sending needle sweep animation.", 3);

    #if F_NBT
      send_nbt_sport_displays_data(true);
    #endif
  }
}


void evaluate_lc_display(void) {
  if (LC_RPM_MIN <= RPM && RPM <= LC_RPM_MAX) {
    if (clutch_pressed && !vehicle_moving) {
      if (!reverse_gear_status) {
        kcan_write_msg(lc_cc_on_buf);
        #if F_NBT
          kcan2_write_msg(lc_cc_on_buf);
        #endif
        lc_cc_active = true;
        if (dsc_program_status == 0) {
          mdm_with_lc = true;
          serial_log("Launch Control request DSC ON -> MDM/DTC.", 2);
          send_dsc_mode(1);
        }
        serial_log("Displayed Launch Control flag CC.", 2);
        #if CONTROL_SHIFTLIGHTS
          activate_shiftlight_segments(shiftlights_max_flash_buf);
        #endif
      }
    } else {
      deactivate_lc_display();
      mdm_with_lc = false;                                                                                                          // Vehicle probably launched. MDM/DTC stays ON
    }
  } else {
    if (lc_cc_active) {
      if (mdm_with_lc && dsc_program_status == 1) {
        serial_log("Launch Control aborted. MDM/DTC -> DSC ON.", 2);
        send_dsc_mode(0);
        mdm_with_lc = false;
      }
    }
    deactivate_lc_display();
  }
}


void deactivate_lc_display(void) {
  if (lc_cc_active) {
    kcan_write_msg(lc_cc_off_buf);
    #if F_NBT
      kcan2_write_msg(lc_cc_off_buf);
    #endif
    lc_cc_active = false;
    serial_log("Deactivated Launch Control flag CC.", 2);
    #if CONTROL_SHIFTLIGHTS
      deactivate_shiftlights();
    #endif
  }  
}


void send_svt_kcan_cc_notification(void) {
  if (pt_msg.buf[1] == 0x49 && pt_msg.buf[2] == 0) {                                                                                // Change from CC-ID 73 (EPS Inoperative) to CC-ID 70 (Servotronic).
    pt_msg.buf[1] = 0x46;
  }
  kcan_write_msg(pt_msg);                                                                                                           // Forward the SVT error status to KCAN.
  #if F_NBT
    kcan2_write_msg(pt_msg);
  #endif
}


void evaluate_fog_status(void) {
  if (bitRead(k_msg.buf[0], 5)) {                                                                                                   // Check bit 5 (from LSB 0) of the first byte represented in binary for front fog status.
    if (!front_fog_status) {
      front_fog_status = true;
      #if FRONT_FOG_LED_INDICATOR
        digitalWrite(FOG_LED_PIN, HIGH);
        serial_log("Front fogs ON. Turned ON FOG LED", 2);
      #endif
      #if FRONT_FOG_CORNER
        left_fog_on = right_fog_on = false;
      #endif
    }
  } else {
    if (front_fog_status) {
      front_fog_status = false;
      #if FRONT_FOG_LED_INDICATOR
        digitalWrite(FOG_LED_PIN, LOW);
        serial_log("Front fogs OFF. Turned OFF FOG LED", 2);
      #endif
    }
  }
}


void evaluate_reverse_beep(void) {
  if (reverse_gear_status) {
    if (reverse_beep_resend_timer >= 2000) {
      if (!reverse_beep_sent) {
        if (!pdc_too_close) {
          serial_log("Sending reverse beep.", 2);
          if (!idrive_died) {
            #if F_NBT
              kcan2_write_msg(idrive_beep_sound_buf);
            #else
              kcan_write_msg(idrive_beep_sound_buf);
            #endif
          } else {
            play_cc_gong(1);
          }
          reverse_beep_sent = true;
          reverse_beep_resend_timer = 0;
        } else {
          reverse_beep_sent = true;                                                                                                 // Cancel beep if already too close.
        }
      }
    } else {
      reverse_beep_sent = true;                                                                                                     // Cancel beep if reverse engaged again quickly.
    }
  }
}


void evaluate_indicate_ftm_status(void) {
  if (pt_msg.buf[0] == 3 && !ftm_indicator_status) {
    kcan_write_msg(ftm_indicator_flash_buf);
    ftm_indicator_status = true;
    serial_log("Activated FTM indicator.", 2);
  } else if (pt_msg.buf[0] == 0 && ftm_indicator_status) {
    kcan_write_msg(ftm_indicator_off_buf);
    ftm_indicator_status = false;
    serial_log("Deactivated FTM indicator.", 2);
  }
}


void send_f_ftm_status(void) {
  uint8_t f_ftm_status[] = {0, 0, 0, 0, 0};
  f_ftm_status[1] = 0xF << 4 | f_ftm_status_alive_counter;
  f_ftm_status_alive_counter == 0xE ? f_ftm_status_alive_counter = 0 
                                    : f_ftm_status_alive_counter++;

  #if F_NBT_EVO6
    if (k_msg.buf[0] == 0) {
      f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0xA0;
    } else if (k_msg.buf[0] == 1) {
      f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0x60;
    } else if (k_msg.buf[0] == 3) {
      f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0xE8;
    } else {
      f_ftm_status[2] = 0xA1;
      f_ftm_status[3] = 0x30;
    }
  #else
    if (ignition) {
      if (k_msg.buf[0] == 0) {                                                                                                      // Active
        if (engine_running) {
          f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0xA0;
        } else {
          f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0xE8;
        }
      } else if (k_msg.buf[0] == 3) {                                                                                               // Re-setting
        f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0xE8;
      } else if (k_msg.buf[0] == 1) {
        f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0x60;                                                                 // Failure
      } else {                                                                                                                      // Other, e.g pressure low.
        f_ftm_status[2] = 0xA1;
        f_ftm_status[3] = 0x30;
      }
    } else {
      f_ftm_status[2] = f_ftm_status[3] = f_ftm_status[4] = 0xE8;
    }
  #endif

  f_ftm_status_crc.restart();
  for (uint8_t i = 1; i < 5; i++) {
    f_ftm_status_crc.add(f_ftm_status[i]);
  }
  f_ftm_status[0] = f_ftm_status_crc.calc();
  CAN_message_t f_ftm_status_buf = make_msg_buf(0x369, 5, f_ftm_status);
  kcan2_write_msg(f_ftm_status_buf);
}


void evaluate_msa_button(void) {
  if (k_msg.buf[0] == 0xF5 || k_msg.buf[0] == 0xF1) {                                                                               // Button pressed.
    if (!msa_button_pressed) {
      #if FAKE_MSA
        if (engine_running) {
          kcan_write_msg(msa_deactivated_cc_on_buf);
          #if F_NBT
            kcan2_write_msg(msa_deactivated_cc_on_buf);
          #endif
          serial_log("Sent MSA OFF CC.", 2);
          m = {msa_deactivated_cc_off_buf, millis() + 3000};
          ihk_extra_buttons_cc_txq.push(&m);
        }
      #endif
      #if MSA_RVC
        if (pdc_bus_status == 0xA5) {
          kcan_write_msg(camera_off_buf);
          #if F_NBT
            kcan2_write_msg(camera_off_buf);
          #endif
          serial_log("Deactivated RVC with PDC ON.", 2);
        } else if (pdc_bus_status == 0xA4) {
          kcan_write_msg(pdc_off_camera_off_buf);
          #if F_NBT
            kcan2_write_msg(pdc_off_camera_off_buf);
          #endif
          serial_log("Deactivated RVC.", 2);
        } else if (pdc_bus_status == 0xA1) {
          kcan_write_msg(camera_on_buf);
          #if F_NBT
            kcan2_write_msg(camera_on_buf);
          #endif
          serial_log("Activated RVC with PDC displayed.", 2);
        } else if (pdc_bus_status == 0x81) {
          kcan_write_msg(pdc_on_camera_on_buf);
          #if F_NBT
            kcan2_write_msg(pdc_on_camera_on_buf);
          #endif
          serial_log("Activated RVC from PDC ON, not displayed.", 2);
        } else if (pdc_bus_status == 0x80) {
          kcan_write_msg(pdc_off_camera_on_buf);
          #if F_NBT
            kcan2_write_msg(pdc_off_camera_on_buf);
          #endif
          serial_log("Activated RVC from PDC OFF.", 2);
        }
      #endif
    }
    msa_button_pressed = true;
  } else {                                                                                                                          // Now receiving released (0xF4 or 0xF0) messages from IHKA.
    msa_button_pressed = false;
  }
}


void check_ihk_buttons_cc_queue(void) {
  if (!ihk_extra_buttons_cc_txq.isEmpty()) {
    ihk_extra_buttons_cc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      #if F_NBT
        kcan2_write_msg(delayed_tx.tx_msg);
      #endif
      ihk_extra_buttons_cc_txq.drop();
    }
  }
}


void evaluate_pdc_button(void) {
  if (k_msg.buf[0] == 0xFD) {                                                                                                       // Button pressed.
    if (!pdc_button_pressed) {
      if (pdc_bus_status == 0xA4) {
        pdc_with_rvc_requested = true;
      }
      pdc_button_pressed = true;
    }
  } else {                                                                                                                          // Now receiving released (0xFC or 0xF4) messages from IHKA.
    pdc_button_pressed = false;
  }
}


void evaluate_pdc_bus_status(void) {
  if (pdc_bus_status != k_msg.buf[2]) {
    #if MSA_RVC
      if (pdc_bus_status == 0xA4 && k_msg.buf[2] == 0x81) {
        kcan_write_msg(pdc_off_camera_off_buf);                                                                                     // Fix for PDC coming on when navingating away from RVC only.
        #if F_NBT
          kcan2_write_msg(pdc_off_camera_off_buf);
        #endif
      } else if (pdc_bus_status == 0xA4 && k_msg.buf[2] == 0xA1) {
        kcan_write_msg(pdc_off_camera_off_buf);                                                                                     // Fix for PDC coming on when turning off RVC from iDrive UI.
        #if F_NBT
          kcan2_write_msg(pdc_off_camera_off_buf);
        #endif
      }
      if (pdc_with_rvc_requested && k_msg.buf[2] == 0x80) {
        kcan_write_msg(pdc_on_camera_on_buf);
        #if F_NBT
          kcan2_write_msg(pdc_on_camera_on_buf);
        #endif
        serial_log("Activated PDC with RVC ON.", 2);
        pdc_with_rvc_requested = false;
        #if F_VSW01 && F_NBT_EVO6_GW7
          vsw_switch_input(1);
        #endif
      }
    #endif
    pdc_bus_status = k_msg.buf[2];
    if (pdc_bus_status > 0x80) {
      serial_log("PDC ON.", 3);
    } else {
      serial_log("PDC OFF.", 3);
    }
    #if AUTO_TOW_VIEW_RVC
      if (pdc_bus_status != 0xA5) {
        rvc_tow_view_by_module = rvc_tow_view_by_driver = false;
      }
    #endif
  }
}


void evaluate_f_pdc_function_request(void) {
  if (k_msg.buf[3] != 4) {                                                                                                          // The first 0x31A should be ignored. Its Byte3 is 4. Next message will be 8.
    if (k_msg.buf[0] > 0x10) {                                                                                                      // Navigated away from reversing screen.
      kcan_write_msg(camera_inactive_buf);
      kcan2_write_msg(camera_inactive_buf);
      #if F_VSW01 && F_NBT_EVO6_GW7
        vsw_switch_input(4);
      #endif
    } else {
      if (f_pdc_request != k_msg.buf[1]) {                                                                                          // PDC only, camera OFF.
        f_pdc_request = k_msg.buf[1];
        if (f_pdc_request == 1) {
          kcan_write_msg(camera_off_buf);
          kcan2_write_msg(camera_off_buf);
          #if F_VSW01 && F_NBT_EVO6_GW7
            vsw_switch_input(4);
          #endif
        } else if (f_pdc_request == 5) {                                                                                            // Camera ON.
          kcan_write_msg(camera_on_buf);
          kcan2_write_msg(camera_on_buf);
          #if F_VSW01 && F_NBT_EVO6_GW7
            vsw_switch_input(1);
          #endif
        }
      }
    }
  }
}


void evaluate_pdc_warning(void) {
  uint8_t max_volume = k_msg.buf[0];
  if (k_msg.buf[1] > max_volume) {
    max_volume = k_msg.buf[1];
  }
  if (max_volume >= 7) {
    pdc_too_close = true;
  } else {
    pdc_too_close = false;
  }
  #if DOOR_VOLUME
    for (uint8_t i = 0; i < 4; i++) {
      if (k_msg.buf[i] >= 7) {
        pdc_tone_on = true;
        return;
      } else {
        pdc_tone_on = false;
      }
    }
  #endif
}


void evaluate_pdc_distance(void) {
  if (!rvc_tow_view_by_driver) {
    uint8_t distance_threshold = 0x36;                                                                                              // There's a slight delay when changing modes. Preempt by switching earlier.
    if (real_speed >= 5) {                                                                                                          // At higher speeds the delay is very noticeable.
      return;
    } else if (real_speed >= 1) {
      distance_threshold = 0x40;
    }

    if (k_msg.buf[2] <= distance_threshold && k_msg.buf[3] <= distance_threshold) {
      if (reverse_gear_status && !rvc_tow_view_by_module && pdc_bus_status == 0xA5) {
        bitWrite(rvc_settings[0], 3, 1);                                                                                            // Set tow hitch view to ON.
        serial_log("Rear inner sensors RED, enabling camera tow view.", 3);
        CAN_message_t new_rvc_settings = make_msg_buf(0x38F, 4, rvc_settings);
        kcan_write_msg(new_rvc_settings);
        #if F_NBT
          kcan2_write_msg(new_rvc_settings);
        #endif
        rvc_tow_view_by_module = true;
        rvc_action_timer = 0;
      }
    } else if (rvc_tow_view_by_module && pdc_bus_status == 0xA5) {
      if (rvc_action_timer >= 2000) {
        bitWrite(rvc_settings[0], 3, 0);                                                                                            // Set tow hitch view to OFF.
        serial_log("Disabled camera tow view after inner sensors no longer RED.", 2);
        CAN_message_t new_rvc_settings = make_msg_buf(0x38F, 4, rvc_settings);
        kcan_write_msg(new_rvc_settings);
        #if F_NBT
          kcan2_write_msg(new_rvc_settings);
        #endif
        rvc_tow_view_by_module = false;
        rvc_action_timer = 0;
      }
    }
  }
}


void evaluate_handbrake_status(void) {
  if (handbrake_status_debounce_timer >= 200) {
    if (k_msg.buf[0] == 0xFE) {
      if (!handbrake_status) {
        handbrake_status = true;
        serial_log("Handbrake ON.", 2);
        #if PDC_AUTO_OFF
          if (!reverse_gear_status && !vehicle_moving && pdc_bus_status > 0x80) {
            unsigned long time_now = millis();
            kcan_write_msg(pdc_button_presssed_buf);
            #if F_NBT
              kcan2_write_msg(pdc_button_presssed_buf);
            #endif
            m = {pdc_button_released_buf, time_now + 100};
            pdc_buttons_txq.push(&m);
            m = {pdc_button_released_buf, time_now + 200};
            pdc_buttons_txq.push(&m);
            serial_log("Disabled PDC after handbrake was pulled up.", 3);
          }
        #endif
      }
    } else {
      if (handbrake_status) {
        handbrake_status = false;
        serial_log("Handbrake OFF.", 2);
      }
    }
    handbrake_status_debounce_timer = 0;
  }
}


void check_pdc_button_queue(void) {
  if (!pdc_buttons_txq.isEmpty()) {
    pdc_buttons_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      #if F_NBT
        kcan2_write_msg(delayed_tx.tx_msg);
      #endif
      pdc_buttons_txq.drop();
    }
  }
}


void play_cc_gong(uint8_t gong_count) {
  if (gong_count == 2) {
    kcan_write_msg(cc_double_gong_buf);
    #if F_NBT
      kcan2_write_msg(cc_double_gong_buf);
    #endif
  } else if (gong_count == 3) {
    kcan_write_msg(cc_triple_gong_buf);
    #if F_NBT
      kcan2_write_msg(cc_triple_gong_buf);
    #endif
  } else {
    kcan_write_msg(cc_single_gong_buf);
    #if F_NBT
      kcan2_write_msg(cc_single_gong_buf);
    #endif
  }
  gong_active = true;
}


void send_f_pdc_function_status(bool disable) {
  uint8_t f_parking_function_status[] = {0, 0, 0, 0xF, 0, 0, 0xFF, 0xFF};                                                           // PDC OFF.
  f_parking_function_status[0] = f_pdc_function_status_alive_counter;
  if (!disable) {
    if (pdc_bus_status == 0xA1 || pdc_bus_status == 0xA5) {
      f_parking_function_status[0] += 0x40;
    }
    f_parking_function_status[1] = k_msg.buf[2] & 0xF;                                                                              // Second half of PDC bus status.
  }
  f_pdc_function_status_alive_counter == 0xE ? f_pdc_function_status_alive_counter = 0 
                                            : f_pdc_function_status_alive_counter++;
  CAN_message_t f_parking_function_status_buf = make_msg_buf(0x2C1, 8, f_parking_function_status);
  kcan2_write_msg(f_parking_function_status_buf);
}


void send_nbt_sport_displays_data(bool startup_animation) {
  #if F_NBT
    if (terminal_r) {
      uint8_t nbt_sport_data[] = {0, 0, 0, 0xFF};
      float max_power = 0, max_torque = 0, power_factor = 1, torque_factor = 1;                                                     // kW and Nm
      if (power_unit[cas_key_number] == 1) {
        nbt_sport_data[1] = MAX_POWER_SCALE_KW << 4;
        max_power = (1 + MAX_POWER_SCALE_KW) * 80;
      } else if (power_unit[cas_key_number] == 2) {
        nbt_sport_data[1] = MAX_POWER_SCALE_HP << 4;
        max_power = (1 + MAX_POWER_SCALE_HP) * 80;
        power_factor = 1.341;
      }
      if (torque_unit[cas_key_number] == 1) {
        nbt_sport_data[1] |= MAX_TORQUE_SCALE_NM;
        max_torque = (1 + MAX_TORQUE_SCALE_NM) * 80;
      } else if (torque_unit[cas_key_number] == 2) {
        nbt_sport_data[1] |= MAX_TORQUE_SCALE_LB;
        max_torque = (1 + MAX_TORQUE_SCALE_LB) * 80;
        torque_factor = 0.7376;
      } else if (torque_unit[cas_key_number] == 3) {
        nbt_sport_data[1] |= MAX_TORQUE_SCALE_KG;
        max_torque = (1 + MAX_TORQUE_SCALE_KG) * 80;
        torque_factor = 0.1;
      }

      if (startup_animation) {
        nbt_sport_data[0] = nbt_sport_data[2] = 0x64;                                                                               // Set to 100% on startup to match needle sweep.
        ignore_sports_data_counter = 10;
      } else {
        if (ignore_sports_data_counter == 0) {
          if (engine_running) {
            uint16_t raw_value = (k_msg.buf[2] << 4) | (k_msg.buf[1] >> 4);                                                         // 12-bit EXX torque (TORQ_AVL - torque actual-value at the clutch).
            int16_t signed_value = (raw_value & 0x800) ? (raw_value | 0xF000) : raw_value;
            engine_torque = signed_value * 0.5 * torque_factor;                                                                     // Signed and scaled Nm value.

            if (engine_torque > max_torque) { engine_torque = max_torque; }                                                         // Ensure we don't exceed the scale.
            if (engine_torque > 10) {                                                                                               // This value can be negative. When coasting? Add a minimum to prevent idle twitching.
              uint8_t torque_percentage = round((engine_torque / max_torque) * 100);
              nbt_sport_data[0] = torque_percentage;

              float calculated_power = ((engine_torque * (RPM / 4)) / 9548) * power_factor;
              if (calculated_power > max_power) { calculated_power = max_power; }
              if (calculated_power > 0) {
                uint8_t power_percentage = round((calculated_power / max_power) * 100);
                nbt_sport_data[2] = power_percentage;
              }
            }
          }
        } else {
          nbt_sport_data[0] = nbt_sport_data[2] = 0x64;
          ignore_sports_data_counter--;
        }
      }

      CAN_message_t nbt_sport_data_buf = make_msg_buf(0x253, 4, nbt_sport_data);
      kcan2_write_msg(nbt_sport_data_buf);
    }
  #endif
}
