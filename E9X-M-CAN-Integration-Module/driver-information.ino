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
      play_cc_gong();                                                                                                               // Send an audible warning.
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
  }
}


void evaluate_lc_display(void) {
  if (LC_RPM_MIN <= RPM && RPM <= LC_RPM_MAX) {
    if (clutch_pressed && !vehicle_moving) {
      if (!reverse_gear_status) {
        kcan_write_msg(lc_cc_on_buf);
        lc_cc_active = true;
        if (dsc_program_status == 0) {
          mdm_with_lc = true;
          serial_log("Launch Control request DSC ON -> MDM/DTC.", 2);
          send_dsc_mode(1);
        }
        serial_log("Displayed LC flag CC.", 2);
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
    lc_cc_active = false;
    serial_log("Deactivated LC flag CC.", 2);
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
  if (reverse_gear_status && engine_running) {
    if (reverse_beep_resend_timer >= 2000) {
      if (!reverse_beep_sent) {
        if (!pdc_too_close) {
          serial_log("Sending reverse beep.", 2);
          if (!idrive_died) {
            kcan_write_msg(cic_beep_sound_buf);
          } else {
            play_cc_gong();
          }
          reverse_beep_sent = true;
        } else {
          reverse_beep_sent = true;                                                                                                 // Cancel beep if already too close.
        }
      }
      reverse_beep_resend_timer = 0;
    } else {
      reverse_beep_sent = true;                                                                                                     // Cancel beep if reverse engaged again quickly.
    }
  } else {
    reverse_beep_sent = false;
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


void evaluate_msa_button(void) {
  if (k_msg.buf[0] == 0xF5 || k_msg.buf[0] == 0xF1) {                                                                               // Button pressed.
    if (!msa_button_pressed) {
      #if FAKE_MSA
        if (engine_running) {
          kcan_write_msg(msa_deactivated_cc_on_buf);
          serial_log("Sent MSA OFF CC.", 2);
          m = {msa_deactivated_cc_off_buf, millis() + 3000};
          ihk_extra_buttons_cc_txq.push(&m);
        }
      #endif
      #if MSA_RVC
        if (pdc_bus_status == 0xA5) {
          kcan_write_msg(camera_off_buf);
          serial_log("Deactivated RVC with PDC ON.", 2);
        } else if (pdc_bus_status == 0xA4) {
          kcan_write_msg(pdc_off_camera_off_buf);
          serial_log("Deactivated RVC.", 2);
        } else if (pdc_bus_status == 0xA1) {
          kcan_write_msg(camera_on_buf);
          serial_log("Activated RVC with PDC displayed.", 2);
        } else if (pdc_bus_status == 0x81) {
          kcan_write_msg(pdc_on_camera_on_buf);
          serial_log("Activated RVC from PDC ON, not displayed.", 2);
        } else if (pdc_bus_status == 0x80) {
          kcan_write_msg(pdc_off_camera_on_buf);
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
      } else if (pdc_bus_status == 0xA4 && k_msg.buf[2] == 0xA1) {
        kcan_write_msg(pdc_off_camera_off_buf);                                                                                     // Fix for PDC coming on when turning off RVC from iDrive UI.
      }
      if (pdc_with_rvc_requested && k_msg.buf[2] == 0x80) {
        kcan_write_msg(pdc_on_camera_on_buf);
        serial_log("Activated PDC with RVC ON.", 2);
        pdc_with_rvc_requested = false;
      }
    #endif
    pdc_bus_status = k_msg.buf[2];
    if (pdc_bus_status > 0x80) {
      serial_log("PDC ON.", 3);
    } else {
      serial_log("PDC OFF.", 3);
    }
    #if AUTO_DIP_RVC
      if (pdc_bus_status != 0xA5) {
        rvc_dipped_by_module = rvc_dipped_by_driver = false;
      }
    #endif
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
  if (!rvc_dipped_by_driver) {
    uint8_t distance_threshold = 0x36;                                                                                              // There's a slight delay when changing modes. Preempt by switching earlier.
    if (vehicle_moving) {
      distance_threshold = 0x40;
    }

    if (k_msg.buf[2] <= distance_threshold && k_msg.buf[3] <= distance_threshold) {
      if (reverse_gear_status && !rvc_dipped_by_module && pdc_bus_status == 0xA5) {
        bitWrite(rvc_settings[0], 3, 1);                                                                                            // Set tow hitch view to ON.
        serial_log("Rear inner sensors RED, enabling camera dip.", 3);
        kcan_write_msg(make_msg_buf(0x38F, 4, rvc_settings));
        rvc_dipped_by_module = true;
      }
    } else if (rvc_dipped_by_module && pdc_bus_status == 0xA5) {
      bitWrite(rvc_settings[0], 3, 0);                                                                                              // Set tow hitch view to OFF.
      serial_log("Disabled camera dip after inner sensors no longer RED.", 2);
      kcan_write_msg(make_msg_buf(0x38F, 4, rvc_settings));
      rvc_dipped_by_module = false;
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
      pdc_buttons_txq.drop();
    }
  }
}


void play_cc_gong(void) {
  if (diag_transmit) {
    kcan_write_msg(cc_gong_buf);
  }
}
