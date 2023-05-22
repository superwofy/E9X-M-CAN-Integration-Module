// Functions that generate warnings or information for the driver go here.


#if CONTROL_SHIFTLIGHTS
void shiftlight_startup_animation() {
  activate_shiftlight_segments(shiftlights_startup_buildup_buf);
  serial_log("Showing shift light on engine startup.");
  #if NEEDLE_SWEEP
    ignore_shiftlights_off_counter = 18;
  #else
    ignore_shiftlights_off_counter = 8;                                                                                             // Skip a few OFF cycles to allow segments to light up.
  #endif
}


void evaluate_shiftlight_display() {
  if (START_UPSHIFT_WARN_RPM_ <= RPM && RPM <= MID_UPSHIFT_WARN_RPM_) {                                                             // First yellow segment.                                                              
    activate_shiftlight_segments(shiftlights_start_buf);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying first warning at RPM: %d", RPM / 4);
      serial_log(serial_debug_string);
    #endif                     
  } else if (MID_UPSHIFT_WARN_RPM_ <= RPM && RPM <= MAX_UPSHIFT_WARN_RPM_) {                                                        // Buildup from second yellow segment to reds.
    activate_shiftlight_segments(shiftlights_mid_buildup_buf);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying increasing warning at RPM: %d", RPM / 4);
      serial_log(serial_debug_string);
    #endif
  } else if (MAX_UPSHIFT_WARN_RPM_ <= RPM) {                                                                                        // Flash all segments.
    activate_shiftlight_segments(shiftlights_max_flash_buf);
    if (GONG_UPSHIFT_WARN_RPM_ <= RPM) {
      kcan_write_msg(cc_gong_buf);                                                                                                  // Send an audible warning.
    }
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Flash max warning at RPM: %d", RPM / 4);
      serial_log(serial_debug_string);
    #endif
  } else {                                                                                                                          // RPM dropped. Disable lights.
    if (shiftlights_segments_active) {
      if (ignore_shiftlights_off_counter == 0) {
        deactivate_shiftlights();
      } else {
        ignore_shiftlights_off_counter--;
      }
    }
  }
}


void deactivate_shiftlights() {
  kcan_write_msg(shiftlights_off_buf);                                                                            
  shiftlights_segments_active = false;
  serial_log("Deactivated shiftlights segments");
}


void activate_shiftlight_segments(CAN_message_t message) {
  kcan_write_msg(message);                                                               
  shiftlights_segments_active = true;
}


void evaluate_update_shiftlight_sync() {
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
        serial_log(serial_debug_string);
      #endif
      last_var_rpm_can = pt_msg.buf[0];
    }
  } else {
    if (START_UPSHIFT_WARN_RPM_ != START_UPSHIFT_WARN_RPM) {
      START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;                                                                             // Return shiftlight RPMs to default setpoints.
      MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
      MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
      GONG_UPSHIFT_WARN_RPM_ = GONG_UPSHIFT_WARN_RPM;
      serial_log("Engine coolant warmed up. Shiftlight setpoints reset to default.");
    }
  }
}
#endif


#if NEEDLE_SWEEP
void check_kombi_needle_queue() {
  if (!kombi_needle_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    kombi_needle_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      kombi_needle_txq.drop();
    }
  }
}


void needle_sweep_animation() {
  if (diag_transmit) {
    delayed_can_tx_msg m;
    unsigned long timeNow = millis();
    m = {speedo_needle_max_buf, timeNow + 100};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_max_buf, timeNow + 200};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_max_buf, timeNow + 300};
    kombi_needle_txq.push(&m);
    m = {oil_needle_max_buf, timeNow + 400};
    kombi_needle_txq.push(&m);

    m = {tacho_needle_min_buf, timeNow + 1000};
    kombi_needle_txq.push(&m);
    m = {speedo_needle_min_buf, timeNow + 1100};
    kombi_needle_txq.push(&m);
    m = {oil_needle_min_buf, timeNow + 1200};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_min_buf, timeNow + 1300};
    kombi_needle_txq.push(&m);

    m = {speedo_needle_release_buf, timeNow + 1900};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_release_buf, timeNow + 2000};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_release_buf, timeNow + 2100};
    kombi_needle_txq.push(&m);
    m = {oil_needle_release_buf, timeNow + 2200};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_release_buf, timeNow + 2700};                                                                                 // Send the release twice to make sure needles now work.
    kombi_needle_txq.push(&m);
    m = {speedo_needle_release_buf, timeNow + 2800};
    kombi_needle_txq.push(&m);
    m = {oil_needle_release_buf, timeNow + 2900};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_release_buf, timeNow + 3000};
    kombi_needle_txq.push(&m);
    serial_log("Sending needle sweep animation.");
  }
}
#endif


#if LAUNCH_CONTROL_INDICATOR
void evaluate_lc_display() {
  if (LC_RPM_MIN <= RPM && RPM <= LC_RPM_MAX) {
    if (clutch_pressed && !vehicle_moving) {
      if (!reverse_status) {
        kcan_write_msg(lc_cc_on_buf);
        lc_cc_active = true;
        if (dsc_program_status == 0) {
          mdm_with_lc = true;
          serial_log("Launch Control request DSC ON -> MDM/DTC.");
          send_dsc_mode(1);
        }
        serial_log("Displayed LC flag CC.");
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
        serial_log("Launch Control aborted. MDM/DTC -> DSC ON.");
        send_dsc_mode(0);
        mdm_with_lc = false;
      }
    }
    deactivate_lc_display();
  }
}


void deactivate_lc_display() {
  if (lc_cc_active) {
    kcan_write_msg(lc_cc_off_buf);
    lc_cc_active = false;
    serial_log("Deactivated LC flag CC.");
    #if CONTROL_SHIFTLIGHTS
      deactivate_shiftlights();
    #endif
  }  
}
#endif


#if SERVOTRONIC_SVT70
void send_svt_kcan_cc_notification() {
  if (pt_msg.buf[1] == 0x49 && pt_msg.buf[2] == 0) {                                                                                // Change from CC-ID 73 (EPS Inoperative) to CC-ID 70 (Servotronic).
    pt_msg.buf[1] = 0x46;
  }
  kcan_write_msg(pt_msg);                                                                                                           // Forward the SVT error status to KCAN.
}


void indicate_svt_diagnosis_on() {
  if (!digitalRead(POWER_BUTTON_PIN)) {                                                                                             // If POWER button is being held when turning ignition ON, allow SVT diagnosis.
    diagnose_svt = true;
    serial_log("Diagnosing SVT70 module now possible.");
    kcan_write_msg(servotronic_cc_on_buf);                                                                                          // Indicate that diagnosing is now possible.
  }
}
#endif


#if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER
void evaluate_fog_status() {
  if ((k_msg.buf[0] & 32) == 32) {                                                                                                  // Check the third bit of the first byte represented in binary for front fog status.
    if (!front_fog_status) {
      front_fog_status = true;
      #if FRONT_FOG_LED_INDICATOR
        digitalWrite(FOG_LED_PIN, HIGH);
        serial_log("Front fogs ON. Turned ON FOG LED");
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
        serial_log("Front fogs OFF. Turned OFF FOG LED");
      #endif
    }
  }
}
#endif


#if FTM_INDICATOR
void evaluate_indicate_ftm_status() {
  if (pt_msg.buf[0] == 3 && !ftm_indicator_status) {
    kcan_write_msg(ftm_indicator_flash_buf);
    ftm_indicator_status = true;
    serial_log("Activated FTM indicator.");
  } else if (pt_msg.buf[0] == 0 && ftm_indicator_status) {
    kcan_write_msg(ftm_indicator_off_buf);
    ftm_indicator_status = false;
    serial_log("Deactivated FTM indicator.");
  }
}
#endif


#if FAKE_MSA
void evaluate_msa_button() {
  if (k_msg.buf[0] == 0xF5 || k_msg.buf[0] == 0xF1) {                                                                               // Button pressed.
    if (!msa_button_pressed) {
      if (engine_running) {
        kcan_write_msg(msa_deactivated_cc_on_buf);
        serial_log("Sent MSA OFF CC.");
        delayed_can_tx_msg m = {msa_deactivated_cc_off_buf, millis() + 3000};
        ihk_extra_buttons_cc_txq.push(&m);
      }
    }
    msa_button_pressed = true;
  } else {                                                                                                                          // Now receiving released (0xF4 or 0xF0) messages from IHKA.
    msa_button_pressed = false;
  }
}
#endif


#if HDC || FAKE_MSA
void check_ihk_buttons_cc_queue() {
  if (!ihk_extra_buttons_cc_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    ihk_extra_buttons_cc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      ihk_extra_buttons_cc_txq.drop();
    }
  }
}
#endif
