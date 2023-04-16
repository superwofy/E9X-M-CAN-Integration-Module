#if CONTROL_SHIFTLIGHTS
void shiftlight_startup_animation()
{
  activate_shiftlight_segments(shiftlights_startup_buildup_buf);
  serial_log("Showing shift light on engine startup.");
  #if NEEDLE_SWEEP
    ignore_shiftlights_off_counter = 10;
  #else
    ignore_shiftlights_off_counter = 8;                                                                                             // Skip a few off cycles to allow segments to light up.
  #endif
}


void evaluate_shiftlight_display()
{
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


void deactivate_shiftlights()
{
  kcan_write_msg(shiftlights_off_buf);                                                                            
  shiftlights_segments_active = false;
  serial_log("Deactivated shiftlights segments");
}


void activate_shiftlight_segments(CAN_message_t message)
{
  kcan_write_msg(message);                                                               
  shiftlights_segments_active = true;
}


void evaluate_update_shiftlight_sync()
{
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
void check_kombi_needle_queue()
{
  if (!kombi_needle_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    kombi_needle_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      kombi_needle_txq.drop();
    }
  }
}


void needle_sweep_animation()
{
  if (diag_transmit) {
    delayed_can_tx_msg m;
    unsigned long timeNow = millis();
    m = {speedo_needle_sweep_buf, timeNow};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_sweep_buf, timeNow + 30};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_sweep_buf, timeNow + 60};
    kombi_needle_txq.push(&m);
    m = {oil_needle_sweep_buf, timeNow + 90};
    kombi_needle_txq.push(&m);
    m = {speedo_needle_release_buf, timeNow + 900};
    kombi_needle_txq.push(&m);
    m = {tacho_needle_release_buf, timeNow + 930};
    kombi_needle_txq.push(&m);
    m = {fuel_needle_release_buf, timeNow + 960};
    kombi_needle_txq.push(&m);
    m = {oil_needle_release_buf, timeNow + 990};
    kombi_needle_txq.push(&m);
    serial_log("Sending needle sweep animation.");
  }
}
#endif
