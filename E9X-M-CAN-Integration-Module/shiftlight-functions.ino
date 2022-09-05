#if CONTROL_SHIFTLIGHTS
void evaluate_shiftlight_display()
{
  if (!engine_running && (RPM > 2000)) {                                                                                            // Show off shift light segments during engine startup (>500rpm).
    engine_running = true;
    activate_shiftlight_segments(shiftlights_startup_buildup);
    #if DEBUG_MODE
      Serial.println(F("Showing shift light on engine startup."));
    #endif
    ignore_shiftlights_off_counter = 10;                                                                                            // Skip a few off cycles to allow segments to light up.

    #if EXHAUST_FLAP_WITH_M_BUTTON
      exhaust_flap_action_timer = millis();                                                                                         // Start tracking the exhaust flap.
    #endif
  }

  if (RPM >= START_UPSHIFT_WARN_RPM_ && RPM <= MID_UPSHIFT_WARN_RPM_) {                                                             // First yellow segment.                                                              
    activate_shiftlight_segments(shiftlights_start);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying first warning at RPM: %ld\n", RPM / 4);
      Serial.print(serial_debug_string);
    #endif                     
  } else if (RPM >= MID_UPSHIFT_WARN_RPM_ && RPM <= MAX_UPSHIFT_WARN_RPM_) {                                                        // Buildup from second yellow segment to reds.
    activate_shiftlight_segments(shiftlights_mid_buildup);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying increasing warning at RPM: %ld\n", RPM / 4);
      Serial.print(serial_debug_string);
    #endif
  } else if (RPM >= MAX_UPSHIFT_WARN_RPM_) {                                                                                        // Flash all segments.
    activate_shiftlight_segments(shiftlights_max_flash);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Flash max warning at RPM: %ld\n", RPM / 4);
      Serial.print(serial_debug_string);
    #endif
  } else {                                                                                                                          // RPM dropped. Disable lights.
    if (shiftlights_segments_active) {
      if (ignore_shiftlights_off_counter == 0) {
        PTCAN.sendMsgBuf(0x206, 2, shiftlights_off);                                                                                
        shiftlights_segments_active = false;
        #if DEBUG_MODE
          Serial.println(F("Deactivated shiftlights segments"));
        #endif 
      } else {
        ignore_shiftlights_off_counter--;
      }
    }
  }
}


void activate_shiftlight_segments(uint8_t* data)
{
    KCAN.sendMsgBuf(0x206, 2, data);                                                                     
    shiftlights_segments_active = true;
}


void evaluate_shiftlight_sync()
{
  if (!engine_warmed_up) {
    if (ptrxBuf[0] != last_var_rpm_can) {
      var_redline_position = ((ptrxBuf[0] * 0x32) + VAR_REDLINE_OFFSET_RPM) * 4;                                                    // This is where the variable redline actually starts on the KOMBI (x4).
      START_UPSHIFT_WARN_RPM_ = var_redline_position;                                                                              
      MID_UPSHIFT_WARN_RPM_ = var_redline_position + 2000;                                                                          // +500 RPM
      MAX_UPSHIFT_WARN_RPM_ = var_redline_position + 4000;                                                                          // +1000 RPM
      if (ptrxBuf[0] == 0x88) {                                                                                                     // DME is sending 6800 RPM.
        engine_warmed_up = true;
      }
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Set shiftlight RPMs to %lu %lu %lu. Variable redline is at %lu \n", 
                (START_UPSHIFT_WARN_RPM_ / 4), (MID_UPSHIFT_WARN_RPM_ / 4), 
                (MAX_UPSHIFT_WARN_RPM_ / 4), (var_redline_position / 4));
        Serial.print(serial_debug_string);
      #endif
      last_var_rpm_can = ptrxBuf[0];
    }
  } else {
    if (START_UPSHIFT_WARN_RPM_ != START_UPSHIFT_WARN_RPM) {
      START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;                                                                             // Return shiftlight RPMs to default setpoints.
      MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
      MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
      #if DEBUG_MODE
        Serial.println(F("Engine warmed up. Shiftlight setpoints reset to default."));
      #endif
    }
  }
}
#endif
