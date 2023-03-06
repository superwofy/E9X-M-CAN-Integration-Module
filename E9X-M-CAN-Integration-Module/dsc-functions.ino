void evaluate_dsc_ign_status()
{
  if (dsc_program_last_status_can != k_msg.buf[1]) {
    if (k_msg.buf[1] == 0xEA) {
      ignition = false;
      reset_runtime_variables();
      scale_mcu_speed();                                                                                                              // Now that the ignition is off, underclock the MCU
      #if DEBUG_MODE
        Serial.println("Ignition OFF. Reset values.");
      #endif
    } else if (k_msg.buf[1] == 0xEC) {
      ignition = true;
      scale_mcu_speed();
      #if DEBUG_MODE
        Serial.println("Ignition ON.");
      #endif
    } else if (k_msg.buf[1] == 0xE0) {
      ignition = true;                                                                                                              // Just in case 0xEC was missed.
      dsc_program_status = 0;
      #if DEBUG_MODE
          Serial.println("Stability control fully activated.");
      #endif
    } else if (k_msg.buf[1] == 0xF0) {
      dsc_program_status = 1;
      #if DEBUG_MODE
          Serial.println("Stability control in DTC mode.");
      #endif
    } else if (k_msg.buf[1] == 0xE4) {
      dsc_program_status = 2;
      #if DEBUG_MODE
          Serial.println("Stability control fully OFF.");
      #endif
    }
    dsc_program_last_status_can = k_msg.buf[1];
  }

  if (k_msg.buf[1] == 0xEA) {
    if (!vehicle_awake) {
      vehicle_awake = true;    
      toggle_transceiver_standby();                                                                                                 // Re-activate the transceivers.                                                                                         
      #if DEBUG_MODE
        Serial.println("Vehicle Awake.");
      #endif
    }
    vehicle_awake_timer = millis();                                                                                                 // Keep track of this message.
  }
}


#if FTM_INDICATOR
void evaluate_ftm_status()
{
  if (pt_msg.buf[0] == 3 && !ftm_indicator_status) {
    KCAN.write(makeMsgBuf(0x5A0, 8, ftm_indicator_flash, 1));
    ftm_indicator_status = true;
    #if DEBUG_MODE
      Serial.println("Activated FTM indicator.");
    #endif
  } else if (pt_msg.buf[0] == 0 && ftm_indicator_status) {
    KCAN.write(makeMsgBuf(0x5A0, 8, ftm_indicator_off, 1));
    ftm_indicator_status = false;
    #if DEBUG_MODE
      Serial.println("Deactivated FTM indicator.");
    #endif
  }
}
#endif


void send_dtc_button_press() 
// Correct timing sequence as per trace is: 
// button press -> delay(100) -> button press -> delay(50) -> button release -> delay(160) -> button release -> delay(160)
// However, that interferes with program timing. A small delay will still be accepted.
{
  if (!sending_dsc_off) {                                                                                                           // Ignore while DSC OFF seq is still being transmitted.
    PTCAN.write(makeMsgBuf(0x316, 2, dtc_button_pressed, 1));                                                                       // Two messages are sent during a quick press of the button (DTC mode).
    delay(5);
    PTCAN.write(makeMsgBuf(0x316, 2, dtc_button_pressed, 1));
    delay(5);
    PTCAN.write(makeMsgBuf(0x316, 2, dtc_button_released, 1));                                                                      // Send one DTC released to indicate end of DTC button press.
    #if DEBUG_MODE                        
      Serial.println("Sent single DTC button press.");
    #endif
  }
} 


void send_dsc_off_sequence() 
{
  if (!sending_dsc_off) {
    sending_dsc_off = true;                                                                                                         // Begin the non-blocking sequence.
    PTCAN.write(makeMsgBuf(0x316, 2, dtc_button_pressed, 1));                                                                       // Send the first press.
    KCAN.write(makeMsgBuf(0x5A9, 8, dsc_off_fake_cc_status, 1));                                                                    // Trigger DSC OFF CC in Kombi, iDrive as soon as sequence starts.
  }
}


void non_blocking_dsc_off()
{
  if (sending_dsc_off) {
    if (sending_dsc_off_counter == 1) {
      KCAN.write(makeMsgBuf(0x5A9, 8, mdm_fake_cc_status, 1));                                                                      // Start flashing MDM/DTC symbol to indicate transition.
    }

    if ((millis() - sending_dsc_off_timer) >= 100) {                                                                                // Hopefully, none of the code blocks for 100ms+.
      if (sending_dsc_off_counter < 25) {
        PTCAN.write(makeMsgBuf(0x316, 2, dtc_button_pressed, 1));
        sending_dsc_off_timer = millis();
        sending_dsc_off_counter++;
      } else {
        KCAN.write(makeMsgBuf(0x5A9, 8, mdm_fake_cc_status_off, 1));
        PTCAN.write(makeMsgBuf(0x316, 2, dtc_button_released, 1));
        #if DEBUG_MODE
          Serial.println("Sent DSC OFF sequence.");
        #endif
        sending_dsc_off = false;
        sending_dsc_off_counter = 0;
      }
    }
  }
}


void non_blocking_second_dtc_press()
{
  if (send_second_dtc_press) {
    if ((millis() - send_second_dtc_press_timer) >= 300) {
      send_second_dtc_press = false;
      send_dtc_button_press();
    } 
  }
}


void non_blocking_mdm_to_off()
{
  if (send_dsc_off_from_mdm) {
    if ((millis() - send_dsc_off_from_mdm_timer) >= 300) {
      send_dsc_off_from_mdm = false;
      send_dsc_off_sequence();
    }
  }       
}
