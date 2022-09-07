void evaluate_dsc_ign_status()
{
  if (dsc_program_last_status_can != krxBuf[1]) {
    if (krxBuf[1] == 0xEA) {
      ignition = false;
      reset_runtime_variables();
      #if DEBUG_MODE
        Serial.println(F("Ignition OFF. Reset values."));
      #endif
    } else if (krxBuf[1] == 0xEC) {
      ignition = true;
      #if DEBUG_MODE
        Serial.println(F("Ignition ON."));
      #endif
    } else if (krxBuf[1] == 0xE0) {
      ignition = true;                                                                                                              // Just in case 0xEC was missed.
      dsc_program_status = 0;
      #if DEBUG_MODE
          Serial.println(F("Stability control fully activated."));
      #endif
    } else if (krxBuf[1] == 0xF0) {
      dsc_program_status = 1;
      #if DEBUG_MODE
          Serial.println(F("Stability control in DTC mode."));
      #endif
    } else if (krxBuf[1] == 0xE4) {
      dsc_program_status = 2;
      #if DEBUG_MODE
          Serial.println(F("Stability control fully OFF."));
      #endif
    }
    dsc_program_last_status_can = krxBuf[1];
  }

  if (krxBuf[1] == 0xEA) {
    if (!vehicle_awake) {
      vehicle_awake = true;    
      toggle_ptcan_sleep();                                                                                                         // Re-activate the controller.                                                                                         
      #if DEBUG_MODE
        Serial.println(F("Vehicle Awake."));
      #endif
    }
    vehicle_awake_timer = millis();                                                                                                 // Keep track of this message.
  }
  
  #if F_ZBE_WAKE
    send_zbe_wakeup();
  #endif
}


#if FTM_INDICATOR
void evaluate_ftm_status()
{
  if (ptrxBuf[0] == 0x03 && !ftm_indicator_status) {
    KCAN.sendMsgBuf(0x5A0, 8, ftm_indicator_flash);
    ftm_indicator_status = true;
    #if DEBUG_MODE
      Serial.println(F("Activated FTM indicator."));
    #endif
  } else if (ptrxBuf[0] == 0 && ftm_indicator_status) {
    KCAN.sendMsgBuf(0x5A0, 8, ftm_indicator_off);
    ftm_indicator_status = false;
    #if DEBUG_MODE
      Serial.println(F("Deactivated FTM indicator."));
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
    PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);                                                                                 // Two messages are sent during a quick press of the button (DTC mode).
    delay(5);
    PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);
    delay(5);
    PTCAN.sendMsgBuf(0x316, 2, dtc_button_released);                                                                                // Send one DTC released to indicate end of DTC button press.
    #if DEBUG_MODE                        
      Serial.println(F("Sent single DTC button press."));
    #endif
  }
} 


void send_dsc_off_sequence() 
{
  if (!sending_dsc_off) {
    sending_dsc_off = true;                                                                                                         // Begin the non-blocking sequence.
    PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);                                                                                 // Send the first press.
    KCAN.sendMsgBuf(0x5A9, 8, dsc_off_fake_cc_status);                                                                              // Trigger DSC OFF CC in Kombi, iDrive as soon as sequence starts.
  }
}


void non_blocking_dsc_off()
{
  if (sending_dsc_off) {
    if (sending_dsc_off_counter == 1) {
      KCAN.sendMsgBuf(0x5A9, 8, mdm_fake_cc_status);                                                                                // Start flashing MDM/DTC symbol
    }

    if ((millis() - sending_dsc_off_timer) >= 100) {                                                                                // Hopefully, none of the code blocks for 100ms+
      if (sending_dsc_off_counter < 25) {
        PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);
        
        sending_dsc_off_timer = millis();
        sending_dsc_off_counter++;
      } else {
        KCAN.sendMsgBuf(0x5A9, 8, mdm_fake_cc_status_off);
        PTCAN.sendMsgBuf(0x316, 2, dtc_button_released);
        #if DEBUG_MODE
          Serial.println(F("Sent DSC OFF sequence."));
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
