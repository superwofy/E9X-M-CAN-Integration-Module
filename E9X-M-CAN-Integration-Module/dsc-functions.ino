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
    ignition = true;                                                                                                          // Just in case 0xEC was missed.
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
#if F_ZBE_WAKE
  send_zbe_wakeup();
#endif
}


void evaluate_ftm_status()
{
  if (prxBuf[0] == 0x03 && !ftm_indicator_status) {
    KCAN.sendMsgBuf(0x5A0, 8, ftm_indicator_flash);
    ftm_indicator_status = true;
    #if DEBUG_MODE
      Serial.println(F("Activated FTM indicator."));
    #endif
  } else if (prxBuf[0] == 0 && ftm_indicator_status) {
    KCAN.sendMsgBuf(0x5A0, 8, ftm_indicator_off);
    ftm_indicator_status = false;
    #if DEBUG_MODE
      Serial.println(F("Deactivated FTM indicator."));
    #endif
  }
}


void send_dtc_button_press() 
// Correct timing sequence as per trace is: 
// button press -> delay(100) -> button press -> delay(50) -> button release -> delay(160) -> button release -> delay(160)
// However, that interferes with program timing. A small delay will still be accepted.
{
  PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);                                                                                   // Two messages are sent during a quick press of the button (DTC mode).
  delay(5);
  PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);
  delay(5);
  PTCAN.sendMsgBuf(0x316, 2, dtc_button_released);                                                                                  // Send one DTC released to indicate end of DTC button press.
  #if DEBUG_MODE                        
    Serial.println(F("Sent single DTC button press."));
  #endif
} 


void send_dsc_off_sequence() 
{
  PTCAN.sendMsgBuf(0x5A9, 8, dsc_off_fake_cc_status);                                                                               // Trigger DSC OFF CC in Kombi, iDrive as soon as sequence starts
  for (int i = 0; i < 26; i++) {                                                                                                    // >2.5s to send full DSC OFF sequence.
    if ((millis() - mbutton_released_timer) >= 1000) {                                                                              // keep sending M button released message
      send_mbutton_message(mbutton_released);
    }
    PTCAN.sendMsgBuf(0x316, 2, dtc_button_pressed);
    delay(100); 
  }
  PTCAN.sendMsgBuf(0x316, 2, dtc_button_released);
  #if DEBUG_MODE
    Serial.println(F("Sent DSC OFF sequence."));
  #endif
}