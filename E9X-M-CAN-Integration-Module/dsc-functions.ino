void evaluate_dsc_ign_status()
{
  if (dsc_program_last_status_can != k_msg.buf[1]) {
    if (k_msg.buf[1] == 0xEA) {
      ignition = false;
      reset_runtime_variables();
      scale_mcu_speed();                                                                                                            // Now that the ignition is off, underclock the MCU
      #if DEBUG_MODE
        Serial.println("Ignition OFF. Reset values.");
      #endif
    } else if (k_msg.buf[1] == 0xEC) {
      ignition = true;
      scale_mcu_speed();
      #if SERVOTRONIC_SVT70
        if (!digitalRead(POWER_BUTTON_PIN)) {                                                                                       // If POWER button is being held when turning on ignition, allow SVT diagnosis.
          digitalWrite(DCAN_STBY_PIN, LOW);
          diagnose_svt = true;
          KCAN.write(servotronic_cc_on_buf);                                                                                        // Indicate that diagnosing is now possible.
        }
      #endif
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
    KCAN.write(ftm_indicator_flash_buf);
    ftm_indicator_status = true;
    #if DEBUG_MODE
      Serial.println("Activated FTM indicator.");
    #endif
  } else if (pt_msg.buf[0] == 0 && ftm_indicator_status) {
    KCAN.write(ftm_indicator_off_buf);
    ftm_indicator_status = false;
    #if DEBUG_MODE
      Serial.println("Deactivated FTM indicator.");
    #endif
  }
}
#endif


void check_dtc_button_queue()
{
  if (!dtcTx.isEmpty()) {
    delayedCanTxMsg delayedTx;
    dtcTx.peek(&delayedTx);
    if (millis() >= delayedTx.transmitTime) {
      PTCAN.write(delayedTx.txMsg);
      dtcTx.drop();
    }
  }
}


void send_dtc_button_press(bool second) 
// Correct timing sequence as per trace is: 
// button press -> delay(100) -> button press -> delay(50) -> button release -> delay(160) -> button release -> delay(160)
// However, that interferes with program timing. A small delay will still be accepted.
{
  if (!sending_dsc_off) {                                                                                                           // Ignore while DSC OFF seq is still being transmitted.
    unsigned long timeNow = millis();
    if (second) {
      timeNow += 300;
    }
    PTCAN.write(dtc_button_pressed_buf);                                                                                            // Two messages are sent during a quick press of the button (DTC mode).
    delayedCanTxMsg m = {dtc_button_pressed_buf, timeNow + 100};
    dtcTx.push(&m);
    m = {dtc_button_released_buf, timeNow + 150};                                                                                   // Send two DTC released to indicate end of DTC button press.
    dtcTx.push(&m);
    m = {dtc_button_released_buf, timeNow + 310};
    dtcTx.push(&m);                                                                   
    #if DEBUG_MODE                        
      Serial.println("Sent single DTC button press.");
    #endif
  }
}


void check_dsc_off_queue()
{
  if (!dscTx.isEmpty()) {
    delayedCanTxMsg delayedTx;
    dscTx.peek(&delayedTx);
    if (millis() >= delayedTx.transmitTime) {
      PTCAN.write(delayedTx.txMsg);
      sending_dsc_off_counter++;
      dscTx.drop();
    }
    if (sending_dsc_off_counter == 25) {
      KCAN.write(mdm_fake_cc_status_off_buf);                                                                                       // Stop flashing MDM/DTC symbol
      #if DEBUG_MODE
        Serial.println("Sent DSC OFF sequence.");
      #endif
      sending_dsc_off = false;
      sending_dsc_off_counter = 0;
    }
  }
}


void send_dsc_off_sequence() 
{
  if (!sending_dsc_off) {
    sending_dsc_off = true;                                                                                                         // Begin the non-blocking sequence.
    unsigned long timeNow = millis();

    PTCAN.write(dtc_button_pressed_buf);                                                                                                           // Send the first press.
    KCAN.write(dsc_off_fake_cc_status_buf);                                                                                         // Trigger DSC OFF CC in Kombi, iDrive as soon as sequence starts.
    KCAN.write(mdm_fake_cc_status_buf);                                                                                             // Start flashing MDM/DTC symbol to indicate transition.
    
    delayedCanTxMsg m;
    uint8_t i;
    for (i = 1; i < 25; i++) {
      m = {dtc_button_pressed_buf, timeNow + (i * 100)};
      dscTx.push(&m);
    }
    m = {dtc_button_released_buf, timeNow + (i * 100)};
    dscTx.push(&m);
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
