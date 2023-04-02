void evaluate_dsc_program_from_cc()
{
  if (pt_msg.buf[7] != 0xAB) {                                                                                                      // Ignore fakes sent by the module.
    if (pt_msg.buf[0] == 0x40) {                                                                                                    // DSC or DTC/MDM light
      if (pt_msg.buf[1] == 0xB8) {                                                                                                  // DTC/MDM
        if (pt_msg.buf[3] == 0x1D) {                                                                                                // Active
          if (dsc_program_status != 1) {
            dsc_program_status = 1;
            #if DEBUG_MODE
              Serial.println("Stability control in DTC mode.");
            #endif
          }
        } else {
          if (dsc_program_status == 1) {
            dsc_program_status = 0;
            #if DEBUG_MODE
              Serial.println("Stability control fully activated from DTC/MDM.");
            #endif
          }
        }

      } else if (pt_msg.buf[1] == 0x24) {                                                                                           // DSC OFF
        if (pt_msg.buf[3] == 0x1D) {                                                                                                // Active
          if (dsc_program_status != 2) {
            dsc_program_status = 2;
            #if DEBUG_MODE
              Serial.println("Stability control fully OFF.");
            #endif
          }
        } else {
          if (dsc_program_status == 2) {
            dsc_program_status = 0;
            #if DEBUG_MODE
              Serial.println("Stability control fully activated from DSC OFF.");
            #endif
          }
        }
      }
    }
  }
}


#if FTM_INDICATOR
void evaluate_indicate_ftm_status()
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


void send_dsc_mode(uint8_t mode) {
  unsigned long timeNow = millis();
  delayedCanTxMsg m;
  if (mode == 0) {
    m = {dsc_on_buf, timeNow};
    dscTx.push(&m);
    m = {dsc_on_buf, timeNow + 50};
    dscTx.push(&m);
    #if DEBUG_MODE                        
      Serial.println("Sending DSC ON.");
    #endif
  } else if (mode == 1) {
    m = {dsc_mdm_dtc_buf, timeNow};
    dscTx.push(&m);
    m = {dsc_mdm_dtc_buf, timeNow + 50};
    dscTx.push(&m);
    #if DEBUG_MODE                        
      Serial.println("Sending DTC/MDM.");
    #endif
  } else {
    m = {dsc_off_buf, timeNow};
    dscTx.push(&m);
    m = {dsc_off_buf, timeNow + 50};
    dscTx.push(&m);
    #if DEBUG_MODE                        
      Serial.println("Sending DSC OFF.");
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
      dscTx.drop();
    }
  }
}
