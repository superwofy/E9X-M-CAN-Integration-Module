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
  delayed_can_tx_msg m;
  if (mode == 0) {
    m = {dsc_on_buf, timeNow};
    dsc_tx.push(&m);
    m = {dsc_on_buf, timeNow + 20};
    dsc_tx.push(&m);
    #if DEBUG_MODE                        
      Serial.println("Sending DSC ON.");
    #endif
  } else if (mode == 1) {
    m = {dsc_mdm_dtc_buf, timeNow};
    dsc_tx.push(&m);
    m = {dsc_mdm_dtc_buf, timeNow + 20};
    dsc_tx.push(&m);
    #if DEBUG_MODE                        
      Serial.println("Sending DTC/MDM.");
    #endif
  } else {
    m = {dsc_off_buf, timeNow};
    dsc_tx.push(&m);
    m = {dsc_off_buf, timeNow + 20};
    dsc_tx.push(&m);
    #if DEBUG_MODE                        
      Serial.println("Sending DSC OFF.");
    #endif
  }
  dsc_program_status = mode;
}


void check_dsc_off_queue()
{
  if (!dsc_tx.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    dsc_tx.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      PTCAN.write(delayed_tx.tx_msg);
      dsc_tx.drop();
    }
  }
}
