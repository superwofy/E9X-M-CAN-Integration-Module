void veh_mode_checksum()
{
  // @amg6975
  // https://www.spoolstreet.com/threads/m-drive-and-mdm-in-non-m-cars.7155/post-107037
  power_mode_only_dme_veh_mode[0] &= 0xF0;                                                                                          // Remove checksum from byte
  uint16_t checksum = 0x7F1 + power_mode_only_dme_veh_mode[0] + power_mode_only_dme_veh_mode[1];                                    // Add up all bytes and the CAN ID
  checksum = (checksum & 0x00FF) + (checksum >> 8); //add upper and lower Bytes
  checksum &= 0x00FF; //throw away anything in upper Byte
  checksum = (checksum & 0b0001) + (checksum >> 4); //add first and second nibble
  checksum &= 0x000F; //throw away anything in upper nibble
  power_mode_only_dme_veh_mode[0] += checksum + 10;  //add checksum back into Byte0.
}

void send_mbutton_message(byte message[]) 
{
  message[2] = mbutton_checksum;

  byte send_stat = PTCAN.sendMsgBuf(0x1D9, 3, message);
  #if DEBUG_MODE
    if (send_stat != CAN_OK) {
      Serial.print(F("Error sending mbutton message. Re-trying. Error: "));
      Serial.println(send_stat);
    } else {
      message[0] == 0xFF ? Serial.println(F("Sent mbutton released.")) : Serial.println(F("Sent mbutton press."));
    }
  #else
  if (send_stat != CAN_OK) {
    delay(100);                                                                                                                     // Attempt to send again
    PTCAN.sendMsgBuf(0x1D9, 3, message);
  }
  #endif
  mbutton_released_timer = millis();
  mbutton_checksum < 0xFF ? mbutton_checksum++ : mbutton_checksum = 0xF0;                                                           // mbutton_checksum is between F0..FF
}


void mdrive_extra_functions()
{
  #if DTC_WITH_M_BUTTON
    if (dsc_program_status == 0) {                                                                                                  // Check to make sure DSC is in normal program before MDrive
      send_dtc_button_press();
    }
  #endif
  #if EDC_WITH_M_BUTTON
    if (edc_status == 1) {                                                                                                          // Make sure EDC is in Comfort/Sport mode
      send_edc_button_press();
      delay(50);
      send_edc_button_press();
      #if DEBUG_MODE
        Serial.println(F("Set EDC to MSport from Comfort with MDrive on."));
      #endif
    } else if (edc_status == 2) {
      send_edc_button_press();
      #if DEBUG_MODE
        Serial.println(F("Set EDC to MSport from Sport with MDrive on."));
      #endif
    }
  #endif
}
