void read_mdrive_settings_from_eeprom()
{
  mdrive_dsc = EEPROM.read(1);
  mdrive_power = EEPROM.read(2);
  mdrive_edc = EEPROM.read(3);
  mdrive_svt = EEPROM.read(4);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Loaded MDrive settings from EEPROM: DSC 0x%X POWER 0x%X EDC 0x%X SVT 0x%X.\n", 
            mdrive_dsc, mdrive_power, mdrive_edc, mdrive_svt);
    Serial.print(serial_debug_string);
  #endif
  if (mdrive_dsc == 0x3) {
      mdrive_message[1] = 0x1;                                                                                                      // DSC unchanged, MDrive off.
  } else if (mdrive_dsc == 0x7) {
     mdrive_message[1] = 0x5;                                                                                                       // DSC off, MDrive off.
  } else if (mdrive_dsc == 0xB) {
      mdrive_message[1] = 0x9;                                                                                                      // DSC on, MDrive off.
  } else if (mdrive_dsc == 0x13) {
      mdrive_message[1] = 0x11;                                                                                                     // DSC MDM, MDrive off.
  }
  mdrive_message[2] = mdrive_power;                                                                                                 // Copy POWER as is.
  mdrive_message[3] = mdrive_edc;                                                                                                   // Copy EDC as is.
  if (mdrive_svt == 0xE9) {
      mdrive_message[4] = 0x41;                                                                                                     // SVT normal, MDrive off.
  } else if (mdrive_svt == 0xF1) {
      mdrive_message[4] = 0x81;                                                                                                     // SVT sport, MDrive off.
  } 
}


void update_mdrive_settings_in_eeprom()
{
  EEPROM.update(1, mdrive_dsc);                                                                                                     // EEPROM lifetime approx. 100k writes. Always update, never write()!
  EEPROM.update(2, mdrive_power);
  EEPROM.update(3, mdrive_edc);
  EEPROM.update(4, mdrive_svt);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Saved MDrive settings to EEPROM.\n");
    Serial.print(serial_debug_string);
  #endif
}


void toggle_mdrive_message_active()
{
  if (!sending_dsc_off) {
    if (mdrive_status) {                                                                                                            // Turn off MDrive.
      #if DEBUG_MODE
        Serial.println(F("Status MDrive off."));
      #endif
      mdrive_message[1] -= 1;                                                                                                       // Decrement bytes 1 (6MT, DSC mode) and 4 (SVT) to deactivate.
      mdrive_message[4] -= 0x10;
      mdrive_status = false;
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = false;
      #endif
    } else {                                                                                                                        // Turn on MDrive.
      #if DEBUG_MODE
        Serial.println(F("Status MDrive on."));
      #endif
      mdrive_message[1] += 1;
      mdrive_message[4] += 0x10;
      mdrive_status = true;
      #if EXHAUST_FLAP_CONTROL
        exhaust_flap_sport = true;
      #endif
    }
  }
}


void toggle_mdrive_dsc()
{
  if (mdrive_status) {
    if (mdrive_dsc == 0x7) {
      if (dsc_program_status == 0) {
        #if DEBUG_MODE
          Serial.println(F("MDrive request DSC ON -> DSC OFF."));
        #endif
        send_dsc_off_sequence();
      } else if (dsc_program_status == 2) {
        //Do nothing
      } else {                                                                                                                      // Must be in MDM/DTC.
        send_dtc_button_press();
        send_dsc_off_from_mdm = true;
        send_dsc_off_from_mdm_timer = millis();
      }
    } else if (mdrive_dsc == 0x13) {                                                                                                // DSC MDM (DTC in non-M) requested.
      if (dsc_program_status == 0) {
        #if DEBUG_MODE
          Serial.println(F("MDrive request DSC ON -> MDM/DTC."));
        #endif
        send_dtc_button_press();
      } else if (dsc_program_status == 2) {
        #if DEBUG_MODE
          Serial.println(F("MDrive request DSC OFF -> MDM/DTC."));
        #endif
        send_dtc_button_press();
        send_second_dtc_press = true;
        send_second_dtc_press_timer = millis();       
      }
    } else if (mdrive_dsc == 0xB) {                                                                                                 // DSC ON requested.
      if (dsc_program_status != 0) {
        #if DEBUG_MODE
          Serial.println(F("MDrive request DSC OFF -> DSC ON."));
        #endif
        send_dtc_button_press();
      }
    }
  } else {
    if (mdrive_dsc == 0x13 || mdrive_dsc == 0x7) {                                                                                  // If MDrive was set to change DSC, restore back to DSC ON.
      if (dsc_program_status != 0) {
        send_dtc_button_press();
        #if DEBUG_MODE
          Serial.println(F("MDrive request DSC back ON."));
        #endif
      }
    }
  }
}


void send_mdrive_message()
{
  mdrive_message[0] += 10;
  can_checksum_update(mdrive_message, 6, 0x399);                                                                                    // Recalculate checksum.
  PTCAN.sendMsgBuf(0x399, 6, mdrive_message);       
  mdrive_message_timer = millis();
  #if DEBUG_MODE
//  debug_can_message(0x399, 6, mdrive_message);
  #endif                                                                      
}


void update_mdrive_message_settings(bool reset)
{
  mdrive_settings_change = true;
  if (reset) {
    mdrive_dsc = 0x03;                                                                                                              // Unchanged
    mdrive_power = 0;                                                                                                               // Unchanged
    mdrive_edc = 0x20;                                                                                                              // Unchanged
    mdrive_svt = 0xE9;                                                                                                              // Normal
	  #if DEBUG_MODE
      Serial.println(F("Reset MDrive settings."));
    #endif
  }

  //Decode settings
  mdrive_dsc = ptrxBuf[0];                                                                                                          // 0x3 unchanged, 0x07 off, 0x13 MDM, 0xB on.
  mdrive_power = ptrxBuf[1];                                                                                                        // 0 unchanged, 0x10 normal, 0x20 sport, 0x30 sport+.
  mdrive_edc = ptrxBuf[2];                                                                                                          // 0x20(Unchanged), 0x21(Comfort) 0x22(Normal) 0x2A(Sport).
  mdrive_svt = ptrxBuf[4];                                                                                                          // 0xE9 Normal, 0xF1 Sport, 0xEC/0xF4/0xE4 Reset. E0/E1-invalid?
  
  //Build acknowledge message
  if (mdrive_dsc == 0x3) {
    if (!mdrive_status) {
      mdrive_message[1] = 0x1;                                                                                                      // DSC unchanged, MDrive off.
    } else {
      mdrive_message[1] = 0x2;                                                                                                      // DSC unchanged, MDrive on.
    }
  } else if (mdrive_dsc == 0x7) {
    if (!mdrive_status) {
      mdrive_message[1] = 0x5;                                                                                                      // DSC off, MDrive off.
    } else {
      mdrive_message[1] = 0x6;                                                                                                      // DSC off, MDrive on.
    }
  } else if (mdrive_dsc == 0xB) {
    if (!mdrive_status) {
      mdrive_message[1] = 0x9;                                                                                                      // DSC on, MDrive off.
    } else {
      mdrive_message[1] = 0xA;                                                                                                      // DSC on, MDrive on.
    }
  } else if (mdrive_dsc == 0x13) {
    if (!mdrive_status) {
      mdrive_message[1] = 0x11;                                                                                                     // DSC MDM, MDrive off.
    } else {
      mdrive_message[1] = 0x12;                                                                                                     // DSC MDM, MDrive on.
    }
  }
  mdrive_message[2] = mdrive_power;                                                                                                 // Copy POWER as is.
  mdrive_message[3] = mdrive_edc;                                                                                                   // Copy EDC as is.
  if (mdrive_svt == 0xE9) {
    if (!mdrive_status) {
      mdrive_message[4] = 0x41;                                                                                                     // SVT normal, MDrive off.
    } else {
      mdrive_message[4] = 0x51;                                                                                                     // SVT normal, MDrive on.
    }
  } else if (mdrive_svt == 0xF1) {
    if (!mdrive_status) {
      mdrive_message[4] = 0x81;                                                                                                     // SVT sport, MDrive off.
    } else {
      mdrive_message[4] = 0x91;                                                                                                     // SVT sport, MDrive on.
    }
  }          
  
  #if DEBUG_MODE
	if (!reset) {
		sprintf(serial_debug_string, "Received iDrive settings: DSC 0x%X POWER 0x%X EDC 0x%X SVT 0x%X.\n", 
				mdrive_dsc, mdrive_power, mdrive_edc, mdrive_svt);
		Serial.print(serial_debug_string);
	}
  #endif
}


// @amg6975
// https://www.spoolstreet.com/threads/MDrive-and-mdm-in-non-m-cars.7155/post-107037
void can_checksum_update(uint8_t *message, uint8_t len, uint16_t canid)
{
  message[0] &= 0xF0;                                                                                                               // Remove checksum from byte.
  // Add up all bytes and the CAN ID
  uint16_t checksum = canid;
  for (uint8_t i = 0; i < len; i++) {
    checksum += message[i];
  }                                 
  checksum = (checksum & 0x00FF) + (checksum >> 8); //add upper and lower Bytes
  checksum &= 0x00FF; //throw away anything in upper Byte
  checksum = (checksum & 0b0001) + (checksum >> 4); //add first and second nibble
  checksum &= 0x000F; //throw away anything in upper nibble

  // Magic number needs to be added depending on CANID.
  if (canid == DME_FAKE_VEH_MODE_CANID) {                                                                                                   
    message[0] += checksum + 10;                                                                                                    // Add the checksum back to Byte0.
  } else if (canid == 0x399) {
    message[0] += checksum + 6;
  }
}


void send_power_mode()
{
  power_mode_only_dme_veh_mode[0] += 0x10;                                                                                          // Increase alive counter.
  if (power_mode_only_dme_veh_mode[0] > 0xEF) {                                                                                     // Alive(first half of byte) must be between 0..E.
    power_mode_only_dme_veh_mode[0] = 0;
  }

  if (power_mode || ((mdrive_power == 0x20 || mdrive_power == 0x30) && mdrive_status)) {                                            // Activate sport throttle mapping if POWER from console on or Sport/Sport+ selected in MDrive.
    power_mode_only_dme_veh_mode[1] = 0x22;                                                                                         // Sport
    can_checksum_update(power_mode_only_dme_veh_mode, 2, DME_FAKE_VEH_MODE_CANID);
    PTCAN.sendMsgBuf(DME_FAKE_VEH_MODE_CANID, 2, power_mode_only_dme_veh_mode);
    digitalWrite(POWER_LED_PIN, HIGH);
  } else {
    power_mode_only_dme_veh_mode[1] = 0x11;                                                                                         // Normal
    can_checksum_update(power_mode_only_dme_veh_mode, 2, DME_FAKE_VEH_MODE_CANID);
    PTCAN.sendMsgBuf(DME_FAKE_VEH_MODE_CANID, 2, power_mode_only_dme_veh_mode);
    digitalWrite(POWER_LED_PIN, LOW);
  }
}

void send_servotronic_message()
{
  if (ignition) {
    servotronic_message[0] += 0x10;                                                                                                 // Increase alive counter.
    if (servotronic_message[0] > 0xEF) {                                                                                            // Alive(first half of byte) must be between 0..E.
      servotronic_message[0] = 0;
    }
    
    servotronic_message[0] &= 0xF0;                                                                                                 // Discard current mode
    if (mdrive_status && mdrive_svt == 0xF1) {                                                                                      // Servotronic in sport mode.
      servotronic_message[0] += 9;
    } else {
      servotronic_message[0] += 8;
    }

    PTCAN.sendMsgBuf(SVT_FAKE_EDC_MODE_CANID, 2, servotronic_message); 
    #if DEBUG_MODE
      //debug_can_message(SVT_FAKE_EDC_MODE_CANID, 2, servotronic_message);
    #endif    
  }
}
