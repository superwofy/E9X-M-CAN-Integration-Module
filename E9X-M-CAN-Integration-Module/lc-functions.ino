#if LAUNCH_CONTROL_INDICATOR
void evaluate_lc_display()
{
  if (LC_RPM_MIN <= RPM && RPM <= LC_RPM_MAX) {
    if (clutch_pressed && !vehicle_moving) {
      KCAN.write(makeMsgBuf(0x598, 8, lc_cc_on, 0));
      lc_cc_active = true;
      if (dsc_program_status == 0) {
        mdm_with_lc = true;
        #if DEBUG_MODE
          Serial.println("Launch Control request DSC ON -> MDM/DTC.");
        #endif
        send_dtc_button_press();
      }
      #if DEBUG_MODE
        Serial.println("Displayed LC flag CC.");
      #endif
    } else {
      deactivate_lc_display();
      mdm_with_lc = false;                                                                                                          //Vehicle probably launched. MDM/DTC stays on
    }
  } else {
    if (lc_cc_active) {
      if (mdm_with_lc && dsc_program_status == 1) {
        #if DEBUG_MODE
          Serial.println("Launch Control aborted. MDM/DTC -> DSC ON.");
        #endif
        send_dtc_button_press();
        mdm_with_lc = false;
      }
    }
    deactivate_lc_display();
  }
}


void deactivate_lc_display()
{
  if (lc_cc_active) {
    KCAN.write(makeMsgBuf(0x598, 8, lc_cc_off, 0));
    lc_cc_active = false;
    #if DEBUG_MODE
        Serial.println("Deactivated LC flag CC.");
    #endif
  }  
}


void evaluate_vehicle_moving()
{
  if (k_msg.buf[0] == 0 && k_msg.buf[1] == 0xD0) {
    if (vehicle_moving) {
      vehicle_moving = false;
      #if DEBUG_MODE
        Serial.println("Vehicle stationary.");
      #endif
    }
  } else {
    if (!vehicle_moving) {
      vehicle_moving = true;
      #if DEBUG_MODE
        Serial.println("Vehicle moving.");
      #endif
    }
  }
}


void evaluate_clutch_status()
{        
  if (k_msg.buf[5] == 0xD) {
    if (!clutch_pressed) {
      clutch_pressed = true;
      #if DEBUG_MODE
        Serial.println("Clutch pressed.");
      #endif
    }
  } else if (clutch_pressed) {
    clutch_pressed = false;
    #if DEBUG_MODE
      Serial.println("Clutch released.");
    #endif
  }
}
#endif
