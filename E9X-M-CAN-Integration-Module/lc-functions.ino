#if LAUNCH_CONTROL_INDICATOR
void evaluate_lc_display()
{
  if (RPM >= LC_RPM_MIN && RPM <= LC_RPM_MAX) {
    if (clutch_pressed && !vehicle_moving) {
      KCAN.sendMsgBuf(0x598, 8, lc_cc_on);
      lc_cc_active = true;
      #if DEBUG_MODE
        Serial.println(F("Displayed LC flag CC."));
      #endif
    } else if (lc_cc_active) {
      KCAN.sendMsgBuf(0x598, 8, lc_cc_off);
      lc_cc_active = false;
      #if DEBUG_MODE
        Serial.println(F("Deactivated LC flag CC."));
      #endif
    }
  } else if (lc_cc_active){
    KCAN.sendMsgBuf(0x598, 8, lc_cc_off);
    lc_cc_active = false;
    #if DEBUG_MODE
        Serial.println(F("Deactivated LC flag CC."));
    #endif
  }
}


void evaluate_vehicle_moving()
{
  if (prxBuf[0] == 0 && prxBuf[1] == 0xD0) {
    if (vehicle_moving) {
      vehicle_moving = false;
      #if DEBUG_MODE
        Serial.println(F("Vehicle stationary."));
      #endif
    }
  } else {
    if (!vehicle_moving) {
      vehicle_moving = true;
      #if DEBUG_MODE
        Serial.println(F("Vehicle moving."));
      #endif
    }
  }
}


void evaluate_clutch_status()
{
  if (ignition) {        
    if (krxBuf[5] == 0x0D) {
      if (!clutch_pressed) {
        clutch_pressed = true;
        #if DEBUG_MODE
          Serial.println(F("Clutch pressed."));
        #endif
      }
    } else if (clutch_pressed) {
      clutch_pressed = false;
      #if DEBUG_MODE
        Serial.println(F("Clutch released."));
      #endif
    }
  }
}
#endif