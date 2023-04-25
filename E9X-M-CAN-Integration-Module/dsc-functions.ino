#if FTM_INDICATOR
void evaluate_indicate_ftm_status()
{
  if (pt_msg.buf[0] == 3 && !ftm_indicator_status) {
    kcan_write_msg(ftm_indicator_flash_buf);
    ftm_indicator_status = true;
    serial_log("Activated FTM indicator.");
  } else if (pt_msg.buf[0] == 0 && ftm_indicator_status) {
    kcan_write_msg(ftm_indicator_off_buf);
    ftm_indicator_status = false;
    serial_log("Deactivated FTM indicator.");
  }
}
#endif


void send_dsc_mode(uint8_t mode) {
  unsigned long timeNow = millis();
  delayed_can_tx_msg m;
  if (mode == 0) {
    m = {dsc_on_buf, timeNow};
    dsc_txq.push(&m);
    m = {dsc_on_buf, timeNow + 20};
    dsc_txq.push(&m);
    serial_log("Sending DSC ON.");
  } else if (mode == 1) {
    m = {dsc_mdm_dtc_buf, timeNow};
    dsc_txq.push(&m);
    m = {dsc_mdm_dtc_buf, timeNow + 20};
    dsc_txq.push(&m);
    serial_log("Sending DTC/MDM.");
  } else {
    m = {dsc_off_buf, timeNow};
    dsc_txq.push(&m);
    m = {dsc_off_buf, timeNow + 20};
    dsc_txq.push(&m);
    serial_log("Sending DSC OFF.");
  }
  dsc_program_status = mode;
}


void check_dsc_queue()
{
  if (!dsc_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    dsc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      if (delayed_tx.tx_msg.id == 0x592 || delayed_tx.tx_msg.id == 0x5A9) {
        kcan_write_msg(delayed_tx.tx_msg);
      } else {
        ptcan_write_msg(delayed_tx.tx_msg);
      }
      dsc_txq.drop();
    }
  }
}


#if LAUNCH_CONTROL_INDICATOR
void evaluate_lc_display()
{
  if (LC_RPM_MIN <= RPM && RPM <= LC_RPM_MAX) {
    if (clutch_pressed && !vehicle_moving) {
      if (!reverse_status) {
        kcan_write_msg(lc_cc_on_buf);
        lc_cc_active = true;
        if (dsc_program_status == 0) {
          mdm_with_lc = true;
          serial_log("Launch Control request DSC ON -> MDM/DTC.");
          send_dsc_mode(1);
        }
        serial_log("Displayed LC flag CC.");
        #if CONTROL_SHIFTLIGHTS
          activate_shiftlight_segments(shiftlights_max_flash_buf);
        #endif
      }
    } else {
      deactivate_lc_display();
      mdm_with_lc = false;                                                                                                          // Vehicle probably launched. MDM/DTC stays on
    }
  } else {
    if (lc_cc_active) {
      if (mdm_with_lc && dsc_program_status == 1) {
        serial_log("Launch Control aborted. MDM/DTC -> DSC ON.");
        send_dsc_mode(0);
        mdm_with_lc = false;
      }
    }
    deactivate_lc_display();
  }
}


void deactivate_lc_display()
{
  if (lc_cc_active) {
    kcan_write_msg(lc_cc_off_buf);
    lc_cc_active = false;
    serial_log("Deactivated LC flag CC.");
    #if CONTROL_SHIFTLIGHTS
      deactivate_shiftlights();
    #endif
  }  
}


void evaluate_clutch_status()
{        
  if (k_msg.buf[5] == 0xD) {
    if (!clutch_pressed) {
      clutch_pressed = true;
      serial_log("Clutch pressed.");
    }
  } else if (clutch_pressed) {
    clutch_pressed = false;
      serial_log("Clutch released.");
  }
}


void evaluate_reverse_status()
{
  if (k_msg.buf[0] == 0xFE) {
    reverse_status = true;
  } else {
    reverse_status = false;
  }
}
#endif


#if LAUNCH_CONTROL_INDICATOR || HDC
void evaluate_vehicle_moving()
{
  if (k_msg.buf[0] == 0 && k_msg.buf[1] == 0xD0) {
    if (vehicle_moving) {
      vehicle_moving = false;
      serial_log("Vehicle stationary.");
    }
  } else if (!vehicle_moving) {
    vehicle_moving = true;
    serial_log("Vehicle moving.");
  }
  #if HDC
    if (vehicle_moving) {
      if (speed_mph) {
       vehicle_speed = (((k_msg.buf[1] - 208 ) * 256) + k_msg.buf[0] ) / 16;
      } else {
        vehicle_speed = (((k_msg.buf[1] - 208 ) * 256) + k_msg.buf[0] ) / 10;
      }
      if (hdc_active) {
        if (vehicle_speed > max_hdc_speed) {
          serial_log("HDC deactivated due to high vehicle speed.");
          hdc_active = false;
          kcan_write_msg(hdc_cc_deactivated_on_buf);
          delayed_can_tx_msg m = {hdc_cc_deactivated_off_buf, millis() + 2000};
          kcan_cc_txq.push(&m);
        }
      }
    }
  #endif
}
#endif


#if HDC
void evaluate_hdc_button()
{
  if (k_msg.buf[0] == 0xFD) {                                                                                                       // Button pressed.
    if (!hdc_button_pressed) {
      if (!hdc_active) {
        if (!cruise_control_status && vehicle_speed >= min_hdc_speed && vehicle_speed <= max_hdc_speed) {
          ptcan_write_msg(set_hdc_cruise_control_buf);
          hdc_requested = true;                                                                                                     // Send request. "HDC" will only activate if cruise control conditions permit.
          serial_log("Sent HDC cruise control on message.");
        } else if (!vehicle_moving) {
          serial_log("Car must be moving for HDC.");
        } else {
          kcan_write_msg(hdc_cc_unavailable_on_buf);
          delayed_can_tx_msg m = {hdc_cc_unavailable_off_buf, millis() + 3000};
          kcan_cc_txq.push(&m);
          serial_log("Conditions not right for HDC. Sent CC.");
        }
      } else {
        ptcan_write_msg(cancel_hdc_cruise_control_buf);
        hdc_active = false;
        serial_log("Sent HDC cruise control on message.");
      }
      hdc_button_pressed = true;
    }
  } else {                                                                                                                          // Now receiving released (0xFC or 0xF4) messages from IHKA.
    hdc_button_pressed = false;
  }
}


void evaluate_cruise_control_status()
{
  if (k_msg.buf[5] == 0x58 || 
      (k_msg.buf[5] == 0x5A || k_msg.buf[5] == 0x5B || k_msg.buf[5] == 0x5C || k_msg.buf[5] == 0x5D)) {                            // Status is different based on ACC distance setting.
    if (!cruise_control_status) {
      cruise_control_status = true;
      if (hdc_requested) {
        kcan_write_msg(hdc_cc_activated_on_buf);
        hdc_active = true;
        hdc_requested = false;
        serial_log("HDC cruise control activated.");
      } else {
        serial_log("Cruise control activated.");
      }
    }
  } else {
    if (cruise_control_status) {
      cruise_control_status = false;
      if (hdc_active) {
        serial_log("HDC cruise control deactivated by user.");
        kcan_write_msg(hdc_cc_activated_off_buf);
        hdc_active = false;
        hdc_requested = false;
      } else {
        serial_log("Cruise control deactivated.");
      }
    }
  }
}


void evaluate_speed_units()
{
  speed_mph = (k_msg.buf[2] & 0xF0) == 0xB0 ? true : false;
  if (speed_mph) {
    min_hdc_speed = 6;
    max_hdc_speed = 37;
  } else {
    min_hdc_speed = 10;
    max_hdc_speed = 60;
  }
}
#endif
