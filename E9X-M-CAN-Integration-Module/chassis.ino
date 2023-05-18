// Functions related to chassis (stability control, edc, steeting etc.) go here.


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


void check_dsc_queue() {
  if (!dsc_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    dsc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      ptcan_write_msg(delayed_tx.tx_msg);
      dsc_txq.drop();
    }
  }
}


#if LAUNCH_CONTROL_INDICATOR
void evaluate_clutch_status() {
  if (k_msg.buf[5] == 0xD) {
    if (!clutch_pressed) {
      clutch_pressed = true;
      serial_log("Clutch pedal pressed.");
    }
  } else if (clutch_pressed) {
    clutch_pressed = false;
    serial_log("Clutch pedal released.");
  }
}
#endif


#if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR
void evaluate_reverse_status() {
  if (k_msg.buf[0] == 0xFE) {
    if (!reverse_status) {
      reverse_status = true;
      serial_log("Reverse gear engaged.");
    }
  } else {
    reverse_status = false;
  }
}
#endif


#if LAUNCH_CONTROL_INDICATOR || HDC
void evaluate_vehicle_moving() {
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
void evaluate_hdc_button() {
  if (k_msg.buf[0] == 0xFD) {                                                                                                       // Button pressed.
    if (!hdc_button_pressed) {
      if (!hdc_active) {
        if (!cruise_control_status && vehicle_speed >= min_hdc_speed && vehicle_speed <= max_hdc_speed) {
          ptcan_write_msg(set_hdc_cruise_control_buf);
          hdc_requested = true;                                                                                                     // Send request. "HDC" will only activate if cruise control conditions permit.
          serial_log("Sent HDC cruise control ON message.");
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
        serial_log("Sent HDC cruise control OFF message.");
      }
      hdc_button_pressed = true;
    }
  } else {                                                                                                                          // Now receiving released (0xFC or 0xF4) messages from IHKA.
    hdc_button_pressed = false;
  }
}


void evaluate_cruise_control_status() {
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


void evaluate_speed_units() {
  speed_mph = (k_msg.buf[2] & 0xF0) == 0xB0 ? true : false;
  if (speed_mph) {
    min_hdc_speed = 12;
    max_hdc_speed = 22;
  } else {
    min_hdc_speed = 20;
    max_hdc_speed = 35;
  }
}
#endif


#if SERVOTRONIC_SVT70
void send_servotronic_message() {
  servotronic_message[0] += 0x10;                                                                                                   // Increase alive counter.
  if (servotronic_message[0] > 0xEF) {                                                                                              // Alive(first half of byte) must be between 0..E.
    servotronic_message[0] = 0;
  }
  
  servotronic_message[0] &= 0xF0;                                                                                                   // Discard current mode
  if (mdrive_status && mdrive_svt == 0xF1) {                                                                                        // Servotronic in sport mode.
    servotronic_message[0] += 9;
  } else {
    servotronic_message[0] += 8;
  }
  ptcan_write_msg(makeMsgBuf(SVT_FAKE_EDC_MODE_CANID, 2, servotronic_message));
}
#endif
