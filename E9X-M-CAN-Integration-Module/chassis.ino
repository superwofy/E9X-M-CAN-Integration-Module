// Functions related to chassis (stability control, edc, steering etc.) go here.


void send_dsc_mode(uint8_t mode) {
  time_now = millis();
  if (mode == 0) {
    m = {dsc_on_buf, time_now};
    dsc_txq.push(&m);
    m = {dsc_on_buf, time_now + 20};
    dsc_txq.push(&m);
    serial_log("Sending DSC ON.");
  } else if (mode == 1) {
    m = {dsc_mdm_dtc_buf, time_now};
    dsc_txq.push(&m);
    m = {dsc_mdm_dtc_buf, time_now + 20};
    dsc_txq.push(&m);
    serial_log("Sending DTC/MDM.");
  } else {
    m = {dsc_off_buf, time_now};
    dsc_txq.push(&m);
    m = {dsc_off_buf, time_now + 20};
    dsc_txq.push(&m);
    serial_log("Sending DSC OFF.");
  }
  dsc_program_status = mode;
}


void check_dsc_queue(void) {
  if (!dsc_txq.isEmpty()) {
    dsc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      ptcan_write_msg(delayed_tx.tx_msg);
      dsc_txq.drop();
    }
  }
}


#if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR || FRONT_FOG_CORNER || MSA_RVC || F_NIVI
void evaluate_reverse_gear_status(void) {
  if (k_msg.buf[0] == 0xFE) {
    if (!reverse_gear_status) {
      reverse_gear_status = true;
      serial_log("Reverse gear ON.");
      #if FRONT_FOG_CORNER
        if ((left_fog_on || right_fog_on) && diag_transmit) {
          serial_log("Resetting corner fogs with reverse ON.");
          kcan_write_msg(front_fogs_all_off_buf);
          left_fog_on = right_fog_on = false;
        }
      #endif
      #if MSA_RVC
        if (pdc_status == 0xA4) {
          if (diag_transmit) {
            kcan_write_msg(pdc_on_camera_on_buf);
            serial_log("Activated full PDC and camera with reverse gear.");
          }
        }
      #endif
    }
  } else {
    if (reverse_gear_status) {
      reverse_gear_status = false;
      serial_log("Reverse gear OFF.");
      if ((left_fog_on || right_fog_on) && diag_transmit) {
        serial_log("Resetting corner fogs with reverse OFF.");
        kcan_write_msg(front_fogs_all_off_buf);
        left_fog_on = right_fog_on = false;
      }
    }
  }
}
#endif


void evaluate_vehicle_moving(void) {
  if (k_msg.buf[0] == 0 && k_msg.buf[1] == 0xD0) {
    if (vehicle_moving) {
      vehicle_moving = false;
      serial_log("Vehicle stationary.");
    }
  } else if (!vehicle_moving) {
    vehicle_moving = true;
    serial_log("Vehicle moving.");
  }
  #if HDC || ANTI_THEFT_SEQ || FRONT_FOG_CORNER || F_NIVI
    if (vehicle_moving) {
      if (speed_mph) {
       indicated_speed = (((k_msg.buf[1] - 208 ) * 256) + k_msg.buf[0] ) / 16;
      } else {
        indicated_speed = (((k_msg.buf[1] - 208 ) * 256) + k_msg.buf[0] ) / 10;
      }
      #if HDC
        if (hdc_active) {
          if (indicated_speed > max_hdc_speed) {
            serial_log("HDC deactivated due to high vehicle speed.");
            hdc_active = false;
            kcan_write_msg(hdc_cc_deactivated_on_buf);
            m = {hdc_cc_deactivated_off_buf, millis() + 2000};
            ihk_extra_buttons_cc_txq.push(&m);
          }
        }
      #endif
    }
  #endif
}


#if F_NIVI
void send_f_road_inclination(void) {
  f_road_inclination[2] = f_vehicle_angle & 0xFF;                                                                                   // Send in LE.
  f_road_inclination[3] = 0x20 | f_vehicle_angle >> 8;                                                                              // Byte3 lower 4 bits fixed to 2 - QU_AVL_LOGR_RW Signal value is valid, permanent.
  f_road_inclination_buf = make_msg_buf(0x163, 8, f_road_inclination);
  ptcan_write_msg(f_road_inclination_buf);
}


void send_f_longitudinal_acceleration(void) {
  e_long_acceleration = ((int16_t)(k_msg.buf[2] << 8 | k_msg.buf[3])) * 0.0001;                                                     // Value in m/s^2.
  longitudinal_acceleration = round((e_long_acceleration + 65) / 0.002);

  f_longitudinal_acceleration[1] = 0xF << 4 | f_longitudinal_acceleration_alive_counter;
  f_longitudinal_acceleration_alive_counter == 0xE ? f_longitudinal_acceleration_alive_counter = 0 
                                                   : f_longitudinal_acceleration_alive_counter++;
  f_longitudinal_acceleration[2] = longitudinal_acceleration & 0xFF;                                                                // Convert and transmit in LE.
  f_longitudinal_acceleration[3] = longitudinal_acceleration >> 8;
  f_longitudinal_acceleration_buf = make_msg_buf(0x199, 6, f_longitudinal_acceleration);
  ptcan_write_msg(f_longitudinal_acceleration_buf);
}


void send_f_yaw_rate(void) {
  f_yaw_rate[1] = 0xF << 4 | f_yaw_alive_counter;
  f_yaw_alive_counter == 0xE ? f_yaw_alive_counter = 0 : f_yaw_alive_counter++;
  f_yaw_rate[2] = yaw_rate;
  f_yaw_rate_buf = make_msg_buf(0x19F, 6, f_yaw_rate);
  ptcan_write_msg(f_yaw_rate_buf);
}


void send_f_speed_status(void) {
  real_speed = round(((pt_msg.buf[1] & 0xF) << 8 | pt_msg.buf[0]) * 0.1);                                                           // KM/h

  vehicle_direction = pt_msg.buf[1] >> 4;
  if (vehicle_direction == 8) {                                                                                                     // Not moving.
    f_speed[4] = 0x81;
  } else if (vehicle_direction == 9) {                                                                                              // Moving forward.
    f_speed[4] = 0x91;
  } else if (vehicle_direction == 0xA) {                                                                                            // Moving backwards.
    f_speed[4] = 0xA1;
  }                                                                                                                                 // QU_V_VEH_COG hardcoded to 1 (Signal valid).

  f_speed[1] = 0xC << 4 | f_speed_alive_counter;
  f_speed_alive_counter == 0xE ? f_speed_alive_counter = 0 : f_speed_alive_counter += 2;                                            // This message increments the alive counter by 2.
  if (vehicle_direction == 9 || vehicle_direction == 0xA) {
    uint16_t f_temp_speed = real_speed / 0.015625;
    if (speed_mph) {
      f_temp_speed = round(f_temp_speed * 1.6);
    }
    f_speed[2] = f_temp_speed & 0xFF;
    f_speed[3] = f_temp_speed >> 8;
  } else {
    f_speed[2] = f_speed[3] = 0;
  }

  f_speed_crc.restart();
  for (uint8_t i = 1; i < 5; i++) {
    f_speed_crc.add(f_speed[i]);
  }
  f_speed[0] = f_speed_crc.calc();
  f_speed_buf = make_msg_buf(0x1A1, 5, f_speed);
  ptcan_write_msg(f_speed_buf);
}
#endif


#if HDC
void evaluate_hdc_button(void) {
  if (k_msg.buf[0] == 0xFD) {                                                                                                       // Button pressed.
    if (!hdc_button_pressed) {
      if (!hdc_active) {
        if (!cruise_control_status && indicated_speed >= min_hdc_speed && indicated_speed <= max_hdc_speed) {
          ptcan_write_msg(set_hdc_cruise_control_buf);
          hdc_requested = true;                                                                                                     // Send request. "HDC" will only activate if cruise control conditions permit.
          serial_log("Sent HDC cruise control ON message.");
        } else if (!vehicle_moving) {
          serial_log("Car must be moving for HDC.");
        } else {
          kcan_write_msg(hdc_cc_unavailable_on_buf);
          m = {hdc_cc_unavailable_off_buf, millis() + 3000};
          ihk_extra_buttons_cc_txq.push(&m);
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


void evaluate_cruise_control_status(void) {
  if (k_msg.buf[5] == 0x58 || 
      (k_msg.buf[5] == 0x5A || k_msg.buf[5] == 0x5B || k_msg.buf[5] == 0x5C || k_msg.buf[5] == 0x5D)) {                             // Status is different based on ACC distance setting.
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
#endif


#if HDC || ANTI_THEFT_SEQ || FRONT_FOG_CORNER || F_NIVI
void evaluate_speed_units(void) {
  speed_mph = (k_msg.buf[2] & 0xF0) == 0xB0 ? true : false;
  #if HDC
    if (speed_mph) {
      min_hdc_speed = 12;
      max_hdc_speed = 22;
    } else {
      min_hdc_speed = 20;
      max_hdc_speed = 35;
    }
  #endif
}
#endif


#if SERVOTRONIC_SVT70
void send_servotronic_message(void) {
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
  ptcan_write_msg(make_msg_buf(SVT_FAKE_EDC_MODE_CANID, 2, servotronic_message));
}
#endif
