// Functions related to chassis (stability control, edc, steering etc.) go here.


void send_dsc_mode(uint8_t mode) {
  unsigned long time_now = millis();
  if (mode == 0) {
    m = {dsc_on_buf, time_now};
    dsc_txq.push(&m);
    m = {dsc_on_buf, time_now + 20};
    dsc_txq.push(&m);
    serial_log("Sending DSC ON.", 2);
  } else if (mode == 1) {
    m = {dsc_mdm_dtc_buf, time_now};
    dsc_txq.push(&m);
    m = {dsc_mdm_dtc_buf, time_now + 20};
    dsc_txq.push(&m);
    serial_log("Sending DTC/MDM.", 2);
  } else {
    m = {dsc_off_buf, time_now};
    dsc_txq.push(&m);
    m = {dsc_off_buf, time_now + 20};
    dsc_txq.push(&m);
    serial_log("Sending DSC OFF.", 2);
  }
  dsc_program_status = mode;
  #if F_NBT
    f_driving_dynamics_timer = 1002;
  #endif
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


void evaluate_reverse_gear_status(void) {
  if (k_msg.buf[0] == 0xFE) {
    if (!reverse_gear_status) {
      reverse_gear_status = true;
      serial_log("Reverse gear ON.", 2);
      #if FRONT_FOG_CORNER
        if ((left_fog_on || right_fog_on) && diag_transmit) {
          serial_log("Resetting corner fogs with reverse ON.", 2);
          kcan_write_msg(front_fogs_all_off_buf);
          left_fog_on = right_fog_on = false;
        }
      #endif
      #if MSA_RVC
        if (pdc_bus_status == 0xA4) {
          if (diag_transmit) {
            kcan_write_msg(pdc_on_camera_on_buf);
            #if F_NBT
              kcan2_write_msg(pdc_on_camera_on_buf);
            #endif
            serial_log("Activated full PDC and camera with reverse gear.", 2);
          }
        }
      #endif
    }
  } else {
    if (reverse_gear_status) {
      reverse_gear_status = false;
      serial_log("Reverse gear OFF.", 2);
      #if FRONT_FOG_CORNER
        if ((left_fog_on || right_fog_on) && diag_transmit) {
          serial_log("Resetting corner fogs with reverse OFF.", 2);
          kcan_write_msg(front_fogs_all_off_buf);
          left_fog_on = right_fog_on = false;
        }
      #endif
      #if PDC_AUTO_OFF
        if (handbrake_status && pdc_bus_status > 0x80) {
          unsigned long time_now = millis();
          kcan_write_msg(pdc_button_presssed_buf);
          #if F_NBT
            kcan2_write_msg(pdc_button_presssed_buf);
          #endif
          m = {pdc_button_released_buf, time_now + 100};
          pdc_buttons_txq.push(&m);
          m = {pdc_button_released_buf, time_now + 200};
          pdc_buttons_txq.push(&m);
          serial_log("Disabled PDC after reverse gear OFF and handbrake already up.", 2);
        }
      #endif
      #if AUTO_TOW_VIEW_RVC
        if (rvc_tow_view_by_module && !rvc_tow_view_by_driver && pdc_bus_status == 0xA5) {
          bitWrite(rvc_settings[0], 3, 0);                                                                                          // Set tow hitch view to OFF.
          serial_log("Disabled camera tow view after reverse gear OFF.", 2);
          CAN_message_t new_rvc_settings = make_msg_buf(0x38F, 4, rvc_settings);
          kcan_write_msg(new_rvc_settings);
          #if F_NBT
            kcan2_write_msg(new_rvc_settings);
          #endif
          rvc_tow_view_by_module = false;
        }
      #endif
      #if REVERSE_BEEP
        reverse_beep_sent = false;                                                                                                  // Reset the beep flag.
      #endif
    }
  }
}


void evaluate_vehicle_moving(void) {
  if (k_msg.buf[0] == 0 && k_msg.buf[1] == 0xD0) {
    if (vehicle_moving) {
      vehicle_moving = false;
      serial_log("Vehicle stationary.", 2);
    }
  } else if (!vehicle_moving) {
    vehicle_moving = true;
    serial_log("Vehicle moving.", 2);
    #if INTERMITTENT_WIPERS
      if (intermittent_wipe_active) {
        intermittent_wipe_timer = intermittent_intervals[intermittent_setting];                                                     // Wipe immediately once car is moving.
        serial_log("Intermittent wiping sped up.", 2);
      }
    #endif
  }
}


void evaluate_indicated_speed(void) {
  if (vehicle_moving) {
    if (speed_mph) {
      indicated_speed = (((k_msg.buf[1] - 208 ) * 256) + k_msg.buf[0] ) / 16;
    } else {
      indicated_speed = (((k_msg.buf[1] - 208 ) * 256) + k_msg.buf[0] ) / 10;
    }
    #if HDC
      if (hdc_active) {
        if (indicated_speed > hdc_deactivate_speed) {
          serial_log("HDC deactivated due to high vehicle speed.", 2);
          hdc_active = false;
          kcan_write_msg(hdc_cc_deactivated_on_buf);
          #if F_NBT
            kcan2_write_msg(hdc_cc_deactivated_on_buf);
          #endif
          m = {hdc_cc_deactivated_off_buf, millis() + 2000};
          ihk_extra_buttons_cc_txq.push(&m);
        }
      }
    #endif
  } else {
    indicated_speed = 0;
  }
}


void send_f_road_inclination(void) {
  if (f_chassis_inclination_timer >= 100) {
    uint8_t f_road_inclination[] = {0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF};                                                      // Angle formula simlar to 56.1.2.
    f_road_inclination[2] = f_vehicle_angle & 0xFF;                                                                                 // Send in LE.
    f_road_inclination[3] = 0x20 | f_vehicle_angle >> 8;                                                                            // Byte3 lower 4 bits fixed to 2 - QU_AVL_LOGR_RW Signal value is valid, permanent.
    CAN_message_t f_road_inclination_buf = make_msg_buf(0x163, 8, f_road_inclination);
    ptcan_write_msg(f_road_inclination_buf);
    f_chassis_inclination_timer = 0;
  }
}


void send_f_longitudinal_acceleration(void) {
  if (f_chassis_longitudinal_timer >= 100) {
    uint8_t f_longitudinal_acceleration[] = {0xFF, 0xFF, 0, 0, 0xFF, 0x2F};                                                         // Similar to 55.0.2. Byte7 fixed 0x2F - Signal value is valid QU_ACLNX_COG.
    uint16_t raw_value = 0;
    // Combine the second half of byte3 with byte2 to form 12-bit signed integer.
    if (pt_msg.buf[2] > 0xF) {
      raw_value = ((pt_msg.buf[3] & 0xF) << 8 | pt_msg.buf[2]);
    } else {
      raw_value = ((pt_msg.buf[3] & 0xF) << 4 | pt_msg.buf[2]);
    }
    int16_t signed_value = (raw_value & 0x800) ? (raw_value | 0xF000) : raw_value;
    e_longitudinal_acceleration = signed_value * 0.025;                                                                             // Value in m/s^2.

    longitudinal_acceleration = round((e_longitudinal_acceleration + 65) / 0.002);
    f_longitudinal_acceleration[1] = 0xF << 4 | f_longitudinal_acceleration_alive_counter;
    f_longitudinal_acceleration_alive_counter == 0xE ? f_longitudinal_acceleration_alive_counter = 0 
                                                    : f_longitudinal_acceleration_alive_counter++;
    f_longitudinal_acceleration[2] = longitudinal_acceleration & 0xFF;                                                              // Convert and transmit in LE.
    f_longitudinal_acceleration[3] = longitudinal_acceleration >> 8;
    
    f_longitudinal_acceleration_crc.restart();
    for (uint8_t i = 1; i < 6; i++) {
      f_longitudinal_acceleration_crc.add(f_longitudinal_acceleration[i]);
    }
    f_longitudinal_acceleration[0] = f_longitudinal_acceleration_crc.calc();
    
    CAN_message_t f_longitudinal_acceleration_buf = make_msg_buf(0x199, 6, f_longitudinal_acceleration);
    ptcan_write_msg(f_longitudinal_acceleration_buf);
    f_chassis_longitudinal_timer = 0;
  }
}


void send_f_lateral_acceleration(void) {
  if (f_chassis_lateral_timer >= 100) {
    uint8_t f_lateral_acceleration[] = {0xFF, 0xFF, 0, 0, 0xFF, 0x2F};                                                              // Similar to 55.0.2. Byte7 fixed 0x2F - Signal value is valid QU_ACLNY_COG.
    uint16_t raw_value = 0;
    // Combine byte 4 with the first half of byte 3 to form 12-bit signed integer.
    raw_value = (pt_msg.buf[4] << 4 | ((pt_msg.buf[3] >> 4) & 0x0F));
    int16_t signed_value = (raw_value & 0x800) ? (raw_value | 0xF000) : raw_value;
    e_lateral_acceleration = signed_value * 0.025;                                                                                  // Value in m/s^2.

    lateral_acceleration = round((e_lateral_acceleration + 65) / 0.002);
    f_lateral_acceleration[1] = 0xF << 4 | f_lateral_acceleration_alive_counter;
    f_lateral_acceleration_alive_counter == 0xE ? f_lateral_acceleration_alive_counter = 0 
                                                    : f_lateral_acceleration_alive_counter++;
    f_lateral_acceleration[2] = longitudinal_acceleration & 0xFF;                                                                   // Convert and transmit in LE.
    f_lateral_acceleration[3] = longitudinal_acceleration >> 8;

    f_lateral_acceleration_crc.restart();
    for (uint8_t i = 1; i < 6; i++) {
      f_lateral_acceleration_crc.add(f_lateral_acceleration[i]);
    }
    f_lateral_acceleration[0] = f_lateral_acceleration_crc.calc();

    CAN_message_t f_lateral_acceleration_buf = make_msg_buf(0x19A, 6, f_lateral_acceleration);
    ptcan_write_msg(f_lateral_acceleration_buf);
    f_chassis_lateral_timer = 0;
  }
}


void send_f_yaw_rate(void) {
  if (f_chassis_yaw_timer >= 100) {
    uint8_t f_yaw_rate[] = {0xFF, 0xFF, 0, 0x80, 0xFF, 0x2F};                                                                       // M4 (VYAW_VEH) 56.0.2. Byte5 fixed 0x2F - Signal value is valid QU_VYAW_VEH.
    
    if (vehicle_moving) {
      uint16_t raw_value = 0;
      // Combine the second half of byte6 with byte5 to form 12-bit signed integer.
      if (pt_msg.buf[5] > 0xF) {
        raw_value = ((pt_msg.buf[6] & 0xF) << 8 | pt_msg.buf[5]);
      } else {
        raw_value = ((pt_msg.buf[6] & 0xF) << 4 | pt_msg.buf[5]);
      }
      int16_t signed_value = (raw_value & 0x800) ? (raw_value | 0xF000) : raw_value;
      e_yaw_rate = signed_value * 0.05;                                                                                             // Value in degrees/sec.

      yaw_rate = round((e_yaw_rate + 163.84) / 0.005);
      f_yaw_rate[2] = yaw_rate & 0xFF;                                                                                              // Convert and transmit in LE.
      f_yaw_rate[3] = yaw_rate >> 8;
    }

    CAN_message_t f_yaw_rate_buf = make_msg_buf(0x19F, 6, f_yaw_rate);
    #if F_NBT
      kcan2_write_msg(f_yaw_rate_buf);
    #endif
    
    #if F_NIVI
      f_yaw_rate[1] = 0xF << 4 | f_yaw_alive_counter;
      f_yaw_alive_counter == 0xE ? f_yaw_alive_counter = 0 : f_yaw_alive_counter++;

      f_yaw_rate_crc.restart();
      for (uint8_t i = 1; i < 6; i++) {
        f_yaw_rate_crc.add(f_yaw_rate[i]);
      }
      f_yaw_rate[0] = f_yaw_rate_crc.calc();

      f_yaw_rate_buf = make_msg_buf(0x19F, 6, f_yaw_rate);
      ptcan_write_msg(f_yaw_rate_buf);
    #endif
    
    f_chassis_yaw_timer = 0;
  }
}


void evaluate_real_speed(void) {
  if (pt_msg.buf[0] > 0xF) {
    real_speed = round(((pt_msg.buf[1] & 0xF) << 8 | pt_msg.buf[0]) * 0.1);                                                         // KM/h
  } else {
    real_speed = round(((pt_msg.buf[1] & 0xF) << 4 | pt_msg.buf[0]) * 0.1);
  }
}


void evaluate_vehicle_direction(void) {
  e_vehicle_direction = (pt_msg.buf[1] >> 4) & 0b0111;                                                                              // ST_VEH_DVCO is the last three bits of the first halfof byte1.
}


void send_f_speed_status(void) {
  if (f_chassis_speed_timer >= 100) {
    uint8_t f_speed[] = {0, 0, 0, 0, 0};                                                                                            // Message is the same format as Flexray 55.3.4.

    // Second half of byte4 is QU_V_VEH_COG. 1 = Signal valid, 0xF = invalid.
    if (e_vehicle_direction == 0) {                                                                                                 // Not moving.
      f_speed[4] = 0x81;
    } else if (e_vehicle_direction == 9) {                                                                                          // Moving forward.
      f_speed[4] = 0x91;
    } else if (e_vehicle_direction == 0xA) {                                                                                        // Moving backwards.
      f_speed[4] = 0xA1;
    } else {                                                                                                                        // Invalid / Unknown.
      f_speed[4] = 0x8F;
    }                                                                                                                                   
    
    f_speed[1] = 0xC << 4 | f_speed_alive_counter;
    f_speed_alive_counter == 0xE ? f_speed_alive_counter = 0 : f_speed_alive_counter += 2;                                          // This message increments the alive counter by 2.
    if (e_vehicle_direction == 9 || e_vehicle_direction == 0xA) {
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
    CAN_message_t f_speed_buf = make_msg_buf(0x1A1, 5, f_speed);
    #if F_NIVI
      ptcan_write_msg(f_speed_buf);
    #endif
    #if F_NBT
      kcan2_write_msg(f_speed_buf);
    #endif
    f_chassis_speed_timer = 0;
  }
}


void send_f_standstill_status(void) {
  if (f_standstill_status_timer >= 1000) {
    uint8_t f_standstill_status[] = {0, 0, 0};
    f_standstill_status[1] = 0xF << 4 | f_standstill_status_alive_counter;
    f_standstill_status_alive_counter == 0xE ? f_standstill_status_alive_counter = 0 
                                          : f_standstill_status_alive_counter++;

    if (vehicle_moving) {
      f_standstill_status[2] = 0;
    } else {
      f_standstill_status[2] = 2;
    }

    f_standstill_status_crc.restart();
    for (uint8_t i = 1; i < 3; i++) {
      f_standstill_status_crc.add(f_standstill_status[i]);
    }
    f_standstill_status[0] = f_standstill_status_crc.calc();
    CAN_message_t f_standstill_status_buf = make_msg_buf(0x2ED, 3, f_standstill_status);
    #if F_NBT
      kcan2_write_msg(f_standstill_status_buf);
    #endif
    f_standstill_status_timer = 0;
  }
}


void send_f_steering_angle(void) {
  uint8_t f_steering_angle[] = {0, 0, 0, 0, 0xFF, 0x7F, 0x2F};
  f_steering_angle[1] = 0xF << 4 | f_steering_angle_alive_counter;
  f_steering_angle_alive_counter == 0xE ? f_steering_angle_alive_counter = 0 
                                        : f_steering_angle_alive_counter++;

  uint16_t converted_angle = round((steering_angle + 1440.11) / 0.04395);
  f_steering_angle[2] = converted_angle & 0xFF;                                                                                     // Convert and transmit in LE.
  f_steering_angle[3] = converted_angle >> 8;

  f_steering_angle_crc.restart();
  for (uint8_t i = 1; i < 7; i++) {
    f_steering_angle_crc.add(f_steering_angle[i]);
  }
  f_steering_angle[0] = f_steering_angle_crc.calc();
  CAN_message_t f_steering_angle_buf = make_msg_buf(0x301, 7, f_steering_angle);
  #if F_NIVI
    ptcan_write_msg(f_steering_angle_buf);
  #endif
  #if F_NBT
    kcan2_write_msg(f_steering_angle_buf);
  #endif


  uint8_t f_steering_angle_effective[] = {0, 0, 0, 0, 0xF1, 0xFF, 0xFF};
  f_steering_angle_effective[1] = f_steering_angle[1];
  f_steering_angle_effective[2] = f_steering_angle[2];
  f_steering_angle_effective[3] = f_steering_angle[3];

  f_steering_angle_effective_crc.restart();
  for (uint8_t i = 1; i < 7; i++) {
    f_steering_angle_effective_crc.add(f_steering_angle_effective[i]);
  }
  f_steering_angle_effective[0] = f_steering_angle_effective_crc.calc();
  CAN_message_t f_steering_angle_effective_buf = make_msg_buf(0x302, 7, f_steering_angle_effective);
  #if F_NIVI
    ptcan_write_msg(f_steering_angle_effective_buf);
  #endif
  #if F_NBT
    kcan2_write_msg(f_steering_angle_effective_buf);
  #endif
}


void evaluate_hdc_button(void) {
  if (k_msg.buf[0] == 0xFD) {                                                                                                       // Button pressed.
    if (!hdc_button_pressed) {
      if (!hdc_active) {
        if (vehicle_moving) {
          if (indicated_speed <= max_hdc_speed) {
            if (!cruise_control_status) {
              stalk_message_counter == 0xFF ? stalk_message_counter = 0xF0 : stalk_message_counter++;
              uint8_t set_hdc_checksums[] = {0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0, 2, 3, 4};
              uint8_t set_hdc_cruise_control[] = {set_hdc_checksums[stalk_message_counter - 0xF0], stalk_message_counter, 8, 0xFC};
              set_hdc_cruise_control_buf = make_msg_buf(0x194, 4, set_hdc_cruise_control);
              unsigned long time_now = millis();
              m = {set_hdc_cruise_control_buf, time_now};
              hdc_txq.push(&m);
              m = {set_hdc_cruise_control_buf, time_now + 20};
              hdc_txq.push(&m);
              hdc_requested = true;                                                                                                 // Send request. "HDC" will only activate if cruise control conditions permit.
              serial_log("Sent HDC cruise control ON message.", 2);
            } else {
              hdc_active = true;
              kcan_write_msg(hdc_cc_activated_on_buf);
              #if F_NBT
                kcan2_write_msg(hdc_cc_activated_on_buf);
              #endif
              serial_log("HDC activated with cruise control already ON.", 2);
            }
          }
        } else {
          serial_log("Car must be moving for HDC.", 2);
        }
      } else {
        stalk_message_counter == 0xFF ? stalk_message_counter = 0xF0 : stalk_message_counter++;
        uint8_t cancel_hdc_checksums[] = {0xFD, 0xFE, 0xFF, 0, 2, 3, 4, 5, 6, 7, 8, 9, 0xA, 0xB, 0xC};
        uint8_t cancel_hdc_cruise_control[] = {cancel_hdc_checksums[stalk_message_counter - 0xF0], stalk_message_counter, 0x10, 0xFC};
        cancel_hdc_cruise_control_buf = make_msg_buf(0x194, 4, cancel_hdc_cruise_control);
        unsigned long time_now = millis();
        m = {cancel_hdc_cruise_control_buf, time_now};
        hdc_txq.push(&m);
        m = {cancel_hdc_cruise_control_buf, time_now + 20};
        hdc_txq.push(&m);
        serial_log("Sent HDC cruise control OFF message.", 2);
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
        #if F_NBT
          kcan2_write_msg(hdc_cc_activated_on_buf);
        #endif
        hdc_active = true;
        hdc_requested = false;
        serial_log("HDC cruise control activated.", 2);
      } else {
        serial_log("Normal cruise control activated.", 2);
      }
    }
  } else {
    if (cruise_control_status) {
      cruise_control_status = false;
      if (hdc_active) {
        serial_log("HDC cruise control deactivated by user.", 2);
        kcan_write_msg(hdc_cc_activated_off_buf);
        #if F_NBT
          kcan2_write_msg(hdc_cc_activated_off_buf);
        #endif
        hdc_active = hdc_requested = false;
      } else {
        serial_log("Normal cruise control deactivated.", 2);
      }
    } else {
      if (hdc_requested) {
        hdc_active = hdc_requested = cruise_control_status = false;
        serial_log("Cruise control did not activate when HDC was requested.", 2);
      }
    }
  }
}


void evaluate_cruise_stalk_message(void) {
  stalk_message_counter = pt_msg.buf[1];
}


void check_hdc_queue(void) {
  if (!hdc_txq.isEmpty()) {
    hdc_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      ptcan_write_msg(delayed_tx.tx_msg);
      hdc_txq.drop();
    }
  }
}


void evaluate_speed_units(void) {
  speed_mph = (k_msg.buf[2] & 0xF0) == 0xB0 ? true : false;
  if (speed_mph) {
    max_hdc_speed = 22;
    hdc_deactivate_speed = 37;
    immobilizer_max_speed = 12;
  } else {
    max_hdc_speed = 35;
    hdc_deactivate_speed = 60;
    immobilizer_max_speed = 20;
  }
}


void send_servotronic_message(void) {
  servotronic_message[0] += 0x10;                                                                                                   // Increase alive counter.
  if (servotronic_message[0] > 0xEF) {                                                                                              // Alive(first half of byte) must be between 0..E.
    servotronic_message[0] = 0;
  }
  
  servotronic_message[0] &= 0xF0;                                                                                                   // Discard current mode
  if (mdrive_status && mdrive_svt[cas_key_number] >= 0xF1) {                                                                        // Servotronic in sport mode.
    servotronic_message[0] += 9;
  } else {
    servotronic_message[0] += 8;
  }
  ptcan_write_msg(make_msg_buf(SVT_FAKE_EDC_MODE_CANID, 2, servotronic_message));
}


void send_f_distance_messages(void) {
  uint8_t distance[] = {0, 0, 0, 0, 0, 0, 0xF2};                                                                                    // Byte7 = signal valid.
  distance[4] = f_distance_alive_counter & 0xFF;                                                                                    // Alive counter is transmitted in LE.
  distance[5] = f_distance_alive_counter >> 8;

  distance[0] = k_msg.buf[4];                                                                                                       // Distance_4_5.
  distance[1] = k_msg.buf[5];
  distance[2] = k_msg.buf[2];                                                                                                       // Distance_2_3.
  distance[3] = k_msg.buf[3];
  kcan2_write_msg(make_msg_buf(0x1C4, 7, distance));                                                                                // 1C4 is flexray 271.4.8.

  distance[0] = k_msg.buf[0];
  distance[1] = k_msg.buf[1];
  distance[2] = k_msg.buf[0];                                                                                                       // Distance_1_0 from 1A6 bytes are repeated.
  distance[3] = k_msg.buf[1];
  kcan2_write_msg(make_msg_buf(0x1C5, 7, distance));


  // Counter increases by 0xA until it overflows 0x2FFF, then restarts.
  f_distance_alive_counter == 0x2FF5 ? f_distance_alive_counter = 0x2000 : f_distance_alive_counter += 0xA;  
}
