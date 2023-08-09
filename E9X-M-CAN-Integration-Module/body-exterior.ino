// General body functions dealing with exterior electronics go here.


#if DIM_DRL
void evaluate_drl_status(void) {
  if (k_msg.buf[1] == 0x32) {
    if (!drl_status) {
      drl_status = true;
      serial_log("DRLs ON.");
    }
  } else {
    if (drl_status) {
      drl_status = false;
      serial_log("DRLs OFF");
    }
  }
}


void check_drl_queue(void) {
  if (!dim_drl_txq.isEmpty()) {
    if (diag_transmit) {
      dim_drl_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        dim_drl_txq.drop();
        last_drl_action_timer = millis();
      }
    } else {
      dim_drl_txq.flush();
    }
  }
}
#endif


#if DIM_DRL || FRONT_FOG_CORNER || INDICATE_TRUNK_OPENED
void evaluate_indicator_status_dim(void) {
  #if DIM_DRL
  time_now = millis();
  if (drl_status && diag_transmit) {
  #endif
    if(k_msg.buf[0] == 0x80 || k_msg.buf[0] == 0xB1) {                                                                              // Off or Hazards
      #if DIM_DRL
        if (right_dimmed) {
          m = {right_drl_bright_buf, time_now + 100};
          dim_drl_txq.push(&m);
          right_dimmed = false;
          serial_log("Restored right DRL brightness.");
        } else if (left_dimmed) {
          m = {left_drl_bright_buf, time_now + 100};
          dim_drl_txq.push(&m);
          left_dimmed = false;
          serial_log("Restored left DRL brightness.");
        }
      #endif
      #if INDICATE_TRUNK_OPENED
        if (k_msg.buf[0] == 0xB1) {
          hazards_on = true;
        } else {
          hazards_on = false;
        }
      #endif
      indicators_on = false;
    } else if (k_msg.buf[0] == 0x91) {                                                                                              // Left indicator
      #if DIM_DRL
        if (right_dimmed) {
          m = {right_drl_bright_buf, time_now + 100};
          dim_drl_txq.push(&m);
          right_dimmed = false;
          serial_log("Restored right DRL brightness.");
        }
        m = {left_drl_dim_buf, time_now + 100};
        dim_drl_txq.push(&m);
        if (!left_dimmed) {
          serial_log("Dimmed left DRL.");
        }
        left_dimmed = true;
      #endif
      indicators_on = true;
    } else if (k_msg.buf[0] == 0xA1) {                                                                                              // Right indicator
      #if DIM_DRL
        if (left_dimmed) {
          m = {left_drl_bright_buf, time_now + 100};
          dim_drl_txq.push(&m);
          left_dimmed = false;
          serial_log("Restored left DRL brightness.");
        }
        m = {right_drl_dim_buf, time_now + 100};
        dim_drl_txq.push(&m);
        if (!right_dimmed) {
          serial_log("Dimmed right DRL.");
        }
        right_dimmed = true;
      #endif
    }
    indicators_on = true;
  #if DIM_DRL
  }
  #endif
}
#endif


#if DOOR_VOLUME || AUTO_MIRROR_FOLD || ANTI_THEFT_SEQ || HOOD_OPEN_GONG
void evaluate_door_status(void) {
  if (k_msg.buf[1] != last_door_status) {
    if (k_msg.buf[1] == 1) {
      if (!right_door_open) {
        right_door_open = true;
        #if RHD
          serial_log("Driver's door open.");
          #if UNFOLD_WITH_DOOR
            if (unfold_with_door_open) {
              toggle_mirror_fold();
              unfold_with_door_open = false;
            }
          #endif
        #else
          serial_log("Passenger's door open.");
        #endif
        #if DOOR_VOLUME
          send_volume_request_door();
        #endif
      }
    } else if (k_msg.buf[1] == 4) {
      if (!left_door_open) {
        left_door_open = true;
        #if RHD
          serial_log("Passenger's door open.");
        #else
          serial_log("Driver's door open.");
          #if UNFOLD_WITH_DOOR
            if (unfold_with_door_open) {
              toggle_mirror_fold();
              unfold_with_door_open = false;
            }
          #endif
        #endif
        #if DOOR_VOLUME
          send_volume_request_door();
        #endif
      }
    } else if (k_msg.buf[1] == 5) {
      left_door_open = right_door_open = true;
      serial_log("Both front doors open.");
      #if UNFOLD_WITH_DOOR
        if (unfold_with_door_open) {
          toggle_mirror_fold();
          unfold_with_door_open = false;
        }
      #endif
    } else if (k_msg.buf[1] == 0) {
      if (left_door_open) {
        left_door_open = false;
        #if RHD
          serial_log("Passenger's door closed.");
        #else
          serial_log("Driver's door closed.");
        #endif
        #if DOOR_VOLUME
          send_volume_request_door();
        #endif
      }
      if (right_door_open) {
        right_door_open = false;
        #if RHD
          serial_log("Driver's door closed.");
        #else
          serial_log("Passenger's door closed.");
        #endif
        #if DOOR_VOLUME
          send_volume_request_door();
        #endif
      }
    } 
    last_door_status = k_msg.buf[1];
  }

  #if HOOD_OPEN_GONG
  if (k_msg.buf[2] != last_hood_status) {
    if (k_msg.buf[2] == 4) {
      serial_log("Hood opened.");
      if (diag_transmit) {
        if (!vehicle_moving && terminal_r) {
          kcan_write_msg(cc_gong_buf); 
        }
      }
      last_hood_status = k_msg.buf[2];
    } else if (k_msg.buf[2] == 0) {
      serial_log("Hood closed.");
      last_hood_status = k_msg.buf[2];
    }
    // Ignore other statuses such as boot open.
  }
  #endif
}
#endif


#if EXHAUST_FLAP_CONTROL
void control_exhaust_flap_user(void) {
  if (engine_running) {
    if (exhaust_flap_sport) {                                                                                                       // Flap always open in sport mode.
      if (exhaust_flap_action_timer >= 500) {
        if (!exhaust_flap_open) {
          actuate_exhaust_solenoid(LOW);
          serial_log("Opened exhaust flap with MDrive.");
        }
      }
    }
  } else {
    #if QUIET_START
      if (ignition || terminal_r || terminal_50) {
        if (exhaust_flap_open) {
          actuate_exhaust_solenoid(HIGH);                                                                                           // Close the flap (if vacuum still available)
          serial_log("Quiet start enabled. Exhaust flap closed.");
        }
      } else {
        if (!exhaust_flap_open) {
          actuate_exhaust_solenoid(LOW);
          serial_log("Released exhaust flap from quiet start.");
        }
      }
    #endif
  }
}


void control_exhaust_flap_rpm(void) {
  if (engine_running) {
    if (!exhaust_flap_sport) {
      if (exhaust_flap_action_timer >= exhaust_flap_action_interval) {                                                              // Avoid vacuum drain, oscillation and apply startup delay.
        if (RPM >= EXHAUST_FLAP_QUIET_RPM) {                                                                                        // Open at defined rpm setpoint.
          if (!exhaust_flap_open) {
            actuate_exhaust_solenoid(LOW);
            serial_log("Exhaust flap opened at RPM setpoint.");
          }
        } else {
          if (exhaust_flap_open) {
            actuate_exhaust_solenoid(HIGH);
            serial_log("Exhaust flap closed.");
          }
        }
      }
    }
  }
}


void actuate_exhaust_solenoid(bool activate) {
  digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, activate);
  exhaust_flap_action_timer = 0;
  exhaust_flap_open = !activate;                                                                                                    // Flap position is the inverse of solenoid state. When active, the flap is closed.
}
#endif


#if CKM || EDC_CKM_FIX
void evaluate_key_number_remote(void) {
  if (k_msg.buf[0] / 11 != cas_key_number) {
    cas_key_number = k_msg.buf[0] / 11;                                                                                             // Key 1 = 0, Key 2 = 11, Key 3 = 22...
    if (cas_key_number > 2) {
      serial_log("Received wrong key personalisation number. Assuming default");
      cas_key_number = 3;                                                                                                           // Default / Key "4".
    }
    #if DEBUG_MODE
    else {
      sprintf(serial_debug_string, "Received remote key number: %d.", cas_key_number + 1);
      serial_log(serial_debug_string);
    }
    #endif
    #if CKM
      console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;
    #endif
  }
}


void check_key_changed(void) {
  uint8_t cas_key_number_can = k_msg.buf[1] & 0b1111;
  if (cas_key_number_can != cas_key_number) {
    if (cas_key_number_can > 2) {
      // Ignore. This should only happen if the key is lost by the PGS. We should still have the number from when the car was unlocked.
    } else {
      cas_key_number = cas_key_number_can;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Received new key number after unlock: %d.", cas_key_number + 1);
        serial_log(serial_debug_string);
      #endif
      #if CKM
        console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;
        send_dme_power_ckm();                                                                                                       // Update iDrive in case key remote changed
      #endif
    }
  }
}
#endif


#if AUTO_MIRROR_FOLD || INDICATE_TRUNK_OPENED
void evaluate_remote_button(void) {
  if (!engine_running) {                                                                                                            // Ignore if locking car while running.
    if (k_msg.buf[2] != last_lock_status_can) {                                                                                     // Lock/Unlock messages are sent many times. Should only react to the first.
      if (k_msg.buf[1] == 0x30 || k_msg.buf[1] == 0x33) {
        time_now = millis();
        if (k_msg.buf[2] == 4) {
          if (!left_door_open && !right_door_open) {
            #if AUTO_MIRROR_FOLD
              if (diag_transmit) {
                lock_button_pressed = true;
                serial_log("Remote lock button pressed. Checking mirror status.");
                m = {frm_status_request_a_buf, time_now + 200};
                mirror_fold_txq.push(&m);
                frm_status_requested = true;
              }
              #if UNFOLD_WITH_DOOR
                unfold_with_door_open = false;
              #endif
              #if ANTI_THEFT_SEQ_ALARM
                if (alarm_led) {
                  m = {alarm_led_off_buf, time_now + 100};                                                                          // Release control of the LED so that alarm can control it.
                  anti_theft_txq.push(&m);
                  m = {alarm_led_off_buf, time_now + 400};
                  anti_theft_txq.push(&m);
                  alarm_led = false;
                  lock_led = true;
                  led_message_counter = 60;                                                                                         // Make sure we're ready once Terminal R cycles.
                  serial_log("Deactivated DWA LED when door locked.");
                }
              #endif
            #endif
          }
        }
        
        #if AUTO_MIRROR_FOLD
        else if (k_msg.buf[2] == 1) {
          if (diag_transmit) {
            unlock_button_pressed = true;
            serial_log("Remote unlock button pressed. Checking mirror status.");
            m = {frm_status_request_a_buf, time_now + 200};
            mirror_fold_txq.push(&m);
            frm_status_requested = true;
          }
          #if ANTI_THEFT_SEQ_ALARM
            lock_led = false;
          #endif
        } 
        #endif

        #if INDICATE_TRUNK_OPENED
        else if (k_msg.buf[2] == 0x10) {
          if (visual_signal_ckm && !engine_running && !hazards_on && !indicators_on) {
            serial_log("Remote trunk button pressed. Flashing hazards.");
            kcan_write_msg(flash_hazards_buf);
          }
        }
        #endif

        else if (k_msg.buf[2] == 0) {
          serial_log("Remote buttons released.");
        }
      }
      last_lock_status_can = k_msg.buf[2];
    }
  }
}
#endif


#if AUTO_MIRROR_FOLD
void evaluate_mirror_fold_status(void) {
  if (frm_status_requested) {                                                                                                       // Make sure the request came from this module.
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x22) {
      if (k_msg.buf[4] == 0) {
        mirrors_folded = false;
        serial_log("Mirrors are un-folded.");
      } else {
        mirrors_folded = true;
        serial_log("Mirrors are folded.");
      }
      frm_status_requested = false;

      if (lock_button_pressed) {
        if (!mirrors_folded) {
          if (!left_door_open && !right_door_open) {
            serial_log("Folding mirrors after lock button pressed.");
            toggle_mirror_fold();
          }
        }
        lock_button_pressed = false;
      } else if (unlock_button_pressed) {
        if (mirrors_folded) {
          serial_log("Will un-fold mirrors when door is opened after unlock button pressed.");
          #if UNFOLD_WITH_DOOR
            unfold_with_door_open = true;
          #else
            toggle_mirror_fold();
          #endif
          unlock_button_pressed = false;
        }
      }
    } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10) {
      kcan_write_msg(frm_status_request_b_buf);
    } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x21) {
      // Ignore {F1 21 80 3 22 0 6C 41}.
    } else {                                                                                                                        // Try again.
      serial_log("Did not receive the mirror status. Re-trying.");
      m = {frm_status_request_a_buf, millis() + 100};
      mirror_fold_txq.push(&m);
    }
  }
}


void toggle_mirror_fold(void) {
  uint16_t delay = 200;
  if (millis() < 2000) {
    delay = 500;
  }
  time_now = millis();
  m = {frm_toggle_fold_mirror_a_buf, time_now + delay};
  mirror_fold_txq.push(&m);
  m = {frm_toggle_fold_mirror_b_buf, time_now + delay + 10};
  mirror_fold_txq.push(&m);
  mirrors_folded = !mirrors_folded;
}


void check_mirror_fold_queue(void) {
  if (!mirror_fold_txq.isEmpty()) {
    if (diag_transmit) {
      mirror_fold_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        mirror_fold_txq.drop();
      }
    } else {
      mirror_fold_txq.flush();
    }
  }
}
#endif


#if FRONT_FOG_CORNER
void check_fog_corner_queue(void) {
  if (!fog_corner_left_txq.isEmpty()) {
    if (diag_transmit) {
      fog_corner_left_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        fog_corner_left_txq.drop();
        last_fog_action_timer = millis();
      }
    } else {
      fog_corner_left_txq.flush();
    }
  }
  if (!fog_corner_right_txq.isEmpty()) {
    if (diag_transmit) {
      fog_corner_right_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        kcan_write_msg(delayed_tx.tx_msg);
        fog_corner_right_txq.drop();
        last_fog_action_timer = millis();
      }
    } else {
      fog_corner_right_txq.flush();
    }
  }
}


void evaluate_dipped_beam_status(void) {
  if ((k_msg.buf[0] & 1) == 1) {                                                                                                    // Check the 8th bit of this byte for dipped beam status.
   if (!dipped_beam_status) {
      dipped_beam_status = true;
      serial_log("Dipped beam ON.");
    }
  } else {
    if (dipped_beam_status) {
      dipped_beam_status = false;
      serial_log("Dipped beam OFF");

      if (left_fog_on || right_fog_on) {
        m = {front_fogs_all_off_buf, millis() + 100};
        fog_corner_left_txq.push(&m);
        left_fog_on = right_fog_on = false;
      }
    }
  }
}


void evaluate_steering_angle_fog(void) {
  if (!front_fog_status && dipped_beam_status && rls_headlights_requested && diag_transmit) {                                       // Cannot tell if Auto-lights are on via CAN. This is the closest without using KWP jobs.
    steering_angle = ((pt_msg.buf[1] * 256) + pt_msg.buf[0]) / 23;

    // Max left angle is 1005 / 23
    if (steering_angle >= 435) { 
      steering_angle = steering_angle - 2849;
    }

    int8_t ANGLE, HYSTERESIS;
    if (indicators_on) {
      ANGLE = FOG_CORNER_STEERTING_ANGLE_INDICATORS;
      HYSTERESIS = STEERTING_ANGLE_HYSTERESIS_INDICATORS;
    } else {
      ANGLE = FOG_CORNER_STEERTING_ANGLE;
      HYSTERESIS = STEERTING_ANGLE_HYSTERESIS;
    }
    
    int16_t FOG_MAX_SPEED = 35, FOG_MAX_SPEED_HYSTERESIS;
    if (left_fog_on || right_fog_on) {
      if (speed_mph) {
        FOG_MAX_SPEED = 22;
        FOG_MAX_SPEED_HYSTERESIS = 5;
      } else {
        FOG_MAX_SPEED_HYSTERESIS = 3;
      }
    } else {
      if (speed_mph) {
        FOG_MAX_SPEED = 22;
      }
      FOG_MAX_SPEED_HYSTERESIS = 0;
    }


    if (indicated_speed >= (FOG_MAX_SPEED + FOG_MAX_SPEED_HYSTERESIS)) {
      if (left_fog_on) {
        left_fog_soft(false);
        left_fog_on = false;
        serial_log("Max speed exceeded. Turned left fog light OFF.");
      }
      if (right_fog_on) {
        right_fog_soft(false);
        right_fog_on = false;
        serial_log("Max speed exceeded. Turned right fog light OFF.");
      }
    } else {
      if (steering_angle > ANGLE) {
        if (!reverse_gear_status) {
          if (!left_fog_on) {
            left_fog_soft(true);
            left_fog_on = true;
            serial_log("Steering angle below setpoint. Turned left fog light ON.");
          }
        } else {
            if (!right_fog_on) {
            right_fog_soft(true);
            right_fog_on = true;
            serial_log("Reverse: Steering angle above setpoint. Turned right fog light ON.");
          }
        }
      } else if (steering_angle < (ANGLE - HYSTERESIS)) {
        if (!reverse_gear_status) {
          if (left_fog_on) {
            left_fog_soft(false);
            left_fog_on = false;
            serial_log("Steering angle returned. Turned left fog light OFF.");
          }
        } else {
          if (right_fog_on) {
            right_fog_soft(false);
            right_fog_on = false;
            serial_log("Reverse: Steering angle returned. Turned right fog light OFF.");
          }
        }
      }

      if (steering_angle < -ANGLE) {
        if (!reverse_gear_status) {
          if (!right_fog_on) {
            right_fog_soft(true);
            right_fog_on = true;
            serial_log("Steering angle above setpoint. Turned right fog light ON.");
          }
        } else {
          if (!left_fog_on) {
            left_fog_soft(true);
            left_fog_on = true;
            serial_log("Reverse: Steering angle below setpoint. Turned left fog light ON.");
          }
        }
      } else if (steering_angle > (-ANGLE + HYSTERESIS)) {
        if (!reverse_gear_status) {
          if (right_fog_on) {
            right_fog_soft(false);
            right_fog_on = false;
            serial_log("Steering angle returned. Turned right fog light OFF.");
          }
        } else {
          if (left_fog_on) {
            left_fog_soft(false);
            left_fog_on = false;
            serial_log("Reverse: Steering angle returned. Turned left fog light OFF.");
          }
        }
      }
    }
  }
}


void left_fog_soft(bool on) {
  fog_corner_left_txq.flush();
  time_now = millis();
  if (on) {
    m = {front_left_fog_on_a_buf, time_now};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_b_buf, time_now + 100};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_c_buf, time_now + 200};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_d_buf, time_now + 300};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_d_buf, time_now + 15000};                                                                                // Extend the maximum ON duration.
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_d_buf, time_now + 25000};
    fog_corner_left_txq.push(&m);
  } else {
    m = {front_left_fog_on_g_softer_buf, time_now};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_f_softer_buf, time_now + 100};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_e_softer_buf, time_now + 200};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_d_softer_buf, time_now + 300};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_c_softer_buf, time_now + 400};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_b_softer_buf, time_now + 500};
    fog_corner_left_txq.push(&m);
    m = {front_left_fog_on_a_softer_buf, time_now + 600};
    fog_corner_left_txq.push(&m);
    m = {front_fogs_all_off_buf, time_now + 700};
    fog_corner_left_txq.push(&m);
  }
}


void right_fog_soft(bool on) {
  fog_corner_right_txq.flush();
  time_now = millis();
  if (on) {
    m = {front_right_fog_on_a_buf, time_now};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_b_buf, time_now + 100};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_c_buf, time_now + 200};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_d_buf, time_now + 300};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_d_buf, time_now + 15000};                                                                               // Extend the maximum ON duration.
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_d_buf, time_now + 25000};
    fog_corner_right_txq.push(&m);
  } else {
    m = {front_right_fog_on_g_softer_buf, time_now};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_f_softer_buf, time_now + 100};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_e_softer_buf, time_now + 200};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_d_softer_buf, time_now + 300};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_c_softer_buf, time_now + 400};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_b_softer_buf, time_now + 500};
    fog_corner_right_txq.push(&m);
    m = {front_right_fog_on_a_softer_buf, time_now + 600};
    fog_corner_right_txq.push(&m);
    m = {front_fogs_all_off_buf, time_now + 700};
    fog_corner_right_txq.push(&m);
  }
}
#endif


#if WIPE_AFTER_WASH
void evaluate_wiping_request(void) {
  if (terminal_r) {
    if (pt_msg.buf[0] == 0x10) {
      if (wash_message_counter >= 2 || wipe_scheduled) {
        serial_log("Washing cycle started.");
        wiper_txq.flush();
        m = {wipe_single_buf, millis() + 7000};
        wiper_txq.push(&m);
        wipe_scheduled = true;                                                                                                      // If a quick pull is detected after the main one, re-schedule the wipe.
      } else {
        wash_message_counter++;
      }
    } else {
      if (pt_msg.buf[0] == 1 || pt_msg.buf[0] == 2 || pt_msg.buf[0] == 3 || pt_msg.buf[0] == 8) {                                   // Abort if wipers are used in the meantime.
        if (!wiper_txq.isEmpty()) {
          serial_log("Wipe after wash aborted.");
          wiper_txq.flush();
          wipe_scheduled = false;
        }
      }
      wash_message_counter = 0;
    }
  }
}


void check_wiper_queue(void) {
  if (terminal_r) {
    if (!wiper_txq.isEmpty()) {
      wiper_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        serial_log("Wiping windscreen after washing.");
        ptcan_write_msg(delayed_tx.tx_msg);
        wiper_txq.drop();
        wipe_scheduled = false;
      }
    }
  }
}
#endif


void evaluate_ambient_temperature(void) {
  ambient_temperature_real = (k_msg.buf[0] - 80) / 2.0;
}


#if FRONT_FOG_CORNER || F_NIVI
void evaluate_rls_light_status(void) {
  if (k_msg.buf[1] == 2) {                                                                                                          // 1 = Twilight mode, 2 = Darkness.
    if (!rls_headlights_requested) {
      rls_headlights_requested = true;
      serial_log("RLS ambient light low enough for auto-headlights.");
    }
  } else {
    if (rls_headlights_requested) {
      rls_headlights_requested = false;
      serial_log("RLS ambient light bright enough to disable auto-headlights.");
    }
  }
  #if F_NIVI
    rls_time_of_day = k_msg.buf[1];
    rls_brightness = k_msg.buf[0];
  #endif
}
#endif


#if F_NIVI
void request_vehicle_tilt_angle(void) {
  if (sine_angle_request_timer >= 500) {
    if (diag_transmit) {
      kcan_write_msg(sine_angle_request_a_buf);
      sine_angle_requested = true;
    }
    sine_angle_request_timer = 0;
  }
}


void evaluate_vehicle_tilt_angle(void) {                                                                                            // Check with sine_65.prg JOB status_inclination_y_axis.
  if (sine_angle_requested) {
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10) {
      sine_tilt_angle = ((int16_t)(k_msg.buf[5] << 8 | k_msg.buf[6])) * 0.001;
      sine_tilt_angle = constrain(sine_tilt_angle, -64.0, 64.0);                                                                    // Boundary check since F message has resolution -64..64.
      f_vehicle_angle = round((sine_tilt_angle + 64) / 0.05);
      sine_angle_requested = false;
      if (diag_transmit) {
        kcan_write_msg(sine_angle_request_b_buf);                                                                                   // Send this to complete the transaction.
      }
    }
  }
}


void send_f_brightness_status(void) {
  if (f_outside_brightness_timer >= 500) {
    f_outside_brightness[0] = f_outside_brightness[1] = rls_brightness;
    f_outside_brightness_buf = make_msg_buf(0x2A5, 2, f_outside_brightness);
    ptcan_write_msg(f_outside_brightness_buf);
    f_outside_brightness_timer = 0;
  }
}
#endif


#if INDICATE_TRUNK_OPENED
void evaluate_door_lock_ckm(void) {
  if (bitRead(k_msg.buf[0], 0) == 0 && bitRead(k_msg.buf[0], 1) == 1) {
    if (!visual_signal_ckm) {
      serial_log("Visual signal CKM enabled.");
      visual_signal_ckm = true;
    }
  } else {
    if (visual_signal_ckm) {
      serial_log("Visual signal CKM disabed.");
      visual_signal_ckm = false;
    }
  }
}
#endif
