// General body functions dealing with exterior electronics go here.


#if DIM_DRL
void evaluate_drl_status() {
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


void evaluate_indicator_status_dim() {
  if (drl_status && diag_transmit) {
    if(k_msg.buf[0] == 0x80 || k_msg.buf[0] == 0xB1) {                                                                              // Off or Hazards
      if (right_dimmed) {
        kcan_write_msg(right_drl_bright_buf);
        right_dimmed = false;
        serial_log("Restored right DRL brightness.");
      } else if (left_dimmed) {
        kcan_write_msg(left_drl_bright_buf);
        left_dimmed = false;
        serial_log("Restored left DRL brightness.");
      }
    } else if (k_msg.buf[0] == 0x91) {                                                                                              // Left indicator
      if (right_dimmed) {
        kcan_write_msg(right_drl_bright_buf);
        right_dimmed = false;
        serial_log("Restored right DRL brightness.");
      }
      kcan_write_msg(left_drl_dim_buf);
      #if DEBUG_MODE
        if (!left_dimmed) {
          serial_log("Dimmed left DRL.");
        }
      #endif
      left_dimmed = true;
    } else if (k_msg.buf[0] == 0xA1) {                                                                                              // Right indicator
      if (left_dimmed) {
        kcan_write_msg(left_drl_bright_buf);
        left_dimmed = false;
        serial_log("Restored left DRL brightness.");
      }
      kcan_write_msg(right_drl_dim_buf);
      #if DEBUG_MODE
        if (!right_dimmed) {
          serial_log("Dimmed right DRL.");
        }
      #endif
      right_dimmed = true;
    }
  }
}
#endif


#if DOOR_VOLUME || AUTO_MIRROR_FOLD
void evaluate_door_status() {
  if (k_msg.id == 0xE2) {
    if (k_msg.buf[3] == 0xFD) {
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
          send_volume_request();
        #endif
      }
    } else if (k_msg.buf[3] == 0xFC) {
      if (left_door_open) {
        left_door_open = false;
        #if RHD
          serial_log("Passenger's door closed.");
        #else
          serial_log("Driver's door closed.");
        #endif
        #if DOOR_VOLUME
          send_volume_request();
        #endif
      }
    }
  } else if (k_msg.id == 0xEA) {
    if (k_msg.buf[3] == 0xFD) {
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
          send_volume_request();
        #endif
      }
    } else if (k_msg.buf[3] == 0xFC) {
      if (right_door_open) {
        right_door_open = false;
        #if RHD
          serial_log("Driver's door closed.");
        #else
          serial_log("Passenger's door closed.");
        #endif
        #if DOOR_VOLUME
          send_volume_request();
        #endif
      }
    }
  }
}
#endif


#if EXHAUST_FLAP_CONTROL
void control_exhaust_flap_user() {
  if (engine_running) {
    if (exhaust_flap_sport) {                                                                                                       // Flap always open in sport mode.
      if ((millis() - exhaust_flap_action_timer) >= 500) {
        if (!exhaust_flap_open) {
          actuate_exhaust_solenoid(LOW);
          serial_log("Opened exhaust flap with MDrive.");
        }
      }
    }
  } else {
    #if QUIET_START
      if (ignition || terminal_r || engine_cranking) {
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


void control_exhaust_flap_rpm() {
  if (engine_running) {
    if (!exhaust_flap_sport) {
      if ((millis() - exhaust_flap_action_timer) >= exhaust_flap_action_interval) {                                                 // Avoid vacuum drain, oscillation and apply startup delay.
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
  exhaust_flap_action_timer = millis();
  exhaust_flap_open = !activate;                                                                                                    // Flap position is the inverse of solenoid state. When active, the flap is closed.
}
#endif


#if CKM || EDC_CKM_FIX
void evaluate_key_number() {
  if (k_msg.buf[0] / 11 != cas_key_number) {
    cas_key_number = k_msg.buf[0] / 11;                                                                                             // Key 1 = 0, Key 2 = 11, Key 3 = 22...
    if (cas_key_number > 2) {
      serial_log("Received wrong key personalisation number. Assuming default");
      cas_key_number = 3;                                                                                                           // Default.
    }
    #if DEBUG_MODE
    else {
      sprintf(serial_debug_string, "Received key number: %d.", cas_key_number + 1);
      serial_log(serial_debug_string);
    }
    #endif
    #if CKM
      console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;
    #endif
  }
}
#endif


#if AUTO_MIRROR_FOLD
void evaluate_remote_button() {
  if (!engine_running) {                                                                                                            // Ignore if locking car while running.
    if (k_msg.buf[2] != last_lock_status_can) {                                                                                     // Lock/Unlock messages are sent many times. Should only react to the first.
      if (k_msg.buf[1] == 0x30 || k_msg.buf[1] == 0x33) {
        if (k_msg.buf[2] == 4) {
          lock_button_pressed = true;
          serial_log("Remote lock button pressed. Checking mirror status.");
          delayed_can_tx_msg m = {frm_status_request_a_buf, millis() + 100};
          mirror_fold_txq.push(&m);
          frm_status_requested = true;
          #if UNFOLD_WITH_DOOR
            unfold_with_door_open = false;
          #endif
        } else if (k_msg.buf[2] == 1) {
          unlock_button_pressed = true;
          serial_log("Remote unlock button pressed. Checking mirror status.");
          delayed_can_tx_msg m = {frm_status_request_a_buf, millis() + 100};
          mirror_fold_txq.push(&m);
          frm_status_requested = true;
        } else if (k_msg.buf[2] == 0) {
          serial_log("Remote buttons released.");
        }
      }
      last_lock_status_can = k_msg.buf[2];
    }
  }
}


void evaluate_mirror_fold_status() {
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
      delayed_can_tx_msg m = {frm_status_request_a_buf, millis() + 200};
      mirror_fold_txq.push(&m);
    }
  }
}


void toggle_mirror_fold() {
  uint16_t delay = 200;
  if (millis() < 2000) {
    delay = 500;
  }
  delayed_can_tx_msg m;
  unsigned long timeNow = millis();
  m = {frm_toggle_fold_mirror_a_buf, timeNow + delay};
  mirror_fold_txq.push(&m);
  m = {frm_toggle_fold_mirror_b_buf, timeNow + delay + 10};
  mirror_fold_txq.push(&m);
  mirrors_folded = !mirrors_folded;
}


void check_mirror_fold_queue() {
  if (!mirror_fold_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    mirror_fold_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      mirror_fold_txq.drop();
    }
  }
}
#endif


#if FRONT_FOG_CORNER
void check_fog_corner_queue() {
  if (!fog_corner_txq.isEmpty()) {
    delayed_can_tx_msg delayed_tx;
    fog_corner_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      fog_corner_txq.drop();
    }
  }
}


void evaluate_dipped_beam_status() {
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
        delayed_can_tx_msg m = {front_fogs_all_off_buf, millis() + 100};
        fog_corner_txq.push(&m);
        left_fog_on = right_fog_on = false;
      }
    }
  }
}


void evaluate_steering_angle_fog() {
  if (!front_fog_status && !reverse_status && dipped_beam_status) {
    steering_angle = ((pt_msg.buf[1] * 256) + pt_msg.buf[0]) / 23;

    // Max left angle is 1005 / 23
    if (steering_angle >= 435) { 
      steering_angle = steering_angle - 2849;
    }

    if (steering_angle > FOG_CORNER_STEERTING_ANGLE) {
      if (!left_fog_on) {
        unsigned long timenow = millis();
        delayed_can_tx_msg m = {front_left_fog_on_a_buf, timenow};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_b_buf, timenow + 100};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_c_buf, timenow + 200};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_d_buf, timenow + 300};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_e_buf, timenow + 400};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_f_buf, timenow + 500};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_g_buf, timenow + 600};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_h_buf, timenow + 700};
        fog_corner_txq.push(&m);
        if (!left_fog_on) {
          left_fog_on = true;
          serial_log("Steering angle below setpoint. Turned left fog light ON.");
        }
      }
    } else if (steering_angle < (FOG_CORNER_STEERTING_ANGLE - STEERTING_ANGLE_HYSTERESIS)) {
      if (left_fog_on) {
        unsigned long timenow = millis();
        delayed_can_tx_msg m = {front_left_fog_on_g_buf, timenow};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_f_buf, timenow + 100};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_e_buf, timenow + 200};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_d_buf, timenow + 300};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_c_buf, timenow + 400};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_b_buf, timenow + 500};
        fog_corner_txq.push(&m);
        m = {front_left_fog_on_a_buf, timenow + 600};
        fog_corner_txq.push(&m);
        m = {front_fogs_all_off_buf, millis() + 100};
        fog_corner_txq.push(&m);
        if (left_fog_on) {
          left_fog_on = false;
          serial_log("Steering angle returned. Turned left fog light OFF.");
        }
      }
    }

    if (steering_angle < -FOG_CORNER_STEERTING_ANGLE) {
      if (!right_fog_on) {
        unsigned long timenow = millis();
        delayed_can_tx_msg m = {front_right_fog_on_a_buf, timenow};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_b_buf, timenow + 100};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_c_buf, timenow + 200};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_d_buf, timenow + 300};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_e_buf, timenow + 400};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_f_buf, timenow + 500};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_g_buf, timenow + 600};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_h_buf, timenow + 700};
        fog_corner_txq.push(&m);
        if (!right_fog_on) {
          right_fog_on = true;
          serial_log("Steering angle above setpoint. Turned right fog light ON.");
        }
      }
    } else if (steering_angle > (-FOG_CORNER_STEERTING_ANGLE + STEERTING_ANGLE_HYSTERESIS)) {
      if (right_fog_on) {
        unsigned long timenow = millis();
        delayed_can_tx_msg m = {front_right_fog_on_g_buf, timenow};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_f_buf, timenow + 100};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_e_buf, timenow + 200};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_d_buf, timenow + 300};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_c_buf, timenow + 400};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_b_buf, timenow + 500};
        fog_corner_txq.push(&m);
        m = {front_right_fog_on_a_buf, timenow + 600};
        fog_corner_txq.push(&m);
        m = {front_fogs_all_off_buf, timenow + 700};
        fog_corner_txq.push(&m);
        if (right_fog_on) {
          right_fog_on = false;
          serial_log("Steering angle returned. Turned right fog light OFF.");
        }
      }
    }
  }
}
#endif