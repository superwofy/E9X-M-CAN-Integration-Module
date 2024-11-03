// General body functions dealing with exterior electronics go here.


void evaluate_drl_status(void) {
  if (k_msg.buf[1] == 0x32) {
    if (!drl_status) {
      drl_status = true;
      serial_log("DRLs ON.", 2);
    }
  } else {
    if (drl_status) {
      drl_status = false;
      serial_log("DRLs OFF", 2);
    }
  }
}


void evaluate_lights_ckm(void) {
  #if DIM_DRL
    if (k_msg.buf[2] == 0xFE) {
      if (!drl_ckm[cas_key_number]) {
        serial_log("DRL CKM ON.", 2);
        drl_ckm[cas_key_number] = true;
      }
    } else {
      if (drl_ckm[cas_key_number]) {
        serial_log("DRL CKM OFF.", 2);
        drl_ckm[cas_key_number] = false;
      }
    }
  #endif

  #if F_NBTE
    if (f_lights_ckm_request != 0) {
      k_msg.buf[2] = f_lights_ckm_request;
      f_lights_ckm_request = 0;
    } else if (idrive_run_timer <= 15000) {
      if (drl_ckm[cas_key_number]) {
        k_msg.buf[2] = 0xF6;                                                                                                          // DRL ON, home lights setting can be changed.
      } else {
        k_msg.buf[2] = 0xF4;
      }
      f_lights_ckm_delayed_msg = k_msg;                                                                                               // Save this message for after iDrive boots.
    }
    kcan2_write_msg(k_msg);
  #endif
}


void evaluate_hba_ckm(void) {
  uint8_t new_hba_status = k_msg.buf[0] >> 4;
  if (new_hba_status != hba_status) {
    if (new_hba_status == 0xE) {
      serial_log("HBA CKM ON.", 2);
    } else {
      serial_log("HBA CKM OFF.", 2);
    }
    hba_status = new_hba_status;
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


void evaluate_indicator_status_dim(void) {
  if (ignition) {
    if(k_msg.buf[0] == 0x80 || k_msg.buf[0] == 0xB1) {                                                                              // Off or Hazards
      #if DIM_DRL
        if (drl_ckm[cas_key_number] && drl_status && diag_transmit) {
          unsigned long time_now = millis();
          if (right_dimmed) {
            m = {right_drl_bright_buf, time_now + 100};
            dim_drl_txq.push(&m);
            right_dimmed = false;
            serial_log("Restored right DRL brightness.", 2);
          } else if (left_dimmed) {
            m = {left_drl_bright_buf, time_now + 100};
            dim_drl_txq.push(&m);
            left_dimmed = false;
            serial_log("Restored left DRL brightness.", 2);
          }
        }
      #endif
      if (k_msg.buf[0] == 0xB1) {
        hazards_on = true;
      } else {
        hazards_on = false;
      }
      indicators_on = false;
    } else if (k_msg.buf[0] == 0x91) {                                                                                              // Left indicator
      #if DIM_DRL
        if (drl_ckm[cas_key_number] && drl_status && diag_transmit) {
          unsigned long time_now = millis();
          if (right_dimmed) {
            m = {right_drl_bright_buf, time_now + 100};
            dim_drl_txq.push(&m);
            right_dimmed = false;
            serial_log("Restored right DRL brightness.", 2);
          }
          m = {left_drl_dim_buf, time_now + 100};
          dim_drl_txq.push(&m);
          if (!left_dimmed) {
            serial_log("Dimmed left DRL.", 2);
          }
          left_dimmed = true;
        }
      #endif
      indicators_on = true;
      #if MIRROR_UNDIM
        undim_mirrors_with_indicators();
      #endif
    } else if (k_msg.buf[0] == 0xA1) {                                                                                              // Right indicator
      #if DIM_DRL
        if (drl_ckm[cas_key_number] && drl_status && diag_transmit) {
          unsigned long time_now = millis();
          if (left_dimmed) {
            m = {left_drl_bright_buf, time_now + 100};
            dim_drl_txq.push(&m);
            left_dimmed = false;
            serial_log("Restored left DRL brightness.", 2);
          }
          m = {right_drl_dim_buf, time_now + 100};
          dim_drl_txq.push(&m);
          if (!right_dimmed) {
            serial_log("Dimmed right DRL.", 2);
          }
          right_dimmed = true;
        }
      #endif
      indicators_on = true;
      #if MIRROR_UNDIM
        undim_mirrors_with_indicators();
      #endif
    }
  }
}


void undim_mirrors_with_indicators(void) {
  // Makes it easier to see cyclists at night in the city when turning...
  if (szl_full_indicator) {
    if (rls_time_of_day == 2 && frm_undim_timer >= 10000) {
      if (engine_running == 2 && indicated_speed <= 30.0 && fzd_ec_dimming >= 0x32) {
        kcan_write_msg(frm_mirror_undim_buf);
        frm_undim_timer = 0;
        serial_log("Undimmed exterior mirrors with indicator.", 2);
      }
    }
  }
}


void evaluate_electrochromic_dimming(void) {
  fzd_ec_dimming = k_msg.buf[0];                                                                                                    // Maxes out at around 0x64
}


void indicate_trunk_opened(uint16_t boot_delay) {
  if (visual_signal_ckm[cas_key_number] && engine_running == 0 && !hazards_on) {
    serial_log("Trunk remote button pressed. Flashing hazards 2x.", 2);
    unsigned long time_now = millis();
    m = {flash_hazards_double_buf, boot_delay + time_now + 100};
    hazards_flash_txq.push(&m);
  }
}


void evaluate_door_status(void) {
  if (k_msg.buf[1] != last_door_status) {
    if (k_msg.buf[1] == 1) {
      if (!right_door_open) {
        right_door_open = true;
        serial_log("Front right door open.", 2);
        #if RHD
          #if AUTO_MIRROR_FOLD
            check_unfold_mirrors_door_open();
          #endif
          #if COMFORT_EXIT
            evaluate_comfort_exit();
          #endif
        #endif
      }
      if (left_door_open) {
        left_door_open = false;
        serial_log("Front left door closed.", 2);
      }
    } else if (k_msg.buf[1] == 4) {
      if (!left_door_open) {
        left_door_open = true;
        serial_log("Front left door open.", 2);
        #if !RHD
          #if AUTO_MIRROR_FOLD
            check_unfold_mirrors_door_open();
          #endif
          #if COMFORT_EXIT
            evaluate_comfort_exit();
          #endif
        #endif
      }
      if (right_door_open) {
        right_door_open = false;
        serial_log("Front right door closed.", 2);
      }
    } else if (k_msg.buf[1] == 5) {
      left_door_open = right_door_open = true;
      serial_log("Both front doors open.", 2);
      #if AUTO_MIRROR_FOLD
        check_unfold_mirrors_door_open();
      #endif
      #if COMFORT_EXIT
        evaluate_comfort_exit();
      #endif
    } else if (k_msg.buf[1] == 0) {
      if (left_door_open) {
        left_door_open = false;
        serial_log("Front left door closed.", 2);
      }
      if (right_door_open) {
        right_door_open = false;
        serial_log("Front right door closed.", 2);
      }
    }
    last_door_status = k_msg.buf[1];
    #if DOOR_VOLUME
      send_volume_request_door();
    #endif
  }

  uint8_t hood_open_status = bitRead(k_msg.buf[2], 2);
  if (hood_status_debounce >= 500) {
    if (hood_open_status != last_hood_status) {
      if (hood_open_status) {
        serial_log("Hood opened.", 2);
        #if HOOD_OPEN_GONG
          if (!vehicle_moving && terminal_r) {
            #if F_NBTE
              if ((engine_coolant_temperature - 48) >= 80) {
                play_cc_gong(1);
                kcan2_write_msg(hood_open_hot_cc_dialog_buf);
              } else {
                play_cc_gong(2);
              }
            #else
              play_cc_gong(2);
            #endif
          }
        #endif
      } else {
        serial_log("Hood closed.", 2);
        #if F_NBTE
          kcan2_write_msg(hood_open_hot_cc_dialog_clear_buf);
        #endif
      }
      last_hood_status = hood_open_status;
    }
    hood_status_debounce = 0;
  }

  uint8_t trunk_open_status = bitRead(k_msg.buf[2], 0);
  if (trunk_open_status != last_trunk_status) {
    if (trunk_open_status) {
      serial_log("Trunk opened.", 2);
      
    } else {
      serial_log("Trunk closed.", 2);
    }
    last_trunk_status = trunk_open_status;
  }
}


void evaluate_key_number_remote(void) {
  uint8_t cas_key_number_can = k_msg.buf[0] / 0x11;
  if (cas_key_number_can != cas_key_number) {
    if (cas_key_number_can > 2) {
      if (cas_key_number != 3) {
        serial_log("Received invalid / Guest key personalisation number.", 1);
        cas_key_number = 3;
        key_guest_profile = true;
      }
    } else {
      cas_key_number = cas_key_number_can;                                                                                          // Key 1 = 0, Key 2 = 0x11, Key 3 = 0x22..., Guest = 0xAA.
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Received remote key profile number: %d.", cas_key_number + 1);
        serial_log(serial_debug_string, 2);
      #endif
    }
    
    key_guest_profile = false;

    #if !F_NBTE
      console_power_mode = dme_ckm[cas_key_number][0] == 0xF1 ? false : true;
      if (vehicle_awakened_timer >= 10000) {                                                                                        // Make sure the car was awake first...
        send_dme_power_ckm();                                                                                                       // Update iDrive in case key remote changed
      }
    #endif
    update_mdrive_can_message();
    send_mdrive_message();
  }
}


void evaluate_remote_button(void) {
  if (k_msg.buf[2] != last_lock_status_can) {                                                                                       // Lock/Unlock messages are sent many times. Should only react to the first.
    if (k_msg.buf[2] == 4) {
      if (!ignition) {                                                                                                              // Ignore if locking car with ignition ON.
        if (!left_door_open && !right_door_open) {
          if (!doors_locked || !doors_alarmed) {                                                                                    // This code should only run when the car is locked. Alarm armed/unarmed.
            serial_log("Remote lock button pressed.", 2);
            fold_lock_button_pressed = true;
            #if AUTO_MIRROR_FOLD
              if (diag_transmit) {
                serial_log("Checking exterior mirror status.", 2);
                kcan_write_msg(frm_mirror_status_request_a_buf);
                frm_mirror_status_requested = true;
              }
            #endif
            #if IMMOBILIZER_SEQ
              unsigned long time_now = millis();
              m = {alarm_led_return_control_buf, time_now + 100};                                                                   // Release control of the LED so that alarm can control it.
              alarm_led_txq.push(&m);
              m = {alarm_led_return_control_buf, time_now + 500};
              alarm_led_txq.push(&m);
              alarm_led_disable_on_lock = true;
              alarm_led_message_timer = 100000;                                                                                     // Reset in case LED will need to be enabled soon.
              serial_log("Deactivated DWA LED when doors locked.", 2);
            #endif
            #if COMFORT_EXIT
              comfort_exit_ready = false;
            #endif
            #if F_NBTE
              hu_bn2000_bus_sleep_ready_timer = HU_ENT_MODE_TIMEOUT;                                                                // Car locked, allow KCAN network to sleep.
            #endif
          }
          
          if (doors_alarmed) {                                                                                                      // Car fully locked.
            // For remote range see: https://www.spoolstreet.com/threads/e92-key-remote-range-one-solution.9072/
            if (vehicle_awakened_timer >= 10000) {                                                                                  // Car was already awake.
              if (doors_locked_timer >= 10000) {                                                                                    // Must be at least 10s to avoid interference with DWA lock button disable sensors.
                indicate_car_locked(0);
              }
            } else {                                                                                                                // Car woke up just now with the remote.
              indicate_car_locked(100);
            }
          }
        }

        lock_button_pressed = true;
        lock_button_pressed_counter = 1;
      } else {
        #if F_NBTE
          kcan2_write_msg(cc_double_gong_buf);                                                                                      // Audible reminder that ignition is still ON.
        #else
          kcan_write_msg(cc_double_gong_buf);
        #endif
      }
    }
    
    else if (k_msg.buf[2] == 1 && !ignition) {                                                                                      // Ignore if unlocking car with ignition ON/running.
      car_locked_indicator_counter = 0;
      if (doors_locked || doors_alarmed) {                                                                                          // Only run once when unlocking the car.
        serial_log("Remote unlock button pressed.", 2);
        #if IMMOBILIZER_SEQ
          alarm_led_disable_on_lock = false;
          alarm_led_message_timer = 100000;
        #endif
      }

      lock_button_pressed = false;
      hazards_flash_txq.flush();
    }

    else if (k_msg.buf[2] == 0x10) {
      lock_button_pressed = false;
      serial_log("Remote trunk button pressed.", 2);
      if (vehicle_awakened_timer >= 10000) {
        indicate_trunk_opened(0);
      } else {                                                                                                                      // Car just woke up from deep sleep.
        indicate_trunk_opened(100);
      }
    }

    else if (k_msg.buf[2] == 0) {
      serial_log("Remote buttons released.", 2);
      lock_button_pressed = false;
      lock_button_pressed_counter = 0;
    }
    last_lock_status_can = k_msg.buf[2];
  } else {                                                                                                                          // A button is being held, or they're all released.
    if (lock_button_pressed) {
      if (!terminal_r) {
        if (lock_button_pressed_counter < 199) {                                                                                    // While the lock message is being received increment counter. Around 20s.
          lock_button_pressed_counter++;
        } else {
          if (diag_transmit) {
            kcan_write_msg(flash_hazards_single_buf);
            lock_button_pressed_counter = 0;
            serial_log("Sending power_down command after holding button for 20s.", 2);
            power_down_requested = true;
            kcan_write_msg(power_down_cmd_a_buf);
          }
        }
      } else {
        lock_button_pressed_counter = 0;
      }
    }
  }
}


void evaluate_mirror_fold_status(void) {
  if (frm_mirror_status_requested) {                                                                                                // Make sure the request came from this module.
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10) {
      kcan_write_msg(frm_mirror_status_request_b_buf);
    } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x21) {
      // Ignore {F1 21 80 3 22 0 6C 41}.
    } else if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x22) {
      if (k_msg.buf[4] == 0) {
        mirrors_folded = false;
        serial_log("Exterior mirrors are un-folded.", 2);
      } else {
        mirrors_folded = true;
        serial_log("Exterior mirrors are folded.", 2);
      }
      frm_mirror_status_requested = false;
      mirror_status_retry = 0;

      if (fold_lock_button_pressed) {
        if (!mirrors_folded) {
          serial_log("Folding exterior mirrors after lock button pressed.", 2);
          toggle_mirror_fold(true);
        }
        fold_lock_button_pressed = false;
      } else if (unfold_with_door_open) {
        if (mirrors_folded) {
          serial_log("Un-folding exterior mirrors after driver's door is opened following unlock.", 2);
          toggle_mirror_fold(false);
        }
      }
    } else {                                                                                                                        // Try again. This will only work if the FRM first sent an error code.
      if (mirror_status_retry < 3 && diag_transmit) {
        serial_log("Did not receive exterior mirror status. Re-trying.", 1);
        m = {frm_mirror_status_request_a_buf, millis() + 500};
        mirror_fold_txq.push(&m);
        mirror_status_retry++;
      } else {
        frm_mirror_status_requested = false;                                                                                        // Retries have failed.
        mirror_status_retry = 0;
        mirror_fold_txq.flush();
      }
    }
  }
}


void check_unfold_mirrors_door_open(void) {
  if (unfold_with_door_open) {
    if (diag_transmit) {
      serial_log("Checking mirror status.", 2);
      kcan_write_msg(frm_mirror_status_request_a_buf);
      frm_mirror_status_requested = true;
    } else {
      unfold_with_door_open = false;
    }
  }
}


void toggle_mirror_fold(bool new_eeprom_state) {
  unsigned long time_now = millis();
  m = {frm_toggle_fold_mirror_a_buf, time_now + 300};
  mirror_fold_txq.push(&m);
  m = {frm_toggle_fold_mirror_b_buf, time_now + 310};
  mirror_fold_txq.push(&m);
  unfold_with_door_open = new_eeprom_state;
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
      if (delayed_tx.tx_msg.buf[3] == 0x28) {
        frm_ahl_flc_status_requested = true;
      }
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
  if ((k_msg.buf[0] & 1) == 1) {                                                                                                    // Check the first bit (LSB 0) of this byte for dipped beam status.
   if (!dipped_beam_status) {
      dipped_beam_status = true;
      serial_log("Dipped beam ON.", 2);
      frm_ahl_flc_status_requested = true;
      kcan_write_msg(frm_ahl_flc_status_request_buf);
      unsigned long time_now = millis();
      m = {frm_ahl_flc_status_request_buf, time_now + 5000};                                                                        // Send a delayed message as this job can return 0 right after startup.
      fog_corner_right_txq.push(&m);
    }
  } else {
    if (dipped_beam_status) {
      dipped_beam_status = false;
      serial_log("Dipped beam OFF", 2);
      ahl_active = flc_active = false;
      if (left_fog_on || right_fog_on) {
        m = {front_fogs_all_off_buf, millis() + 100};
        fog_corner_left_txq.push(&m);
        left_fog_on = right_fog_on = false;
      }
    }
  }
}


void evaluate_corner_fog_activation(void) {
  if (front_fog_corner_timer >= 1500 && ignition) {
    if (!front_fog_status && dipped_beam_status && rls_time_of_day == 2 && ahl_active && diag_transmit) {     
      float ANGLE, HYSTERESIS;
      if (indicators_on) {
        ANGLE = FOG_CORNER_STEERTING_ANGLE_INDICATORS;
        HYSTERESIS = STEERTING_ANGLE_HYSTERESIS_INDICATORS;
      } else {
        ANGLE = FOG_CORNER_STEERTING_ANGLE;
        HYSTERESIS = STEERTING_ANGLE_HYSTERESIS;
      }
      
      float FOG_MAX_SPEED = 35.0, FOG_MAX_SPEED_HYSTERESIS;
      if (left_fog_on || right_fog_on) {
        if (speed_mph) {
          FOG_MAX_SPEED = 22.0;
          FOG_MAX_SPEED_HYSTERESIS = 3.0;
        } else {
          FOG_MAX_SPEED_HYSTERESIS = 5.0;
        }
      } else {
        if (speed_mph) {
          FOG_MAX_SPEED = 22.0;
        }
        FOG_MAX_SPEED_HYSTERESIS = 0;
      }

      if (indicated_speed > (FOG_MAX_SPEED + FOG_MAX_SPEED_HYSTERESIS)) {
        if (left_fog_on) {
          left_fog_soft(false);
          left_fog_on = false;
          serial_log("Max speed exceeded. Turned left fog corner light OFF.", 2);
        }
        if (right_fog_on) {
          right_fog_soft(false);
          right_fog_on = false;
          serial_log("Max speed exceeded. Turned right fog corner light OFF.", 2);
        }
      } else {
        if (steering_angle > ANGLE) {
          if (!reverse_gear_status) {
            if (!left_fog_on) {
              left_fog_soft(true);
              left_fog_on = true;
              serial_log("Steering angle below setpoint. Turned left fog corner light ON.", 2);
            }
          } else {
              if (!right_fog_on) {
              right_fog_soft(true);
              right_fog_on = true;
              serial_log("Reverse: Steering angle above setpoint. Turned right fog corner light ON.", 2);
            }
          }
        } else if (steering_angle < (ANGLE - HYSTERESIS)) {
          if (!reverse_gear_status) {
            if (left_fog_on) {
              left_fog_soft(false);
              left_fog_on = false;
              serial_log("Steering angle returned. Turned left fog corner light OFF.", 2);
            }
          } else {
            if (right_fog_on) {
              right_fog_soft(false);
              right_fog_on = false;
              serial_log("Reverse: Steering angle returned. Turned right fog corner light OFF.", 2);
            }
          }
        }

        if (steering_angle < -ANGLE) {
          if (!reverse_gear_status) {
            if (!right_fog_on) {
              right_fog_soft(true);
              right_fog_on = true;
              serial_log("Steering angle above setpoint. Turned right fog corner light ON.", 2);
            }
          } else {
            if (!left_fog_on) {
              left_fog_soft(true);
              left_fog_on = true;
              serial_log("Reverse: Steering angle below setpoint. Turned left fog corner light ON.", 2);
            }
          }
        } else if (steering_angle > (-ANGLE + HYSTERESIS)) {
          if (!reverse_gear_status) {
            if (right_fog_on) {
              right_fog_soft(false);
              right_fog_on = false;
              serial_log("Steering angle returned. Turned right fog corner light OFF.", 2);
            }
          } else {
            if (left_fog_on) {
              left_fog_soft(false);
              left_fog_on = false;
              serial_log("Reverse: Steering angle returned. Turned left fog corner light OFF.", 2);
            }
          }
        }
      }
    }
    front_fog_corner_timer = 0;
  }
}


void left_fog_soft(bool on) {
  fog_corner_left_txq.flush();
  unsigned long time_now = millis();
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
  unsigned long time_now = millis();
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


void evaluate_ahl_flc_status(void) {
  if (frm_ahl_flc_status_requested) {
    flc_active = bitRead(k_msg.buf[5], 0);
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 4 && k_msg.buf[3] == 0x28) {
      uint8_t ahl_active_ = bitRead(k_msg.buf[5], 3);
      if (ahl_active != ahl_active_) {
        ahl_active = ahl_active_;
        #if DEBUG_MODE
          sprintf(serial_debug_string, "Received status AHL: %s FLC: %s",
                  ahl_active ? "ON" : "OFF", flc_active ? "ON" : "OFF");
          serial_log(serial_debug_string, 2);
        #endif
      }
      frm_ahl_flc_status_requested = false;
    } else {

    }
  }
}


void evaluate_ambient_temperature(void) {
  ambient_temperature_real = (k_msg.buf[0] - 80) / 2.0;
}


void evaluate_rls_light_status(void) {
  rls_time_of_day = k_msg.buf[1] & 0xF;
  if (rls_time_of_day == 8) {                                                                                                       // Either invalid or sensor blinded?
    rls_time_of_day = 2;
  }
  rls_brightness = k_msg.buf[0];
}


void request_vehicle_pitch_roll_angle(void) {
  #if IMMOBILIZER_SEQ
  if (sine_pitch_angle_request_timer >= 500 && immobilizer_released) {
  #else
  if (sine_pitch_angle_request_timer >= 500) {
  #endif
    if (diag_transmit) {
      kcan_write_msg(sine_pitch_angle_request_a_buf);
      sine_pitch_angle_request_timer = 0;
      sine_pitch_angle_requested = true;
    }
  }

  #if IMMOBILIZER_SEQ
  if (sine_roll_angle_request_timer >= 800 && immobilizer_released) {
  #else
  if (sine_roll_angle_request_timer >= 800) {
  #endif
    if (diag_transmit) {
      kcan_write_msg(sine_roll_angle_request_a_buf);
      sine_roll_angle_request_timer = 0;
      sine_roll_angle_requested = true;
    }
  }
}


void evaluate_vehicle_pitch_roll_angles(void) {
  if (sine_pitch_angle_requested) {                                                                                                 // Check with sine_65.prg JOB status_inclination_y_axis.
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10 && k_msg.buf[4] == 5) {
      sine_pitch_angle = ((int16_t)(k_msg.buf[5] << 8 | k_msg.buf[6])) * 0.001;
      sine_pitch_angle = constrain(sine_pitch_angle, -64.0, 64.0);                                                                  // Boundary check since F message has resolution -64..64.

      #if X_VIEW
        xview_grade_percentage = 0x64 + round(100 * tan(0.0174532925 * abs(sine_pitch_angle)));                                     // Max is 0xFE - 154%.
        xview_pitch_angle = (0x500 + (int)round( -sine_pitch_angle ) * 0x14) << 4;
      #endif

      f_vehicle_pitch_angle = 0x2000;                                                                                               // Make leading 4 bits signal valid (QU_AVL_LOGR_RW).
      if (vehicle_awakened_timer <= 5000) {                                                                                         // Force the status to initialization after the car just woke.
        f_vehicle_pitch_angle = 0x8000;
      }
      f_vehicle_pitch_angle = 0x2000 + round((sine_pitch_angle + 64) / 0.05);
      if (diag_transmit) {
        kcan_write_msg(sine_pitch_angle_request_b_buf);                                                                             // Send this to complete the transaction.
      }
    }
    sine_pitch_angle_requested = false;
  }
  if (sine_roll_angle_requested) {                                                                                                  // Check with sine_65.prg JOB status_inclination_x_axis.
    if (k_msg.buf[0] == 0xF1 && k_msg.buf[1] == 0x10 && k_msg.buf[4] == 4) {
      sine_roll_angle = ((int16_t)(k_msg.buf[5] << 8 | k_msg.buf[6])) * 0.001;
      sine_roll_angle = constrain(sine_roll_angle, -64.0, 64.0);                                                                    // Boundary check since F message has resolution -64..64.

      f_vehicle_roll_angle = 0x2000;
      if (vehicle_awakened_timer <= 5000) {                                                                                         // Force the status to initialization after the car just woke.
        f_vehicle_roll_angle = 0x8000;
      }
      f_vehicle_roll_angle = 0x2000 + round((sine_roll_angle + 64) / 0.05);
      if (diag_transmit) {
        kcan_write_msg(sine_roll_angle_request_b_buf);                                                                              // Send this to complete the transaction.
      }
    }
    sine_roll_angle_requested = true;
  }
}


void send_f_brightness_status(void) {
  if (f_outside_brightness_timer >= 500) {
    uint8_t f_outside_brightness[] = {0xFE, 0xFE};                                                                                  // Daytime?. The two bytes may represent the two photosensors (driver's/passenger's side in FXX).
    f_outside_brightness[0] = f_outside_brightness[1] = rls_brightness;
    CAN_message_t f_outside_brightness_buf = make_msg_buf(0x2A5, 2, f_outside_brightness);
    #if F_NIVI
      if (ignition) {
        ptcan_write_msg(f_outside_brightness_buf);
      }
    #endif
    #if F_NBTE
      kcan2_write_msg(f_outside_brightness_buf);
    #endif
    f_outside_brightness_timer = 0;
  }
}


void evaluate_door_lock_ckm(void) {
  if (bitRead(k_msg.buf[0], 0) == 0 && bitRead(k_msg.buf[0], 1) == 1) {
    if (!visual_signal_ckm[cas_key_number]) {
      serial_log("Visual signal CKM ON.", 2);
      visual_signal_ckm[cas_key_number] = true;
    }
  } else {
    if (visual_signal_ckm[cas_key_number]) {
      serial_log("Visual signal CKM OFF.", 2);
      visual_signal_ckm[cas_key_number] = false;
    }
  }
}


void evaluate_drivers_door_lock_status(void) {                                                                                      // The car won't lock fully with driver's door open, this should be enough.
  if (k_msg.buf[3] == 0xFC) {                                                                                                       // Door closed.
    if (k_msg.buf[0] == 0x83) {                                                                                                     // Deadlock ON.
      if (!doors_locked || !doors_alarmed) {
        doors_locked_timer = 0;
        serial_log("Doors locked and alarmed.", 2);
        #if DEBUG_MODE
          if (serial_commands_unlocked) {
            serial_commands_unlocked = false;
            EEPROM.update(43, serial_commands_unlocked);
            update_eeprom_checksum();
            serial_log("Locked serial interface.", 0);
          }
        #endif
        doors_locked = doors_alarmed = true;
      }
    } else if (k_msg.buf[0] == 0x82) {
      if (!doors_locked) {
        serial_log("Doors locked, alarm not set.", 2);
        doors_locked = true;
        doors_alarmed = false;
      }
    } else {
      if (doors_locked || doors_alarmed) {
        serial_log("Doors unlocked.", 2);
        doors_locked = doors_alarmed = false;
      }
    }
  } else {
    if (doors_locked || doors_alarmed) {
      serial_log("Doors unlocked.", 2);
      doors_locked = doors_alarmed = false;
    }
  }
}


void indicate_car_locked(uint16_t boot_delay) {
  if (car_locked_indicator_counter < 10) {                                                                                          // Limit excessive activation of this (10 max).
    if (visual_signal_ckm[cas_key_number] && !hazards_on) {
      if (!bitRead(last_trunk_status, 0)) {                                                                                         // Check that the trunk is shut.
        serial_log("Car locked indicator triggered. Flashing.", 2);
        hazards_flash_txq.flush();                                                                                                  // Cancel pending flashes.
        kcan_write_msg(stop_flashing_lights_buf);
        if (!alarm_active) {
          alarm_siren_txq.flush();
        }
        unsigned long time_now = millis();
        if (car_locked_indicator_counter > 1) {                                                                                     // Should make the car easier to find at night. On the 3rd activation.
          m = {flash_hazards_angel_eyes_xenons_buf, boot_delay + time_now + 50};
          if (car_locked_indicator_counter > 2) {
            if (diag_transmit) {
              kcan_write_msg(alarm_beep_6x_buf);
            }
          }
        } else {
          m = {flash_hazards_angel_eyes_buf, boot_delay + time_now + 50};
        }
        hazards_flash_txq.push(&m);
        if (car_locked_indicator_counter > 2) {
          m = {stop_flashing_lights_buf, boot_delay + time_now + 3500};
        } else {
          m = {stop_flashing_lights_buf, boot_delay + time_now + 1900};                                                             // Enough delay for 3x flashes.
        }
        hazards_flash_txq.push(&m);
        car_locked_indicator_counter++;
      }
    }
  }
}


void check_hazards_queue(void) {
  if (!hazards_flash_txq.isEmpty()) {
    hazards_flash_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      hazards_flash_txq.drop();
    }
  }
}


void activate_intermittent_wipers(void) {
  intermittent_wipe_active = true;
  serial_log("Intermittent wiping ON.", 2);
  intermittent_wipe_timer = 0;
  #if F_NBTE
    kcan2_write_msg(idrive_horn_sound_buf);
    send_cc_message("Intermittent wiping ON.", true, 3000);
  #else
    kcan_write_msg(idrive_button_sound_buf);                                                                                        // Acoustic indicator.
  #endif
}


void disable_intermittent_wipers(void) {
  intermittent_wipe_active = false;
  serial_log("Intermittent wiping OFF.", 2);
  intermittent_wipe_timer = 0;
  #if F_NBTE
    kcan2_write_msg(idrive_horn_sound_buf);
    send_cc_message("Intermittent wiping OFF.", true, 3000);
  #else
    kcan_write_msg(idrive_button_sound_buf);                                                                                        // Acoustic indicator.
  #endif
}


void check_intermittent_wipers(void) {
  if (intermittent_wipe_active) {
    uint16_t calculated_interval = intermittent_intervals[intermittent_setting];
    if (!vehicle_moving) {
      calculated_interval += intermittent_intervals_offset_stopped[intermittent_setting];
    } else {                                                                                                                        // Reduce intervals with speed.
      if (indicated_speed >= 50.0) {
        calculated_interval *= 0.8;
      } else if (indicated_speed >= 100.0) {
        calculated_interval *= 0.7;
      }
    }
    if (intermittent_wipe_timer >= calculated_interval) {
      uint8_t single_wipe[] = {8, intermittent_setting_can};
      ptcan_write_msg(make_msg_buf(0x2A6, 2, single_wipe));
      intermittent_wipe_timer = 0;
    }
  }
}


void abort_wipe_after_wash(void) {
  if (!wiper_txq.isEmpty()) {
    serial_log("Wipe after wash aborted.", 2);
    wiper_txq.flush();
    wipe_scheduled = false;
  }
}


void check_wiper_queue(void) {
  if (terminal_r) {
    if (!wiper_txq.isEmpty()) {
      wiper_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        serial_log("Wiping windscreen after washing.", 2);
        ptcan_write_msg(delayed_tx.tx_msg);
        wiper_txq.drop();
        wipe_scheduled = false;
      }
    }
  }
}


void evaluate_front_windows_position(void) {
  if (k_msg.id == 0x3B6) {
    front_left_window_status = k_msg.buf[1];
  } else if (k_msg.id == 0x3B8) {
    front_right_window_status = k_msg.buf[1];
  }
  indicate_comfort_closure();
}


void evaluate_sunroof_position(void) {
  sunroof_status = k_msg.buf[0] + k_msg.buf[1];                                                                                     // Byte0 - longitudinal position, Byte1 - tilt. All 0 when closed.
  indicate_comfort_closure();
}


void indicate_comfort_closure(void) {
  bool windows_closed_ = front_left_window_status == 0xFC 
                         && front_right_window_status == 0xFC
                         && sunroof_status == 0;
  if (windows_closed != windows_closed_) {
    if (lock_button_pressed && windows_closed_) {
      if (visual_signal_ckm[cas_key_number] && !hazards_on) {
        serial_log("Comfort close complete. Flashing hazards 1x long.", 2);
        hazards_flash_txq.flush();                                                                                                  // Cancel any pending flashes.
        kcan_write_msg(stop_flashing_lights_buf);
        unsigned long time_now = millis();
        m = {flash_hazards_single_long_buf, time_now + 300};                                                                        // Slight delay to ensure flash is visible after other flashes.
        hazards_flash_txq.push(&m);
      }
    }
    windows_closed = windows_closed_;
  }
}
