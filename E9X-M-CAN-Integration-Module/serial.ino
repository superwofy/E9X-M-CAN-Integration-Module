// Functions that allow the user to send commands to the program go here.


#if DEBUG_MODE
void serial_interpreter() {
  String cmd = Serial.readStringUntil('\n');
  if (cmd == "module_reboot") {
    serial_log("Serial: Rebooting.");
    delay(200);
    module_reboot();
  }
  if (cmd == "test_watchdog") {
    serial_log("Serial: Will reboot after watchdog timeout.");
    while(1);
  }
  if (cmd == "print_status") {
    print_current_state(Serial);
  }
  if (cmd == "print_voltage") {
    if (battery_voltage > 0) {
      sprintf(serial_debug_string, " Serial: %.2f V", battery_voltage);
      Serial.println(serial_debug_string);
    } else {
      Serial.println(" Serial: voltage unavailable.");
    }
  }
  if (cmd == "cc_gong") {
    serial_log("Serial: Sending CC gong.");
    kcan_write_msg(cc_gong_buf);
  } 
  #if EXHAUST_FLAP_CONTROL
  else if (cmd == "open_exhaust_flap") {
    actuate_exhaust_solenoid(LOW);
    Serial.print("  Serial: Opened exhaust flap.");
  }
  else if (cmd == "close_exhaust_flap") {
    actuate_exhaust_solenoid(HIGH);
    Serial.print("  Serial: Closed exhaust flap.");
  }
  #endif
  else if (cmd == "toggle_mdrive") {
    toggle_mdrive_message_active();
    send_mdrive_message();
    toggle_mdrive_dsc_mode(); 
  } 
  else if (cmd == "reset_eeprom") {
    for (uint8_t i = 0; i < 16; i++) {
      EEPROM.update(i, 0xFF);
    }
    serial_log("  Serial: Reset EEPROM values. Rebooting");
    module_reboot();
  } 
  else if (cmd == "reset_mdrive") {
    Serial.print("  Serial: ");
    reset_mdrive_settings();
  } 
  else if (cmd == "sleep_ptcan") {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Deactivated PT-CAN transceiver.");
  } 
  else if (cmd == "sleep_dcan") {
    digitalWrite(DCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Deactivated D-CAN transceiver.");
  } 
  else if (cmd == "wake_ptcan") {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Activated PT-CAN transceiver.");
  } 
  else if (cmd == "wake_dcan") {
    digitalWrite(DCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Activated D-CAN transceiver.");
  }
  #if FRONT_FOG_CORNER
  else if (cmd == "front_right_fog_on") {
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
    serial_log("  Serial: Activated front right fog light.");
  }
  else if (cmd == "front_left_fog_on") {
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
    serial_log("  Serial: Activated front left fog light.");
  }
  else if (cmd == "front_right_fog_off") {
    kcan_write_msg(front_right_fog_off_buf);
    serial_log("  Serial: Deactivated front right fog light.");
  }
  else if (cmd == "front_left_fog_off") {
    kcan_write_msg(front_left_fog_off_buf);
    serial_log("  Serial: Deactivated front left fog light.");
  }
  else if (cmd == "front_fogs_off") {
    if (ignition) {
      kcan_write_msg(front_fogs_all_off_buf);
      serial_log("  Serial: Deactivated front fog lights.");
    } else {
      serial_log("  Serial: Activate ignition first.");
    }
  }
  #endif
  #if NEEDLE_SWEEP || CONTROL_SHIFTLIGHTS
  else if (cmd == "startup_animation") {
    #if CONTROL_SHIFTLIGHTS
      Serial.print("  Serial: ");
      shiftlight_startup_animation();
    #endif
    #if NEEDLE_SWEEP
      if (ignition) {
        Serial.print("  Serial: ");
        needle_sweep_animation();
      }
    #endif
  }
  #endif
  #if AUTO_MIRROR_FOLD
    else if (cmd == "toggle_mirror_fold") {
      toggle_mirror_fold();
      serial_log("  Serial: Sent mirror fold request.");
    }
    else if (cmd == "mirror_fold_status") {
      lock_button_pressed = unlock_button_pressed = false;
      frm_status_requested = true;
      kcan_write_msg(frm_status_request_a_buf);
      Serial.print("  Serial: ");
    }
  #endif
  #if ANTI_THEFT_SEQ
    else if (cmd == "release_anti_theft") {
      kcan_write_msg(ekp_return_to_normal_buf);
      kcan_write_msg(key_cc_off_buf);
      anti_theft_released = key_cc_sent = true;
      serial_log("  Serial: Released EKP anti theft.");
    }
    else if (cmd == "lock_anti_theft") {
      reset_key_cc();
      anti_theft_released = false;
      anti_theft_pressed_count = 0;
      serial_log("  Serial: Locked EKP anti theft.");
    }
    #if ANTI_THEFT_SEQ_ALARM
      else if (cmd == "alarm_siren_on") {
        kcan_write_msg(alarm_siren_on_buf);
        serial_log("  Serial: Alarm siren ON.");
      }
      else if (cmd == "alarm_siren_off") {
        kcan_write_msg(alarm_siren_off_buf);
        serial_log("  Serial: Alarm siren OFF.");
      }
      else if (cmd == "test_trip_stall_alarm") {
        if (ignition) {
          engine_running = alarm_after_engine_stall = true;
          serial_log("  Serial: Stall alarm tripped.");
        } else {
          serial_log("  Serial: Activate ignition first.");
        }
      }
    #endif
  #endif
}


void module_reboot() {
  wdt.reset();
}
#endif
