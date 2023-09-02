// Functions that allow the user to send commands to the program go here.


#if DEBUG_MODE
void serial_interpreter(void) {
  String cmd = Serial.readStringUntil('\n', 64);                                                                                    // Limit command the size to prevent overflow.
  if (serial_commands_unlocked) {                                                                                                   // In the unlikely event someone picks up the USB cable and starts sending things.
    if (cmd == "lock_serial") {
      serial_commands_unlocked = false;
      serial_log("  Serial: Locked.");
    }
    else if (cmd == "module_reboot") {
      serial_log("  Serial: Rebooting.");
      Serial.print("   Serial: ");
      update_data_in_eeprom();
      delay(1000);
      module_reboot();
    }
    else if (cmd == "test_watchdog") {
      serial_log("  Serial: Will reboot after watchdog timeout.");
      update_data_in_eeprom();
      delay(1000);
      while(1);
    }
    else if (cmd == "print_status") {
      print_current_state(Serial);
    }
    else if (cmd == "print_setup_time") {
      sprintf(serial_debug_string, "  Serial: %.2f ms", setup_time / 1000.0);
      serial_log(serial_debug_string);
    }
    else if (cmd == "print_voltage") {
      if (battery_voltage > 0) {
        sprintf(serial_debug_string, " Serial: %.2f V", battery_voltage);
        serial_log(serial_debug_string);
      } else {
        serial_log("  Serial: voltage unavailable.");
      }
    }
    else if (cmd == "clear_all_dtcs") {                                                                                             // Should take around 6s to complete.
      if (ignition) {
        if (diag_transmit) {
          serial_log("  Serial: Clearing error and shadow memories.");
          time_now = millis();
          uint8_t clear_fs_job[] = {0, 3, 0x14, 0xFF, 0xFF, 0, 0, 0}, i;
          for (i = 0; i < 0x8C; i++) {
            clear_fs_job[0] = i;
            m = {make_msg_buf(0x6F1, 8, clear_fs_job), time_now + i * 20};
            serial_diag_txq.push(&m);
          }
          time_now += 20;
          uint8_t clear_is_job[] = {0, 3, 0x31, 6, 0, 0, 0, 0}, j;
          for (j = 0; j < 0x8C; j++) {
            clear_fs_job[0] = j;
            m = {make_msg_buf(0x6F1, 8, clear_is_job), time_now + (i + j) * 20};
            serial_diag_txq.push(&m);
          }
          time_now += 20;
          uint8_t clear_dme_hs_job[] = {0x12, 2, 0x31, 3, 0, 0, 0, 0};
          m = {make_msg_buf(0x6F1, 8, clear_dme_hs_job), time_now + (i + j) * 20};
          serial_diag_txq.push(&m);
          clearing_dtcs = true;
        } else {
          serial_log("  Serial: Function unavailable due to OBD tool presence.");
        }
      } else {
        serial_log("  Serial: Activate ignition first.");
      }
    }
    else if (cmd == "cc_gong") {
      if (diag_transmit) {
        serial_log("  Serial: Sending CC gong.");
        play_cc_gong();
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
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
    else if (cmd == "reset_max_temp") {
      max_cpu_temp = 0;
      EEPROM.update(15, max_cpu_temp);
      update_eeprom_checksum();
      serial_log("  Serial: Reset max recorded CPU temp.");
    }
    else if (cmd == "reset_eeprom") {
      for (uint8_t i = 0; i < 1024; i++) {
        EEPROM.update(i, 0);
      }
      serial_log("  Serial: Reset EEPROM values. Rebooting");
      delay(1000);
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
    #if AUTO_STEERING_HEATER
    else if (cmd == "toggle_steering_heater") {
      digitalWrite(STEERING_HEATER_SWITCH_PIN, HIGH);
      delay(100);
      digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);
      serial_log("  Serial: Operated steering heater switch.");
    }
    #endif
    #if FRONT_FOG_CORNER
    else if (cmd == "front_right_fog_on") {
      if (diag_transmit) {
        right_fog_soft(true);
        serial_log("  Serial: Activated front right fog light.");
      }
    }
    else if (cmd == "front_left_fog_on") {
      if (diag_transmit) {
        left_fog_soft(true);
        serial_log("  Serial: Activated front left fog light.");
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
    }
    else if (cmd == "front_right_fog_off") {
      if (diag_transmit) {
        kcan_write_msg(front_right_fog_off_buf);
        serial_log("  Serial: Deactivated front right fog light.");
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
    }
    else if (cmd == "front_left_fog_off") {
      if (diag_transmit) {
        kcan_write_msg(front_left_fog_off_buf);
        serial_log("  Serial: Deactivated front left fog light.");
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
    }
    else if (cmd == "front_fogs_off") {
      if (ignition) {
        if (diag_transmit) {
          kcan_write_msg(front_fogs_all_off_buf);
          serial_log("  Serial: Deactivated front fog lights.");
        } else {
          serial_log("  Serial: Function unavailable due to OBD tool presence.");
        }
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
          if (diag_transmit) {
            Serial.print("  Serial: ");
            needle_sweep_animation();
          } else {
            serial_log("  Serial: Function unavailable due to OBD tool presence.");
          }
        } else {
          serial_log("  Serial: Activate ignition first.");
        }
      #endif
    }
    #endif
    #if AUTO_MIRROR_FOLD
    else if (cmd == "toggle_mirror_fold") {
      if (diag_transmit) {
        toggle_mirror_fold();
        serial_log("  Serial: Sent mirror fold/unfold request.");
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
    }
    else if (cmd == "mirror_fold_status") {
      if (diag_transmit) {
        lock_button_pressed = unlock_button_pressed = false;
        frm_mirror_status_requested = true;
        kcan_write_msg(frm_mirror_status_request_a_buf);
        Serial.print("  Serial: ");
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
    }
    #endif
    #if MIRROR_UNDIM
    else if (cmd == "undim_mirrors") {
      if (diag_transmit) {
        kcan_write_msg(frm_mirror_undim_buf);
        serial_log("  Serial: Sent undim request.");
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.");
      }
    }
    #endif
    #if IMMOBILIZER_SEQ
    else if (cmd == "release_immobilizer") {
      release_immobilizer();
      Serial.print("  Serial: ");
    }
    else if (cmd == "lock_immobilizer") {
      reset_key_cc();
      activate_immobilizer();
      serial_log("  Serial: Locked EKP anti theft.");
    }
    #if IMMOBILIZER_SEQ_ALARM
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
        activate_immobilizer();
        alarm_after_engine_stall = true;
        trip_alarm_after_stall();
        serial_log("  Serial: Stall alarm tripped. Deactivate with IMMOBILIZER_SEQ process.");
      } else {
        serial_log("  Serial: Activate ignition first.");
      }
    }
    #endif
    #endif
    #if RTC
    else if (cmd == "reset_rtc") {
      setTime(0, 0, 0, 1, 1, 2019); 
      time_t t = now();
      Teensy3Clock.set(t); 
      rtc_valid = false;
      serial_log("  Serial: RTC reset to initial value (1/1/19 0:0:0).");
    }
    #endif
    #if MSA_RVC
    else if (cmd == "activate_rvc") {
      if (ignition) {
        kcan_write_msg(pdc_off_camera_on_buf);
        serial_log("  Serial: Activated RVC on display.");
      } else {
        serial_log("  Serial: Activate ignition first.");
      }
    }
    #endif
    #if F_VSW01
    else if (cmd == "vsw_0") {
      Serial.print("  Serial: ");
      vsw_switch_input(0);
    }
    else if (cmd == "vsw_1") {
      Serial.print("  Serial: ");
      vsw_switch_input(1);
    }
    else if (cmd == "vsw_2") {
      Serial.print("  Serial: ");
      vsw_switch_input(2);
    }
    else if (cmd == "vsw_3") {
      Serial.print("  Serial: ");
      vsw_switch_input(3);
    }
    else if (cmd == "vsw_4") {
      Serial.print("  Serial: ");
      vsw_switch_input(4);
    }
    else if (cmd == "vsw_5") {
      Serial.print("  Serial: ");
      vsw_switch_input(5);
    }
    else if (cmd == "vsw_6") {
      Serial.print("  Serial: ");
      vsw_switch_input(6);
    }
    else if (cmd == "vsw_7") {
      Serial.print("  Serial: ");
      vsw_switch_input(7);
    }
    #endif
    else if (cmd == "help") {
      print_help();
    }
  } else {
    if (cmd == serial_password) {
      serial_commands_unlocked = true;
      serial_log("  Serial: Commands unlocked.");
    } else {
      serial_log("  Serial: Locked.");
    }
  }
}


void module_reboot(void) {
  wdt.reset();
}


void print_help(void) {
  serial_log("========================== Help ==========================");
  serial_log("  Commands:");
  serial_log("  lock_serial - Restores the password prompt before accepting serial commands.");
  serial_log("  module_reboot - Saves EEPROM data and restarts the program.");
  serial_log("  test_watchdog - Create an infinite loop to test the watchdog reset.");
  serial_log("  print_status - Prints a set of current runtime variables.");
  serial_log("  print_setup_time - Prints the time it took for the program to complete setup().");
  serial_log("  print_voltage - Prints the battery voltage.");
  serial_log("  clear_all_dtcs - Clear the error memories of every module on the bus.");
  serial_log("  cc_gong - Create an audible check-control gong.");
  #if EXHAUST_FLAP_CONTROL
    serial_log("  open_exhaust_flap - Energize the exhaust solenoid to open the flap.");
    serial_log("  close_exhaust_flap - Release the exhaust solenoid to close the flap.");
  #endif
  serial_log("  toggle_mdrive - Change MDrive state ON-OFF.");
  serial_log("  reset_max_temp - Sets max recorded CPU temperature to 0 in RAM and EEPROM.");
  serial_log("  reset_eeprom - Sets EEPROM bytes to 0xFF and reboots. EEPROM will be rebuilt on reboot.");
  serial_log("  reset_mdrive - Reset MDrive settings to defaults.");
  serial_log("  sleep_ptcan - Deactivates the PT-CAN transceiver.");
  serial_log("  sleep_dcan - Deactivates the D-CAN transceiver.");
  serial_log("  wake_ptcan - Activates the PT-CAN transceiver.");
  serial_log("  wake_dcan - Activates the D-CAN transceiver.");
  #if AUTO_STEERING_HEATER
    serial_log("  toggle_steering_heater - Operates the steering wheel heater switch.");
  #endif
  #if FRONT_FOG_CORNER
    serial_log("  front_right_fog_on - Activates the front right fog light.");
    serial_log("  front_left_fog_on - Activates the front left fog light.");
    serial_log("  front_right_fog_off - Deactivates the front right fog light.");
    serial_log("  front_left_fog_off - Deactivates the front left fog light.");
    serial_log("  front_fogs_off - Deactivates both front fog lights.");
  #endif
  #if NEEDLE_SWEEP || CONTROL_SHIFTLIGHTS
    serial_log("  startup_animation - Displays the KOMBI startup animation(s).");
  #endif
  #if AUTO_MIRROR_FOLD
    serial_log("  toggle_mirror_fold - Change mirror fold state ON-OFF.");
    serial_log("  mirror_fold_status - Prints the mirror fold state.");
  #endif
  #if MIRROR_UNDIM
    serial_log("  undim_mirrors - Undim electrochromic exterior mirrors.");
  #endif
  #if IMMOBILIZER_SEQ
    serial_log("  release_immobilizer - Deactivates the EKP immobilizer.");
    serial_log("  lock_immobilizer - Activates the EKP immobilizer.");
    #if IMMOBILIZER_SEQ_ALARM
      serial_log("  alarm_siren_on - Activates the Alarm siren.");
      serial_log("  alarm_siren_off - Deactivates the Alarm siren.");
      serial_log("  test_trip_stall_alarm - Simulates tripping the EKP immobilizer without disabling it first.");
    #endif
  #endif
  #if RTC
    serial_log("  reset_rtc - Sets teensy's RTC to 01/01/2019 00:00:00 UTC.");
  #endif
  #if MSA_RVC
    serial_log("  activate_rvc - Activates the reversing camera and displays the feed on the CID.");
  #endif
  #if F_VSW01
    serial_log("  vsw_0 - Sets the VideoSwitch to input 0 - disabled.");
    serial_log("  vsw_1 - Sets the VideoSwitch to input 1 (A40*1B Pins: 1 FBAS+, 19 FBAS-, 37 Shield).");
    serial_log("  vsw_2 - Sets the VideoSwitch to input 2 (A40*1B Pins: 2 FBAS+, 20 FBAS-, 38 Shield).");
    serial_log("  vsw_3 - Sets the VideoSwitch to input 3 (A40*1B Pins: 3 FBAS+, 21 FBAS-, 39 Shield).");
    serial_log("  vsw_4 - Sets the VideoSwitch to input 4 (A40*1B Pins: 4 FBAS+, 22 FBAS-, 40 Shield).");
    serial_log("  vsw_5 - Sets the VideoSwitch to input 5 (A40*1B Pins: 5 FBAS+, 23 FBAS-, 41 Shield).");
    serial_log("  vsw_6 - Sets the VideoSwitch to input 6. This position exists but is not wired originally.");
    serial_log("  vsw_7 - Sets the VideoSwitch to input 7. This position exists but is not wired originally.");
  #endif
  serial_log("==========================================================");
}


void check_serial_diag_queue(void) {
if (!serial_diag_txq.isEmpty()) {
    serial_diag_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      dcan_write_msg(delayed_tx.tx_msg);
      serial_diag_txq.drop();
    }
  } else {
    if (clearing_dtcs) {
      clearing_dtcs = false;
      diag_transmit = true;
      serial_log("  Serial: Error memories cleared. Cycle Terminal R ON/OFF.");
    }
  }
}
#endif
