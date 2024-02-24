// Functions that allow the user to send commands to the program go here.


void serial_debug_interpreter(void) {
  String cmd = Serial.readStringUntil('\n', 64);                                                                                    // Limit command the size to prevent overflow.
  if (serial_commands_unlocked) {                                                                                                   // In the unlikely event someone picks up the USB cable and starts sending things.
    if (cmd == "lock_serial") {
      serial_commands_unlocked = false;
      EEPROM.update(43, serial_commands_unlocked);
      update_eeprom_checksum();
      serial_log("  Serial: Commands locked.", 0);
    }
    else if (cmd == "loglevel_0") {
      LOGLEVEL = 0;
      serial_log("  Serial: LOGLEVEL set to 0.", 0);
    }
    else if (cmd == "loglevel_1") {
      LOGLEVEL = 1;
      serial_log("  Serial: LOGLEVEL set to 1.", 0);
    }
    else if (cmd == "loglevel_2") {
      LOGLEVEL = 2;
      serial_log("  Serial: LOGLEVEL set to 2.", 0);
    }
    else if (cmd == "loglevel_3") {
      LOGLEVEL = 3;
      serial_log("  Serial: LOGLEVEL set to 3.", 0);
    }
    else if (cmd == "loglevel_4") {
      LOGLEVEL = 4;
      serial_log("  Serial: LOGLEVEL set to 4.", 0);
    }
    else if (cmd == "module_reboot") {
      serial_log("  Serial: Rebooting.", 0);
      if (LOGLEVEL >= 2) {
        Serial.print("  Serial: ");
      } else {
        serial_log("  Serial: Saved data to EEPROM.", 0);
      }
      update_data_in_eeprom();
      delay(1000);
      wdt.reset();
    }
    else if (cmd == "test_watchdog") {
      serial_log("  Serial: Will reboot after watchdog timeout.", 0);
      update_data_in_eeprom();
      delay(1000);
      while(1);
    }
    else if (cmd == "power_down") {
      if (vehicle_awake) {
        if (!terminal_r) {
          if (diag_transmit) {
            serial_log("  Serial: Sending power down command.", 0);
            power_down_requested = true;
            kcan_write_msg(power_down_cmd_a_buf);
          } else {
            serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
          }
        } else {
          serial_log("  Serial: Terminal R must be OFF.", 0);
        }
      } else {
        serial_log("  Serial: Wake vehicle first.", 0);
      }
    }
    else if (cmd == "print_status") {
      print_current_state(Serial);
    }
    else if (cmd == "print_boot_log") {
      Serial.println("======================== Boot Log ========================");
      Serial.print(boot_debug_string);
      Serial.println("==========================================================");
    }
    else if (cmd == "print_can_config") {
      serial_log("========================== KCAN ==========================", 0);
      KCAN.mailboxStatus();
      serial_log("", 0);
      serial_log("========================= PT-CAN =========================", 0);
      PTCAN.mailboxStatus();
      serial_log("", 0);
      serial_log("========================== DCAN ==========================", 0);
      DCAN.mailboxStatus();
      serial_log("", 0);
      uint8_t clocksrc[4] = {60, 24, 80, 0};
      sprintf(serial_debug_string, "  Serial: FlexCAN clock speed is %d MHz.", clocksrc[(CCM_CSCMR2 & 0x300) >> 8]);
      serial_log(serial_debug_string, 0);
    }
    else if (cmd == "clear_all_dtcs") {                                                                                             // Should take around 6s to complete.
      if (ignition) {
        if (diag_transmit) {
          serial_log("  Serial: Clearing error and shadow memories.", 0);
          unsigned long time_now = millis();
          uint8_t clear_fs_job[] = {0, 3, 0x14, 0xFF, 0xFF, 0, 0, 0};
          for (uint8_t i = 0; i < 0x8C; i++) {
            #if F_NBT
              if (i == 0x63 || i == 0x67 || i == 0x35) {                                                                            // NBT, ZBE, TBX.
                continue;
              }
            #endif
            clear_fs_job[0] = i;
            m = {make_msg_buf(0x6F1, 8, clear_fs_job), time_now};
            serial_diag_kcan1_txq.push(&m);
            time_now += 50;
          }
          uint8_t clear_is_job[] = {0, 3, 0x31, 6, 0, 0, 0, 0};
          for (uint8_t i = 0; i < 0x8C; i++) {
            clear_fs_job[0] = i;
            m = {make_msg_buf(0x6F1, 8, clear_is_job), time_now};
            serial_diag_kcan1_txq.push(&m);
            time_now += 50;
          }
          uint8_t clear_dme_hs_job[] = {0x12, 2, 0x31, 3, 0, 0, 0, 0};
          m = {make_msg_buf(0x6F1, 8, clear_dme_hs_job), time_now};
          serial_diag_kcan1_txq.push(&m);
          time_now += 50;
          #if F_NBT
            m = {clear_fs_job_uds_nbt_buf, time_now};
            serial_diag_kcan2_txq.push(&m);
            time_now += 50;
            m = {clear_is_job_uds_nbt_buf, time_now};
            serial_diag_kcan2_txq.push(&m);
            time_now += 50;
            m = {clear_fs_job_uds_zbe_buf, time_now};                                                                               // Send the clear command to KCAN2 in case there's a ZBE retrofitted there.
            serial_diag_kcan2_txq.push(&m);
            time_now += 50;
            #if !F_NBT_CCC_ZBE
              m = {clear_is_job_uds_zbe_buf, time_now};
              serial_diag_kcan2_txq.push(&m);
              time_now += 50;
              m = {clear_fs_job_uds_tbx_buf, time_now};
              serial_diag_kcan2_txq.push(&m);
              time_now += 50;
              m = {clear_is_job_uds_tbx_buf, time_now};
              serial_diag_kcan2_txq.push(&m);
              time_now += 50;
            #endif
          #endif
          #if F_ZBE_KCAN1 && !F_NBT
            m = {clear_fs_job_uds_zbe_buf, time_now};
            serial_diag_kcan1_txq.push(&m);
            time_now += 50;
            m = {clear_is_job_uds_zbe_buf, time_now};
            serial_diag_kcan1_txq.push(&m);
            time_now += 50;
          #endif
          #if F_VSW01
            m = {clear_fs_job_vsw_buf, time_now};
            serial_diag_kcan1_txq.push(&m);
            time_now += 50;
            m = {clear_is_job_vsw_buf, time_now};
            serial_diag_kcan1_txq.push(&m);
            time_now += 50;
          #endif
          clearing_dtcs = true;
        } else {
          serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
        }
      } else {
        serial_log("  Serial: Activate ignition first.", 0);
      }
    }
    else if (cmd == "cc_gong") {
      if (diag_transmit) {
        serial_log("  Serial: Sending CC gong.", 0);
        play_cc_gong(1);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }

    else if (cmd == "lc_on"){
      if (diag_transmit) {
        serial_log("  Serial: Launch Control Display activated.", 0);
        kcan_write_msg(lc_cc_on_buf);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }

    else if (cmd == "lc_off"){
      if (diag_transmit) {
        serial_log("  Serial: Launch Control Display deactivated.", 0);
        kcan_write_msg(lc_cc_off_buf);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }

    #if EXHAUST_FLAP_CONTROL
    else if (cmd == "open_exhaust_flap") {
      actuate_exhaust_solenoid(LOW);
      serial_log("  Serial: Opened exhaust flap.", 0);
    }
    else if (cmd == "close_exhaust_flap") {
      actuate_exhaust_solenoid(HIGH);
      serial_log("  Serial: Closed exhaust flap.", 0);
    }
    #endif
    else if (cmd == "toggle_mdrive") {
      toggle_mdrive_message_active();
      send_mdrive_message();
      toggle_mdrive_dsc_mode(); 
    }
    else if (cmd == "reset_eeprom") {
      for (uint8_t i = 0; i < 1024; i++) {
        EEPROM.update(i, 0);
      }
      serial_log("  Serial: Reset EEPROM values. Rebooting", 0);
      delay(1000);
      wdt.reset();
    } 
    else if (cmd == "reset_mdrive") {
      if (LOGLEVEL >= 2) {
        Serial.print("  Serial: ");
      } else {
        serial_log("  Serial: Reset MDrive settings.", 0);
      }
      reset_mdrive_settings();
    } 
    else if (cmd == "sleep_ptcan") {
      digitalWrite(PTCAN_STBY_PIN, HIGH);
      serial_log("  Serial: Deactivated PT-CAN transceiver.", 0);
    } 
    else if (cmd == "sleep_dcan") {
      digitalWrite(DCAN_STBY_PIN, HIGH);
      serial_log("  Serial: Deactivated D-CAN transceiver.", 0);
    } 
    else if (cmd == "wake_ptcan") {
      digitalWrite(PTCAN_STBY_PIN, HIGH);
      serial_log("  Serial: Activated PT-CAN transceiver.", 0);
    } 
    else if (cmd == "wake_dcan") {
      digitalWrite(DCAN_STBY_PIN, HIGH);
      serial_log("  Serial: Activated D-CAN transceiver.", 0);
    }
    #if AUTO_STEERING_HEATER
    else if (cmd == "toggle_steering_heater") {
      digitalWrite(STEERING_HEATER_SWITCH_PIN, HIGH);
      delay(100);
      digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);
      serial_log("  Serial: Operated steering heater switch.", 0);
    }
    #endif
    #if FRONT_FOG_CORNER
    else if (cmd == "front_right_fog_on") {
      if (diag_transmit) {
        right_fog_soft(true);
        serial_log("  Serial: Activated front right fog light.", 0);
      }
    }
    else if (cmd == "front_left_fog_on") {
      if (diag_transmit) {
        left_fog_soft(true);
        serial_log("  Serial: Activated front left fog light.", 0);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    else if (cmd == "front_right_fog_off") {
      if (diag_transmit) {
        kcan_write_msg(front_right_fog_off_buf);
        serial_log("  Serial: Deactivated front right fog light.", 0);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    else if (cmd == "front_left_fog_off") {
      if (diag_transmit) {
        kcan_write_msg(front_left_fog_off_buf);
        serial_log("  Serial: Deactivated front left fog light.", 0);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    else if (cmd == "front_fogs_off") {
      if (ignition) {
        if (diag_transmit) {
          kcan_write_msg(front_fogs_all_off_buf);
          serial_log("  Serial: Deactivated front fog lights.", 0);
        } else {
          serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
        }
      } else {
        serial_log("  Serial: Activate ignition first.", 0);
      }
    }
    #endif
    #if NEEDLE_SWEEP || CONTROL_SHIFTLIGHTS
    else if (cmd == "startup_animation") {
      if (ignition) {
        #if CONTROL_SHIFTLIGHTS
          if (LOGLEVEL >= 3) {
            Serial.print("  Serial: ");
          } else {
            serial_log("  Serial: Showing shift light on engine startup.", 0);
          }
          shiftlight_startup_animation();
        #endif
        #if NEEDLE_SWEEP
          if (diag_transmit) {
            if (LOGLEVEL >= 3) {
              Serial.print("  Serial: ");
            } else {
              serial_log("  Serial: Sending needle sweep animation.", 0);
            }
            needle_sweep_animation();
          } else {
            serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
          }
        #endif
      } else {
        serial_log("  Serial: Activate ignition first.", 0);
      }
    }
    #endif
    #if AUTO_MIRROR_FOLD
    else if (cmd == "toggle_mirror_fold") {
      if (diag_transmit) {
        bool unfold_with_door_open_ = unfold_with_door_open, eeprom_unsaved_ = eeprom_unsaved;                                      // Back these values up.
        toggle_mirror_fold(false);
        unfold_with_door_open = unfold_with_door_open_;
        eeprom_unsaved = eeprom_unsaved_;
        serial_log("  Serial: Sent mirror fold/unfold request.", 0);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    else if (cmd == "mirror_fold_status") {
      if (diag_transmit) {
        fold_lock_button_pressed = fold_unlock_button_pressed = false;
        frm_mirror_status_requested = true;
        kcan_write_msg(frm_mirror_status_request_a_buf);
        if (LOGLEVEL >= 2) {
          Serial.print("  Serial: ");
        } else {
          serial_log("  Serial: Checking mirror status.", 0);
        }
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    #endif
    #if MIRROR_UNDIM
    else if (cmd == "undim_mirrors") {
      if (diag_transmit) {
        kcan_write_msg(frm_mirror_undim_buf);
        serial_log("  Serial: Sent undim request.", 0);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    #endif
    #if IMMOBILIZER_SEQ
    else if (cmd == "release_immobilizer") {
      Serial.print("  Serial: ");
      release_immobilizer();
    }
    else if (cmd == "lock_immobilizer") {
      reset_key_cc();
      Serial.print("  Serial: ");
      activate_immobilizer();
      serial_log("  Serial: Locked EKP anti theft.", 0);
    }
    else if (cmd == "alarm_siren_on") {
      alarm_siren_txq.flush();
      kcan_write_msg(alarm_siren_on_buf);
      serial_log("  Serial: Alarm siren ON.", 0);
    }
    else if (cmd == "alarm_siren_off") {
      alarm_siren_txq.flush();
      kcan_write_msg(alarm_siren_return_control_buf);
      serial_log("  Serial: Alarm siren OFF.", 0);
    }
    else if (cmd == "alarm_led_on") {
      alarm_led_txq.flush();
      kcan_write_msg(alarm_led_on_buf);
      serial_log("  Serial: Alarm LED ON.", 0);
    }
    else if (cmd == "alarm_led_off") {
      alarm_led_txq.flush();
      kcan_write_msg(alarm_led_return_control_buf);
      serial_log("  Serial: Alarm LED OFF.", 0);
    }
    else if (cmd == "test_trip_stall_alarm") {
      if (diag_transmit) {
        if (ignition) {
          if (!engine_running) {
            Serial.print("  Serial: ");
            activate_immobilizer();
            Serial.print("  Serial: ");
            enable_alarm_after_stall();
            while (!alarm_warnings_txq.isEmpty()) {                                                                                 // Simulate engine stall by waiting and executing warning sound.
              alarm_warnings_txq.peek(&delayed_tx);
              if (millis() >= delayed_tx.transmit_time) {
                kcan_write_msg(delayed_tx.tx_msg);
                #if F_NBT
                  kcan2_write_msg(delayed_tx.tx_msg);
                #endif
                alarm_warnings_txq.drop();
              }
            }
            idrive_alive_timer = 0;
            execute_alarm_after_stall();
            serial_log("  Serial: Stall alarm tripped. Deactivate with IMMOBILIZER_SEQ process.", 0);
          } else {
            serial_log("  Serial: Shut down engine first.", 0);
          }
        } else {
          serial_log("  Serial: Activate ignition first.", 0);
        }
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    #endif
    #if MSA_RVC
    else if (cmd == "activate_rvc") {
      if (ignition) {
        kcan_write_msg(pdc_off_camera_on_buf);
        #if F_NBT
          kcan2_write_msg(pdc_off_camera_on_buf);
        #endif
        serial_log("  Serial: Activated RVC on display.", 0);
      } else {
        serial_log("  Serial: Activate ignition first.", 0);
      }
    }
    #endif
    #if F_VSW01
    else if (cmd == "vsw_0") {
      if (LOGLEVEL >= 3) {
        Serial.print("  Serial: ");
      } else {
        sprintf(serial_debug_string, "  Serial: Sent VSW/0 (%s) request.", vsw_positions[0]);
        serial_log(serial_debug_string, 0);
      }
      vsw_switch_input(0);
    }
    else if (cmd == "vsw_1") {
      if (LOGLEVEL >= 3) {
        Serial.print("  Serial: ");
      } else {
        sprintf(serial_debug_string, "  Serial: Sent VSW/1 (%s) request.", vsw_positions[1]);
        serial_log(serial_debug_string, 0);
      }
      vsw_switch_input(1);
    }
    else if (cmd == "vsw_2") {
      if (LOGLEVEL >= 3) {
        Serial.print("  Serial: ");
      } else {
        sprintf(serial_debug_string, "  Serial: Sent VSW/2 (%s) request.", vsw_positions[2]);
        serial_log(serial_debug_string, 0);
      }
      vsw_switch_input(2);
    }
    else if (cmd == "vsw_3") {
      if (LOGLEVEL >= 3) {
        Serial.print("  Serial: ");
      } else {
        sprintf(serial_debug_string, "  Serial: Sent VSW/3 (%s) request.", vsw_positions[3]);
        serial_log(serial_debug_string, 0);
      }
      vsw_switch_input(3);
    }
    else if (cmd == "vsw_4") {
      if (LOGLEVEL >= 3) {
        Serial.print("  Serial: ");
      } else {
        sprintf(serial_debug_string, "  Serial: Sent VSW/4 (%s) request.", vsw_positions[4]);
        serial_log(serial_debug_string, 0);
      }
      vsw_switch_input(4);
    }
    else if (cmd == "vsw_5") {
      if (LOGLEVEL >= 3) {
        Serial.print("  Serial: ");
      } else {
        sprintf(serial_debug_string, "  Serial: Sent VSW/5 (%s) request.", vsw_positions[5]);
        serial_log(serial_debug_string, 0);
      }
      vsw_switch_input(5);
    }
    #endif
    #if F_NBT
    else if (cmd == "hu_reboot") {
      if (diag_transmit) {
        kcan2_write_msg(f_hu_nbt_reboot_buf);
        serial_log("  Serial: Sent HU reboot job.", 0);
      } else {
        serial_log("  Serial: Function unavailable due to OBD tool presence.", 0);
      }
    }
    else if (cmd == "custom_cc_test") {
      send_cc_message_text("Hello world!                  ", 5000);
    }
    else if (cmd == "faceplate_eject") {
      unsigned long time_now = millis();
      m = {faceplate_eject_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      time_now += 200;
      m = {faceplate_a1_released_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      serial_log("  Serial: Sent eject button.", 0);
    }
    else if (cmd == "faceplate_mute") {
      unsigned long time_now = millis();
      m = {faceplate_power_mute_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      time_now += 200;
      m = {faceplate_a1_released_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      serial_log("  Serial: Sent power/mute button.", 0);
    }
    else if (cmd == "faceplate_seek_left") {
      unsigned long time_now = millis();
      m = {faceplate_seek_left_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      time_now += 200;
      m = {faceplate_a3_released_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      serial_log("  Serial: Sent seek left button.", 0);
    }
    else if (cmd == "faceplate_seek_right") {
      unsigned long time_now = millis();
      m = {faceplate_seek_right_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      time_now += 200;
      m = {faceplate_a3_released_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      serial_log("  Serial: Sent seek right button.", 0);
    }
    else if (cmd == "faceplate_decrease_volume") {
      unsigned long time_now = millis();
      m = {faceplate_volume_decrease_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      time_now += 200;
      m = {faceplate_f1_released_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      serial_log("  Serial: Sent volume down.", 0);
    }
    else if (cmd == "faceplate_increase_volume") {
      unsigned long time_now = millis();
      m = {faceplate_volume_increase_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      time_now += 200;
      m = {faceplate_f1_released_buf, time_now};
      serial_diag_kcan2_txq.push(&m);
      serial_log("  Serial: Sent volume up.", 0);
    }
    #endif
    else if (cmd == "help") {
      print_help();
    }
    else {
      if (cmd == serial_password) {
        serial_log("  Serial: Already unlocked.", 0);
      } else {
        sprintf(serial_debug_string, "  Serial: command '%s' not recognized. "
                "Use 'help' to see a list of available commands.", cmd.c_str());
        serial_log(serial_debug_string, 0);
      }
    }
    serial_unlocked_timer = 0;
  } else {
    if (cmd == serial_password) {
      serial_unlocked_timer = 0;
      serial_commands_unlocked = true;
      EEPROM.update(43, serial_commands_unlocked);
      update_eeprom_checksum();
      serial_log("  Serial: Commands unlocked.", 0);
    } else {
      serial_log("  Serial: Locked. Please input the password as the next command.", 0);
    }
  }
}


void print_help(void) {
  serial_log("  ========================== Help ==========================\r\n"
  "  Commands:\r\n"
  "  lock_serial - Restores the password prompt before accepting serial commands.\r\n"
  "  loglevel_0 - Sets LOGLEVEL to 0 - critical.\r\n"
  "  loglevel_1 - Sets LOGLEVEL to 1 - error.\r\n"
  "  loglevel_2 - Sets LOGLEVEL to 2 - info.\r\n"
  "  loglevel_3 - Sets LOGLEVEL to 3 - extra_info.\r\n"
  "  loglevel_4 - Sets LOGLEVEL to 4 - debug.\r\n"
  "  module_reboot - Saves EEPROM data and restarts the program.\r\n"
  "  test_watchdog - Create an infinite loop to test the watchdog reset.\r\n"
  "  power_down - Assume sleep mode immediately. KL30G will be turned OFF.\r\n"
  "  print_status - Prints a set of current runtime variables.\r\n"
  "  print_boot_log - Prints the first 10s of serial messages and timing information.\r\n"
  "  print_can_config - Prints the configuration of the FlexCAN module.\r\n"
  "  clear_all_dtcs - Clear the error memories of every module on the bus.\r\n"
  "  cc_gong - Create an audible check-control gong.", 0);
  #if EXHAUST_FLAP_CONTROL
    serial_log("  open_exhaust_flap - Energize the exhaust solenoid to open the flap.\r\n"
    "  close_exhaust_flap - Release the exhaust solenoid to close the flap.", 0);
  #endif

  #if LAUNCH_CONTROL_INDICATOR
  serial_log("  lc_on - Activates the Launch Control Display\r\n", 0);
  serial_log("  lc_off - Disables the Launch Control", 0);
  #endif

  serial_log("  toggle_mdrive - Change MDrive state ON-OFF.\r\n"
    "  reset_eeprom - Sets EEPROM bytes to 0xFF and reboots. EEPROM will be rebuilt on reboot.\r\n"
    "  reset_mdrive - Reset MDrive settings to defaults.\r\n"
    "  sleep_ptcan - Deactivates the PT-CAN transceiver.\r\n"
    "  sleep_dcan - Deactivates the D-CAN transceiver.\r\n"
    "  wake_ptcan - Activates the PT-CAN transceiver.\r\n"
    "  wake_dcan - Activates the D-CAN transceiver.", 0);
  #if AUTO_STEERING_HEATER
    serial_log("  toggle_steering_heater - Operates the steering wheel heater switch.", 0);
  #endif
  #if FRONT_FOG_CORNER
    serial_log("  front_right_fog_on - Activates the front right fog light.\r\n"
    "  front_left_fog_on - Activates the front left fog light.\r\n"
    "  front_right_fog_off - Deactivates the front right fog light.\r\n"
    "  front_left_fog_off - Deactivates the front left fog light.\r\n"
    "  front_fogs_off - Deactivates both front fog lights.", 0);
  #endif
  #if NEEDLE_SWEEP || CONTROL_SHIFTLIGHTS
    serial_log("  startup_animation - Displays the KOMBI startup animation(s).", 0);
  #endif
  #if AUTO_MIRROR_FOLD
    serial_log("  toggle_mirror_fold - Change mirror fold state ON-OFF.\r\n"
    "  mirror_fold_status - Prints the mirror fold state.", 0);
  #endif
  #if MIRROR_UNDIM
    serial_log("  undim_mirrors - Undim electrochromic exterior mirrors.", 0);
  #endif
  #if IMMOBILIZER_SEQ
    serial_log("  release_immobilizer - Deactivates the EKP immobilizer.\r\n"
    "  lock_immobilizer - Activates the EKP immobilizer.", 0);
    serial_log("  alarm_siren_on - Activates the Alarm siren.\r\n"
    "  alarm_siren_off - Deactivates the Alarm siren.\r\n"
    "  alarm_led_on - Activates the Alarm interior mirror LED.\r\n"
    "  alarm_led_off - Deactivates the Alarm interior mirror LED.\r\n"
    "  test_trip_stall_alarm - Simulates tripping the EKP immobilizer without disabling it first.", 0);
  #endif
  #if MSA_RVC
    serial_log("  activate_rvc - Activates the reversing camera and displays the feed on the CID.", 0);
  #endif
  #if F_VSW01
    serial_log("  vsw_0 - Sets the VideoSwitch to input 0 - disabled.\r\n"
    "  vsw_1 - Sets the VideoSwitch to input 1 (A40*1B Pins: 1 FBAS+, 19 FBAS-, 37 Shield).\r\n"
    "  vsw_2 - Sets the VideoSwitch to input 2 (A40*1B Pins: 2 FBAS+, 20 FBAS-, 38 Shield).\r\n"
    "  vsw_3 - Sets the VideoSwitch to input 3 (A40*1B Pins: 3 FBAS+, 21 FBAS-, 39 Shield).\r\n"
    "  vsw_4 - Sets the VideoSwitch to input 4 (A40*1B Pins: 4 FBAS+, 22 FBAS-, 40 Shield).\r\n"
    "  vsw_5 - Sets the VideoSwitch to input 5 (A40*1B Pins: 5 FBAS+, 23 FBAS-, 41 Shield).", 0);
  #endif
  #if F_NBT
    serial_log("  hu_reboot - Restart the NBT_HU immediately."
    "  custom_cc_test - Print hello world as a Check Control notification.\r\n"
    "  faceplate_eject - Simulate pressing the eject button on the faceplate.\r\n"
    "  faceplate_mute - Simulate pressing the power/mute button on the faceplate.\r\n"
    "  faceplate_seek_left - Simulate pressing the previous track button on the faceplate.\r\n"
    "  faceplate_seek_right - Simulate pressing the next track button on the faceplate.\r\n"
    "  faceplate_decrease_volume - Simulate turning the volume knob anti-clockwise on the faceplate.\r\n"
    "  faceplate_increase_volume - Simulate turning the volume knob clockwise on the faceplate.", 0);
  #endif
  serial_log("  ==========================================================", 0);
}


void check_serial_diag_actions(void) {
  if (!serial_diag_kcan1_txq.isEmpty() || !serial_diag_kcan2_txq.isEmpty()) {
    if (!serial_diag_kcan1_txq.isEmpty()) {
      serial_diag_kcan1_txq.peek(&delayed_tx);
      if (millis() >= delayed_tx.transmit_time) {
        dcan_write_msg(delayed_tx.tx_msg);
        serial_diag_kcan1_txq.drop();
      }
    }
    #if F_NBT 
      if (!serial_diag_kcan2_txq.isEmpty()) {
        serial_diag_kcan2_txq.peek(&delayed_tx);
        if (millis() >= delayed_tx.transmit_time) {
          kcan2_write_msg(delayed_tx.tx_msg);
          serial_diag_kcan2_txq.drop();
        }
      }
    #endif
  } else {
    if (clearing_dtcs) {
      clearing_dtcs = false;
      diag_transmit = true;
      serial_log("  Serial: Error memories cleared. Cycle Terminal R OFF/ON.", 0);
    }
  }

  if (serial_commands_unlocked && serial_unlocked_timer >= 900000) {
    serial_commands_unlocked = false;
    EEPROM.update(43, serial_commands_unlocked);
    update_eeprom_checksum();
    serial_log("  Serial: Locked after timeout.", 0);
  }
}
