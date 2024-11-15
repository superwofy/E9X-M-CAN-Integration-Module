#include "E9X-M-CAN-Integration-Module.h"


void setup() {
  if (F_CPU_ACTUAL != MEDIUM_UNDERCLOCK) {                                                                                          // Prevent accidental overclocks/underclocks. Remove if needed.
    serial_log("CPU clock is not set to startup value, correcting.", 1);
    set_arm_clock(MEDIUM_UNDERCLOCK);
  }
  initialize_watchdog();                                                                                                            // systick: 880 +/- 20 μs is when this function completes. If clock speed is wrong, this takes longer.
  configure_flexcan();                                                                                                              // systick: 1.15 +/- 0.05 ms is when the CAN subsystem is fully ready.
  configure_mcp2515();                                                                                                              // Starting the MCP2515 takes around 2.2 ms because of delay(2)!
  configure_IO();
  activate_usb();                                                                                                                   // This code ensures compatibility with unmodified Teensy cores since USB init will work anyway.
  read_initialize_eeprom();                                                                                                         // EEPROM read takes about 0.5 ms. If EEPROM is corrupt, this takes longer.
  update_mdrive_settings_can_message();
  cache_can_message_buffers();
  initialize_can_handlers();
  sprintf(serial_debug_string, "Setup complete at systick: %.2f ms.", micros() / 1000.0);
  serial_log(serial_debug_string, 2);                                                                                               // systick: 4 ms, setup() is complete.
}


void loop() {
/***********************************************************************************************************************************************************************************************************************************************
  General
***********************************************************************************************************************************************************************************************************************************************/

  check_teensy_cpu_temp();                                                                                                          // Monitor processor temperature.
  #if defined(USB_TRIPLE_SERIAL)
    if (SerialUSB2.dtr()) {
      if (!slcan_connected) {
        slcan_connected = true;
        slcan_timer = 0;
        serial_log("SLCAN interface connected.", 0);
      }
      read_slcan_cmd();
    } else {
      if (slcan_connected) {
        slcan_connected = false;
        serial_log("SLCAN interface disconnected.", 0);
      }
    }
  #endif

  if (vehicle_awake) {
    check_can_resend_queues();
    #if EXHAUST_FLAP_CONTROL
      control_exhaust_flap_user();
    #endif
    check_diag_transmit_status();
    #if DOOR_VOLUME
      check_idrive_queue();
    #endif
    #if NEEDLE_SWEEP
      check_kombi_needle_queue();
    #endif
    #if WIPE_AFTER_WASH
      check_wiper_queue();
    #endif
    #if AUTO_MIRROR_FOLD
      check_mirror_fold_queue();
    #endif
    #if IMMOBILIZER_SEQ
      check_immobilizer_status();
    #endif
    #if DIM_DRL
      check_drl_queue();
    #endif
    #if FRONT_FOG_CORNER
      check_fog_corner_queue();
    #endif
    check_hu_application_alive_monitor();
    #if F_VSW01 || F_NIVI || F_NBTE
      send_f_energy_condition();
    #endif
    #if F_NBTE
      evaluate_faceplate_buttons();
      evaluate_faceplate_uart();
      check_faceplate_buttons_queue();
      check_faceplate_watchdog();
      #if CUSTOM_MONITORING_CC
        send_custom_info_cc();
      #endif
      check_nbt_cc_queue();
      #if F_NBTE_CPS_VIN
        send_nbt_vin_request();
      #endif
      // send_f_standstill_status();                                                                                                  // Used by EGS?
    #endif
    #if ASD89_RAD_ON
      check_radon_queue();
    #endif
    check_hazards_queue();
    send_mdrive_status_message();                                                                                                   // Send this message with to populate the fields in iDrive.
    if (eeprom_update_timer >= 30000) {                                                                                             // Periodically update EEPROM.
      update_data_in_eeprom();
    }

    if (terminal_r) {
      check_teensy_cpu_clock();                                                                                                     // Dynamically scale clock with temperature to extend lifetime.
      #if INTERMITTENT_WIPERS
        check_intermittent_wipers();
      #endif
      #if F_NIVI || F_NBTE
        send_f_brightness_status();
        send_f_object_warning_coord();
      #endif
      #if F_NBTE
        send_f_throttle_pedal();
      #endif
    }

    if (ignition) {
      check_dsc_queue();
      check_console_buttons();
      if (veh_mode_timer >= 500) {
        send_power_mode();                                                                                                          // state_spt request from DME.   
        #if SERVOTRONIC_SVT70
          send_servotronic_message();
        #endif
        veh_mode_timer = 0;
      }
      #if AUTO_SEAT_HEATING
        check_seatheating_queue();
      #endif
      #if AUTO_STEERING_HEATER
        evaluate_steering_heating_request();
      #endif
      #if HDC || FAKE_MSA
        check_ihk_buttons_cc_queue();
        #if HDC
          check_hdc_queue();
        #endif
        #if FAKE_MSA
          send_msa_status();
        #endif
      #endif
      #if PDC_AUTO_OFF
        check_pdc_button_queue();
      #endif
      #if F_NIVI || X_VIEW
        request_vehicle_pitch_roll_angle();
        #if X_VIEW
          send_f_xview_pitch_angle();
        #endif
        send_f_road_incline();
      #endif
      #if F_NIVI || F_NBTE
        send_f_powertrain_2_status();
        #if F_NBTE
          #if CUSTOM_MONITORING_CC
            send_dme_boost_request();
          #endif
          send_f_driving_dynamics_switch_evo();
        #endif
      #endif
      #if TRSVC70
        evaluate_trsvc_watchdog();
      #endif
    } else {
      if (vehicle_awake_timer >= 2000) {
        vehicle_awake = false;                                                                                                      // Vehicle must now be asleep. Stop monitoring.
        serial_log("Vehicle Sleeping.", 2);
        deactivate_optional_transceivers();
        scale_cpu_speed();
        reset_sleep_variables();
      }
    }
  }


/***********************************************************************************************************************************************************************************************************************************************
  KCAN2 reception
***********************************************************************************************************************************************************************************************************************************************/

  #if F_NBTE
    if (!digitalRead(MCP2515_INT_PIN)) {
      KCAN2.readMsgBuf(&k2rxId, &k2len, k2rxBuf);
      k_msg = make_msg_buf(k2rxId, k2len, k2rxBuf);
      #if defined(USB_TRIPLE_SERIAL)
        if (millis() >= 2000 && SerialUSB2.dtr()) {
          if (slcan_bus == 2) {
            xfer_can2tty(k_msg);
          }
        }
      #endif
      process_kcan2_message();
    }
  #endif


/***********************************************************************************************************************************************************************************************************************************************
  KCAN reception
***********************************************************************************************************************************************************************************************************************************************/

  if (KCAN.read(k_msg)) {
    check_vehicle_awake();                                                                                                          // Wake-up whenever there's any activity on KCAN.
    #if defined(USB_TRIPLE_SERIAL)
      if (millis() >= 2000 && SerialUSB2.dtr()) {
        if (slcan_bus == 1) {
          xfer_can2tty(k_msg);
        }
      }
      #endif
    process_kcan_message();
  }


/***********************************************************************************************************************************************************************************************************************************************
  PTCAN reception
***********************************************************************************************************************************************************************************************************************************************/
  
  if (vehicle_awake) {
    if (PTCAN.read(pt_msg)) {                                                                                                       // Read data.
      if (ptcan_handlers[pt_msg.id] != NULL) {
        ptcan_handlers[pt_msg.id]();                                                                                                // Use the pre-cached function call for this message ID.
      }
    }


/***********************************************************************************************************************************************************************************************************************************************
  DCAN reception and processing
***********************************************************************************************************************************************************************************************************************************************/
    
    if (DCAN.read(d_msg)) {
      // Per filter(s), only 6F1 will be received by the DCAN transceiver.
      if (clearing_dtcs) {}                                                                                                         // Ignore 6F1s while this module is clearing DTCs.

      // MHD monitoring exceptions:
      // else if (d_msg.buf[0] == 0x12){
      //   if ((d_msg.buf[3] == 0x34 && d_msg.buf[4] == 0x80)) {                                                                     // SID 34h requestDownload Service
      //     disable_diag_transmit_jobs();
      //   }
      // }
      // else if (d_msg.buf[0] == 0x60 && (d_msg.buf[1] == 0x30 || d_msg.buf[1] == 3)){}
      // else if (d_msg.buf[0] == 0x40 && (d_msg.buf[1] == 0x30 || d_msg.buf[1] == 3)){}
      // End MHD exceptions.

      #if SERVOTRONIC_SVT70
      else if (d_msg.buf[0] == 0xEF && d_msg.buf[1] == 2 && d_msg.buf[2] == 0x1A                                                   // UIF read or ISTA/D module identification.
                && (d_msg.buf[3] == 0x80 || d_msg.buf[3] == 0x86)) {
        serial_log("UIF being read. Skipping SVT.", 2);
        uif_read = true;
      }

      else if (d_msg.buf[0] == 0xE) {                                                                                               // SVT_70 address is 0xE.
        if (uif_read) {
          if (d_msg.buf[1] == 0x30) {
            // Ignore this KWP continue message as the JBE forwards it anyway.
            uif_read = false;
          }
        } else {
          ptcan_write_msg(d_msg);                                                                                                   // Forward Diagnostic requests to the SVT module from DCAN to PTCAN.
          disable_diag_transmit_jobs();                                                                                             // Deactivate other 6F1 jobs now that the SVT is being diagnosed.
        }
      } 
      #endif

      #if F_VSW01
      else if (d_msg.buf[0] == 0x48) {                                                                                              // F_VSW01 address is 0x48
        kcan_write_msg(d_msg);
        disable_diag_transmit_jobs();                                                                                               // Deactivate other 6F1 jobs now that the VSW is being diagnosed.
      }
      #endif

      #if F_NBTE
      else if (d_msg.buf[0] == 0x35) {                                                                                              // TBX address is 0x35.
        kcan2_write_msg(d_msg);
        disable_diag_transmit_jobs();
      }
      #endif

      #if F_NIVI
      else if (d_msg.buf[0] == 0x57) {                                                                                              // NIVI address is 0x57.
        kcan2_write_msg(d_msg);
        disable_diag_transmit_jobs();
      }
      #endif

      else {
        disable_diag_transmit_jobs();                                                                                               // Implement a check so as to not interfere with other DCAN jobs sent to the car by an OBD tool.    
      }
      diag_deactivate_timer = 0;
    }
  }
  
/**********************************************************************************************************************************************************************************************************************************************/

  #if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)                                                                        // Check if debug printout interface is enabled.
    if (millis() >= 2000) {
      if (SerialUSB1.dtr()) {
        if (!debug_print_connected) {
          debug_print_connected = true;
          debug_print_timer = 500;
          serial_log("Debug printout interface connected.", 0);
        }
        if (debug_print_timer >= 500) {
          print_current_state(SerialUSB1);                                                                                          // Print program status to the second Serial port if open.
          debug_print_timer = 0;
        }
      } else {
        if (debug_print_connected) {
          debug_print_connected = false;
          serial_log("Debug printout interface disconnected.", 0);
        }
      }
    }
  #endif

  loop_timer = micros();
  while (Serial.available()) {
    serial_command_interpreter();
  }
  check_serial_diag_actions();
  
  wdt.feed();                                                                                                                       // Reset the watchdog timer.
}


/***********************************************************************************************************************************************************************************************************************************************
  PTCAN processing
***********************************************************************************************************************************************************************************************************************************************/

void process_ptcan_1A0(void) {
  evaluate_real_speed();
  evaluate_vehicle_moving();
  #if F_NBTE
    send_f_lateral_acceleration();
  #endif
  #if F_NIVI || F_NBTE
    send_f_longitudinal_acceleration();
    send_f_speed_status();
    send_f_yaw_rate_chassis();                                                                                                      // Equivalent to the old internal Gyro.
  #endif
  #if F_NBTE && SERVOTRONIC_SVT70
    send_servotronic_vehicle_speed();
  #endif
}


void process_ptcan_C4(void) {
  evaluate_steering_angle();
  #if FRONT_FOG_CORNER
    evaluate_corner_fog_activation();
  #endif
  #if F_NIVI || F_NBTE
    send_f_steering_angle();
  #endif
}


void process_ptcan_60E(void) {
  if (!uif_read) {
    dcan_write_msg(pt_msg);
  }
}


/***********************************************************************************************************************************************************************************************************************************************
  KCAN processing
***********************************************************************************************************************************************************************************************************************************************/

void process_kcan_message(void) {
  #if F_NBTE
    if (kcan_to_kcan2_forward_filter_list[k_msg.id]){
      kcan2_write_msg(k_msg);                                                                                                       // Write filtered messages from the car to the KCAN2 network.
    }
  #endif

  #if TRSVC70
    if (k_msg.id == 0x486) {
      reset_trsvc_watchdog();
    }
  #endif

  if (k_msg.id >= 0x480 && k_msg.id <= 0x4FF) {                                                                                     // BN2000 OSEK NM IDs (not all).
    #if F_VSW01 || F_NIVI || F_NBTE
      if (k_msg.id == 0x4E0) {                                                                                                      // KOMBI network management. Sent when KOMBI is ON, cycle time 500ms.
        if (!kl30g_cutoff_imminent) {
          send_f_kombi_network_management();
        }
      }
      #if F_NBTE
        else if (k_msg.id == 0x480) {                                                                                               // JBE NM message. Cycle time 1s. This is slower than BN2010's 640ms but less than timeout (2s).
          if (!kl30g_cutoff_imminent) {
            kcan2_write_msg(f_zgw_network_management_buf);
          }
        }

        if (kcan2_mode == MCP_NORMAL) {                                                                                             // Only participate in NM if the KNCA2 bus is active.
          if (k_msg.buf[0] == 0x62) {                                                                                               // HU is registered and its previous neighbour (between 0x480 and 0x4E1 inclusive) requested NM status.
            hu_bn2000_nm_initialized = true;
            convert_f_nbt_network_management();
            hu_bn2000_nm_timer = 0;
          } else if (hu_bn2000_nm_timer >= 3000) {                                                                                  // Registration/timeout handled here. Timeout will occur when nodes appear/disappear.
            hu_bn2000_nm_initialized = false;                                                                                       // If PDC or the driver's seat module are missing this system will fail!
            convert_f_nbt_network_management();
            hu_bn2000_nm_timer = 2500;                                                                                              // Retry in 500ms.
          }
        }
      #endif
    #endif
  } else if (kcan_handlers[k_msg.id] != NULL) {
    kcan_handlers[k_msg.id]();                                                                                                      // Use the pre-cached function call for this message ID.
  }
}


void process_kcan_AA(void) {
  if (ignition) {
    evaluate_engine_status();
    #if CONTROL_SHIFTLIGHTS
      evaluate_shiftlight_display();
    #endif
    #if LAUNCH_CONTROL_INDICATOR
      evaluate_lc_display();
    #endif
    #if EXHAUST_FLAP_CONTROL
      control_exhaust_flap_rpm();
    #endif
  }
}


void process_kcan_A8(void) {
  if (ignition) {
    send_nbt_sport_displays_data(false);
    send_f_torque_1();                                                                                                              // Visible using AAIdrive.
  }
}


void process_kcan_21A(void) {
  if (ignition) {
    #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER
      evaluate_fog_status();
    #endif
    #if FRONT_FOG_CORNER
      evaluate_dipped_beam_status();
    #endif
    #if DIM_DRL
      evaluate_drl_status();
    #endif
  }
}


void process_kcan_23A(void) {
  evaluate_key_number_remote();                                                                                                     // Get the key number first (for CKM states).
  evaluate_remote_button();
}


void process_kcan_2F7(void) {
  evaluate_speed_unit();
  evaluate_temperature_unit();
  #if F_NBTE
    convert_f_units(false);
  #endif
}


void process_kcan_34A(void) {
  hu_application_watchdog = 0;
  #if DOOR_VOLUME 
    send_initial_volume_cic();
  #endif
  #if ASD89_MDRIVE
    initialize_asd_mdrive();
  #endif
}


void process_kcan_3AF(void) {
  if (ignition) {
    evaluate_pdc_bus_status();
    #if F_NBTE
      send_f_pdc_function_status(false);
    #endif
  }
}


void process_kcan_3B0(void) {
  if (ignition) {
    evaluate_reverse_gear_status();
    #if REVERSE_BEEP
      evaluate_reverse_beep();
    #endif
  }
}


void process_kcan_648(void) {
  dcan_write_msg(k_msg);
}


void process_kcan_672(void) {
  #if AUTO_MIRROR_FOLD 
    evaluate_mirror_fold_status();
  #endif
  #if FRONT_FOG_CORNER
    evaluate_ahl_flc_status();
  #endif
}


/***********************************************************************************************************************************************************************************************************************************************
  KCAN2 processing - Relevant messages sent by modules on KCAN2 to be used by this module follow
***********************************************************************************************************************************************************************************************************************************************/

void process_kcan2_message() {
  if (kcan2_handlers[k_msg.id] != NULL) {
    kcan2_handlers[k_msg.id]();                                                                                                     // Use the pre-cached function call for this message ID.
  }

  if (terminal_r) {
    if (kcan2_to_kcan_forward_filter_list[k_msg.id]){
      kcan_write_msg(k_msg);                                                                                                        // Write filtered messages from KCAN2 to the car.
    }
  } else {                                                                                                                          // Outside of Terminal R, messages have to be filtered else they will wake the car.
    if (k_msg.id == 0x5E3) {
      if (k_msg.buf[0] == 0x26 && k_msg.buf[1] == 0x88) {
        if (k_msg.buf[3] == 1) {                                                                                                    // The HU was activated using the faceplate power button.
          if (!hu_ent_mode) {
            serial_log("HU now running in ENT_MODE.", 2);
            hu_ent_mode = true;
          }
        } else if (k_msg.buf[3] == 0) {
          if (hu_ent_mode) {
            serial_log("HU no longer in ENT_MODE.", 2);
            hu_bn2000_bus_sleep_ready_timer = 0;                                                                                    // Exiting ENT_MODE should reset this timer.
            hu_ent_mode = false;
          }
        }
      }
    }
  }
}


void process_kcan2_34A(void) {
  hu_application_watchdog = 0;                                                                                                      // Since this message is created by HU-OMAP, it means the HU HMI application layer is alive.
  #if ASD89_MDRIVE
    initialize_asd_mdrive();
  #endif
  #if ASD89_RAD_ON
    initialize_asd_rad_on();
  #endif
}


void process_kcan2_635(void) {
  dcan_write_msg(k_msg);
}


void process_kcan2_657(void) {
  ptcan_write_msg(k_msg);
}


void process_kcan2_663(void) {
  #if F_NBTE_CPS_VIN
    evaluate_nbt_vin_response();
  #endif
  #if DOOR_VOLUME
    evaluate_audio_volume_nbt();
  #endif
  show_mdrive_settings_screen_evo();
  #if X_VIEW
    show_xview_screen_b_c();
  #endif
}
