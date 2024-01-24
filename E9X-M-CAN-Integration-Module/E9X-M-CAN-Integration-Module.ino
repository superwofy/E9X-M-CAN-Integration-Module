#include "E9X-M-CAN-Integration-Module.h"


void setup() {
  if (F_CPU_ACTUAL != MEDIUM_UNDERCLOCK) {                                                                                          // Prevent accidental overclocks/underclocks. Remove if needed.
    serial_log("CPU clock is not set to startup value, correcting.", 2);
    set_arm_clock(MEDIUM_UNDERCLOCK);
  }
  initialize_watchdog();                                                                                                            // systick: 880 +/- 20 μs is when this function completes. If clock speed is wrong, this takes longer.
  configure_flexcan();                                                                                                              // systick: 1.25 +/- 0.05 ms is when the CAN subsystem is fully ready.
  configure_mcp2515();                                                                                                              // Starting the MCP2515 takes around 2.2 ms!
  configure_IO();
  activate_usb(0);                                                                                                                  // This code ensures compatibility with unmodified Teensy cores since USB init will work anyway.
  read_initialize_eeprom();                                                                                                         // systick: 1.80 +/- 0.01 ms is when the EEPROM is read. If EEPROM is corrupt, this takes longer.
  update_mdrive_can_message();
  cache_can_message_buffers();
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Setup complete at systick: %ld ms.", millis());
    serial_log(serial_debug_string, 2);                                                                                             // systick: 1.90 ms, setup() is complete.
  #endif
}


void loop() {
/***********************************************************************************************************************************************************************************************************************************************
  General section.
***********************************************************************************************************************************************************************************************************************************************/

  check_teensy_cpu_temp();                                                                                                          // Monitor processor temperature.

  if (vehicle_awake) {
    #if EXHAUST_FLAP_CONTROL
      control_exhaust_flap_user();
    #endif
    check_diag_transmit_status();
    #if DOOR_VOLUME
      check_idrive_queue();
      send_volume_request_periodic();
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
    check_idrive_alive_monitor();
    #if F_ZBE_KCAN1 || F_VSW01 || F_NIVI || F_NBT
      send_f_energy_condition();
    #endif
    #if F_NBT
      send_f_oil_level();
      check_faceplate_buttons_queue();
      check_nbt_cc_queue();
      #if F_NBT_VIN_PATCH
        send_nbt_vin_request();
      #endif
    #endif
    check_hazards_queue();
    check_can_resend_queues();

    if (terminal_r) {
      check_teensy_cpu_clock();                                                                                                     // Dynamically scale clock with temperature to extend lifetime.
      #if INTERMITTENT_WIPERS
        check_intermittent_wipers();
      #endif
      #if F_NIVI || F_NBT
        send_f_brightness_status();
      #endif
      if (eeprom_update_timer >= 60000) {                                                                                           // Periodically update EEPROM.
        if (eeprom_unsaved) {
          update_data_in_eeprom();
        }
        eeprom_update_timer = 0;
      }
    }

    if (ignition) {
      check_power_led_state();                                                                                                      // This delay function syncs the power button LED with EDC button LEDS.
      check_dsc_queue();
      check_console_buttons();
      send_mdrive_alive_message(10000);
      #if AUTO_SEAT_HEATING
        check_seatheating_queue();
      #endif
      #if AUTO_STEERING_HEATER
        evaluate_steering_heating_request();
      #endif
      #if HDC || FAKE_MSA
        check_ihk_buttons_cc_queue();
      #endif
      #if HDC
        check_hdc_queue();
      #endif
      #if PDC_AUTO_OFF
        check_pdc_button_queue();
      #endif
      #if F_NIVI || F_NBT
        send_f_powertrain_2_status();
        send_f_standstill_status();
        #if F_NBT
          send_f_driving_dynamics_switch();
        #endif
      #endif
      #if F_NIVI
        request_vehicle_tilt_angle();
      #endif
    } else {
      if (vehicle_awake_timer >= 2000) {
        vehicle_awake = false;                                                                                                      // Vehicle must now be asleep. Stop monitoring .
        serial_log("Vehicle Sleeping.", 0);
        toggle_transceiver_standby(true);
        scale_cpu_speed();
        reset_sleep_variables();
      } else {                                                                                                                      // Ignition OFF, Terminal R ON/OFF, vehicle_awake.
        send_mdrive_alive_message(15000);                                                                                           // Send this message with Terminal R to populate the fields in iDrive.
      }
    }
  }


/***********************************************************************************************************************************************************************************************************************************************
  K-CAN2 section.
***********************************************************************************************************************************************************************************************************************************************/
  #if F_NBT
    if (!digitalRead(MCP2515_INT_PIN)) {
      KCAN2.readMsgBuf(&k2rxId, &k2len, k2rxBuf);

      uint8_t nbt_to_car[k2len];
      for (uint8_t i = 0; i < k2len; i++) {
        nbt_to_car[i] = k2rxBuf[i];
      }
      
      k_msg = make_msg_buf(k2rxId, k2len, nbt_to_car);
      process_kcan2_message();
    }
  #endif


/***********************************************************************************************************************************************************************************************************************************************
  K-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if (KCAN.read(k_msg)) {

    #if F_NBT
      if (k_msg.id != 0x2F7) {                                                                                                      // Skip the units message as this requires further processing.
        kcan2_write_msg(k_msg);                                                                                                     // Write messages from the car to the NBT.
      }
    #endif

    if (ignition) {
      if (k_msg.id == 0xAA) {                                                                                                       // Monitor 0xAA (rpm/throttle status). 0xAA also contains torq_dvch - torque request, driver.
        evaluate_engine_status();

        // Shiftlights and LC depend on RPM. This message is cycled every 100ms, it makes sense to run the calculations now.
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

      #if HDC
      else if (k_msg.id == 0x193) {                                                                                                 // Monitor state of cruise control
        evaluate_cruise_control_status();
      }
      else if (k_msg.id == 0x31A) {                                                                                                 // Received HDC button press from IHKA.
        evaluate_hdc_button();
      }
      #endif

      #if FAKE_MSA || MSA_RVC
      else if (k_msg.id == 0x195) {                                                                                                 // Monitor state of MSA button
        evaluate_msa_button();
      }
      #endif

      else if (k_msg.id == 0x1B4) {                                                                                                 // Monitor if the car is stationary/moving
        evaluate_vehicle_moving();
        evaluate_indicated_speed();
      }

      #if REVERSE_BEEP || DOOR_VOLUME
      else if (k_msg.id == 0x1C6) {                                                                                                 // Monitor PDC warning.
        evaluate_pdc_warning();
      }
      #endif

      else if (k_msg.id == 0x1D0) {
        evaluate_engine_temperature();
      }

      #if MIRROR_UNDIM
      else if (k_msg.id == 0x1EE) {
        evaluate_indicator_stalk();
      }
      #endif

      else if (k_msg.id == 0x1F6) {                                                                                                 // Monitor indicator status
        evaluate_indicator_status_dim();
      }

      #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER || DIM_DRL
      else if (k_msg.id == 0x21A) {                                                                                                 // Light status sent by the FRM.
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
      #endif

      #if AUTO_SEAT_HEATING_PASS
      else if (k_msg.id == 0x22A) {                                                                                                 // Monitor passenger's seat heating.
        evaluate_seat_heating_status();
      }
      #endif

      #if AUTO_SEAT_HEATING
      else if (k_msg.id == 0x232) {                                                                                                 // Monitor driver's seat heating.
        evaluate_seat_heating_status();
      }
      #endif

      #if F_NBT
      else if (k_msg.id == 0x23D) {
        send_climate_popup_acknowledge();
      }

      else if (k_msg.id == 0x2E6) {                                                                                                 // Air distribution status message.
        evaluate_ihka_auto_state();
      }
      #endif

      #if F_NBT || F_NIVI || MIRROR_UNDIM || FRONT_FOG_CORNER
      else if (k_msg.id == 0x314) {                                                                                                 // RLS light status.
        evaluate_rls_light_status();
      }
      #endif

      #if MSA_RVC
      else if (k_msg.id == 0x317) {                                                                                                 // Received PDC button press from IHKA.
        evaluate_pdc_button();
      }
      #endif

      #if PDC_AUTO_OFF
      else if (k_msg.id == 0x34F) {
        evaluate_handbrake_status();
      }
      #endif

      #if AUTO_TOW_VIEW_RVC
      else if (k_msg.id == 0x36D) {
        evaluate_pdc_distance();
      }
      
      #if !F_NBT
      else if (k_msg.id == 0x38F) {                                                                                                 // Camera settings request from iDrive.
        store_rvc_settings_idrive();
      }
      #endif

      else if (k_msg.id == 0x39B) {                                                                                                 // Camera settings/acknowledge from TRSVC.
        store_rvc_settings_trsvc();
      }
      #endif

      #if MSA_RVC || PDC_AUTO_OFF || AUTO_TOW_VIEW_RVC || F_NBT
      else if (k_msg.id == 0x3AF) {                                                                                                 // Monitor PDC bus status.
        evaluate_pdc_bus_status();
        #if F_NBT
          send_f_pdc_function_status(false);
        #endif
      }
      #endif

      #if !F_NBT
      else if (k_msg.id == 0x3A8) {                                                                                                 // Received POWER M Key settings from iDrive.
        update_dme_power_ckm();
      }
      #endif

      else if (k_msg.id == 0x3B0) {                                                                                                 // Monitor reverse status.
        evaluate_reverse_gear_status();
        #if REVERSE_BEEP
          evaluate_reverse_beep();
        #endif
      }
     
      #if !F_NBT
      else if (k_msg.id == 0x3CA) {                                                                                                 // Receive settings from iDrive (BN2000).
        update_mdrive_message_settings_cic();
      }
      #endif

      #if F_NIVI
      else if (k_msg.id == 0x650) {                                                                                                 // SINE is at address 0x50.
        evaluate_vehicle_tilt_angle();
      }
      #endif
    }

    if (k_msg.id == 0xEA) {                                                                                                         // Driver's door status.
      evaluate_drivers_door_status();
    }

    #if F_NBT
    else if (k_msg.id == 0xC0) {                                                                                                    // JBE alive message. Cycle time 200ms.
      send_f_zgw_network_management();
    }
    #endif

    else if (k_msg.id == 0x130) {                                                                                                   // Monitor terminal, clutch and key number.
      evaluate_terminal_clutch_keyno_status();
    }

    #if F_NBT
    else if (k_msg.id == 0x1A6) {                                                                                                   // Distance message from DSC. Cycle time 100ms.
      send_f_distance_messages();
    }

    else if (k_msg.id == 0x205) {
      evaluate_cc_gong_status();
    }
    #endif

    #if !F_NBT
    else if (k_msg.id == 0x1AA) {                                                                                                   // Time POWER CKM message with iDrive ErgoCommander.
      send_dme_power_ckm();
    }
    #endif

    #if F_NBT_CCC_ZBE
    else if (k_msg.id == 0x1B8) {                                                                                                   // Convert old CCC controller data for NBT.
      convert_zbe1_message();
    }
    #endif

    else if (k_msg.id == 0x23A) {                                                                                                   // Monitor remote function status
      evaluate_key_number_remote();                                                                                                 // Get the key number first (for CKM states).
      evaluate_remote_button();
    }

    #if !F_NBT
    else if (k_msg.id == 0x273) {
      idrive_alive_timer = 0;
      #if F_ZBE_KCAN1
        send_zbe_acknowledge();
      #endif
      #if DOOR_VOLUME 
        send_initial_volume_cic();
      #endif
      #if ASD
        initialize_asd();
      #endif
    }
    #endif

    #if F_NBT
    else if (k_msg.id == 0x2C0) {                                                                                                   // Kombi LCD brightness. Cycle time 10s.
      send_f_kombi_lcd_brightness();
    }
    #endif

    else if (k_msg.id == 0x2CA) {                                                                                                   // Monitor and update ambient temperature.
      evaluate_ambient_temperature();
    }

    #if AUTO_SEAT_HEATING_PASS
    else if (k_msg.id == 0x2FA) {                                                                                                   // Monitor and update seat status
      evaluate_passenger_seat_status();
    } 
    #endif

    else if (k_msg.id == 0x2F7) {
      evaluate_speed_units();
      #if F_NBT
        convert_f_units(false);
      #endif
    }

    #if DOOR_VOLUME || AUTO_MIRROR_FOLD || IMMOBILIZER_SEQ || HOOD_OPEN_GONG
    else if (k_msg.id == 0x2FC) {
      evaluate_door_status();
    }
    #endif

    #if F_NBT
    else if (k_msg.id == 0x31D) {
      send_f_ftm_status();
    }

    else if (k_msg.id == 0x381) {                                                                                                   // Store the Oil level for use with NBT. 10s cycle time.
      e_oil_level = k_msg.buf[0];
      f_oil_level_timer = 500;
    }
    #endif

    else if (k_msg.id == 0x3BD) {                                                                                                   // Received consumer shutdown message from FRM.
      evaluate_frm_consumer_shutdown();
    }

    else if (k_msg.id == 0x3D7) {                                                                                                   // Received CKM setting status for door locks.
      evaluate_door_lock_ckm();
    }

    #if COMFORT_EXIT
    else if (k_msg.id == 0x3DB) {                                                                                                   // Received CKM setting status for driver's seat.
      evaluate_dr_seat_ckm();
    }
    #endif

    #if DIM_DRL
    else if (k_msg.id == 0x3DD) {                                                                                                   // Received CKM setting status for lights.
      evaluate_drl_ckm();
    }
    #endif

    #if F_NBT
    else if (k_msg.id == 0x3DF) {                                                                                                   // Received CKM setting for AUTO blower speed.
      evaluate_ihka_auto_ckm();
    }
    #endif

    #if F_ZBE_KCAN1 || F_VSW01 || F_NIVI || F_NBT
    else if (k_msg.id == 0x4E0) {                                                                                                   // Kombi network management.
      send_f_kombi_network_management();
    }
    #endif

    #if DOOR_VOLUME
    else if (k_msg.id == 0x5C0) {
      disable_door_open_ignition_on_cc();
    }
    #endif

    #if DOOR_VOLUME && !F_NBT
    else if (k_msg.id == 0x663) {                                                                                                   // iDrive diagnostic response.
      #if DOOR_VOLUME
        evaluate_audio_volume_cic();
      #endif
    }
    #endif

    #if DEBUG_MODE
    else if (k_msg.id == 0x640) {
      evaluate_power_down_response();
    }
    #endif

    #if F_VSW01
    else if (k_msg.id == 0x648) {
      dcan_write_msg(k_msg);
    }
    #endif

    #if AUTO_MIRROR_FOLD || FRONT_FOG_CORNER
    else if (k_msg.id == 0x672) {
      #if AUTO_MIRROR_FOLD 
        evaluate_mirror_fold_status();
      #endif
      #if FRONT_FOG_CORNER
        evaluate_ahl_flc_status();
      #endif
    }
    #endif
  }


/***********************************************************************************************************************************************************************************************************************************************
  PT-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if (vehicle_awake) {
    if (PTCAN.read(pt_msg)) {                                                                                                       // Read data.
      if (pt_msg.id == 0x3B4) {                                                                                                     // Monitor battery voltage from DME.
        evaluate_battery_voltage();
      }

      #if IMMOBILIZER_SEQ                                                                                                           // We need this button data with ignition off too
      else if (pt_msg.id == 0x1D6) {                                                                                                // A button was pressed on the steering wheel.
        evaluate_m_mfl_button_press();
      }
      #endif

      #if WIPE_AFTER_WASH || INTERMITTENT_WIPERS
      else if (pt_msg.id == 0x2A6) {
        evaluate_wiper_stalk_status();
      }
      #endif
     
      if (ignition) {              
        if (pt_msg.id == 0x315) {                                                                                                   // Monitor EDC status message from the JBE.
          send_power_mode();                                                                                                        // state_spt request from DME.   
          #if SERVOTRONIC_SVT70
            send_servotronic_message();
          #endif
        }

        #if F_NBT
        else if (pt_msg.id == 0xA8) {                                                                                               // Crankshaft torque.
          send_nbt_sport_displays_data();                                                                                           // Normally this is on KCAN at 100ms.
        }
        #endif

        #if FRONT_FOG_CORNER || F_NIVI || F_NBT
        else if (pt_msg.id == 0xC8) {
          evaluate_steering_angle();
          #if FRONT_FOG_CORNER
            evaluate_corner_fog_activation();
          #endif
          #if F_NIVI || F_NBT
            send_f_steering_angle();
          #endif
        }
        #endif

        #if HDC
        else if (pt_msg.id == 0x194) {
          evaluate_cruise_stalk_message();
        }
        #endif

        else if (pt_msg.id == 0x1A0) {
          evaluate_real_speed();
          #if F_NIVI
            send_f_longitudinal_acceleration();
            send_f_lateral_acceleration();
            send_f_road_inclination();
          #endif
          #if F_NIVI || F_NBT
            evaluate_vehicle_direction();
            send_f_yaw_rate();                                                                                                      // Equivalent to Gyro.
            send_f_speed_status();
          #endif
        }

        #if !IMMOBILIZER_SEQ
        else if (pt_msg.id == 0x1D6) {
          evaluate_m_mfl_button_press();
        }
        #endif
      
        #if FTM_INDICATOR
        else if (pt_msg.id == 0x31D) {                                                                                              // FTM initialization is ongoing.
          evaluate_indicate_ftm_status();
        }
        #endif  

        #if CONTROL_SHIFTLIGHTS
        else if (pt_msg.id == 0x332) {                                                                                              // Monitor variable redline broadcast from DME.
          evaluate_update_shiftlight_sync();
        }
        #endif

        #if SERVOTRONIC_SVT70
        else if (pt_msg.id == 0x58E) {                                                                                              // Since the JBE doesn't forward Servotronic errors from SVT70, we have to do it.
          send_svt_kcan_cc_notification();
        }
        else if (pt_msg.id == 0x60E) {                                                                                              // Forward Diagnostic responses from SVT module to DCAN
          if (!uif_read) {
            dcan_write_msg(pt_msg);
          }
        }
        #endif
      }
    }


/***********************************************************************************************************************************************************************************************************************************************
  D-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
    
    if (DCAN.read(d_msg)) {
      if (d_msg.id == 0x6F1) {
        if (clearing_dtcs) {}                                                                                                       // Ignore 6F1s while this module is clearing DTCs.

        // MHD monitoring exceptions:
        else if (d_msg.buf[0] == 0x12){
          if ((d_msg.buf[3] == 0x34 && d_msg.buf[4] == 0x80)) {                                                                     // SID 34h requestDownload Service
            disable_diag_transmit_jobs();
          }
        }
        else if (d_msg.buf[0] == 0x60 && (d_msg.buf[1] == 0x30 || d_msg.buf[1] == 3)){}
        else if (d_msg.buf[0] == 0x40 && (d_msg.buf[1] == 0x30 || d_msg.buf[1] == 3)){}
        // End MHD exceptions.

        #if SERVOTRONIC_SVT70
        // UIF read or ISTA/D module identification:
        else if (d_msg.buf[0] == 0xEF && d_msg.buf[1] == 2 && d_msg.buf[2] == 0x1A 
                  && (d_msg.buf[3] == 0x80 || d_msg.buf[3] == 0x86)) {
          serial_log("UIF being read. Skipping SVT.", 0);
          uif_read = true;
        }

        else if (d_msg.buf[0] == 0xE) {                                                                                             // SVT_70 address is 0xE.
          if (uif_read) {
            if (d_msg.buf[1] == 0x30) {
              // Ignore this KWP continue message as the JBE forwards it anyway.
              uif_read = false;
            }
          } else {
            ptcan_write_msg(d_msg);                                                                                                 // Forward Diagnostic requests to the SVT module from DCAN to PTCAN.
            disable_diag_transmit_jobs();                                                                                           // Deactivate other 6F1 jobs now that the SVT is being diagnosed.
          }
        } 
        #endif

        #if F_VSW01
        else if (d_msg.buf[0] == 0x48) {                                                                                            // F_VSW01 address is 0x48
          kcan_write_msg(d_msg);
          disable_diag_transmit_jobs();                                                                                             // Deactivate other 6F1 jobs now that the VSW is being diagnosed.
        }
        #endif

        #if F_NBT
        else if (d_msg.buf[0] == 0x35) {                                                                                            // TBX address is 0x35.
          kcan2_write_msg(d_msg);
        }
        #endif

        else {
          disable_diag_transmit_jobs();                                                                                             // Implement a check so as to not interfere with other DCAN jobs sent to the car by an OBD tool.    
        }
        diag_deactivate_timer = 0;
      }
    }
  }
  
/**********************************************************************************************************************************************************************************************************************************************/

  #if F_NBT
    if (vehicle_awake) {
      evaluate_faceplate_buttons();
      evaluate_faceplate_uart();
    }
  #endif

  #if DEBUG_MODE
    #if CDC2_STATUS_INTERFACE == 2                                                                                                  // Check if Dual Serial is set.
      if (debug_print_timer >= 500) {
        if (millis() >= 1000) {
          print_current_state(SerialUSB1);                                                                                          // Print program status to the second Serial port.
        }
      }
    #endif

    loop_timer = micros();
    while (Serial.available()) {
      serial_debug_interpreter();
    }
    check_serial_diag_actions();
  #endif
  
  wdt.feed();                                                                                                                       // Reset the watchdog timer.
}


/***********************************************************************************************************************************************************************************************************************************************
  K-CAN2 section. Relevant messages sent by NBT to be used by this module follow and are copied from the K-CAN section...
***********************************************************************************************************************************************************************************************************************************************/
void process_kcan2_message(void) {
  if (ignition) {
    if (k_msg.id == 0x42F) {                                                                                                        // Receive M drive settings from iDrive (BN2010).
      update_mdrive_message_settings_nbt();
      return;                                                                                                                       // This message is only for BN2010.
    }

    else if (k_msg.id == 0x31A) {
      evaluate_f_pdc_function_request();
      return;
    }

    #if AUTO_TOW_VIEW_RVC
    else if (k_msg.id == 0x38F) {                                                                                                   // Camera settings request from iDrive.
      store_rvc_settings_idrive();
    }
    #endif
  }

  if (k_msg.id == 0xBF) { return; }                                                                                                 // Skip touchpad data message. This message is only for BN2010.
  
  #if F_NBT_CCC_ZBE
  else if (k_msg.id == 0x273) {                                                                                                     // NBT tried to initialize a controller.
    send_zbe_acknowledge();                                                                                                         // Without this response, rotation data is ignored.
  }
  #endif

  else if (k_msg.id == 0x291) {
    evaluate_sport_units();
  }

  #if F_VSW01
  else if (k_msg.id == 0x2FB) {                                                                                                     // VSW switch request sent by NBT.
    evaluate_vsw_position_request();
  }
  #endif

  else if (k_msg.id == 0x317) {                                                                                                     // This message is used for the ZBE instead of PDC activation. Causes sporadinc PDC activation. Ignore.
    return;
  }

  else if (k_msg.id == 0x34A) {                                                                                                     // 0x273 does not work the same way on NBT. Use 34A instead. Cycle time 1s.
    idrive_alive_timer = 0;
    #if ASD
      initialize_asd();
    #endif
  }

  #if AUTO_TOW_VIEW_RVC
  else if (k_msg.id == 0x38F) {                                                                                                     // Camera settings request from iDrive.
    store_rvc_settings_idrive();
  }
  #endif

  else if (k_msg.id == 0x635) {                                                                                                     // TBX diagnostic response.
    dcan_write_msg(k_msg);
    return;
  }

  else if (k_msg.id == 0x663) {                                                                                                     // iDrive diagnostic response.
    #if F_NBT_VIN_PATCH
      evaluate_nbt_vin_response();
    #endif
    #if DOOR_VOLUME
      evaluate_audio_volume_nbt();
    #endif
  }

  else if (k_msg.id == 0x6F1) {                                                                                                     // Block some exploits through browser injecting into the network.
    serial_log("KCAN2 network sent a 6F1 message!", 0);
    return;
  }

  if (terminal_r) {                                                                                                                 // No needed data is sent from KCAN2 modules to the car when KL_R is off. This can stop the car from sleeping.
    kcan_write_msg(k_msg);                                                                                                          // Write the message received from KCAN2 to the rest of the car.
  }
}
