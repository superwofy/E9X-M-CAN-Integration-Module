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
  activate_usb();                                                                                                                   // This code ensures compatibility with unmodified Teensy cores since USB init will work anyway.
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
    #if F_VSW01 || F_NIVI || F_NBT
      send_f_energy_condition();
    #endif
    #if F_NBT
      evaluate_faceplate_buttons();
      evaluate_faceplate_uart();
      check_faceplate_buttons_queue();
      #if CUSTOM_MONITORING_CC
        send_custom_info_cc();
      #endif
      check_nbt_cc_queue();
      #if F_NBT_VIN_PATCH
        send_nbt_vin_request();
      #endif
      send_f_standstill_status();
      send_f_throttle();
    #endif
    #if ASD89_RAD_ON
      check_radon_queue();
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
      send_mdrive_alive_message();                                                                                                  // Send this message with Terminal R to populate the fields in iDrive.
    }

    if (ignition) {
      check_dsc_queue();
      check_console_buttons();
      if (veh_mode_timer >= 500) {
        send_power_mode();                                                                                                          // state_spt request from DME.   
        #if SERVOTRONIC_SVT70
          send_servotronic_message();
          #if F_NBT
            send_servotronic_sport_plus();
          #endif
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
      #endif
      #if PDC_AUTO_OFF
        check_pdc_button_queue();
      #endif
      #if F_NIVI || X_VIEW
        request_vehicle_pitch_angle();
        request_vehicle_roll_angle();
        #if X_VIEW
          send_f_xview_pitch_angle();
        #endif
        send_f_road_incline();
      #endif
      #if F_NIVI || F_NBT
        send_f_powertrain_2_status();
        #if F_NBT
          #if CUSTOM_MONITORING_CC
            send_dme_boost_request();
          #endif
          #if F_NBT_EVO6
            send_f_driving_dynamics_switch_evo();
          #else
            send_f_driving_dynamics_switch_nbt();
          #endif
        #endif
      #endif
    } else {
      if (vehicle_awake_timer >= 2000) {
        vehicle_awake = false;                                                                                                      // Vehicle must now be asleep. Stop monitoring.
        serial_log("Vehicle Sleeping.", 0);
        toggle_transceiver_standby(true);
        scale_cpu_speed();
        reset_sleep_variables();
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

    check_vehicle_awake();                                                                                                          // Wake-up whenever there's any activity on KCAN.

    #if F_NBT
      if (k_msg.id != 0xAA &&                                                                                                       // BN2000 engine status and torques.
          k_msg.id != 0xA8 &&
          k_msg.id != 0xC4 &&                                                                                                       // BN2000 steering angle.
          k_msg.id != 0xC8 &&
          k_msg.id != 0x1A0 &&                                                                                                      // BN2000 Speed / BN2010 Gearbox check-control.
          k_msg.id != 0x1B6 &&                                                                                                      // BN2000 Engine heat flow. Unknown in BN2010. It causes NBT EVO to block phone calls.
          k_msg.id != 0x2C0 &&                                                                                                      // BN2000 LCD brightness.
          k_msg.id != 0x2F3 &&                                                                                                      // BN2000 gear shift instruction / BN2010 gyro.
          k_msg.id != 0x2F7 &&                                                                                                      // KOMBI units. Requires further processing.
          k_msg.id != 0x317 &&                                                                                                      // BN2000 PDC button.
          k_msg.id != 0x31D &&                                                                                                      // BN2000 FTM status.
          k_msg.id != 0x35C &&                                                                                                      // Speed warning setting. Requires further processing.
          k_msg.id != 0x3DD &&                                                                                                      // Lights CKM. Requires further processing.
          k_msg.id != 0x336 &&                                                                                                      // CC list display. Requires further processing.
          k_msg.id != 0x338 &&                                                                                                      // CC dialog display. Requires further processing.
          k_msg.id != 0x399 &&                                                                                                      // BN2000 MDrive / BN2010 Status energy voltage.
          k_msg.id != 0x3B3 &&                                                                                                      // DME consumer control. Requires further processing.
          !(k_msg.id >= 0x480 && k_msg.id < 0x580) &&                                                                               // BN2000 NM (incompatible).
          !(k_msg.id >= 0x580 && k_msg.id < 0x5E0) &&                                                                               // BN2000 CCs except KOMBI response.
          !(k_msg.id > 0x5E0 && k_msg.id < 0x6F0)                                                                                   // More BN2000 CCs and KWP/UDS diagnosis responses from various modules.
         ){
        kcan2_write_msg(k_msg);                                                                                                     // Write filtered messages from the car to the NBT.
      }
    #endif

    if (ignition) {
      if (k_msg.id == 0xAA) {                                                                                                       // Monitor 0xAA (rpm/throttle pos). 0xAA also contains torq_dvch - torque request, driver. Cycle time 100ms (KCAN).
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

      #if F_NBT
      else if (k_msg.id == 0xA8) {                                                                                                  // Crankshaft torque.
        send_nbt_sport_displays_data(false);
        send_f_torque_1();                                                                                                          // Visible using AAIdrive.
      }
      #endif

      #if HDC
      else if (k_msg.id == 0x193) {                                                                                                 // Monitor state of cruise control from KOMBI. Sent when changed.
        evaluate_cruise_control_status();
      }
      else if (k_msg.id == 0x31A) {                                                                                                 // Received HDC button press from IHKA. Sent when changed.
        evaluate_hdc_button();
      }
      #endif

      #if FAKE_MSA || MSA_RVC
      else if (k_msg.id == 0x195) {                                                                                                 // Monitor state of MSA button. Sent when changed.
        evaluate_msa_button();
      }
      #endif

      else if (k_msg.id == 0x1B4) {                                                                                                 // Monitor KOMBI status (indicated speed, handbrake). Cycle time 100ms (terminal R ON).
        evaluate_indicated_speed();
      }

      #if REVERSE_BEEP || DOOR_VOLUME
      else if (k_msg.id == 0x1C6) {                                                                                                 // Monitor PDC acoustic warning.
        evaluate_pdc_warning();
      }
      #endif

      #if !IMMOBILIZER_SEQ
      else if (k_msg.id == 0x1D6) {                                                                                                 // MFL (Multi Function Steering Wheel) button status. Cycle time 1s (idle), 100ms (pressed).
        evaluate_m_mfl_button_press();
      }
      #endif

      else if (k_msg.id == 0x1F6) {                                                                                                 // Monitor indicator status. Cycle time 1s. Sent when changed.
        evaluate_indicator_status_dim();
      }

      #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER || DIM_DRL
      else if (k_msg.id == 0x21A) {                                                                                                 // Light status sent by the FRM. Cycle time 5s (idle). Sent when changed.
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
      else if (k_msg.id == 0x22A) {                                                                                                 // Monitor passenger's seat heating. Cycle time 10s (idle), 150ms (change).
        evaluate_seat_heating_status();
      }
      #endif

      #if AUTO_SEAT_HEATING
      else if (k_msg.id == 0x232) {                                                                                                 // Monitor driver's seat heating. Cycle time 10s (idle), 150ms (change).
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

      #if MSA_RVC
      else if (k_msg.id == 0x317) {                                                                                                 // Received PDC button press from IHKA. Sent when changed.
        evaluate_pdc_button();
      }
      #endif

      #if CONTROL_SHIFTLIGHTS
      else if (k_msg.id == 0x332) {                                                                                                 // Monitor variable redline broadcast from DME. Cycle time 1s.
        evaluate_update_shiftlight_sync();
      }
      #endif

      #if PDC_AUTO_OFF
      else if (k_msg.id == 0x34F) {                                                                                                 // Handbrake status sent by JBE. Sent when changed.
        evaluate_handbrake_status();
      }
      #endif

      #if AUTO_TOW_VIEW_RVC
      else if (k_msg.id == 0x36D) {                                                                                                 // Distance status sent by PDC. Sent when active.
        evaluate_pdc_distance();
      }
      
      #if !F_NBT
      else if (k_msg.id == 0x38F) {                                                                                                 // Camera settings request from iDrive. Sent when activating camera and when changed.
        store_rvc_settings_idrive();
      }
      #endif

      else if (k_msg.id == 0x39B) {                                                                                                 // Camera settings/acknowledge from TRSVC. Sent when activating ignition and when changed.
        store_rvc_settings_trsvc();
      }
      #endif

      #if MSA_RVC || PDC_AUTO_OFF || AUTO_TOW_VIEW_RVC || F_NBT
      else if (k_msg.id == 0x3AF) {                                                                                                 // Monitor PDC bus status. Cycle time 2s (idle). Sent when changed.
        evaluate_pdc_bus_status();
        #if F_NBT
          send_f_pdc_function_status(false);
        #endif
      }
      #endif

      #if !F_NBT
      else if (k_msg.id == 0x3A8) {                                                                                                 // Received POWER M Key (CKM) settings from iDrive. Sent when changed.
        update_dme_power_ckm();
      }
      #endif

      else if (k_msg.id == 0x3B0) {                                                                                                 // Monitor reverse status. Cycle time 1s (idle).  
        evaluate_reverse_gear_status();
        #if REVERSE_BEEP
          evaluate_reverse_beep();
        #endif
      }

      #if !F_NBT
      else if (k_msg.id == 0x3CA) {                                                                                                 // Receive settings from iDrive (BN2000). Sent when changed.
        update_mdrive_message_settings_cic();
      }
      #endif

      #if F_NIVI || F_NBT
      else if (k_msg.id == 0x586) {                                                                                                 // TRSVC CCs.
        evaluate_trsvc_cc();
      }

      else if (k_msg.id == 0x650) {                                                                                                 // SINE diagnostic responses. SINE is at address 0x50.
        evaluate_vehicle_pitch_angle();
        evaluate_vehicle_roll_angle();
      }
      #endif
    }

    if (k_msg.id == 0xEA) {                                                                                                         // Driver's door lock status.
      evaluate_drivers_door_lock_status();
    }

    else if (k_msg.id == 0x130) {                                                                                                   // Monitor terminal, clutch and key number. Cycle time 100ms.
      evaluate_terminal_clutch_keyno_status();
    }

    #if F_NBT
    else if (k_msg.id == 0x1A6) {                                                                                                   // Distance message from DSC. Cycle time 100ms.
      send_f_distance_messages();
    }
    #endif

    #if !F_NBT
    else if (k_msg.id == 0x1AA) {                                                                                                   // Time POWER CKM message with iDrive ErgoCommander. Sent at boot time and Terminal R cycling.
      send_dme_power_ckm();
    }
    #endif

    #if F_NBT_CCC_ZBE
    else if (k_msg.id == 0x1B8) {                                                                                                   // Convert old CCC controller data for NBT.
      convert_zbe1_message();
    }
    #endif

    else if (k_msg.id == 0x1D0) {                                                                                                   // Engine temperatures and ambient pressure.
      evaluate_engine_data();
    }

    #if IMMOBILIZER_SEQ                                                                                                             // We need this button data with ignition off too.
    if (k_msg.id == 0x1D6) {                                                                                                        // A button was pressed on the steering wheel.
      evaluate_m_mfl_button_press();
    }
    #endif

    #if MIRROR_UNDIM || F_NBT
    else if (k_msg.id == 0x1EE) {                                                                                                   // Indicator stalk status from FRM (KCAN only). Read-only message whose state can't be changed.
      evaluate_indicator_stalk();
      #if F_NBT
        evaluate_high_beam_stalk();
      #endif
    }
    #endif

    else if (k_msg.id == 0x205) {
      evaluate_cc_gong_status();
    }

    else if (k_msg.id == 0x23A) {                                                                                                   // Monitor remote function status. Sent 3x when changed.
      evaluate_key_number_remote();                                                                                                 // Get the key number first (for CKM states).
      evaluate_remote_button();
    }

    #if WIPE_AFTER_WASH || INTERMITTENT_WIPERS
    else if (k_msg.id == 0x2A6) {                                                                                                   // Wiper stalk status from SZL. Cycle time 1s (idle).
      evaluate_wiper_stalk_status();
    }
    #endif

    #if F_NBT
    else if (k_msg.id == 0x2C0) {                                                                                                   // Kombi LCD brightness. Cycle time 10s.
      send_f_kombi_lcd_brightness();
    }
    #endif

    else if (k_msg.id == 0x2CA) {                                                                                                   // Monitor and update ambient temperature. Cycle time 1s.
      evaluate_ambient_temperature();
    }

    #if F_VSW01
    else if (k_msg.id == 0x2FD) {                                                                                                   // VSW actual status.
      vsw_current_input = k_msg.buf[0];
    }
    #endif

    #if AUTO_SEAT_HEATING_PASS
    else if (k_msg.id == 0x2FA) {                                                                                                   // Seat occupancy and belt status. Cycle time 5s.
      evaluate_passenger_seat_status();
    }
    #endif

    else if (k_msg.id == 0x2F7) {                                                                                                   // Units from KOMBI. Sent 3x on Terminal R. Sent when changed.
      evaluate_speed_unit();
      evaluate_temperature_unit();
      #if F_NBT
        convert_f_units(false);
      #endif
    }

    #if DOOR_VOLUME || AUTO_MIRROR_FOLD || IMMOBILIZER_SEQ || HOOD_OPEN_GONG
    else if (k_msg.id == 0x2FC) {                                                                                                   // Door, hood status sent by CAS. Cycle time 5s. Sent when changed.
      evaluate_door_status();
    }
    #endif

    #if F_NBT || F_NIVI || MIRROR_UNDIM || FRONT_FOG_CORNER
    else if (k_msg.id == 0x314) {                                                                                                   // RLS light status. Cycle time 3s. Sent when changed.
      evaluate_rls_light_status();
    }
    #endif

    #if FTM_INDICATOR || F_NBT
    else if (k_msg.id == 0x31D) {                                                                                                   // FTM status broadcast by DSC. Cycle time 5s (idle). Sent when changed.
      #if FTM_INDICATOR
        evaluate_indicate_ftm_status();
      #endif
      #if F_NBT
        send_f_ftm_status();
      #endif
    }

    else if (k_msg.id == 0x336) {
      process_bn2000_cc_display();
    }

    else if (k_msg.id == 0x338) {
      process_bn2000_cc_dialog();
    }
    #endif

    #if !F_NBT
    else if (k_msg.id == 0x34A) {                                                                                                   // GPS position, appears consistently regardless of Terminal status. Cycle time 1s.
      idrive_alive_timer = 0;
      #if DOOR_VOLUME 
        send_initial_volume_cic();
      #endif
      #if ASD89_MDRIVE
        initialize_asd();
      #endif
    }
    #endif

    #if F_NBT
    else if (k_msg.id == 0x35C) {                                                                                                   // EVO does not support settings less than 15kph.
      evaluate_speed_warning_status();
    }

    else if (k_msg.id == 0x3B3) {
      evaluate_consumer_control(); 
    }
    #endif

    else if (k_msg.id == 0x3B4) {                                                                                                   // Monitor battery voltage from DME.
      evaluate_battery_voltage();
    }

    else if (k_msg.id == 0x3BD) {                                                                                                   // Received consumer shutdown message from FRM. Cycle time 5s (idle). Sent when changed.
      evaluate_frm_consumer_shutdown();
    }

    else if (k_msg.id == 0x3BE) {                                                                                                   // Received terminal follow-up time from CAS. Cycle time 10s depending on bus activity.
      evaluate_terminal_followup();
    }

    else if (k_msg.id == 0x3D7) {                                                                                                   // Received CKM setting status for door locks. Sent when changed.
      evaluate_door_lock_ckm();
    }

    #if COMFORT_EXIT
    else if (k_msg.id == 0x3DB) {                                                                                                   // Received CKM setting status for driver's seat. Sent when changed.
      evaluate_dr_seat_ckm();
    }
    #endif

    #if DIM_DRL
    else if (k_msg.id == 0x3DD) {                                                                                                   // Received CKM setting status for lights. Sent when changed.
      evaluate_drl_ckm();
      fix_f_lights_ckm();
    }
    #endif

    #if F_NBT
    else if (k_msg.id == 0x3DF) {                                                                                                   // Received CKM setting for AUTO blower speed.
      evaluate_ihka_auto_ckm();
    }
    #endif

    else if (k_msg.id == 0x3F1) {                                                                                                   // Received CKM setting for High beam assistant.
      evaluate_hba_ckm();
    }
    
    #if F_VSW01 || F_NIVI || F_NBT
    else if (k_msg.id >= 0x480 && k_msg.id <= 0x4FF) {                                                                              // BN2000 OSEK NM IDs (not all).
      if (k_msg.id == 0x4E0) {                                                                                                      // KOMBI network management. Sent when KOMBI is ON, cycle time 500ms.
        send_f_kombi_network_management();
      }
      #if F_NBT
        else if (k_msg.id == 0x480) {                                                                                               // JBE NM message. Cycle time 1s. This is slower than BN2010's 640ms but less than timeout (2s).
          kcan2_write_msg(f_zgw_network_management_buf);
        }

        if (k_msg.buf[0] == 0x62) {                                                                                                 // HU is registered and its previous neighbour (between 0x480 and 0x4E1 inclusive) requested NM status.
          nbt_network_management_initialized = true;
          convert_f_nbt_network_management();
          nbt_network_management_timer = 0;
        } else if (nbt_network_management_timer >= 3000) {                                                                          // Registration/timeout handled here. Timeout will occur when nodes appear/disappear.
          nbt_network_management_initialized = false;                                                                               // If PDC or the driver's seat module are missing this system will fail!
          convert_f_nbt_network_management();
          nbt_network_management_timer = 2500;                                                                                      // Retry in 500ms.
        }
      #endif
    }
    #endif

    else if (k_msg.id == 0x592) {
      process_dme_cc();
    }

    #if DOOR_VOLUME && !F_NBT
    else if (k_msg.id == 0x663) {                                                                                                   // iDrive diagnostic response. Sent when requested.
      #if DOOR_VOLUME
        evaluate_audio_volume_cic();
      #endif
    }
    #endif

    #if DEBUG_MODE
    else if (k_msg.id == 0x640) {                                                                                                   // CAS diagnostic responses. Sent when requested.
      evaluate_power_down_response();
    }
    #endif

    #if F_VSW01
    else if (k_msg.id == 0x648) {                                                                                                   // VSW diagnostic responses. Sent when requested.
      dcan_write_msg(k_msg);
    }
    #endif

    #if AUTO_MIRROR_FOLD || FRONT_FOG_CORNER
    else if (k_msg.id == 0x672) {                                                                                                   // FRM diagnostic responses. Sent when requested.
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
      if (pt_msg.id == 0x1A0) {
        evaluate_real_speed();
        evaluate_vehicle_moving();
        #if F_NBT
          send_f_lateral_acceleration();
        #endif
        #if F_NIVI || F_NBT
          send_f_longitudinal_acceleration();
          send_f_speed_status();
          send_f_yaw_rate_chassis();                                                                                                // Equivalent to Gyro.
        #endif
      }
      
      #if FRONT_FOG_CORNER || F_NIVI || F_NBT
      else if (pt_msg.id == 0xC4) {
        evaluate_steering_angle();
        #if FRONT_FOG_CORNER
          evaluate_corner_fog_activation();
        #endif
        #if F_NIVI || F_NBT
          send_f_steering_angle();
        #endif
      }
      #endif

      #if SERVOTRONIC_SVT70
      else if (pt_msg.id == 0x58E) {                                                                                                // Since the JBE doesn't forward Servotronic errors from SVT70, we have to do it.
        send_svt_kcan_cc_notification();
      }
      
      else if (pt_msg.id == 0x60E) {                                                                                                // Forward Diagnostic responses from SVT module to DCAN
        if (!uif_read) {
          dcan_write_msg(pt_msg);
        }
      }
      #endif
      
      else {
        if (ignition) {
          #if HDC
          if (pt_msg.id == 0x194) {
            evaluate_cruise_stalk_message();
          }
          #else
          if(0);
          #endif

          #if F_NBT && CUSTOM_MONITORING_CC
          else if (pt_msg.id == 0x612) {
            evaluate_dme_boost_response();
          }
          #endif

          #if F_NIVI
          else if (pt_msg.id == 0x657) {                                                                                            // Forward Diagnostic responses from NVE module to DCAN
            dcan_write_msg(pt_msg);
          }
          #endif
        }
      }
    }


/***********************************************************************************************************************************************************************************************************************************************
  D-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
    
    if (DCAN.read(d_msg)) {
      if (d_msg.id == 0x6F1) {
        if (clearing_dtcs) {}                                                                                                       // Ignore 6F1s while this module is clearing DTCs.

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
        else if (d_msg.buf[0] == 0xEF && d_msg.buf[1] == 2 && d_msg.buf[2] == 0x1A                                                  // UIF read or ISTA/D module identification.
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
          disable_diag_transmit_jobs();
        }
        #endif

        #if F_NIVI
        else if (d_msg.buf[0] == 0x57) {                                                                                            // NIVI address is 0x57.
          ptcan_write_msg(d_msg);
          disable_diag_transmit_jobs();
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
    evaluate_idrive_units();
  }

  else if (k_msg.id == 0x2B8) {
    evaluate_speed_warning_setting();
    return;
  }

  else if (k_msg.id == 0x317) {                                                                                                     // This message is used for the ZBE instead of PDC activation. Causes sporadinc PDC activation. Ignore.
    return;
  }

  else if (k_msg.id == 0x34A) {                                                                                                     // GPS position, appears consistently regardless of Terminal status. Cycle time 1s.
    idrive_alive_timer = 0;
    #if ASD89_MDRIVE
      initialize_asd();
    #endif
    #if ASD89_RAD_ON
      initialize_asd_rad_on();
    #endif
  }

  #if AUTO_TOW_VIEW_RVC
  else if (k_msg.id == 0x38F) {                                                                                                     // Camera settings request from iDrive.
    store_rvc_settings_idrive();
  }
  #endif

  else if (k_msg.id == 0x39E) {                                                                                                     // Received new date/time from NBT.
    evaluate_idrive_zero_time();
    return;
  }

  else if (k_msg.id == 0x3DC) {
    evaluate_idrive_lights_settings();                                                                                              // Force light settings through a parser that forwards to KOMBI.
    return;
  }

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
    show_mdrive_settings_screen_evo();    
  }

  else if (k_msg.id >= 0x6F1) { return; }                                                                                           // Firewall HU from injecting diagnostic messages to other networks.

  if (terminal_r) {
    kcan_write_msg(k_msg);                                                                                                          // Write the message received from KCAN2 to the rest of the car.
  } else {                                                                                                                          // Outside of Terminal R, messages have to be filtered else they will wake the car.
    if (k_msg.id == 0x5E3) {
      if (k_msg.buf[0] == 0x26 && k_msg.buf[1] == 0x88) {
        if (k_msg.buf[3] == 1) {                                                                                                    // The HU was activated using the faceplate power button.
          nbt_bus_sleep = false;
          nbt_bus_sleep_ready_timer = 0;
          nbt_active_after_terminal_r = true;
        } else if (k_msg.buf[3] == 0) {
          nbt_bus_sleep = true;
          nbt_active_after_terminal_r = false;
        }
      }
    }

    if (idrive_alive_timer >= 15000 && idrive_died) {                                                                               // HU must have crashed/became unresponsive. Allow the car to kill KCAN.
      nbt_bus_sleep = true;
      nbt_bus_sleep_ready_timer = 60000;
    }
  }
}
