// Functions that interface with the controller network(s) go here.
// KWP2000 message structure (8 bytes): {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x16};
// [0] - Controller diagnostic address. e.g FRM (0x72), JBE (0).
// [1] - ?. Sometimes represents the sequence of responses. I.e 0x10, 0x21, 0x22...
// [2] - KWP2000 SID, e.g. InputOutputControlByLocalIdentifier (0x30).
// [3] - Control target, e.g PWM-port dim value (3)
// [4] - Control type, e.g ShortTermAdjustment (7)
// [5] - Job dependent
// [6] - Job dependent
// [7] - Job dependent
// KWP jobs are reflected by the JBE across buses. I.e. sending 6F1 to KCAN will be forwarded to PTCAN too.


void cache_can_message_buffers(void) {                                                                                              // Put all static the buffers in memory during setup().
  uint8_t dsc_on[] = {0xCF, 0xE3}, dsc_mdm_dtc[] = {0xCF, 0xF3}, dsc_off[] = {0xCF, 0xE7};
  dsc_on_buf = make_msg_buf(0x398, 2, dsc_on);
  dsc_mdm_dtc_buf = make_msg_buf(0x398, 2, dsc_mdm_dtc);
  dsc_off_buf = make_msg_buf(0x398, 2, dsc_off);
  uint8_t idrive_mdrive_settings_a[] = {0x63, 0x10, 0xA, 0x31, 0x52, 0, 0, 6};
  uint8_t idrive_mdrive_settings_b[] = {0x63, 0x21, 0x5C, 0, 0, 0, 0, 0};
  idrive_mdrive_settings_a_buf = make_msg_buf(0x6F1, 8, idrive_mdrive_settings_a);
  idrive_mdrive_settings_b_buf = make_msg_buf(0x6F1, 8, idrive_mdrive_settings_b);
  uint8_t cc_gong[] = {0x60, 3, 0x31, 0x22, 2, 0, 0, 0};
  cc_gong_buf = make_msg_buf(0x6F1, 8, cc_gong);
  #if EDC_CKM_FIX
    uint8_t edc_button_press[] = {0, 5, 0x30, 1, 7, 0x1A, 0, 0};
    edc_button_press_buf = make_msg_buf(0x6F1, 8, edc_button_press);
  #endif
  #if FTM_INDICATOR
    uint8_t ftm_indicator_flash[] = {0x40, 0x50, 1, 0x69, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t ftm_indicator_off[] = {0x40, 0x50, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF};
    ftm_indicator_flash_buf = make_msg_buf(0x5A0, 8, ftm_indicator_flash);
    ftm_indicator_off_buf = make_msg_buf(0x5A0, 8, ftm_indicator_off);
  #endif
  #if FRM_HEADLIGHT_MODE
    uint8_t frm_ckm_ahl_komfort[] = {0, 4};
    uint8_t frm_ckm_ahl_sport[] = {0, 0xA};
    frm_ckm_ahl_komfort_buf = make_msg_buf(0x3F0, 2, frm_ckm_ahl_komfort);
    frm_ckm_ahl_sport_buf = make_msg_buf(0x3F0, 2, frm_ckm_ahl_sport);
  #endif
  #if WIPE_AFTER_WASH
    uint8_t wipe_single[] = {8, 0xF8};
    wipe_single_buf = make_msg_buf(0x2A6, 2, wipe_single);
  #endif
  #if AUTO_MIRROR_FOLD
    uint8_t frm_toggle_fold_mirror_a[] = {0x72, 0x10, 7, 0x30, 0x10, 7, 1, 5};
    uint8_t frm_toggle_fold_mirror_b[] = {0x72, 0x21, 0, 1, 0, 0, 0, 0};
    uint8_t frm_status_request_a[] = {0x72, 3, 0x30, 0x16, 1, 0, 0, 0};
    uint8_t frm_status_request_b[] = {0x72, 0x30, 0, 0, 0, 0, 0, 0};
    frm_toggle_fold_mirror_a_buf = make_msg_buf(0x6F1, 8, frm_toggle_fold_mirror_a);
    frm_toggle_fold_mirror_b_buf = make_msg_buf(0x6F1, 8, frm_toggle_fold_mirror_b);
    frm_status_request_a_buf = make_msg_buf(0x6F1, 8, frm_status_request_a);
    frm_status_request_b_buf = make_msg_buf(0x6F1, 8, frm_status_request_b);
  #endif
  #if INDICATE_TRUNK_OPENED
    uint8_t flash_hazards[] = {0, 0xF1};
    flash_hazards_buf = make_msg_buf(0x2B4, 2, flash_hazards);
  #endif
  #if ANTI_THEFT_SEQ
    #if ANTI_THEFT_SEQ_ALARM
      uint8_t alarm_siren_on[] = {0x41, 3, 0x31, 4, 2, 0, 0, 0};
      uint8_t alarm_siren_off[] = {0x41, 3, 0x31, 4, 3, 0, 0, 0};
      uint8_t alarm_led_on[] = {0x41, 4, 0x30, 2, 7, 1, 0, 0};
      uint8_t alarm_led_off[] = {0x41, 3, 0x30, 2, 0, 0, 0, 0};
      alarm_siren_on_buf = make_msg_buf(0x6F1, 8, alarm_siren_on);
      alarm_siren_off_buf = make_msg_buf(0x6F1, 8, alarm_siren_off);
      alarm_led_on_buf = make_msg_buf(0x6F1, 8, alarm_led_on);
      alarm_led_off_buf = make_msg_buf(0x6F1, 8, alarm_led_off);
    #endif
    uint8_t ekp_pwm_off[] = {0x17, 4, 0x30, 6, 4, 0, 0, 0};
    uint8_t ekp_return_to_normal[] = {0x17, 2, 0x30, 0, 0, 0, 0, 0};
    uint8_t key_cc_on[] = {0x40, 0x26, 0, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t key_cc_off[] = {0x40, 0x26, 0, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t start_cc_on[] = {0x40, 0x2F, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t start_cc_off[] = {0x40, 0x2F, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    key_cc_on_buf = make_msg_buf(0x5C0, 8, key_cc_on);
    key_cc_off_buf = make_msg_buf(0x5C0, 8, key_cc_off);
    start_cc_on_buf = make_msg_buf(0x5C0, 8, start_cc_on);
    start_cc_off_buf = make_msg_buf(0x5C0, 8, start_cc_off);
    ekp_pwm_off_buf = make_msg_buf(0x6F1, 8, ekp_pwm_off);
    ekp_return_to_normal_buf = make_msg_buf(0x6F1, 8, ekp_return_to_normal);
  #endif
  #if REVERSE_BEEP
    #if RHD 
      uint8_t pdc_beep[] = {0, 0, 0, 1};                                                                                            // Front right beep.
    #else
      uint8_t pdc_beep[] = {0, 0, 1, 0};                                                                                            // Front left beep.
    #endif
    uint8_t pdc_quiet[] = {0, 0, 0, 0};
    pdc_beep_buf = make_msg_buf(0x1C6, 4, pdc_beep);
    pdc_quiet_buf = make_msg_buf(0x1C6, 4, pdc_quiet);
  #endif
  #if FRONT_FOG_CORNER
    uint8_t front_left_fog_on_a[] = {0x72, 6, 0x30, 3, 7, 6, 0, 8};                                                                 // Soft on/off buffers.
    uint8_t front_left_fog_on_b[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x16};
    uint8_t front_left_fog_on_c[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x32};
    uint8_t front_left_fog_on_d[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x64};
    uint8_t front_left_fog_on_a_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 8};
    uint8_t front_left_fog_on_b_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x16};
    uint8_t front_left_fog_on_c_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x24};
    uint8_t front_left_fog_on_d_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x32};
    uint8_t front_left_fog_on_e_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x40};
    uint8_t front_left_fog_on_f_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x48};
    uint8_t front_left_fog_on_g_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x56};
    uint8_t front_left_fog_on_h_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x64};
    uint8_t front_left_fog_off[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0};
    uint8_t front_right_fog_on_a[] = {0x72, 6, 0x30, 3, 7, 7, 0, 8};
    uint8_t front_right_fog_on_b[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x16};
    uint8_t front_right_fog_on_c[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x32};
    uint8_t front_right_fog_on_d[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x64};
    uint8_t front_right_fog_on_a_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 8};
    uint8_t front_right_fog_on_b_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x16};
    uint8_t front_right_fog_on_c_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x24};
    uint8_t front_right_fog_on_d_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x32};
    uint8_t front_right_fog_on_e_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x40};
    uint8_t front_right_fog_on_f_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x48};
    uint8_t front_right_fog_on_g_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x56};
    uint8_t front_right_fog_on_h_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x64};
    uint8_t front_right_fog_off[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0};
    uint8_t front_fogs_all_off[] = {0x72, 6, 0x30, 0x29, 7, 0, 1, 2};
    front_left_fog_on_a_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_a);
    front_left_fog_on_b_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_b);
    front_left_fog_on_c_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_c);
    front_left_fog_on_d_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_d);
    front_left_fog_on_a_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_a_softer);
    front_left_fog_on_b_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_b_softer);
    front_left_fog_on_c_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_c_softer);
    front_left_fog_on_d_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_d_softer);
    front_left_fog_on_e_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_e_softer);
    front_left_fog_on_f_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_f_softer);
    front_left_fog_on_g_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_g_softer);
    front_left_fog_on_h_softer_buf = make_msg_buf(0x6F1, 8, front_left_fog_on_h_softer);
    front_left_fog_off_buf = make_msg_buf(0x6F1, 8, front_left_fog_off);
    front_right_fog_on_a_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_a);
    front_right_fog_on_b_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_b);
    front_right_fog_on_c_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_c);
    front_right_fog_on_d_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_d);
    front_right_fog_on_a_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_a_softer);
    front_right_fog_on_b_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_b_softer);
    front_right_fog_on_c_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_c_softer);
    front_right_fog_on_d_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_d_softer);
    front_right_fog_on_e_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_e_softer);
    front_right_fog_on_f_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_f_softer);
    front_right_fog_on_g_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_g_softer);
    front_right_fog_on_h_softer_buf = make_msg_buf(0x6F1, 8, front_right_fog_on_h_softer);
    front_right_fog_off_buf = make_msg_buf(0x6F1, 8, front_right_fog_off);
    front_fogs_all_off_buf = make_msg_buf(0x6F1, 8, front_fogs_all_off);                                                            // This job only works with ignition ON.
  #endif
  #if DIM_DRL
    uint8_t left_drl_off[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0};
    uint8_t left_drl_dim[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x16};
    uint8_t left_drl_bright[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x64};
    uint8_t right_drl_off[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0};
    uint8_t right_drl_dim[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0x16};
    uint8_t right_drl_bright[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0x64};
    left_drl_dim_off = make_msg_buf(0x6F1, 8, left_drl_off);
    left_drl_dim_buf = make_msg_buf(0x6F1, 8, left_drl_dim);
    left_drl_bright_buf = make_msg_buf(0x6F1, 8, left_drl_bright);
    right_drl_dim_off = make_msg_buf(0x6F1, 8, right_drl_off);
    right_drl_dim_buf = make_msg_buf(0x6F1, 8, right_drl_dim);
    right_drl_bright_buf = make_msg_buf(0x6F1, 8, right_drl_bright);
  #endif
  #if F_ZBE_WAKE || F_VSW01 || F_NIVI
    uint8_t f_kombi_network_mgmt[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                             // Network management KOMBI - F-series.
    f_kombi_network_mgmt_buf = make_msg_buf(0x560, 8, f_kombi_network_mgmt);
  #endif
  #if F_VSW01
    uint8_t vsw_switch[] = {0, 0, 0, 0, 0, 0, 0, 0xF1};
    for (uint8_t i = 0; i < 6; i++) {                                                                                               // Typical VSWs have 5 outputs. The PCB could have 7 inputs if components were soldered.
      vsw_switch[0] = i;
      vsw_switch_buf[i] = make_msg_buf(0x2FB, 8, vsw_switch);
    }
    // uint8_t idrive_menu_request[] = {};
    // idrive_menu_request_buf = make_msg_buf(0x6F1, 8, idrive_menu_request);
  #endif
  #if F_NIVI
    uint8_t sine_angle_request_a[] = {0x50, 2, 0x21, 5};
    uint8_t sine_angle_request_b[] = {0x50, 0x30, 0, 2, 0xFF, 0xFF, 0xFF, 0xFF};
    sine_angle_request_a_buf = make_msg_buf(0x6F1, 4, sine_angle_request_a);
    sine_angle_request_b_buf = make_msg_buf(0x6F1, 8, sine_angle_request_b);
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    uint8_t lc_cc_on[] = {0x40, 0xBE, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t lc_cc_off[] = {0x40, 0xBE, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    lc_cc_on_buf = make_msg_buf(0x598, 8, lc_cc_on);
    lc_cc_off_buf = make_msg_buf(0x598, 8, lc_cc_off);
  #endif
  #if AUTO_SEAT_HEATING
    uint8_t seat_heating_button_pressed[] = {0xFD, 0xFF};
    uint8_t seat_heating_button_released[] = {0xFC, 0xFF};
    seat_heating_button_pressed_dr_buf = make_msg_buf(0x1E7, 2, seat_heating_button_pressed);
    seat_heating_button_released_dr_buf = make_msg_buf(0x1E7, 2, seat_heating_button_released);
    #if AUTO_SEAT_HEATING_PASS
      seat_heating_button_pressed_pas_buf = make_msg_buf(0x1E8, 2, seat_heating_button_pressed);
      seat_heating_button_released_pas_buf = make_msg_buf(0x1E8, 2, seat_heating_button_released);
    #endif
  #endif
  #if RTC
    uint8_t set_time_cc[] = {0x40, 0xA7, 0, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t set_time_cc_off[] = {0x40, 0xA7, 0, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    set_time_cc_buf = make_msg_buf(0x5E0, 8, set_time_cc);
    set_time_cc_off_buf = make_msg_buf(0x5E0, 8, set_time_cc_off);
  #endif
  #if CONTROL_SHIFTLIGHTS
    uint8_t shiftlights_start[] = {0x86, 0x3E};
    uint8_t shiftlights_mid_buildup[] = {0xF6, 0};
    #if NEEDLE_SWEEP
      uint8_t shiftlights_startup_buildup[] = {0x86, 0};
    #else
      uint8_t shiftlights_startup_buildup[] = {0x56, 0};                                                                            // Faster sequential buildup. First bit 0-0xF (0xF - slowest).
    #endif
    uint8_t shiftlights_max_flash[] = {0xA, 0};
    uint8_t shiftlights_off[] = {5, 0};
    shiftlights_start_buf = make_msg_buf(0x206, 2, shiftlights_start);
    shiftlights_mid_buildup_buf = make_msg_buf(0x206, 2, shiftlights_mid_buildup);
    shiftlights_startup_buildup_buf = make_msg_buf(0x206, 2, shiftlights_startup_buildup);
    shiftlights_max_flash_buf = make_msg_buf(0x206, 2, shiftlights_max_flash);
    shiftlights_off_buf = make_msg_buf(0x206, 2, shiftlights_off);
  #endif
  #if NEEDLE_SWEEP
    uint8_t speedo_needle_max[] = {0x60, 5, 0x30, 0x20, 6, 0x12, 0x11, 0};                                                          // Set to 325 KM/h
    uint8_t speedo_needle_min[] = {0x60, 5, 0x30, 0x20, 6, 0, 0, 0};                                                                // Set to 0
    uint8_t speedo_needle_release[] = {0x60, 3, 0x30, 0x20, 0, 0, 0, 0};
    uint8_t tacho_needle_max[] = {0x60, 5, 0x30, 0x21, 6, 0x12, 0x3D, 0};                                                           // Set to 8000 RPM
    uint8_t tacho_needle_min[] = {0x60, 5, 0x30, 0x21, 6, 0, 0, 0};                                                                 // Set to 0
    uint8_t tacho_needle_release[] = {0x60, 3, 0x30, 0x21, 0, 0, 0, 0};
    uint8_t fuel_needle_max[] = {0x60, 5, 0x30, 0x22, 6, 7, 0x4E, 0};                                                               // Set to 100%
    uint8_t fuel_needle_min[] = {0x60, 5, 0x30, 0x22, 6, 0, 0, 0};                                                                  // Set to 0%
    uint8_t fuel_needle_release[] = {0x60, 3, 0x30, 0x22, 0, 0, 0, 0};
    uint8_t oil_needle_max[] = {0x60, 5, 0x30, 0x23, 6, 7, 0x12, 0};                                                                // Set to 150 C
    uint8_t oil_needle_min[] = {0x60, 5, 0x30, 0x23, 6, 0, 0, 0};                                                                   // Set to 0 C
    uint8_t oil_needle_release[] = {0x60, 3, 0x30, 0x23, 0, 0, 0, 0};
    speedo_needle_max_buf = make_msg_buf(0x6F1, 8, speedo_needle_max);
    speedo_needle_min_buf = make_msg_buf(0x6F1, 8, speedo_needle_min);
    speedo_needle_release_buf = make_msg_buf(0x6F1, 8, speedo_needle_release);
    tacho_needle_max_buf = make_msg_buf(0x6F1, 8, tacho_needle_max);
    tacho_needle_min_buf = make_msg_buf(0x6F1, 8, tacho_needle_min);
    tacho_needle_release_buf = make_msg_buf(0x6F1, 8, tacho_needle_release);
    fuel_needle_max_buf = make_msg_buf(0x6F1, 8, fuel_needle_max);
    fuel_needle_min_buf = make_msg_buf(0x6F1, 8, fuel_needle_min);
    fuel_needle_release_buf = make_msg_buf(0x6F1, 8, fuel_needle_release);
    oil_needle_max_buf = make_msg_buf(0x6F1, 8, oil_needle_max);
    oil_needle_min_buf = make_msg_buf(0x6F1, 8, oil_needle_min);
    oil_needle_release_buf = make_msg_buf(0x6F1, 8, oil_needle_release);
  #endif
  #if DOOR_VOLUME
    uint8_t vol_request[] = {0x63, 3, 0x31, 0x24, 0, 0, 0, 0}; 
    uint8_t door_open_cc_off[] = {0x40, 0x4F, 1, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
    vol_request_buf = make_msg_buf(0x6F1, 8, vol_request);
    door_open_cc_off_buf = make_msg_buf(0x5C0, 8, door_open_cc_off);
  #endif
  #if HDC
    uint8_t set_hdc_cruise_control[] = {0, 0xFB, 8, 0xFC};
    uint8_t cancel_hdc_cruise_control[] = {0, 4, 0x10, 0xFC};
    uint8_t hdc_cc_activated_on[] = {0x40, 0x4B, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_unavailable_on[] = {0x40, 0x4D, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_deactivated_on[] = {0x40, 0x4C, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_activated_off[] = {0x40, 0x4B, 1, 0, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_unavailable_off[] = {0x40, 0x4D, 1, 0, 0xFF, 0xFF, 0xFF, 0xAB};
    uint8_t hdc_cc_deactivated_off[] = {0x40, 0x4C, 1, 0, 0xFF, 0xFF, 0xFF, 0xAB};
    set_hdc_cruise_control_buf = make_msg_buf(0x194, 4, set_hdc_cruise_control);
    cancel_hdc_cruise_control_buf = make_msg_buf(0x194, 4, cancel_hdc_cruise_control);
    hdc_cc_activated_on_buf = make_msg_buf(0x5A9, 8, hdc_cc_activated_on);
    hdc_cc_unavailable_on_buf = make_msg_buf(0x5A9, 8, hdc_cc_unavailable_on);
    hdc_cc_deactivated_on_buf = make_msg_buf(0x5A9, 8, hdc_cc_deactivated_on);
    hdc_cc_activated_off_buf = make_msg_buf(0x5A9, 8, hdc_cc_activated_off);
    hdc_cc_unavailable_off_buf = make_msg_buf(0x5A9, 8, hdc_cc_unavailable_off);
    hdc_cc_deactivated_off_buf = make_msg_buf(0x5A9, 8, hdc_cc_deactivated_off);
  #endif
  #if FAKE_MSA
    uint8_t msa_deactivated_cc_on[] = {0x40, 0xC2, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t msa_deactivated_cc_off[] = {0x40, 0xC2, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
    msa_deactivated_cc_on_buf = make_msg_buf(0x592, 8, msa_deactivated_cc_on);
    msa_deactivated_cc_off_buf = make_msg_buf(0x592, 8, msa_deactivated_cc_off);
  #endif
  #if MSA_RVC
    uint8_t camera_off[] = {0xA1, 0xFF};
    uint8_t camera_on[] = {0xA5, 0xFF};
    uint8_t pdc_off_camera_on[] = {0x64, 4, 0x30, 9, 7, 4, 0, 0};
    uint8_t pdc_on_camera_on[] = {0x64, 4, 0x30, 9, 7, 5, 0, 0};
    uint8_t pdc_off_camera_off[] = {0x64, 4, 0x30, 9, 7, 0, 0, 0};
    camera_off_buf = make_msg_buf(0x3AE, 2, camera_off);
    camera_on_buf = make_msg_buf(0x3AE, 2, camera_on);
    pdc_off_camera_on_buf = make_msg_buf(0x6F1, 8, pdc_off_camera_on);
    pdc_on_camera_on_buf = make_msg_buf(0x6F1, 8, pdc_on_camera_on);
    pdc_off_camera_off_buf = make_msg_buf(0x6F1, 8, pdc_off_camera_off);
  #endif
  #if FAKE_MSA || MSA_RVC
    uint8_t msa_fake_status[] = {0xFF, 0xFF};
    msa_fake_status_buf = make_msg_buf(0x308, 2, msa_fake_status);
  #endif
}


CAN_message_t make_msg_buf(uint16_t txID, uint8_t txLen, uint8_t* txBuf) {
  CAN_message_t tx_msg;
  tx_msg.id = txID;
  tx_msg.len = txLen;
  for (uint8_t i = 0; i < txLen; i++) {
      tx_msg.buf[i] = txBuf[i];
  }
  return tx_msg;
}


void kcan_write_msg(const CAN_message_t &msg) {
  if (msg.id == 0x6F1 && !diag_transmit) {
    if (msg.buf[0] == 0x41 && (msg.buf[2] == 0x30 || msg.buf[2] == 0x31)) {                                                         // Exception for alarm jobs.
    } else {
      #if DEBUG_MODE
        serial_log("6F1 message not sent to KCAN due to OBD tool presence.");
        can_debug_print_buffer(msg);
      #endif
      return;
    }
  }
  uint8_t result = KCAN.write(msg);
  if (result != 1) {
    if (kcan_retry_counter < 100) {                                                                                                 // Safeguard to avoid polluting the network in case of unrecoverable issue.
      m = {msg, millis() + 50};
      kcan_resend_txq.push(&m);
      kcan_retry_counter++;
    }
    #if DEBUG_MODE
      sprintf(serial_debug_string, "KCAN write failed for ID: %lX with error %d. Re-sending.", msg.id, result);
      serial_log(serial_debug_string);
      can_debug_print_buffer(msg);
      kcan_error_counter++;
    #endif
  }
}


void ptcan_write_msg(const CAN_message_t &msg) {
  if (msg.id == 0x6F1 && !diag_transmit) {
    if (msg.buf[2] == 0x30 && msg.buf[3] == 6 && msg.buf[4] == 4) {                                                                  // Exception for EKP disable.
    } else if (msg.buf[0] == 0xE) {                                                                                                  // Exception for SVT70 diag.
    } else {
      #if DEBUG_MODE
        serial_log("6F1 message not sent to PTCAN due to OBD tool presence.");
        can_debug_print_buffer(msg);
      #endif
      return;
    }
  }

  uint8_t result = PTCAN.write(msg);
  if (result != 1) {
    if (ptcan_retry_counter < 100) {                                                                                                // Safeguard to avoid polluting the network in case of unrecoverable issue.
      m = {msg, millis() + 20};
      ptcan_resend_txq.push(&m);
      ptcan_retry_counter++;
    }
    #if DEBUG_MODE
      sprintf(serial_debug_string, "PTCAN write failed for ID: %lX with error %d. Re-sending.", msg.id, result);
      serial_log(serial_debug_string);
      can_debug_print_buffer(msg);
      ptcan_error_counter++;
    #endif
  }
}


void dcan_write_msg(const CAN_message_t &msg) {
  uint8_t result = DCAN.write(msg);
  if (result != 1) {
    if (dcan_retry_counter < 100) {                                                                                                 // Safeguard to avoid polluting the network in case of unrecoverable issue.
      m = {msg, millis() + 100};
      dcan_resend_txq.push(&m);
      dcan_retry_counter++;
    }
    #if DEBUG_MODE
      sprintf(serial_debug_string, "DCAN write failed for ID: %lX with error %d.", msg.id, result);
      serial_log(serial_debug_string);
      can_debug_print_buffer(msg);
      dcan_error_counter++;
    #endif
  }
}


void check_can_resend_queues(void) {
  if (!kcan_resend_txq.isEmpty()) {
    kcan_resend_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      kcan_resend_txq.drop();
    }
  }
  if (!ptcan_resend_txq.isEmpty()) {
    ptcan_resend_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      ptcan_resend_txq.drop();
    }
  }
  if (!dcan_resend_txq.isEmpty()) {
    dcan_resend_txq.peek(&delayed_tx);
    if (millis() >= delayed_tx.transmit_time) {
      kcan_write_msg(delayed_tx.tx_msg);
      dcan_resend_txq.drop();
    }
  }
}


#if DEBUG_MODE
void can_debug_print_buffer(const CAN_message_t &msg) {
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  }
  Serial.println();
}
#endif
