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
  uint8_t idrive_mdrive_settings_menu_a[] = {0x63, 0x10, 0xA, 0x31, 0x52, 0, 0, 6},
          idrive_mdrive_settings_menu_b[] = {0x63, 0x21, 0x5C, 0, 0, 0, 0, 0};
  idrive_mdrive_settings_menu_a_buf = make_msg_buf(0x6F1, 8, idrive_mdrive_settings_menu_a);
  idrive_mdrive_settings_menu_b_buf = make_msg_buf(0x6F1, 8, idrive_mdrive_settings_menu_b);

  uint8_t gws_sport_on[] = {0, 0, 0, 4, 0, 0, 0},
          gws_sport_off[] = {0, 0, 0, 0, 0, 0, 0};
  gws_sport_on_buf = make_msg_buf(0x1D2, 6, gws_sport_on);
  gws_sport_off_buf = make_msg_buf(0x1D2, 6, gws_sport_off);

  uint8_t cc_single_gong[] = {0xF1, 0xFF};
  uint8_t cc_double_gong[] = {0xF2, 0xFF};
  uint8_t cc_triple_gong[] = {0xF3, 0xFF};
  cc_single_gong_buf = make_msg_buf(0x205, 2, cc_single_gong);
  cc_double_gong_buf = make_msg_buf(0x205, 2, cc_double_gong);
  cc_triple_gong_buf = make_msg_buf(0x205, 2, cc_triple_gong);

  #if F_NBT
    uint8_t idrive_button_sound[] = {0x63, 5, 0x31, 1, 0xA0, 0, 7, 0},
            idrive_beep_sound[] = {0x63, 5, 0x31, 1, 0xA0, 0, 0x12, 0},
            idrive_double_beep_sound[] = {0x63, 5, 0x31, 1, 0xA0, 0, 0x10, 0};
    idrive_button_sound_buf = make_msg_buf(0x6F1, 8, idrive_button_sound);
    idrive_beep_sound_buf = make_msg_buf(0x6F1, 8, idrive_beep_sound);
    idrive_double_beep_sound_buf = make_msg_buf(0x6F1, 8, idrive_double_beep_sound);
  #else
    uint8_t idrive_button_sound[] = {0x63, 3, 0x31, 0x21, 7},
            idrive_beep_sound[] = {0x63, 3, 0x31, 0x21, 0x12},
            idrive_double_beep_sound[] = {0x63, 3, 0x31, 0x21, 0x10};
    idrive_button_sound_buf = make_msg_buf(0x6F1, 5, idrive_button_sound);
    idrive_beep_sound_buf = make_msg_buf(0x6F1, 5, idrive_beep_sound);
    idrive_double_beep_sound_buf = make_msg_buf(0x6F1, 5, idrive_double_beep_sound);
  #endif

  uint8_t clear_fs_job_uds_nbt[] = {0x63, 4, 0x14, 0xFF, 0xFF, 0xFF, 0, 0},
          clear_is_job_uds_nbt[] = {0x63, 4, 0x31, 1, 0xF, 6, 0, 0},
          clear_fs_job_uds_zbe[] = {0x67, 4, 0x14, 0xFF, 0xFF, 0xFF, 0, 0},
          clear_is_job_uds_zbe[] = {0x67, 4, 0x31, 1, 0xF, 6, 0, 0},
          clear_fs_job_uds_tbx[] = {0x35, 4, 0x14, 0xFF, 0xFF, 0xFF, 0, 0},
          clear_is_job_uds_tbx[] = {0x35, 4, 0x31, 1, 0xF, 6, 0, 0},
          clear_fs_job_vsw[] = {0x48, 4, 0x14, 0xFF, 0xFF, 0xFF, 0, 0},
          clear_is_job_vsw[] = {0x48, 4, 0x31, 1, 0xF, 6, 0, 0};
  clear_fs_job_uds_nbt_buf = make_msg_buf(0x6F1, 8, clear_fs_job_uds_nbt);
  clear_is_job_uds_nbt_buf = make_msg_buf(0x6F1, 8, clear_is_job_uds_nbt);
  clear_fs_job_uds_zbe_buf = make_msg_buf(0x6F1, 8, clear_fs_job_uds_zbe);
  clear_is_job_uds_zbe_buf = make_msg_buf(0x6F1, 8, clear_is_job_uds_zbe);
  clear_fs_job_uds_tbx_buf = make_msg_buf(0x6F1, 8, clear_fs_job_uds_tbx);
  clear_is_job_uds_tbx_buf = make_msg_buf(0x6F1, 8, clear_is_job_uds_tbx);
  clear_fs_job_vsw_buf = make_msg_buf(0x6F1, 8, clear_fs_job_vsw);
  clear_is_job_vsw_buf = make_msg_buf(0x6F1, 8, clear_is_job_vsw);

  uint8_t ccc_zbe_wake[] = {0xFE, 3, 0, 0, 0, 0, 0, 0};
  ccc_zbe_wake_buf = make_msg_buf(0x1AA, 8, ccc_zbe_wake);

  uint8_t nbt_vin_request[] = {0x63, 3, 0x22, 0xF1, 0x90, 0, 0, 0x40};
  nbt_vin_request_buf = make_msg_buf(0x6F1, 8, nbt_vin_request);

  uint8_t faceplate_a1_released[] = {0, 0xFF},
          faceplate_power_mute[] = {4, 0xFF},
          faceplate_eject[] = {1, 0xFF},
          faceplate_a2_released[] = {0, 0},
          faceplate_button1_hover[] = {1, 0},
          faceplate_button1_press[] = {2, 0},
          faceplate_button2_hover[] = {4, 0},
          faceplate_button2_press[] = {8, 0},
          faceplate_button3_hover[] = {0x10, 0},
          faceplate_button3_press[] = {0x20, 0},
          faceplate_button4_hover[] = {0x40, 0},
          faceplate_button4_press[] = {0x80, 0},
          faceplate_button5_hover[] = {0, 1},
          faceplate_button5_press[] = {0, 2},
          faceplate_button6_hover[] = {0, 4},
          faceplate_button6_press[] = {0, 8},
          faceplate_button7_hover[] = {0, 0x10},
          faceplate_button7_press[] = {0, 0x20},
          faceplate_button8_hover[] = {0, 0x40},
          faceplate_button8_press[] = {0, 0x80},
          faceplate_a3_released[] = {0xFC, 0xFF},
          faceplate_seek_left[] = {0xFD, 0xFF},
          faceplate_seek_right[] = {0xFE, 0xFF},
          faceplate_f1_released[] = {0, 0xFC},
          faceplate_volume_decrease[] = {1, 0xFE},
          faceplate_volume_increase[] = {1, 0xFD};
  faceplate_a1_released_buf = make_msg_buf(0xA1, 2, faceplate_a1_released);
  faceplate_power_mute_buf = make_msg_buf(0xA1, 2, faceplate_power_mute);
  faceplate_eject_buf = make_msg_buf(0xA1, 2, faceplate_eject);
  faceplate_a2_released_buf = make_msg_buf(0xA2, 2, faceplate_a2_released);
  faceplate_button1_hover_buf = make_msg_buf(0xA2, 2, faceplate_button1_hover);
  faceplate_button1_press_buf = make_msg_buf(0xA2, 2, faceplate_button1_press);
  faceplate_button2_hover_buf = make_msg_buf(0xA2, 2, faceplate_button2_hover);
  faceplate_button2_press_buf = make_msg_buf(0xA2, 2, faceplate_button2_press);
  faceplate_button3_hover_buf = make_msg_buf(0xA2, 2, faceplate_button3_hover);
  faceplate_button3_press_buf = make_msg_buf(0xA2, 2, faceplate_button3_press);
  faceplate_button4_hover_buf = make_msg_buf(0xA2, 2, faceplate_button4_hover);
  faceplate_button4_press_buf = make_msg_buf(0xA2, 2, faceplate_button4_press);
  faceplate_button5_hover_buf = make_msg_buf(0xA2, 2, faceplate_button5_hover);
  faceplate_button5_press_buf = make_msg_buf(0xA2, 2, faceplate_button5_press);
  faceplate_button6_hover_buf = make_msg_buf(0xA2, 2, faceplate_button6_hover);
  faceplate_button6_press_buf = make_msg_buf(0xA2, 2, faceplate_button6_press);
  faceplate_button7_hover_buf = make_msg_buf(0xA2, 2, faceplate_button7_hover);
  faceplate_button7_press_buf = make_msg_buf(0xA2, 2, faceplate_button7_press);
  faceplate_button8_hover_buf = make_msg_buf(0xA2, 2, faceplate_button8_hover);
  faceplate_button8_press_buf = make_msg_buf(0xA2, 2, faceplate_button8_press);
  faceplate_a3_released_buf = make_msg_buf(0xA3, 2, faceplate_a3_released);
  faceplate_seek_left_buf = make_msg_buf(0xA3, 2, faceplate_seek_left);
  faceplate_seek_right_buf = make_msg_buf(0xA3, 2, faceplate_seek_right);
  faceplate_f1_released_buf = make_msg_buf(0xF1, 2, faceplate_f1_released);
  faceplate_volume_decrease_buf = make_msg_buf(0xF1, 2, faceplate_volume_decrease);
  faceplate_volume_increase_buf = make_msg_buf(0xF1, 2, faceplate_volume_increase);
  
  uint8_t edc_button_press[] = {0, 5, 0x30, 1, 7, 0x1A, 0, 0};
  edc_button_press_buf = make_msg_buf(0x6F1, 8, edc_button_press);

  uint8_t ftm_indicator_flash[] = {0x40, 0x50, 1, 0x69, 0xFF, 0xFF, 0xFF, 0xFF},
          ftm_indicator_off[] = {0x40, 0x50, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF};
  ftm_indicator_flash_buf = make_msg_buf(0x5A0, 8, ftm_indicator_flash);
  ftm_indicator_off_buf = make_msg_buf(0x5A0, 8, ftm_indicator_off);

  uint8_t frm_ckm_ahl_komfort[] = {0, 4}, frm_ckm_ahl_sport[] = {0, 0xA};
  frm_ckm_ahl_komfort_buf = make_msg_buf(0x3F0, 2, frm_ckm_ahl_komfort);
  frm_ckm_ahl_sport_buf = make_msg_buf(0x3F0, 2, frm_ckm_ahl_sport);

  uint8_t frm_toggle_fold_mirror_a[] = {0x72, 0x10, 7, 0x30, 0x10, 7, 1, 5},
          frm_toggle_fold_mirror_b[] = {0x72, 0x21, 0, 1, 0, 0, 0, 0},
          frm_mirror_status_request_a[] = {0x72, 3, 0x30, 0x16, 1, 0, 0, 0},
          frm_mirror_status_request_b[] = {0x72, 0x30, 0, 0, 0, 0, 0, 0};
  frm_toggle_fold_mirror_a_buf = make_msg_buf(0x6F1, 8, frm_toggle_fold_mirror_a);
  frm_toggle_fold_mirror_b_buf = make_msg_buf(0x6F1, 8, frm_toggle_fold_mirror_b);
  frm_mirror_status_request_a_buf = make_msg_buf(0x6F1, 8, frm_mirror_status_request_a);
  frm_mirror_status_request_b_buf = make_msg_buf(0x6F1, 8, frm_mirror_status_request_b);

  uint8_t frm_mirror_undim[] = {0x72, 5, 0x30, 0x11, 7, 0, 0x90, 0};
  frm_mirror_undim_buf = make_msg_buf(0x6F1, 8, frm_mirror_undim);
  
  uint8_t flash_hazards_single[] = {0, 0xF1};
  uint8_t flash_hazards_double[] = {0, 0xF2};
  flash_hazards_single_buf = make_msg_buf(0x2B4, 2, flash_hazards_single);
  flash_hazards_double_buf = make_msg_buf(0x2B4, 2, flash_hazards_double);

  uint8_t alarm_siren_on[] = {0x41, 3, 0x31, 4, 2, 0, 0, 0},
          alarm_siren_return_control[] = {0x41, 3, 0x31, 4, 3, 0, 0, 0},
          alarm_led_on[] = {0x41, 4, 0x30, 2, 7, 1, 0, 0},
          alarm_led_return_control[] = {0x41, 3, 0x30, 2, 0, 0, 0, 0};
  alarm_siren_on_buf = make_msg_buf(0x6F1, 8, alarm_siren_on);
  alarm_siren_return_control_buf = make_msg_buf(0x6F1, 8, alarm_siren_return_control);
  alarm_led_on_buf = make_msg_buf(0x6F1, 8, alarm_led_on);
  alarm_led_return_control_buf = make_msg_buf(0x6F1, 8, alarm_led_return_control);

  uint8_t ekp_pwm_off[] = {0x17, 4, 0x30, 6, 4, 0, 0, 0},
          ekp_return_to_normal[] = {0x17, 2, 0x30, 0, 0, 0, 0, 0},
          key_cc_on[] = {0x40, 0x26, 0, 0x39, 0xFF, 0xFF, 0xFF, 0xFF},
          key_cc_off[] = {0x40, 0x26, 0, 0x30, 0xFF, 0xFF, 0xFF, 0xFF},
          start_cc_on[] = {0x40, 0x2F, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF},
          start_cc_off[] = {0x40, 0x2F, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
  key_cc_on_buf = make_msg_buf(0x5C0, 8, key_cc_on);
  key_cc_off_buf = make_msg_buf(0x5C0, 8, key_cc_off);
  start_cc_on_buf = make_msg_buf(0x5C0, 8, start_cc_on);
  start_cc_off_buf = make_msg_buf(0x5C0, 8, start_cc_off);
  ekp_pwm_off_buf = make_msg_buf(0x6F1, 8, ekp_pwm_off);
  ekp_return_to_normal_buf = make_msg_buf(0x6F1, 8, ekp_return_to_normal);

  uint8_t dr_seat_move_back[] = {0x6D, 6, 0x30, 0x10, 6, 8, 2, 0xA};
  dr_seat_move_back_buf = make_msg_buf(0x6F1, 8, dr_seat_move_back);

  uint8_t front_left_fog_on_a[] = {0x72, 6, 0x30, 3, 7, 6, 0, 8},                                                                   // Soft on/off buffers.
          front_left_fog_on_b[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x16},
          front_left_fog_on_c[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x32},
          front_left_fog_on_d[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x64},
          front_left_fog_on_a_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 8},
          front_left_fog_on_b_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x16},
          front_left_fog_on_c_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x24},
          front_left_fog_on_d_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x32},
          front_left_fog_on_e_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x40},
          front_left_fog_on_f_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x48},
          front_left_fog_on_g_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x56},
          front_left_fog_on_h_softer[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0x64},
          front_left_fog_off[] = {0x72, 6, 0x30, 3, 7, 6, 0, 0},
          front_right_fog_on_a[] = {0x72, 6, 0x30, 3, 7, 7, 0, 8},
          front_right_fog_on_b[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x16},
          front_right_fog_on_c[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x32},
          front_right_fog_on_d[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x64},
          front_right_fog_on_a_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 8},
          front_right_fog_on_b_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x16},
          front_right_fog_on_c_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x24},
          front_right_fog_on_d_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x32},
          front_right_fog_on_e_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x40},
          front_right_fog_on_f_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x48},
          front_right_fog_on_g_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x56},
          front_right_fog_on_h_softer[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0x64},
          front_right_fog_off[] = {0x72, 6, 0x30, 3, 7, 7, 0, 0},
          front_fogs_all_off[] = {0x72, 6, 0x30, 0x29, 7, 0, 1, 2};
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
  front_fogs_all_off_buf = make_msg_buf(0x6F1, 8, front_fogs_all_off);                                                              // This job only works with ignition ON.

  uint8_t frm_ahl_status_request[] = {0x72, 3, 0x30, 0x28, 1, 0, 0, 0};
  frm_ahl_flc_status_request_buf = make_msg_buf(0x6F1, 8, frm_ahl_status_request);

  uint8_t left_drl_off[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0},
          left_drl_dim[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x16},
          left_drl_bright[] = {0x72, 6, 0x30, 3, 7, 0x1D, 0, 0x64},
          right_drl_off[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0},
          right_drl_dim[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0x16},
          right_drl_bright[] = {0x72, 6, 0x30, 3, 7, 0x1E, 0, 0x64};
  left_drl_dim_off = make_msg_buf(0x6F1, 8, left_drl_off);
  left_drl_dim_buf = make_msg_buf(0x6F1, 8, left_drl_dim);
  left_drl_bright_buf = make_msg_buf(0x6F1, 8, left_drl_bright);
  right_drl_dim_off = make_msg_buf(0x6F1, 8, right_drl_off);
  right_drl_dim_buf = make_msg_buf(0x6F1, 8, right_drl_dim);
  right_drl_bright_buf = make_msg_buf(0x6F1, 8, right_drl_bright);

  uint8_t f_kombi_network_mgmt[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                               // Network management KOMBI - F-series.
  f_kombi_network_mgmt_buf = make_msg_buf(0x560, 8, f_kombi_network_mgmt);

  uint8_t f_zgw_network_mgmt[] = {0x40, 0x10, 0x10, 0, 0, 2, 1, 0};                                                                 // Network management ZGW - F-series.
  f_zgw_network_mgmt_buf = make_msg_buf(0x510, 8, f_zgw_network_mgmt);

  uint8_t sine_angle_request_a[] = {0x50, 2, 0x21, 5},
          sine_angle_request_b[] = {0x50, 0x30, 0, 2, 0xFF, 0xFF, 0xFF, 0xFF};
  sine_angle_request_a_buf = make_msg_buf(0x6F1, 4, sine_angle_request_a);
  sine_angle_request_b_buf = make_msg_buf(0x6F1, 8, sine_angle_request_b);

  uint8_t lc_cc_on[] = {0x40, 0xBE, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF},
          lc_cc_off[] = {0x40, 0xBE, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
  lc_cc_on_buf = make_msg_buf(0x598, 8, lc_cc_on);
  lc_cc_off_buf = make_msg_buf(0x598, 8, lc_cc_off);

  uint8_t generic_button_pressed[] = {0xFD, 0xFF}, generic_button_released[] = {0xFC, 0xFF};
  seat_heating_button_pressed_dr_buf = make_msg_buf(0x1E7, 2, generic_button_pressed);
  seat_heating_button_released_dr_buf = make_msg_buf(0x1E7, 2, generic_button_released);
  seat_heating_button_pressed_pas_buf = make_msg_buf(0x1E8, 2, generic_button_pressed);
  seat_heating_button_released_pas_buf = make_msg_buf(0x1E8, 2, generic_button_released);

  uint8_t shiftlights_start[] = {0x86, 0x3E}, 
          shiftlights_mid_buildup[] = {0xF6, 0},
  #if NEEDLE_SWEEP
          shiftlights_startup_buildup[] = {0x86, 0},
  #else
          shiftlights_startup_buildup[] = {0x56, 0},                                                                                // Faster sequential buildup. First 8 bits: 0-0xF (0xF - slowest).
  #endif
          shiftlights_max_flash[] = {0xA, 0},
          shiftlights_off[] = {5, 0};
  shiftlights_start_buf = make_msg_buf(0x206, 2, shiftlights_start);
  shiftlights_mid_buildup_buf = make_msg_buf(0x206, 2, shiftlights_mid_buildup);
  shiftlights_startup_buildup_buf = make_msg_buf(0x206, 2, shiftlights_startup_buildup);
  shiftlights_max_flash_buf = make_msg_buf(0x206, 2, shiftlights_max_flash);
  shiftlights_off_buf = make_msg_buf(0x206, 2, shiftlights_off);

  uint8_t speedo_needle_max[] = {0x60, 5, 0x30, 0x20, 6, 0x12, 0x11, 0},                                                            // Set to 325 KM/h
          speedo_needle_min[] = {0x60, 5, 0x30, 0x20, 6, 0, 0, 0},                                                                  // Set to 0
          speedo_needle_release[] = {0x60, 3, 0x30, 0x20, 0, 0, 0, 0},
          tacho_needle_max[] = {0x60, 5, 0x30, 0x21, 6, 0x12, 0x3D, 0},                                                             // Set to 8000 RPM
          tacho_needle_min[] = {0x60, 5, 0x30, 0x21, 6, 0, 0, 0},                                                                   // Set to 0
          tacho_needle_release[] = {0x60, 3, 0x30, 0x21, 0, 0, 0, 0},
          fuel_needle_max[] = {0x60, 5, 0x30, 0x22, 6, 7, 0x4E, 0},                                                                 // Set to 100%
          fuel_needle_min[] = {0x60, 5, 0x30, 0x22, 6, 0, 0, 0},                                                                    // Set to 0%
          fuel_needle_release[] = {0x60, 3, 0x30, 0x22, 0, 0, 0, 0},
          oil_needle_max[] = {0x60, 5, 0x30, 0x23, 6, 7, 0x12, 0},                                                                  // Set to 150 C
          oil_needle_min[] = {0x60, 5, 0x30, 0x23, 6, 0, 0, 0},                                                                     // Set to 0 C
          oil_needle_release[] = {0x60, 3, 0x30, 0x23, 0, 0, 0, 0};
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

  #if F_NBT
    uint8_t vol_request[] = {0x63, 5, 0x31, 1, 0xA0, 0x39, 0};
    vol_request_buf = make_msg_buf(0x6F1, 7, vol_request);
  #else
    uint8_t vol_request[] = {0x63, 3, 0x31, 0x24, 0, 0, 0, 0};
    vol_request_buf = make_msg_buf(0x6F1, 8, vol_request);
  #endif
  
  uint8_t door_open_cc_off[] = {0x40, 0x4F, 1, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
  door_open_cc_off_buf = make_msg_buf(0x5C0, 8, door_open_cc_off);

  uint8_t hdc_cc_activated_on[] = {0x40, 0x4B, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF},
          hdc_cc_unavailable_on[] = {0x40, 0x4D, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF},
          hdc_cc_deactivated_on[] = {0x40, 0x4C, 1, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF},
          hdc_cc_activated_off[] = {0x40, 0x4B, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF},
          hdc_cc_unavailable_off[] = {0x40, 0x4D, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF},
          hdc_cc_deactivated_off[] = {0x40, 0x4C, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF};
  hdc_cc_activated_on_buf = make_msg_buf(0x5A9, 8, hdc_cc_activated_on);
  hdc_cc_unavailable_on_buf = make_msg_buf(0x5A9, 8, hdc_cc_unavailable_on);
  hdc_cc_deactivated_on_buf = make_msg_buf(0x5A9, 8, hdc_cc_deactivated_on);
  hdc_cc_activated_off_buf = make_msg_buf(0x5A9, 8, hdc_cc_activated_off);
  hdc_cc_unavailable_off_buf = make_msg_buf(0x5A9, 8, hdc_cc_unavailable_off);
  hdc_cc_deactivated_off_buf = make_msg_buf(0x5A9, 8, hdc_cc_deactivated_off);

  uint8_t msa_deactivated_cc_on[] = {0x40, 0xC2, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF},
          msa_deactivated_cc_off[] = {0x40, 0xC2, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
  msa_deactivated_cc_on_buf = make_msg_buf(0x592, 8, msa_deactivated_cc_on);
  msa_deactivated_cc_off_buf = make_msg_buf(0x592, 8, msa_deactivated_cc_off);

  uint8_t camera_off[] = {0xA1, 0xFF}, camera_on[] = {0xA5, 0xFF}, 
          camera_inactive[] = {0x81, 0xFF},
          pdc_off_camera_on[] = {0x64, 4, 0x30, 9, 7, 4, 0, 0},
          pdc_on_camera_on[] = {0x64, 4, 0x30, 9, 7, 5, 0, 0},
          pdc_off_camera_off[] = {0x64, 4, 0x30, 9, 7, 0, 0, 0};
  camera_off_buf = make_msg_buf(0x3AE, 2, camera_off);
  camera_on_buf = make_msg_buf(0x3AE, 2, camera_on);
  camera_inactive_buf = make_msg_buf(0x3AE, 2, camera_inactive);
  pdc_off_camera_on_buf = make_msg_buf(0x6F1, 8, pdc_off_camera_on);
  pdc_on_camera_on_buf = make_msg_buf(0x6F1, 8, pdc_on_camera_on);
  pdc_off_camera_off_buf = make_msg_buf(0x6F1, 8, pdc_off_camera_off);
  pdc_button_presssed_buf = make_msg_buf(0x317, 2, generic_button_pressed);
  pdc_button_released_buf = make_msg_buf(0x317, 2, generic_button_released);

  uint8_t msa_fake_status[] = {0xFF, 0xFF};
  msa_fake_status_buf = make_msg_buf(0x308, 2, msa_fake_status);

  uint8_t mute_asd[] = {0x3F, 5, 0x31, 0xB8, 0xC, 1, 1, 0},
          demute_asd[] = {0x3F, 5, 0x31, 0xB8, 0xC, 1, 0, 0};
  mute_asd_buf = make_msg_buf(0x6F1, 8, mute_asd);
  demute_asd_buf = make_msg_buf(0x6F1, 8, demute_asd);

  nivi_button_pressed_buf = make_msg_buf(0x28A, 2, generic_button_pressed);
  nivi_button_released_buf = make_msg_buf(0x28A, 2, generic_button_released);

  uint8_t power_down_cmd_a[] = {0x40, 3, 0x22, 0x3F, 0, 0, 0, 0},
          power_down_cmd_b[] = {0x40, 0x30, 0, 0, 0, 0, 0, 0},
          power_down_cmd_c[] = {0xEF, 3, 0x31, 5, 0, 0, 0, 0};
  power_down_cmd_a_buf = make_msg_buf(0x6F1, 8, power_down_cmd_a);
  power_down_cmd_b_buf = make_msg_buf(0x6F1, 8, power_down_cmd_b);
  power_down_cmd_c_buf = make_msg_buf(0x6F1, 8, power_down_cmd_c);

  uint8_t f_oil_level_measuring[] = {0, 0xF0, 4, 0xC0};
  f_oil_level_measuring_buf = make_msg_buf(0x435, 4, f_oil_level_measuring);

  uint8_t f_hu_nbt_reboot[] = {0x63, 2, 0x11, 1, 0, 0, 0, 0};
  f_hu_nbt_reboot_buf = make_msg_buf(0x6F1, 8, f_hu_nbt_reboot);
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
    } else if (msg.buf[0] == 0x48) {                                                                                                // Exception for VSW01 diagnosis.
    } else {
      #if DEBUG_MODE
        serial_log("6F1 message not sent to KCAN due to OBD tool presence.", 2);
        can_debug_print_buffer(msg);
      #endif
      return;
    }
  }
  uint8_t result = KCAN.write(msg);
  if (result != 1) {
    if (kcan_retry_counter < 10) {                                                                                                  // Safeguard to avoid polluting the network in case of unrecoverable issue.
      m = {msg, millis() + 100};
      kcan_resend_txq.push(&m);
      kcan_retry_counter++;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "KCAN write failed for ID: %lX with error %d. Re-sending.", msg.id, result);
        serial_log(serial_debug_string, 1);
        can_debug_print_buffer(msg);
      #endif
    } else {
      serial_log("KCAN resend max counter exceeded.", 1);
    }
    kcan_error_counter++;
  } else {
    kcan_error_counter = 0;
    kcan_resend_txq.flush();
  }
}



void kcan2_write_msg(const CAN_message_t &msg) {
  #if F_NBT
    if (kcan2_mode == MCP_NORMAL && vehicle_awakened_timer >= 5000) {                                                               // Prevent writing to the bus when sleeping or waking up.
      byte send_buf[msg.len];

      // Sinkhole for unused messages.
      if (msg.id == 0xAA || msg.id == 0xA8) { return; }                                                                             // BN2000 engine status and torques.
      else if (msg.id == 0xC4 || msg.id == 0xC8) { return; }                                                                        // BN2000 steering angle.
      else if (msg.id == 0x2C0) { return; }                                                                                         // BN2000 LCD brightness.
      else if (msg.id == 0x317) { return; }                                                                                         // BN2000 PDC button.
      else if (msg.id == 0x31D) { return; }                                                                                         // BN2000 FTM status.
      else if (msg.id == 0x399) { return; }                                                                                         // BN2000 MDrive / BN2010 Status energy voltage.
      else if (msg.id >= 0x5FF && msg.id <= 0x662) { return; }                                                                      // Diagnosis response messages from various modules.
      else if (msg.id == 0x6F1 && !(msg.buf[0] == 0x63 || msg.buf[0] == 0x67 || msg.buf[0] == 0x35)) { return; }                    // 6F1s not meant for the NBT/ZBE/TBX.
      else if (msg.id >= 0x6F5) { return; }
      
      #if F_NBT_VIN_PATCH
        else if (msg.id == 0x380) {                                                                                                 // Patch NBT VIN to donor.
          if (donor_vin_initialized) {
            for (uint8_t i = 0; i < msg.len; i++) {
              send_buf[i] = DONOR_VIN[i];
            }
          } else {
            return;
          }
        }
      #endif 
      
      else {
        for (uint8_t i = 0; i < msg.len; i++) {
          send_buf[i] = msg.buf[i];
        }
      }

      if (CAN_OK != KCAN2.sendMsgBuf(msg.id, 0, msg.len, send_buf)) {
        #if DEBUG_MODE
          sprintf(serial_debug_string, "KCAN2 write failed for ID: %lX.", msg.id);
          serial_log(serial_debug_string, 1);
          can_debug_print_buffer(msg);
        #endif
        kcan2_error_counter++;
      }
    }
  #endif
}



void ptcan_write_msg(const CAN_message_t &msg) {
  if (msg.id == 0x6F1 && !diag_transmit) {
    if (msg.buf[0] == 0x17 && msg.buf[2] == 0x30 && (msg.buf[1] == 4 || msg.buf[1] == 2)) {                                         // Exception for EKP disable.
    } else if (msg.buf[0] == 0xE) {                                                                                                 // Exception for SVT70 diag.
    } else {
      #if DEBUG_MODE
        serial_log("6F1 message not sent to PTCAN due to OBD tool presence.", 2);
        can_debug_print_buffer(msg);
      #endif
      return;
    }
  }

  uint8_t result = PTCAN.write(msg);
  if (result != 1) {
    if (ptcan_retry_counter < 10) {                                                                                                 // Safeguard to avoid polluting the network in case of unrecoverable issue.
      m = {msg, millis() + 50};
      ptcan_resend_txq.push(&m);
      ptcan_retry_counter++;
      #if DEBUG_MODE
        sprintf(serial_debug_string, "PTCAN write failed for ID: %lX with error %d. Re-sending.", msg.id, result);
        serial_log(serial_debug_string, 1);
        can_debug_print_buffer(msg);
      #endif
    } else {
      serial_log("PTCAN resend max counter exceeded.", 1);
    }
    ptcan_error_counter++;
  } else {
    ptcan_retry_counter = 0;
    ptcan_resend_txq.flush();
  }
}


void dcan_write_msg(const CAN_message_t &msg) {
  if (vehicle_awake) {
    uint8_t result = DCAN.write(msg);
    if (result != 1) {
      if (dcan_retry_counter < 10) {                                                                                                // Safeguard to avoid polluting the network in case of unrecoverable issue.
        m = {msg, millis() + 50};
        dcan_resend_txq.push(&m);
        dcan_retry_counter++;
        #if DEBUG_MODE
          sprintf(serial_debug_string, "DCAN write failed for ID: %lX with error %d.", msg.id, result);
          serial_log(serial_debug_string, 1);
          can_debug_print_buffer(msg);
        #endif
      } else {
        serial_log("DCAN resend max counter exceeded.", 1);
      }
      dcan_error_counter++;
    } else {
      dcan_retry_counter = 0;
      dcan_resend_txq.flush();
    }
  }
  #if DEBUG_MODE
  else {
    sprintf(serial_debug_string, "DCAN write failed for ID: %lX because vehicle is asleep.", msg.id);
    serial_log(serial_debug_string, 1);
  }
  #endif
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
  if (LOGLEVEL >= 1) {
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    }
    Serial.println();
  }
}
#endif
