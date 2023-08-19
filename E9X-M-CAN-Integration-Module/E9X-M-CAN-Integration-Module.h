// Program settings and global variables go here.


#include <FlexCAN_T4.h>
#include <EEPROM.h>
#include "src/CRC/src/CRC8.h"                                                                                                       // https://github.com/RobTillaart/CRC
#include "src/CRC/src/CRC16.h"
#include "src/wdt4/Watchdog_t4.h"                                                                                                   // https://github.com/tonton81/WDT_T4
#include "src/queue/cppQueue.h"                                                                                                     // https://github.com/SMFSW/Queue
#include "usb_dev.h"
extern "C" uint32_t set_arm_clock(uint32_t frequency);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> KCAN;                                                                                     // RX: 32 messages with length 8, TX: 8 messages with length 8.
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64> PTCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_64> DCAN;
WDT_T4<WDT1> wdt;
const uint8_t wdt_timeout_sec = 10;
CRC16 teensy_eep_crc(0x1021, 0, 0, false, false);                                                                                   // XMODEM


/***********************************************************************************************************************************************************************************************************************************************
  Program configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#define DEBUG_MODE 1                                                                                                                // Toggle serial debug messages. Disable in production.

#define PWR_CKM 1                                                                                                                   // Persistently remember POWER when set in iDrive.
#define DOOR_VOLUME 1                                                                                                               // Reduce audio volume on door open. Also disables the door open with ignition warning CC.
#define RHD 1                                                                                                                       // Where does the driver sit?
#define FTM_INDICATOR 1                                                                                                             // Indicate FTM (Flat Tyre Monitor) status when using M3 RPA hazards button cluster.
#define HOOD_OPEN_GONG 1                                                                                                            // Plays CC gong warning when opening hood.
#define FRM_HEADLIGHT_MODE 1                                                                                                        // Switches FRM AHL mode from Komfort and Sport.
#define WIPE_AFTER_WASH 1                                                                                                           // One more wipe cycle after washing the windscreen.
#define AUTO_MIRROR_FOLD 1                                                                                                          // Fold/Unfold mirrors when locking. Can be done with coding but this integrates nicer.
#if AUTO_MIRROR_FOLD
#define UNFOLD_WITH_DOOR 1                                                                                                          // Un-fold with door open event instead of unlock button.
#endif
#define INDICATE_TRUNK_OPENED 1                                                                                                     // Flash hazards when remote opens trunk.
#define IMMOBILIZER_SEQ 1                                                                                                           // Disable fuel pump until the steering wheel M button is pressed a number of times.
#if __has_include ("src/secrets.h")                                                                                                 // Optionally, create this file to store sensitive settings.
  #include "src/secrets.h"
#endif
#if IMMOBILIZER_SEQ                                                                                                                 // If this is not deactivated before start, DME will store errors!
#if SECRETS
  uint8_t IMMOBILIZER_SEQ_NUMBER = SECRET_IMMOBILIZER_SEQ;
#else
  uint8_t IMMOBILIZER_SEQ_NUMBER = 3;                                                                                               // Number of times to press the button for the EKP to be re-activated.
#endif
#define IMMOBILIZER_SEQ_ALARM 1                                                                                                     // Sound the alarm if engine is started without disabling anti-theft.
#endif
#if IMMOBILIZER_SEQ_ALARM
#if SECRETS
  uint8_t IMMOBILIZER_SEQ_ALARM_NUMBER = SECRET_IMMOBILIZER_SEQ_ALARM_NUMBER;
#else
  const uint8_t IMMOBILIZER_SEQ_ALARM_NUMBER = 6;                                                                                   // Number of times to press the button for the alarm to be silenced and EKP re-activated.
#endif
#endif
#define FRONT_FOG_LED_INDICATOR 1                                                                                                   // Turn ON an external LED when front fogs are ON. M3 clusters lack this indicator.
#define FRONT_FOG_CORNER 1                                                                                                          // Turn ON/OFF corresponding fog light when turning.
#if FRONT_FOG_CORNER
const int8_t FOG_CORNER_STEERTING_ANGLE = 87;                                                                                       // Steering angle at which to activate fog corner function.
const int8_t STEERTING_ANGLE_HYSTERESIS = 48;
const int8_t FOG_CORNER_STEERTING_ANGLE_INDICATORS = 45;
const int8_t STEERTING_ANGLE_HYSTERESIS_INDICATORS = 15;
#endif
#define DIM_DRL 1                                                                                                                   // Dims DLR ON the side that the indicator is ON.
#define SERVOTRONIC_SVT70 1                                                                                                         // Control steering assist with modified SVT70 module.
#define EXHAUST_FLAP_CONTROL 1                                                                                                      // Take control of the exhaust flap solenoid.
#if EXHAUST_FLAP_CONTROL
#define QUIET_START 1                                                                                                               // Close the exhaust valve as soon as Terminal R is turned ON.
#endif
#define LAUNCH_CONTROL_INDICATOR 1                                                                                                  // Show launch control indicator (use with MHD lauch control, 6MT).
#define CONTROL_SHIFTLIGHTS 1                                                                                                       // Display shiftlights, animation and sync with the variable redline of M3 KOMBI.
#define NEEDLE_SWEEP 1                                                                                                              // Needle sweep animation with engine ON. Calibrated for M3 speedo with 335i tacho.
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver at low temperatures.
#if AUTO_SEAT_HEATING
#define AUTO_SEAT_HEATING_PASS 1                                                                                                    // Enable automatic heated seat for passenger at low temperatures.
#endif
#define RTC 1                                                                                                                       // Sets the time/date if power is lost. Requires external battery.
#if SERVOTRONIC_SVT70
  const uint16_t SVT_FAKE_EDC_MODE_CANID = 0x327;                                                                                   // New CAN-ID replacing 0x326 in SVT70 firmware bin. This stops it from changing modes together with EDC.
#endif
const uint16_t DME_FAKE_VEH_MODE_CANID = 0x31F;                                                                                     // New CAN-ID replacing 0x315 in DME [Program] section of the firmware.
const double AUTO_SEAT_HEATING_TRESHOLD = 10.0;                                                                                     // Degrees Celsius temperature.
const uint16_t AUTO_HEATING_START_DELAY = 5 * 1000;                                                                                 // Time to wait for battery voltage to catch up after starting (time in seconds * 1000)
#if CONTROL_SHIFTLIGHTS
  const uint16_t START_UPSHIFT_WARN_RPM = 5500 * 4;                                                                                 // RPM setpoints when warmed up (warning = desired RPM * 4).
  const uint16_t MID_UPSHIFT_WARN_RPM = 6000 * 4;
  const uint16_t MAX_UPSHIFT_WARN_RPM = 6500 * 4;
  const uint16_t GONG_UPSHIFT_WARN_RPM = 7000 * 4;
  const int16_t VAR_REDLINE_OFFSET_RPM = -300;                                                                                      // RPM difference between DME requested redline and KOMBI displayed redline. Varies with cluster.
#endif
#if EXHAUST_FLAP_CONTROL
  const uint16_t EXHAUST_FLAP_QUIET_RPM = 3000 * 4;                                                                                 // RPM setpoint to open the exhaust flap in normal mode (desired RPM * 4).
#endif
#if LAUNCH_CONTROL_INDICATOR
  const uint16_t LC_RPM = 4000 * 4;                                                                                                 // RPM setpoint to display launch control flag CC (desired RPM * 4). Match with MHD setting.
  const uint16_t LC_RPM_MIN = LC_RPM - (250 * 4);                                                                                   // RPM hysteresis when bouncing off the limiter.
  const uint16_t LC_RPM_MAX = LC_RPM + (250 * 4);
#endif
const float MAX_THRESHOLD = 72.0;                                                                                                   // CPU temperature thresholds for the processor clock scaling function.
const float HIGH_THRESHOLD = 68.0;
const float MEDIUM_THRESHOLD = 66.0;
const float MILD_THRESHOLD = 62.0;
const float HYSTERESIS = 2.0;
const unsigned long MAX_UNDERCLOCK = 24 * 1000000;                                                                                  // temperature <= 90 (Tj) should be ok for more than 100,000 Power on Hours at 1.15V (freq <= 528 MHz).
unsigned long HIGH_UNDERCLOCK = 150 * 1000000;
const unsigned long HIGH_UNDERCLOCK_ = 150 * 1000000;
unsigned long MEDIUM_UNDERCLOCK = 396 * 1000000;
const unsigned long MEDIUM_UNDERCLOCK_ = 396 * 1000000;
unsigned long MILD_UNDERCLOCK = 450 * 1000000;
const unsigned long MILD_UNDERCLOCK_ = 450 * 1000000;
const unsigned long STANDARD_CLOCK = 528 * 1000000;
const unsigned long CRITICAL_UNDERVOLT = ((0.9 - 0.8) * 1000) / 25;                                                                 // Desired voltage - 0.8...  DCDC range is from 0.8 to 1.575V. Safe range is 0.925 to 1.25V


/***********************************************************************************************************************************************************************************************************************************************
  These features are *very* implementation specific - usually requiring fabrication. As such they are disabled by default since they're unlikely to be used by anyone except me.
***********************************************************************************************************************************************************************************************************************************************/

#if __has_include ("src/custom-settings.h")
  #include "src/custom-settings.h"
#endif
#if !DEBUG_MODE
  #ifndef USB_DISABLE
    #define USB_DISABLE 0                                                                                                           // USB can be disabled if not using serial for security reasons. Use caution when enabling this.
      // USB can be re-activated by holding POWER while turning on ignition.
      // To make this work, the following changes need to be made to startup.c 
      // (Linux path: /home/**Username**/.arduino15/packages/teensy/hardware/avr/**Version**/cores/teensy4/startup.c)
      // Comment these lines:
      // usb_pll_start();
      // while (millis() < TEENSY_INIT_USB_DELAY_BEFORE) ;
      // usb_init();
      // while (millis() < TEENSY_INIT_USB_DELAY_AFTER + TEENSY_INIT_USB_DELAY_BEFORE) ; // wait
  #endif
#endif
#ifndef EDC_CKM_FIX
  #define EDC_CKM_FIX 0                                                                                                             // Sometimes the M Key setting for EDC is not recalled correctly - especially with CA.
#endif
#ifndef HDC
  #define HDC 0                                                                                                                     // Gives a function to the HDC console button in non 4WD cars.
#endif
#ifndef FAKE_MSA
  #define FAKE_MSA 0                                                                                                                // Display Auto Start-Stop OFF CC message when the Auto Start-Stop button is pressed. Must be coded in IHK.
  #if !FAKE_MSA
    #define MSA_RVC 0                                                                                                               // Turn on the OEM rear camera (TRSVC and E84 PDC ECUs) when pressing MSA button. E70 button from 61319202037.
  #endif                                                                                                                            // RVC can be controlled independently of PDC with this button.
#endif
#ifndef REVERSE_BEEP
  #define REVERSE_BEEP 0                                                                                                            // Play a beep throught the speaker closest to the driver when engaging reverse.
#endif
#ifndef AUTO_STEERING_HEATER
  #define AUTO_STEERING_HEATER 0                                                                                                    // Enable automatic heated steering wheel at low temperatures.
#endif
#ifndef F_ZBE_WAKE
  #define F_ZBE_WAKE 0                                                                                                              // Enable/disable FXX CIC ZBE wakeup functions. Do not use with an EXX ZBE.
  // *Should* work with:
  // 6582 9267955 - 4-pin MEDIA button. **Tested - controller build date 19.04.13
  // 6131 9253944 - 4-pin CD button
  // 6582 9206444 - 10-pin CD button
  // 6582 9212449 - 10-pin CD button
#endif
#ifndef F_VSW01
  #define F_VSW01 0                                                                                                                 // Enable/disable F01 Video Switch diagnosis and wakeup. Tested with 9201542.
#endif
#ifndef F_NIVI
  #define F_NIVI 0                                                                                                                  // Enable/disable FXX NVE diagnosis, wakeup and BN2000->BN2010 message translation.
#endif


/***********************************************************************************************************************************************************************************************************************************************
  Board configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#define POWER_BUTTON_PIN 2
#define POWER_LED_PIN 3
#define FOG_LED_PIN 4
#define DCAN_STBY_PIN 14
#define PTCAN_STBY_PIN 15                                                                                                           // STBY pins are optional but recommended to save power when the car prepares to deep sleep.
#define DSC_BUTTON_PIN 16
#define POWER_BUTTON_PIN 2
#define POWER_LED_PIN 3
#define FOG_LED_PIN 4
#if AUTO_STEERING_HEATER
  #define STEERING_HEATER_SWITCH_PIN 5
#endif
#define DCAN_STBY_PIN 14
#define PTCAN_STBY_PIN 15                                                                                                           // STBY pins are optional but recommended to save power when the car prepares to deep sleep.
#define DSC_BUTTON_PIN 16
#if EXHAUST_FLAP_CONTROL
  #define EXHAUST_FLAP_SOLENOID_PIN 17
#endif

/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/


#if !USB_DISABLE                                                                                                                    // Not needed if startup.c is modified.
  #if AUTO_MIRROR_FOLD || INDICATE_TRUNK_OPENED
    extern "C" void startup_middle_hook(void);
    extern "C" volatile uint32_t systick_millis_count;
    void startup_middle_hook(void) {
      // Force millis() to be 300 to skip USB startup delays. The module needs to boot very quickly to receive the unlock message.
      // This makes receiving early boot serial messages difficult.
      systick_millis_count = 300;
    }
  #endif
#endif

CAN_message_t pt_msg, k_msg, d_msg;
typedef struct delayed_can_tx_msg {
	CAN_message_t	tx_msg;
	unsigned long	transmit_time;
} delayed_can_tx_msg;
cppQueue kcan_resend_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
cppQueue ptcan_resend_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
cppQueue dcan_resend_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint8_t kcan_retry_counter = 0, ptcan_retry_counter = 0, dcan_retry_counter = 0;
uint16_t stored_eeprom_checksum = 0xFFFF, calculated_eeprom_checksum = 0;
delayed_can_tx_msg delayed_tx, m;
unsigned long time_now;
bool key_valid = false, terminal_r = false, ignition = false, vehicle_awake = false, vehicle_moving = false;
bool terminal_50 = false, engine_running = false, clutch_pressed = false, frm_consumer_shutdown = false;
bool clearing_dtcs = false;
unsigned long engine_runtime = 0;
float battery_voltage = 0;
elapsedMillis vehicle_awake_timer = 0, vehicle_awakened_time = 0;
CAN_message_t dsc_on_buf, dsc_mdm_dtc_buf, dsc_off_buf;
cppQueue dsc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint16_t RPM = 0;
#if PWR_CKM
  uint8_t dme_ckm[4][2] = {{0, 0xFF}, {0, 0xFF}, {0, 0xFF}, {0xF1, 0xFF}};
#endif
#if EDC_CKM_FIX
  uint8_t edc_ckm[] = {0, 0, 0, 0xF1};
  uint8_t edc_mismatch_check_counter = 0;
  CAN_message_t edc_button_press_buf;
  cppQueue edc_ckm_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if PWR_CKM || EDC_CKM_FIX
  uint8_t cas_key_number = 0;                                                                                                       // 0 = Key 1, 1 = Key 2...
#endif
uint8_t mdrive_dsc = 3, mdrive_power = 0, mdrive_edc = 0x20, mdrive_svt = 0xE9;
bool mdrive_status = false, mdrive_power_active = false;
bool console_power_mode, restore_console_power_mode = false;
bool power_led_delayed_off_action = false;
unsigned long power_led_delayed_off_action_time;
uint8_t power_mode_only_dme_veh_mode[] = {0xE8, 0xF1};                                                                              // E8 is the last checksum. Start will be from 0A.
uint8_t dsc_program_status = 0;                                                                                                     // 0 = ON, 1 = DTC, 2 = DSC OFF
bool holding_dsc_off_console = false;
elapsedMillis mdrive_message_timer = 0;
uint8_t m_mfl_held_count = 0;
CAN_message_t idrive_mdrive_settings_a_buf, idrive_mdrive_settings_b_buf;
const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 500, dsc_hold_time_ms = 300;
elapsedMillis power_button_debounce_timer = power_debounce_time_ms;
elapsedMillis dsc_off_button_debounce_timer = dsc_debounce_time_ms, dsc_off_button_hold_timer = 0;
bool ignore_m_press = false, ignore_m_hold = false, holding_both_console = false;
uint8_t clock_mode = 0;
float last_cpu_temp = 0, max_cpu_temp = 0;
CAN_message_t cc_gong_buf;
#if SERVOTRONIC_SVT70
  bool uif_read = false;
  uint8_t servotronic_message[] = {0, 0xFF};
#endif
#if CONTROL_SHIFTLIGHTS
  CAN_message_t shiftlights_start_buf, shiftlights_mid_buildup_buf, shiftlights_startup_buildup_buf;
  CAN_message_t shiftlights_max_flash_buf, shiftlights_off_buf;
  bool shiftlights_segments_active = false;
  uint8_t ignore_shiftlights_off_counter = 0;
  uint16_t START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
  uint16_t MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
  uint16_t MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
  uint16_t GONG_UPSHIFT_WARN_RPM_ = GONG_UPSHIFT_WARN_RPM;
  bool engine_coolant_warmed_up = false;
  uint16_t var_redline_position = 0;
  uint8_t last_var_rpm_can = 0;
  uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x97};                                                                                   // Byte5: shiftlights always on
#else
  uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x87};                                                                                   // Byte5: shiftlights always off
#endif
#if NEEDLE_SWEEP
  CAN_message_t speedo_needle_max_buf, speedo_needle_min_buf, speedo_needle_release_buf;
  CAN_message_t tacho_needle_max_buf, tacho_needle_min_buf, tacho_needle_release_buf;
  CAN_message_t fuel_needle_max_buf, fuel_needle_min_buf, fuel_needle_release_buf;
  CAN_message_t oil_needle_max_buf, oil_needle_min_buf, oil_needle_release_buf;
  CAN_message_t any_needle_max_b_buf;
  cppQueue kombi_needle_txq(sizeof(delayed_can_tx_msg), 32, queue_FIFO);
#endif
#if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER
  bool front_fog_status = false;
#endif
#if FRONT_FOG_CORNER || INDICATE_TRUNK_OPENED
  bool indicators_on = false;
#endif
#if FRONT_FOG_CORNER
  unsigned long last_fog_action_timer = 15000;
  bool dipped_beam_status = false, left_fog_on = false, right_fog_on = false;
  int16_t steering_angle = 0;
  CAN_message_t front_left_fog_on_a_buf, front_left_fog_on_b_buf, front_left_fog_on_c_buf, front_left_fog_on_d_buf;
  CAN_message_t front_left_fog_on_a_softer_buf, front_left_fog_on_b_softer_buf, front_left_fog_on_c_softer_buf;
  CAN_message_t front_left_fog_on_d_softer_buf, front_left_fog_on_e_softer_buf, front_left_fog_on_f_softer_buf;
  CAN_message_t front_left_fog_on_g_softer_buf, front_left_fog_on_h_softer_buf;
  CAN_message_t front_right_fog_on_a_softer_buf, front_right_fog_on_b_softer_buf, front_right_fog_on_c_softer_buf;
  CAN_message_t front_right_fog_on_d_softer_buf, front_right_fog_on_e_softer_buf, front_right_fog_on_f_softer_buf;
  CAN_message_t front_right_fog_on_g_softer_buf, front_right_fog_on_h_softer_buf;
  CAN_message_t front_right_fog_on_a_buf, front_right_fog_on_b_buf, front_right_fog_on_c_buf, front_right_fog_on_d_buf;  
  CAN_message_t front_left_fog_off_buf, front_right_fog_off_buf, front_fogs_all_off_buf;
  cppQueue fog_corner_left_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
  cppQueue fog_corner_right_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if DIM_DRL
  unsigned long last_drl_action_timer = 15000;
  bool drl_status = false, left_dimmed = false, right_dimmed = false;
  CAN_message_t left_drl_dim_off, left_drl_dim_buf, left_drl_bright_buf;
  CAN_message_t right_drl_dim_off, right_drl_dim_buf, right_drl_bright_buf;
  cppQueue dim_drl_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if FTM_INDICATOR
  bool ftm_indicator_status = false;
  CAN_message_t ftm_indicator_flash_buf, ftm_indicator_off_buf;
#endif
#if HOOD_OPEN_GONG
  uint8_t last_hood_status = 0;
#endif
#if FRM_HEADLIGHT_MODE
  CAN_message_t frm_ckm_ahl_komfort_buf, frm_ckm_ahl_sport_buf;
#endif
#if WIPE_AFTER_WASH
  bool wipe_scheduled = false;
  uint8_t wash_message_counter = 0;
  CAN_message_t wipe_single_buf;
  cppQueue wiper_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if AUTO_MIRROR_FOLD
  uint8_t mirror_status_retry = 0;
  bool mirrors_folded = false, frm_mirror_status_requested = false;
  bool lock_button_pressed  = false, unlock_button_pressed = false;
  CAN_message_t frm_status_request_a_buf, frm_status_request_b_buf;
  CAN_message_t frm_toggle_fold_mirror_a_buf, frm_toggle_fold_mirror_b_buf;
  cppQueue mirror_fold_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if AUTO_MIRROR_FOLD || INDICATE_TRUNK_OPENED
  uint8_t last_lock_status_can = 0;
#endif
#if UNFOLD_WITH_DOOR
  bool unfold_with_door_open = false;
#endif
#if INDICATE_TRUNK_OPENED
  CAN_message_t flash_hazards_buf;
  bool visual_signal_ckm, hazards_on = false;
#endif
#if IMMOBILIZER_SEQ
  bool immobilizer_released = false, immobilizer_persist = true;
  unsigned long immobilizer_send_interval = 0;                                                                                      // Skip the first interval delay.
  elapsedMillis immobilizer_timer = 0, both_console_buttons_timer, immobilizer_activate_release_timer = 0;
  uint8_t immobilizer_pressed_release_count = 0, immobilizer_pressed_activate_count = 0;
  CAN_message_t key_cc_on_buf, key_cc_off_buf;
  CAN_message_t start_cc_on_buf, start_cc_off_buf;
  CAN_message_t ekp_pwm_off_buf, ekp_return_to_normal_buf;
  cppQueue immobilizer_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
  cppQueue ekp_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if IMMOBILIZER_SEQ_ALARM
  bool alarm_after_engine_stall = false, alarm_active = false, alarm_led = false, lock_led = false;
  uint8_t led_message_counter = 60;
  CAN_message_t alarm_led_on_buf, alarm_led_off_buf;
  CAN_message_t alarm_siren_on_buf, alarm_siren_off_buf;
  cppQueue alarm_siren_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if REVERSE_BEEP
  bool pdc_beep_sent = false, pdc_too_close = false;
#endif
#if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR || FRONT_FOG_CORNER || MSA_RVC || F_NIVI
  bool reverse_gear_status = false;
#endif
#if FRONT_FOG_CORNER || F_NIVI
  bool rls_headlights_requested = false;
#endif
#if F_ZBE_WAKE || F_VSW01 || F_NIVI
  uint8_t f_terminal_status_alive_counter = 0;
  uint8_t f_terminal_status[] = {0, 0, 0, 0xFF, 0, 0, 0x3F, 0xFF};                                                                  // These messages do not exist in BN2000.
  uint8_t f_vehicle_mode[] = {0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0};
  elapsedMillis f_vehicle_mode_timer = 5000;
  CAN_message_t f_kombi_network_mgmt_buf, f_terminal_status_buf, f_vehicle_mode_buf;
  CRC8 f_terminal_status_crc(0x1D, 0, 0xB1, false, false);                                                                          // SAE J1850 POLY, 0 init and XOR-OUT 0xB1 for ARB-ID 0x12F.
#endif
#if F_ZBE_WAKE
  uint8_t zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
#endif
#if F_VSW01
  bool vsw_initialized = false;
  uint8_t vsw_current_input = 0, vsw_switch_counter = 0xF1;
  // uint16_t idrive_current_menu;
  // CAN_message_t idrive_menu_request_buf;
#endif
#if F_NIVI
  float sine_tilt_angle = 0;
  bool sine_angle_requested = false;
  int f_vehicle_angle = 0x500;                                                                                                      // 0 deg.
  uint8_t f_road_inclination[] = {0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF};                                                        // Angle formula simlar to 56.1.2.
  float e_long_acceleration = 0;
  int longitudinal_acceleration = 0x7EF4;                                                                                           // 32500 * 0.002 - 65 = 0 m/s^2.
  uint8_t f_longitudinal_acceleration[] = {0xFF, 0xFF, 0, 0, 0xFF, 0x2F};                                                           // Similar to 55.0.2. Byte7 fixed 0x2F - Signal value is valid QU_ACLNX_COG.
  uint8_t f_longitudinal_acceleration_alive_counter = 0;
  int yaw_rate = 0xA4;                                                                                                              // Nearly 0 deg/s.
  uint8_t f_yaw_rate[] = {0xFF, 0xFF, 0, 0, 0xFF, 0x2F};                                                                            // M4 (VYAW_VEH) 56.0.2. Byte5 fixed 0x2F - Signal value is valid QU_VYAW_VEH.
  uint8_t f_yaw_alive_counter = 0;
  uint16_t real_speed = 0;
  uint8_t vehicle_direction = 0;
  uint8_t f_speed[] = {0, 0, 0, 0, 0};                                                                                              // Message is the same format as Flexray 55.3.4.
  uint8_t f_speed_alive_counter = 0;
  uint8_t rls_brightness = 0xFE, rls_time_of_day = 0;
  uint8_t f_outside_brightness[] = {0xFE, 0xFE};                                                                                    // Daytime?. The two bytes may represent the two photosensors (driver's/passenger's side in FXX).
  uint8_t f_data_powertrain_2_alive_counter = 0;
  uint8_t f_data_powertrain_2[] = {0, 0, 0, 0, 0, 0, 0, 0x8C};                                                                      // Byte7 max rpm: 50 * 8C = 7000.
  elapsedMillis sine_angle_request_timer = 500, f_outside_brightness_timer = 500, f_data_powertrain_2_timer = 1000;
  elapsedMillis f_chassis_inclination_timer = 98, f_chassis_longitudinal_timer = 100, f_chassis_yaw_timer = 102;
  elapsedMillis f_chassis_speed_timer = 104;
  CAN_message_t sine_angle_request_a_buf, sine_angle_request_b_buf;
  CAN_message_t f_road_inclination_buf, f_longitudinal_acceleration_buf, f_yaw_rate_buf;
  CAN_message_t f_speed_buf, f_outside_brightness_buf, f_data_powertrain_2_buf;
  CRC8 f_speed_crc(0x1D, 0, 0xF, false, false);                                                                                     // SAE J1850 POLY, 0 init and XOR-OUT 0xF for ARB-ID 0x1A1.
  CRC8 f_data_powertrain_2_crc(0x1D, 0, 4, false, false);                                                                           // SAE J1850 POLY, 0 init and XOR-OUT 4 for ARB-ID 0x3F9.
#endif
#if EXHAUST_FLAP_CONTROL
  bool exhaust_flap_sport = false, exhaust_flap_open = true;
  elapsedMillis exhaust_flap_action_timer;                                                                                          // Initialized when the engine is started.
  unsigned long exhaust_flap_action_interval = 1000;
#endif
#if LAUNCH_CONTROL_INDICATOR
  CAN_message_t lc_cc_on_buf, lc_cc_off_buf;
  bool lc_cc_active = false, mdm_with_lc = false;
#endif
float ambient_temperature_real = 87.5;
#if AUTO_SEAT_HEATING
  bool driver_seat_heating_status = false;
  bool driver_sent_seat_heating_request = false;
  CAN_message_t seat_heating_button_pressed_dr_buf, seat_heating_button_released_dr_buf;
  cppQueue seat_heating_dr_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if AUTO_SEAT_HEATING_PASS
  bool passenger_seat_heating_status = false;
  uint8_t passenger_seat_status = 0;                                                                                                // 0 - Not occupied not belted, 1 - not occupied and belted, 8 - occupied not belted, 9 - occupied and belted
  bool passenger_sent_seat_heating_request = false;
  CAN_message_t seat_heating_button_pressed_pas_buf, seat_heating_button_released_pas_buf;
  cppQueue seat_heating_pas_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if AUTO_STEERING_HEATER
  bool sent_steering_heating_request = false, transistor_active = false;
  elapsedMillis transistor_active_timer;
#endif
#if RTC
  #include <TimeLib.h>
  CAN_message_t set_time_cc_buf, set_time_cc_off_buf;
  bool rtc_valid = true, pc_time_incoming = false;
#endif
#if DOOR_VOLUME
  bool volume_reduced = false, initial_volume_set = false;
  uint8_t volume_restore_offset = 0, volume_changed_to, peristent_volume = 0;
  elapsedMillis volume_request_timer = 3000;
  CAN_message_t vol_request_buf, door_open_cc_off_buf;
  cppQueue idrive_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if DOOR_VOLUME || AUTO_MIRROR_FOLD || IMMOBILIZER_SEQ
  bool left_door_open = false, right_door_open = false;
  uint8_t last_door_status = 0;
#endif
#if PWR_CKM || DOOR_VOLUME || REVERSE_BEEP || F_VSW01
  elapsedMillis idrive_alive_timer = 0;
  bool idrive_died = false;
#endif
#if HDC || IMMOBILIZER_SEQ || FRONT_FOG_CORNER || F_NIVI
  uint16_t indicated_speed = 0;
  bool speed_mph = false;
#endif
#if HDC
  uint8_t max_hdc_speed = 35, hdc_deactivate_speed = 60, stalk_message_counter = 0;
  uint8_t set_hdc_checksums[] = {0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0, 2, 3, 4};
  uint8_t cancel_hdc_checksums[] = {0xFD, 0xFE, 0xFF, 0, 2, 3, 4, 5, 6, 7, 8, 9, 0xA, 0xB, 0xC};
  bool cruise_control_status = false, hdc_button_pressed = false, hdc_requested = false, hdc_active = false;
  CAN_message_t set_hdc_cruise_control_buf, cancel_hdc_cruise_control_buf;
  CAN_message_t hdc_cc_activated_on_buf, hdc_cc_unavailable_on_buf, hdc_cc_deactivated_on_buf;
  CAN_message_t hdc_cc_activated_off_buf, hdc_cc_unavailable_off_buf, hdc_cc_deactivated_off_buf;
  cppQueue hdc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if FAKE_MSA
  CAN_message_t msa_deactivated_cc_on_buf, msa_deactivated_cc_off_buf;
#endif
#if MSA_RVC
  uint8_t pdc_status = 0x80;
  bool pdc_button_pressed = false, pdc_with_rvc_requested = false;
  CAN_message_t camera_off_buf, camera_on_buf, pdc_off_camera_on_buf, pdc_on_camera_on_buf, pdc_off_camera_off_buf;
#endif
#if FAKE_MSA || MSA_RVC
  bool msa_button_pressed = false;
  uint8_t msa_fake_status_counter = 0;
  CAN_message_t msa_fake_status_buf;
#endif
#if HDC || FAKE_MSA
  cppQueue ihk_extra_buttons_cc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
#endif
#if DEBUG_MODE
  extern float tempmonGetTemp(void);
  char serial_debug_string[512];
  elapsedMillis debug_print_timer = 500;
  unsigned long max_loop_timer = 0, loop_timer = 0, setup_time = 0;
  uint32_t kcan_error_counter = 0, ptcan_error_counter = 0, dcan_error_counter = 0;
  bool serial_commands_unlocked = false;
  #if SECRETS
    String serial_password = secret_serial_password;
  #else
    String serial_password = "coldboot";                                                                                            // Default password.
  #endif
  cppQueue serial_diag_txq(sizeof(delayed_can_tx_msg), 384, queue_FIFO);
#endif
bool diag_transmit = true;
elapsedMillis diag_deactivate_timer;
