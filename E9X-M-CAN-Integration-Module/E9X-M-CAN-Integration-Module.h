// Program settings and global variables go in this file.


/***********************************************************************************************************************************************************************************************************************************************
  Program configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#define DEBUG_MODE 1                                                                                                                // Toggle serial debug messages. Disable in production.
uint8_t LOGLEVEL = 4;                                                                                                               // 0 - critical, 1 - errors, 2 - info, 3 - extra_info, 4 = debug.

#define PDC_AUTO_OFF 1                                                                                                              // Deactivates PDC when handbrake is pulled.
#define AUTO_TOW_VIEW_RVC 1                                                                                                         // Turn on top down (tow view) rear view camera option when close to an obstacle.
#define DOOR_VOLUME 1                                                                                                               // Reduce audio volume on door open. Must code KOMBI to disable door open with ignition CC.
#define RHD 1                                                                                                                       // Where does the driver sit?
#define FTM_INDICATOR 1                                                                                                             // Indicate FTM (Flat Tyre Monitor) status when using M3 RPA hazards button cluster. Do not use with RDC.
#define HOOD_OPEN_GONG 1                                                                                                            // Plays CC gong warning when opening hood. NBT: also shows CC dialog.
#define FRM_AHL_MODE 1                                                                                                              // Switches FRM AHL mode from Komfort and Sport.
#define WIPE_AFTER_WASH 1                                                                                                           // One more wipe cycle after washing the windscreen.
#define INTERMITTENT_WIPERS 1                                                                                                       // Inermittent wiping alongside auto wipers when holding the stalk down for 1.3s.
#define AUTO_MIRROR_FOLD 1                                                                                                          // Fold/Unfold exterior mirrors when locking. Un-fold with door open event instead of remote unlock button.
#define MIRROR_UNDIM 1                                                                                                              // Undim electrochromic exterior mirrors when indicating at night.
#define COMFORT_EXIT 1                                                                                                              // Move driver's seat back when exiting car.
#define IMMOBILIZER_SEQ 1                                                                                                           // Disable EKP until the M button is pressed X times. Sound the alarm if engine ON without disabling immobilizer.
#define FRONT_FOG_LED_INDICATOR 1                                                                                                   // Turn ON an external LED when front fogs are ON. M3 clusters lack an indicator.
#define FRONT_FOG_CORNER 1                                                                                                          // Turn ON/OFF corresponding fog light when turning with Adaptive Headlights.
#define DIM_DRL 1                                                                                                                   // Dims DLR ON the side that the indicator is ON.
#define SERVOTRONIC_SVT70 1                                                                                                         // Control steering assist with modified SVT70 module.
#define EXHAUST_FLAP_CONTROL 1                                                                                                      // Take control of the exhaust flap solenoid.
#define QUIET_START 0                                                                                                               // Close the exhaust valve as soon as Terminal R is turned ON.
#define LAUNCH_CONTROL_INDICATOR 1                                                                                                  // Show launch control indicator (use with MHD lauch control, 6MT).
#define CONTROL_SHIFTLIGHTS 1                                                                                                       // Display shiftlights, animation and sync with the variable redline of M3 KOMBI.
#define NEEDLE_SWEEP 1                                                                                                              // Needle sweep animation with engine ON. Calibrated for M3 speedo with 335i tacho.
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver at low temperatures.
#define AUTO_SEAT_HEATING_PASS 1                                                                                                    // Enable automatic heated seat for passenger at low temperatures.
#define PTC_HEATER 1                                                                                                                // Convert alternator status from DME into the missing PWM "E_ZH" signal needed for the PTC auxiliary heater.
#define MDSC_ZB 0                                                                                                                   // If the MK60E5 is flashed with a 1M or M3 file, MDM is toggled through MDrive status (0x399) only.
                                                                                                                                    // For DSC OFF, the switch must be wired to pin 41. Non-M DSC modules flashed to M3 ZB need a 12V pull-up.

const float FOG_CORNER_STEERTING_ANGLE = 87.0;                                                                                      // Steering angle at which to activate fog corner function.
const float STEERTING_ANGLE_HYSTERESIS = 48.0;
const float FOG_CORNER_STEERTING_ANGLE_INDICATORS = 45.0;
const float STEERTING_ANGLE_HYSTERESIS_INDICATORS = 15.0;
const uint16_t SVT_FAKE_EDC_MODE_CANID = 0x327;                                                                                     // New CAN-ID replacing 0x326 in SVT70 firmware bin. This stops it from changing modes together with EDC.
const uint16_t DME_FAKE_VEH_MODE_CANID = 0x31F;                                                                                     // New CAN-ID replacing 0x315 in DME [Program] section of the firmware.
const double AUTO_SEAT_HEATING_THRESHOLD_HIGH = 12.0;                                                                               // Degrees Celsius temperature, heater fully on.
const double AUTO_SEAT_HEATING_THRESHOLD_MEDIUM = 15.0;                                                                             // Degrees Celsius temperature, heater on middle position.
const uint16_t AUTO_HEATING_START_DELAY = 2 * 1000;                                                                                 // Time to wait for battery voltage to catch up after starting (time in seconds * 1000)
const uint16_t START_UPSHIFT_WARN_RPM = 5500 * 4;                                                                                   // RPM setpoints when warmed up (warning = desired RPM * 4).
const uint16_t MID_UPSHIFT_WARN_RPM = 6000 * 4;
const uint16_t MAX_UPSHIFT_WARN_RPM = 6500 * 4;
const uint16_t GONG_UPSHIFT_WARN_RPM = 7000 * 4;
const int16_t VAR_REDLINE_OFFSET_RPM = -300;                                                                                        // RPM difference between DME requested redline and KOMBI displayed redline. Varies with cluster.
const uint16_t EXHAUST_FLAP_QUIET_RPM = 2500 * 4;                                                                                   // RPM setpoint to open the exhaust flap in normal mode (desired RPM * 4).
const uint16_t LC_RPM = 3700 * 4;                                                                                                   // RPM setpoint to display launch control flag CC (desired RPM * 4). Match with MHD setting.
const uint16_t LC_RPM_MIN = LC_RPM - (200 * 4);                                                                                     // RPM hysteresis when bouncing off the limiter.
const uint16_t LC_RPM_MAX = LC_RPM + (200 * 4);
const float MAX_THRESHOLD = 78.0;                                                                                                   // CPU temperature thresholds for the processor clock scaling function.
const float HIGH_THRESHOLD = 74.0;
const float MEDIUM_THRESHOLD = 70.0;
const float MILD_THRESHOLD = 66.0;
const unsigned long MAX_UNDERCLOCK = 24 * 1000000;                                                                                  // Temperature <= 90 (Tj) should be ok for more than 100,000 Power on Hours at 1.15V (freq <= 528 MHz).
const unsigned long HIGH_UNDERCLOCK = 150 * 1000000;
const unsigned long MEDIUM_UNDERCLOCK = 396 * 1000000;
const unsigned long MILD_UNDERCLOCK = 450 * 1000000;
const unsigned long STANDARD_CLOCK = 528 * 1000000;
const unsigned long OBD_DETECT_TIMEOUT = 120000;                                                                                    // Timeout before resuming 6F1 transmission after a diagnostic tool is detected.
#if __has_include ("src/custom-settings.h")                                                                                         // Optionally, create this file to store sensitive settings.
  #include "src/custom-settings.h"
#endif
#if SECRETS
  uint8_t IMMOBILIZER_SEQ_NUMBER = SECRET_IMMOBILIZER_SEQ;
  uint8_t IMMOBILIZER_SEQ_ALARM_NUMBER = SECRET_IMMOBILIZER_SEQ_ALARM_NUMBER;
  String serial_password = secret_serial_password;
#else
  uint8_t IMMOBILIZER_SEQ_NUMBER = 3;                                                                                               // Number of times to press the button for the EKP to be re-activated.
  uint8_t IMMOBILIZER_SEQ_ALARM_NUMBER = 6;                                                                                         // Number of times to press the button for the alarm to be silenced and EKP to be re-activated.
  String serial_password = "coldboot";                                                                                              // Default password.
#endif

/***********************************************************************************************************************************************************************************************************************************************
  F-series retrofits:
***********************************************************************************************************************************************************************************************************************************************/

#ifndef F_NBTE
  #define F_NBTE 0                                                                                                                  // Emulate KCAN2 to integrate HU_NBT2 ID5/ID6 into Exx cars.
  #if F_NBTE
    #define F_NBTE_VIN_PATCH 1                                                                                                      // Requests CPS VIN from NBT before sending 0x380. Disable if using patched NBT binary.
    #define F_NBTE_CCC_ZBE 0                                                                                                        // Converts CCC ZBE1 messages for use with NBT.
    #define X_VIEW 0                                                                                                                // Convert the angles required to make the xDrive status 3D graphic work.
    #define ASD89_RAD_ON 0                                                                                                          // Use RAD_ON from the ASD module to power Diversity. Pin 7 of ASD must be wired to pin 3 of Diversity.
    #define CUSTOM_MONITORING_CC 1                                                                                                  // Print additional information (Water temp, voltage, IAT and boost) in the iDrive CC list.
  #endif
#endif

const int HU_ENT_MODE_TIMEOUT = 60000;                                                                                              // Amount of time within which the HU can be re-activated after Terminal R OFF.

uint8_t MAX_TORQUE_SCALE_NM = 6;                                                                                                    // Used to scale the NBT sport displays. Torque = (x + 1) * 80. I.e. (6 + 1) * 80 = 560.
uint8_t MAX_TORQUE_SCALE_LBFT = 5;                                                                                                  // 480
uint8_t MAX_TORQUE_SCALE_KGM = 0;                                                                                                   // 80
uint8_t MAX_POWER_SCALE_KW = 3;                                                                                                     // Power = (x + 1) * 80. I.e. (3 + 1) * 80 = 320. 
uint8_t MAX_POWER_SCALE_HP = 4;                                                                                                     // 400
float MAX_TURBO_BOOST = 950.0;                                                                                                      // In hPa. 1M overboost: 0.90-0.95 bar.

#ifndef F_VSW01
  #define F_VSW01 0                                                                                                                 // Enable/disable F01 Video Switch diagnosis and wakeup. Tested with p/n 9201542.
  #define F_VSW01_MANUAL 0                                                                                                          // Manual control of the VSW if used without HU support. CIC, NBTEVO GW7 etc.
#endif
#ifndef F_NIVI
  #define F_NIVI 0                                                                                                                  // Enable/disable FXX NVE diagnosis, wakeup and BN2000->BN2010 message translation.
#endif
const char *vsw_positions[] = {"Disabled", "Rear view camera: TRSVC", "Night Vision: NiVi", "DVD changer: MMC",
                               "Digital TV: VM", "N/A", "N/A", "N/A", "N/A", "N/A"};

/***********************************************************************************************************************************************************************************************************************************************
  These features are *very* implementation specific. They are disabled by default since they're unlikely to be used by anyone except me.
***********************************************************************************************************************************************************************************************************************************************/

#ifndef HDC
  #define HDC 0                                                                                                                     // Gives a function to the HDC console button in non 4WD cars.
#endif
#ifndef FAKE_MSA
  #define FAKE_MSA 0                                                                                                                // Display Auto Start-Stop OFF CC message when the Auto Start-Stop button is pressed. Must be coded in IHK.
#endif
#ifndef REVERSE_BEEP
  #define REVERSE_BEEP 0                                                                                                            // Play a beep through the speakers when engaging reverse.
#endif
#ifndef AUTO_STEERING_HEATER
  #define AUTO_STEERING_HEATER 0                                                                                                    // Enable automatic heated steering wheel at low temperatures. Requires transistor.
#endif
#ifndef ASD89_MDRIVE
  #define ASD89_MDRIVE 0                                                                                                            // Enable / Disable mute control of E89 ASD with MDrive. ASD is muted when MDrive POWER Sport+ is inactive.
#endif

/***********************************************************************************************************************************************************************************************************************************************
  Board configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#define POWER_BUTTON_PIN 2
#define POWER_LED_PIN 3
#define FOG_LED_PIN 4
#define STEERING_HEATER_SWITCH_PIN 5
#define MCP2515_INT_PIN 9
#define SPI_CS_PIN 10
#define DCAN_STBY_PIN 14
#define PTCAN_STBY_PIN 15                                                                                                           // STBY pins are optional but recommended to save power when the car prepares to deep sleep.
#define DSC_BUTTON_PIN 16
#define EXHAUST_FLAP_SOLENOID_PIN 17
#define FACEPLATE_EJECT_PIN 18
#define FACEPLATE_POWER_MUTE_PIN 19
#define FACEPLATE_UART_PIN 21                                                                                                       // UART RX5

/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

#include <EEPROM.h>
#include "src/CRC/src/CRC8.h"                                                                                                       // https://github.com/RobTillaart/CRC
#include "src/FlexCAN_T4/FlexCAN_T4.h"
#include "src/CRC/src/CRC16.h"
#include "src/wdt4/Watchdog_t4.h"                                                                                                   // https://github.com/tonton81/WDT_T4
#include "src/queue/cppQueue.h"                                                                                                     // https://github.com/SMFSW/Queue
#if F_NBTE
  #include "src/MCP_CAN_lib/mcp_can.h"                                                                                              // https://github.com/coryjfowler/MCP_CAN_lib
#endif
#include "usb_dev.h"
extern "C" uint32_t set_arm_clock(uint32_t frequency);
FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_128> KCAN;
FlexCAN_T4<CAN2, RX_SIZE_512, TX_SIZE_128> PTCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_64> DCAN;

void (*kcan_handlers[0x800])(void) = {NULL};
uint8_t kcan_to_kcan2_forward_filter_list[0x800];                                                                                   // 1 - allowed, 0 - blocked.
void (*ptcan_handlers[0x800])(void) = {NULL};
void (*kcan2_handlers[0x800])(void) = {NULL};
uint8_t kcan2_to_kcan_forward_filter_list[0x800];

#if F_NBTE
  MCP_CAN KCAN2(SPI_CS_PIN);
#endif
#define FACEPLATE_UART Serial5
WDT_T4<WDT1> wdt;
CRC16 teensy_eeprom_crc(0x1021, 0, 0, false, false);                                                                                // XMODEM

CAN_message_t pt_msg, k_msg, d_msg;
typedef struct delayed_can_tx_msg {
	CAN_message_t	tx_msg;
	unsigned long	transmit_time;
} delayed_can_tx_msg;
cppQueue kcan_resend_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         ptcan_resend_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         dcan_resend_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint8_t kcan_retry_counter = 0, ptcan_retry_counter = 0, dcan_retry_counter = 0, kcan2_mode = 0xE0;
uint8_t ptcan_mode = 0, dcan_mode = 0;
unsigned long int k2rxId;
unsigned char k2rxBuf[8], k2len;
elapsedMillis eeprom_update_timer = 0;
delayed_can_tx_msg delayed_tx, m;
bool key_valid = false, terminal_r = false, ignition = false, vehicle_awake = false, vehicle_moving = false,
     terminal_50 = false, clutch_pressed = false, frm_consumer_shutdown = false, gong_active = false,
     clearing_dtcs = false, donor_vin_initialized = false, receiving_donor_vin = false, requested_donor_vin = false,
     low_battery_cc_active = false;
uint8_t engine_running = 0;                                                                                                         // 3 - signal invalid, 2 - engine running, 1 - engine starting, 0 - engine stopped.
uint16_t terminal30g_followup_time = 0;
bool kl30g_cutoff_imminent = false;
CAN_message_t nbt_vin_request_buf, dme_request_consumers_off_buf, fzm_wake_buf, fzm_sleep_buf;
CAN_message_t faceplate_a1_released_buf, faceplate_a2_released_buf, faceplate_a3_released_buf, faceplate_f1_released_buf,
              faceplate_power_mute_buf, faceplate_eject_buf, faceplate_seek_left_buf, faceplate_seek_right_buf,
              faceplate_volume_decrease_buf, faceplate_volume_increase_buf;
CAN_message_t faceplate_button1_hover_buf, faceplate_button1_press_buf, faceplate_button2_hover_buf, faceplate_button2_press_buf, 
              faceplate_button3_hover_buf, faceplate_button3_press_buf, faceplate_button4_hover_buf, faceplate_button4_press_buf,
              faceplate_button5_hover_buf, faceplate_button5_press_buf, faceplate_button6_hover_buf, faceplate_button6_press_buf,
              faceplate_button7_hover_buf, faceplate_button7_press_buf, faceplate_button8_hover_buf, faceplate_button8_press_buf;
elapsedMillis nbt_vin_request_timer = 3000, engine_run_timer = 0, vehicle_awake_timer = 0, vehicle_awakened_timer = 0;
uint8_t DONOR_VIN[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
float battery_voltage = 12.0;
uint16_t faceplate_volume = 0;
bool faceplate_eject_pressed = false, faceplate_power_mute_pressed = false, faceplate_hu_reboot = false;
elapsedMillis faceplate_eject_pressed_timer = 0, faceplate_power_mute_pressed_timer = 0,
              faceplate_power_mute_debounce_timer = 300, faceplate_eject_debounce_timer = 1000,
              faceplate_uart_watchdog_timer = 0;
uint8_t faceplate_reset_counter = 0;
cppQueue faceplate_buttons_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         radon_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
CAN_message_t dsc_on_buf, dsc_mdm_dtc_buf, dsc_off_buf;
cppQueue dsc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint16_t RPM = 0;
float e_throttle_position = 0;
float engine_torque = 0, engine_torque_nm = 0;
bool engine_idling = false;                                                                                                         // Not idling.
bool handbrake_status = true;
elapsedMillis handbrake_status_debounce_timer = 300;
uint8_t dme_ckm[4][2] = {{0xF1, 0xFF}, {0xF1, 0xFF}, {0xF1, 0xFF}, {0xF1, 0xFF}};
uint8_t edc_mode = 8,                                                                                                               // 8 - Comfort, 9 - Sport, 0xB - Sport+.                                                                                                               
        cas_key_number = 3;                                                                                                         // 0 = Key 1, 1 = Key 2...
bool key_guest_profile = false;
uint8_t mdrive_dsc[4] = {0x13, 0x13, 0x13, 0xB}, mdrive_power[4] = {0x30, 0x30, 0x30, 0x10},                                        // Defaults for when EEPROM is not initialized.
        mdrive_edc[4] = {0x2A, 0x2A, 0x2A, 0x21}, mdrive_svt[4] = {0xF1, 0xF1, 0xF1, 0xE9};
bool mdrive_status = false, mdrive_power_active = false, console_power_mode, restore_console_power_mode = false;
uint8_t power_mode_only_dme_veh_mode[] = {0xE8, 0xF1};                                                                              // E8 is the last checksum. Start will be from 0A.
uint8_t dsc_program_status = 0;                                                                                                     // 0 = ON, 1 = DSC OFF, 4 = DTC/MDM. 
uint8_t dsc_intervention = 0;
bool holding_dsc_off_console = false, dsc_mode_change_disable = false;
elapsedMillis mdrive_message_timer = 0, veh_mode_timer = 0;
uint8_t m_mfl_held_count = 0;
CAN_message_t idrive_mdrive_settings_menu_cic_a_buf, idrive_mdrive_settings_menu_cic_b_buf,
              idrive_mdrive_settings_menu_nbt_a_buf, idrive_mdrive_settings_menu_nbt_b_buf,
              idrive_mdrive_settings_menu_nbt_c_buf, gws_sport_on_buf, gws_sport_off_buf,
              idrive_bn2000_time_12h_buf, idrive_bn2000_time_24h_buf, 
              idrive_bn2000_date_ddmmyyyy_buf, idrive_bn2000_date_mmddyyyy_buf,
              idrive_bn2000_consumption_l100km_buf, idrive_bn2000_consumption_kml_buf,
              idrive_bn2000_consumption_mpg_buf, idrive_bn2000_distance_km_buf, idrive_bn2000_distance_mi_buf,
              idrive_bn2000_pressure_bar_buf, idrive_bn2000_pressure_kpa_buf, idrive_bn2000_pressure_psi_buf,
              idrive_bn2000_temperature_c_buf, idrive_bn2000_temperature_f_buf,
              idrive_bn2000_hba_on_buf, idrive_bn2000_hba_off_buf, idrive_bn2000_indicator_single_buf,
              idrive_bn2000_indicator_triple_buf, idrive_bn2000_drl_on_buf, idrive_bn2000_drl_off_buf,
              set_warning_15kph_on_buf, set_warning_15kph_off_buf;
uint8_t hba_status = 0;
bool mdrive_settings_requested = false;
bool requested_hu_off_t1 = false, requested_hu_off_t2 = false, hu_ent_mode = false,
     hu_bn2000_nm_initialized = false, hu_bn2000_bus_sleep_active = false;

// Since E9X cars are quite similar, we can predict the neighbours without having to observe the network.
// All E92s have PDC which means when ignition is ON the nearest neighbour to 0x4E2 (CCC) is 0x4E4.
// With ignition OFF, it must be the Driver's seat module. Unless there's a ZBE1 installed.
#if F_NBTE_CCC_ZBE
  uint8_t hu_bn2000_nm_next_neighbour = 0x67;                                                                                       // ZBE1.
#else 
  uint8_t hu_bn2000_nm_next_neighbour = 0x6D;                                                                                       // Driver's seat module.
#endif

elapsedMillis hu_bn2000_bus_sleep_ready_timer = 0, hu_bn2000_nm_timer = 1000;                                                       // Start NM registration 2s from now.

const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 500, dsc_hold_time_ms = 300;
elapsedMillis power_button_debounce_timer = power_debounce_time_ms,
              dsc_off_button_debounce_timer = dsc_debounce_time_ms, dsc_off_button_hold_timer = 0;
bool ignore_m_press = false, ignore_m_hold = false, holding_both_console = false;
int8_t clock_mode = -1;
float cpu_temp = 0, last_cpu_temp = 0, max_cpu_temp = 0;
CAN_message_t cc_single_gong_buf, cc_double_gong_buf, cc_triple_gong_buf, idrive_button_sound_buf,
              idrive_beep_sound_buf, idrive_horn_sound_buf;
bool uif_read = false;
uint8_t servotronic_message[] = {0, 0xFF};
CAN_message_t svt70_zero_pwm_buf, svt70_pwm_release_control_buf, shiftlights_start_buf,
              shiftlights_mid_buildup_buf, shiftlights_startup_buildup_buf,
              shiftlights_max_flash_buf, shiftlights_off_buf;
elapsedMillis svt70_pwm_control_timer = 3000;
bool svt70_sport_plus = false, shiftlights_segments_active = false, startup_animation_active = false;
uint8_t ignore_shiftlights_off_counter = 0, ignore_sports_data_counter = 0;
uint16_t START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM,
         MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM,
         MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM,
         GONG_UPSHIFT_WARN_RPM_ = GONG_UPSHIFT_WARN_RPM;
bool engine_coolant_warmed_up = false;
uint16_t var_redline_position = 0;
uint8_t last_var_rpm_can = 0;
#if CONTROL_SHIFTLIGHTS
  uint8_t mdrive_message_bn2000[] = {0, 0, 0, 0, 0, 0x97};                                                                          // Byte5: shiftlights always on
#else
  uint8_t mdrive_message_bn2000[] = {0, 0, 0, 0, 0, 0x87};                                                                          // Byte5: shiftlights always off
#endif
uint8_t mdrive_message_bn2010[] = {0, 0, 0, 0, 0, 0x8A, 0x90, 0xE8};
CAN_message_t speedo_needle_max_buf, speedo_needle_min_buf, speedo_needle_release_buf,
              tacho_needle_max_buf, tacho_needle_min_buf, tacho_needle_release_buf,
              fuel_needle_max_buf, fuel_needle_min_buf, fuel_needle_release_buf,
              oil_needle_max_buf, oil_needle_min_buf, oil_needle_release_buf;
cppQueue kombi_needle_txq(sizeof(delayed_can_tx_msg), 32, queue_FIFO);
bool front_fog_status = false;
bool indicators_on = false;
unsigned long last_fog_action_timer = 15000;
elapsedMillis front_fog_corner_timer = 1500;
bool dipped_beam_status = false, left_fog_on = false, right_fog_on = false;
bool ahl_active = false, flc_active = false, frm_ahl_flc_status_requested = false;
CAN_message_t frm_ahl_flc_status_request_buf;
float steering_angle = 0, steering_angle_speed = 0;
CAN_message_t front_left_fog_on_a_buf, front_left_fog_on_b_buf, front_left_fog_on_c_buf, front_left_fog_on_d_buf,
              front_left_fog_on_a_softer_buf, front_left_fog_on_b_softer_buf, front_left_fog_on_c_softer_buf,
              front_left_fog_on_d_softer_buf, front_left_fog_on_e_softer_buf, front_left_fog_on_f_softer_buf,
              front_left_fog_on_g_softer_buf, front_left_fog_on_h_softer_buf,
              front_right_fog_on_a_softer_buf, front_right_fog_on_b_softer_buf, front_right_fog_on_c_softer_buf,
              front_right_fog_on_d_softer_buf, front_right_fog_on_e_softer_buf, front_right_fog_on_f_softer_buf,
              front_right_fog_on_g_softer_buf, front_right_fog_on_h_softer_buf,
              front_right_fog_on_a_buf, front_right_fog_on_b_buf, front_right_fog_on_c_buf, front_right_fog_on_d_buf,
              front_left_fog_off_buf, front_right_fog_off_buf, front_fogs_all_off_buf;
cppQueue fog_corner_left_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         fog_corner_right_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
unsigned long last_drl_action_timer = 15000;
uint8_t drl_ckm[4] = {1, 1, 1, 1};
bool drl_status = false, left_dimmed = false, right_dimmed = false;
CAN_message_t left_drl_dim_off, left_drl_dim_buf, left_drl_bright_buf,
              right_drl_dim_off, right_drl_dim_buf, right_drl_bright_buf;
cppQueue dim_drl_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
bool ftm_indicator_status = false;
CAN_message_t ftm_indicator_flash_buf, ftm_indicator_off_buf,
              hood_open_hot_cc_dialog_buf, hood_open_hot_cc_dialog_clear_buf;
elapsedMillis hood_status_debounce = 500;
uint8_t last_hood_status = 0, last_trunk_status = 0;
CAN_message_t frm_ckm_ahl_komfort_buf, frm_ckm_ahl_sport_buf;
bool wipe_scheduled = false;
uint16_t wash_message_counter = 0, wiper_stalk_down_message_counter = 0;
uint16_t indicator_stalk_pushed_message_counter = 0;
unsigned long wiper_stalk_down_last_press_time = 0;
CAN_message_t wipe_single_buf;
uint8_t intermittent_setting = 0, intermittent_setting_can = 0;
uint16_t intermittent_intervals[] = {13100, 9100, 5100, 0, 2100},                                                                   // Settings: 1 (12s), 2 (8s), 3 (4s), n/a and 5 (max, 1s). 1100ms is needed for a cycle.
         intermittent_intervals_offset_stopped[] = {3000, 2000, 1000, 0, 0, 1500};
elapsedMillis intermittent_wipe_timer = 3000;
bool intermittent_wipe_active = false, auto_wipe_active = false;
cppQueue wiper_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint8_t mirror_status_retry = 0;
bool mirrors_folded = false, frm_mirror_status_requested = false;
bool fold_lock_button_pressed = false;
bool lock_button_pressed = false;
int16_t lock_button_pressed_counter = 0;
CAN_message_t frm_mirror_status_request_a_buf, frm_mirror_status_request_b_buf,
              frm_toggle_fold_mirror_a_buf, frm_toggle_fold_mirror_b_buf, frm_mirror_undim_buf;
bool szl_full_indicator = false;
cppQueue mirror_fold_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
elapsedMillis frm_undim_timer = 10000;
uint8_t fzd_ec_dimming = 0;
uint8_t auto_seat_ckm[4] = {0, 0, 0, 0};
bool comfort_exit_ready = false;
CAN_message_t dr_seat_move_back_buf;
uint8_t last_lock_status_can = 0;
bool unfold_with_door_open = false;
CAN_message_t flash_hazards_single_buf, flash_hazards_double_buf, flash_hazards_single_long_buf,
              flash_hazards_angel_eyes_buf, flash_hazards_angel_eyes_xenons_buf, stop_flashing_lights_buf,
              alarm_beep_6x_buf;
uint8_t car_locked_indicator_counter = 0;
cppQueue hazards_flash_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint8_t visual_signal_ckm[4] = {0, 0, 0, 0};
bool hazards_on = false;
bool immobilizer_released = false, immobilizer_persist = true;
float immobilizer_max_speed = 20.0;
unsigned long immobilizer_send_interval = 0;                                                                                        // Skip the first interval delay.
elapsedMillis immobilizer_timer = 0, both_console_buttons_timer, immobilizer_activate_release_timer = 0;
uint8_t immobilizer_pressed_release_count = 0, immobilizer_pressed_activate_count = 0;
CAN_message_t key_cc_on_buf, key_cc_off_buf, start_cc_on_buf, start_cc_off_buf,
              ekp_pwm_off_buf, ekp_return_to_normal_buf;
cppQueue alarm_led_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         alarm_warnings_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         ekp_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         alarm_siren_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
bool alarm_after_engine_stall = false, alarm_active = false, alarm_led_disable_on_lock = false, eml_light = false;
elapsedMillis alarm_led_message_timer = 100000;
CAN_message_t alarm_led_on_buf, alarm_led_return_control_buf, alarm_siren_on_buf, alarm_siren_return_control_buf;
bool reverse_beep_sent = false, pdc_too_close = false;
elapsedMillis reverse_beep_resend_timer = 2000;
bool reverse_gear_status = false;
uint8_t f_terminal_status_alive_counter = 0;
elapsedMillis f_energy_condition_timer = 5000;
CAN_message_t f_kombi_network_management_buf, f_zgw_network_management_buf;
uint8_t vsw_current_input = 0, vsw_switch_counter = 0xF1;
uint16_t idrive_current_menu;
CAN_message_t idrive_menu_request_a_buf, idrive_menu_request_b_buf;
float sine_pitch_angle = 0, sine_roll_angle = 0;
bool sine_pitch_angle_requested = false, sine_roll_angle_requested = false;
uint16_t xview_pitch_angle = 0x5000;                                                                                                // 0 degrees
uint8_t xview_grade_percentage = 0x64;                                                                                              // 0%
uint16_t f_vehicle_pitch_angle = 0x2500, f_vehicle_roll_angle = 0x2500;                                                             // 1280 * 0.05 - 64 = 0 degrees, 2 - Valid.
float e_longitudinal_acceleration = 0, e_lateral_acceleration = 0, e_yaw_rate = 0;
uint16_t f_longitudinal_acceleration = 0x7EF4, f_lateral_acceleration = 0x7EF4;                                                     // 32500 * 0.002 - 65 = 0 m/s^2.
uint8_t f_longitudinal_acceleration_alive_counter = 0, f_lateral_acceleration_alive_counter = 0;
uint16_t f_yaw_rate = 0x8000;                                                                                                       // 32768 * 0.005 - 163.84 = 0 degrees/sec.
uint8_t f_yaw_alive_counter = 0;
float real_speed = 0;
uint8_t e_vehicle_direction = 0, f_speed_alive_counter = 0, rls_brightness = 0xFE, rls_time_of_day = 0,
        f_data_powertrain_2_alive_counter = 0, f_torque_1_alive_counter = 0, f_vehicle_status_alive_counter = 0,
        f_driving_dynamics_alive_counter = 0, f_steering_angle_alive_counter = 0, f_pdc_function_status_alive_counter = 0,
        f_ftm_status_alive_counter = 0, f_standstill_status_alive_counter = 0, f_mdrive_alive_counter = 0,
        f_xview_pitch_alive_counter = 0, f_road_incline_alive_counter = 0, f_steering_angle_effective_alive_counter = 0,
        f_throttle_pedal_alive_counter = 0;
uint8_t f_mdrive_settings[] = {0, 0, 0, 0, 0}, f_driving_dynamics_ignore = 3;
uint32_t f_distance_alive_counter = 0x2000;
uint8_t f_lights_ckm_request = 0;
CAN_message_t f_lights_ckm_delayed_msg;
uint16_t f_converted_steering_angle = 0, f_front_axle_wheel_angle = 0;
elapsedMillis sine_pitch_angle_request_timer = 500, sine_roll_angle_request_timer = 500,
              f_outside_brightness_timer = 500, f_data_powertrain_2_timer = 1000,
              f_xview_pitch_timer = 1000, f_driving_dynamics_timer = 1000, f_standstill_status_timer = 1000,
              f_road_incline_timer_ptcan = 100, f_road_incline_timer_kcan2 = 200, f_torque_1_timer = 20,
              f_chassis_lateral_timer_ptcan = 20, f_chassis_lateral_timer_kcan2 = 200,
              f_chassis_longitudinal_timer_kcan2 = 200, f_chassis_longitudinal_timer_ptcan = 20,
              f_chassis_yaw_timer = 20, f_chassis_speed_timer = 20,
              f_chassis_steering_timer = 200, f_chassis_steering_effective_timer = 20, f_throttle_pedal_timer = 20;
CAN_message_t sine_pitch_angle_request_a_buf, sine_pitch_angle_request_b_buf,
              sine_roll_angle_request_a_buf, sine_roll_angle_request_b_buf, nivi_button_pressed_buf,
              nivi_button_released_buf, f_hu_nbt_reboot_buf;
CRC8 f_vehicle_status_crc(0x1D, 0, 0x64, false, false),                                                                             // SAE J1850 POLY, 0 init and XOR-OUT 0x64 for ARB-ID 0x3C.
     f_torque_1_crc(0x1D, 0, 0x6A, false, false),                                                                                   // SAE J1850 POLY, 0 init and XOR-OUT 0x6A for ARB-ID 0xA5.
     f_throttle_pedal_crc(0x1D, 0, 0xD7, false, false),                                                                             // SAE J1850 POLY, 0 init and XOR-OUT 0xD7 for ARB-ID 0xD9.
     f_terminal_status_crc(0x1D, 0, 0xB1, false, false),                                                                            // SAE J1850 POLY, 0 init and XOR-OUT 0xB1 for ARB-ID 0x12F.
     f_road_incline_crc(0x1D, 0, 0xCE, false, false),                                                                               // SAE J1850 POLY, 0 init and XOR-OUT 0xCE for ARB-ID 0x163.
     f_longitudinal_acceleration_crc(0x1D, 0, 0x5F, false, false),                                                                  // SAE J1850 POLY, 0 init and XOR-OUT 0x5F for ARB-ID 0x199.
     f_lateral_acceleration_crc(0x1D, 0, 0xE5, false, false),                                                                       // SAE J1850 POLY, 0 init and XOR-OUT 0xE5 for ARB-ID 0x19A.
     f_yaw_rate_msg_crc(0x1D, 0, 1, false, false),                                                                                  // SAE J1850 POLY, 0 init and XOR-OUT 1 for ARB-ID 0x19F.
     f_speed_crc(0x1D, 0, 0xF, false, false),                                                                                       // SAE J1850 POLY, 0 init and XOR-OUT 0xF for ARB-ID 0x1A1.
     f_standstill_status_crc(0x1D, 0, 0x8F, false, false),                                                                          // SAE J1850 POLY, 0 init and XOR-OUT 0x8F for ARB-ID 0x2DC.
     f_steering_angle_crc(0x1D, 0, 0xD1, false, false),                                                                             // SAE J1850 POLY, 0 init and XOR-OUT 0xD1 for ARB-ID 0x301.
     f_steering_angle_effective_crc(0x1D, 0, 0x3A, false, false),                                                                   // SAE J1850 POLY, 0 init and XOR-OUT 0x3A for ARB-ID 0x302.
     f_ftm_status_crc(0x1D, 0, 0x63, false, false),                                                                                 // SAE J1850 POLY, 0 init and XOR-OUT 0x63 for ARB-ID 0x369.
     f_driving_dynamics_crc(0x1D, 0, 0x43, false, false),                                                                           // SAE J1850 POLY, 0 init and XOR-OUT 4 for ARB-ID 0x3A7.
     f_data_powertrain_2_crc(0x1D, 0, 4, false, false);                                                                             // SAE J1850 POLY, 0 init and XOR-OUT 4 for ARB-ID 0x3F9.
bool exhaust_flap_sport = false, exhaust_flap_open = true;
elapsedMillis exhaust_flap_action_timer;                                                                                            // Initialized when the engine is started.
unsigned long exhaust_flap_action_interval = 1000;
CAN_message_t lc_cc_on_buf, lc_cc_off_buf;
bool lc_cc_active = false, mdm_with_lc = false;
float ambient_temperature_real = -255.0, interior_temperature = -255.0;                                                             // Set below the minimum ranges.
bool driver_seat_heating_status = false, driver_sent_seat_heating_request = false;
CAN_message_t seat_heating_button_pressed_dr_buf, seat_heating_button_released_dr_buf;
cppQueue seat_heating_dr_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
bool passenger_seat_heating_status = false, passenger_sent_seat_heating_request = false;
uint8_t passenger_seat_status = 0;                                                                                                  // 0 - Not occupied not belted, 1 - not occupied and belted, 8 - occupied not belted, 9 - occupied and belted
CAN_message_t seat_heating_button_pressed_pas_buf, seat_heating_button_released_pas_buf;
cppQueue seat_heating_pas_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
bool sent_steering_heating_request = false, steering_heater_transistor_active = false;
elapsedMillis steering_heater_transistor_active_timer;
bool volume_reduced = false, initial_volume_set = false, pdc_tone_on = false;
uint8_t volume_restore_offset = 0, volume_changed_to, peristent_volume = 0;
elapsedMillis volume_request_door_timer = 300;
CAN_message_t vol_request_buf;
cppQueue idrive_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
bool left_door_open = false, right_door_open = false, doors_locked = false, doors_alarmed = false, windows_closed = false;
elapsedMillis doors_locked_timer = 0;
uint8_t front_left_window_status = 0xFC, front_right_window_status = 0xFC, last_door_status = 0, sunroof_status = 0;
elapsedMillis idrive_watchdog_timer = 3000, idrive_run_timer = 0;
bool idrive_died = false;
uint8_t zbe_buttons[] = {0xE1, 0xFD, 0, 0, 0, 1}, zbe_rotation[] = {0xE1, 0xFD, 0, 0, 0x80, 0x1E}, zbe_action_counter = 0;
float indicated_speed = 0;
bool speed_mph = false;
float max_hdc_speed = 35.0, hdc_deactivate_speed = 60.0;
uint8_t stalk_message_counter = 0;
bool cruise_control_status = false, hdc_button_pressed = false, hdc_requested = false, hdc_active = false;
CAN_message_t set_hdc_cruise_control_buf, cancel_hdc_cruise_control_buf,
              hdc_cc_activated_on_buf, hdc_cc_unavailable_on_buf, hdc_cc_deactivated_on_buf,
              hdc_cc_activated_off_buf, hdc_cc_unavailable_off_buf, hdc_cc_deactivated_off_buf;
cppQueue hdc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
CAN_message_t msa_deactivated_cc_on_buf, msa_deactivated_cc_off_buf;
uint8_t pdc_bus_status = 0x80;
CAN_message_t camera_off_buf, camera_on_buf, camera_inactive_buf, pdc_off_camera_on_buf, pdc_on_camera_on_buf,
              pdc_off_camera_off_buf, pdc_button_presssed_buf, pdc_button_released_buf;
cppQueue pdc_buttons_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
uint8_t f_pdc_request = 1, rvc_settings[] = {0xE5, 0x4B, 0x2D, 0xE1};                                                               // Camera OFF, Brightness 50%, Contrast 50%, Parking lines OFF, Obstacle marking ON, Tow view OFF.
bool rvc_tow_view_by_module = false, rvc_tow_view_by_driver = false, msa_button_pressed = false;
elapsedMillis rvc_action_timer = 500, msa_fake_status_timer = 0;
cppQueue ihk_extra_buttons_cc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO);
bool asd_initialized = false, asd_rad_on_initialized = false;
CAN_message_t msa_fake_status_buf, mute_asd_buf, demute_asd_buf, radon_asd_buf,
              clear_hs_kwp_dme_buf, ccc_zbe_wake_buf, jbe_reboot_buf,
              ihka_5v_on_buf, ihka_5v_off_buf;
extern float tempmonGetTemp(void);
char serial_debug_string[512];
char boot_debug_string[16384];                                                                                                      // Can store a minimum of 64 messages that are 256 bytes long (unlikely).
unsigned long max_loop_timer = 0, loop_timer = 0;
uint32_t kcan_error_counter = 0, kcan2_error_counter = 0, ptcan_error_counter = 0, dcan_error_counter = 0;
bool serial_commands_unlocked = false;
uint8_t torque_unit[] = {1, 1, 1, 1}, power_unit[] = {1, 1, 1, 1}, pressure_unit_date_format[] = {9, 9, 9, 9},
        driving_mode = 0, f_units[] = {0, 0, 0, 0, 0, 0xF1}, temperature_unit = 1;
uint8_t engine_coolant_temperature = 48, engine_oil_temperature = 48;                                                               // Celsius temperature is: value - 48.
CAN_message_t custom_cc_dismiss_buf, custom_cc_clear_buf, cc_list_clear_buf,
              dme_boost_request_a_buf, dme_boost_request_b_buf;
elapsedMillis custom_info_cc_timer = 100, boost_request_timer = 100;
uint16_t engine_manifold_sensor = 0, engine_cp_sensor = 0, ambient_pressure = 1000;
int16_t boost = 0, intake_air_temperature = 0;
bool dme_boost_requested = false, trsvc_cc_gong = false;
unsigned long cc_message_expires = millis();
uint8_t ihka_auto_fan_speed = 5, ihka_auto_fan_state = 3, ihka_recirc_state = 0, ihka_auto_distr_state = 0;
cppQueue nbt_cc_txq(sizeof(delayed_can_tx_msg), 16, queue_FIFO),
         serial_diag_dcan_txq(sizeof(delayed_can_tx_msg), 256, queue_FIFO),                                                         // Queues must fit error and shadow clear calls for all modules.
         serial_diag_kcan1_txq(sizeof(delayed_can_tx_msg), 256, queue_FIFO),  
         serial_diag_kcan2_txq(sizeof(delayed_can_tx_msg), 256, queue_FIFO),
         serial_diag_ptcan_txq(sizeof(delayed_can_tx_msg), 256, queue_FIFO);
CAN_message_t power_down_cmd_a_buf, power_down_cmd_b_buf, power_down_cmd_c_buf;
bool diag_transmit = true, diag_timeout_active = true, power_down_requested = false;
elapsedMillis debug_print_timer = 500, diag_deactivate_timer, serial_unlocked_timer = 0, slcan_timer = 0;
bool debug_print_connected = false, slcan_connected = false, slcan_enabled = true, timestamp = true;
uint8_t slcan_bus = 1;                                                                                                              // KCAN.
bool date_time_valid = false;
uint16_t t_hours = 0, t_minutes = 0, t_seconds = 0, d_day = 1, d_month = 1, d_year = 2024;
