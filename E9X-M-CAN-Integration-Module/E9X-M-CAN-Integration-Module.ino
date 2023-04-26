// See program-notes.txt for details on how this sketch works.

#include <FlexCAN_T4.h>
#include <EEPROM.h>
#include "src/wdt4/Watchdog_t4.h"
#include "src/queue/cppQueue.h"
extern "C" uint32_t set_arm_clock(uint32_t frequency);

/***********************************************************************************************************************************************************************************************************************************************
  Board configuration section.
***********************************************************************************************************************************************************************************************************************************************/

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> KCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> PTCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> DCAN;
WDT_T4<WDT1> wdt;

#define PTCAN_STBY_PIN 15
#define DCAN_STBY_PIN 14
#define EXHAUST_FLAP_SOLENOID_PIN 17
#define DSC_BUTTON_PIN 16
#define POWER_BUTTON_PIN 2
#define POWER_LED_PIN 3
#define FOG_LED_PIN 4

/***********************************************************************************************************************************************************************************************************************************************
  Program configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#define DEBUG_MODE 1                                                                                                                // Toggle serial debug messages. Disable in production.

#define CKM 1                                                                                                                       // Persistently remember POWER when set in iDrive.
#define EDC_CKM_FIX 1                                                                                                               // With Chinese keys/invalid profiles, sometimes the M Key setting for EDC is not recalled correctly.
#define DOOR_VOLUME 1                                                                                                               // Reduce audio volume on door open.
#define RHD 1                                                                                                                       // Where does the driver sit?
#define FTM_INDICATOR 1                                                                                                             // Indicate FTM (Flat Tyre Monitor) status when using M3 RPA hazards button cluster.
#define HDC 1                                                                                                                       // Gives a function to the HDC console button in non 4WD cars.
#define FAKE_MSA 1                                                                                                                  // Display Auto Start-Stop OFF CC message when the Auto Start-Stop button is pressed. Must be coded in IHK.
#define REVERSE_BEEP 1                                                                                                              // Play a beep throught he speaker closest to the driver when engaging reverse.
#define FRONT_FOG_INDICATOR 1                                                                                                       // Turn on an external LED when front fogs are on. M3 clusters lack this indicator.
#define DIM_DRL 1                                                                                                                   // Dims DLR on the side that the indicator is on.
#define SERVOTRONIC_SVT70 1                                                                                                         // Control steering assist with modified SVT70 module.
#define EXHAUST_FLAP_CONTROL 1                                                                                                      // Take control of the exhaust flap solenoid.
#if EXHAUST_FLAP_CONTROL
#define QUIET_START 1                                                                                                               // Close the exhaust valve as soon as Terminal R is turned on.
#endif
#define LAUNCH_CONTROL_INDICATOR 1                                                                                                  // Show launch control indicator (use with MHD lauch control, 6MT).
#define CONTROL_SHIFTLIGHTS 1                                                                                                       // Display shiftlights, animation and sync with the variable redline of M3 KOMBI.
#define NEEDLE_SWEEP 1                                                                                                              // Needle sweep animation with engine on. Calibrated for M3 speedo with 335i tacho.
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver and passenger in low temperatures.
#define RTC 1                                                                                                                       // Set the time/date if power is lost. Requires external battery.
#define F_ZBE_WAKE 0                                                                                                                // Enable/disable FXX CIC ZBE wakeup functions. Do not use with an EXX ZBE.

#if SERVOTRONIC_SVT70
  const uint16_t SVT_FAKE_EDC_MODE_CANID = 0x327;                                                                                   // New CAN-ID replacing 0x326 in SVT70 firmware bin. This stops it from changing modes together with EDC.
#endif
const uint16_t DME_FAKE_VEH_MODE_CANID = 0x7F1;                                                                                     // New CAN-ID replacing 0x315 in DME [Program] section of the firmware.
const uint8_t AUTO_SEAT_HEATING_TRESHOLD = 10 * 2 + 80;                                                                             // Degrees Celsius temperature * 2 + 80.
#if CONTROL_SHIFTLIGHTS
  const uint16_t START_UPSHIFT_WARN_RPM = 5500 * 4;                                                                                 // RPM setpoints (warning = desired RPM * 4).
  const uint16_t MID_UPSHIFT_WARN_RPM = 6000 * 4;
  const uint16_t MAX_UPSHIFT_WARN_RPM = 6500 * 4;
  const uint16_t GONG_UPSHIFT_WARN_RPM = 6800 * 4;
#endif
#if EXHAUST_FLAP_CONTROL
  const uint16_t EXHAUST_FLAP_QUIET_RPM = 3500 * 4;                                                                                 // RPM setpoint to open the exhaust flap in normal mode (desired RPM * 4).
#endif
#if LAUNCH_CONTROL_INDICATOR
  const uint16_t LC_RPM = 4000 * 4;                                                                                                 // RPM setpoint to display launch control flag CC (desired RPM * 4). Match with MHD setting.
  const uint16_t LC_RPM_MIN = LC_RPM - (250 * 4);
  const uint16_t LC_RPM_MAX = LC_RPM + (250 * 4);
#endif
#if CONTROL_SHIFTLIGHTS
  const int16_t VAR_REDLINE_OFFSET_RPM = -300;                                                                                      // RPM difference between DME requested redline and KOMBI displayed redline. Varies with cluster.
#endif
#if DOOR_VOLUME
  const uint16_t IDRIVE_BOOT_TIME = 30 * 1000;                                                                                      // Amount of time until the iDrive QNX OS accepts tool32 jobs from a cold boot.
#endif

const float TOP_THRESHOLD = 65.0;                                                                                                   // CPU temperature thresholds for the processor clock scaling function.
const float MEDIUM_THRESHOLD = 60.0;
const float HYSTERESIS = 2.0;
const unsigned long MEDIUM_UNDERCLOCK = 396 * 1000000;
const unsigned long MAX_UNDERCLOCK = 24 * 1000000;

/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

CAN_message_t pt_msg, k_msg, d_msg;
typedef struct delayed_can_tx_msg {
	CAN_message_t	tx_msg;
	unsigned long	transmit_time;
} delayed_can_tx_msg;

uint32_t cpu_speed_ide;
bool terminal_r = false, ignition = false,  vehicle_awake = false;
bool engine_cranking = false, engine_running = false;
unsigned long vehicle_awake_timer;
CAN_message_t dsc_on_buf, dsc_mdm_dtc_buf, dsc_off_buf;
cppQueue dsc_txq(sizeof(delayed_can_tx_msg), 4, queue_FIFO);
uint16_t RPM = 0;
#if CKM
  uint8_t dme_ckm[] = {0, 0xFF};
  uint8_t dme_ckm_counter = 0;
#endif
#if EDC_CKM_FIX
  uint8_t edc_ckm[] = {0, 0xFE};
#endif
uint8_t mdrive_dsc, mdrive_power, mdrive_edc, mdrive_svt;
bool mdrive_status = false, mdrive_settings_updated = false, mdrive_power_active = false;
bool console_power_mode, restore_console_power_mode = false;
uint8_t power_mode_only_dme_veh_mode[] = {0xE8, 0xF1};                                                                              // E8 is the last checksum. Start will be from 0A.
uint8_t dsc_program_status = 0;                                                                                                     // 0 = on, 1 = DTC, 2 = DSC OFF
bool holding_dsc_off_console = false;
unsigned long mdrive_message_timer;
uint8_t mfl_pressed_count = 0;
CAN_message_t idrive_mdrive_settings_a_buf, idrive_mdrive_settings_b_buf;
unsigned long power_button_debounce_timer, dsc_off_button_debounce_timer, dsc_off_button_hold_timer;
const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 500, dsc_hold_time_ms = 300;
bool ignore_m_press = false, ignore_m_hold = false;
uint8_t clock_mode = 0;
float last_cpu_temp = 0;
bool deactivate_ptcan_temporariliy = false;
unsigned long deactivate_ptcan_timer;
CAN_message_t cc_gong_buf;

#if SERVOTRONIC_SVT70
  uint8_t servotronic_message[] = {0, 0xFF};
  CAN_message_t servotronic_cc_on_buf;
  bool diagnose_svt = false;
  #if DEBUG_MODE
    uint16_t dcan_forwarded_count = 0, ptcan_forwarded_count = 0;
  #endif
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
  uint16_t var_redline_position;
  uint8_t last_var_rpm_can = 0;
  uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x97};                                                                                 // byte 5: shiftlights always on
#endif
#if NEEDLE_SWEEP
  CAN_message_t speedo_needle_sweep_buf, speedo_needle_release_buf;
  CAN_message_t tacho_needle_sweep_buf, tacho_needle_release_buf;
  CAN_message_t fuel_needle_sweep_buf, fuel_needle_release_buf;
  CAN_message_t oil_needle_sweep_buf, oil_needle_release_buf;
  CAN_message_t any_needle_sweep_b_buf;
  cppQueue kombi_needle_txq(sizeof(delayed_can_tx_msg), 10, queue_FIFO);
#endif
#if FRONT_FOG_INDICATOR
  bool front_fog_status = false;
#endif
#if DIM_DRL
  bool drl_status = false, left_dimmed = false, right_dimmed = false;
  CAN_message_t left_drl_dim_off, left_drl_dim_buf, left_drl_bright_buf;
  CAN_message_t right_drl_dim_off, right_drl_dim_buf, right_drl_bright_buf;
#endif
#if FTM_INDICATOR
  bool ftm_indicator_status = false;
  CAN_message_t ftm_indicator_flash_buf, ftm_indicator_off_buf;
#endif
#if REVERSE_BEEP
  CAN_message_t pdc_beep_buf, pdc_quiet_buf;
  bool pdc_beep_sent = false;
  cppQueue pdc_beep_txq(sizeof(delayed_can_tx_msg), 4, queue_FIFO);
#endif
#if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR
  bool reverse_status = false;
#endif
#if F_ZBE_WAKE
  CAN_message_t f_wakeup_buf;
  uint8_t zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  unsigned long zbe_wakeup_last_sent;
#endif
#if EXHAUST_FLAP_CONTROL
  bool exhaust_flap_sport = false, exhaust_flap_open = true;
  unsigned long exhaust_flap_action_timer, exhaust_flap_action_interval = 1500;
#endif
#if LAUNCH_CONTROL_INDICATOR
  CAN_message_t lc_cc_on_buf, lc_cc_off_buf;
  bool lc_cc_active = false, mdm_with_lc = false, clutch_pressed = false;
#endif
#if AUTO_SEAT_HEATING
  bool driver_seat_heating_status = false, passenger_seat_heating_status = false;
  uint8_t ambient_temperature_can = 255;
  uint8_t passenger_seat_status = 0;                                                                                                // 0 - Not occupied not belted, 1 - not occupied and belted, 8 - occupied not belted, 9 - occupied and belted
  bool driver_sent_seat_heating_request = false, passenger_sent_seat_heating_request = false;
  CAN_message_t seat_heating_button_pressed_dr_buf, seat_heating_button_released_dr_buf;
  CAN_message_t seat_heating_button_pressed_pas_buf, seat_heating_button_released_pas_buf;
  cppQueue seat_heating_dr_txq(sizeof(delayed_can_tx_msg), 4, queue_FIFO);
  cppQueue seat_heating_pas_txq(sizeof(delayed_can_tx_msg), 4, queue_FIFO);
#endif
#if RTC
  #include <TimeLib.h>
  CAN_message_t set_time_cc_buf;
  bool rtc_valid = true;
#endif
#if DOOR_VOLUME
  bool left_door_open = false, right_door_open = false;
  bool volume_reduced = false, volume_requested = false;
  bool default_volume_sent = false;
  uint8_t volume_restore_offset = 0, volume_changed_to;
  CAN_message_t vol_request_buf, default_vol_set_buf;
  cppQueue idrive_txq(sizeof(delayed_can_tx_msg), 6, queue_FIFO);
#endif
#if HDC
  uint16_t vehicle_speed = 0;
  bool speed_mph = false;
  uint8_t min_hdc_speed = 15, max_hdc_speed = 35;
  bool cruise_control_status = false, hdc_button_pressed = false, hdc_requested = false, hdc_active = false;
  CAN_message_t set_hdc_cruise_control_buf, cancel_hdc_cruise_control_buf;
  CAN_message_t hdc_cc_activated_on_buf, hdc_cc_unavailable_on_buf, hdc_cc_deactivated_on_buf;
  CAN_message_t hdc_cc_activated_off_buf, hdc_cc_unavailable_off_buf, hdc_cc_deactivated_off_buf;
#endif
#if HDC || LAUNCH_CONTROL_INDICATOR
  bool vehicle_moving = false;
#endif
#if FAKE_MSA
  bool msa_button_pressed = false;
  uint8_t msa_fake_status_counter = 0;
  CAN_message_t msa_deactivated_cc_on_buf, msa_deactivated_cc_off_buf, msa_fake_status_buf;
#endif
#if HDC || FAKE_MSA
  cppQueue kcan_cc_txq(sizeof(delayed_can_tx_msg), 4, queue_FIFO);
#endif
#if DEBUG_MODE
  float battery_voltage = 0;
  extern float tempmonGetTemp(void);
  char serial_debug_string[512];
  unsigned long loop_timer, debug_print_timer, max_loop_timer = 0;
  uint32_t kcan_error_counter = 0, ptcan_error_counter = 0, dcan_error_counter = 0;
#endif
bool diag_transmit = true;
unsigned long diag_deactivate_timer;


void setup() 
{
  #if DEBUG_MODE
    unsigned long setup_timer = millis();
  #endif
  if ( F_CPU_ACTUAL > (528 * 1000000)) {
    set_arm_clock(528 * 1000000);                                                                                                   // Prevent accidental overclocks. Remove if needed.
  }
  initialize_watchdog();
  cpu_speed_ide = F_CPU_ACTUAL;
  configure_IO();
  configure_can_controllers();
  cache_can_message_buffers();
  read_settings_from_eeprom();
  #if RTC
    check_rtc_valid();
  #endif
  initialize_timers();
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Setup finished in %lu ms, module is ready.", millis() - setup_timer);
    serial_log(serial_debug_string);
  #endif
}


void loop()
{
/***********************************************************************************************************************************************************************************************************************************************
  General section.
***********************************************************************************************************************************************************************************************************************************************/
  
  #if EXHAUST_FLAP_CONTROL
    control_exhaust_flap_user();
  #endif
  check_ptcan_transmit_status();
  check_diag_transmit_status();
  #if DOOR_VOLUME
    check_idrive_queue();
  #endif
  #if NEEDLE_SWEEP
    check_kombi_needle_queue();
  #endif
  if (ignition) {
    check_cpu_temp();                                                                                                               // Monitor processor temperature to extend lifetime.
    check_dsc_queue();
    check_console_buttons();
    send_mdrive_alive_message(10000);
    #if AUTO_SEAT_HEATING
      check_seatheating_queue();
    #endif
    #if REVERSE_BEEP
      check_pdc_queue();
    #endif
    #if HDC || FAKE_MSA
      check_kcan_cc_queue();
    #endif
  } else {
    if (vehicle_awake) {
      if ((millis() - vehicle_awake_timer) >= 2000) {
        vehicle_awake = false;                                                                                                      // Vehicle must now be asleep. Stop monitoring .
        serial_log("Vehicle Sleeping.");
        toggle_transceiver_standby();
        scale_mcu_speed();
        reset_sleep_variables();
      }
      send_mdrive_alive_message(15000);                                                                                             // Send this message while car is awake (but with ignition off) to populate the fields in iDrive.
    }
  }


/***********************************************************************************************************************************************************************************************************************************************
  K-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if (KCAN.read(k_msg)) {
    if (ignition) {
      if (k_msg.id == 0xAA) {                                                                                                       // Monitor 0xAA (rpm/throttle status).
        evaluate_engine_rpm();

        // Shiftlights and LC depend on RPM. Since this message is cycled every 100ms, it makes sense to run the calculations now.
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

      #if LAUNCH_CONTROL_INDICATOR
      else if (k_msg.id == 0xA8) {                                                                                                  // Monitor clutch pedal status
        evaluate_clutch_status();
      }
      #endif

      #if HDC
      else if (k_msg.id == 0x193) {                                                                                                 // Monitor state of cruise control
        evaluate_cruise_control_status();
      }
      else if (k_msg.id == 0x31A) {                                                                                                 // Received HDC button press from IHKA.
        evaluate_hdc_button();
      }
      #endif

      #if FAKE_MSA
      else if (k_msg.id == 0x195) {                                                                                                 // Monitor state of MSA button
        evaluate_msa_button();
      }
      #endif

      #if LAUNCH_CONTROL_INDICATOR || HDC
      else if (k_msg.id == 0x1B4) {                                                                                                 // Monitor if the car is stationary/moving
        evaluate_vehicle_moving();
      }
      #endif

      #if DIM_DRL
      else if (k_msg.id == 0x1F6) {                                                                                                 // Monitor indicator status
        evaluate_indicator_status_dim();
      }
      #endif

      #if FRONT_FOG_INDICATOR || DIM_DRL
      else if (k_msg.id == 0x21A) {                                                                                                 // Light status sent by the FRM.
        evaluate_fog_status();
        evaluate_drl_status();
      }
      #endif

      #if AUTO_SEAT_HEATING
      else if (k_msg.id == 0x22A || k_msg.id == 0x232) {                                                                            // Monitor passenger and driver's seat heating.
        evaluate_seat_heating_status();
      }
      #endif

      #if CKM
      else if (k_msg.id == 0x3A8) {                                                                                                 // Received POWER M Key settings from iDrive.
        save_dme_power_ckm();
      }
      #endif

      #if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR
      else if (k_msg.id == 0x3B0) {                                                                                                 // Monitor reverse status.
        evaluate_reverse_status();
        evaluate_pdc_beep();
      }
      #endif

      #if EDC_CKM_FIX
      else if (k_msg.id == 0x3C5) {                                                                                                 // Received EDC M Key settings from iDrive.
        save_edc_ckm();
      }
      #endif

      else if (k_msg.id == 0x3CA) {                                                                                                 // Receive settings from iDrive.      
        update_mdrive_message_settings();
      }
    }

    #if DOOR_VOLUME
    if (k_msg.id == 0xE2 || k_msg.id == 0xEA) {
      evaluate_door_status();
    }
    else if (k_msg.id == 0x663) {
      evaluate_audio_volume();
    }
    #endif

    else if (k_msg.id == 0x130) {                                                                                                   // Monitor ignition status
      evaluate_ignition_status();
      if (ignition) {
        send_power_mode();                                                                                                          // state_spt request from DME.   
        #if SERVOTRONIC_SVT70
          send_servotronic_message();
        #endif
      }
    }

    #if AUTO_SEAT_HEATING
    else if (k_msg.id == 0x2CA) {                                                                                                   // Monitor and update ambient temperature.
      evaluate_ambient_temperature();
    }

    else if (k_msg.id == 0x2FA) {                                                                                                   // Monitor and update seat status
      evaluate_passenger_seat_status();
    } 
    #endif

    #if HDC
    else if (k_msg.id == 0x2F7) {
      evaluate_speed_units();
    }
    #endif

    #if RTC
    else if (k_msg.id == 0x2F8) {
      if (k_msg.buf[0] == 0xFD && k_msg.buf[7] == 0xFC) {                                                                           // Date/time not set
        update_car_time_from_rtc();
      }
    }

    else if (k_msg.id == 0x39E) {                                                                                                   // Received new date/time from CIC.
      update_rtc_from_idrive();
    }
    #endif

    #if CKM
    else if (k_msg.id == 0x3AB) {                                                                                                   // Time POWER CKM message with Shiftlight CKM.
      send_dme_power_ckm();
    }
    #endif

    #if DEBUG_MODE
    else if (k_msg.id == 0x3B4) {                                                                                                   // Monitor battery voltage from DME.
      evaluate_battery_voltage();
    }
    #endif

    #if EDC_CKM_FIX
    else if (k_msg.id == 0x3C4) {                                                                                                   // Monitor EDC CKM message from the JBE.
      evaluate_edc_ckm_mismatch();
    }
    #endif

    #if F_ZBE_WAKE
    else if (k_msg.id == 0x273) {                                                                                                   // This message wakes up the F CIC controller.
      send_zbe_acknowledge();
    }

    else if (k_msg.id == 0x4E2) {
      send_zbe_wakeup();
    }
    #endif
  }


/***********************************************************************************************************************************************************************************************************************************************
  PT-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if (vehicle_awake) {
    if (PTCAN.read(pt_msg)) {                                                                                                       // Read data.
      if (ignition) {
        if (pt_msg.id == 0x1D6) {                                                                                                   // A button was pressed on the steering wheel.
          evaluate_m_mfl_button_press();
        }

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
          if (diagnose_svt) {
            ptcan_to_dcan();
          }
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
      if (d_msg.buf[0] == 0x1C) {                                                                                                   // Fix for LDM program issues.
        temp_deactivate_ptcan();
      }
      #if SERVOTRONIC_SVT70
      else if (d_msg.buf[0] == 0xE) {                                                                                               // SVT_70 address is 0xE
        if (diagnose_svt) {
          dcan_to_ptcan();                                                                                                          // Forward Diagnostic requests to the SVT module from DCAN to PTCAN
        }
      } 
      #endif
      #if RTC
      else if (d_msg.buf[0] == 0x60 && (d_msg.buf[1] == 0x10 || d_msg.buf[1] == 0x21)) {                                            // KOMBI is at address 0x60. ISTA sets time by sending it to KOMBI.
        update_rtc_from_dcan();
      }
      #endif
      else if (d_msg.buf[0] == 0x12) {                                                                                              // DME jobs such as MHD monitoring should be exempt.
        // do nothing.
      }
      disable_diag_transmit_jobs();                                                                                                 // Implement a check so as to not interfere with other DCAN jobs sent to the CIC by an OBD tool.    
    }
  }
  
/**********************************************************************************************************************************************************************************************************************************************/

  #if DEBUG_MODE && CDC2_STATUS_INTERFACE == 2
    if (millis() - debug_print_timer >= 500) {
      print_current_state();                                                                                                        // Print program status to the second Serial port.
    }
    loop_timer = micros();
  #endif
  
  wdt.feed();                                                                                                                       // Reset the watchdog timer.
}
