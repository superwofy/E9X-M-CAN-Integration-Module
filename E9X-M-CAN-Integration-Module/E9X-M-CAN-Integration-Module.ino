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
#if DEBUG_MODE
#define EXTRA_DEBUG 0
#else
#define DISABLE_USB 0                                                                                                               // In production operation the USB interface is not needed.
#endif

#define RHD 1
#define FTM_INDICATOR 1                                                                                                             // Indicate FTM (Flat Tyre Monitor) status when using M3 RPA hazards button cluster.
#define REVERSE_BEEP 1                                                                                                              // Play a beep throught he speaker closest to the driver when engaging reverse.
#define FRONT_FOG_INDICATOR 1                                                                                                       // Turn on an external LED when front fogs are on. M3 clusters lack this indicator.
#define SERVOTRONIC_SVT70 1                                                                                                         // Control steering assist with modified SVT70 module.
#define EXHAUST_FLAP_CONTROL 1                                                                                                      // Take control of the exhaust flap solenoid.
#if EXHAUST_FLAP_CONTROL
#define QUIET_START 1                                                                                                               // Close the exhaust valve as soon as the module is powered (30G).
#endif
#define LAUNCH_CONTROL_INDICATOR 1                                                                                                  // Show launch control indicator (use with MHD lauch control, 6MT).
#define CONTROL_SHIFTLIGHTS 1                                                                                                       // Display shiftlights, animation and sync with the variable redline.
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver in low temperatures.
#define RTC 1                                                                                                                       // Set the time/date if power is lost. Requires external battery.
#if RTC
  #include <TimeLib.h>
#endif
#define F_ZBE_WAKE 0                                                                                                                // Enable/disable FXX CIC ZBE wakeup functions. Do not use with an existing EXX ZBE.

#if SERVOTRONIC_SVT70
  const uint16_t SVT_FAKE_EDC_MODE_CANID = 0x327;                                                                                   // New CAN-ID replacing 0x326 in SVT70 bin.
#endif
const uint16_t DME_FAKE_VEH_MODE_CANID = 0x7F1;                                                                                     // New CAN-ID replacing 0x315 in DME [Program] section.
const uint8_t AUTO_SEAT_HEATING_TRESHOLD = 10 * 2 + 80;                                                                             // Degrees Celsius temperature * 2 + 80.
#if CONTROL_SHIFTLIGHTS
  const uint32_t START_UPSHIFT_WARN_RPM = 5500*4;                                                                                   // RPM setpoints (warning = desired RPM * 4).
  const uint32_t MID_UPSHIFT_WARN_RPM = 6000*4;
  const uint32_t MAX_UPSHIFT_WARN_RPM = 6500*4;
#endif
#if EXHAUST_FLAP_CONTROL
  const uint32_t EXHAUST_FLAP_QUIET_RPM = 3500*4;                                                                                   // RPM setpoint to open the exhaust flap in normal mode (desired RPM * 4).
#endif
#if LAUNCH_CONTROL_INDICATOR
  const uint32_t LC_RPM = 4000*4;                                                                                                   // RPM setpoint to display launch control flag CC (desired RPM * 4). Match with MHD setting.
  const uint32_t LC_RPM_MIN = LC_RPM - (250 * 4);
  const uint32_t LC_RPM_MAX = LC_RPM + (250 * 4);
#endif
#if CONTROL_SHIFTLIGHTS
  const uint32_t VAR_REDLINE_OFFSET_RPM = -300;                                                                                     // RPM difference between DME requested redline and KOMBI displayed redline. Varies with cluster.
#endif

const float TOP_THRESHOLD = 65.0;                                                                                                   // CPU temperature thresholds for the processor clock scaling function.
const float MEDIUM_THRESHOLD = 60.0;
const float HYSTERESIS = 2.0;
const unsigned long MEDIUM_UNDERCLOCK = 396 * 1000000;
const unsigned long MAX_UNDERCLOCK = 24 * 1000000;

/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

CAN_message_t pt_msg, k_msg, d_msg;
typedef struct delayedCanTxMsg {
	CAN_message_t	txMsg;
	unsigned long	transmitTime;
} delayedCanTxMsg;

cppQueue dtcTx(sizeof(delayedCanTxMsg), 6, queue_FIFO); 
cppQueue dscTx(sizeof(delayedCanTxMsg), 26, queue_FIFO);

uint32_t cpu_speed_ide;

bool ignition = false;
bool vehicle_awake = true;
unsigned long vehicle_awake_timer;
uint8_t dtc_button_pressed[] = {0xFD, 0xFF}, dtc_button_released[] = {0xFC, 0xFF};
CAN_message_t dtc_button_pressed_buf, dtc_button_released_buf;
uint8_t dsc_off_fake_cc_status[] = {0x40, 0x24, 0, 0x1D, 0xFF, 0xFF, 0xFF, 0xAB};
uint8_t mdm_fake_cc_status[] = {0x40, 0xB8, 0, 0x45, 0xFF, 0xFF, 0xFF, 0xAB};
uint8_t mdm_fake_cc_status_off[] = {0x40, 0xB8, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB};
CAN_message_t dsc_off_fake_cc_status_buf, mdm_fake_cc_status_buf, mdm_fake_cc_status_off_buf;

bool engine_running = false;
uint32_t RPM = 0;
uint8_t mdrive_dsc, mdrive_power, mdrive_edc, mdrive_svt;
bool mdrive_status = false;                                                                                                         // false = off, true = on
bool mdrive_settings_updated = false;
bool console_power_mode, restore_console_power_mode = false;
bool mdrive_power_active = false;
uint8_t power_mode_only_dme_veh_mode[] = {0xE8, 0xF1};                                                                              // E8 is the last checksum. Start will be from 0A.
uint8_t dsc_program_status = 0;                                                                                                     // 0 = on, 1 = DTC, 2 = DSC OFF
bool holding_dsc_off_console = false;
unsigned long mdrive_message_timer;
unsigned long mfl_debounce_timer, power_button_debounce_timer, dsc_off_button_debounce_timer, dsc_off_button_hold_timer;
uint16_t mfl_debounce_time_ms = 0;
const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 200, dsc_hold_time_ms = 400;
bool sending_dsc_off = false;
uint8_t sending_dsc_off_counter = 0;
bool send_dsc_off_from_mdm = false;
unsigned long send_dsc_off_from_mdm_timer;
bool ignore_m_press = false;
uint8_t clock_mode = 0;
float last_cpu_temp = 0;


#if SERVOTRONIC_SVT70
  uint8_t servotronic_message[] = {0, 0xFF};
  uint8_t servotronic_cc_on[] = {0x40, 0x46, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
  CAN_message_t servotronic_cc_on_buf;
  bool diagnose_svt = false;
  #if DEBUG_MODE
    uint32_t dcan_forwarded_count = 0, ptcan_forwarded_count = 0;
  #endif
#endif
#if CONTROL_SHIFTLIGHTS
uint8_t shiftlights_start[] = {0x86, 0x3E};
uint8_t shiftlights_mid_buildup[] = {0xF6, 0};
uint8_t shiftlights_startup_buildup[] = {0x56, 0};                                                                                   // Faster sequential buildup. First byte 0-0xF (0xF - slowest).
uint8_t shiftlights_max_flash[] = {0xA, 0};
uint8_t shiftlights_off[] = {5, 0};
CAN_message_t shiftlights_start_buf, shiftlights_mid_buildup_buf, shiftlights_startup_buildup_buf;
CAN_message_t shiftlights_max_flash_buf, shiftlights_off_buf;
bool shiftlights_segments_active = false;
uint8_t ignore_shiftlights_off_counter = 0;
uint32_t START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
uint32_t MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
uint32_t MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
  bool engine_coolant_warmed_up = false;
  uint32_t var_redline_position;
  uint8_t last_var_rpm_can = 0;
uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x97};                                                                                   // byte 5: shiftlights always on
#else
uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x87};                                                                                   // byte 5: shiftlights unchanged
#endif
#if FRONT_FOG_INDICATOR
  bool front_fog_status = false;
  uint8_t last_light_status = 0;
#endif
#if FTM_INDICATOR
  bool ftm_indicator_status = false;
  uint8_t ftm_indicator_flash[] = {0x40, 0x50, 1, 0x69, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t ftm_indicator_off[] = {0x40, 0x50, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF};
  CAN_message_t ftm_indicator_flash_buf, ftm_indicator_off_buf;
#endif
#if REVERSE_BEEP
  #if RHD 
    uint8_t pdc_beep[] = {0, 0, 0, 1};                                                                                              // Front right beep.
  #else
    uint8_t pdc_beep[] = {0, 0, 1, 0};                                                                                              // Front left beep.
  #endif
  uint8_t pdc_quiet[] = {0, 0, 0, 0};
  CAN_message_t pdc_beep_buf, pdc_quiet_buf;
  bool pdc_beep_sent = false;
  cppQueue pdcBeepTx(sizeof(delayedCanTxMsg), 4, queue_FIFO);
#endif
#if F_ZBE_WAKE
  uint8_t f_wakeup[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                                           // Network management KOMBI - F-series.
  CAN_message_t f_wakeup_buf;
  uint8_t zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  unsigned long zbe_wakeup_last_sent;
#endif
#if EXHAUST_FLAP_CONTROL
  bool exhaust_flap_sport = false, exhaust_flap_open = true;
  unsigned long exhaust_flap_action_timer;
  unsigned long exhaust_flap_action_interval = 1500;
#endif
#if LAUNCH_CONTROL_INDICATOR
  uint8_t lc_cc_on[] = {0x40, 0xBE, 1, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t lc_cc_off[] = {0x40, 0xBE, 1, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
  CAN_message_t lc_cc_on_buf, lc_cc_off_buf;
  bool lc_cc_active = false;
  bool mdm_with_lc = false;
  bool clutch_pressed = false;
  bool vehicle_moving = false;
#endif
#if AUTO_SEAT_HEATING
  bool driver_seat_heating_status = false, passenger_seat_heating_status = false;
  uint8_t ambient_temperature_can = 255;
  uint8_t passenger_seat_status = 0;                                                                                                // 0 - Not occupied not belted, 1 - not occupied and belted, 8 - occupied not belted, 9 - occupied and belted
  bool driver_sent_seat_heating_request = false, passenger_sent_seat_heating_request = false;
  uint8_t seat_heating_button_pressed[] = {0xFD, 0xFF}, seat_heating_button_released[] = {0xFC, 0xFF};
  cppQueue seatHeatingTx(sizeof(delayedCanTxMsg), 6, queue_FIFO); 
#endif
#if DEBUG_MODE
  float battery_voltage = 0;
  extern float tempmonGetTemp(void);
  char serial_debug_string[256];
  unsigned long loop_timer, debug_print_timer, max_loop_timer = 0;
#endif


void setup() 
{
  #if DEBUG_MODE
    unsigned long setup_timer = millis();
  #endif
  initialize_watchdog();
  if ( F_CPU_ACTUAL > (600 * 1000000)) set_arm_clock(600 * 1000000);                                                                // Prevent accidental overclocks. Remove if needed.
  cpu_speed_ide = F_CPU_ACTUAL;
  configure_IO();
  disable_mcu_peripherals();
  configure_can_controllers();
  cache_can_message_buffers();
  initialize_timers();
  read_settings_from_eeprom();
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Setup finished in %lu ms, module is ready.", millis() - setup_timer);
    Serial.println(serial_debug_string);
  #endif
}


void loop()
{
/***********************************************************************************************************************************************************************************************************************************************
  General section.
***********************************************************************************************************************************************************************************************************************************************/
  check_cpu_temp();                                                                                                                 // Monitor processor temperature to extend lifetime.
  check_dtc_button_queue();
  check_dsc_off_queue();
  non_blocking_mdm_to_off();
  #if EXHAUST_FLAP_CONTROL
    control_exhaust_flap_user();
  #endif

  if (ignition) {
    check_console_buttons();
    send_mdrive_alive_message(10000);
    #if AUTO_SEAT_HEATING
      check_seatheating_queue();
    #endif
    #if REVERSE_BEEP
      check_pdc_queue();
    #endif
  } else {
    if (vehicle_awake) {
      if ((millis() - vehicle_awake_timer) >= 10000) {
        vehicle_awake = false;                                                                                                      // Vehicle must now be asleep. Stop monitoring .
        #if DEBUG_MODE
          Serial.println("Vehicle Sleeping.");
        #endif
        toggle_transceiver_standby();
        scale_mcu_speed();
        #if AUTO_SEAT_HEATING
          driver_sent_seat_heating_request = false;                                                                                 // Reset the seat heating request now that the car's asleep.
        #endif
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

      else if (k_msg.id == 0x1B4) {                                                                                                 // Monitor if the car is stationary/moving
        evaluate_vehicle_moving();
      }
      #endif

      #if REVERSE_BEEP
      else if (k_msg.id == 0x3B0) {                                                                                                 // Monitor reverse status.
        evaluate_pdc_beep();
      }
      #endif
    }

    if (k_msg.id == 0x130) {                                                                                                        // Monitor ignition status
      evaluate_ignition_status();
      if (ignition) {
        send_power_mode();                                                                                                          // state_spt request from DME.   
        #if SERVOTRONIC_SVT70
          send_servotronic_message();
        #endif
      }
    }

    #if FRONT_FOG_INDICATOR
    else if (k_msg.id == 0x21A) {
      evaluate_fog_status();
    }
    #endif

    #if AUTO_SEAT_HEATING
    else if (k_msg.id == 0x22A) {                                                                                                   // Passenger's seat heating status message is only sent with ignition on.
      if (!k_msg.buf[0]) {                                                                                                          // Check if seat heating is already on.
        //This will be ignored if already on and cycling ignition. Press message will be ignored by IHK anyway.
        if (!passenger_sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD) 
            && passenger_seat_status == 9) {                                                                                        // Passenger sitting and their seatbelt is on
          send_seat_heating_request(false);
        }
      } else {
        passenger_sent_seat_heating_request = true;                                                                                 // Seat heating already on. No need to request anymore.
      }
      #if DEBUG_MODE
        passenger_seat_heating_status = !k_msg.buf[0] ? false : true;
      #endif
    }

    else if (k_msg.id == 0x232) {                                                                                                   // Driver's seat heating status message is only sent with ignition on.
      if (!k_msg.buf[0]) {                                                                                                          // Check if seat heating is already on.
        if (!driver_sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) {
          send_seat_heating_request(true);
        }
      } else {
        driver_sent_seat_heating_request = true;                                                                                    // Seat heating already on. No need to request anymore.
      }
      #if DEBUG_MODE
        driver_seat_heating_status = !k_msg.buf[0] ? false : true;
      #endif
    }

    else if (k_msg.id == 0x2CA) {                                                                                                   // Monitor and update ambient temperature.
      ambient_temperature_can = k_msg.buf[0];
    }

    else if (k_msg.id == 0x2FA) {                                                                                                   // Monitor and update seat status
      passenger_seat_status = k_msg.buf[1];
    } 
    #endif

    #if RTC
    else if (k_msg.id == 0x2F8) {
      if (k_msg.buf[0] == 0xFD && k_msg.buf[7] == 0xFC) {                                                                           // Date/time not set
        update_idrive_time_from_rtc();
      }
    }

    else if (k_msg.id == 0x39E) {                                                                                                   // Received new date/time from CIC.
      update_rtc_from_idrive();
    }
    #endif

    else if (k_msg.id == 0x3AB) {
      send_dme_ckm();
    }

    #if DEBUG_MODE
    else if (k_msg.id == 0x3B4) {                                                                                                   // Monitor battery voltage from DME.
      evaluate_battery_voltage();
    }
    #endif

    else if (k_msg.id == 0x3CA) {                                                                                                   // Receive settings from iDrive.      
      update_mdrive_message_settings();
    }

    #if F_ZBE_WAKE
    else if (k_msg.id == 0x4E2) {
      send_zbe_wakeup();
    }

    else if (k_msg.id == 0x273) {                                                                                                   // This message wakes up the F CIC controller.
      send_zbe_acknowledge();
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
        #endif

        else if (pt_msg.id == 0x5A9) {
          evaluate_dsc_program_from_cc();
        }

        #if SERVOTRONIC_SVT70
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
    
  #if SERVOTRONIC_SVT70 || RTC
  if (DCAN.read(d_msg)) {
    if (d_msg.id == 0x6F1) {
      if (d_msg.buf[0] == 0xE) {                                                                                                    // SVT_70 address is 0xE
        if (diagnose_svt) {                                                                                                         // Forward Diagnostic requests to the SVT module from DCAN to PTCAN
          dcan_to_ptcan();
        }
      } 
      #if RTC
      else if (d_msg.buf[0] == 0x60) {                                                                                              // KOMBI is at address 0x60. ISTA sets time by sending it to KOMBI.
        update_rtc_from_dcan();
      }
      #endif
    }
  }
  #endif
  
/**********************************************************************************************************************************************************************************************************************************************/

  #if DEBUG_MODE
    if (millis() - debug_print_timer >= 300) {
      print_current_state();                                                                                                        // Print program status to the second Serial port.
    }
    loop_timer = micros();
  #endif
  
  wdt.feed();                                                                                                                       // Reset the watchdog timer.
}
