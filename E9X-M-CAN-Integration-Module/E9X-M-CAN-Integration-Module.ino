#include "src/mcp_can.h"
#include <avr/power.h>
#include <EEPROM.h>

/***********************************************************************************************************************************************************************************************************************************************
  Board configuration section.
***********************************************************************************************************************************************************************************************************************************************/

MCP_CAN PTCAN(17), KCAN(9);                                                                                                         // CS pins. Adapt to your board.
#define PTCAN_INT_PIN 7                                                                                                             // INT pins. Adapt to your board.
#define KCAN_INT_PIN 8                                                                              
#define POWER_LED_PIN 4
#define POWER_BUTTON_PIN 5
#define DSC_BUTTON_PIN 6
#define FOG_LED_PIN 12
#define EXHAUST_FLAP_SOLENOID_PIN 10
const int MCP2515_PTCAN = 1;                                                                                                        // Set 1 for 16MHZ or 2 for 8MHZ.
const int MCP2515_KCAN = 1;

/***********************************************************************************************************************************************************************************************************************************************
  Program configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#pragma GCC optimize ("-O3")                                                                                                        // Compiler optimisation level. For this file only. Edit platform.txt for all files.
#define DEBUG_MODE 0                                                                                                                // Toggle serial debug messages. Disable in production.
#define DISABLE_USB 0                                                                                                               // In production operation the USB interface is not needed.

#define FTM_INDICATOR 1                                                                                                             // Indicate FTM (Flat Tyre Monitor) status when using M3 RPA hazards button cluster.
#define FRONT_FOG_INDICATOR 1                                                                                                       // Turn on an external LED when front fogs are on. M3 clusters lack this indicator.
#define EXHAUST_FLAP_CONTROL 1                                                                                                      // Take control of the exhaust flap solenoid.
#define LAUNCH_CONTROL_INDICATOR 1                                                                                                  // Show launch control indicator (use with MHD lauch control, 6MT).
#define CONTROL_SHIFTLIGHTS 1                                                                                                       // Display shiftlights, animation and sync with the variable redline.
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver in low temperatures.
#define F_ZBE_WAKE 0                                                                                                                // Enable/disable F CIC ZBE wakeup functions.

const uint16_t DME_FAKE_VEH_MODE_CANID = 0x7F1;                                                                                     // New CAN-ID replacing 0x315 in DME [Program] section.
const uint8_t AUTO_SEAT_HEATING_TRESHOLD = 10 * 2 + 80;                                                                             // Degrees Celsius temperature * 2 + 80.
const uint8_t DTC_BUTTON_TIME = 7;                                                                                                  // Set duration for Enabling/Disabling DTC mode on with long press of M button. 100ms increments.
#if CONTROL_SHIFTLIGHTS
const uint32_t START_UPSHIFT_WARN_RPM = 5500*4;                                                                                     // RPM setpoints (warning = desired RPM * 4).
const uint32_t MID_UPSHIFT_WARN_RPM = 6000*4;
const uint32_t MAX_UPSHIFT_WARN_RPM = 6500*4;
#endif
#if EXHAUST_FLAP_CONTROL
  const uint32_t EXHAUST_FLAP_QUIET_RPM = 3500*4;                                                                                   // RPM setpoint to open the exhaust flap in normal mode (desired RPM * 4).
#endif
#if LAUNCH_CONTROL_INDICATOR
  const uint32_t LC_RPM = 2500*4;                                                                                                   // RPM setpoint to display launch control flag CC (desired RPM * 4). Match with MHD setting.
  const uint32_t LC_RPM_MIN = LC_RPM - 250;
  const uint32_t LC_RPM_MAX = LC_RPM + 250;
#endif
#if CONTROL_SHIFTLIGHTS
  const uint32_t VAR_REDLINE_OFFSET_RPM = -300;                                                                                     // RPM difference between DME requested redline and KOMBI displayed redline. Varies with cluster.
#endif

/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

unsigned long int ptrxId, krxId;
unsigned char ptrxBuf[8], krxBuf[8], ptlen, klen;

bool ignition = false;
bool vehicle_awake = true;
bool mdrive_settings_change = false;
unsigned long vehicle_awake_timer;
uint8_t dtc_button_pressed[] = {0xFD, 0xFF}, dtc_button_released[] = {0xFC, 0xFF};
uint8_t dsc_off_fake_cc_status[] = {0x40, 0x24, 0, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF};

bool engine_running = false;
uint32_t RPM = 0;
#if CONTROL_SHIFTLIGHTS
uint8_t shiftlights_start[] = {0x86, 0x3E};
uint8_t shiftlights_mid_buildup[] = {0xF6, 0};
uint8_t shiftlights_startup_buildup[] = {0x56, 0};                                                                                   // Faster sequential buildup. First byte 0-0xF (0xF - slowest).
uint8_t shiftlights_max_flash[] = {0x0A, 0};
uint8_t shiftlights_off[] = {0x05, 0};
bool shiftlights_segments_active = false;
uint8_t ignore_shiftlights_off_counter = 0;
uint32_t START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
uint32_t MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
uint32_t MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
  bool engine_warmed_up = false;
  uint32_t var_redline_position;
  uint8_t last_var_rpm_can = 0;
#endif

uint8_t mdrive_dsc = 0x03, mdrive_power = 0, mdrive_edc = 0x20, mdrive_svt = 0xE9;
#if CONTROL_SHIFTLIGHTS
uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x97};                                                                                   // byte 5: shiftlights always on
#else
uint8_t mdrive_message[] = {0, 0, 0, 0, 0, 0x87};
#endif
bool mdrive_status = false;                                                                                                         // false = off, true = on
bool power_mode = false;
uint8_t power_mode_only_dme_veh_mode[] = {0, 0x11};
uint8_t dsc_program_status = 0;                                                                                                     // 0 = on, 1 = DTC, 2 = DSC OFF
uint8_t dsc_program_last_status_can = 0xEA;
bool holding_dsc_off_console = false;
unsigned long mdrive_message_timer, m_button_debounce_timer;
unsigned long power_button_debounce_timer, dsc_off_button_debounce_timer, dsc_off_button_hold_timer;
const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 200, dsc_hold_time_ms = 400;

#if FRONT_FOG_INDICATOR
  bool front_fog_status = false;
  uint8_t last_light_status = 0;
#endif
#if FTM_INDICATOR
  bool ftm_indicator_status = false;
  uint8_t ftm_indicator_flash[] = {0x40, 0x50, 0x01, 0x69, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t ftm_indicator_off[] = {0x40, 0x50, 0x01, 0, 0xFF, 0xFF, 0xFF, 0xFF};
#endif
#if F_ZBE_WAKE
  uint8_t f_wakeup[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                                           // Network management kombi, F-series
  uint8_t zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  unsigned long zbe_wakeup_last_sent;
#endif
#if EXHAUST_FLAP_CONTROL
  bool exhaust_flap_sport = false, exhaust_flap_open = true;
  unsigned long exhaust_flap_action_timer;
#endif
#if LAUNCH_CONTROL_INDICATOR
  uint8_t lc_cc_on[] = {0x40, 0xBE, 0x01, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t lc_cc_off[] = {0x40, 0xBE, 0x01, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
  bool lc_cc_active = false;
  bool clutch_pressed = false;
  bool vehicle_moving = false;
#endif
#if AUTO_SEAT_HEATING
  uint8_t ambient_temperature_can = 255;
  bool sent_seat_heating_request = false;
  uint8_t seat_heating_button_pressed[] = {0xFD, 0xFF}, seat_heating_button_released[] = {0xFC, 0xFF};
#endif
#if DEBUG_MODE
  char serial_debug_string[128];
#endif


void setup() 
{
  configure_pins();
  disable_unused_mcu_peripherals();
  initialize_can_controllers();
  read_mdrive_settings_from_eeprom();
  initialize_timers();
}

void loop()
{

/***********************************************************************************************************************************************************************************************************************************************
  Centre console button section.
***********************************************************************************************************************************************************************************************************************************************/

  if (ignition) {
    if (!digitalRead(POWER_BUTTON_PIN)) {
      if ((millis() - power_button_debounce_timer) >= power_debounce_time_ms) {
        power_button_debounce_timer = millis();
        if (!mdrive_status) {
          power_mode = !power_mode;                                                                                                 // POWER console button should only change throttle mapping.
          if (power_mode) {
            #if DEBUG_MODE
              Serial.println(F("Console: POWER mode ON."));
            #endif 
          } else {
            #if DEBUG_MODE
              Serial.println(F("Console: POWER mode OFF."));
            #endif 
          }
        }
      }
    } else if (!digitalRead(DSC_BUTTON_PIN)) {
      if (dsc_program_status == 0) {
        if (!holding_dsc_off_console) {
          holding_dsc_off_console = true;
          dsc_off_button_hold_timer = millis();
        } else {
          if ((millis() - dsc_off_button_hold_timer) >= dsc_hold_time_ms) {                                                         // DSC OFF sequence should only be sent after user holds button for a configured time
            #if DEBUG_MODE
              Serial.println(F("Console: DSC OFF button held. Sending DSC OFF."));
            #endif
            send_dsc_off_sequence();
            dsc_off_button_debounce_timer = millis();
          }
        }      
      } else {
        if ((millis() - dsc_off_button_debounce_timer) >= dsc_debounce_time_ms) {                                                   // A quick tap re-enables everything
          #if DEBUG_MODE
            Serial.println(F("Console: DSC button tapped. Re-enabling DSC normal program."));
          #endif
          dsc_off_button_debounce_timer = millis();
          send_dtc_button_press();
        }
      }
    } else {
      holding_dsc_off_console = false;
    }

    if ((millis() - mdrive_message_timer) > 9000) {                                                                                 // Time MDrive message outside of CAN loops. Original cycle time is 10s (idle).                                                                     
      #if DEBUG_MODE
        Serial.println(F("Sending Ignition MDrive alive message."));
      #endif
      send_mdrive_message();
    }
  } else {
    if (((millis() - vehicle_awake_timer) > 10000) && vehicle_awake) {
      vehicle_awake = false;                                                                                                        // Vehicle must now Asleep. Stop transmitting.
      #if DEBUG_MODE
        Serial.println(F("Vehicle Sleeping."));
      #endif
      toggle_ptcan_sleep();
    }
    if (((millis() - mdrive_message_timer) > 15000) && vehicle_awake) {                                                             // Send this message while car is awake to populate the fields in iDrive.                                                                     
      #if DEBUG_MODE
        Serial.println(F("Sending Vehicle Awake MDrive alive message."));
      #endif
      send_mdrive_message();
    }
  }

/***********************************************************************************************************************************************************************************************************************************************
  PT-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if (!digitalRead(PTCAN_INT_PIN)) {                                                                                                // If INT pin is pulled low, read PT-CAN receive buffer
    PTCAN.readMsgBuf(&ptrxId, &ptlen, ptrxBuf);                                                                                     // Read data: rxId = CAN ID, buf = data byte(s)
    if (ignition) {
      if (ptrxId == 0x1D6) {
        if ((millis() - m_button_debounce_timer) >= 200) {       
          if (ptrxBuf[1] == 0x4C) {                                                                                                 // M button is pressed
            toggle_mdrive_message_active();
            send_mdrive_message();
          } 
          m_button_debounce_timer = millis();
        }
      }
     
      #if FRONT_FOG_INDICATOR
      else if (ptrxId == 0x21A) {
        evaluate_fog_status();
      }
      #endif

      #if FTM_INDICATOR
      else if (ptrxId == 0x31D) {                                                                                                   // FTM initialization is ongoing.
        evaluate_ftm_status();
      }
      #endif

      #if CONTROL_SHIFTLIGHTS
        else if (ptrxId == 0x332) {                                                                                                  // Monitor variable redline broadcast from DME.
          evaluate_shiftlight_sync();
        }
      #endif

      else if (ptrxId == 0x3CA) {                                                                                                   // Receive settings from iDrive.      
        if (ptrxBuf[4] == 0xEC || ptrxBuf[4] == 0xF4 || ptrxBuf[4] == 0xE4) {                                                       // Reset requested.
          update_mdrive_message_settings(true);
          send_mdrive_message();
        } else if ((ptrxBuf[4] == 0xE0 || ptrxBuf[4] == 0xE1)) {                                                                    // Ignore E0/E1 (Invalid) 
        } else {
          update_mdrive_message_settings(false);
          send_mdrive_message();                                                                                                    // Respond to iDrive.
        }
      }
    }
  }

/***********************************************************************************************************************************************************************************************************************************************
  K-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if(!digitalRead(KCAN_INT_PIN)) {                                                                                                  // If INT pin is pulled low, read K-CAN receive buffer.
    KCAN.readMsgBuf(&krxId, &klen, krxBuf);

    if (krxId == 0x19E) {                                                                                                           // Monitor DSC K-CAN status.
      evaluate_dsc_ign_status();
    }

    #if AUTO_SEAT_HEATING
    else if (krxId == 0x2CA){                                                                                                       // Monitor and update ambient temperature.
      ambient_temperature_can = krxBuf[0];
    } 
    
    else if (krxId == 0x232) {                                                                                                      // Driver's seat heating status message is only sent with ignition on.
      if (!krxBuf[0]) {                                                                                                             // Check if seat heating is already on.
        //This will be ignored if already on and cycling ignition. Press message will be ignored by IHK anyway.
        if (!sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) {
          send_seat_heating_request();
        }
      }
    }
    #endif

    if (ignition) {
      if (krxId == 0xAA) {                                                                                                          // Monitor 0xAA (rpm/throttle status).
        send_power_mode();                                                                                                          // state_spt request from DME.   

        RPM = ((uint32_t)krxBuf[5] << 8) | (uint32_t)krxBuf[4];
        #if CONTROL_SHIFTLIGHTS
          evaluate_shiftlight_display();
        #endif
        #if EXHAUST_FLAP_CONTROL
          if (engine_running) {
            evaluate_exhaust_flap_position();
          }
        #endif
        #if LAUNCH_CONTROL_INDICATOR
          evaluate_lc_display();
        #endif
      }

      #if LAUNCH_CONTROL_INDICATOR
      else if (krxId == 0xA8) {
        evaluate_clutch_status();
      }

      else if (krxId == 0x1B4) {
        if (ignition) {    
          evaluate_vehicle_moving();
        } 
      }
      #endif
    }
  }
}

/***********************************************************************************************************************************************************************************************************************************************
  EOF
***********************************************************************************************************************************************************************************************************************************************/