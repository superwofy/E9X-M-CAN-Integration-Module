// Centre console section

// Enable use of M3 centre console button block.POWER and DSC OFF buttons. Control POWER LED when MDrive is on.
// 1. Request MDrive when POWER button is pressed.
// 2. Illuminate POWER LED when MDrive is on.
// 3. Request full DSC OFF after holding DSC OFF button. Turn everything back on with a short press.


// PT-CAN section

// 1. Toggles sport mode in the IKM0S MSD81 DME in the absence of a DSCM90 or DSCM80 ZB by re-creating 0x1D9 message.
//    Toggle DTC mode. 
//    Toggle EDC MSport program.
// 2. Monitor steering wheel buttons and request MDrive when Source/M is pressed.
// 3. Monitor MDrive status as broadcast by 1M DME.
// 4. Monitor EDC status.
// 5. Monitor and indicate front foglight status.
// 6. Monitor and indicate FTM status in KOMBI by flashing tyre icon when initializing.


// K-CAN section

// 1. Monitor ignition and DSC program status.
// 2. Create missing 0x206 message to animate DCT KOMBI shift lights.
// 3. Open/close exhaust flap.
// 4. Display Launch control flag
// 5. Create the two missing messages required to use an F series ZBE (KKCAN) in an E6,7,8,9X car.
//    *Should* work with:
//    6582 9267955 - 4-pin MEDIA button. **Tested - controller build date 19.04.13
//    6131 9253944 - 4-pin CD button
//    6582 9206444 - 10-pin CD button
//    6582 9212449 - 10-pin CD button
// 6. Turn on driver's seat heating when ignition is turned on and below configured temperature treshold.
// Credit to Trevor for providing insight into 0x273, 0x277, 0x2CA and 0x0AA http://www.loopybunny.co.uk/CarPC/k_can.html


// MSD81 IKM0S .bin modification

// 0x83738/9  -> F1 07     CAN message table
// 0x1f3ad4/5 -> F1 07     CAN filters
// Replace 15 03 (0x315 represented in LE) to stop MSD81 from reacting to Vehicle Mode changes (triggered by JBBF through EDC button)
// This also allows state_spt (throttle map) to be controlled independently from the main "MDrive" with the console switch
// Re-calculate checksum at 0x80304 


// Using a slightly streamlined version of Cory's library https://github.com/coryjfowler/MCP_CAN_lib
// Hardware used: CANBED V1.2c http://docs.longan-labs.cc/1030008/ (32U4+MCP2515+MCP2551, LEDs removed) and Generic 16MHz MCP2515 CAN shield.

#include "src/mcp_can.h"
#include <avr/power.h>

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
#define EDC_BUTTON_PIN 11
#define EXHAUST_FLAP_SOLENOID_PIN 10
const int MCP2515_PTCAN = 1;                                                                                                        // Set 1 for 16MHZ or 2 for 8MHZ.
const int MCP2515_KCAN = 1;

/***********************************************************************************************************************************************************************************************************************************************
  Program configuration section.
***********************************************************************************************************************************************************************************************************************************************/

#pragma GCC optimize ("-O3")                                                                                                        // Compiler optimisation level. For this file only. Edit platform.txt for all files.
#define DEBUG_MODE 0                                                                                                                // Toggle serial debug messages. Disable in production.
#define DISABLE_USB 0                                                                                                               // In production operation the USB interface is not needed.

#define FTM_INDICATOR 1                                                                                                             // Indicate FTM status when using M3 RPA hazards button cluster.
#define FRONT_FOG_INDICATOR 1                                                                                                       // Turn on an LED when front fogs are on. M3 clusters lack this.
#define F_ZBE_WAKE 0                                                                                                                // Enable/disable F CIC ZBE wakeup functions.
#define DTC_WITH_M_BUTTON 1                                                                                                         // Toggle DTC mode with M MFL button.
#define EXHAUST_FLAP_WITH_M_BUTTON 1                                                                                                // Take control of the E9X exhaust flap solenoid
#define EDC_WITH_M_BUTTON 1                                                                                                         // Toggle EDC mode with M MFL button. If SVT is installed, Servotronic assist curve changes too.
#define LAUNCH_CONTROL_INDICATOR 1                                                                                                  // Show launch control indicator (use with MHD lauch control, 6MT)
#define SYNC_SHIFTLIGHTS_WITH_REDLINE 1                                                                                             // Display shiftlights with the variable redline
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver in low temperatures.

const uint8_t AUTO_SEAT_HEATING_TRESHOLD = 10 * 2 + 80;                                                                             // Degrees Celsius temperature * 2 + 80.
const uint8_t DTC_BUTTON_TIME = 7;                                                                                                  // Set duration for Enabling/Disabling DTC mode on with long press of M button. 100ms increments.
const uint32_t START_UPSHIFT_WARN_RPM = 5500*4;                                                                                     // RPM setpoints (warning = desired RPM * 4).
const uint32_t MID_UPSHIFT_WARN_RPM = 6000*4;
const uint32_t MAX_UPSHIFT_WARN_RPM = 6500*4;

#if EXHAUST_FLAP_WITH_M_BUTTON
  const uint32_t EXHAUST_FLAP_QUIET_RPM = 3500*4;                                                                                   // RPM setpoint to open the exhaust flap in normal mode (desired RPM * 4).
#endif
#if LAUNCH_CONTROL_INDICATOR
  const uint32_t LC_RPM = 2500*4;                                                                                                   // RPM setpoint to display launch control flag CC (desired RPM * 4). Match with MHD setting.
  const uint32_t LC_RPM_MIN = LC_RPM - 250;
  const uint32_t LC_RPM_MAX = LC_RPM + 250;
#endif
#if SYNC_SHIFTLIGHTS_WITH_REDLINE
  const uint32_t VAR_REDLINE_OFFSET_RPM = 250;                                                                                      // RPM difference between DME requested redline and KOMBI displayed redline. Varies with cluster.
#endif


/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

unsigned long int prxId, krxId;
unsigned char prxBuf[8], krxBuf[8], plen, klen;

bool ignition = false;

byte mbutton_released[] = {0xFF, 0x3F, 0}, mbutton_pressed[] = {0xBF, 0x7F, 0};
uint8_t mbutton_checksum = 0xF0;
unsigned long mbutton_released_timer;

byte dtc_button_pressed[] = {0xFD, 0xFF}, dtc_button_released[] = {0xFC, 0xFF};
byte dsc_off_fake_cc_status[] = {0x40, 0x24, 0, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF};

byte shiftlights_start[] = {0x86, 0x3E};
byte shiftlights_mid_buildup[] = {0xF6, 0};
byte shiftlights_startup_buildup[] = {0x56, 0};																						                                          // Faster sequential buildup. First byte 0-0xF (0xF - slowest).
byte shiftlights_max_flash[] = {0x0A, 0};
byte shiftlights_off[] = {0x05, 0};
bool shiftlights_segments_active = false;
bool engine_running = false;
uint8_t ignore_shiftlights_off_counter = 0;
uint32_t RPM = 0;
uint32_t START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
uint32_t MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
uint32_t MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
#if SYNC_SHIFTLIGHTS_WITH_REDLINE
  bool engine_warmed_up = false;
  uint32_t var_redline_position;
  uint8_t last_var_rpm_can = 0;
#endif

bool mdrive_status = false;                                                                                                         // false = off, true = on
bool power_mode_only = false;
byte power_mode_only_dme_veh_mode[] = {0, 0x11};
uint8_t mdrive_last_status_can = 0xCF;                                                                                              // OFF by default
uint8_t dsc_program_status = 0;                                                                                                     // 0 = on, 1 = DTC, 2 = DSC OFF
uint8_t dsc_program_last_status_can = 0xEA;
bool holding_dsc_off_console = false;
unsigned long power_button_debounce_timer, dsc_off_button_debounce_timer, dsc_off_button_hold_timer;
const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 200, dsc_hold_time_ms = 400;

#if FRONT_FOG_INDICATOR
  bool front_fog_status = false;
  uint8_t last_light_status = 0;
#endif
#if FTM_INDICATOR
  bool ftm_indicator_status = false;
  byte ftm_indicator_flash[] = {0x40, 0x50, 0x01, 0x69, 0xFF, 0xFF, 0xFF, 0xFF};
  byte ftm_indicator_off[] = {0x40, 0x50, 0x01, 0, 0xFF, 0xFF, 0xFF, 0xFF};
#endif
#if F_ZBE_WAKE
  byte f_wakeup[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                                              // Network management kombi, F-series
  byte zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  unsigned long zbe_wakeup_last_sent;
#endif
#if EDC_WITH_M_BUTTON
  uint8_t edc_status = 1;                                                                                                           // 1 = comfort, 2 = sport, 0xA = msport
  uint8_t edc_last_status_can = 0xF1;
#endif
#if EXHAUST_FLAP_WITH_M_BUTTON
  bool exhaust_flap_sport = false, exhaust_flap_open = true;
  unsigned long exhaust_flap_action_timer;
#endif
#if LAUNCH_CONTROL_INDICATOR
  byte lc_cc_on[] = {0x40, 0xBE, 0x01, 0x39, 0xFF, 0xFF, 0xFF, 0xFF};
  byte lc_cc_off[] = {0x40, 0xBE, 0x01, 0x30, 0xFF, 0xFF, 0xFF, 0xFF};
  bool lc_cc_active = false;
  bool clutch_pressed = false;
  bool vehicle_moving = false;
#endif
#if AUTO_SEAT_HEATING
  uint8_t ambient_temperature_can = 255;
  bool sent_seat_heating_request = false;
  byte seat_heating_button_pressed[] = {0xFD, 0xFF}, seat_heating_button_released[] = {0xFC, 0xFF};
#endif
#if DEBUG_MODE
  char serial_debug_string[128];
#endif


void setup() 
{
  configure_pins();
  disable_unused_peripherals();
  #if DEBUG_MODE
    Serial.begin(115200);
    while(!Serial);                                                                                                                 // 32U4, wait until virtual port initialized
  #endif
  
  initialize_can_controllers();
  configure_filters();
    
  #if F_ZBE_WAKE
    power_button_debounce_timer = dsc_off_button_debounce_timer = mbutton_released_timer = zbe_wakeup_last_sent = millis();
  #else
    power_button_debounce_timer = dsc_off_button_debounce_timer = mbutton_released_timer = millis();
  #endif
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
          power_mode_only = !power_mode_only;                                                                                       // POWER console button should only change throttle mapping.
          if (power_mode_only) {
            digitalWrite(POWER_LED_PIN, HIGH);
            #if DEBUG_MODE
              Serial.println(F("Console: POWER mode on."));
            #endif 
          } else {
            digitalWrite(POWER_LED_PIN, LOW);
            #if DEBUG_MODE
              Serial.println(F("Console: POWER mode off."));
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
  } 

/***********************************************************************************************************************************************************************************************************************************************
  PT-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if (!digitalRead(PTCAN_INT_PIN)) {                                                                                                // If INT pin is pulled low, read PT-CAN receive buffer
    PTCAN.readMsgBuf(&prxId, &plen, prxBuf);                                                                                           // Read data: rxId = CAN ID, buf = data byte(s)

    if (ignition) {
      if (prxId == 0x1D6) {       
        if (prxBuf[1] == 0x4C) {                                                                                                     // M button is pressed
            send_mbutton_message(mbutton_pressed);
        } else if (prxBuf[0] == 0xC0 && prxBuf[1] == 0x0C) {                                                                          // MFL buttons released, send alive ping.
          send_mbutton_message(mbutton_released);                                                                                       
        } else {
          if ((millis() - mbutton_released_timer) >= 1000) {                                                                        // keep sending MDrive released messages when other buttons are pressed/held
            send_mbutton_message(mbutton_released);
          }
        }
      }
      #if LAUNCH_CONTROL_INDICATOR
        else if (prxId == 0x1B4) {    

          // Time with 1B4 for faster reaction
          power_mode_only_dme_veh_mode[0] += 0x10;                                                                                  // Increase alive counter
          if (power_mode_only_dme_veh_mode[0] > 0xEF) {                                                                             // Alive(first half of byte) 0..E
            power_mode_only_dme_veh_mode[0] = 0;
          }
          if (power_mode_only) {
            power_mode_only_dme_veh_mode[1] = 0x22;                                                                                 // Sport
            veh_mode_checksum();
            PTCAN.sendMsgBuf(0x7F1, 2, power_mode_only_dme_veh_mode);
          } else {
            power_mode_only_dme_veh_mode[1] = 0x11;                                                                                 // Normal
            veh_mode_checksum();
            PTCAN.sendMsgBuf(0x7F1, 2, power_mode_only_dme_veh_mode);
          }

          evaluate_vehicle_moving();
        }
      #endif
     
      #if FRONT_FOG_INDICATOR
        else if (prxId == 0x21A) {
          evaluate_fog_status();
        }
      #endif

      else if (prxId == 0x315) {      
        #if EDC_WITH_M_BUTTON
          if (prxBuf[1] != edc_last_status_can) {
            edc_status = prxBuf[1] - 0xF0;
            #if DEBUG_MODE
              switch(edc_status) {
                case 1:
                  Serial.println(F("EDC now in Comfort mode."));
                  break;
                case 2:
                  Serial.println(F("EDC now in Sport mode."));
                  break;
                case 0xA:
                  Serial.println(F("EDC now in MSport mode."));
                  break;
              }
            #endif
            edc_last_status_can = prxBuf[1];
          }
        #endif
      } 

      #if FTM_INDICATOR
        else if (prxId == 0x31D) {                                                                                                   // FTM initialization is ongoing.
          evaluate_ftm_status();
        }
      #endif

      else if (prxId == 0x399) {                                                                                                     // Monitor MDrive status on PT-CAN and control centre console POWER LED
        if (mdrive_last_status_can != prxBuf[4]) {
          if (prxBuf[4] == 0xDF) {
            mdrive_status = true;
            if (!power_mode_only) {
              digitalWrite(POWER_LED_PIN, HIGH);
            }
            #if DEBUG_MODE
              Serial.println(F("Status MDrive on."));
            #endif
            #if EXHAUST_FLAP_WITH_M_BUTTON
              exhaust_flap_sport = true;
            #endif
            mdrive_extra_functions();
          } else {
            mdrive_status = false;
            if (!power_mode_only) {
              digitalWrite(POWER_LED_PIN, LOW);
            }
            #if DEBUG_MODE
              Serial.println(F("Status MDrive off."));
            #endif
            #if DTC_WITH_M_BUTTON
              if (dsc_program_status == 1) {                                                                                        // Turn off DTC together with MDrive
                send_dtc_button_press();
                #if DEBUG_MODE
                  Serial.println(F("Turned off DTC with MDrive off."));
                #endif
              }
            #endif
            #if EDC_WITH_M_BUTTON
              if (edc_status == 0xA) {                                                                                              // Turn off EDC MSport program together with MDrive (only if toggled with MDrive)
                send_edc_button_press();
                #if DEBUG_MODE
                  Serial.println(F("Turned off EDC MSport with MDrive off."));
                #endif
              }
            #endif
            #if EXHAUST_FLAP_WITH_M_BUTTON
              exhaust_flap_sport = false;
            #endif
          }
          mdrive_last_status_can = prxBuf[4];
        }
      }
    }
  }

/***********************************************************************************************************************************************************************************************************************************************
  K-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if(!digitalRead(KCAN_INT_PIN)) {                                                                                                  // If INT pin is pulled low, read K-CAN receive buffer.
    KCAN.readMsgBuf(&krxId, &klen, krxBuf);

    if (krxId == 0x19E) {                                                                                                            // Monitor DSC K-CAN status.
      evaluate_dsc_ign_status();
    }

    else if (krxId == 0xAA) {                                                                                                        // Monitor 0xAA (rpm/throttle status).
      if (ignition) {
        RPM = ((uint32_t)krxBuf[5] << 8) | (uint32_t)krxBuf[4];
        evaluate_shiftlight_display();
        #if EXHAUST_FLAP_WITH_M_BUTTON
          if (engine_running) {
            evaluate_exhaust_flap_position();
          }
        #endif
        #if LAUNCH_CONTROL_INDICATOR
          evaluate_lc_display();
        #endif
      }
    }

    #if LAUNCH_CONTROL_INDICATOR
      else if (krxId == 0xA8) {
        evaluate_clutch_status();
      }
    #endif

    #if AUTO_SEAT_HEATING
      else if (krxId == 0x2CA){                                                                                                      // Monitor and update ambient temperature.
        ambient_temperature_can = krxBuf[0];
      } 
      else if (krxId == 0x232) {                                                                                                     // Driver's seat heating status message is only sent with ignition on.
        if (!krxBuf[0]) {                                                                                                            // Check if seat heating is already on.
          //This will be ignored if already on and cycling ignition. Press message will be ignored by IHK anyway.
          if (!sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) {
            send_seat_heating_request();
          }
        }
      }
    #endif
    
    #if SYNC_SHIFTLIGHTS_WITH_REDLINE
      else if (krxId == 0x332) {                                                                                                     // Monitor variable redline broadcast from DME.
        evaluate_shiftlight_sync();
      }
    #endif
    
  }
}

/***********************************************************************************************************************************************************************************************************************************************
  Helper functions
***********************************************************************************************************************************************************************************************************************************************/

void reset_runtime_variables() 
{
  dsc_program_last_status_can = 0xEA;
  dsc_program_status = ignore_shiftlights_off_counter = RPM = 0;
  mdrive_status = power_mode_only = false;
  shiftlights_segments_active = false;
  engine_running = false;
  mdrive_last_status_can = 0xCF; 
  #if EDC_WITH_M_BUTTON 
    edc_last_status_can = 0xF1;
  #endif
  #if EXHAUST_FLAP_WITH_M_BUTTON
    exhaust_flap_sport = false;
    digitalWrite(EXHAUST_FLAP_SOLENOID_PIN, LOW);
    exhaust_flap_open = true;
  #endif
  #if LAUNCH_CONTROL_INDICATOR
    lc_cc_active = clutch_pressed = vehicle_moving = false;
  #endif
  #if SYNC_SHIFTLIGHTS_WITH_REDLINE
    engine_warmed_up = false;
    last_var_rpm_can = 0;
    START_UPSHIFT_WARN_RPM_ = START_UPSHIFT_WARN_RPM;
    MID_UPSHIFT_WARN_RPM_ = MID_UPSHIFT_WARN_RPM;
    MAX_UPSHIFT_WARN_RPM_ = MAX_UPSHIFT_WARN_RPM;
  #endif
  digitalWrite(POWER_LED_PIN, LOW);
  #if FRONT_FOG_INDICATOR
    front_fog_status = false;
    last_light_status = 0;
    digitalWrite(FOG_LED_PIN, LOW);
  #endif
  #if FTM_INDICATOR
    ftm_indicator_status = false;
  #endif
}


#if EDC_WITH_M_BUTTON
void send_edc_button_press()                                                                                                        // Turn the transistor on/off to simulate EDC button press.
{
  digitalWrite(EDC_BUTTON_PIN, HIGH);
  delay(50);
  digitalWrite(EDC_BUTTON_PIN, LOW);
}
#endif

/***********************************************************************************************************************************************************************************************************************************************
  EOF
***********************************************************************************************************************************************************************************************************************************************/
