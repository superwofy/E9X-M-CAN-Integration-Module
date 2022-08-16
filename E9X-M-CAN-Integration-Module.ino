// Centre console section

// Enable use of M3 centre console switch block.POWER and DSC OFF switches. Control POWER LED when MDrive is on.
// 1. Request MDrive when POWER button is pressed.
// 2. Illuminate POWER LED when MDrive is on.
// 3. Request full DSC OFF after holding DSC OFF button. Turn everything back on with a short press.


// PT-CAN section

// 1. Toggles sport mode in the IKM0S MSD81 DME in the absence of a DSCM90 or DSCM80 ZB by re-creating 0x1D9 message.
//    Toggle DTC mode. 
//    Toggle EDC MSport program.
// 2. Monitor steering wheel switches and request MDrive when Source/M is pressed.
// 3. Monitor MDrive status as broadcast by 1M DME.
// 4. Monitor EDC status.
// 5. Monitor and indicate front foglight status.
// 6. Monitor and indicate FTM status in KOMBI by flashing tyre icon when initializing.


// K-CAN section

// 1. Monitor ignition and DSC program status.
// 2. Create missing 0x206 message to animate DCT KOMBI shift lights.
// 3. Create the two missing messages required to use an F series ZBE (KKCAN) in an E6,7,8,9X car.
//    *Should* work with:
//    6582 9267955 - 4-pin MEDIA button. **Tested - controller build date 19.04.13
//    6131 9253944 - 4-pin CD button
//    6582 9206444 - 10-pin CD button
//    6582 9212449 - 10-pin CD button
// 4. Turn on driver's seat heating when ignition is turned on and below configured temperature treshold.
// Credit to Trevor for providing insight into 0x273, 0x277, 0x2CA and 0x0AA http://www.loopybunny.co.uk/CarPC/k_can.html


// Using a slightly streamlined version of Cory's library https://github.com/coryjfowler/MCP_CAN_lib
// Hardware used: CANBED V1.2c http://docs.longan-labs.cc/1030008/ (32U4+MCP2515+MCP2551, LEDs removed) and Generic 16MHz MCP2515 CAN shield.

#include "src/mcp_can.h"
#include <avr/power.h>

/***********************************************************************************************************************************************************************************************************************************************
  Board adjustment section.
***********************************************************************************************************************************************************************************************************************************************/

MCP_CAN PTCAN(17), KCAN(9);                                                                                                         // CS pins. Adapt to your board
#define PTCAN_INT_PIN 7                                                                                                             // INT pins. Adapt to your board
#define KCAN_INT_PIN 8                                                                              
#define POWER_LED_PIN 4
#define POWER_SWITCH_PIN 5
#define DSC_SWITCH_PIN 6
#define FOG_LED_PIN 12
#define EDC_SWITCH_PIN 11
const int MCP2515_PTCAN = 1;                                                                                                        // Set 1 for 16MHZ or 2 for 8MHZ
const int MCP2515_KCAN = 1;

/***********************************************************************************************************************************************************************************************************************************************
  Program adjustment section.
***********************************************************************************************************************************************************************************************************************************************/

#pragma GCC optimize ("-O3")                                                                                                        // Compiler optimisation level. For this file only. Edit platform.txt for all files.
#define DEBUG_MODE 0                                                                                                                // Toggle serial debug messages. Disable in production.
#define DISABLE_USB 0                                                                                                               // In production operation the USB interface is not needed

#define FTM_INDICATOR 1                                                                                                             // Indicate FTM status when using M3 RPA hazards switch.
#define FRONT_FOG_INDICATOR 1                                                                                                       // Turn on an LED when front fogs are on. M3 clusters lack this.
#define F_ZBE_WAKE 0                                                                                                                // Enable/disable F CIC ZBE wakeup functions
#define DTC_WITH_M_BUTTON 1                                                                                                         // Toggle DTC mode with M MFL button
#define EDC_WITH_M_BUTTON 1                                                                                                         // Toggle EDC mode with M MFL button. If SVT is installed, Servotronic curve changes too
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver in low temperatures
const uint8_t AUTO_SEAT_HEATING_TRESHOLD = 10 * 2 + 80;                                                                             // Degrees Celsius temperature * 2 + 80
const uint8_t DTC_SWITCH_TIME = 7;                                                                                                  // Set duration for Enabling/Disabling DTC mode on with long press of M key. 100ms increments.
const uint32_t START_UPSHIFT_WARN_RPM = 5500*4;                                                                                     // RPM setpoints (warning = desired RPM * 4).
const uint32_t MID_UPSHIFT_WARN_RPM = 6000*4;
const uint32_t MAX_UPSHIFT_WARN_RPM = 6500*4;


/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

unsigned long int rxId;
unsigned char rxBuf[8], len;

bool ignition = false;

byte mbutton_released[] = {0xFF, 0x3F, 0}, mbutton_pressed[] = {0xBF, 0x7F, 0};
uint8_t mbutton_checksum = 0xF0;
unsigned long mbutton_released_timer;

byte dtc_key_pressed[] = {0xFD, 0xFF}, dtc_key_released[] = {0xFC, 0xFF};
byte dsc_off_fake_cc_status[] = {0x40, 0x24, 0, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF};

byte shiftlights_start[] = {0x86, 0x3E};
byte shiftlights_mid_buildup[] = {0xF6, 0};
byte shiftlights_startup_buildup[] = {0x56, 0};																						                                          // faster sequential buildup. first byte 0-F (F slowest)
byte shiftlights_max_flash[] = {0x0A, 0};
byte shiftlights_off[] = {0x05, 0};
bool shiftlights_segments_active = false;
bool engine_running = false;
uint8_t ignore_shiftlights_off_counter = 0;

bool mdrive_status = false;                                                                                                         // false = off, true = on
bool ignore_full_mdrive = false;
uint8_t mdrive_last_status_can = 0xCF;                                                                                              // OFF by default
uint8_t dsc_program_status = 0;                                                                                                     // 0 = on, 1 = DTC, 2 = DSC OFF
uint8_t dsc_program_last_status_can = 0xEA;
bool holding_dsc_off_console = false;
unsigned long power_switch_debounce_timer, dsc_off_switch_debounce_timer, dsc_off_switch_hold_timer;
const uint16_t power_debounce_time_ms = 300, dsc_debounce_time_ms = 200, dsc_hold_time_ms = 400;

#if FRONT_FOG_INDICATOR
  bool front_fog_status = false;
  uint8_t last_light_status = 0;
#endif
#if FTM_INDICATOR
  bool ftm_indicator_status = false;
  byte ftm_indicator_flash[] = {0x40, 0x50, 0x01, 0x69, 0xFF, 0xFF, 0xFF, 0xFF};
  byte ftm_indicator_off[] = {0x40, 0x50, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
#endif
#if F_ZBE_WAKE
  byte f_wakeup[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                                              // Network management kombi, F-series
  byte zbe_response[] = {0xE1, 0x9D, 0, 0xFF};
  long zbe_wakeup_last_sent;
#endif
#if EDC_WITH_M_BUTTON
  uint8_t edc_status = 1;                                                                                                           // 1 = comfort, 2 = sport, 0xA = msport
  uint8_t edc_last_status_can = 0xF1;
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
  pinMode(PTCAN_INT_PIN, INPUT);                                                                                                    // Configure pins
  pinMode(KCAN_INT_PIN, INPUT);
  pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);                                                                                                 
  pinMode(DSC_SWITCH_PIN, INPUT_PULLUP);
  pinMode(POWER_LED_PIN, OUTPUT);
  #if FRONT_FOG_INDICATOR
    pinMode(FOG_LED_PIN, OUTPUT);
  #endif
  #if EDC_WITH_M_BUTTON
    pinMode(EDC_SWITCH_PIN, OUTPUT);
  #endif
  disable_unused_peripherals();
  SPI.setClockDivider(SPI_CLOCK_DIV2);                                                                                              // Set SPI to run at 8MHz (16MHz / 2 = 8 MHz) from default 4 

  #if DEBUG_MODE
    Serial.begin(115200);
    while(!Serial);                                                                                                                 // 32U4, wait until virtual port initialized
  #endif
  
  while (CAN_OK != PTCAN.begin(MCP_STDEXT, CAN_500KBPS, MCP2515_PTCAN) || 
         CAN_OK != KCAN.begin(MCP_STDEXT, CAN_100KBPS, MCP2515_KCAN)) {
    #if DEBUG_MODE
      Serial.println(F("Error initializing MCP2515s. Re-trying."));
    #endif
    delay(5000);
  }

  #if DEBUG_MODE
    Serial.println(F("MCP2515s initialized successfully."));
  #endif 

  PTCAN.init_Mask(0, 0x07FF0000);                                                                                                   // Mask matches: 07FFFFFF (standard ID) and first two bytes  
  PTCAN.init_Mask(1, 0x07FF0000);                                                                                                   // Mask matches: 07FF (standard ID) and all bytes

  PTCAN.init_Filt(0, 0x01D60000);                                                                                                   // Filter MFL button status.
  PTCAN.init_Filt(1, 0x03150000);                                                                                                   // Get vehicle mode (EDC)
  PTCAN.init_Filt(2, 0x03990000);                                                                                                   // Filter MDrive status.
  #if FRONT_FOG_INDICATOR
    PTCAN.init_Filt(3, 0x021A0000);                                                                                                 // Filter light status
  #endif
  #if FTM_INDICATOR
     PTCAN.init_Filt(4, 0x031D0000);                                                                                                // Filter FTM status broadcast by DSC
  #endif
  PTCAN.setMode(MCP_NORMAL);
  
  KCAN.init_Mask(0, 0x07FF0000);                                                                                                    // Mask matches: 07FF (standard ID) and all bytes
  KCAN.init_Mask(1, 0x07FF0000);                                                                                                    // Mask matches: 07FF (standard ID) and all bytes 
  KCAN.init_Filt(0, 0x00AA0000);                                                                                                    // Filter RPM, throttle pos.
  KCAN.init_Filt(1, 0x019E0000);                                                                                                    // Filter DSC status and ignition
  #if AUTO_SEAT_HEATING
    KCAN.init_Filt(2, 0x02320000);                                                                                                  // Driver's seat heating status
    KCAN.init_Filt(3, 0x02CA0000);                                                                                                  // Ambient temperature
  #endif
  #if F_ZBE_WAKE
    KCAN.init_Filt(4, 0x02730000);                                                                                                  // Filter CIC status.
    KCAN.init_Filt(5, 0x04E20000);                                                                                                  // Filter CIC Network management (sent when CIC is on)
  #endif
  KCAN.setMode(MCP_NORMAL);
  
  #if F_ZBE_WAKE
    power_switch_debounce_timer = dsc_off_switch_debounce_timer = mbutton_released_timer = zbe_wakeup_last_sent = millis();
  #else
     power_switch_debounce_timer = dsc_off_switch_debounce_timer = mbutton_released_timer = millis();
  #endif
}

void loop()
{

/***********************************************************************************************************************************************************************************************************************************************
  Centre console button section.
***********************************************************************************************************************************************************************************************************************************************/

  if (ignition) {
    if (!digitalRead(POWER_SWITCH_PIN)) {
      if ((millis() - power_switch_debounce_timer) > power_debounce_time_ms) {
        #if DEBUG_MODE
          Serial.println("Console: POWER button pressed. Requesting throttle-only MDrive.");
        #endif 
        
        send_mbutton_message(mbutton_pressed);                                                                                      // Emulate key press
        delay(100);
        send_mbutton_message(mbutton_released);
        power_switch_debounce_timer = millis();
        ignore_full_mdrive = true;                                                                                                  // POWER console key should only change throttle mapping.
      }
    } else if (!digitalRead(DSC_SWITCH_PIN)) {
      if (dsc_program_status == 0) {
        if (!holding_dsc_off_console) {
          holding_dsc_off_console = true;
          dsc_off_switch_hold_timer = millis();
        } else {
          if ((millis() - dsc_off_switch_hold_timer) > dsc_hold_time_ms) {                                                          // DSC OFF sequence should only be sent after user holds key for a configured time
            #if DEBUG_MODE
              Serial.println("Console: DSC OFF button held. Sending DSC OFF.");
            #endif
            send_dsc_off_sequence();
            dsc_off_switch_debounce_timer = millis();
          }
        }      
      } else {
        if ((millis() - dsc_off_switch_debounce_timer) > dsc_debounce_time_ms) {                                                    // A quick tap re-enables everything
          #if DEBUG_MODE
            Serial.println("Console: DSC button tapped. Re-enabling DSC normal program.");
          #endif
          dsc_off_switch_debounce_timer = millis();
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
    PTCAN.readMsgBuf(&rxId, &len, rxBuf);                                                                                           // Read data: rxId = CAN ID, buf = data byte(s)

    if (ignition) {
      if (rxId == 0x1D6) {       
        if (ignition) {
          if (rxBuf[1] == 0x4C) {                                                                                                   // M button is pressed
            send_mbutton_message(mbutton_pressed);
          } else if (rxBuf[0] == 0xC0 && rxBuf[1] == 0x0C) {                                                                        // MFL buttons released, send alive ping.
            send_mbutton_message(mbutton_released);                                                                                       
          } else {
            if ((millis() - mbutton_released_timer) > 999) {                                                                        // keep sending MDrive released messages when other buttons are pressed/held
              send_mbutton_message(mbutton_released);
            }
          }
        }
      }
     
      #if FRONT_FOG_INDICATOR
        else if (rxId == 0x21A) {
          if (rxBuf[0] != last_light_status) {
            if ((rxBuf[0] & 32) == 32) {                                                                                            // Check the third bit of the first byte represented in binary for front fog status.
              front_fog_status = true;
              digitalWrite(FOG_LED_PIN, HIGH);
              #if DEBUG_MODE
                Serial.println("Front fogs on. Turned on FOG LED");
              #endif
            } else {
              front_fog_status = false;
              digitalWrite(FOG_LED_PIN, LOW);
              #if DEBUG_MODE
                Serial.println("Front fogs off. Turned off FOG LED");
              #endif
            }
            last_light_status = rxBuf[0];
          }
        }
      #endif

      if (rxId == 0x315) {
        #if EDC_WITH_M_BUTTON
          if (rxBuf[1] != edc_last_status_can) {
            edc_status = rxBuf[1] - 0xF0;
            #if DEBUG_MODE
              switch(edc_status) {
                case 1:
                  Serial.println("EDC now in Comfort mode.");
                  break;
                case 2:
                  Serial.println("EDC now in Sport mode.");
                  break;
                case 0xA:
                  Serial.println("EDC now in MSport mode.");
                  break;
              }
            #endif
            edc_last_status_can = rxBuf[1];
          }
        #endif
      } 

      #if FTM_INDICATOR
        else if (rxId == 0x31D) {                                                                                                   // FTM initialization is ongoing.
          if (rxBuf[0] == 0x03 && !ftm_indicator_status) {
            KCAN.sendMsgBuf(0x5A0, 8, ftm_indicator_flash);
            ftm_indicator_status = true;
            #if DEBUG_MODE
              Serial.println("Activated FTM indicator.");
            #endif
          } else if (rxBuf[0] == 0x00 && ftm_indicator_status) {
            KCAN.sendMsgBuf(0x5A0, 8, ftm_indicator_off);
            ftm_indicator_status = false;
            #if DEBUG_MODE
              Serial.println("Deactivated FTM indicator.");
            #endif
          }
        }
      #endif

      else if (rxId == 0x399) {                                                                                                   // Monitor MDrive status on PT-CAN and control centre console POWER LED
        if (mdrive_last_status_can != rxBuf[4]) {
          if (rxBuf[4] == 0xDF) {
            mdrive_status = true;
            digitalWrite(POWER_LED_PIN, HIGH);
            #if DEBUG_MODE
              Serial.println("Status MDrive on. Turned on POWER LED");
            #endif
            if (!ignore_full_mdrive) {
              #if DTC_WITH_M_BUTTON
                if (dsc_program_status == 0) {                                                                                      // Check to make sure DSC is in normal program before MDrive
                  send_dtc_button_press();
                }
              #endif
              #if EDC_WITH_M_BUTTON
                if (edc_status == 1) {                                                                                              // Make sure EDC is in Comfort mode
                  send_edc_button_press();
                  delay(50);
                  send_edc_button_press();
                  #if DEBUG_MODE
                    Serial.println("Set EDC to MSport from Comfort with MDrive off.");
                  #endif
                } else if (edc_status == 2) {
                  send_edc_button_press();
                  #if DEBUG_MODE
                    Serial.println("Set EDC to MSport from Sport with MDrive off.");
                  #endif
                }
              #endif
            }
          } else {
            mdrive_status = false;
            digitalWrite(POWER_LED_PIN, LOW);
            #if DEBUG_MODE
              Serial.println("Status MDrive off. Turned off POWER LED");
            #endif
            #if DTC_WITH_M_BUTTON
              if (dsc_program_status == 1) {                                                                                        // Turn off DTC together with MDrive
                send_dtc_button_press();
                #if DEBUG_MODE
                  Serial.println("Turned off DTC with MDrive off.");
                #endif
              }
            #endif
            #if EDC_WITH_M_BUTTON
              if (edc_status == 0xA) {                                                                                              // Turn off EDC MSport program together with MDrive (only if toggled with MDrive)
                send_edc_button_press();
                #if DEBUG_MODE
                  Serial.println("Turned off EDC MSport with MDrive off.");
                #endif
              }
            #endif
          }
          mdrive_last_status_can = rxBuf[4];
          ignore_full_mdrive = false;
        }
      }

      #if F_ZBE_WAKE
        else if (rxId == 0x4E2){
            send_zbe_wakeup();
        }
      #endif
    }
  }

/***********************************************************************************************************************************************************************************************************************************************
  K-CAN section.
***********************************************************************************************************************************************************************************************************************************************/
  
  if(!digitalRead(KCAN_INT_PIN)) {                                                                                                  // If INT pin is pulled low, read K-CAN receive buffer
    KCAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == 0x19E) {                                                                                                            // Monitor DSC K-CAN status
      if (dsc_program_last_status_can != rxBuf[1]) {
        if (rxBuf[1] == 0xEA) {
          ignition = false;
          reset_runtime_variables();
          #if DEBUG_MODE
            Serial.println("Ignition OFF. Reset values.");
          #endif
        } else if (rxBuf[1] == 0xEC) {
          ignition = true;
          #if DEBUG_MODE
            Serial.println("Ignition ON.");
          #endif
        } else if (rxBuf[1] == 0xE0) {
          ignition = true;                                                                                                          // Just in case 0xEC was missed
          dsc_program_status = 0;
          #if DEBUG_MODE
              Serial.println("Stability control fully activated");
          #endif
        } else if (rxBuf[1] == 0xF0) {
          dsc_program_status = 1;
          #if DEBUG_MODE
              Serial.println("Stability control in DTC mode");
          #endif
        } else if (rxBuf[1] == 0xE4) {
          dsc_program_status = 2;
          #if DEBUG_MODE
              Serial.println("Stability control fully OFF");
          #endif
        }
        dsc_program_last_status_can = rxBuf[1];
      }
    }

    else if (rxId == 0xAA) {                                                                                                        // Monitor 0xAA (throttle status) and calculate shiftlight status
      if (ignition) {
        evaluate_shiftlight_display();
      }
    }

    #if AUTO_SEAT_HEATING
      else if (rxId == 0x2CA){                                                                                                      // Monitor and update ambient temperature
        ambient_temperature_can = rxBuf[0];
      } 
      else if (rxId == 0x232) {                                                                                                     // Driver's seat heating status message is only sent with ignition on.
        if (!rxBuf[0]) {                                                                                                            // Check if seat heating is already on.
          //This will be ignored if already on and cycling ignition. Press message will be ignored by IHK anyway.
          if (!sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) {
            send_seat_heating_request();
          }
        }
      }
    #endif
    
    #if F_ZBE_WAKE
      else if (rxId == 0x273) {                                                                                                     // Monitor CIC challenge request and respond
        zbe_response[2] = rxBuf[7];
        KCAN.sendMsgBuf(0x277, 4, zbe_response);                                                                                    // Acknowledge must be sent three times
        KCAN.sendMsgBuf(0x277, 4, zbe_response);
        KCAN.sendMsgBuf(0x277, 4, zbe_response);
        #if DEBUG_MODE
          sprintf(serial_debug_string, "Sent ZBE response to CIC with counter: 0x%X\n", rxBuf[7]);
          Serial.print(serial_debug_string);
        #endif
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
  dsc_program_status = ignore_shiftlights_off_counter = 0;
  mdrive_status = ignore_full_mdrive = false;
  shiftlights_segments_active = false;
  engine_running = false;
  mdrive_last_status_can = 0xCF; 
  #if EDC_WITH_M_BUTTON 
    edc_last_status_can = 0xF1;
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


#if F_ZBE_WAKE
void send_zbe_wakeup()
{
  KCAN.sendMsgBuf(0x560, 8, f_wakeup);
  zbe_wakeup_last_sent = millis();
  #if DEBUG_MODE
    Serial.println("Sent F-ZBE wake-up message");
  #endif
}
#endif


void send_mbutton_message(byte message[]) 
{
  message[2] = mbutton_checksum;

  byte send_stat = PTCAN.sendMsgBuf(0x1D9, 3, message);
  #if DEBUG_MODE
    if (send_stat != CAN_OK) {
      Serial.print("Error sending mbutton message. Re-trying. Error: ");
      Serial.println(send_stat);
    } else {
      message[0] == 0xFF ? Serial.println("Sent mbutton released.") : Serial.println("Sent mbutton press.");
    }
  #else
  if (send_stat != CAN_OK) {
    delay(100);                                                                                                                     // Attempt to send again
    PTCAN.sendMsgBuf(0x1D9, 3, message);
  }
  #endif
  mbutton_released_timer = millis();
  mbutton_checksum < 0xFF ? mbutton_checksum++ : mbutton_checksum = 0xF0;                                                           // mbutton_checksum is between F0..FF
}


void evaluate_shiftlight_display()
{
  uint32_t RPM = ((uint32_t)rxBuf[5] << 8) | (uint32_t)rxBuf[4];

  if (!engine_running && (RPM > 2000)) {                                                                                            // Show off shift light segments during engine startup (>500rpm)
    engine_running = true;
    activate_shiftlight_segments(shiftlights_startup_buildup);
    #if DEBUG_MODE
      Serial.println("Showing shift light on engine startup.");
    #endif
    ignore_shiftlights_off_counter = 10;                                                                                            // Skip a few off cycles to allow segments to light up
  }

  if (rxBuf[2] >= 0xFA) {                                                                                                           // Disable lights if throttle is released
    deactivate_shiftlight_segments();
  } else if (RPM >= START_UPSHIFT_WARN_RPM && RPM <= MID_UPSHIFT_WARN_RPM) {                                                        // First yellow segment                                                              
    activate_shiftlight_segments(shiftlights_start);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying first warning at RPM: %ld\n", RPM / 4);
      Serial.print(serial_debug_string);
    #endif                     
  } else if (RPM >= MID_UPSHIFT_WARN_RPM && RPM <= MAX_UPSHIFT_WARN_RPM) {                                                          // Buildup from second yellow segment to reds
    activate_shiftlight_segments(shiftlights_mid_buildup);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Displaying increasing warning at RPM: %ld\n", RPM / 4);
      Serial.print(serial_debug_string);
    #endif
  } else if (RPM >= MAX_UPSHIFT_WARN_RPM) {                                                                                         // Flash all segments
    activate_shiftlight_segments(shiftlights_max_flash);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Flash max warning at RPM: %ld\n", RPM / 4);
      Serial.print(serial_debug_string);
    #endif
  } else {                                                                                                                          // RPM dropped. Disable lights
    deactivate_shiftlight_segments();
  }
}


#if AUTO_SEAT_HEATING
void send_seat_heating_request()
{
  delay(10);
  KCAN.sendMsgBuf(0x1E7, 2, seat_heating_button_pressed);
  delay(20);
  KCAN.sendMsgBuf(0x1E7, 2, seat_heating_button_released);
  delay(20);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "Sent dr seat heating request at ambient %dC, treshold %dC\n", (ambient_temperature_can - 80) / 2, (AUTO_SEAT_HEATING_TRESHOLD - 80) / 2);
    Serial.print(serial_debug_string);
  #endif
  sent_seat_heating_request = true;
}
#endif


void activate_shiftlight_segments(byte* data)
{
    PTCAN.sendMsgBuf(0x206, 2, data);                                                                     
    shiftlights_segments_active = true;
}


void deactivate_shiftlight_segments()
{
  if (shiftlights_segments_active) {
    if (ignore_shiftlights_off_counter == 0) {
      PTCAN.sendMsgBuf(0x206, 2, shiftlights_off);                                                                                
      shiftlights_segments_active = false;
      #if DEBUG_MODE
        Serial.println("Deactivated shiftlights segments");
      #endif 
    } else {
      ignore_shiftlights_off_counter--;
    }
  }
}


void send_dtc_button_press() 
// Correct timing sequence as per trace is: 
// key press -> delay(100) -> key press -> delay(50) -> key release -> delay(160) -> key release -> delay(160)
// However, that interferes with program timing. A small delay will still be accepted.
{
  PTCAN.sendMsgBuf(0x316, 2, dtc_key_pressed);                                                                                      // Two messages are sent during a quick press of the button (DTC mode).
  delay(5);
  PTCAN.sendMsgBuf(0x316, 2, dtc_key_pressed);
  delay(5);
  PTCAN.sendMsgBuf(0x316, 2, dtc_key_released);                                                                                     // Send one DTC released to indicate end of DTC key press.
  #if DEBUG_MODE                        
    Serial.println("Sent single DTC key press.");
  #endif
} 


void send_dsc_off_sequence() 
{
  PTCAN.sendMsgBuf(0x5A9, 8, dsc_off_fake_cc_status);                                                                               // Trigger DSC OFF CC in Kombi, iDrive as soon as sequence starts
  for (int i = 0; i < 26; i++) {                                                                                                    // >2.5s to send full DSC OFF sequence.
    if ((millis() - mbutton_released_timer) > 999) {                                                                                // keep sending MDrive key released message
      send_mbutton_message(mbutton_released);
    }
    PTCAN.sendMsgBuf(0x316, 2, dtc_key_pressed);
    delay(100); 
  }
  PTCAN.sendMsgBuf(0x316, 2, dtc_key_released);
  #if DEBUG_MODE
    Serial.println("Sent DSC OFF sequence.");
  #endif
}

#if EDC_WITH_M_BUTTON
void send_edc_button_press()                                                                                                        // Turn the transistor on/off to simulate EDC switch press.
{
  digitalWrite(EDC_SWITCH_PIN, HIGH);
  delay(50);
  digitalWrite(EDC_SWITCH_PIN, LOW);
}
#endif


void disable_unused_peripherals()
// 32U4 Specific!
{
  power_usart0_disable();                                                                                                           // Disable UART
  power_usart1_disable();                                                                       
  power_twi_disable();                                                                                                              // Disable I2C
  power_timer1_disable();                                                                                                           // Disable unused timers. 0 still on.
  power_timer2_disable();
  power_timer3_disable();
  ADCSRA = 0;
  power_adc_disable();                                                                                                              // Disable Analog to Digital converter
  #if !DEBUG_MODE && DISABLE_USB
    if (digitalRead(POWER_SWITCH_PIN)) {                                                                                            // Bypass USB disable by holding POWER when powering module. This pin should be LOW when holding
        power_usb_disable();
        USBCON |= (1 << FRZCLK);
        PLLCSR &= ~(1 << PLLE);
        USBCON &=  ~(1 << USBE); 
    }
  #endif
}
/***********************************************************************************************************************************************************************************************************************************************
  EOF
***********************************************************************************************************************************************************************************************************************************************/
