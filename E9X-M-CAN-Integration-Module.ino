// (ZBE), K-CAN section

// 1. Create the two missing messages required to use an F series ZBE (KKCAN) in an E6,7,8,9X car.
// *Should* work with:
// 6582 9267955 - 4-pin MEDIA button
// 6131 9253944 - 4-pin CD button
// 6582 9206444 - 10-pin CD button
// 6582 9212449 - 10-pin CD button
// Tested with 4-pin controller P/N 9267955 Date 19.04.13

// 2. Monitor DSC program status.
// 3. Create missing 0x206 message to animate DCT KOMBI shift lights.
// 4. Send dummy key memory setting MSS6X DME would send for POWER. Used with M_KEY_SETTINGS in CIC.
// 5. Turn on driver's seat heating when ignition is turned on and below configured temperature treshold.

// Using a slightly streamlined version of Cory's library https://github.com/coryjfowler/MCP_CAN_lib
// Credit to Trevor for providing insight into 0x273 and 0x274 http://www.loopybunny.co.uk/CarPC/k_can.html
// Hardware used: CANBED V1.2c http://docs.longan-labs.cc/1030008/ (32U4+MCP2515+MCP2551, LEDs removed) and Generic 16MHz MCP2515 CAN shield.



// (MDrive), PT-CAN section

// 1. Enable use of M3 centre console switch block with actions for POWER and DSC OFF switches. Control POWER LED when MDrive is on.
// 2. Toggles sport mode in the IKM0S MSD81 DME in the absence of a DSCM90 or DSCM80 ZB by re-creating 0x1D9 message.
// 3. Toggle DTC mode through long press of M key.
// 4. Monitor MDrive status as broadcast by 1M DME.

// Credit to Trevor for providing 0x0AA formulas http://www.loopybunny.co.uk/CarPC/can/0AA.html


#include "src/mcp_can.h"
#include <avr/power.h>

/***********************************************************************************************************************************************************************************************************************************************
  Adjustment section. Configure board here
***********************************************************************************************************************************************************************************************************************************************/

MCP_CAN PTCAN(17), KCAN(9);                                                                                                         // CS pins. Adapt to your board
#define PTCAN_INT_PIN 7                                                                                                             // INT pins. Adapt to your board
#define KCAN_INT_PIN 8                                                                              
#define POWER_LED_PIN 4
#define POWER_SWITCH_PIN 5
#define DSC_SWITCH_PIN 6
const int MCP2515_PTCAN = 1;                                                                                                        // Set 1 for 16MHZ or 2 for 8MHZ
const int MCP2515_KCAN = 1;

/***********************************************************************************************************************************************************************************************************************************************
  Adjustment section 2. Configure program functionality here.
***********************************************************************************************************************************************************************************************************************************************/

#pragma GCC optimize ("-Ofast")                                                                                                     // Max compiler optimisation level.
#define DEBUG_MODE 1                                                                                                                // Toggle serial debug messages
#define F_ZBE_WAKE 0                                                                                                                // Enable/disable F CIC ZBE wakeup functions
#define DTC_WITH_M_BUTTON 1                                                                                                         // Toggle DTC mode with M MFL button
#define AUTO_SEAT_HEATING 1                                                                                                         // Enable automatic heated seat for driver in low temperatures
const int AUTO_SEAT_HEATING_TRESHOLD = ((8 + 80) * 2);                                                                              // Degrees Celsius ((temperature + 80) * 2)
const int DTC_SWITCH_TIME = 7;                                                                                                      // Set duration for Enabling/Disabling DTC mode on with long press of M key. 100ms increments.
const int START_UPSHIFT_WARN_RPM = 6000*4;                                                                                          // RPM setpoints (warning = desired RPM * 4).
const int MID_UPSHIFT_WARN_RPM = 6500*4;
const int MAX_UPSHIFT_WARN_RPM = 6900*4;

/***********************************************************************************************************************************************************************************************************************************************
***********************************************************************************************************************************************************************************************************************************************/

unsigned long int rxId;
unsigned char rxBuf[8], len;

byte f_wakeup[] = {0, 0, 0, 0, 0x57, 0x2F, 0, 0x60};                                                                                // Network management kombi, F-series
byte zbe_response[] = {0xE1, 0x9D, 0, 0xFF};

byte mbutton_idle[] = {0xFF, 0x3F, 0}, mbutton_pressed[] = {0xBF, 0x7F, 0};
int mbutton_checksum = 0xF0, mbutton_hold_counter = 0;

byte dtc_key_pressed[] = {0xFD, 0xFF}, dtc_key_released[] = {0xFC, 0xFF};

byte shiftlights_start[2] = {0x86, 0x3E};
byte shiftlights_mid_buildup[2] = {0xF6, 0};
byte shiftlights_max_flash[2] = {0x0A, 0};
byte shiftlights_off[2] = {0x05, 0};
bool enable_shiftlights = true;
bool shiftlights_segments_active = false;

byte dme_ckm[] = {0xF2, 0xFF};

int mdrive_status = 0;                                                                                                              // 0 = off, 1 = on
int dsc_status = 0;                                                                                                                 // 0 = on, 1 = DTC, 2 = DSC OFF
int last_dsc_state = 0;
int last_mdrive_state = 0;
unsigned long power_switch_debounce, dsc_switch_debounce;
int debounce_time = 200;

int ambient_temperature_can = 256;
bool sent_seat_heating_request = false;
byte seat_heating_button_press[] = {0xFD, 0xFF}, seat_heating_button_release[] = {0xFC, 0xFF};

#if DEBUG_MODE
  char serial_debug_string[128];
#endif


void setup() 
{
  disable_unused_peripherals();
  SPI.setClockDivider(SPI_CLOCK_DIV2);         																						                                          // Set SPI to run at 8MHz (16MHz / 2 = 8 MHz) from default 4 

  #if DEBUG_MODE
    Serial.begin(115200);
    while(!Serial);                                                                                                                 // 32U4, wait until virtual port initialized
  #endif
  
  while (CAN_OK != PTCAN.begin(MCP_STDEXT, CAN_500KBPS, MCP2515_PTCAN) || 
         CAN_OK != KCAN.begin(MCP_STDEXT, CAN_100KBPS, MCP2515_KCAN)) {
    #if DEBUG_MODE
      Serial.println("Error initializing MCP2515s. Re-trying.");
    #endif
    delay(5000);
  }

  #if DEBUG_MODE
    Serial.println("MCP2515s initialized successfully.");
  #endif 

  PTCAN.init_Mask(0, 0, 0x07FF0000);                                                                                                // Mask matches: 07FF (standard ID) and all bytes
  PTCAN.init_Mask(1, 0, 0x07FF0000);                                                                                                // Mask matches: 07FF (standard ID) and all bytes
  PTCAN.init_Filt(0, 0, 0x01D60000);                                                                                                // Filter MFL button status.
  PTCAN.init_Filt(1, 0, 0x03990000);                                                                                                // Filter MDrive status.
  PTCAN.init_Filt(2, 0, 0x00AA0000);                                                                                                // Filter RPM, throttle pos.
  PTCAN.setMode(MCP_NORMAL);
  
  KCAN.init_Mask(0, 0, 0x07FF0000);                                                                                                 // Mask matches: 07FF (standard ID) and all bytes
  KCAN.init_Mask(1, 0, 0x07FF0000);                                                                                                 // Mask matches: 07FF (standard ID) and all bytes 
  KCAN.init_Filt(0, 0, 0x03AB0000);                                                                                                 // Filter Shiftligths car key memory.
  KCAN.init_Filt(1, 0, 0x019E0000);                                                                                                 // Filter DSC status.
  #if F_ZBE_WAKE
    KCAN.init_Filt(2, 0, 0x04E20000);                                                                                               // Filter CIC Network management (sent when CIC is on)                 
    KCAN.init_Filt(3, 0, 0x02730000);                                                                                               // Filter CIC status.
  #endif
  #if AUTO_SEAT_HEATING
    KCAN.init_Filt(4, 0, 0x02CA0000);                                                                                               // Ambient temperature
    KCAN.init_Filt(5, 0, 0x02320000);                                                                                               // Driver's seat heating status
  #endif
  KCAN.setMode(MCP_NORMAL);
  
  pinMode(PTCAN_INT_PIN, INPUT);                                                                                                    // Configure pins
  pinMode(KCAN_INT_PIN, INPUT);
  pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);                                                                                                 
  pinMode(DSC_SWITCH_PIN, INPUT_PULLUP);
  pinMode(POWER_LED_PIN, OUTPUT);

  power_switch_debounce = dsc_switch_debounce = millis();
}

void loop()
{
  if (!digitalRead(POWER_SWITCH_PIN)) {
    if ((millis() - power_switch_debounce) > debounce_time) {
      #if DEBUG_MODE
        Serial.println("POWER toggle requested from console switch.");
      #endif 
      power_switch_debounce = millis();
      send_mbutton_message(mbutton_pressed);
    }
  }
  
  else if (!digitalRead(DSC_SWITCH_PIN)) {
    if ((millis() - dsc_switch_debounce) > debounce_time) {
      #if DEBUG_MODE
        Serial.println("DSC toggle requested from console switch.");
      #endif
      dsc_switch_debounce = millis();
      if (dsc_status == 0) {
        send_dtc_button_press(false);
      } else if (dsc_status == 1) {
        send_dtc_button_press(false);
      } else {
        send_dtc_button_press(true);
      }
    }
  }
  
  if (!digitalRead(PTCAN_INT_PIN)) {                                                                                                // If INT pin is pulled low, read PT-CAN receive buffer
    PTCAN.readMsgBuf(&rxId, &len, rxBuf);                                                                                           // Read data: rxId = CAN ID, buf = data byte(s)
    if (rxId == 0x1D6) {       
      if (rxBuf[1] == 0x4C) {                                                                                                       // M button is pressed
        send_mbutton_message(mbutton_pressed);
        #if DTC_WITH_M_BUTTON
          mbutton_hold_counter++;
          if (mbutton_hold_counter == DTC_SWITCH_TIME) {
            if (dsc_status < 2) {                                                                                                   // Check to make sure DSC is not off
              send_dtc_button_press(true);
            }
            mbutton_hold_counter = 0;
          }
        #endif
      } else {                                                                                                                      // If MFL button is released or other buttons are pressed then send alive ping.
          send_mbutton_message(mbutton_idle);
          #if DTC_WITH_M_BUTTON
            mbutton_hold_counter = 0;
          #endif
      }
    } else if (rxId == 0x399) {                                                                                                     // Monitor MDrive status on K-CAN and control centre console POWER LED
      if (last_mdrive_state != rxBuf[4]) {
        if (rxBuf[4] == 0xDF) {
          mdrive_status = 1;
          digitalWrite(POWER_LED_PIN, HIGH);
          #if DEBUG_MODE
            Serial.println("PT-CAN: Status MDrive on. Turned on POWER LED");
          #endif
        } else {
          mdrive_status = 0;
          digitalWrite(POWER_LED_PIN, LOW);
          #if DEBUG_MODE
            Serial.println("PT-CAN: Status MDrive off. Turned off POWER LED");
          #endif
        }
        last_mdrive_state = rxBuf[4];
      }
    } else if (enable_shiftlights) {                                                                                                // Monitor 0xAA (throttle status) and calculate shiftlight status
      evaluate_shiftlight_display();
    }
  }
  
  if(!digitalRead(KCAN_INT_PIN)) {                                                                                                  // If INT pin is pulled low, read K-CAN receive buffer
    KCAN.readMsgBuf(&rxId, &len, rxBuf);
    
   if (rxId == 0x3AB) {                                                                                                             // Monitor Shiftligths CKM status and broadcast dummy for missing POWER CKM
      rxBuf[0] == 0xF1 ? enable_shiftlights = false : enable_shiftlights = false;                                                   // Deactivate Shiftlight calculations if key memory says they're off
      #if DEBUG_MODE
        rxBuf[0] == 0xF1 ? Serial.println("CKM: deactivated shiftlights.") : Serial.println("CKM: activated shiftlights.");
      #endif
      KCAN.sendMsgBuf(0x3A9, 2, dme_ckm);
      Serial.println("K-CAN: Sent dummy POWER CKM.");
    }

    #if F_ZBE_WAKE
      else if (rxId == 0x4E2){                                                                                                      // Monitor CIC status and send ZBE wakeup
        KCAN.sendMsgBuf(0x560, 8, f_wakeup);
        #if DEBUG_MODE
          Serial.println("K-CAN: Sent F-ZBE wake-up message");
        #endif
      } else if (rxId == 0x273) {                                                                                                   // Monitor CIC challenge request and respond
        zbe_response[2] = rxBuf[7];
        KCAN.sendMsgBuf(0x277, 4, zbe_response);                                                                                    // Acknowledge must be sent three times
        KCAN.sendMsgBuf(0x277, 4, zbe_response);
        KCAN.sendMsgBuf(0x277, 4, zbe_response);
        #if DEBUG_MODE
          sprintf(serial_debug_string, "K-CAN: Sent ZBE response to CIC with counter: 0x%X\n", rxBuf[7]);
          Serial.print(serial_debug_string);
        #endif
      }
    #endif

    #if AUTO_SEAT_HEATING
      else if (rxId == 0x2CA){                                                                                                      // Monitor and update ambient temperature
        ambient_temperature_can = rxBuf[0];
      } else if (rxId == 0x232) {                                                                                                   // Driver's seat heating status message is only sent with ignition on.
          if (!rxBuf[0]) {                                                                                                          // Check if seat heating is already on.
            //This will be ignored if already on and cycling ignition. Press message will be ignored by IHK anyway.
            if (!sent_seat_heating_request && (ambient_temperature_can <= AUTO_SEAT_HEATING_TRESHOLD)) {
              send_seat_heating_request();
              sent_seat_heating_request = true;
            }
          }
      }
    #endif
    
    else {                                                                                                                          // Monitor 0x19E DSC status on K-CAN
      if (last_dsc_state != rxBuf[1]){
        if (rxBuf[1] == 0xE0 || rxBuf[1] == 0xEA){
          dsc_status = 0;
        } else if (rxBuf[1] == 0xF0) {
          dsc_status = 1;
          #if DEBUG_MODE
            Serial.println("K-CAN: Status DTC on");
          #endif
        } else if (rxBuf[1] == 0xE4) {
          dsc_status = 2;
          #if DEBUG_MODE
            Serial.println("K-CAN: Status DSC off");
          #endif
        } 
        last_dsc_state = rxBuf[1];
      }
    }
  }
}


void send_mbutton_message(byte message[]) 
{
  message[2] = mbutton_checksum;
  byte send_stat = PTCAN.sendMsgBuf(0x1D9, 3, message);

  #if DEBUG_MODE
  if (send_stat != CAN_OK) {
    Serial.println("PT-CAN: Error sending mbutton message.");
  } else {
    //message[0] == 0xFF ? Serial.println("PT-CAN: Sent mbutton idle.") : Serial.println("PT-CAN: Sent mbutton press.");
    message[0] == 0xFF ? Serial.print(".") : Serial.println("PT-CAN: Sent mbutton press.");
  }
  #endif

  mbutton_checksum < 0xFF ? mbutton_checksum++ : mbutton_checksum = 0xF0;                                                           // mbutton_checksum is between F0..FF
}


void evaluate_shiftlight_display()
{
  int32_t RPM = ((int32_t)rxBuf[5] << 8) | (int32_t)rxBuf[4];
  
  if (RPM > START_UPSHIFT_WARN_RPM && RPM < MID_UPSHIFT_WARN_RPM) {                                                                 // First segment                                                              
    if (rxBuf[2] != 3) {                                                                                                            // Send the warning only if the throttle pedal is pressed
      activate_shiftlight_segments(shiftlights_start);
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Displaying first warning at RPM: %d\n", RPM / 4);
        Serial.print(serial_debug_string);
      #endif                     
    } else {
      deactivate_shiftlight_segments();
    }
  } else if (RPM > MID_UPSHIFT_WARN_RPM && RPM < MAX_UPSHIFT_WARN_RPM) {                                                            // Buildup from second segment to reds
    if (rxBuf[2] != 3) {
      activate_shiftlight_segments(shiftlights_mid_buildup);
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Displaying increasing warning at RPM: %d\n", RPM / 4);
        Serial.print(serial_debug_string);
      #endif
    } else {
      deactivate_shiftlight_segments();
    }
  } else if (RPM >= MAX_UPSHIFT_WARN_RPM) {                                                                                         // Flash all segments
    if (rxBuf[2] != 3) {
      activate_shiftlight_segments(shiftlights_max_flash);
      #if DEBUG_MODE
        sprintf(serial_debug_string, "Flash max warning at RPM: %d\n", RPM / 4);
        Serial.print(serial_debug_string);
      #endif
    } else {
      deactivate_shiftlight_segments();
    }
  } else {                                                                                                                          // RPM dropped. Disable lights
    deactivate_shiftlight_segments();
  }
}

void send_seat_heating_request()
{
  delay(10);
  KCAN.sendMsgBuf(0x1E7, 0, 2, seat_heating_button_press);
  delay(20);
  KCAN.sendMsgBuf(0x1E7, 0, 2, seat_heating_button_release);
  delay(20);
  #if DEBUG_MODE
    sprintf(serial_debug_string, "K-CAN: Sent dr seat heating request at ambient %dC, treshold %dC\n", ((ambient_temperature_can - 80) / 2), AUTO_SEAT_HEATING_TRESHOLD);
    Serial.print(serial_debug_string);
  #endif
}

void activate_shiftlight_segments(byte data)
{   
    KCAN.sendMsgBuf(0x206, 0, 2, data);                                                                                             // Transmit via KCAN instead of PTCAN to reduce delay                                                                      
    shiftlights_segments_active = true;
}

void deactivate_shiftlight_segments()
{
  if (shiftlights_segments_active) {
    KCAN.sendMsgBuf(0x206, 0, 2, shiftlights_off);                                                                                
    shiftlights_segments_active = false;
    #if DEBUG_MODE
      Serial.println("K-CAN: Deactivated shiftlights segments");
    #endif 
  }
}


void send_dtc_button_press(bool single_press) 
// Correct timing sequence as per trace is: 
// key press -> delay(100) -> key press -> delay(50) -> key release -> delay(160) -> key release -> delay(160)
// However, that interferes with program timing. A small delay will still be accepted.
{
    if (single_press) {
      PTCAN.sendMsgBuf(0x316, 2, dtc_key_pressed);                                                                                    // Two messages are sent during a quick press of the button (DTC mode).
      delay(5);
      PTCAN.sendMsgBuf(0x316, 2, dtc_key_pressed);
      delay(5);
      PTCAN.sendMsgBuf(0x316, 2, dtc_key_released);                                                                                   // Send one DTC released to indicate end of DTC key press.
      #if DEBUG_MODE                        
        Serial.println("PT-CAN: Sent DTC key press.");
      #endif
    } else {
        for (int i = 0; i < 25; i++) {                                                                                                // 2.5s to send full DSC OFF sequence.
        PTCAN.sendMsgBuf(0x316, 2, dtc_key_pressed);
          delay(100);
        }
        PTCAN.sendMsgBuf(0x316, 2, dtc_key_released);
        Serial.println("PT-CAN: Sent DSC off sequence.");
    }
}


void disable_unused_peripherals()
{
  power_usart0_disable();                                                                                                           // Disable UART
  power_usart1_disable();                                                                       
  power_twi_disable();                                                                                                              // Disable I2C
  power_timer1_disable();                                                                                                           // Disable unused timers. 0 still on.
  power_timer2_disable();
  power_timer3_disable();
  ADCSRA = 0;
  power_adc_disable();                                                                                                              // Disable Analog to Digital converter
}
