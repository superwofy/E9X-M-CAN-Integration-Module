void configure_pins()
{
  pinMode(PTCAN_INT_PIN, INPUT);                                                                                                    // Configure pins
  pinMode(KCAN_INT_PIN, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);                                                                                                 
  pinMode(DSC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_LED_PIN, OUTPUT);
  #if FRONT_FOG_INDICATOR
    pinMode(FOG_LED_PIN, OUTPUT);
  #endif
  pinMode(EXHAUST_FLAP_SOLENOID_PIN, OUTPUT);
}


void initialize_can_controllers()
{
  #if DEBUG_MODE
    Serial.begin(115200);
    while(!Serial);                                                                                                                 // 32U4, wait until virtual port initialized
  #endif
  SPI.setClockDivider(SPI_CLOCK_DIV2);                                                                                              // Set SPI to run at 8MHz (16MHz / 2 = 8 MHz) from default 4 
  while (CAN_OK != PTCAN.begin(MCP_STDEXT, CAN_500KBPS, 1) || 
         CAN_OK != KCAN.begin(MCP_STDEXT, CAN_100KBPS, 1)) {                                                                        // Set 1 for 16MHZ or 2 for 8MHZ.
    #if DEBUG_MODE
      Serial.println(F("Error initializing MCP2515s. Re-trying."));
    #endif
    delay(5000);
  }

  PTCAN.init_Mask(0, 0x07FF0000);                                                                                                   // Mask matches: 7FF (all standard 11bit IDs) and all bytes 
  PTCAN.init_Mask(1, 0x07FF0000);                                                                                                   // Mask matches: 7FF (all standard 11bit IDs) and all bytes
  PTCAN.init_Filt(0, 0x01D60000);                                                                                                   // MFL button status.                                           Cycle time 1s, 100ms (pressed)
  #if FRONT_FOG_INDICATOR
    PTCAN.init_Filt(1, 0x021A0000);                                                                                                 // Light status                                                 Cycle time 5s (idle) 
  #endif
  #if FTM_INDICATOR
     PTCAN.init_Filt(2, 0x031D0000);                                                                                                // FTM status broadcast by DSC                                  Cycle time 5s (idle)
  #endif
  #if CONTROL_SHIFTLIGHTS
    PTCAN.init_Filt(3, 0x03320000);                                                                                                 // Variable redline position                                    Cycle time 1s
  #endif
  PTCAN.init_Filt(4, 0x03CA0000);                                                                                                   // CIC MDrive settings 
  #if SERVOTRONIC_SVT70
    PTCAN.init_Filt(5, 0x58E0000);                                                                                                  // Forward SVT CC to KCAN for KOMBI to display                  Cycle time 10
  #endif
  PTCAN.setMode(MCP_NORMAL);
  
  KCAN.init_Mask(0, 0x07FF0000);                                                                                                    // Mask matches: 7FF (standard 11bit ID) and all bytes
  KCAN.init_Mask(1, 0x07FF0000);                                                                                                    // Mask matches: 7FF (standard 11bit ID) and all bytes 
  KCAN.init_Filt(0, 0x019E0000);                                                                                                    // DSC status and ignition                                      Cycle time 200ms (KCAN)
  KCAN.init_Filt(1, 0x00AA0000);                                                                                                    // RPM, throttle pos.                                           Cycle time 100ms (KCAN)
  #if LAUNCH_CONTROL_INDICATOR
    KCAN.init_Filt(2, 0x00A80000);                                                                                                  // Clutch status                                                Cycle time 100ms (KCAN)
    KCAN.init_Filt(3, 0x01B40000);                                                                                                  // Kombi status (speed, handbrake)                              Cycle time 100ms (terminal R on)
  #endif
  #if AUTO_SEAT_HEATING
    KCAN.init_Filt(4, 0x02320000);                                                                                                  // Driver's seat heating status                                 Cycle time 10s (idle), 150ms (change)
    KCAN.init_Filt(5, 0x02CA0000);                                                                                                  // Ambient temperature                                          Cycle time 1s
  #endif 

  KCAN.setMode(MCP_NORMAL);

  #if DEBUG_MODE
    Serial.println(F("MCP2515s initialized successfully."));
  #endif
}


void toggle_ptcan_sleep()
{
  if (!vehicle_awake) {
    PTCAN.setMode(MCP_SLEEP);
    #if DEBUG_MODE
      Serial.println(F("Deactivated PT-CAN MCP2515."));
    #endif
  } else {
    PTCAN.setMode(MCP_NORMAL);
    #if DEBUG_MODE
      Serial.println(F("Re-activated PT-CAN MCP2515."));
    #endif
  }
}


void disable_unused_mcu_peripherals()
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
    if (digitalRead(POWER_BUTTON_PIN)) {                                                                                            // Bypass USB disable by holding POWER when powering module. This pin should be LOW when holding
        power_usb_disable();
        USBCON |= (1 << FRZCLK);
        PLLCSR &= ~(1 << PLLE);
        USBCON &=  ~(1 << USBE); 
    }
  #endif
}
