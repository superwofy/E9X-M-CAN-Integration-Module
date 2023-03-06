void configure_IO()
{
  #if DEBUG_MODE
    Serial.begin(115200);
    SerialUSB1.begin(115200);
    //while(!Serial.dtr());                                                                                                           // Wait until serial monitor is attached.
  #endif
  
  pinMode(PTCAN_STBY_PIN, OUTPUT); 
  pinMode(DCAN_STBY_PIN, OUTPUT); 
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);                                                                                                 
  pinMode(DSC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_LED_PIN, OUTPUT);
  #if FRONT_FOG_INDICATOR
    pinMode(FOG_LED_PIN, OUTPUT);
  #endif
  pinMode(EXHAUST_FLAP_SOLENOID_PIN, OUTPUT);
}


void scale_mcu_speed()
{
  if (ignition) {
    set_arm_clock(cpu_speed_ide);
  } else {
      set_arm_clock(24 * 1000000);                                                                                                  // Set core clock to 24MHz.
  }
}


void disable_mcu_peripherals()
{
  #if !DEBUG_MODE && DISABLE_USB
    if (digitalRead(POWER_BUTTON_PIN)) {                                                                                            // Bypass USB disable by holding POWER when powering module (waking up the car). This pin should be LOW when holding.
      USB1_USBCMD = 0;        
    }
  #endif
}


void configure_can_controller()
{
  KCAN.begin();
  PTCAN.begin();
  DCAN.begin();

  KCAN.setClock(CLK_60MHz);                                                                                                         // Increase from the default 24MHz clock. Run before baudrate.
  PTCAN.setClock(CLK_60MHz);
  DCAN.setClock(CLK_60MHz);

  KCAN.setBaudRate(100000);                                                                                                         // 100k
  PTCAN.setBaudRate(500000);                                                                                                        // 500k
  DCAN.setBaudRate(125000);                                                                                                         // 125k

  KCAN.enableFIFO();                                                                                                                // Activate FIFO mode.
  PTCAN.enableFIFO();
  DCAN.enableFIFO();

  KCAN.setFIFOFilter(REJECT_ALL);                                                                                                   // Reject unfiltered messages
  PTCAN.setFIFOFilter(REJECT_ALL);
  DCAN.setFIFOFilter(REJECT_ALL);

  KCAN.setFIFOFilter(0, 0xAA, STD);                                                                                                 // RPM, throttle pos.                                           Cycle time 100ms (KCAN)
  KCAN.setFIFOFilter(1, 0x19E, STD);                                                                                                // DSC status and ignition                                      Cycle time 200ms (KCAN)
  #if LAUNCH_CONTROL_INDICATOR
    KCAN.setFIFOFilter(2, 0xA8, STD);                                                                                               // Clutch status                                                Cycle time 100ms (KCAN)
    KCAN.setFIFOFilter(3, 0x1B4, STD);                                                                                              // Kombi status (speed, handbrake)                              Cycle time 100ms (terminal R on)
  #endif
  #if AUTO_SEAT_HEATING
    KCAN.setFIFOFilter(4, 0x232, STD);                                                                                              // Driver's seat heating status                                 Cycle time 10s (idle), 150ms (change)
    KCAN.setFIFOFilter(5, 0x2CA, STD);                                                                                              // Ambient temperature                                          Cycle time 1s
  #endif
  KCAN.setFIFOFilter(6, 0x3AB, STD);                                                                                                // Filter Shiftligths car key memory.
  KCAN.setFIFOFilter(7, 0x3B4, STD);                                                                                                // Engine ON status from DME and battery voltage, 
  #if F_ZBE_WAKE
    KCAN.setFIFOFilter(8, 0x273, STD);                                                                                              // Filter CIC status.
    KCAN.setFIFOFilter(9, 0x4E2, STD);                                                                                              // Filter CIC Network management (sent when CIC is on)
  #endif

  PTCAN.setFIFOFilter(0, 0x1D6, STD);                                                                                               // MFL button status.                                           Cycle time 1s, 100ms (pressed)
  #if FRONT_FOG_INDICATOR
    PTCAN.setFIFOFilter(1, 0x21A, STD);                                                                                             // Light status                                                 Cycle time 5s (idle) 
  #endif
  #if FTM_INDICATOR
     PTCAN.setFIFOFilter(2, 0x31D, STD);                                                                                            // FTM status broadcast by DSC                                  Cycle time 5s (idle)
  #endif
  #if CONTROL_SHIFTLIGHTS
    PTCAN.setFIFOFilter(3, 0x332, STD);                                                                                             // Variable redline position                                    Cycle time 1s
  #endif
  PTCAN.setFIFOFilter(4, 0x3CA, STD);                                                                                               // CIC MDrive settings 
  #if SERVOTRONIC_SVT70
    PTCAN.setFIFOFilter(5, 0x58E, STD);                                                                                             // Forward SVT CC to KCAN for KOMBI to display                  Cycle time 10
    PTCAN.setFIFOFilter(6, 0x4B0, STD);                                                                                             // Receive Network messages from SVT module to forward.

    DCAN.setFIFOFilter(0, 0x4B0, STD);                                                                                              // Receive Network messages from DCAN tool to forward.
  #endif  

  digitalWrite(PTCAN_STBY_PIN, LOW);                                                                                                // Activate the secondary transceivers.
  digitalWrite(DCAN_STBY_PIN, LOW);

  #if DEBUG_MODE
    PTCAN.mailboxStatus();
    KCAN.mailboxStatus();
    DCAN.mailboxStatus();
  #endif
}


void initialize_timers()
{
  power_button_debounce_timer = dsc_off_button_debounce_timer = mdrive_message_timer = vehicle_awake_timer = millis();
  #if DEBUG_MODE
    debug_print_timer = millis();
    loop_timer = micros();
  #endif
}


void toggle_transceiver_standby()
{
  if (!vehicle_awake) {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    digitalWrite(DCAN_STBY_PIN, HIGH);
    #if DEBUG_MODE
      Serial.println("Deactivated PT-CAN and DCAN transceiver.");
    #endif
  } else {
    digitalWrite(PTCAN_STBY_PIN, LOW);
    digitalWrite(DCAN_STBY_PIN, LOW);
    #if DEBUG_MODE
      Serial.println("Re-activated PT-CAN and DCAN transceiver.");
    #endif
  }
}
