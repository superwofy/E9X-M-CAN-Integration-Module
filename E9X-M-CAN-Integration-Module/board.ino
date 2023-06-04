// Functions configuring Teensy, peripherals and its transceivers go here.


void configure_IO(void) {
  #if DEBUG_MODE
    #if !AUTO_MIRROR_FOLD                                                                                                           // This delay causes the mirrors to unfold slower on init.
      while (!Serial && millis() <= 5000);
    #endif
  #endif
  pinMode(PTCAN_STBY_PIN, OUTPUT); 
  digitalWrite(PTCAN_STBY_PIN, HIGH);
  pinMode(DCAN_STBY_PIN, OUTPUT);
  digitalWrite(DCAN_STBY_PIN, HIGH); 
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);                                                                                                 
  pinMode(DSC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_LED_PIN, OUTPUT);
  #if FRONT_FOG_LED_INDICATOR
    pinMode(FOG_LED_PIN, OUTPUT);
  #endif
  
  #if EXHAUST_FLAP_CONTROL
    pinMode(EXHAUST_FLAP_SOLENOID_PIN, OUTPUT);
    actuate_exhaust_solenoid(LOW);                                                                                                  // Keep the solenoid de-energised (flap open)
  #endif
  #if RTC
    setSyncProvider(get_teensy_time);
  #endif
}


void scale_mcu_speed(void) {
  if (ignition) {
    set_arm_clock(cpu_speed_ide);
    #if DEBUG_MODE
      max_loop_timer = 0;
    #endif
  } else {
      set_arm_clock(MAX_UNDERCLOCK);                                                                                                // Reduce core clock.
      #if DEBUG_MODE
        max_loop_timer = 0;
      #endif
  }
}


void configure_can_controllers(void) {
  KCAN.begin();
  PTCAN.begin();
  DCAN.begin();

  KCAN.setClock(CLK_60MHz);                                                                                                         // Increase from the default 24MHz clock. Run before setBaudRate.
  PTCAN.setClock(CLK_60MHz);
  DCAN.setClock(CLK_60MHz);

  KCAN.setBaudRate(100000);                                                                                                         // 100k
  PTCAN.setBaudRate(500000);                                                                                                        // 500k
  DCAN.setBaudRate(500000);                                                                                                         // 500k

  KCAN.enableFIFO();                                                                                                                // Activate FIFO mode.
  PTCAN.enableFIFO();
  DCAN.enableFIFO();

  KCAN.setMaxMB(40);                                                                                                                // Increase max filters
  KCAN.setRFFN(RFFN_40);

  KCAN.setFIFOFilter(REJECT_ALL);                                                                                                   // Reject unfiltered messages
  PTCAN.setFIFOFilter(REJECT_ALL);
  DCAN.setFIFOFilter(REJECT_ALL);

  uint8_t filter_count = 0;

  // KCAN
  #if LAUNCH_CONTROL_INDICATOR
    KCAN.setFIFOFilter(filter_count, 0xA8, STD);                                                                                    // Clutch status                                                Cycle time 100ms (KCAN)
    filter_count++;
  #endif
    KCAN.setFIFOFilter(filter_count, 0xAA, STD);                                                                                    // RPM, throttle pos.                                           Cycle time 100ms (KCAN)
    filter_count++;
    KCAN.setFIFOFilter(filter_count, 0x130, STD);                                                                                   // Key/ignition status                                          Cycle time 100ms
    filter_count++;
  #if HDC
    KCAN.setFIFOFilter(filter_count, 0x193, STD);                                                                                   // Kombi cruise control status
    filter_count++;
  #endif
  #if FAKE_MSA || MSA_RVC
    KCAN.setFIFOFilter(filter_count, 0x195, STD);                                                                                   // MSA button status sent by IHKA.
    filter_count++;
  #endif
  #if CKM
    KCAN.setFIFOFilter(filter_count, 0x1AA, STD);                                                                                   // iDrive ErgoCommander (rear entertainment?)                   Sent at boot time and when cycling Terminal R.
    filter_count++;
  #endif
  #if LAUNCH_CONTROL_INDICATOR || HDC || ANTI_THEFT_SEQ || FRONT_FOG_CORNER || HOOD_OPEN_GONG
    KCAN.setFIFOFilter(filter_count, 0x1B4, STD);                                                                                   // Kombi status (speed, handbrake)                              Cycle time 100ms (terminal R ON)
    filter_count++;
  #endif
  #if DIM_DRL || FRONT_FOG_CORNER
    KCAN.setFIFOFilter(filter_count, 0x1F6, STD);                                                                                   // Indicator status                                             Cycle time 1s. Sent when changed.
    filter_count++;
  #endif
  #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER || DIM_DRL
    KCAN.setFIFOFilter(filter_count, 0x21A, STD);                                                                                   // FRM Light status                                             Cycle time 5s (idle). Sent when changed.
    filter_count++;
  #endif
  #if AUTO_SEAT_HEATING
    #if AUTO_SEAT_HEATING_PASS
      KCAN.setFIFOFilter(filter_count, 0x22A, STD);                                                                                 // Passenger's seat heating status
      filter_count++;
    #endif
    KCAN.setFIFOFilter(filter_count, 0x232, STD);                                                                                   // Driver's seat heating status                                 Cycle time 10s (idle), 150ms (change)
    filter_count++;
  #endif
  #if AUTO_MIRROR_FOLD || CKM
    KCAN.setFIFOFilter(filter_count, 0x23A, STD);                                                                                   // Remote button and number sent by CAS                         Sent 3x when changed.
    filter_count++;
  #endif
  #if F_ZBE_WAKE || CKM || DOOR_VOLUME || REVERSE_BEEP
    KCAN.setFIFOFilter(filter_count, 0x273, STD);                                                                                   // CIC status and ZBE challenge.                                Sent when CIC is idle or a button is pressed on the ZBE.
    filter_count++;
  #endif
  KCAN.setFIFOFilter(filter_count, 0x2CA, STD);                                                                                     // Ambient temperature                                          Cycle time 1s.
  filter_count++;                                                                         
  #if HDC || ANTI_THEFT_SEQ || FRONT_FOG_CORNER
    KCAN.setFIFOFilter(filter_count, 0x2F7, STD);                                                                                   // Units from KOMBI                                             Sent 3x on Terminal R. Sent when changed.
    filter_count++;
  #endif
  #if RTC
    KCAN.setFIFOFilter(filter_count, 0x2F8, STD);                                                                                   // Time from KOMBI                                              Cycle time 15s (idle). Sent when changed.
    filter_count++;
  #endif
  #if AUTO_SEAT_HEATING_PASS
    KCAN.setFIFOFilter(filter_count, 0x2FA, STD);                                                                                   // Seat occupancy and belt status                               Cycle time 5s
    filter_count++;
  #endif
  #if DOOR_VOLUME || AUTO_MIRROR_FOLD || ANTI_THEFT_SEQ || HOOD_OPEN_GONG
    KCAN.setFIFOFilter(filter_count, 0x2FC, STD);                                                                                   // Door, hood status sent by CAS.                               Cycle time 5s. Sent when changed.
    filter_count++;
  #endif
  #if MSA_RVC
    KCAN.setFIFOFilter(filter_count, 0x317, STD);                                                                                   // Monitor PDC button status.                                   Sent when changed.
    filter_count++;
  #endif
  #if HDC
    KCAN.setFIFOFilter(filter_count, 0x31A, STD);                                                                                   // HDC button status sent by IHKA.
    filter_count++;
  #endif
  #if RTC
    KCAN.setFIFOFilter(filter_count, 0x39E, STD);                                                                                   // Time and date set by the user in CIC.
    filter_count++;    
  #endif
  #if CKM
    KCAN.setFIFOFilter(filter_count, 0x3A8, STD);                                                                                   // M Key POWER setting from iDrive.                             Sent when changed.
    filter_count++;
  #endif
  #if MSA_RVC
    KCAN.setFIFOFilter(filter_count, 0x3AF, STD);                                                                                   // PDC bus status.                                              Cycle time 2s (idle). Sent when changed.
    filter_count++;
  #endif
  #if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR || FRONT_FOG_CORNER || MSA_RVC || MSA_RVC
    KCAN.setFIFOFilter(filter_count, 0x3B0, STD);                                                                                   // Reverse gear status.                                         Cycle time 1s (idle).
    filter_count++;
  #endif
  #if EDC_CKM_FIX
    KCAN.setFIFOFilter(filter_count, 0x3C5, STD);                                                                                   // M Key EDC setting from iDrive.                               Sent when changed.
    filter_count++;
  #endif
  KCAN.setFIFOFilter(filter_count, 0x3CA, STD);                                                                                     // CIC MDrive settings                                          Sent when changed.
  filter_count++;
  #if F_ZBE_WAKE
    KCAN.setFIFOFilter(filter_count, 0x4E2, STD);                                                                                   // CIC Network management.                                      Sent when CIC is ON, cycle time 1.5s.
    filter_count++;
  #endif
  #if DOOR_VOLUME
    KCAN.setFIFOFilter(filter_count, 0x5C0, STD);                                                                                   // CAS CC notifications.
    filter_count++;
    KCAN.setFIFOFilter(filter_count, 0x663, STD);                                                                                   // iDrive diagnostic responses.                                 Sent when response is requested.
    filter_count++;
  #endif
  #if AUTO_MIRROR_FOLD || FRONT_FOG_CORNER 
    KCAN.setFIFOFilter(filter_count, 0x672, STD);                                                                                   // FRM diagnostic responses.                                    Sent when response is requested.
  #endif
  filter_count = 0;


  // PTCAN
  #if FRONT_FOG_CORNER
    PTCAN.setFIFOFilter(filter_count, 0xC8, STD);                                                                                   // Steering angle.                                              Cycle time 200ms.
    filter_count++;
  #endif
  PTCAN.setFIFOFilter(filter_count, 0x1D6, STD);                                                                                    // MFL button status.                                           Cycle time 1s (idle), 100ms (pressed)
  filter_count++;
  #if WIPE_AFTER_WASH
    PTCAN.setFIFOFilter(filter_count, 0x2A6, STD);                                                                                  // Wiper stalk status from SZL
    filter_count++;
  #endif
  PTCAN.setFIFOFilter(filter_count, 0x315, STD);                                                                                    // Vehicle mode (+EDC) from JBE.                                Cycle time 500ms (idle).
  filter_count++;
  #if FTM_INDICATOR
    PTCAN.setFIFOFilter(filter_count, 0x31D, STD);                                                                                  // FTM status broadcast by DSC                                  Cycle time 5s (idle)
    filter_count++;
  #endif
  #if CONTROL_SHIFTLIGHTS
    PTCAN.setFIFOFilter(filter_count, 0x332, STD);                                                                                  // Variable redline position from DME                           Cycle time 1s
    filter_count++;
  #endif
  #if DEBUG_MODE
    PTCAN.setFIFOFilter(filter_count, 0x3B4, STD);                                                                                  // Battery voltage from DME.
    filter_count++;
  #endif
  #if SERVOTRONIC_SVT70
    PTCAN.setFIFOFilter(filter_count, 0x58E, STD);                                                                                  // Forward SVT CC to KCAN for KOMBI to display                  Cycle time 10s
    filter_count++;
    PTCAN.setFIFOFilter(filter_count, 0x60E, STD);                                                                                  // Receive diagnostic messages from SVT module to forward.
  #endif

  // DCAN
  DCAN.setFIFOFilter(0, 0x6F1, STD);                                                                                                // Receive diagnostic queries from DCAN tool to forward.
}


void toggle_transceiver_standby(void) {
  if (!vehicle_awake) {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    serial_log("Deactivated PT-CAN transceiver.");
    digitalWrite(DCAN_STBY_PIN, HIGH);
    serial_log("Deactivated D-CAN transceiver.");
  } else {
    digitalWrite(PTCAN_STBY_PIN, LOW);
    serial_log("Re-activated PT-CAN transceiver.");
    digitalWrite(DCAN_STBY_PIN, LOW);
    serial_log("Re-activated D-CAN transceiver.");
  }
}


void check_teensy_cpu_temp(void) {                                                                                                  // Underclock the processor if it starts to heat up.
  if (ignition) {                                                                                                                   // If ignition is OFF, the processor will already be underclocked.
    float cpu_temp = tempmonGetTemp();

    if (+(cpu_temp - last_cpu_temp) > HYSTERESIS) {
      if (cpu_temp >= MAX_THRESHOLD) {
        if (clock_mode != 4) {
          set_arm_clock(MAX_UNDERCLOCK);
          serial_log("Processor temperature above max overheat threshold. Underclocking.");
          clock_mode = 4;
        }
      } else if (cpu_temp >= HIGH_THRESHOLD) {
        if (clock_mode != 3) {
          set_arm_clock(HIGH_UNDERCLOCK);
          serial_log("Processor temperature above high overheat threshold. Underclocking.");
          clock_mode = 3;
        }
      } else if (cpu_temp >= MEDIUM_THRESHOLD) {
        if (clock_mode != 2) {
          set_arm_clock(MEDIUM_UNDERCLOCK);
          serial_log("Processor temperature above medium overheat threshold. Underclocking.");
          clock_mode = 2;
        }
      } else if (cpu_temp >= MILD_THRESHOLD) {
        if (clock_mode != 1) {
          set_arm_clock(MILD_UNDERCLOCK);
          serial_log("Processor temperature above mild overheat threshold. Underclocking.");
          clock_mode = 1;
        }
      } else {
        if (clock_mode != 0) {
          set_arm_clock(cpu_speed_ide);
          clock_mode = 0;
          serial_log("Restored clock speed.");
        }
      }
    }
    last_cpu_temp = cpu_temp;
  }
}


#if RTC
void update_rtc_from_idrive(void) {
  if (k_msg.buf[3] == 0xFE) {                                                                                                       // Only time is updated
    uint8_t idrive_hour = k_msg.buf[0];
    uint8_t idrive_minutes = k_msg.buf[1];
    //uint8_t idrive_seconds = k_msg.buf[2];                                                                                        // Seconds are always 0
    time_t t = now();
    uint8_t rtc_day = day(t);
    uint8_t rtc_month = month(t);
    uint16_t rtc_year = year(t);
    
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received time from iDrive: %s%d:%s%d", 
              idrive_hour > 9 ? "" : "0", idrive_hour, idrive_minutes > 9 ? "" : "0", idrive_minutes);
      serial_log(serial_debug_string);
    #endif
    setTime(idrive_hour, idrive_minutes, 0, rtc_day, rtc_month, rtc_year);
    t = now();
    Teensy3Clock.set(t); 
  } else if (k_msg.buf[0] == 0xFE) {                                                                                                // Only date was updated
    uint8_t idrive_day = k_msg.buf[3];
    uint8_t idrive_month = k_msg.buf[4] >> 4;
    uint16_t idrive_year = k_msg.buf[6] << 8 | k_msg.buf[5];
    time_t t = now();
    uint8_t rtc_hours = hour(t);
    uint8_t rtc_minutes = minute(t);
    uint8_t rtc_seconds = second(t);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received date from iDrive: %s%d/%s%d/%d", 
              idrive_day > 9 ? "" : "0", idrive_day, idrive_month > 9 ? "" : "0", idrive_month, idrive_year);
      serial_log(serial_debug_string);
    #endif
    setTime(rtc_hours, rtc_minutes, rtc_seconds, idrive_day, idrive_month, idrive_year); 
    t = now();
    Teensy3Clock.set(t); 
  }
}


void update_rtc_from_dcan(void) {
  if (d_msg.buf[1] == 0x10) {                                                                                                       // Only time was updated
    uint8_t dcan_hour = d_msg.buf[5];
    uint8_t dcan_minutes = d_msg.buf[6];
    uint8_t dcan_seconds = d_msg.buf[7]; 
    time_t t = now();
    uint8_t rtc_day = day(t);
    uint8_t rtc_month = month(t);
    uint16_t rtc_year = year(t);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received time from DCAN: %s%d:%s%d:%s%d", 
              dcan_hour > 9 ? "" : "0", dcan_hour, dcan_minutes > 9 ? "" : "0", dcan_minutes,
              dcan_seconds > 9 ? "" : "0", dcan_seconds);
      serial_log(serial_debug_string);
    #endif
    setTime(dcan_hour, dcan_minutes, dcan_seconds, rtc_day, rtc_month, rtc_year);
    t = now();
    Teensy3Clock.set(t); 
  } else if (d_msg.buf[1] == 0x21) {                                                                                                // Only date is updated
    uint8_t dcan_day = d_msg.buf[2];
    uint8_t dcan_month = d_msg.buf[3];
    uint16_t dcan_year = d_msg.buf[4] << 8 | d_msg.buf[5];
    time_t t = now();
    uint8_t rtc_hours = hour(t);
    uint8_t rtc_minutes = minute(t);
    uint8_t rtc_seconds = second(t);
    #if DEBUG_MODE
      sprintf(serial_debug_string, "Received date from DCAN: %s%d/%s%d/%d", 
              dcan_day > 9 ? "" : "0", dcan_day, dcan_month > 9 ? "" : "0", dcan_month, dcan_year);
      serial_log(serial_debug_string);
    #endif
    setTime(rtc_hours, rtc_minutes, rtc_seconds, dcan_day, dcan_month, dcan_year); 
    t = now();
    Teensy3Clock.set(t); 
  }
}


time_t get_teensy_time(void) {
  return Teensy3Clock.get();
}


void check_rtc_valid(void) {
  time_t t = now();
  if (1546300800 >= t && t <= 1546301100) {                                                                                         // See startup.c (1546300800). Will only warn for the first 5 min after failure.
    serial_log("Teensy RTC invalid. Check RTC battery.");
    rtc_valid = false;
  }
}
#endif


void disable_diag_transmit_jobs(void) {
  if (diag_transmit) {
    serial_log("Detected OBD port diagnosis request. Pausing all diagnostic jobs.");
    diag_transmit = false;
    #if DOOR_VOLUME
      volume_changed_to = 0;
      volume_restore_offset = 0;
    #endif
    digitalWrite(DCAN_STBY_PIN, HIGH);                                                                                              // Also deactivate the DCAN transceiver as it will not be used.
  }
}


void check_diag_transmit_status(void) {
  if (!diag_transmit) {
    if (diag_deactivate_timer >= 60000) {                                                                                           // Re-activate after period of no DCAN requests.
      diag_transmit = true;
      serial_log("Resuming diagnostic jobs after timeout.");
      digitalWrite(DCAN_STBY_PIN, LOW);
    }
  }
}


void usb_pll_start() {                                                                                                              // From startup.c
	while (1) {
		uint32_t n = CCM_ANALOG_PLL_USB1; // pg 759
		if (n & CCM_ANALOG_PLL_USB1_DIV_SELECT) {
			CCM_ANALOG_PLL_USB1_CLR = 0xC000;			// bypass 24 MHz
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_BYPASS;	// bypass
			CCM_ANALOG_PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_POWER |	// power down
				CCM_ANALOG_PLL_USB1_DIV_SELECT |		// use 480 MHz
				CCM_ANALOG_PLL_USB1_ENABLE |			// disable
				CCM_ANALOG_PLL_USB1_EN_USB_CLKS;		// disable usb
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_ENABLE)) {
			// TODO: should this be done so early, or later??
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_ENABLE;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_POWER)) {
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_POWER;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_LOCK)) {
			continue;
		}
		if (n & CCM_ANALOG_PLL_USB1_BYPASS) {
			CCM_ANALOG_PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_BYPASS;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_EN_USB_CLKS)) {
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_EN_USB_CLKS;
			continue;
		}
		return; // everything is as it should be  :-)
	}
}


#if USB_DISABLE
void activate_usb() {
  if (!(CCM_CCGR6 & CCM_CCGR6_USBOH3(CCM_CCGR_ON))){
    usb_pll_start();
    delay(100);
    usb_init();
  }
}
#endif
