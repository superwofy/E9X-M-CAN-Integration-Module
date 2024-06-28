// Functions configuring Teensy, peripherals and its transceivers go here.


void configure_IO(void) {
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  #if !MDSC_ZB                                                                                                 
    pinMode(DSC_BUTTON_PIN, INPUT_PULLUP);
  #endif
  pinMode(POWER_LED_PIN, OUTPUT);
  #if FRONT_FOG_LED_INDICATOR
    pinMode(FOG_LED_PIN, OUTPUT);
  #endif
  #if EXHAUST_FLAP_CONTROL
    pinMode(EXHAUST_FLAP_SOLENOID_PIN, OUTPUT);
    actuate_exhaust_solenoid(LOW);                                                                                                  // Keep the solenoid de-energised (flap open)
  #endif
  #if AUTO_STEERING_HEATER
    pinMode(STEERING_HEATER_SWITCH_PIN, OUTPUT);
    digitalWrite(STEERING_HEATER_SWITCH_PIN, LOW);
  #endif

  uint8_t disable_pins[40] = {1};                                                                                                   // Disable unused pins to save a tiny bit of current.
  #if F_NBTE
    pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
    pinMode(FACEPLATE_EJECT_PIN, INPUT_PULLUP);
    pinMode(FACEPLATE_POWER_MUTE_PIN, INPUT_PULLUP);
    disable_pins[SPI_CS_PIN] = 0;
    disable_pins[MCP2515_INT_PIN] = 0;
    disable_pins[11] = 0;                                                                                                           // MOSI
    disable_pins[12] = 0;                                                                                                           // MISO
    disable_pins[13] = 0;                                                                                                           // SCK
    disable_pins[FACEPLATE_EJECT_PIN] = 0;
    disable_pins[FACEPLATE_POWER_MUTE_PIN] = 0;
    disable_pins[FACEPLATE_UART_PIN] = 0;
  #endif
  disable_pins[0] = 0;                                                                                                              // CAN2
  disable_pins[1] = 0;
  disable_pins[22] = 0;                                                                                                             // CAN1
  disable_pins[23] = 0;
  disable_pins[30] = 0;                                                                                                             // CAN3
  disable_pins[31] = 0;
  disable_pins[POWER_BUTTON_PIN] = 0;                                                                                               // Console buttons
  disable_pins[POWER_LED_PIN] = 0;
  disable_pins[FOG_LED_PIN] = 0;
  #if AUTO_STEERING_HEATER
    disable_pins[STEERING_HEATER_SWITCH_PIN] = 0;
  #endif
  disable_pins[DCAN_STBY_PIN] = 0;
  disable_pins[PTCAN_STBY_PIN] = 0;
  disable_pins[DSC_BUTTON_PIN] = 0;
  #if EXHAUST_FLAP_CONTROL
    disable_pins[EXHAUST_FLAP_SOLENOID_PIN] = 0;
  #endif

  for (uint i = 0; i < 40; i++) {
    if (disable_pins[i]) {
      pinMode(disable_pins[i], INPUT_DISABLE);
    }
  }

  serial_log("I/O pins configured.", 2);
}


void scale_cpu_speed(void) {
  if (ignition) {
    if (F_CPU_ACTUAL != STANDARD_CLOCK) {
      set_arm_clock(STANDARD_CLOCK);
    }
    clock_mode = 0;
    #if DEBUG_MODE
      max_loop_timer = 0;
    #endif
  } else {
    if (F_CPU_ACTUAL != MAX_UNDERCLOCK) {
      set_arm_clock(MAX_UNDERCLOCK);                                                                                                // Reduce core clock.
    }
    clock_mode = 4;
    #if DEBUG_MODE
      max_loop_timer = 0;
    #endif
  }
}


void configure_flexcan(void) {
  uint8_t filter_position_counter = 0, filter_set_ok_counter = 0;
  CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(0);                                  // Increase from the default 24MHz clock to 60MHz.

  // KCAN
  KCAN.begin();
  KCAN.setBaudRate(100000);                                                                                                         // 100k
  KCAN.enableFIFO();                                                                                                                // Activate FIFO mode.
  KCAN.FLEXCAN_EnterFreezeMode();
  KCAN.setFIFOFilter(ACCEPT_ALL);
  KCAN.FLEXCAN_ExitFreezeMode();


  // PTCAN
  PTCAN.begin();
  PTCAN.setBaudRate(500000);                                                                                                        // 500k
  PTCAN.enableFIFO();
  PTCAN.setMaxMB(16);
  PTCAN.setRFFN(RFFN_16);

  PTCAN.FLEXCAN_EnterFreezeMode();

  #if FRONT_FOG_CORNER || F_NIVI || F_NBTE
    filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0xC4, STD);                                               // Steering angle:                                              Cycle time 10ms.
    filter_position_counter++;
  #endif
  #if HDC
    filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0x194, STD);                                              // Cruise stalk position:                                       Cycle time 50ms.
    filter_position_counter++;
  #endif
  filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0x1A0, STD);                                                // Real speed:                                                  Cycle time 20ms.
  filter_position_counter++;
  #if SERVOTRONIC_SVT70
    filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0x58E, STD);                                              // Forward SVT CC to KCAN for KOMBI to display:                 Cycle time 10s.
    filter_position_counter++;
    filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0x60E, STD);                                              // Diagnostic responses from SVT module to forward.
    filter_position_counter++;
  #endif
  #if F_NBTE && CUSTOM_MONITORING_CC
    filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0x612, STD);                                              // Diagnostic responses from DME.
    filter_position_counter++;
  #endif
  #if F_NIVI
    filter_set_ok_counter += PTCAN.setFIFOFilter(filter_position_counter, 0x657, STD);                                              // Diagnostic responses from NVE module to forward.
    filter_position_counter++;
  #endif
  PTCAN.setFIFOFilter(REJECT_ALL, filter_position_counter);
  PTCAN.FLEXCAN_ExitFreezeMode();
  ptcan_mode = 1;

  pinMode(PTCAN_STBY_PIN, OUTPUT);
  if (filter_position_counter != filter_set_ok_counter) {
    sprintf(serial_debug_string, "PTCAN filter initialization failure %d != %d.",
            filter_set_ok_counter, filter_position_counter);
  } else {
    sprintf(serial_debug_string, "PTCAN initialized with %d filter(s).", filter_position_counter);
  }
  serial_log(serial_debug_string, 2);
  filter_position_counter = filter_set_ok_counter = 0;


  // DCAN
  DCAN.begin();
  DCAN.setBaudRate(500000);                                                                                                         // 500k
  DCAN.enableFIFO();

  DCAN.FLEXCAN_EnterFreezeMode();
  filter_set_ok_counter += DCAN.setFIFOFilter(0, 0x6F1, STD);                                                                       // Diagnostic queries from OBD tool to forward.
  DCAN.setFIFOFilter(REJECT_ALL, 1);
  DCAN.FLEXCAN_ExitFreezeMode();
  dcan_mode = 1;
  
  pinMode(DCAN_STBY_PIN, OUTPUT);
  if (1 != filter_set_ok_counter) {
    serial_log("DCAN filter initialization failure.", 2);
  } else {
    serial_log("DCAN filter initialized.", 2);
  }

  serial_log("FlexCAN module ready.", 2);
}


void configure_mcp2515(void) {
  #if F_NBTE
    if (CAN_OK != KCAN2.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ)) {
      serial_log("Error initializing MCP2515 for KCAN2.", 2);
      return;
    } else {
      serial_log("MCP2515 initialized for KCAN2.", 2);
    }
    kcan2_mode = MCP_NORMAL;
    KCAN2.setMode(kcan2_mode);
  #endif
}


void deactivate_optional_transceivers(void) {
  if (ptcan_mode == 1) {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    ptcan_mode = 0;
    serial_log("Deactivated PT-CAN transceiver.", 0);
  }
  if (dcan_mode == 1) {
    digitalWrite(DCAN_STBY_PIN, HIGH);
    dcan_mode = 0;
    serial_log("Deactivated D-CAN transceiver.", 0);
  }
  #if F_NBTE
    if (kcan2_mode == MCP_NORMAL) {
      kcan2_mode = MCP_SLEEP;
      KCAN2.setMode(kcan2_mode);
      serial_log("Deactivated K-CAN2 transceiver.", 0);
      nbt_bus_sleep = true;
      nbt_bus_sleep_ready_timer = 50000;
    }
  #endif
}


void activate_optional_transceivers(void) {
  if (ptcan_mode == 0) {
    digitalWrite(PTCAN_STBY_PIN, LOW);
    ptcan_mode = 1;
    serial_log("Activated PT-CAN transceiver.", 0);
  }

  if (dcan_mode == 0) {
    digitalWrite(DCAN_STBY_PIN, LOW);
    dcan_mode = 1;
    serial_log("Activated D-CAN transceiver.", 0);
  }

  #if F_NBTE
    if (kcan2_mode == MCP_SLEEP) {
      kcan2_mode = MCP_NORMAL;
      KCAN2.setMode(kcan2_mode);
      serial_log("Activated K-CAN2 transceiver.", 0);
    }
  #endif
}


void check_teensy_cpu_temp(void) {
  cpu_temp = tempmonGetTemp();
  if (cpu_temp > max_cpu_temp) {
    max_cpu_temp = cpu_temp;
  }
}


void check_teensy_cpu_clock(void) {                                                                                                 // Underclock the processor if it starts to heat up.
  // If ignition is OFF, the processor will already be underclocked.
  if (+(cpu_temp - last_cpu_temp) > 2.0) {
    if (cpu_temp >= MILD_THRESHOLD) {
      if (clock_mode != 1) {
        set_arm_clock(MILD_UNDERCLOCK);
        serial_log("Processor temperature above mild overheat threshold. Underclocking.", 0);
        clock_mode = 1;
        #if DEBUG_MODE
          max_loop_timer = 0;
        #endif
      }
    } else if (cpu_temp >= MEDIUM_THRESHOLD) {
      if (clock_mode != 2) {
        set_arm_clock(MEDIUM_UNDERCLOCK);
        serial_log("Processor temperature above medium overheat threshold. Underclocking.", 0);
        clock_mode = 2;
        #if DEBUG_MODE
          max_loop_timer = 0;
        #endif
      }
    } else if (cpu_temp >= HIGH_THRESHOLD) {
      if (clock_mode != 3) {
        set_arm_clock(HIGH_UNDERCLOCK);
        serial_log("Processor temperature above high overheat threshold. Underclocking.", 0);
        clock_mode = 3;
        #if DEBUG_MODE
          max_loop_timer = 0;
        #endif
      }
    } else if (cpu_temp >= MAX_THRESHOLD) {
      if (clock_mode != 4) {
        set_arm_clock(MAX_UNDERCLOCK);
        serial_log("Processor temperature above max overheat threshold. Underclocking.", 0);
        clock_mode = 4;
        #if DEBUG_MODE
          max_loop_timer = 0;
        #endif
        #if F_NBTE
          send_cc_message("Teensy CPU above crtitical temperature!", true, 20000);
        #endif
      }
    } else {
      if (clock_mode != 0) {
        set_arm_clock(STANDARD_CLOCK);
        serial_log("Restored standard clock speed.", 0);
        clock_mode = 0;
        #if DEBUG_MODE
          max_loop_timer = 0;
        #endif
      }
    }
  }
  last_cpu_temp = cpu_temp;
}


void disable_diag_transmit_jobs(void) {                                                                                             // Without this, other KWP jobs sent by Ediabas will receive strange reponse codes.
  if (diag_transmit) {
    serial_log("Detected OBD port diagnosis request. Pausing module initiated KWP/UDS jobs.", 0);
    diag_transmit = false;
    #if F_NBTE
      char diag_cc_string[46] = {' '};
      snprintf(diag_cc_string, 46, "OBD tool detected: KWP/UDS jobs OFF for %ds.",
               (int) (OBD_DETECT_TIMEOUT / 1000));
      send_cc_message(diag_cc_string, true, 3000);
    #endif
    #if DOOR_VOLUME
      volume_changed_to = 0;
      volume_restore_offset = 0;
    #endif
  }
}


void check_diag_transmit_status(void) {
  if (!diag_transmit) {
    if (diag_deactivate_timer >= OBD_DETECT_TIMEOUT) {                                                                              // Re-activate after period of no DCAN requests.
      diag_transmit = true;
      serial_log("Resuming diagnostic jobs after timeout.", 0);
    }
  }
}


void activate_usb() {
  if (!(CCM_CCGR6 & CCM_CCGR6_USBOH3(CCM_CCGR_ON))){
    usb_pll_start();
    usb_init();
    serial_log("USB initialized.", 2);
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
