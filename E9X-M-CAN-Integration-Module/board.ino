// Functions configuring Teensy and its transceivers go here.


void configure_IO() {
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


void scale_mcu_speed()
{
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


void configure_can_controllers() {
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

  KCAN.setMaxMB(32);                                                                                                                // Increase max filters
  KCAN.setRFFN(RFFN_32);

  KCAN.setFIFOFilter(REJECT_ALL);                                                                                                   // Reject unfiltered messages
  PTCAN.setFIFOFilter(REJECT_ALL);
  DCAN.setFIFOFilter(REJECT_ALL);

  uint16_t filterId;
  uint8_t filterCount = 0;
  cppQueue canFilters(sizeof(filterId), 28, queue_FIFO);

  // KCAN
  #if LAUNCH_CONTROL_INDICATOR
    filterId = 0xA8;                                                                                                                // Clutch status                                                Cycle time 100ms (KCAN)
    canFilters.push(&filterId);
  #endif
  filterId = 0xAA;                                                                                                                  // RPM, throttle pos.                                           Cycle time 100ms (KCAN)
  canFilters.push(&filterId);
  #if DOOR_VOLUME
    filterId = 0xE2;                                                                                                                // Left door status
    canFilters.push(&filterId);
    filterId = 0xEA;                                                                                                                // Right door status
    canFilters.push(&filterId);
  #endif
  filterId = 0x130;                                                                                                                 // Key/ignition status                                          Cycle time 100ms
  canFilters.push(&filterId);
  #if HDC
    filterId = 0x193;                                                                                                               // Kombi cruise control status
    canFilters.push(&filterId);
  #endif
  #if FAKE_MSA
    filterId = 0x195;                                                                                                               // HDC button status sent by IHKA.
    canFilters.push(&filterId);
  #endif
  #if LAUNCH_CONTROL_INDICATOR || HDC || ANTI_THEFT_SEQ
    filterId = 0x1B4;                                                                                                               // Kombi status (speed, handbrake)                              Cycle time 100ms (terminal R ON)
    canFilters.push(&filterId);
  #endif
  #if DIM_DRL
    filterId = 0x1F6;                                                                                                               // Indicator status                                             Cycle time 1s
    canFilters.push(&filterId);
  #endif
  #if FRONT_FOG_LED_INDICATOR || FRONT_FOG_CORNER || DIM_DRL
    filterId = 0x21A;
    canFilters.push(&filterId);                                                                                                     // FRM Light status                                             Cycle time 5s (idle). Sent when changed.
  #endif
  #if AUTO_SEAT_HEATING
    #if AUTO_SEAT_HEATING_PASS
      filterId = 0x22A;                                                                                                             // Passenger's seat heating status
      canFilters.push(&filterId);
    #endif
    filterId = 0x232;                                                                                                               // Driver's seat heating status                                 Cycle time 10s (idle), 150ms (change)
    canFilters.push(&filterId);
  #endif
  #if AUTO_MIRROR_FOLD || CKM
    filterId = 0x23A;                                                                                                               // Remote button and number sent by CAS                         Sent 3x when changed.
    canFilters.push(&filterId);
  #endif
  #if F_ZBE_WAKE || CKM || DOOR_VOLUME
    filterId = 0x273;                                                                                                               // Filter CIC status and ZBE challenge.                         Sent when CIC is idle or a button is pressed on the ZBE.
    canFilters.push(&filterId);
  #endif
  filterId = 0x2CA;                                                                                                                 // Ambient temperature                                          Cycle time 1s
  canFilters.push(&filterId);                                                                                            
  #if HDC
    filterId = 0x2F7;                                                                                                               // Units from KOMBI                                             Sent 3x on Terminal R. Sent when changed.
    canFilters.push(&filterId);
  #endif
  #if RTC
    filterId = 0x2F8;                                                                                                               // Time from KOMBI                                              Cycle time 15s (idle). Sent when changed.
    canFilters.push(&filterId);
  #endif
  #if AUTO_SEAT_HEATING_PASS
    filterId = 0x2FA;                                                                                                               // Seat occupancy and belt status                               Cycle time 5s
    canFilters.push(&filterId);
  #endif
  #if HDC
    filterId = 0x31A;                                                                                                               // HDC button status sent by IHKA.
    canFilters.push(&filterId);
  #endif
  #if RTC
    filterId = 0x39E;                                                                                                               // Time and date set by the user in CIC.
    canFilters.push(&filterId);
  #endif
  #if CKM
    filterId = 0x3A8;                                                                                                               // Filter M Key POWER setting from iDrive.                      Sent when changed.
    canFilters.push(&filterId);
    filterId = 0x3AB;                                                                                                               // Filter Shiftligths car key memory.
    canFilters.push(&filterId);
  #endif
  #if REVERSE_BEEP || LAUNCH_CONTROL_INDICATOR || FRONT_FOG_CORNER
    filterId = 0x3B0;                                                                                                               // Reverse gear status.                                         Cycle time 1s (idle).
    canFilters.push(&filterId);
  #endif
  #if EDC_CKM_FIX
    filterId = 0x3C5;                                                                                                               // Filter M Key EDC setting from iDrive.                        Sent when changed.
    canFilters.push(&filterId);
  #endif
  filterId = 0x3CA;                                                                                                                 // CIC MDrive settings                                          Sent when changed.
  canFilters.push(&filterId);
  #if F_ZBE_WAKE
    filterId = 0x4E2;                                                                                                               // Filter CIC Network management.                               Sent when CIC is ON, cycle time 1.5s.
    canFilters.push(&filterId);
  #endif
  #if DOOR_VOLUME
    filterId = 0x663;                                                                                                               // iDrive diagnostic responses.                                 Sent when response is requested.
    canFilters.push(&filterId);
  #endif
  #if AUTO_MIRROR_FOLD || FRONT_FOG_CORNER 
    filterId = 0x672;                                                                                                               // FRM diagnostic responses.                                    Sent when response is requested.
    canFilters.push(&filterId);
  #endif
  filterCount = canFilters.getCount();
  #if DEBUG_MODE
    sprintf(serial_debug_string, "KCAN filters [%d]:", filterCount);
    serial_log(serial_debug_string);
  #endif
  for (uint8_t i = 0; i < filterCount; i++) {
    canFilters.pop(&filterId);
    #if DEBUG_MODE
      bool setResult;
      setResult = KCAN.setFIFOFilter(i, filterId, STD);
      if (!setResult) {
        sprintf(serial_debug_string, " %X failed", filterId);
        serial_log(serial_debug_string);
      } else {
        sprintf(serial_debug_string, " %X", filterId);
        serial_log(serial_debug_string);
      }
    #else
      KCAN.setFIFOFilter(i, filterId, STD);
    #endif
  }

  // PTCAN
  #if FRONT_FOG_CORNER
    filterId = 0xC8;                                                                                                                // Steering angle.                                              Cycle time 200ms.
    canFilters.push(&filterId);
  #endif
  filterId = 0x1D6;                                                                                                                 // MFL button status.                                           Cycle time 1s (idle), 100ms (pressed)
  canFilters.push(&filterId);
  filterId = 0x315;                                                                                                                 // Vehicle mode (+EDC) from JBE.                                Cycle time 500ms (idle).
  canFilters.push(&filterId);
  #if FTM_INDICATOR
    filterId = 0x31D;                                                                                                               // FTM status broadcast by DSC                                  Cycle time 5s (idle)
    canFilters.push(&filterId);
  #endif
  #if CONTROL_SHIFTLIGHTS
    filterId = 0x332;                                                                                                               // Variable redline position from DME                           Cycle time 1s
    canFilters.push(&filterId); 
  #endif
  #if DEBUG_MODE && CDC2_STATUS_INTERFACE == 2
    filterId = 0x3B4;                                                                                                               // Battery voltage from DME.
    canFilters.push(&filterId);
  #endif
  #if SERVOTRONIC_SVT70
    filterId = 0x58E;                                                                                                               // Forward SVT CC to KCAN for KOMBI to display                  Cycle time 10s
    canFilters.push(&filterId);
    filterId = 0x60E;                                                                                                               // Receive diagnostic messages from SVT module to forward.
    canFilters.push(&filterId);
  #endif
  filterCount = canFilters.getCount();
  #if DEBUG_MODE
    sprintf(serial_debug_string, "PTCAN filters [%d]:", filterCount);
    serial_log(serial_debug_string);
  #endif
  for (uint8_t i = 0; i < filterCount; i++) {
    canFilters.pop(&filterId);
    #if DEBUG_MODE
      bool setResult;
      setResult = PTCAN.setFIFOFilter(i, filterId, STD);
      if (!setResult) {
        sprintf(serial_debug_string, " %X failed", filterId);
        serial_log(serial_debug_string);
      } else {
        sprintf(serial_debug_string, " %X", filterId);
        serial_log(serial_debug_string);
      }
    #else
      PTCAN.setFIFOFilter(i, filterId, STD);
    #endif
  }
  digitalWrite(PTCAN_STBY_PIN, LOW);                                                                                                // PTCAN configured, activate transceiver.

  // DCAN
  filterId = 0x6F1;                                                                                                                 // Receive diagnostic queries from DCAN tool to forward.
  canFilters.push(&filterId);
  filterCount = canFilters.getCount();
  #if DEBUG_MODE
    sprintf(serial_debug_string, "DCAN filters [%d]:", filterCount);
    serial_log(serial_debug_string);
  #endif
  for (uint8_t i = 0; i < filterCount; i++) {
    canFilters.pop(&filterId);
    #if DEBUG_MODE
      bool setResult;
      setResult = DCAN.setFIFOFilter(i, filterId, STD);
      if (!setResult) {
        sprintf(serial_debug_string, " %X failed", filterId);
        serial_log(serial_debug_string);
      } else {
        sprintf(serial_debug_string, " %X", filterId);
        serial_log(serial_debug_string);
      }
    #else
      DCAN.setFIFOFilter(i, filterId, STD);
    #endif
  }
  digitalWrite(DCAN_STBY_PIN, LOW);                                                                                                 // DCAN configured, activate transceiver.
}


void toggle_transceiver_standby() {
  if (!vehicle_awake) {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    serial_log("Deactivated PT-CAN transceiver.");
    serial_log("Opened exhaust flap.");
    #if SERVOTRONIC_SVT70 || RTC
      digitalWrite(DCAN_STBY_PIN, HIGH);
      serial_log("Deactivated D-CAN transceiver.");
    #endif
  } else {
    digitalWrite(PTCAN_STBY_PIN, LOW);
    serial_log("Re-activated PT-CAN transceiver.");
    serial_log("Closed exhaust flap.");
    #if SERVOTRONIC_SVT70 || RTC
      digitalWrite(DCAN_STBY_PIN, LOW);
      serial_log("Re-activated D-CAN transceiver.");
    #endif
  }
}


void check_teensy_cpu_temp() {                                                                                                      // Underclock the processor if it starts to heat up.
  if (ignition) {                                                                                                                   // If ignition is OFF, the processor will already be underclocked.
    float cpu_temp = tempmonGetTemp();

    if (+(cpu_temp - last_cpu_temp) > HYSTERESIS) {
      if (cpu_temp >= MAX_THRESHOLD) {
        if (clock_mode != 3) {
          set_arm_clock(MAX_UNDERCLOCK);
          serial_log("Processor temperature above max overheat threshold. Underclocking.");
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
void update_rtc_from_idrive() {
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
  } else if (k_msg.buf[0] == 0xFE) {                                                                                                // Only date is updated
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


void update_rtc_from_dcan() {
  if (d_msg.buf[1] == 0x10) {                                                                                                       // Only time is updated
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


time_t get_teensy_time() {
  return Teensy3Clock.get();
}


void check_rtc_valid() {
  time_t t = now();
  uint8_t rtc_day = day(t);
  uint8_t rtc_month = month(t);
  uint16_t rtc_year = year(t);
  if (rtc_day == 1 && rtc_month == 1 && rtc_year == 1970) {
    serial_log("Teensy RTC invalid. Check RTC battery.");
    kcan_write_msg(set_time_cc_buf);                                                                                                // Warn that the time needs to be updated by the user.
    rtc_valid = false;
  }
}
#endif


void disable_diag_transmit_jobs() {
  if (diag_transmit) {
    serial_log("Detected OBD port diagnosis request. Pausing all diagnostic jobs.");
    diag_transmit = false;
    #if DOOR_VOLUME
      volume_requested = false;
      volume_changed_to = 0;
      volume_restore_offset = 0;
    #endif
  }
}


void check_diag_transmit_status() {
  if (!diag_transmit) {
    if ((millis() - diag_deactivate_timer) >= 60000) {                                                                              // Re-activate after period of no DCAN requests.
      diag_transmit = true;
      serial_log("Resuming diagnostic jobs after timeout.");
    }
  }
}
