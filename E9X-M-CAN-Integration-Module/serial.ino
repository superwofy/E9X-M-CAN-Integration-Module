// Functions that allow the user to send commands to the program go here.


#if DEBUG_MODE
void serial_interpreter()
{
  String cmd = Serial.readStringUntil('\n');
  if (cmd == "module_reboot") {
    module_reboot();
  } else if (cmd == "open_exhaust_flap") {
    #if EXHAUST_FLAP_CONTROL
      actuate_exhaust_solenoid(LOW);
      Serial.print("  Serial: Opened exhaust flap.");
    #endif
  } else if (cmd == "close_exhaust_flap") {
    #if EXHAUST_FLAP_CONTROL
      actuate_exhaust_solenoid(HIGH);
      Serial.print("  Serial: Closed exhaust flap.");
    #endif
  } else if (cmd == "toggle_mdrive") {
    toggle_mdrive_message_active();
    send_mdrive_message();
    toggle_mdrive_dsc_mode(); 
  } else if (cmd == "reset_eeprom") {
      for (uint8_t i = 0; i < 16; i++) {
        EEPROM.update(i, 0xFF);
      }
    serial_log("  Serial: Reset EEPROM values.");
  } else if (cmd == "reset_mdrive") {
    reset_mdrive_settings();
    Serial.print("  Serial: ");
  } else if (cmd == "sleep_ptcan") {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Deactivated PT-CAN transceiver.");
  } else if (cmd == "sleep_dcan") {
    digitalWrite(DCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Deactivated D-CAN transceiver.");
  } else if (cmd == "wake_ptcan") {
    digitalWrite(PTCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Activated PT-CAN transceiver.");
  } else if (cmd == "wake_dcan") {
    digitalWrite(DCAN_STBY_PIN, HIGH);
    serial_log("  Serial: Activated D-CAN transceiver.");
  }
}


void module_reboot()
{
  serial_log("Serial: Will reboot after watchdog timeout.");
  exit(0);
}
#endif
