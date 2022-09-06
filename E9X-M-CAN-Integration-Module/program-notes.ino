// Centre console section

// Enable use of M3 centre console button block. POWER and DSC OFF buttons. Control POWER LED when MDrive is on.
// 1. Request MDrive when POWER button is pressed.
// 2. Illuminate POWER LED when MDrive is on.
// 3. Request full DSC OFF after holding DSC OFF button. Turn everything back on with a short press.


// PT-CAN section

// 1. Fully replaces the 0x399 MDrive status allowing control of:
// 	  EDC
//    DSC
//    DME throttle map
//    Servotronic via SVT70 module
// 2. Monitor steering wheel buttons and request MDrive when Source/M is pressed.
// 3. Activate throttle map if console button is pressed or MDrive is active.
// 5. Monitor and indicate front foglight status.
// 6. Monitor and indicate FTM status in KOMBI by flashing tyre icon when initializing.


// K-CAN section

// 1. Monitor ignition and DSC program status.
// 2. Receive, save and update MDrive settings from iDrive.
// 3. Create missing 0x206 message to animate DCT KOMBI shift lights.
// 4. Open/close exhaust flap.
// 5. Display Launch control flag
// 6. Create the two missing messages required to use an F series ZBE (KKCAN) in an E6,7,8,9X car.
//    *Should* work with:
//    6582 9267955 - 4-pin MEDIA button. **Tested - controller build date 19.04.13
//    6131 9253944 - 4-pin CD button
//    6582 9206444 - 10-pin CD button
//    6582 9212449 - 10-pin CD button
// 7. Turn on driver's seat heating when ignition is turned on and below configured temperature treshold.
// Credit to Trevor for providing insight into 0x273, 0x277, 0x2CA and 0x0AA http://www.loopybunny.co.uk/CarPC/k_can.html


// MSD81 IKM0S .bin modification

// [PROGRAM]

// 0x83738/9  -> F1 07     CAN message table
// 0x1f3ad4/5 -> F1 07     CAN filters
// Replace 15 03 (0x315 represented in LE) to stop MSD81 from reacting to Vehicle Mode changes (triggered by JBBF through EDC button)
// This also allows state_spt (throttle map) to be controlled independently from the main "MDrive" with the console switch
// Re-calculate checksum at 0x80304 

// [MAP]

// lc_var_spt_swi at 0x4CD1D set to 00. This disables the DME MDrive logic. 0x399, 0x1D9 etc.


// Using a slightly streamlined version of Cory's library https://github.com/coryjfowler/MCP_CAN_lib
// Hardware used: CANBED V1.2c http://docs.longan-labs.cc/1030008/ (32U4+MCP2515+MCP2551, LEDs removed) and Generic 16MHz MCP2515 CAN shield.