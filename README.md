# E9X-M-CAN-Integration-Module
 
Unlike my previous modules, this one has access to PTCAN and KCAN. While the code is quite specific for my particular car, some of the CAN message logic may be useful.
Included are also tools to allow full manipulation of the program section in the bin file.


Hardware used: 
* CANBED V1.2c http://docs.longan-labs.cc/1030008/ (32U4+MCP2515+MCP2551, LEDs removed) 
* Generic 16MHz MCP2515 CAN shield
* S8050 NPN transistor
* 1K SMD resistor
* 150ohm SMD resistor
* OSRAM LO M676-Q2S1
* KINGBRIGHT KM2520ZGC-G03
* Micro USB right-angle cable
* Old PDC module PCB and housing



I use it to:

* Control Mdrive (Throttle map)
* Control DTC with M button
* Control EDC with M button
* Display Shiftlights - including startup animation
* Control Centre console buttons and associated LEDs (POWER, DSC OFF)
* Display Front fog lights on
* Enable FXX KCAN1 CIC controllers
* Turn on heated seats below a set temperature


![shiftlights](img/shiftlight.jpg "shiftlights")

![fog]("img/fog/indicatoron.jpg" "fog")

![case](img/case.jpg "case")

