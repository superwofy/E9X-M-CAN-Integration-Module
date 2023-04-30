#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> KCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> PTCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> DCAN;

#define DCAN_STBY_PIN 14
#define PTCAN_STBY_PIN 15

CAN_message_t k_msg, pt_msg, d_msg;

void setup() 
{
	KCAN.begin();
	PTCAN.begin();
	DCAN.begin();
	KCAN.setClock(CLK_60MHz);
	PTCAN.setClock(CLK_60MHz);
	DCAN.setClock(CLK_60MHz);
	KCAN.setBaudRate(100000);
	PTCAN.setBaudRate(500000);
	DCAN.setBaudRate(500000);
	KCAN.enableFIFO();
	PTCAN.enableFIFO();
	DCAN.enableFIFO();

  KCAN.setFIFOFilter(ACCEPT_ALL);
  PTCAN.setFIFOFilter(ACCEPT_ALL);
  DCAN.setFIFOFilter(ACCEPT_ALL);

	// KCAN.setFIFOFilter(REJECT_ALL);
	// KCAN.setFIFOFilter(0, 0xAA, STD);

	// DCAN.setFIFOFilter(REJECT_ALL);
	// DCAN.setFIFOFilter(0, 0xAA, STD);

	// PTCAN.setFIFOFilter(REJECT_ALL);
	// PTCAN.setFIFOFilter(0, 0xAA, STD);

	digitalWrite(PTCAN_STBY_PIN, LOW);
	digitalWrite(DCAN_STBY_PIN, LOW);

  while(!Serial);
  Serial.println("Program configured, CAN data should follow.");
}


void loop()
{

	if (KCAN.read(k_msg)) {
		Serial.print(" KCAN: ");
		Serial.print(" ID: "); Serial.print(k_msg.id, HEX);
		Serial.print(" Buffer: ");
    if (k_msg.flags.remote) {
        Serial.print(" Remote ");
    }
		for ( uint8_t i = 0; i < k_msg.len; i++ ) {
		  Serial.print(k_msg.buf[i], HEX); Serial.print(" ");
		}
		Serial.println();
	}

	if (PTCAN.read(pt_msg)) {
		Serial.print(" PTCAN: ");
		Serial.print(" ID: "); Serial.print(pt_msg.id, HEX);
		Serial.print(" Buffer: ");
    if (pt_msg.flags.remote) {
        Serial.print(" Remote ");
    }
		for ( uint8_t i = 0; i < pt_msg.len; i++ ) {
		  Serial.print(pt_msg.buf[i], HEX); Serial.print(" ");
		}
		Serial.println();
	}

	if (DCAN.read(d_msg)) {
		Serial.print(" DCAN: ");
		Serial.print(" ID: "); Serial.print(d_msg.id, HEX);
		Serial.print(" Buffer: ");
    if (d_msg.flags.remote) {
        Serial.print(" Remote ");
    }
		for ( uint8_t i = 0; i < d_msg.len; i++ ) {
		  Serial.print(d_msg.buf[i], HEX); Serial.print(" ");
		}
		Serial.println();
	}
}



CAN_message_t makeMsgBuf(uint16_t txID, uint8_t txLen, uint8_t* txBuf) 
{
  CAN_message_t tx_msg;
  tx_msg.id = txID;
  tx_msg.len = txLen;
  for (uint8_t i = 0; i < txLen; i++) {
      tx_msg.buf[i] = txBuf[i];
  }
  return tx_msg;
}
