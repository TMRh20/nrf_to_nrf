#include <nrf_to_nrf.h>

nrf_to_nrf radio;

static uint32_t                   packet;              /**< Packet to transmit. */


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {delay(100);}
  radio.begin();
  delay(8000);
  radio.setChannel(7);
  radio.enableDynamicPayloads();
  radio.startListening();

  Serial.println("Radio NRF24L01 to NRF52840 begin");
  Serial.println(NRF_RADIO->CRCCNF);
}

uint32_t timer = 0;
bool rx = false;
uint8_t pLength = 4;
uint8_t pCounter = 1;
uint8_t state = 0;
uint8_t lastState = 0;
bool txMode = 0;
uint32_t txTimer = 0;


void loop() {

  state = NRF_RADIO->STATE;
  if(state != lastState){
    Serial.print("LState: ");
    Serial.println(state);
    lastState = state;
  }

  if(millis() - txTimer > 1000){
    txTimer = millis();
    radio.stopListening();
    float test = 43.34;
    NRF_RADIO->TXADDRESS = 0x01;
    NRF_RADIO->PACKETPTR = (uint32_t)radio.radioData;

    radio.write(&test,sizeof(test));
    radio.startListening();
    Serial.println("tx");

  }

  if(radio.available()){
    uint8_t length = min(32,radio.getDynamicPayloadSize());
    Serial.println("P Len:");
    Serial.println(length);
    float data[8];
    radio.read(&data,length);
    Serial.print("Rx: ");
    Serial.println(data[0]);
    Serial.print("Pipe: ");
    Serial.println(NRF_RADIO->RXMATCH);
    Serial.println(radio.radioData[0]);
    Serial.println(radio.radioData[1]);
    radio.stopListening();
    delayMicroseconds(25);
    float buffer = 0;
    NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;
    radio.write(&buffer,0);
    radio.startListening();
  }

}

