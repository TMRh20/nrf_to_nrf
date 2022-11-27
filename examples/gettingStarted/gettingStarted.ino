#include <nrf_to_nrf.h>

nrf_to_nrf radio;

static uint32_t                   packet;              /**< Packet to transmit. */



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {delay(10);}
  radio.begin();
  delay(5000);
  radio.setChannel(7);
  radio.enableDynamicPayloads();
  radio.startListening();
  Serial.println("Radio NRF24L01 to NRF52840 begin");
}

uint32_t timer = 0;
bool rx = false;

void loop() {

  if(radio.available()){
    
    // Attempting to send Auto-Ack
    //delayMicroseconds(200);
    //memset(&radio.radioData[2],0,32);
    //NRF_RADIO->TASKS_RXEN=0;
    //NRF_RADIO->TASKS_TXEN=1;
    //while(NRF_RADIO->STATE == 11){}

    uint8_t length = radio.getDynamicPayloadSize();
    Serial.println("Packet Length:");
    Serial.println(length);
    float data = 0.0;
    radio.read(&data,length);
    Serial.print("Received: ");
    Serial.println(data);
  }

}
