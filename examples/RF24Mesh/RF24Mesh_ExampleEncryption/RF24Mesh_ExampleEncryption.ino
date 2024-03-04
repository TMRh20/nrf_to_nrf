/** RF24Mesh_Example.ino by TMRh20

   This example sketch shows how to manually configure a node via RF24Mesh, and send data to the
   master node.
   The nodes will refresh their network address as soon as a single write fails. This allows the
   nodes to change position in relation to each other and the master node.
*/


#include "nrf_to_nrf.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#ifndef __MBED__
#include "Adafruit_TinyUSB.h" //Needed for Serial.print on non-MBED enabled core
#endif

//Set up our encryption key
uint8_t myKey[16] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6};

/**** Configure the nrf24l01 CE and CS pins ****/
nrf_to_nrf radio;

RF52Network network(radio);
RF52Mesh mesh(radio, network);

/*
 * User Configuration: nodeID - A unique identifier for each radio. Allows addressing
 * to change dynamically with physical changes to the mesh.
 *
 * In this example, configuration takes place below, prior to uploading the sketch to the device
 * A unique value from 1-255 must be configured for each node.
 */
#define nodeID 1


uint32_t displayTimer = 0;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    // some boards need this because of native USB capability
  }
delay(5000);
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  radio.begin();
  radio.setKey(myKey);           // Set our key and IV
  radio.enableEncryption = true; // Enable encryption
  radio.enableDynamicPayloads(123); //This is important to call so the encryption overhead will not be included in the 32-byte limit
                                    //To overcome the 32-byte limit, edit RF24Network.h and set MAX_FRAME_SIZE to 111
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  if (!mesh.begin()) {
    if (radio.isChipConnected()) {
      do {
        // mesh.renewAddress() will return MESH_DEFAULT_ADDRESS on failure to connect
        Serial.println(F("Could not connect to network.\nConnecting to the mesh..."));
      } while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
    } else {
      Serial.println(F("Radio hardware not responding."));
      while (1) {
        // hold in an infinite loop
      }
    }
  }
  radio.setPALevel(NRF_PA_MIN);
}



void loop() {

  mesh.update();

  // Send to the master node every second
  if (millis() - displayTimer >= 1000) {
    displayTimer = millis();

    // Send an 'M' type message containing the current millis()
    //if (!mesh.write(&displayTimer, 'M', sizeof(displayTimer))) {
      RF24NetworkHeader header(0,'M');
    if(!network.write(header,&displayTimer,sizeof(displayTimer) ) ){
      // If a write fails, check connectivity to the mesh network
      if (!mesh.checkConnection()) {
        //refresh the network address
        Serial.println("Renewing Address");
        if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
          //If address renewal fails, reconfigure the radio and restart the mesh
          //This allows recovery from most if not all radio errors
          mesh.begin();
        }
      } else {
        Serial.println("Send fail, Test OK");
      }
    } else {
      Serial.print("Send OK: ");
      Serial.println(displayTimer);
    }
  }

  while (network.available()) {
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
  }
}
