#include <nrf_to_nrf.h>



/** RF24Mesh_Example_Master.ino by TMRh20
 *
 *
 * This example sketch shows how to manually configure a node via RF24Mesh as a master node, which
 * will receive all data from sensor nodes.
 *
 * The nodes can change physical or logical position in the network, and reconnect through different
 * routing nodes as required. The master node manages the address assignments for the individual nodes
 * in a manner similar to DHCP.
 *
 */

#include "nrf_to_nrf.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>

//Set up our encryption key
uint8_t myKey[16] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6};

/***** Configure the chosen CE,CS pins *****/
nrf_to_nrf radio;
RF52Network network(radio);
RF52Mesh mesh(radio, network);

uint32_t displayTimer = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // some boards need this because of native USB capability
  }

  radio.begin();
  radio.setKey(myKey);           // Set our key and IV
  radio.enableEncryption = true; // Enable encryption
  radio.enableDynamicPayloads(123); //This is important to call so the encryption overhead will not be included in the 32-byte limit
                                    //To overcome the 32-byte limit, edit RF24Network.h and set MAX_FRAME_SIZE to 111

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);
  Serial.println(mesh.getNodeID());
  // Connect to the mesh
  if (!mesh.begin()) {
    // if mesh.begin() returns false for a master node, then radio.begin() returned false.
    Serial.println(F("Radio hardware not responding."));
    while (1) {
      // hold in an infinite loop
    }
  }
}


void loop() {

  // Call mesh.update to keep the network updated
  mesh.update();

  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();


  // Check for incoming data from the sensors
  if (network.available()) {
    RF24NetworkHeader header;
    network.peek(header);

    uint32_t dat = 0;
    switch (header.type) {
      // Display the incoming millis() values from the sensor nodes
      case 'M':
        network.read(header, &dat, sizeof(dat));
        Serial.println(dat);
        break;
      default:
        network.read(header, 0, 0);
        Serial.println(header.type);
        break;
    }
  }

  if (millis() - displayTimer > 5000) {
    displayTimer = millis();
    Serial.println(" ");
    Serial.println(F("********Assigned Addresses********"));
    for (int i = 0; i < mesh.addrListTop; i++) {
      Serial.print("NodeID: ");
      Serial.print(mesh.addrList[i].nodeID);
      Serial.print(" RF24Network Address: 0");
      Serial.println(mesh.addrList[i].address, OCT);
    }
    Serial.println(F("**********************************"));
  }
}
