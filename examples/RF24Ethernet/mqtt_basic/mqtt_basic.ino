/*
 *************************************************************************
   RF24Ethernet Arduino library by TMRh20 - 2014-2021

   Automated (mesh) wireless networking and TCP/IP communication stack for RF24 radio modules

   RF24 -> RF24Network -> UIP(TCP/IP) -> RF24Ethernet
                       -> RF24Mesh

        Documentation: http://nRF24.github.io/RF24Ethernet/

 *************************************************************************

 **** EXAMPLE REQUIRES: Arduino MQTT library: https://github.com/256dpi/arduino-mqtt/ ***
   Shown in Arduino Library Manager as 'MQTT' by Joel Gaehwiler

 *************************************************************************
  Basic MQTT example

  This sketch demonstrates the basic capabilities of the library.
  It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic", printing out any messages
    it receives.
  - it assumes the received payloads are strings not binary
  - Continually publishes its nodeID to the outTopic

  It will reconnect to the server if the connection is lost using a blocking
  reconnect function.

*/

#include <SPI.h>
#include <nrf_to_nrf.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <RF24Ethernet.h>
#include <MQTT.h>

nrf_to_nrf radio;
RF52Network network(radio);
RF52Mesh mesh(radio, network);
RF52EthernetClass RF24Ethernet(radio, network, mesh);

IPAddress ip(10, 1, 3, 244);
IPAddress gateway(10, 1, 3, 33);  //Specify the gateway in case different from the server
IPAddress server(10, 1, 3, 33);   //The ip of the MQTT server
char clientID[] = { "arduinoClient   " };

void messageReceived(MQTTClient* client, char topic[], char payload[], int length) {
  Serial.println("incoming: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.println(payload);
}

EthernetClient ethClient;
MQTTClient client;
bool connectFail = false;

void connect() {
  Serial.print("connecting...");
  uint32_t clTimeout = millis();
  while (!client.connect(clientID)) {
    Serial.print(".");
    if (millis() - clTimeout > 5001) {
      Serial.println();
      connectFail = true;
      return;
    }
    uint32_t timer = millis();
    //Instead of delay, keep the RF24 stack updating
    while (millis() - timer < 1000) {
      Ethernet.update();
    }
  }

  Serial.println("\nconnected!");
  client.publish("outTopic", "hello world");
  client.subscribe("inTopic", 2);
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Ethernet.begin(ip, gateway);

  if (mesh.begin()) {
    Serial.println(" OK");
  } else {
    Serial.println(" Failed");
  }

  //Convert the last octet of the IP address to an identifier used
  char str[4];
  int test = ip[3];
  itoa(ip[3], str, 10);
  memcpy(&clientID[13], &str, strlen(str));
  Serial.println(clientID);

  client.begin(server, ethClient);
  client.onMessageAdvanced(messageReceived);
  radio.printDetails();
}

uint32_t mesh_timer = 0;
uint32_t pub_timer = 0;

void loop() {
  Ethernet.update();

  if (millis() - mesh_timer > 10000 || connectFail) {  //Every 30 seconds, test mesh connectivity
    connectFail = false;
    mesh_timer = millis();
    if (!mesh.checkConnection()) {
      if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
        mesh.begin();
      }
    }
    Serial.println();
  }
  //radio.setPALevel(NRF_PA_MAX);
  if (!client.connected()) {
    connect();
  }

  client.loop();

  // Every second, report to the MQTT server the Node ID of this node
  if (client.connected() && millis() - pub_timer > 3000) {
    pub_timer = millis();
    char str[4];
    int test = ip[3];
    itoa(ip[3], str, 10);
    char str1[] = "Node      \r\n";
    memcpy(&str1[5], &str, strlen(str));

    client.publish("outTopic", str1);
  }
}
