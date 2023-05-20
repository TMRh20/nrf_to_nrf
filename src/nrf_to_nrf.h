
/**
 * @file nrf_to_nrf.h
 *
 * Class declaration for nrf52840_nrf24l01
 */
#ifndef __nrf52840_nrf24l01_H__
#define __nrf52840_nrf24l01_H__
#include <Arduino.h>

#define NRF52_RADIO_LIBRARY
#define DEFAULT_MAX_PAYLOAD_SIZE 32
#define ACTUAL_MAX_PAYLOAD_SIZE 127
#define ACK_TIMEOUT_1MBPS 500//200 with static payloads
#define ACK_TIMEOUT_2MBPS 300//165 with static payloads
#define ACK_TIMEOUT_1MBPS_OFFSET 300
#define ACK_TIMEOUT_2MBPS_OFFSET 135

//AES CCM ENCRYPTION
//#if defined NRF_CCM
  #define CCM_ENCRYPTION_ENABLED  
//#endif 
#if defined CCM_ENCRYPTION_ENABLED
  #define MAX_PACKET_SIZE ACTUAL_MAX_PAYLOAD_SIZE//Max Payload Size
  #define CCM_KEY_SIZE 16
  #define CCM_IV_SIZE 8
  #define CCM_COUNTER_SIZE 5
  #define CCM_MIC_SIZE 4
  #define CCM_START_SIZE 3
  #define CCM_MODE_LENGTH_EXTENDED 16
#endif

typedef enum {
  /**
   * (0) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  -18 dBm | -6 dBm | -12 dBm
   */
  NRF_PA_MIN = 0,
  /**
   * (1) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  -12 dBm | 0 dBm | -4 dBm
   */
  NRF_PA_LOW,
  /**
   * (2) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  -6 dBm | 3 dBm | 1 dBm
   */
  NRF_PA_HIGH,
  /**
   * (3) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  0 dBm | 7 dBm | 4 dBm
   */
  NRF_PA_MAX,
  /**
   * (4) This should not be used and remains for backward compatibility.
   */
  NRF_PA_ERROR
} nrf_pa_dbm_e;

/**
 * 
 * 
 * How fast data moves through the air. Units are in bits per second (bps).
 * @see
 * - RF24::setDataRate()
 * - RF24::getDataRate()
 * 
 */
typedef enum {
  /** (0) represents 1 Mbps */
  NRF_1MBPS = 0,
  /** (1) represents 2 Mbps */
  NRF_2MBPS,
  /** (2) represents 250 kbps */
  NRF_250KBPS
} nrf_datarate_e;

/**
 * 
 * 
 * The length of a CRC checksum that is used (if any). Cyclical Redundancy
 * Checking (CRC) is commonly used to ensure data integrity.
 * @see
 * - RF24::setCRCLength()
 * - RF24::getCRCLength()
 * - RF24::disableCRC()
 * 
 */
typedef enum {
  /** (0) represents no CRC checksum is used */
  NRF_CRC_DISABLED = 0,
  /** (1) represents CRC 8 bit checksum is used */
  NRF_CRC_8,
  /** (2) represents CRC 16 bit checksum is used */
  NRF_CRC_16
} nrf_crclength_e;

/**
 * 
 * @brief Driver class for nRF52840 2.4GHz Wireless Transceiver
 */

class nrf_to_nrf {

public:

  /**
   * Constructor for nrf_to_nrf
   *
   * @code 
   * nrf_to_nrf radio;
   * @endcode
   */
  nrf_to_nrf();

  /**
   * Data buffer for radio data
   * The radio can handle packets up to 127 bytes if CRC is disabled
   * 125 bytes if using 16-bit CRC
   *
   */
  uint8_t radioData[ACTUAL_MAX_PAYLOAD_SIZE + 2];
  
  /**
   * Call this before operating the radio
   * @code
   *  radio.begin();
   * @endcode
   */
  bool begin();
  uint8_t sample_ed(void);
  bool available();
  bool available(uint8_t *pipe_num);
  void read(void *buf, uint8_t len);
  bool write(void *buf, uint8_t len, bool multicast = false, bool doEncryption = true);
  bool writeFast(const void *buf, uint8_t len, const bool multicast = 0);
  bool startWrite(void *buf, uint8_t len, bool multicast);
  bool writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
  void enableAckPayload();
  void disableAckPayload();
  void enableDynamicAck();
  void startListening(bool resetAddresses = true);
  void stopListening(bool setWritingPipe = true, bool resetAddresses = true);
  uint8_t getDynamicPayloadSize();
  bool isValid();
  bool isChipConnected();
  void setChannel(uint8_t channel);
  uint8_t getChannel();
  bool setDataRate(uint8_t speed);
  void setPALevel(uint8_t level, bool lnaEnable = true);
  uint8_t getPALevel();
  void setAutoAck(bool enable);
  void setAutoAck(uint8_t pipe, bool enable);
  void enableDynamicPayloads(uint8_t payloadSize = DEFAULT_MAX_PAYLOAD_SIZE);
  void disableDynamicPayloads();
  void setPayloadSize(uint8_t size);
  uint8_t getPayloadSize();
  void setCRCLength(nrf_crclength_e length);
  nrf_crclength_e getCRCLength();
  void disableCRC();
  void setRetries(uint8_t retryVar, uint8_t attempts);
  void openReadingPipe(uint8_t child, uint64_t address);
  void openWritingPipe(uint64_t address);
  void openReadingPipe(uint8_t child, const uint8_t *address);
  void openWritingPipe(const uint8_t *address);
  void setAddressWidth(uint8_t a_width);
  void printDetails();
  
  /**
   * Same as NRF24 radio.available();
   */
  bool available();
  
  /**
   * Same as NRF24 radio.available();
   */
  bool available(uint8_t *pipe_num);
  
  /**
   * Same as NRF24 radio.read();
   */
  void read(void *buf, uint8_t len);
  
  /**
   * Same as NRF24 radio.write();
   */   
  bool write(void *buf, uint8_t len, bool multicast = false);
  
  /**
   * Not currently fully functional, calls the regular write();
   */  
  bool writeFast(const void *buf, uint8_t len, const bool multicast = 0);
  
  /**
   * Not currently functional, use the regular write();
   */
  bool startWrite(void *buf, uint8_t len, bool multicast);
  
  /**
   * Same as NRF24
   */ 
  bool writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
  
   /**
   * Same as NRF24
   */
  void enableAckPayload();
  
  /**
   * Same as NRF24
   */
  void disableAckPayload();
  
   /**
   * Same as NRF24
   */
  void enableDynamicAck();
  
  /**
   * Same as NRF24
   * @param resetAddresses Used internally to reset addresses
   */
  void startListening(bool resetAddresses = true);
  
  /**
   * Same as NRF24
   * @param setWritingPipe Same as RF24
   * @param resetAddresses Used internally to reset addresses
   */
  void stopListening(bool setWritingPipe = true, bool resetAddresses = true);
  
  /**
   * Same as NRF24
   */
  uint8_t getDynamicPayloadSize();

  /**
   * Same as NRF24
   */
  bool isValid();

  /**
   * Same as NRF24
   */  
  bool isChipConnected();
  
  /**
   * Same as NRF24
   */
  void setChannel(uint8_t channel);
  
  /**
   * Same as NRF24
   */
  uint8_t getChannel();

  /**
   * Supported speeds: NRF_1MBPS NRF_2MBPS
   */
  bool setDataRate(uint8_t speed);
  
  /**
   * Same as NRF24
   */
  void setPALevel(uint8_t level, bool lnaEnable = true);

  /**
   * Same as NRF24
   */
  uint8_t getPALevel();

  /**
   * Same as NRF24
   */
  void setAutoAck(bool enable);

  /**
   * @note If using static payloads, acks cannot be enabled & disabled on separate pipes, either ACKs are enabled or not
   */
  void setAutoAck(uint8_t pipe, bool enable);
  
  /**
   * @param payloadSize Set the maximum payload size. Up to 127 with CRC disabled or 125 with 16-bit CRC
   */
  void enableDynamicPayloads(uint8_t payloadSize = DEFAULT_MAX_PAYLOAD_SIZE);

  /**
   * Same as NRF24
   */
  void disableDynamicPayloads();

  /**
   * NRF52840 will handle up to 127 byte payloads with CRC disabled, 125 with 16-bit CRC
   */
  void setPayloadSize(uint8_t size);
  
  /**
   * Same as NRF24
   */
  uint8_t getPayloadSize();
  
  /**
   * Same as NRF24
   */  
  void setCRCLength(nrf_crclength_e length);

  /**
   * Same as NRF24
   */
  nrf_crclength_e getCRCLength();

  /**
   * Same as NRF24
   */
  void disableCRC();

  /**
   * Same as NRF24
   */
  void setRetries(uint8_t retryVar, uint8_t attempts);

  /**
   * Same as NRF24
   */
  void openReadingPipe(uint8_t child, uint64_t address);

  /**
   * Same as NRF24
   */
  void openWritingPipe(uint64_t address);

  /**
   * Same as NRF24
   */
  void openReadingPipe(uint8_t child, const uint8_t *address);

  /**
   * Same as NRF24
   */
  void openWritingPipe(const uint8_t *address);

  /**
   * Same as NRF24
   */
  void setAddressWidth(uint8_t a_width);

  /**
   * Same as NRF24
   */
  void printDetails();

  /**
   * Same as NRF24
   */  
  void powerUp();
  
  /**
   * Same as NRF24
   */
  void powerDown();

  /**
   * Not implemented due to SOC
   */
  bool failureDetected;

  /**
   * Not functional
   */
  bool txStandBy();
  
  /**
   * Not functional
   */
  bool txStandBy(uint32_t timeout, bool startTx = 0);

  /**
   * Similar to NRF24, but can be modified to look for signals better than specified RSSI value
   * @param RSSI The function will return 1 if a value better than -65dBm by default
   */
  bool testCarrier(uint8_t RSSI = 65);
  
  /**
   * Similar to NRF24, but can be modified to look for signals better than specified RSSI value
   * @param RSSI The function will return 1 if a value better than -65dBm by default
   */
  bool testRPD(uint8_t RSSI = 65);
  
  /**
   * Same as NRF24
   */
  uint8_t getARC();
  
  /**
   * Used internally to convert addresses
   */
  uint32_t addrConv32(uint32_t addr);
#if defined CCM_ENCRYPTION_ENABLED

  uint8_t encrypt(void *bufferIn, uint8_t size);
  uint8_t decrypt(void *bufferIn, uint8_t size);
  uint8_t outBuffer[MAX_PACKET_SIZE + CCM_MIC_SIZE + CCM_START_SIZE];
  void setKeyIV(uint8_t key[CCM_KEY_SIZE], uint8_t IV[CCM_IV_SIZE]);
  bool enableEncryption;
#endif

private:
  bool acksEnabled(uint8_t pipe);
  bool acksPerPipe[8];
  uint8_t retries;
  uint8_t retryDuration;
  uint8_t rxBuffer[ACTUAL_MAX_PAYLOAD_SIZE + 1];
  uint8_t ackBuffer[ACTUAL_MAX_PAYLOAD_SIZE + 1];
  uint8_t rxFifoAvailable;
  bool DPL;
  bool ackPayloadsEnabled;
  bool inRxMode;
  uint8_t staticPayloadSize;
  uint8_t ackPID;
  uint8_t ackPipe;
  bool lastTxResult;
  bool overWrite;
  uint32_t rxBase;
  uint32_t rxPrefix;
  uint32_t txBase;
  uint32_t txPrefix;
  bool radioConfigured;
  bool ackPayloadAvailable;
  uint8_t ackAvailablePipeNo;
  uint8_t lastPacketCounter;
  uint16_t lastData;
  bool dynamicAckEnabled;
  uint8_t ARC;
  uint8_t addressWidth;
  uint16_t ackTimeout;
#if defined CCM_ENCRYPTION_ENABLED
  uint8_t inBuffer[MAX_PACKET_SIZE + CCM_MIC_SIZE + CCM_START_SIZE];
  uint8_t scratchPTR[MAX_PACKET_SIZE + CCM_MODE_LENGTH_EXTENDED];
  typedef struct  {
  uint8_t key[CCM_KEY_SIZE];
  uint64_t counter;
  uint8_t direction;
  uint8_t iv[CCM_IV_SIZE];
  } ccmData_t;
  ccmData_t ccmData;
#endif
};

/*! \mainpage nrf_to_nrf - NRF52 radio driver
 *
 * \section Documentation for nrf_to_nrf
 *
 * See https://nrf24.github.io/RF24/ for the main nrf24 docs
 * 
 * These docs are considered supplimental to the NRF24 documentation, mainly documenting the differences between NRF52 and NRF24 drivers
 */

/**
* @example examples/RF24/GettingStarted/GettingStarted.ino
*/

/**
* @example examples/RF24/AcknowledgementPayloads/AcknowledgementPayloads.ino
*/

/**
* @example examples/RF24/scanner/scanner.ino
*/

/**
* @example examples/RF24Network/helloworld_rx/helloworld_rx.ino
*/

/**
* @example examples/RF24Network/helloworld_tx/helloworld_tx.ino
*/

/**
* @example examples/RF24Mesh/RF24Mesh_Example/RF24Mesh_Example.ino
*/

/**
* @example examples/RF24Ethernet/mqtt_basic/mqtt_basic.ino
*/


#endif //__nrf52840_nrf24l01_H__
