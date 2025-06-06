
/**
 * @file nrf_to_nrf.h
 *
 * Class declaration for nrf52840_nrf24l01
 */
#ifndef __nrf52840_nrf24l01_H__
#define __nrf52840_nrf24l01_H__
#include <Arduino.h>
#if defined(USE_TINYUSB)
    // Needed for Serial.print on non-MBED enabled or adafruit-based nRF52 cores
    #include "Adafruit_TinyUSB.h"
#endif

#if defined(NRF52811_XXAA) || defined(NRF52820_XXAA) || defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
    #define NRF_HAS_ENERGY_DETECT
#endif

#define NRF52_RADIO_LIBRARY
#define DEFAULT_MAX_PAYLOAD_SIZE   32
#define ACTUAL_MAX_PAYLOAD_SIZE    127
#define ACK_TIMEOUT_1MBPS          600 // 300 with static payloads
#define ACK_TIMEOUT_2MBPS          400 // 265 with static payloads
#define ACK_TIMEOUT_250KBPS        800 // 500 with staticPayloads
#define ACK_TIMEOUT_1MBPS_OFFSET   300
#define ACK_TIMEOUT_2MBPS_OFFSET   135
#define ACK_TIMEOUT_250KBPS_OFFSET 300
#define ACK_PAYLOAD_TIMEOUT_OFFSET 750

// AES CCM ENCRYPTION
#if defined NRF_CCM || defined(DOXYGEN)
    #define CCM_ENCRYPTION_ENABLED
#endif
#if defined CCM_ENCRYPTION_ENABLED || defined(DOXYGEN)
    #define MAX_PACKET_SIZE          ACTUAL_MAX_PAYLOAD_SIZE // Max Payload Size
    #define CCM_KEY_SIZE             16
    #define CCM_IV_SIZE              5
    #define CCM_IV_SIZE_ACTUAL       8
    #define CCM_COUNTER_SIZE         3
    #define CCM_MIC_SIZE             4
    #define CCM_START_SIZE           3
    #define CCM_MODE_LENGTH_EXTENDED 16
#endif

typedef enum
{
    /**
     * (0) represents -12dBm
     */
    NRF_PA_MIN = 0,
    /**
     * (1) represents 2dBm
     */
    NRF_PA_LOW,
    /**
     * (2) represents 6dBm
     */
    NRF_PA_HIGH,
    /**
     * (3) represents 8dBm
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
typedef enum
{
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
typedef enum
{
    /** (0) represents no CRC checksum is used */
    NRF_CRC_DISABLED = 0,
    /** (1) represents CRC 8 bit checksum is used */
    NRF_CRC_8,
    /** (2) represents CRC 16 bit checksum is used */
    NRF_CRC_16,
    /** (3) represents CRC 24 bit checksum is used */
    NRF_CRC_24
} nrf_crclength_e;

/**
 *
 * @brief Driver class for nRF52840 2.4GHz Wireless Transceiver
 */

class nrf_to_nrf
{

public:
    /**
     * @name Primary public interface
     *
     *  These are the main methods you need to operate the chip
     */
    /**@{*/

    /**
     * Constructor for nrf_to_nrf
     *
     * @code
     * nrf_to_nrf radio;
     * @endcode
     */
    nrf_to_nrf();

    /**
     * Call this before operating the radio
     * @code
     *  radio.begin();
     * @endcode
     */
    bool begin();

    /**
     * Same as NRF24 radio.available();
     */
    bool available();

    /**
     * Same as NRF24 radio.available();
     */
    bool available(uint8_t* pipe_num);

    /**
     * Same as NRF24 radio.read();
     */
    void read(void* buf, uint8_t len);

    /**
     * Same as NRF24 radio.write();
     */
    bool write(void* buf, uint8_t len, bool multicast = false, bool doEncryption = true);

    /**
     * Same as NRF24
     * @param resetAddresses Used internally to reset addresses
     */
    void startListening(bool resetAddresses = true);

    /**
     * Same as NRF24
     * @param setWritingPipe Used internally
     * @param resetAddresses Used internally to reset addresses
     */
    void stopListening(bool setWritingPipe = true, bool resetAddresses = true);

    void stopListening(const uint8_t* txAddress, bool setWritingPipe = true, bool resetAddresses = true);

    /**
     * Same as NRF24 except the 52840 has 8 pipes
     */
    void openReadingPipe(uint8_t child, const uint8_t* address);

    /**
     * Same as NRF24
     */
    void openWritingPipe(const uint8_t* address);

    /**
     * Same as NRF24
     */
    bool isChipConnected();

    /**@}*/
    /**
     * @name Advanced Operation
     *
     * Methods you can use to drive the chip in more advanced ways
     */
    /**@{*/

    /**
     * Data buffer for radio data
     * The radio can handle packets up to 127 bytes if CRC & DPL is disabled
     * 125 bytes if using 16-bit CRC & DPL is disabled
     *
     */
    uint8_t radioData[ACTUAL_MAX_PAYLOAD_SIZE + 2];

    /**
     * Not currently fully functional, calls the regular write();
     */
    bool writeFast(void* buf, uint8_t len, bool multicast = 0);

    /**
     * Writes a payload to the radio without waiting for completion.
     *
     * This function will not wait for the radio to finish transmitting or for an ACK
     *
     * Call txStandBy() to take the radio out of TX state or call startListening() to go into RX mode
     *
     */
    bool startWrite(void* buf, uint8_t len, bool multicast, bool doEncryption = true);

    /**
     * Same as NRF24
     */
    bool writeAckPayload(uint8_t pipe, void* buf, uint8_t len);

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
     */
    uint8_t getDynamicPayloadSize();

    /**
     * Same as NRF24
     */
    bool isValid();

    /**
     * Same as NRF24, except we can map the channels lower
     * @param channel Set 0 to 100 for 2400-2500Mhz if map == 0, 2360-2460Mhz if map == 1
     * @param map 1 is low range (2360-2460Mhz), 0 is high range (2400-2500Mhz)
     */
    void setChannel(uint8_t channel, bool map = 0);

    /**
     * Same as NRF24
     */
    uint8_t getChannel();

    /**
     * Supported speeds: NRF_250KBPS NRF_1MBPS NRF_2MBPS
     */
    bool setDataRate(uint8_t speed);

    /**
     * Same as NRF24 except there is no LNA and the PA levels are as follows:
     *
     * (0) represents -12dBm
     * (1) represents 2dBm
     * (2) represents 6dBm
     * (3) represents 8dBm
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
     * Once this function has been called, users need to call disableDynamicPayloads if they want to call it again, else it has no effect.
     * @note If using with RF24Network or RF24Mesh, users can call `radio.begin()` then `radio.enableDynamicPayloads(123);` before
     * initializing the network. This will enable maximum payload sizes and fragmentation/reassembly won't engage until this length is exceeded.
     * @param payloadSize Set the maximum payload size. Up to 125 with DPL enabled & CRC disabled or 123 with 16-bit CRC & DPL enabled
     */
    void enableDynamicPayloads(uint8_t payloadSize = DEFAULT_MAX_PAYLOAD_SIZE);

    /**
     * Same as NRF24
     */
    void disableDynamicPayloads();

    /**
     * NRF52840 will handle up to 127 byte payloads with CRC & DPL disabled, 125 with 16-bit CRC and DPL disabled
     */
    void setPayloadSize(uint8_t size);

    /**
     * Same as NRF24
     */
    uint8_t getPayloadSize();

    /**
     * Set the CRCLength (in bits)
     *
     * CRC cannot be disabled if auto-ack is enabled
     * @param length Specify one of the values (as defined by @ref nrf_crclength_e)
     * | @p length (enum value)     | description                    |
     * |:--------------------------:|:------------------------------:|
     * | @ref NRF_CRC_DISABLED (0)  | to disable using CRC checksums |
     * | @ref NRF_CRC_8 (1)         | to use 8-bit checksums         |
     * | @ref NRF_CRC_16 (2)        | to use 16-bit checksums        |
     * | @ref NRF_CRC_24 (3)        | to use 24-bit checksums        |
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
    void setAddressWidth(uint8_t a_width);

    /**
     * Same as NRF24
     */
    void printDetails();

    /**
     * When powering up from powerdown ALL radio settings will need to be re-applied
     *
     * Enables the Radio & HF Clock
     *
     * If encryption is enabled, also enables RNG & CCM peripherals
     */
    void powerUp();

    /**
     * When called ALL radio settings will be reverted to default
     *
     * Disables the Radio & HF Clock
     *
     * If encryption is enabled, also disables RNG & CCM peripherals
     */
    void powerDown();

    /**
     * Not implemented due to SOC
     */
    bool failureDetected;

    /**
     * Takes the radio out of TX state.
     *
     * Can be called after any type of write to put the radio into a disabled state
     */
    bool txStandBy();

    /**
     * Takes the radio out of TX state
     *
     * Can be called after any type of write to put the radio into a disabled state
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
     * A new function specific to the NRF52x devices, not available on NRF24
     * @return The function will return the RSSI, which is measured continuously and the value
     * filtered using a single-pole IIR filter. This is a negative value: received signal strength = -A dBm
     */
    uint8_t getRSSI();

    /**
     * Same as NRF24
     */
    uint8_t getARC();

    /**
     * Does nothing, added for backward compatibility
     */
    uint8_t flush_rx();

    /**
     * The IEEE 802.15.4 standard defines a specific time that is alotted for the MAC sublayer to process received data.
     * Usage of this interframe spacing (IFS) comes into play to avoid that two frames are transmitted too close to
     * each other in time.
     *
     * This is the time period in uS the radio will wait between the last bit of the previous transmission, and the
     * first bit of the next transmission.
     *
     * This is configured for compatibility with nRF24L01 radios. Can be set lower if communicating between nRF52x devices.
     */
    uint16_t interframeSpacing;

#ifdef NRF_HAS_ENERGY_DETECT
    uint8_t sample_ed(void);
#endif

    /**@}*/
    /**
     * @name Encryption
     *
     * Methods you can use to enable encryption & authentication
     */
    /**@{*/

    /**
     * Used internally to convert addresses
     */
    uint32_t addrConv32(uint32_t addr);
#if defined CCM_ENCRYPTION_ENABLED || defined(DOXYGEN)

    /**
     * Function to encrypt data
     *
     * Called automatically when a write is performed
     */
    uint8_t encrypt(void* bufferIn, uint8_t size);

    /**
     * Function to decrypt data
     *
     * Called automatically when incoming data is received
     */
    uint8_t decrypt(void* bufferIn, uint8_t size);

    /**
     * The data buffer where encrypted data is placed. See the datasheet p115 for the CCM data structure
     */
    uint8_t outBuffer[MAX_PACKET_SIZE + CCM_MIC_SIZE + CCM_START_SIZE];

    /**
     * Set our 16-byte (128-bit) encryption key
     */
    void setKey(uint8_t key[CCM_KEY_SIZE]);

    /**
     * Set the (default 3-byte) packet counter used for encryption
     */
    void setCounter(uint64_t counter);

    /**
     * Set IV for encryption.
     * This is only used for manual encryption, a random IV is generated using the on-board RNG for encryption
     * during normal operation.
     */
    void setIV(uint8_t IV[CCM_IV_SIZE]);

    /**
     * Enable use of the on-board AES CCM mode encryption
     *
     * Cipher block chaining - message authentication code (CCM) mode is an authenticated encryption
     * algorithm designed to provide both authentication and confidentiality during data transfer. CCM combines
     * counter mode encryption and CBC-MAC authentication
     *
     * Users need to take extra steps to prevent specific attacks, such as replay attacks, which can be prevented by
     * transmitting a timestamp or counter value, and only accepting packets with a current timestamp/counter value,
     * rejecting old data.
     *
     * Encryption uses a 5-byte IV and 3-byte counter, the sizes of which can be configured in nrf_to_nrf.h
     * Maximum: 8-byte IV, 4-byte counter, plus the MAC/MIC is 4-bytes
     */
    bool enableEncryption;
        /**@}*/
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
    uint32_t rxBase;
    uint32_t rxPrefix;
    uint32_t txBase;
    uint32_t txPrefix;
    bool ackPayloadAvailable;
    uint8_t ackAvailablePipeNo;
    uint8_t lastPacketCounter;
    uint16_t lastData;
    bool dynamicAckEnabled;
    uint8_t arcCounter;
    uint16_t ackTimeout;
    bool payloadAvailable;
    bool restartReturnRx();
    void openReadingPipe(uint8_t child, uint32_t base, uint32_t prefix);
    void openWritingPipe(uint32_t base, uint32_t prefix);
#if defined CCM_ENCRYPTION_ENABLED
    uint8_t inBuffer[MAX_PACKET_SIZE + CCM_MIC_SIZE + CCM_START_SIZE];
    uint8_t scratchPTR[MAX_PACKET_SIZE + CCM_MODE_LENGTH_EXTENDED];
    typedef struct
    {
        uint8_t key[CCM_KEY_SIZE];
        uint64_t counter;
        uint8_t direction;
        uint8_t iv[CCM_IV_SIZE_ACTUAL];
    } ccmData_t;
    ccmData_t ccmData;
    uint32_t packetCounter;
#endif
};

/*! \mainpage nrf_to_nrf - NRF52 radio driver
 *
 * \section Documentation for nrf_to_nrf
 *
 * See https://nrf24.github.io/RF24/ for the main nrf24 docs
 *
 * These docs are considered supplimental to the NRF24 documentation, mainly documenting the differences between NRF52 and NRF24 drivers
 *
 *
 */

/**
 * @example examples/RF24/GettingStarted/GettingStarted.ino
 */

/**
 * @example examples/RF24/GettingStartedEncryption/GettingStartedEncryption.ino
 */

/**
 * @example examples/RF24/GettingStartedMicros/GettingStartedMicros.ino
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
 * @example examples/RF24Network/helloworld_rxEncryption/helloworld_rxEncryption.ino
 */

/**
 * @example examples/RF24Network/helloworld_tx/helloworld_tx.ino
 */

/**
 * @example examples/RF24Network/helloworld_txEncryption/helloworld_txEncryption.ino
 */

/**
 * @example examples/RF24Mesh/RF24Mesh_Example/RF24Mesh_Example.ino
 */

/**
 * @example examples/RF24Mesh/RF24Mesh_ExampleEncryption/RF24Mesh_ExampleEncryption.ino
 */

/**
 * @example examples/RF24Mesh/RF24Mesh_Example_MasterEncryption/RF24Mesh_Example_MasterEncryption.ino
 */

/**
 * @example examples/RF24Ethernet/mqtt_basic/mqtt_basic.ino
 */

#endif //__nrf52840_nrf24l01_H__
