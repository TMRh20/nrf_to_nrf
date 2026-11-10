
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
#define ACTUAL_MAX_PAYLOAD_SIZE    258
#define ACK_TIMEOUT_1MBPS          600 // 300 with static payloads
#define ACK_TIMEOUT_2MBPS          400 // 265 with static payloads
#define ACK_TIMEOUT_250KBPS        800 // 500 with staticPayloads
#define ACK_TIMEOUT_1MBPS_OFFSET   300
#define ACK_TIMEOUT_2MBPS_OFFSET   135
#define ACK_TIMEOUT_250KBPS_OFFSET 300
#define ACK_PAYLOAD_TIMEOUT_OFFSET 750

// AES CCM ENCRYPTION
// The nRF54L15 CCM is a different peripheral with no NRF_CCM; with the nrf54l15clean core
// (https://github.com/lolren/nrf54-arduino-core) encryption uses its Ccm and CracenRng classes
#if defined(ARDUINO_NRF54L15) && defined(ARDUINO_ARCH_NRF54L15CLEAN)
    #define NRF54_CCM_ENCRYPTION
#endif
#if defined NRF_CCM || defined NRF54_CCM_ENCRYPTION || defined(DOXYGEN)
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

/**
 * Power Amplifier (PA) levels. The output power depends on the chip:
 *
 * | level (enum value)   | nRF52840, nRF52833, nRF52820, nRF54L15 | nRF52832, nRF52811, nRF52810, nRF52805 |
 * |:--------------------:|:--------------------------------------:|:--------------------------------------:|
 * | @ref NRF_PA_MIN (0)  | -12 dBm                                | -12 dBm                                |
 * | @ref NRF_PA_LOW (1)  | 2 dBm                                  | -4 dBm                                 |
 * | @ref NRF_PA_HIGH (2) | 6 dBm                                  | 0 dBm                                  |
 * | @ref NRF_PA_MAX (3)  | 8 dBm                                  | 4 dBm                                  |
 *
 * @see
 * - nrf_to_nrf::setPALevel()
 * - nrf_to_nrf::getPALevel()
 */
typedef enum
{
    /**
     * (0) represents -12dBm
     */
    NRF_PA_MIN = 0,
    /**
     * (1) represents 2dBm (-4dBm on nRF52832, nRF52811, nRF52810, nRF52805)
     */
    NRF_PA_LOW,
    /**
     * (2) represents 6dBm (0dBm on nRF52832, nRF52811, nRF52810, nRF52805)
     */
    NRF_PA_HIGH,
    /**
     * (3) represents 8dBm (4dBm on nRF52832, nRF52811, nRF52810, nRF52805)
     */
    NRF_PA_MAX,
    /**
     * (4) Returned by getPALevel() if the TX power was set to something other than the levels above.
     */
    NRF_PA_ERROR
} nrf_pa_dbm_e;

/**
 *
 *
 * How fast data moves through the air. Units are in bits per second (bps).
 * @see
 * - nrf_to_nrf::setDataRate()
 *
 */
typedef enum
{
    /** (0) represents 1 Mbps */
    NRF_1MBPS = 0,
    /** (1) represents 2 Mbps */
    NRF_2MBPS,
    /** (2) represents 250 kbps. Nordic has deprecated this mode, and it might not work reliably on all devices. */
    NRF_250KBPS
#ifdef ARDUINO_NRF54L15
        /** (3) represents 4 Mbps OBT4 */
        ,
    NRF_4MBPS_OBT4,
    /** (4) represents 4 Mbps OBT6 */
    NRF_4MBPS_OBT6
#endif
} nrf_datarate_e;

/**
 *
 *
 * The length of a CRC checksum that is used (if any). Cyclical Redundancy
 * Checking (CRC) is commonly used to ensure data integrity.
 * @see
 * - nrf_to_nrf::setCRCLength()
 * - nrf_to_nrf::getCRCLength()
 * - nrf_to_nrf::disableCRC()
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
 * @brief Driver for the 2.4GHz radio in nRF52x and nRF54x chips, using the RF24 API
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
     * Begin operation of the radio
     *
     * Call this in setup(), before calling any other methods. It starts the high and low frequency clocks
     * and configures the radio with these defaults: channel 76 (2476 MHz), 1 Mbps, 16-bit CRC, maximum PA level,
     * 5-byte addresses, 32-byte static payloads, auto-ack on all pipes, and 5 retries with a delay of 5.
     * It also enables the RX interrupt with enableRxInterrupt().
     * @code
     * if (!radio.begin()) {
     *   Serial.println(F("radio hardware not responding!"));
     *   while (1) {} // hold program in infinite loop to prevent subsequent errors
     * }
     * @endcode
     * @return
     * - `true` if the radio was initialized
     * - `false` if a clock failed to start
     */
    bool begin();

    /**
     * Check whether there is a payload available to be read
     * @code
     * if (radio.available()) {
     *   radio.read(&data, sizeof(data));
     * }
     * @endcode
     *
     * With the RX interrupt enabled (the default), this checks the RX queue filled by the interrupt.
     * In polling mode (disableRxInterrupt()), this checks the radio itself, so call it often:
     * only one received payload is held until it is read.
     *
     * After write() receives an ACK payload, this returns `true` for that payload until it is read.
     *
     * @see available(uint8_t*)
     * @return `true` if there is a payload available, `false` if there is none
     */
    bool available();

    /**
     * Check whether there is a payload available to be read, and which pipe received it
     *
     * @param[out] pipe_num Which pipe received the payload. For an ACK payload received by write(), this is pipe 0.
     * @code
     * uint8_t pipeNum;
     * if (radio.available(&pipeNum)) {
     *   radio.read(&data, sizeof(data));
     *   Serial.print("Received data on pipe ");
     *   Serial.println(pipeNum);
     * }
     * @endcode
     *
     * @see available()
     * @return `true` if there is a payload available, `false` if there is none
     */
    bool available(uint8_t* pipe_num);

    /**
     * Read the next available payload
     *
     * Call available() first. Each call removes one payload, however many bytes are read.
     * This is different from RF24, where a payload stays in the RX FIFO until all of it has been read.
     * @code
     * if (radio.available()) {
     *   uint8_t length = radio.getDynamicPayloadSize(); // or getPayloadSize() for static payloads
     *   radio.read(&data, length);
     * }
     * @endcode
     *
     * @param buf Pointer to a buffer where the data should be written
     * @param len Number of bytes to copy into @p buf. Use getDynamicPayloadSize() to get the payload's length.
     * Bytes past the end of the payload are not meaningful.
     *
     * @note If nothing is available, the last payload received is copied again.
     */
    void read(void* buf, uint8_t len);

    /**
     * Transmit a payload to the address set with openWritingPipe()
     *
     * Call stopListening() first. This blocks until an ACK is received or all retries have been used
     * (see setRetries()). If the ACK carries an ACK payload, read it with available() and read() afterwards.
     * @code
     * radio.stopListening();
     * radio.write(&data, sizeof(data));
     * @endcode
     *
     * @param buf Pointer to the data to be sent
     * @param len Number of bytes to be sent. With static payloads, a full getPayloadSize() packet is always sent,
     * and the bytes after @p len are not zeroed. With dynamic payloads, @p len can be up to the size passed to
     * enableDynamicPayloads(). With encryption, 12 more bytes (IV, counter and MIC) are added, and they must fit too.
     * @param multicast `true` sends the payload once without waiting for an ACK. enableDynamicAck() is not needed.
     * The receiver still sends an ACK if auto-ack is enabled on its pipe.
     * @param doEncryption `false` sends this payload unencrypted when encryption is enabled
     *
     * @return
     * - `true` if an ACK was received. Also `true`, even if the payload was not received, if @p multicast is
     *   `true` or auto-ack is disabled on pipe 0.
     * - `false` if no ACK was received after all retries
     */
    bool write(void* buf, uint8_t len, bool multicast = false, bool doEncryption = true);

    /**
     * Start listening on the pipes opened for reading
     *
     * 1. Call openReadingPipe() first.
     * 2. Call stopListening() before calling write().
     * 3. Call available() to check for incoming payloads, and read() to get them.
     *
     * Open reading pipe 1 using address `0xCCCECCCECC`
     * @code
     * uint8_t address[] = {0xCC, 0xCE, 0xCC, 0xCE, 0xCC};
     * radio.openReadingPipe(1, address);
     * radio.startListening();
     * @endcode
     *
     * @note openWritingPipe() uses pipe 0's address for transmitting, so this restores the address last set
     * for reading pipe 0 by openReadingPipe().
     *
     * @param resetAddresses Used internally; leave as the default
     */
    void startListening(bool resetAddresses = true);

    /**
     * Stop listening for incoming payloads, and switch to transmit mode
     *
     * Do this before calling write(). This restores the TX address set by openWritingPipe(), and payloads already
     * received can still be read.
     * @code
     * radio.stopListening();
     * radio.write(&data, sizeof(data));
     * @endcode
     *
     * @note Unlike RF24, ACK payloads loaded with writeAckPayload() are not discarded. Call flush_tx() to discard them.
     *
     * @param setWritingPipe Used internally; leave as the default
     * @param resetAddresses Used internally; leave as the default
     */
    void stopListening(bool setWritingPipe = true, bool resetAddresses = true);

    /**
     * Same as stopListening(), but also sets the TX address, like calling openWritingPipe() afterwards
     *
     * @param txAddress The new TX address
     * @param setWritingPipe Used internally; leave as the default
     * @param resetAddresses Used internally; leave as the default
     */
    void stopListening(const uint8_t* txAddress, bool setWritingPipe = true, bool resetAddresses = true);

    /**
     * Receive packets from the radio interrupt, so a slow loop() doesn't leave the radio idle.
     * available() and read() are used as normal. Don't use BLE in the same sketch.
     *
     * @note The library defines the radio interrupt handler, so a sketch can't define its own.
     *
     * This is called automatically on begin(), so if you want to disable it, call disableRxInterrupt()
     * after the radio and/or network/mesh has been configured.
     *
     * @param depth The number of packets to queue, from 1 to 254. Uses (depth + 1) * (payload size + 2) bytes of RAM,
     * plus depth * (payload size + 2) for the ACK payload queue if enableAckPayload() is called.
     * @return false if depth is invalid, there isn't enough memory, or this was already called
     */
    bool enableRxInterrupt(uint8_t depth = 3);

    /**
     * Disable the RX interrupt and free the RX queue and the ACK payload queue
     *
     * Payloads still in the RX queue are discarded. available() then checks the radio itself, so call it often:
     * only one received payload is held until it is read. The newest ACK payload still queued is kept.
     */
    void disableRxInterrupt();

    /** Called by the library's radio interrupt handler */
    void handleInterrupt();

    /**
     * Open a pipe for reading
     *
     * Up to 8 pipes (0-7) can be open for reading at once. Open all the required reading pipes, then call
     * startListening().
     *
     * @note Pipe 0 stores a full address. Pipes 1-7 share every byte except the first, and each call for one of them
     * sets the shared bytes for all of them. Pass the full address for each pipe, with only the first byte different:
     * @code
     * uint8_t addresses[][6] = {"Prime", "2Node", "3Node", "4Node"};
     * radio.openReadingPipe(0, addresses[0]); // address used is "Prime"
     * radio.openReadingPipe(1, addresses[1]); // address used is "2Node"
     * radio.openReadingPipe(2, addresses[2]); // address used is "3Node"
     * radio.openReadingPipe(3, addresses[3]); // address used is "4Node"
     * @endcode
     *
     * @warning openWritingPipe() uses pipe 0's address for transmitting. If pipe 0 is opened for reading, its
     * address is restored by every startListening().
     *
     * @see
     * - openWritingPipe()
     * - setAddressWidth()
     *
     * @param child Which pipe to open, 0-7
     * @param address The address, using the number of bytes set with setAddressWidth()
     */
    void openReadingPipe(uint8_t child, const uint8_t* address);

    /**
     * Open a pipe for writing
     *
     * Only one writing pipe can be open at a time. This sets the address that write() transmits to.
     * Call stopListening() first.
     * @code
     * uint8_t addresses[][6] = {"1Node", "2Node"};
     * radio.openWritingPipe(addresses[0]);
     * @endcode
     *
     * @warning This overwrites the address of reading pipe 0, which receives the ACKs. startListening() restores
     * the reading pipe 0 address set with openReadingPipe().
     *
     * @see
     * - setAddressWidth()
     * - startListening()
     * - stopListening()
     *
     * @param address The address to transmit to, using the number of bytes set with setAddressWidth().
     * Coordinate it with the receiving nodes (the pipe numbers don't need to match).
     */
    void openWritingPipe(const uint8_t* address);

    /**
     * Check that the radio responds, the same as isValid()
     *
     * The radio is part of the chip, so unlike RF24 this doesn't check an SPI connection.
     * @return `true` if the radio is usable
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
     * The radio can handle packets up to 255 bytes if CRC & DPL is disabled
     * 254 bytes if using 16-bit CRC & DPL is disabled
     *
     */
    uint8_t radioData[ACTUAL_MAX_PAYLOAD_SIZE + 2];

    /**
     * Same as write(), so it blocks until an ACK is received or all retries have been used
     *
     * Provided for compatibility with RF24, where writeFast() queues up to 3 payloads in the TX FIFO.
     * @param buf Pointer to the data to be sent
     * @param len Number of bytes to be sent
     * @param multicast `true` sends the payload once without waiting for an ACK
     * @return Same as write()
     */
    bool writeFast(void* buf, uint8_t len, bool multicast = 0);

    /**
     * Start transmitting a payload without waiting for it to finish
     *
     * Unlike write(), there are no retries and no ACK is received.
     *
     * @param buf Pointer to the data to be sent
     * @param len Number of bytes to be sent
     * @param multicast Not used, since no ACK is waited for either way
     * @param doEncryption `false` sends this payload unencrypted when encryption is enabled
     * @return Always `true`
     */
    bool startWrite(void* buf, uint8_t len, bool multicast, bool doEncryption = true);

    /**
     * Load a payload to send back in the ACK for the next packet received on @p pipe
     *
     * With the RX interrupt enabled (the default), payloads are queued like the NRF24 TX FIFO.
     * Up to the enableRxInterrupt() depth can be queued, so one can be loaded for each packet the RX queue holds.
     * Each packet received takes the oldest payload queued for its pipe and sends it once; a retransmitted
     * packet gets the same payload again. If none is queued for that pipe, a plain ACK is sent.
     * Payloads can be at most getPayloadSize() bytes, including encryption overhead.
     * Call enableAckPayload() first; it allocates the queue.
     *
     * In polling mode (disableRxInterrupt()), one payload is stored. It is sent with the ACK for the next
     * packet on @p pipe, re-sent if that packet is retransmitted, and discarded when the next new packet arrives.
     *
     * This can be called before and after startListening().
     *
     * @note Use dynamic payloads (enableDynamicPayloads()) on both nodes. Unlike RF24, enableAckPayload() doesn't
     * enable them.
     *
     * @see
     * - enableAckPayload()
     * - flush_tx()
     *
     * @param pipe Which pipe (0-7) will get this response
     * @param buf Pointer to the data to send
     * @param len Length of the data to send
     *
     * @return false if the queue is full, the payload is too large, or with the RX interrupt, if ACK payloads
     * aren't enabled or there wasn't enough memory for the queue
     */
    bool writeAckPayload(uint8_t pipe, void* buf, uint8_t len);

    /**
     * Enable custom payloads in the acknowledgement packets
     *
     * ACK payloads are a handy way to return data to senders without changing the radio modes on both nodes.
     * Call this on both the transmitting and receiving nodes.
     *
     * With the RX interrupt, this also allocates the ACK payload queue used by writeAckPayload():
     * depth * (payload size + 2) bytes of RAM, where depth is the enableRxInterrupt() depth.
     *
     * @note Unlike RF24, this doesn't enable dynamic payloads. Call enableDynamicPayloads() on both nodes too.
     * @remarks ACK payloads need auto-ack, which begin() enables on all pipes.
     *
     * @see
     * - writeAckPayload()
     * - setAutoAck()
     */
    void enableAckPayload();

    /**
     * Disable custom payloads in the acknowledgement packets
     *
     * This also frees the ACK payload queue, discarding any payloads loaded with writeAckPayload().
     * @see enableAckPayload()
     */
    void disableAckPayload();

    /**
     * Provided for compatibility with RF24. It isn't needed: the @p multicast parameter of write() works without it.
     */
    void enableDynamicAck();

    /**
     * Get the length of the next available payload
     * @code
     * if (radio.available()) {
     *   uint8_t length = radio.getDynamicPayloadSize();
     *   radio.read(&data, length);
     * }
     * @endcode
     * @return The length of the payload that read() returns next. With static payloads, this is getPayloadSize().
     */
    uint8_t getDynamicPayloadSize();

    /**
     * Check that the radio responds, by writing and reading back the channel register
     * @return `true` if the radio is usable
     */
    bool isValid();

    /**
     * Set the RF channel
     *
     * The frequency used is 2400 MHz + @p channel, or 2360 MHz + @p channel if @p map is 1. With @p map set to 0,
     * channels match the nRF24 channels. The default is channel 76 (2476 MHz).
     * @param channel Which RF channel to communicate on, 0-100
     * @param map 0 for 2400-2500 MHz (default), 1 for 2360-2460 MHz
     */
    void setChannel(uint8_t channel, bool map = 0);

    /**
     * Get the RF channel
     * @return The channel set with setChannel(). The @p map setting is not included.
     */
    uint8_t getChannel();

    /**
     * Set the transmission data rate. Both nodes must use the same rate.
     *
     * @param speed Specify one of the following values (as defined by @ref nrf_datarate_e):
     * | @p speed (enum value)     | description                                                  |
     * |:-------------------------:|:------------------------------------------------------------:|
     * | @ref NRF_1MBPS (0)        | for 1 Mbps                                                   |
     * | @ref NRF_2MBPS (1)        | for 2 Mbps                                                   |
     * | @ref NRF_250KBPS (2)      | for 250 kbps. Deprecated by Nordic, might not work reliably  |
     * | `NRF_4MBPS_OBT4` (3)      | for 4 Mbps, nRF54L15 only                                    |
     * | `NRF_4MBPS_OBT6` (4)      | for 4 Mbps, nRF54L15 only                                    |
     *
     * @return Always `true`
     */
    bool setDataRate(uint8_t speed);

    /**
     * Set the Power Amplifier (PA) level
     *
     * @param level One of the values defined by @ref nrf_pa_dbm_e. The output power depends on the chip:
     * | @p level (enum value) | nRF52840, nRF52833, nRF52820, nRF54L15 | nRF52832, nRF52811, nRF52810, nRF52805 |
     * |:---------------------:|:--------------------------------------:|:--------------------------------------:|
     * | @ref NRF_PA_MIN (0)   | -12 dBm                                | -12 dBm                                |
     * | @ref NRF_PA_LOW (1)   | 2 dBm                                  | -4 dBm                                 |
     * | @ref NRF_PA_HIGH (2)  | 6 dBm                                  | 0 dBm                                  |
     * | @ref NRF_PA_MAX (3)   | 8 dBm                                  | 4 dBm                                  |
     * @param lnaEnable Not used, since there is no LNA
     */
    void setPALevel(uint8_t level, bool lnaEnable = true);

    /**
     * Get the Power Amplifier (PA) level
     * @return One of the values defined by @ref nrf_pa_dbm_e, or @ref NRF_PA_ERROR if the TX power was set to
     * something else. See the table in setPALevel()
     */
    uint8_t getPALevel();

    /**
     * Enable or disable the auto-acknowledgement feature for all pipes
     *
     * This is enabled by default. Auto-acknowledgement responds to every received payload with an ACK packet.
     * To attach an ACK payload to an ACK packet, use writeAckPayload().
     *
     * If this is disabled on a transmitting node, write() always reports that the payload was received, even if
     * it wasn't. This setting needs to match on the transmitting and receiving nodes.
     *
     * @see
     * - write()
     * - writeAckPayload()
     *
     * @param enable Whether to enable (`true`) or disable (`false`) auto-acknowledgement for all pipes
     */
    void setAutoAck(bool enable);

    /**
     * Enable or disable the auto-acknowledgement feature for a specific pipe
     *
     * This is enabled by default for all pipes. Pipe 0 is used for transmitting: if this is disabled on pipe 0
     * of a transmitting node, write() always reports that the payload was received, even if it wasn't.
     * Enable this on any pipe listening to a transmitter that uses auto-ack.
     *
     * @note With static payloads, the packet format depends on whether auto-ack is enabled on pipe 0, so
     * auto-ack can't be enabled on some pipes and disabled on others: either ACKs are enabled or not.
     *
     * @param pipe Which pipe to configure, 0-7
     * @param enable Whether to enable (`true`) or disable (`false`) auto-acknowledgement for @p pipe
     */
    void setAutoAck(uint8_t pipe, bool enable);

    /**
     * Enable dynamically-sized payloads on all pipes
     *
     * Payloads are sent at the length passed to write(), instead of always using the static payload size.
     *
     * Once this function has been called, users need to call disableDynamicPayloads if they want to call it again, else it has no effect.
     * @note If using with RF24Network or RF24Mesh, users can call `radio.begin()` then `radio.enableDynamicPayloads(254);` before
     * initializing the network. This will enable maximum payload sizes and fragmentation/reassembly won't engage until this length is exceeded.
     * @param payloadSize Set the maximum payload size. Up to 254 with DPL enabled
     */
    void enableDynamicPayloads(uint8_t payloadSize = DEFAULT_MAX_PAYLOAD_SIZE);

    /**
     * Disable dynamically-sized payloads on all pipes, and use the static payload size
     *
     * Unlike RF24, this doesn't disable ACK payloads.
     */
    void disableDynamicPayloads();

    /**
     * Set the static payload size
     *
     * Every packet is sent at this size, however many bytes were passed to write(). If this is never called,
     * the size is 32 bytes. This also disables dynamic payloads.
     *
     * NRF52840 will handle up to 255 byte payloads with CRC & DPL disabled, 254 with 16-bit CRC and DPL
     * NRF54L15 will do the same
     * @param size The number of bytes in each payload
     */
    void setPayloadSize(uint8_t size);

    /**
     * Get the payload size
     * @return The static payload size, or with dynamic payloads, the maximum size passed to enableDynamicPayloads()
     */
    uint8_t getPayloadSize();

    /**
     * Set the CRCLength (in bits)
     *
     * Both nodes must use the same setting. Disabling CRC is allowed with auto-ack enabled, but then corrupt
     * packets aren't detected, and retransmitted packets aren't recognized as duplicates.
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
     * Get the CRCLength (in bits)
     * @return One of the values defined by @ref nrf_crclength_e. See the table in setCRCLength()
     */
    nrf_crclength_e getCRCLength();

    /**
     * Disable CRC validation, the same as setCRCLength(NRF_CRC_DISABLED)
     */
    void disableCRC();

    /**
     * Set the number of retries and the delay between them when transmitting with write()
     *
     * After each attempt, write() waits for an ACK for a time that depends on the data rate and payload type,
     * then waits for the delay before retrying.
     * @param retryVar How long to wait between retries, in multiples of about 258 us. The default is 5.
     * @param attempts How many retries before giving up, so write() makes up to @p attempts + 1 attempts.
     * The default is 5. Use 0 to disable retries.
     */
    void setRetries(uint8_t retryVar, uint8_t attempts);

    /**
     * Open a pipe for reading, using a 40-bit address
     *
     * Kept for compatibility with older code. See openReadingPipe(uint8_t, const uint8_t*), which this works like.
     * Pipes 1-7 share every byte except the least significant one, e.g.
     * @code
     * radio.openReadingPipe(1, 0xF0F0F0F0AA);
     * radio.openReadingPipe(2, 0xF0F0F0F0BB);
     * @endcode
     * @param child Which pipe to open, 0-7
     * @param address The 40-bit address of the pipe to open
     */
    void openReadingPipe(uint8_t child, uint64_t address);

    /**
     * Open a pipe for writing, using a 40-bit address
     *
     * Kept for compatibility with older code. See openWritingPipe(const uint8_t*), which this works like.
     * @code
     * radio.openWritingPipe(0xF0F0F0F0F0);
     * @endcode
     * @param address The 40-bit address of the pipe to open
     */
    void openWritingPipe(uint64_t address);

    /**
     * Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
     *
     * Both nodes must use the same width. The default is 5 bytes.
     * @param a_width The address width (in bytes) to use; this can be 3, 4 or 5
     */
    void setAddressWidth(uint8_t a_width);

    /**
     * Print a block of debugging information about the radio configuration to Serial
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
     * Get the result of the last write
     *
     * Provided for compatibility with RF24. write() and writeFast() already wait until the transmission is done,
     * so this doesn't wait or change the radio state.
     * @return `true` if the last write() or writeFast() succeeded, or after startWrite(); `false` if it failed
     */
    bool txStandBy();

    /**
     * Same as txStandBy()
     * @param timeout Not used
     * @param startTx Not used
     * @return Same as txStandBy()
     */
    bool txStandBy(uint32_t timeout, bool startTx = 0);

    /**
     * Test whether a signal stronger than -@p RSSI dBm is present on the channel
     *
     * Useful to check for interference on the current channel and for channel hopping strategies.
     * Unlike RF24, this samples the signal strength now, instead of reporting the last listening period.
     * The radio must be listening. Always returns `false` on the nRF54L15.
     * @code
     * if (radio.testCarrier()) {
     *   Serial.println("Signal stronger than -65dBm");
     * }
     * @endcode
     * @param RSSI The threshold in -dBm. The default of 65 means -65 dBm.
     * @return `true` if the signal is stronger than -@p RSSI dBm
     */
    bool testCarrier(uint8_t RSSI = 65);

    /**
     * Same as testCarrier()
     * @param RSSI The threshold in -dBm. The default of 65 means -65 dBm, close to the nRF24L01+ RPD threshold of -64 dBm.
     * @return `true` if the signal is stronger than -@p RSSI dBm
     */
    bool testRPD(uint8_t RSSI = 65);

    /**
     * A new function specific to the NRF52x devices, not available on NRF24
     *
     * The radio must be listening. Always returns 0 on the nRF54L15.
     * @return The function will return the RSSI, which is measured continuously and the value
     * filtered using a single-pole IIR filter. This is a negative value: received signal strength = -A dBm
     */
    uint8_t getRSSI();

    /**
     * Get the number of retries used by the last write()
     *
     * Allows roughly estimating signal quality.
     * @return From 0 (acknowledged on the first attempt) to the @p attempts value passed to setRetries()
     */
    uint8_t getARC();

    /**
     * Discard all received payloads that haven't been read, including an ACK payload received by write()
     * @return Always 0. RF24 returns the status register, which the nRF52 radio doesn't have.
     */
    uint8_t flush_rx();

    /**
     * Discard any ACK payloads loaded with writeAckPayload()
     *
     * Unlike RF24, stopListening() doesn't call this, and there are no regular payloads to discard,
     * since write() sends its payload immediately.
     * @return Always 0
     */
    uint8_t flush_tx();

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
    /**
     * Sample the energy level on the channel (nRF52840, nRF52833, nRF52820 and nRF52811 only)
     * @return The energy level on the IEEE 802.15.4 scale, from 0 to 255
     */
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
    volatile bool inRxMode;
    uint8_t staticPayloadSize;
    uint8_t ackPID;
    uint8_t ackPipe;
    bool lastTxResult;
    uint32_t rxBase;
    uint32_t rxPrefix;
    uint32_t txBase;
    uint32_t txPrefix;
    volatile bool ackPayloadAvailable;
    uint8_t ackAvailablePipeNo;
    uint8_t lastPacketCounter;
    uint16_t lastData;
    bool dynamicAckEnabled;
    uint8_t arcCounter;
    uint16_t ackTimeout;
    volatile bool payloadAvailable;
    bool restartReturnRx();
    void openReadingPipe(uint8_t child, uint32_t base, uint32_t prefix);
    void openWritingPipe(uint32_t base, uint32_t prefix);
    bool pollRadio(uint8_t* pipe_num);
    uint8_t* rxQueueFront();
    uint8_t* rxQueue;
    uint16_t rxQueueSlotSize;
    uint8_t rxQueueSlots;
    volatile uint8_t rxQueueHead;
    volatile uint8_t rxQueueTail;
    uint8_t* ackQueue;
    uint16_t ackQueueSlotSize;
    uint8_t ackQueueDepth;
    volatile uint8_t ackQueueHead;
    volatile uint8_t ackQueueCount;
    bool ackPayloadSent;
    void loadAckPayload(uint8_t pipe);
    void allocateAckQueue();
    bool inInterrupt;
    void resizeRxQueue();
    void setStaticLength(uint8_t size);
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
    bool generateIV(uint8_t iv[CCM_IV_SIZE]);
#endif
};

/*! \mainpage nrf_to_nrf - NRF52 radio driver
 *
 * \section Documentation for nrf_to_nrf
 *
 *
 * nrf_to_nrf uses the same API as RF24. These docs cover the whole API, and note where nrf_to_nrf behaves differently from RF24
 *
 * <img width="2560" height="960" alt="nrf_to_nrf_banner" src="https://github.com/user-attachments/assets/c248691a-8c60-4ed6-bfa6-0d55177e9f93" />
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
