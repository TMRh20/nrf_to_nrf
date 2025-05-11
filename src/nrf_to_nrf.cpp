

#include "nrf_to_nrf.h"

#if defined(NRF52832_XXAA) || defined(NRF52832_XXAB) || defined(NRF52811_XXAA) || defined(NRF52810_XXAA) || defined(NRF52805_XXAA)
    // TX power range (Product Specification): -20 .. +4dbm, configurable in 4 dB steps
    #define TXPOWER_PA_MIN  0xF4 // -12dBm
    #define TXPOWER_PA_LOW  0xFC // -4dBm
    #define TXPOWER_PA_HIGH 0x00 // 0dBm
    #define TXPOWER_PA_MAX  0x04 // 4dBm
#else                            // nRF52840, nRF52833, nRF52820
    // TX power range (Product Specification): -20 .. +8dbm, configurable in 4 dB steps
    #define TXPOWER_PA_MIN  0xF4 // -12dBm
    #define TXPOWER_PA_LOW  0x02 // 2dBm
    #define TXPOWER_PA_HIGH 0x06 // 6dBm
    #define TXPOWER_PA_MAX  0x08 // 8dBm
#endif

// Note that 250Kbit mode is deprecated and might not work reliably on all devices.
// See: https://devzone.nordicsemi.com/f/nordic-q-a/78469/250-kbit-s-nordic-proprietary-radio-mode-on-nrf52840
#ifndef RADIO_MODE_MODE_Nrf_250Kbit
    #define RADIO_MODE_MODE_Nrf_250Kbit (2UL)
#endif

/**********************************************************************************************************/

// Function to do bytewise bit-swap on an unsigned 32-bit value
static uint32_t bytewise_bit_swap(uint8_t const* p_inp)
{
    uint32_t inp = (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;
}

/**********************************************************************************************************/

// Convert a base address from nRF24L format to nRF5 format
static uint32_t addr_conv(uint8_t const* p_addr)
{
    return __REV(
        bytewise_bit_swap(p_addr)); // lint -esym(628, __rev) -esym(526, __rev) */
}

/**********************************************************************************************************/

uint32_t nrf_to_nrf::addrConv32(uint32_t addr)
{

    uint8_t buffer[4];
    buffer[0] = addr & 0xFF;
    buffer[1] = (addr >> 8) & 0xFF;
    buffer[2] = (addr >> 16) & 0xFF;
    buffer[3] = (addr >> 24) & 0xFF;

    return addr_conv(buffer);
}

/**********************************************************************************************************/

nrf_to_nrf::nrf_to_nrf()
{
    DPL = false;
    staticPayloadSize = 32;
    retries = 5;
    retryDuration = 5;
    ackPayloadsEnabled = false;
    ackPipe = 0;
    inRxMode = false;
    arcCounter = 0;
    ackTimeout = ACK_TIMEOUT_1MBPS;
    payloadAvailable = false;
    enableEncryption = false;

#if defined CCM_ENCRYPTION_ENABLED
    NRF_CCM->INPTR = (uint32_t)inBuffer;
    NRF_CCM->OUTPTR = (uint32_t)outBuffer;
    NRF_CCM->CNFPTR = (uint32_t)&ccmData;
    NRF_CCM->SCRATCHPTR = (uint32_t)scratchPTR;
    ccmData.counter = 12345;

#endif
};

/**********************************************************************************************************/

bool nrf_to_nrf::begin()
{

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
        // Do nothing.
    }

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    /* Wait for the low frequency clock to start up */
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
        // Do nothing.
    }

    NRF_RADIO->POWER = 1;

    NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (1 << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) | (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) | (4 << RADIO_PCNF1_BALEN_Pos) | (staticPayloadSize << RADIO_PCNF1_STATLEN_Pos) | (staticPayloadSize << RADIO_PCNF1_MAXLEN_Pos);

    NRF_RADIO->BASE0 = 0xE7E7E7E7; /* Base address 0 */
    NRF_RADIO->BASE1 = 0x43434343;
    NRF_RADIO->PREFIX0 = 0x23C343E7; /* Prefixes bytes for logical addresses 0 */
    NRF_RADIO->PREFIX1 = 0x13E363A3;
    NRF_RADIO->RXADDRESSES = 0x01;
    NRF_RADIO->TXADDRESS = 0x00;
    /* Receive address select    */
    txBase = NRF_RADIO->BASE0;
    txPrefix = NRF_RADIO->PREFIX0;
    rxBase = NRF_RADIO->BASE0;
    rxPrefix = NRF_RADIO->PREFIX0;
    // Configure CRC for 16-bit
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two; /* CRC configuration: 16bit */
    NRF_RADIO->CRCINIT = 0xFFFFUL;            // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;           // CRC poly: x^16+x^12^x^5+1

    NRF_RADIO->PACKETPTR = (uint32_t)radioData;
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->MODECNF0 = 0x200;
    NRF_RADIO->MODECNF0 |= 1;
    NRF_RADIO->TXPOWER = (TXPOWER_PA_MAX << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->SHORTS = 1 << 19;
    NRF_RADIO->FREQUENCY = 0x4C;

    DPL = false;
    // Enable auto ack on all pipes by default
    setAutoAck(1);

    return 1;
}

/**********************************************************************************************************/

#ifdef NRF_HAS_ENERGY_DETECT
    #define ED_RSSISCALE 4 // From electrical specifications
uint8_t nrf_to_nrf::sample_ed(void)
{
    int val;
    NRF_RADIO->TASKS_EDSTART = 1; // Start
    while (NRF_RADIO->EVENTS_EDEND != 1) {
        // CPU can sleep here or do something else
        // Use of interrupts are encouraged
    }
    val = NRF_RADIO->EDSAMPLE; // Read level
    return (uint8_t)(val > 63
                         ? 255
                         : val * ED_RSSISCALE); // Convert to IEEE 802.15.4 scale
}
#endif

/**********************************************************************************************************/

bool nrf_to_nrf::available()
{
    uint8_t pipe_num = 0;
    return available(&pipe_num);
}

/**********************************************************************************************************/

bool nrf_to_nrf::available(uint8_t* pipe_num)
{

    if (payloadAvailable) {
        *pipe_num = (uint8_t)NRF_RADIO->RXMATCH;
        return true;
    }

    if (!inRxMode) {
        if (ackPayloadAvailable) {
            *pipe_num = ackAvailablePipeNo;
            return true;
        }
    }
    if (NRF_RADIO->EVENTS_CRCOK) {
        uint32_t counter = 0;
#if defined CCM_ENCRYPTION_ENABLED
        uint8_t tmpIV[CCM_IV_SIZE];
#endif
        NRF_RADIO->EVENTS_CRCOK = 0;
        if (DPL) {
            if (radioData[0] > ACTUAL_MAX_PAYLOAD_SIZE - 4 && NRF_RADIO->CRCCNF == RADIO_CRCCNF_LEN_Two) {
                return restartReturnRx();
            }
            else if (radioData[0] > ACTUAL_MAX_PAYLOAD_SIZE - 3 && NRF_RADIO->CRCCNF == RADIO_CRCCNF_LEN_One) {
                return restartReturnRx();
            }
            else if (radioData[0] > ACTUAL_MAX_PAYLOAD_SIZE - 2 && NRF_RADIO->CRCCNF == 0) {
                return restartReturnRx();
            }
        }

        *pipe_num = (uint8_t)NRF_RADIO->RXMATCH;
        if (!DPL && acksEnabled(*pipe_num) == false) {
#if defined CCM_ENCRYPTION_ENABLED
            if (enableEncryption) {
                memcpy(&rxBuffer[1], &radioData[CCM_IV_SIZE + CCM_COUNTER_SIZE], staticPayloadSize - CCM_MIC_SIZE - CCM_IV_SIZE - CCM_COUNTER_SIZE);
                memcpy(tmpIV, &radioData[0], CCM_IV_SIZE);
                memcpy(&counter, &radioData[CCM_IV_SIZE], CCM_COUNTER_SIZE);
            }
            else {
#endif
                memcpy(&rxBuffer[1], &radioData[0], staticPayloadSize);
#if defined CCM_ENCRYPTION_ENABLED
            }
#endif
        }
        else {
#if defined CCM_ENCRYPTION_ENABLED
            if (enableEncryption) {
                if (DPL) {
                    memcpy(&rxBuffer[1], &radioData[2 + CCM_IV_SIZE + CCM_COUNTER_SIZE], max(0, radioData[0] - CCM_IV_SIZE - CCM_COUNTER_SIZE));
                }
                else {
                    memcpy(&rxBuffer[1], &radioData[2 + CCM_IV_SIZE + CCM_COUNTER_SIZE], max(0, staticPayloadSize - CCM_IV_SIZE - CCM_COUNTER_SIZE));
                }
                memcpy(tmpIV, &radioData[2], CCM_IV_SIZE);
                memcpy(&counter, &radioData[2 + CCM_IV_SIZE], CCM_COUNTER_SIZE);
            }
            else {
#endif
                if (DPL) {
                    memcpy(&rxBuffer[1], &radioData[2], radioData[0]);
                }
                else {
                    memcpy(&rxBuffer[1], &radioData[2], staticPayloadSize);
                }
#if defined CCM_ENCRYPTION_ENABLED
            }
#endif
        }

        rxFifoAvailable = true;
        uint8_t packetCtr = 0;
        if (DPL) {
            packetCtr = radioData[1];
            rxBuffer[0] = radioData[0];
        }
        else {
            packetCtr = radioData[0];
            rxBuffer[0] = staticPayloadSize;
        }

        ackPID = packetCtr;
        uint16_t packetData = NRF_RADIO->RXCRC;
        // If ack is enabled on this receiving pipe
        if (acksEnabled(NRF_RADIO->RXMATCH)) {
            stopListening(false, false);
            uint32_t txAddress = NRF_RADIO->TXADDRESS;
            NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;
            delayMicroseconds(75);
            if (ackPayloadsEnabled) {
                if (*pipe_num == ackPipe) {
                    write(&ackBuffer[1], ackBuffer[0], 1, 0);
                }
                else {
                    write(0, 0, 1, 0);
                }
            }
            else {
                uint8_t payloadSize = 0;
                if (!DPL) {
                    payloadSize = getPayloadSize();
                    setPayloadSize(0);
                }
                write(0, 0, 1, 0); // Send an ACK
                if (!DPL) {
                    setPayloadSize(payloadSize);
                }
            }
            NRF_RADIO->TXADDRESS = txAddress;
            startListening(false);

            // If the packet has the same ID number and data, it is most likely a
            // duplicate
            if (NRF_RADIO->CRCCNF != 0) { // If CRC enabled, check this data
                if (packetCtr == lastPacketCounter && packetData == lastData) {
                    return restartReturnRx();
                }
            }
        }

#if defined CCM_ENCRYPTION_ENABLED
        if (enableEncryption) {
            ccmData.counter = counter;
            memcpy(&ccmData.iv[0], &tmpIV[0], CCM_IV_SIZE);
            uint8_t bufferLength = 0;
            if (DPL) {
                bufferLength = rxBuffer[0] - CCM_IV_SIZE - CCM_COUNTER_SIZE;
            }
            else {
                bufferLength = staticPayloadSize - CCM_IV_SIZE - CCM_COUNTER_SIZE;
            }
            if (!decrypt(&rxBuffer[1], bufferLength)) {
                Serial.println("DECRYPT FAIL");
                return restartReturnRx();
            }

            memset(&rxBuffer[1], 0, sizeof(rxBuffer) - 1);

            if (DPL) {
                rxBuffer[0] -= (CCM_MIC_SIZE + CCM_IV_SIZE + CCM_COUNTER_SIZE);
                memcpy(&rxBuffer[1], &outBuffer[CCM_START_SIZE], rxBuffer[0]);
            }
            else {
                memcpy(&rxBuffer[1], &outBuffer[CCM_START_SIZE], staticPayloadSize - (CCM_MIC_SIZE - CCM_IV_SIZE - CCM_COUNTER_SIZE));
            }
        }
#endif
        lastPacketCounter = packetCtr;
        lastData = packetData;
        if (inRxMode) {
            NRF_RADIO->TASKS_START = 1;
        }
        if (rxBuffer[0]) {
            payloadAvailable = true;
            return 1;
        }
    }
    if (NRF_RADIO->EVENTS_CRCERROR) {
        NRF_RADIO->EVENTS_CRCERROR = 0;
        NRF_RADIO->TASKS_START = 1;
    }
    return 0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::restartReturnRx()
{
    if (inRxMode) {
        NRF_RADIO->TASKS_START = 1;
    }
    return 0;
}

/**********************************************************************************************************/

void nrf_to_nrf::read(void* buf, uint8_t len)
{
    memcpy(buf, &rxBuffer[1], len);
    ackPayloadAvailable = false;
    payloadAvailable = false;
}

/**********************************************************************************************************/

bool nrf_to_nrf::write(void* buf, uint8_t len, bool multicast, bool doEncryption)
{

    uint8_t PID = ackPID;
    if (DPL) {
        PID = ((ackPID += 1) % 7) << 1;
    }
    else {
        PID = ackPID++;
    }
    uint8_t payloadSize = 0;

#if defined CCM_ENCRYPTION_ENABLED
    uint8_t tmpIV[CCM_IV_SIZE];
    uint32_t tmpCounter = 0;
    uint8_t tmpBuffer[MAX_PACKET_SIZE + CCM_MIC_SIZE + CCM_START_SIZE];

    if (enableEncryption && doEncryption) {
        if (len) {

            for (int i = 0; i < CCM_IV_SIZE; i++) {
                while (!NRF_RNG->EVENTS_VALRDY) {
                }
                NRF_RNG->EVENTS_VALRDY = 0;
                tmpIV[i] = NRF_RNG->VALUE;
                ccmData.iv[i] = tmpIV[i];
            }
            tmpCounter = packetCounter;
            ccmData.counter = tmpCounter;

            if (!encrypt(buf, len)) {
                return 0;
            }

            memcpy(tmpBuffer, &outBuffer[CCM_START_SIZE], len + CCM_MIC_SIZE);
            len += CCM_IV_SIZE + CCM_COUNTER_SIZE + CCM_MIC_SIZE;
            packetCounter++;
            if (packetCounter > 200000) {
                packetCounter = 0;
            }
        }
    }
#endif

    for (int i = 0; i < (retries + 1); i++) {
        arcCounter = i;
        if (DPL) {
            radioData[0] = len;
            radioData[1] = PID;
        }
        else {
            radioData[1] = 0;
            radioData[0] = PID;
        }

        uint8_t dataStart = 0;

#if defined CCM_ENCRYPTION_ENABLED

        if (enableEncryption && doEncryption) {
            dataStart = (!DPL && acksEnabled(0) == false) ? CCM_IV_SIZE + CCM_COUNTER_SIZE : CCM_IV_SIZE + CCM_COUNTER_SIZE + 2;
        }
        else {
#endif
            dataStart = (!DPL && acksEnabled(0) == false) ? 0 : 2;
#if defined CCM_ENCRYPTION_ENABLED
        }
#endif

#if defined CCM_ENCRYPTION_ENABLED
        if (enableEncryption && doEncryption) {
            memcpy(&radioData[dataStart - CCM_COUNTER_SIZE], &tmpCounter, CCM_COUNTER_SIZE);
            memcpy(&radioData[dataStart - CCM_IV_SIZE - CCM_COUNTER_SIZE], &tmpIV[0], CCM_IV_SIZE);
            memcpy(&radioData[dataStart], &tmpBuffer[0], len - (CCM_IV_SIZE + CCM_COUNTER_SIZE));
        }
        else {
#endif
            memcpy(&radioData[dataStart], buf, len);
#if defined CCM_ENCRYPTION_ENABLED
        }
#endif

        NRF_RADIO->EVENTS_END = 0;
        NRF_RADIO->TASKS_START = 1;
        while (NRF_RADIO->EVENTS_END == 0) {
        }
        NRF_RADIO->EVENTS_END = 0;
        if (!multicast && acksPerPipe[NRF_RADIO->TXADDRESS] == true) {
            uint32_t rxAddress = NRF_RADIO->RXADDRESSES;
            NRF_RADIO->RXADDRESSES = 1 << NRF_RADIO->TXADDRESS;
            if (!DPL) {
                payloadSize = getPayloadSize();
                setPayloadSize(0);
            }
            startListening(false);

            uint32_t realAckTimeout = ackTimeout;
            if (!DPL) {
                if (NRF_RADIO->MODE == (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos)) {
                    realAckTimeout -= ACK_TIMEOUT_1MBPS_OFFSET;
                }
                else if (NRF_RADIO->MODE == (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos)) {
                    realAckTimeout -= ACK_TIMEOUT_250KBPS_OFFSET;
                }
                else {
                    realAckTimeout -= ACK_TIMEOUT_2MBPS_OFFSET;
                }
            }
            else {
                if (ackPayloadsEnabled && staticPayloadSize <= DEFAULT_MAX_PAYLOAD_SIZE) {
                    realAckTimeout += 200;
                }
                else {
                    realAckTimeout += ACK_PAYLOAD_TIMEOUT_OFFSET;
                }
            }
            uint32_t ack_timeout = micros();
            while (!NRF_RADIO->EVENTS_CRCOK && !NRF_RADIO->EVENTS_CRCERROR) {
                if (micros() - ack_timeout > realAckTimeout) {
                    break;
                }
            }
            if (NRF_RADIO->EVENTS_CRCOK) {
                if (ackPayloadsEnabled && radioData[0] > 0) {
#if defined CCM_ENCRYPTION_ENABLED
                    if (enableEncryption && doEncryption) {
                        memcpy(&rxBuffer[1], &radioData[2 + CCM_COUNTER_SIZE + CCM_IV_SIZE], max(0, radioData[0] - CCM_COUNTER_SIZE - CCM_IV_SIZE));
                    }
                    else {
                        memcpy(&rxBuffer[1], &radioData[2], radioData[0]);
                    }
#else
                    memcpy(&rxBuffer[1], &radioData[2], radioData[0]);
#endif

#if defined CCM_ENCRYPTION_ENABLED
                    if (enableEncryption && radioData[0] > 0) {
                        memcpy(ccmData.iv, &radioData[2], CCM_IV_SIZE);
                        memcpy(&ccmData.counter, &radioData[2 + CCM_IV_SIZE], CCM_COUNTER_SIZE);

                        if (!decrypt(&rxBuffer[1], radioData[0])) {
                            Serial.println("DECRYPT FAIL");
                            return 0;
                        }
                        if (radioData[0] >= CCM_MIC_SIZE + CCM_COUNTER_SIZE + CCM_IV_SIZE) {
                            radioData[0] -= CCM_MIC_SIZE + CCM_COUNTER_SIZE + CCM_IV_SIZE;
                        }
                        memcpy(&rxBuffer[1], &outBuffer[CCM_START_SIZE], radioData[0]);
                    }
#endif
                    rxBuffer[0] = radioData[0];
                    ackPayloadAvailable = true;
                    ackAvailablePipeNo = NRF_RADIO->RXMATCH;
                }
                NRF_RADIO->EVENTS_CRCOK = 0;
                stopListening(false, false);
                if (!DPL) {
                    setPayloadSize(payloadSize);
                }
                NRF_RADIO->RXADDRESSES = rxAddress;
                lastTxResult = true;
                return 1;
            }
            else if (NRF_RADIO->EVENTS_CRCERROR) {
                NRF_RADIO->EVENTS_CRCERROR = 0;
            }
            uint32_t duration = 258 * retryDuration;
            delayMicroseconds(duration);
            stopListening(false, false);
            if (!DPL) {
                setPayloadSize(payloadSize);
            }
            NRF_RADIO->RXADDRESSES = rxAddress;
        }
        else {
            lastTxResult = true;
            return 1;
        }
    }
    lastTxResult = false;
    return 0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::startWrite(void* buf, uint8_t len, bool multicast, bool doEncryption)
{

    uint8_t PID = ackPID;
    if (DPL) {
        PID = ((ackPID += 1) % 7) << 1;
    }
    else {
        PID = ackPID++;
    }

#if defined CCM_ENCRYPTION_ENABLED
    uint8_t tmpIV[CCM_IV_SIZE];
    uint32_t tmpCounter = 0;
    uint8_t tmpBuffer[MAX_PACKET_SIZE + CCM_MIC_SIZE + CCM_START_SIZE];

    if (enableEncryption && doEncryption) {
        if (len) {

            for (int i = 0; i < CCM_IV_SIZE; i++) {
                while (!NRF_RNG->EVENTS_VALRDY) {
                }
                NRF_RNG->EVENTS_VALRDY = 0;
                tmpIV[i] = NRF_RNG->VALUE;
                ccmData.iv[i] = tmpIV[i];
            }
            tmpCounter = packetCounter;
            ccmData.counter = tmpCounter;

            if (!encrypt(buf, len)) {
                return 0;
            }

            memcpy(tmpBuffer, &outBuffer[CCM_START_SIZE], len + CCM_MIC_SIZE);
            len += CCM_IV_SIZE + CCM_COUNTER_SIZE + CCM_MIC_SIZE;
            packetCounter++;
            if (packetCounter > 200000) {
                packetCounter = 0;
            }
        }
    }
#endif

    //   for (int i = 0; i < retries; i++) {
    arcCounter = 0;
    if (DPL) {
        radioData[0] = len;
        radioData[1] = PID;
    }
    else {
        radioData[1] = 0;
        radioData[0] = PID;
    }

    uint8_t dataStart = 0;

#if defined CCM_ENCRYPTION_ENABLED

    if (enableEncryption && doEncryption) {
        dataStart = (!DPL && acksEnabled(0) == false) ? CCM_IV_SIZE + CCM_COUNTER_SIZE : CCM_IV_SIZE + CCM_COUNTER_SIZE + 2;
    }
    else {
#endif
        dataStart = (!DPL && acksEnabled(0) == false) ? 0 : 2;
#if defined CCM_ENCRYPTION_ENABLED
    }
#endif

#if defined CCM_ENCRYPTION_ENABLED
    if (enableEncryption && doEncryption) {
        memcpy(&radioData[dataStart - CCM_COUNTER_SIZE], &tmpCounter, CCM_COUNTER_SIZE);
        memcpy(&radioData[dataStart - CCM_IV_SIZE - CCM_COUNTER_SIZE], &tmpIV[0], CCM_IV_SIZE);
        memcpy(&radioData[dataStart], &tmpBuffer[0], len - (CCM_IV_SIZE + CCM_COUNTER_SIZE));
    }
    else {
#endif
        memcpy(&radioData[dataStart], buf, len);
#if defined CCM_ENCRYPTION_ENABLED
    }
#endif

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    lastTxResult = true;

    return true;
}

/**********************************************************************************************************/

bool nrf_to_nrf::writeAckPayload(uint8_t pipe, void* buf, uint8_t len)
{

#if defined CCM_ENCRYPTION_ENABLED
    if (enableEncryption) {
        if (len) {

            for (int i = 0; i < CCM_IV_SIZE; i++) {
                while (!NRF_RNG->EVENTS_VALRDY) {
                }
                NRF_RNG->EVENTS_VALRDY = 0;
                ccmData.iv[i] = NRF_RNG->VALUE;
                ackBuffer[i + 1] = ccmData.iv[i];
            }

            ccmData.counter = packetCounter;
            memcpy(&ackBuffer[1 + CCM_IV_SIZE], &ccmData.counter, CCM_COUNTER_SIZE);

            if (!encrypt(buf, len)) {
                return 0;
            }

            len += CCM_IV_SIZE + CCM_COUNTER_SIZE + CCM_MIC_SIZE;
            memcpy(&ackBuffer[1 + CCM_IV_SIZE + CCM_COUNTER_SIZE], &outBuffer[CCM_START_SIZE], len - CCM_IV_SIZE - CCM_COUNTER_SIZE);
            packetCounter++;
            if (packetCounter > 200000) {
                packetCounter = 0;
            }
        }
    }
    else {
#endif
        memcpy(&ackBuffer[1], buf, len);
#if defined CCM_ENCRYPTION_ENABLED
    }
#endif
    ackBuffer[0] = len;
    ackPipe = pipe;
    return true;
}

/**********************************************************************************************************/

void nrf_to_nrf::enableAckPayload() { ackPayloadsEnabled = true; }

/**********************************************************************************************************/

void nrf_to_nrf::disableAckPayload() { ackPayloadsEnabled = false; }

/**********************************************************************************************************/

void nrf_to_nrf::startListening(bool resetAddresses)
{

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0) {
    }
    NRF_RADIO->EVENTS_DISABLED = 0;

    if (resetAddresses == true) {
        NRF_RADIO->BASE0 = rxBase;
        NRF_RADIO->PREFIX0 = rxPrefix;
    }

    NRF_RADIO->EVENTS_RXREADY = 0;
    NRF_RADIO->EVENTS_CRCOK = 0;
    NRF_RADIO->TASKS_RXEN = 1;

    inRxMode = true;
}

/**********************************************************************************************************/

void nrf_to_nrf::stopListening(bool setWritingPipe, bool resetAddresses)
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0) {
    }
    NRF_RADIO->EVENTS_DISABLED = 0;

    if (resetAddresses) {
        NRF_RADIO->BASE0 = txBase;
        NRF_RADIO->PREFIX0 = txPrefix;
    }
    if (setWritingPipe) {
        NRF_RADIO->TXADDRESS = 0x00;
    }

    if (NRF_RADIO->STATE < 9) {
        NRF_RADIO->EVENTS_TXREADY = 0;
        NRF_RADIO->TASKS_TXEN = 1;
        while (NRF_RADIO->EVENTS_TXREADY == 0) {
        }
        NRF_RADIO->EVENTS_TXREADY = 0;
    }

    inRxMode = false;
}

/**********************************************************************************************************/

void nrf_to_nrf::stopListening(const uint8_t* txAddress, bool setWritingPipe, bool resetAddresses)
{
    stopListening(setWritingPipe, resetAddresses);
    openWritingPipe(txAddress);
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getDynamicPayloadSize()
{
    uint8_t size = min(staticPayloadSize, rxBuffer[0]);
    return size;
}

/**********************************************************************************************************/

bool nrf_to_nrf::isValid()
{

    uint32_t freq = NRF_RADIO->FREQUENCY;
    NRF_RADIO->FREQUENCY = 0x4C;
    if (NRF_RADIO->FREQUENCY == 0x4C) {
        NRF_RADIO->FREQUENCY = freq;
        return 1;
    }
    return 0;
}

/**********************************************************************************************************/

void nrf_to_nrf::setChannel(uint8_t channel) { NRF_RADIO->FREQUENCY = channel; }

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getChannel() { return (uint8_t)NRF_RADIO->FREQUENCY; }

/**********************************************************************************************************/

void nrf_to_nrf::setAutoAck(bool enable)
{

    for (int i = 0; i < 8; i++) {
        acksPerPipe[i] = enable;
    }
    if (!DPL) {
        disableDynamicPayloads(); // Called to re-configure the PCNF0 register
    }
}

/**********************************************************************************************************/

void nrf_to_nrf::setAutoAck(uint8_t pipe, bool enable)
{

    acksPerPipe[pipe] = enable;
    if (!DPL) {
        disableDynamicPayloads(); // Called to re-configure the PCNF0 register
    }
}

/**********************************************************************************************************/

void nrf_to_nrf::enableDynamicPayloads(uint8_t payloadSize)
{

    if (!DPL) {
        DPL = true;
        staticPayloadSize = payloadSize;

        if (payloadSize <= 63) {
            NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (6 << RADIO_PCNF0_LFLEN_Pos) | (3 << RADIO_PCNF0_S1LEN_Pos);
        }
        else {
            // Using 8 bits for length
            NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (8 << RADIO_PCNF0_LFLEN_Pos) | (3 << RADIO_PCNF0_S1LEN_Pos);
        }

        NRF_RADIO->PCNF1 &= ~(0xFF << RADIO_PCNF1_MAXLEN_Pos | 0xFF << RADIO_PCNF1_STATLEN_Pos);
        NRF_RADIO->PCNF1 |= payloadSize << RADIO_PCNF1_MAXLEN_Pos;
    }
}

/**********************************************************************************************************/

void nrf_to_nrf::disableDynamicPayloads()
{
    DPL = false;

    uint8_t lenConfig = 0;
    if (acksEnabled(0)) {
        lenConfig = 1;
    }
    NRF_RADIO->PCNF0 = (lenConfig << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (lenConfig << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO->PCNF1 &= ~(0xFF << RADIO_PCNF1_MAXLEN_Pos | 0xFF << RADIO_PCNF1_STATLEN_Pos);
    NRF_RADIO->PCNF1 |= staticPayloadSize << RADIO_PCNF1_STATLEN_Pos | staticPayloadSize << RADIO_PCNF1_MAXLEN_Pos;
}

/**********************************************************************************************************/

void nrf_to_nrf::setPayloadSize(uint8_t size)
{
    staticPayloadSize = size;
    DPL = false;

    uint8_t lenConfig = 0;
    if (acksEnabled(0)) {
        lenConfig = 1;
    }
    NRF_RADIO->PCNF0 = (lenConfig << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (lenConfig << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO->PCNF1 &= ~(0xFF << RADIO_PCNF1_MAXLEN_Pos | 0xFF << RADIO_PCNF1_STATLEN_Pos);
    NRF_RADIO->PCNF1 |= staticPayloadSize << RADIO_PCNF1_STATLEN_Pos | staticPayloadSize << RADIO_PCNF1_MAXLEN_Pos;
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getPayloadSize()
{
    return staticPayloadSize;
}

/**********************************************************************************************************/

void nrf_to_nrf::setRetries(uint8_t retryVar, uint8_t attempts)
{

    retries = attempts;
    retryDuration = retryVar;
}

/**********************************************************************************************************/

void nrf_to_nrf::openReadingPipe(uint8_t child, uint64_t address)
{
    uint32_t base = addrConv32(address >> 8);
    uint32_t prefix = addrConv32(address & 0xFF) >> 24;

    openReadingPipe(child, base, prefix);
}

/**********************************************************************************************************/

void nrf_to_nrf::openWritingPipe(uint64_t address)
{
    uint32_t base = addrConv32(address >> 8);
    uint32_t prefix = addrConv32(address & 0xFF) >> 24;

    openWritingPipe(base, prefix);
}

/**********************************************************************************************************/

void nrf_to_nrf::openReadingPipe(uint8_t child, const uint8_t* address)
{
    uint32_t base = addr_conv(&address[1]);
    uint32_t prefix = addr_conv(&address[0]) >> 24;

    openReadingPipe(child, base, prefix);
}

/**********************************************************************************************************/

void nrf_to_nrf::openReadingPipe(uint8_t child, uint32_t base, uint32_t prefix)
{

    // Using pipes 1-7 for reading pipes, leaving pipe0 for a tx pipe
    if (!child) {
        NRF_RADIO->PREFIX0 = rxPrefix;
        NRF_RADIO->BASE0 = base;
        NRF_RADIO->PREFIX0 &= ~(0xFF);
        NRF_RADIO->PREFIX0 |= prefix;
        rxBase = NRF_RADIO->BASE0;
        rxPrefix = NRF_RADIO->PREFIX0;
    }
    else if (child < 4) { // prefixes AP1-3 are in prefix0
        NRF_RADIO->PREFIX0 = rxPrefix;
        NRF_RADIO->BASE1 = base;
        NRF_RADIO->PREFIX0 &= ~(0xFF << (8 * child));
        NRF_RADIO->PREFIX0 |= prefix << (8 * child);
        rxPrefix = NRF_RADIO->PREFIX0;
    }
    else {
        NRF_RADIO->BASE1 = base;
        NRF_RADIO->PREFIX1 &= ~(0xFF << (8 * (child - 4)));
        NRF_RADIO->PREFIX1 |= prefix << (8 * (child - 4));
    }
    NRF_RADIO->RXADDRESSES |= 1 << child;
}

/**********************************************************************************************************/

void nrf_to_nrf::openWritingPipe(const uint8_t* address)
{

    uint32_t base = addr_conv(&address[1]);
    uint32_t prefix = addr_conv(&address[0]) >> 24;

    openWritingPipe(base, prefix);
}

/**********************************************************************************************************/

void nrf_to_nrf::openWritingPipe(uint32_t base, uint32_t prefix)
{

    NRF_RADIO->BASE0 = base;
    NRF_RADIO->PREFIX0 &= ~(0xFF);
    NRF_RADIO->PREFIX0 |= prefix;
    NRF_RADIO->TXADDRESS = 0x00;
    txBase = NRF_RADIO->BASE0;
    txPrefix = NRF_RADIO->PREFIX0;
}
/**********************************************************************************************************/

bool nrf_to_nrf::txStandBy()
{

    if (NRF_RADIO->STATE == 11) {
        while (NRF_RADIO->EVENTS_END == 0) {
        }
        NRF_RADIO->EVENTS_END = 0;
    }

    NRF_RADIO->TASKS_STOP = 1;

    return lastTxResult;
}

/**********************************************************************************************************/

bool nrf_to_nrf::txStandBy(uint32_t timeout, bool startTx)
{

    if (NRF_RADIO->STATE == 11) {
        while (NRF_RADIO->EVENTS_END == 0) {
        }
        NRF_RADIO->EVENTS_END = 0;
    }

    NRF_RADIO->TASKS_STOP = 1;

    return lastTxResult;
}

/**********************************************************************************************************/

bool nrf_to_nrf::writeFast(void* buf, uint8_t len, bool multicast)
{
    lastTxResult = write((void*)buf, len, multicast);
    return lastTxResult;
}

/**********************************************************************************************************/

bool nrf_to_nrf::acksEnabled(uint8_t pipe)
{

    if (acksPerPipe[pipe]) {
        return 1;
    }
    return 0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::isChipConnected() { return isValid(); }

/**********************************************************************************************************/

bool nrf_to_nrf::setDataRate(uint8_t speed)
{

    if (speed == NRF_1MBPS) {
        NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
        ackTimeout = ACK_TIMEOUT_1MBPS;
    }
    else if (speed == NRF_250KBPS) {
        NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos);
        ackTimeout = ACK_TIMEOUT_250KBPS;
    }
    else { // NRF_2MBPS
        NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);
        ackTimeout = ACK_TIMEOUT_2MBPS;
    }

    return 1;
}

/**********************************************************************************************************/

void nrf_to_nrf::setPALevel(uint8_t level, bool lnaEnable)
{

    uint8_t paLevel = 0x00;

    if (level == NRF_PA_MIN) {
        paLevel = TXPOWER_PA_MIN;
    }
    else if (level == NRF_PA_LOW) {
        paLevel = TXPOWER_PA_LOW;
    }
    else if (level == NRF_PA_HIGH) {
        paLevel = TXPOWER_PA_HIGH;
    }
    else if (level == NRF_PA_MAX) {
        paLevel = TXPOWER_PA_MAX;
    }
    NRF_RADIO->TXPOWER = paLevel;
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getPALevel()
{

    uint8_t paLevel = NRF_RADIO->TXPOWER;

    if (paLevel == TXPOWER_PA_MIN) {
        return NRF_PA_MIN;
    }
    else if (paLevel == TXPOWER_PA_LOW) {
        return NRF_PA_LOW;
    }
    else if (paLevel == TXPOWER_PA_HIGH) {
        return NRF_PA_HIGH;
    }
    else if (paLevel == TXPOWER_PA_MAX) {
        return NRF_PA_MAX;
    }
    else {
        return NRF_PA_ERROR;
    }
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getARC()
{
    return arcCounter;
}

/**********************************************************************************************************/

void nrf_to_nrf::setCRCLength(nrf_crclength_e length)
{
    if (length == NRF_CRC_24) {
        NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Three; /* CRC configuration: 24bit */
        NRF_RADIO->CRCINIT = 0xFFFFFFUL;            // Initial value
        NRF_RADIO->CRCPOLY = 0x65BUL;
    }
    else if (length == NRF_CRC_16) {
        NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two; /* CRC configuration: 16bit */
        NRF_RADIO->CRCINIT = 0xFFFFUL;            // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;           // CRC poly: x^16+x^12^x^5+1
    }
    else if (length == NRF_CRC_8) {
        NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_One; /* CRC configuration: 8bit */
        NRF_RADIO->CRCINIT = 0xFFUL;
        NRF_RADIO->CRCPOLY = 0x107UL;
    }
    else {
        NRF_RADIO->CRCCNF = 0; /* CRC configuration: Disabled */
        NRF_RADIO->CRCINIT = 0x00L;
        NRF_RADIO->CRCPOLY = 0x00UL;
    }
}

/**********************************************************************************************************/

nrf_crclength_e nrf_to_nrf::getCRCLength()
{
    if (NRF_RADIO->CRCCNF == 0) {
        return NRF_CRC_DISABLED;
    }
    else if (NRF_RADIO->CRCCNF == RADIO_CRCCNF_LEN_One) {
        return NRF_CRC_8;
    }
    if (NRF_RADIO->CRCCNF == RADIO_CRCCNF_LEN_Two) {
        return NRF_CRC_16;
    }
    else {
        return NRF_CRC_24;
    }
}

/**********************************************************************************************************/

bool nrf_to_nrf::testCarrier(uint8_t RSSI)
{

    NRF_RADIO->EVENTS_RSSIEND = 0;
    NRF_RADIO->TASKS_RSSISTART = 1;
    while (!NRF_RADIO->EVENTS_RSSIEND) {
    }
    if (NRF_RADIO->RSSISAMPLE < RSSI) {
        return 1;
    }
    return 0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::testRPD(uint8_t RSSI)
{
    return testCarrier(RSSI);
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getRSSI()
{
    NRF_RADIO->EVENTS_RSSIEND = 0;
    NRF_RADIO->TASKS_RSSISTART = 1;
    while (!NRF_RADIO->EVENTS_RSSIEND) {
    }
    return (uint8_t)NRF_RADIO->RSSISAMPLE;
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::flush_rx()
{
    return 0;
}

/**********************************************************************************************************/

void nrf_to_nrf::powerUp()
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
        // Do nothing.
    }

    NRF_RADIO->POWER = 1;

    if (enableEncryption) {
        NRF_RNG->CONFIG = 1;
        NRF_RNG->TASKS_START = 1;
        NRF_CCM->ENABLE = 2;
    }
}

/**********************************************************************************************************/

void nrf_to_nrf::powerDown()
{
    NRF_RADIO->POWER = 0;
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
    if (enableEncryption) {
        NRF_RNG->TASKS_STOP = 1;
        NRF_RNG->CONFIG = 0;
        NRF_CCM->ENABLE = 0;
    }
}

/**********************************************************************************************************/
void nrf_to_nrf::setAddressWidth(uint8_t a_width)
{
    NRF_RADIO->PCNF1 &= ~(0xFF << RADIO_PCNF1_BALEN_Pos);
    NRF_RADIO->PCNF1 |= (a_width - 1) << RADIO_PCNF1_BALEN_Pos;
}

/**********************************************************************************************************/

void nrf_to_nrf::printDetails()
{
    uint8_t addressWidth = ((NRF_RADIO->PCNF1 >> 16) & 0xFF) + 1;

    Serial.println("================ Radio Configuration ================");
    Serial.print("STATUS\t\t= ");
    Serial.println(NRF_RADIO->STATE);

    // Serial.println(addrConv32(NRF_RADIO->PREFIX0);
    Serial.print("RX_ADDR_P0-1\t= 0x");
    uint32_t base = addrConv32(NRF_RADIO->BASE0);
    for (int i = addressWidth - 2; i > -1; i--) {
        Serial.print((base >> (i * 8)) & 0xFF, HEX);
    }
    uint32_t prefixes = addrConv32(NRF_RADIO->PREFIX0);
    uint8_t prefix = (prefixes >> 24) & 0xFF;
    Serial.print(prefix, HEX);
    Serial.print(" 0x");
    base = addrConv32(NRF_RADIO->BASE1);
    for (int i = addressWidth - 2; i > -1; i--) {
        Serial.print((base >> (i * 8)) & 0xFF, HEX);
    }
    prefix = (prefixes >> 16) & 0xFF;
    Serial.println(prefix, HEX);

    Serial.print("RX_ADDR_P2-7\t= 0x");
    prefix = (prefixes >> 8) & 0xFF;
    Serial.print(prefix, HEX);
    Serial.print(" 0x");
    prefix = (prefixes)&0xFF;
    Serial.print(prefix, HEX);
    Serial.print(" 0x");
    prefixes = addrConv32(NRF_RADIO->PREFIX1);
    prefix = (prefixes >> 24) & 0xFF;
    Serial.print(prefix, HEX);
    Serial.print(" 0x");
    prefix = (prefixes >> 16) & 0xFF;
    Serial.print(prefix, HEX);
    Serial.print(" 0x");
    prefix = (prefixes >> 8) & 0xFF;
    Serial.print(prefix, HEX);
    Serial.print(" 0x");
    prefix = (prefixes)&0xFF;
    Serial.println(prefix, HEX);

    uint8_t enAA = 0;
    for (int i = 0; i < 6; i++) {
        enAA |= acksPerPipe[i] << i;
    }
    Serial.print("EN_AA\t\t= 0x");
    Serial.println(enAA, HEX);
    Serial.print("EN_RXADDR\t= 0x");
    Serial.println(NRF_RADIO->RXADDRESSES, HEX);
    Serial.print("RF_CH\t\t= 0x");
    Serial.println(NRF_RADIO->FREQUENCY, HEX);
    Serial.println("DYNPD/FEATURE\t= 0x");
    Serial.print("Data Rate\t= ");
    Serial.println(NRF_RADIO->MODE ? "2 MBPS" : "1MBPS");
    Serial.println("Model\t\t= NRF52");
    Serial.print("CRC Length\t= ");
    uint8_t crcLen = getCRCLength();
    if (crcLen == NRF_CRC_16) {
        Serial.println("16 bits");
    }
    else if (crcLen == NRF_CRC_8) {
        Serial.println("8 bits");
    }
    else {
        Serial.println("Disabled");
    }
    Serial.print("PA Power\t= ");
    uint8_t paLevel = getPALevel();
    if (paLevel == NRF_PA_MAX) {
        Serial.println("PA_MAX");
    }
    else if (paLevel == NRF_PA_HIGH) {
        Serial.println("PA_HIGH");
    }
    else if (paLevel == NRF_PA_LOW) {
        Serial.println("PA_LOW");
    }
    else if (paLevel == NRF_PA_MIN) {
        Serial.println("PA_MIN");
    }
    else {
        Serial.println("?");
    }
    Serial.print("ARC\t\t= ");
    Serial.println(arcCounter);
}

/**********************************************************************************************************/

#if defined CCM_ENCRYPTION_ENABLED

uint8_t nrf_to_nrf::encrypt(void* bufferIn, uint8_t size)
{
    NRF_CCM->MODE = 0 | 1 << 24 | 1 << 16;

    if (!size) {
        return 0;
    }
    if (size > MAX_PACKET_SIZE) {
        return 0;
    }

    inBuffer[0] = 0;
    inBuffer[1] = size;
    inBuffer[2] = 0;

    memcpy(&inBuffer[CCM_START_SIZE], bufferIn, size);
    memset(outBuffer, 0, sizeof(outBuffer));

    NRF_CCM->OUTPTR = (uint32_t)outBuffer;
    NRF_CCM->EVENTS_ENDKSGEN = 0;
    NRF_CCM->EVENTS_ENDCRYPT = 0;
    NRF_CCM->TASKS_KSGEN = 1;
    while (!NRF_CCM->EVENTS_ENDCRYPT) {
    };

    if (NRF_CCM->EVENTS_ERROR) {
        return 0;
    }
    return outBuffer[1];
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::decrypt(void* bufferIn, uint8_t size)
{
    NRF_CCM->MODE = 1 | 1 << 24 | 1 << 16;

    if (!size) {
        return 0;
    }
    if (size > MAX_PACKET_SIZE) {
        return 0;
    }

    memcpy(&inBuffer[3], bufferIn, size);

    inBuffer[0] = 0;
    inBuffer[1] = size;
    inBuffer[2] = 0;

    memset(outBuffer, 0, sizeof(outBuffer));

    NRF_CCM->EVENTS_ENDKSGEN = 0;
    NRF_CCM->EVENTS_ENDCRYPT = 0;
    NRF_CCM->TASKS_KSGEN = 1;

    while (!NRF_CCM->EVENTS_ENDCRYPT) {
    };

    if (NRF_CCM->EVENTS_ERROR) {
        return 0;
    }

    if (NRF_CCM->MICSTATUS == (CCM_MICSTATUS_MICSTATUS_CheckFailed << CCM_MICSTATUS_MICSTATUS_Pos)) {
        return 0;
    }

    return outBuffer[1];
}

/**********************************************************************************************************/

void nrf_to_nrf::setKey(uint8_t key[CCM_KEY_SIZE])
{

    NRF_CCM->MODE = 1 << 24 | 1 << 16;
    NRF_CCM->MAXPACKETSIZE = MAX_PACKET_SIZE;
    NRF_CCM->SHORTS = 1;
    NRF_CCM->ENABLE = 2;

    NRF_RNG->CONFIG = 1;
    NRF_RNG->TASKS_START = 1;

    memcpy(ccmData.key, key, CCM_KEY_SIZE);
}

/**********************************************************************************************************/

void nrf_to_nrf::setCounter(uint64_t counter)
{

    ccmData.counter = counter;
    packetCounter = counter;
}
/**********************************************************************************************************/

void nrf_to_nrf::setIV(uint8_t IV[CCM_IV_SIZE])
{

    for (int i = 0; i < CCM_IV_SIZE; i++) {
        ccmData.iv[i] = IV[i];
    }
}

#endif // defined CCM_ENCRYPTION_ENABLED
