

#include "nrf_to_nrf.h"

/**********************************************************************************************************/

// Function to do bytewise bit-swap on an unsigned 32-bit value
static uint32_t bytewise_bit_swap(uint8_t const *p_inp) {
  uint32_t inp =
      (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
  inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
  inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
  inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
  return inp;
}

/**********************************************************************************************************/

// Convert a base address from nRF24L format to nRF5 format
static uint32_t addr_conv(uint8_t const *p_addr) {
  return __REV(
      bytewise_bit_swap(p_addr)); // lint -esym(628, __rev) -esym(526, __rev) */
}

/**********************************************************************************************************/

uint32_t nrf_to_nrf::addrConv32(uint32_t addr) {

  uint8_t buffer[4];
  buffer[0] = addr & 0xFF;
  buffer[1] = (addr >> 8) & 0xFF;
  buffer[2] = (addr >> 16) & 0xFF;
  buffer[3] = (addr >> 24) & 0xFF;

  return addr_conv(buffer);
}

/**********************************************************************************************************/

nrf_to_nrf::nrf_to_nrf() {
  // Enable auto ack on all pipes by default
  for (uint8_t i = 0; i < 8; i++) {
    acksPerPipe[i] = true;
  }
  staticPayloadSize = 32;
  DPL = false;
  retries = 5;
  retryDuration = 5;
  ackPayloadsEnabled = false;
  ackPipe = 0;
  inRxMode = false;
  radioConfigured = false;
  ARC = 0;
  addressWidth = 5;
};

/**********************************************************************************************************/

bool nrf_to_nrf::begin() {

  if (radioConfigured) {
    return 1;
  }

  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    // Do nothing.
  }

  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;

  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
    // Do nothing.
  }

  NRF_RADIO->POWER = 1;
  // NRF_POWER->DCDCEN=1;

  NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                     (0 << RADIO_PCNF0_LFLEN_Pos) |
                     (1 << RADIO_PCNF0_S1LEN_Pos);

  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                     (4 << RADIO_PCNF1_BALEN_Pos) |
                     (staticPayloadSize << RADIO_PCNF1_STATLEN_Pos) |
                     (staticPayloadSize << RADIO_PCNF1_MAXLEN_Pos);

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
  NRF_RADIO->TXPOWER = (0x8 << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->SHORTS = 1 << 19;
  NRF_RADIO->FREQUENCY = 0x4C;

  radioConfigured = true;
  return 1;
}

/**********************************************************************************************************/

#define ED_RSSISCALE 4 // From electrical specifications
uint8_t nrf_to_nrf::sample_ed(void) {
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

/**********************************************************************************************************/

bool nrf_to_nrf::available() { return available(NULL); }

/**********************************************************************************************************/

bool nrf_to_nrf::available(uint8_t *pipe_num) {


  if (!inRxMode) {
    if (ackPayloadAvailable) {
      *pipe_num = ackAvailablePipeNo;
      return true;
    }
  }
  if (NRF_RADIO->EVENTS_CRCOK) {
    NRF_RADIO->EVENTS_CRCOK = 0;
    *pipe_num = (uint8_t)NRF_RADIO->RXMATCH;
    memcpy(&rxBuffer[1], &radioData[2], staticPayloadSize);
    rxBuffer[0] = radioData[0];
    rxFifoAvailable = true;
    uint8_t packetCtr = 0;
    if (DPL) {
      packetCtr = radioData[1];
    } else {
      packetCtr = radioData[0];
    }

    ackPID = packetCtr;
    uint8_t packetData = radioData[2];
    // If ack is enabled on this receiving pipe
    if (acksEnabled(NRF_RADIO->RXMATCH)) {
      stopListening(false, false);
      uint32_t txAddress = NRF_RADIO->TXADDRESS;
      NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;
      delayMicroseconds(55);
      if (ackPayloadsEnabled) {
        if (*pipe_num == ackPipe) {
          write(&ackBuffer[1], ackBuffer[0], 1);
        } else {
          write(0, 0, 1);
        }
      } else {
        write(0, 0, 1); // Send an ACK
      }
      NRF_RADIO->TXADDRESS = txAddress;
      startListening(false);
    }

    // If the packet has the same ID number and data, it is most likely a
    // duplicate
    if (packetCtr == lastPacketCounter && packetData == lastData) {
      NRF_RADIO->TASKS_START = 1;
      return 0;
    }
    lastPacketCounter = packetCtr;
    lastData = packetData;
    return 1;
  }
  if (NRF_RADIO->EVENTS_CRCERROR) {
    NRF_RADIO->EVENTS_CRCERROR = 0;
    NRF_RADIO->TASKS_START = 1;
    // Serial.println("CRC ERR");
  }

  return 0;
}

/**********************************************************************************************************/

void nrf_to_nrf::read(void *buf, uint8_t len) {
  memcpy(buf, &rxBuffer[1], len);
  ackPayloadAvailable = false;
  if (inRxMode) {
    NRF_RADIO->TASKS_START = 1;
  }
}

/**********************************************************************************************************/

bool nrf_to_nrf::write(void *buf, uint8_t len, bool multicast) {

  if (DPL) {
    radioData[0] = len;
    radioData[1] = ((ackPID += 1) % 7) << 1;
  } else {
    radioData[1] = 0;      // ackPID++;//((radioData[0] + 1) % 4) << 1;
    radioData[0] = ackPID++; //((ackPID+=1) % 7) << 1;;
  }

  for (int i = 0; i < retries; i++) {
    ARC = i;
    memset(&radioData[2], 0, staticPayloadSize);
    memcpy(&radioData[2], buf, len);

    // radioData[0] = ackPID++;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    while (NRF_RADIO->EVENTS_END == 0) {
    }
    NRF_RADIO->EVENTS_END = 0;
    if (!multicast && acksPerPipe[NRF_RADIO->TXADDRESS] == true) {
      uint32_t rxAddress = NRF_RADIO->RXADDRESSES;
      NRF_RADIO->RXADDRESSES = 1 << NRF_RADIO->TXADDRESS;
      startListening(false);
      uint32_t ack_timeout = micros();
      while (!NRF_RADIO->EVENTS_CRCOK) { 
        if (micros() - ack_timeout > 400) {
          break;
        }
      }
      if (NRF_RADIO->EVENTS_CRCOK) {
        if (ackPayloadsEnabled && radioData[0] > 0) {
          memcpy(&rxBuffer[1], &radioData[2], staticPayloadSize);
          rxBuffer[0] = radioData[0];
          ackPayloadAvailable = true;
          ackAvailablePipeNo = NRF_RADIO->RXMATCH;
        }
        NRF_RADIO->EVENTS_CRCOK = 0;
        stopListening(false, false);
        NRF_RADIO->RXADDRESSES = rxAddress;
        return 1;
      }
      uint32_t duration = 258 * retryDuration;
      delayMicroseconds(duration);
      stopListening(false, false);
      NRF_RADIO->RXADDRESSES = rxAddress;
    } else {
      return 1;
    }
  }
  return 0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::startWrite(void *buf, uint8_t len, bool multicast) {

  NRF_RADIO->TASKS_START = 1;

}
/**********************************************************************************************************/

bool nrf_to_nrf::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len) {
  memcpy(&ackBuffer[1], buf, len);
  ackBuffer[0] = len;
  ackPipe = pipe;
}

/**********************************************************************************************************/

void nrf_to_nrf::enableAckPayload() { ackPayloadsEnabled = true; }

/**********************************************************************************************************/

void nrf_to_nrf::disableAckPayload() { ackPayloadsEnabled = false; }

/**********************************************************************************************************/

void nrf_to_nrf::startListening(bool resetAddresses) {

  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0)
    ;
  NRF_RADIO->EVENTS_DISABLED = 0;
  if (resetAddresses == true) {
    //   Serial.println("rst ad");
    NRF_RADIO->BASE0 = rxBase;
    NRF_RADIO->PREFIX0 = rxPrefix;
    // Serial.println(addrConv32(NRF_RADIO->BASE0),HEX);
    // Serial.println(addrConv32(NRF_RADIO->PREFIX0),HEX);
  }
  NRF_RADIO->EVENTS_RXREADY = 0;
  NRF_RADIO->EVENTS_CRCOK = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  inRxMode = true;
}

/**********************************************************************************************************/

void nrf_to_nrf::stopListening(bool setWritingPipe, bool resetAddresses) {

  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);
  NRF_RADIO->EVENTS_DISABLED = 0;
  if (resetAddresses) {
    NRF_RADIO->BASE0 = txBase;
    NRF_RADIO->PREFIX0 = txPrefix;
  }
  if (setWritingPipe) {
    NRF_RADIO->TXADDRESS = 0x00;
  }
  NRF_RADIO->EVENTS_TXREADY = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (NRF_RADIO->EVENTS_TXREADY == 0);
  NRF_RADIO->EVENTS_TXREADY = 0;
  inRxMode = false;
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getDynamicPayloadSize() {
  uint8_t size = min(staticPayloadSize, rxBuffer[0]);
  return size;
}

/**********************************************************************************************************/

bool nrf_to_nrf::isValid() {
  
  uint32_t freq = NRF_RADIO->FREQUENCY;
  NRF_RADIO->FREQUENCY = 0x4C;
  if(NRF_RADIO->FREQUENCY == 0x4C){
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

void nrf_to_nrf::setAutoAck(bool enable) {

  for (int i = 0; i < 8; i++) {
    acksPerPipe[i] = enable;
  }
}

/**********************************************************************************************************/

void nrf_to_nrf::setAutoAck(uint8_t pipe, bool enable) {

  acksPerPipe[pipe] = enable;
}

/**********************************************************************************************************/

void nrf_to_nrf::enableDynamicPayloads(uint8_t payloadSize) {
  DPL = true;
  staticPayloadSize = payloadSize;
 
  if(payloadSize <= 32){
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (6 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos);
  }else{                   
    // Using 8 bits for length
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos) ;
  }
  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                     (addressWidth-1 << RADIO_PCNF1_BALEN_Pos) |
                     (0 << RADIO_PCNF1_STATLEN_Pos) |
                     (payloadSize << RADIO_PCNF1_MAXLEN_Pos);
}

/**********************************************************************************************************/

void nrf_to_nrf::disableDynamicPayloads() {
  DPL = false;
  NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                     (0 << RADIO_PCNF0_LFLEN_Pos) |
                     (1 << RADIO_PCNF0_S1LEN_Pos);

  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                     (addressWidth-1 << RADIO_PCNF1_BALEN_Pos) |
                     (staticPayloadSize << RADIO_PCNF1_STATLEN_Pos) |
                     (staticPayloadSize << RADIO_PCNF1_MAXLEN_Pos);
}

/**********************************************************************************************************/

void nrf_to_nrf::setPayloadSize(uint8_t size) {
  staticPayloadSize = size;
  DPL = false;
  NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                     (0 << RADIO_PCNF0_LFLEN_Pos) |
                     (1 << RADIO_PCNF0_S1LEN_Pos);

  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
                     (addressWidth-1 << RADIO_PCNF1_BALEN_Pos) |
                     (staticPayloadSize << RADIO_PCNF1_STATLEN_Pos) |
                     (staticPayloadSize << RADIO_PCNF1_MAXLEN_Pos);
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getPayloadSize(){
  return staticPayloadSize;
}

/**********************************************************************************************************/

void nrf_to_nrf::setRetries(uint8_t retryVar, uint8_t attempts) {
  
  retries = attempts;
  retryDuration = retryVar;
  
}

/**********************************************************************************************************/

void nrf_to_nrf::openReadingPipe(uint8_t child, uint64_t address) {

  // child += 1;
  uint32_t base = address >> 8;
  uint32_t prefix = address & 0xFF;

  base = addrConv32(base);

  prefix = addrConv32(address);
  prefix = prefix >> 24;

  if (!child) {
    NRF_RADIO->BASE0 = base;
    NRF_RADIO->PREFIX0 &= ~(0xFF);
    NRF_RADIO->PREFIX0 |= prefix;
  } else if (child < 4) { // prefixes AP1-3 are in prefix0
    NRF_RADIO->BASE1 = base;
    NRF_RADIO->PREFIX0 &= ~(0xFF << (8 * child));
    NRF_RADIO->PREFIX0 |= prefix << (8 * child);
  } else {
    NRF_RADIO->BASE1 = base;
    NRF_RADIO->PREFIX1 &= ~(0xFF << (8 * (child - 4)));
    NRF_RADIO->PREFIX1 |= prefix << (8 * (child - 4));
  }
  NRF_RADIO->RXADDRESSES |= 1 << child;
  rxBase = NRF_RADIO->BASE0;
  rxPrefix = NRF_RADIO->PREFIX0;
  // Serial.println(addrConv32(NRF_RADIO->BASE1),HEX);
  // Serial.println(addrConv32(NRF_RADIO->PREFIX0),HEX);
  // Serial.println(NRF_RADIO->RXADDRESSES);
}
void nrf_to_nrf::openWritingPipe(uint64_t address) {
  uint32_t base = address >> 8;
  uint32_t prefix = address & 0xFF;
  base = addrConv32(base);

  prefix = addrConv32(address);
  prefix = prefix >> 24;

  NRF_RADIO->BASE0 = base;
  NRF_RADIO->PREFIX0 &= 0xFFFFFF00;
  NRF_RADIO->PREFIX0 |= prefix;
  NRF_RADIO->TXADDRESS = 0x00;
  txBase = NRF_RADIO->BASE0;
  txPrefix = NRF_RADIO->PREFIX0;
  //    Serial.println(addrConv32(NRF_RADIO->BASE0),HEX);
  // Serial.println(addrConv32(NRF_RADIO->PREFIX0),HEX);
}

/**********************************************************************************************************/

void nrf_to_nrf::openReadingPipe(uint8_t child, const uint8_t *address) {

  // child +=1;

  uint32_t base = addr_conv(&address[1]);
  uint32_t prefix = 0;
  uint8_t prefixArray[5];
  prefixArray[0] = address[0];
  prefix = addr_conv(prefixArray);
  prefix = prefix >> 24;

  // Using pipes 1-7 for reading pipes, leaving pipe0 for a tx pipe
  if (!child) {
    NRF_RADIO->BASE0 = base;
    NRF_RADIO->PREFIX0 &= ~(0xFF);
    NRF_RADIO->PREFIX0 |= prefix;
  } else if (child < 4) { // prefixes AP1-3 are in prefix0
    NRF_RADIO->BASE1 = base;
    NRF_RADIO->PREFIX0 &= ~(0xFF << (8 * child));
    NRF_RADIO->PREFIX0 |= prefix << (8 * child);
  } else {
    NRF_RADIO->BASE1 = base;
    NRF_RADIO->PREFIX1 &= ~(0xFF << (8 * (child - 4)));
    NRF_RADIO->PREFIX1 |= prefix << (8 * (child - 4));
  }
  NRF_RADIO->RXADDRESSES |= 1 << child;
  rxBase = NRF_RADIO->BASE0;
  rxPrefix = NRF_RADIO->PREFIX0;
  //    Serial.println(addrConv32(NRF_RADIO->BASE0),HEX);
  // Serial.println(addrConv32(NRF_RADIO->PREFIX0),HEX);
  // Serial.println(NRF_RADIO->RXADDRESSES);
}

/**********************************************************************************************************/

void nrf_to_nrf::openWritingPipe(const uint8_t *address) {

  uint32_t base = 0;
  uint32_t prefix = 0;

  base = addr_conv(&address[1]);
  prefix = addr_conv(&address[0]);
  prefix = prefix >> 24;

  NRF_RADIO->BASE0 = base;
  NRF_RADIO->PREFIX0 &= 0xFFFFFF00;
  NRF_RADIO->PREFIX0 |= prefix;
  NRF_RADIO->TXADDRESS = 0x00;
  txBase = NRF_RADIO->BASE0;
  txPrefix = NRF_RADIO->PREFIX0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::txStandBy() { return lastTxResult; }

/**********************************************************************************************************/

bool nrf_to_nrf::txStandBy(uint32_t timeout, bool startTx) {
  return lastTxResult;
}

/**********************************************************************************************************/

bool nrf_to_nrf::writeFast(const void *buf, uint8_t len, const bool multicast) {
  lastTxResult = write((void *)buf, len, multicast);
  return lastTxResult;
}

/**********************************************************************************************************/

bool nrf_to_nrf::acksEnabled(uint8_t pipe) {

  if (acksPerPipe[pipe]) {
    return 1;
  }
  return 0;
}

/**********************************************************************************************************/

bool nrf_to_nrf::isChipConnected() { return isValid(); }

/**********************************************************************************************************/

bool nrf_to_nrf::setDataRate(uint8_t speed) { 

  if (!speed) {
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
  } else {
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);
  }
  return 1;

}

/**********************************************************************************************************/

void nrf_to_nrf::setPALevel(uint8_t level, bool lnaEnable) {

  uint8_t paLevel = 0x00;

  if (level == 0) {
    paLevel = 0xF4;
  } else if (level == 1) {
    paLevel = 0x2;
  } else if (level == 2) {
    paLevel = 0x6;
  } else if (level == 3) {
    paLevel = 0x8;
  }
  NRF_RADIO->TXPOWER = paLevel;
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getPALevel() {

  uint8_t paLevel = NRF_RADIO->TXPOWER;

  if (paLevel == 0xF4){
    return 0;
  } else if (paLevel == 0x2) {
    return 1;
  } else if (paLevel == 0x6) {
    return 2;
  } else if (paLevel == 0x8) {
    return 3;
  } else {
    return 4;
  }
}

/**********************************************************************************************************/

uint8_t nrf_to_nrf::getARC(){
  return ARC;
}

/**********************************************************************************************************/

void nrf_to_nrf::setCRCLength(nrf_crclength_e length) {

  if (length == NRF_CRC_16) {
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two; /* CRC configuration: 16bit */
    NRF_RADIO->CRCINIT = 0xFFFFUL;            // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;           // CRC poly: x^16+x^12^x^5+1
  } else if (length == NRF_CRC_8) {
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_One; /* CRC configuration: 16bit */
    NRF_RADIO->CRCINIT = 0xFFUL;              // Initial value
    NRF_RADIO->CRCPOLY = 0x107UL;             // CRC poly: x^16+x^12^x^5+1
  } else {
    NRF_RADIO->CRCCNF = 0;       /* CRC configuration: 16bit       */
    NRF_RADIO->CRCINIT = 0x00L;  // Initial value
    NRF_RADIO->CRCPOLY = 0x00UL; // CRC poly: x^16+x^12^x^5+1
  }
}

/**********************************************************************************************************/

nrf_crclength_e nrf_to_nrf::getCRCLength() {
  if (NRF_RADIO->CRCCNF == 0) {
    return NRF_CRC_DISABLED;
  } else if (NRF_RADIO->CRCCNF == RADIO_CRCCNF_LEN_One) {
    return NRF_CRC_8;
  } else {
    return NRF_CRC_16;
  }
}

/**********************************************************************************************************/

bool nrf_to_nrf::testCarrier(){
    
  NRF_RADIO->EVENTS_RSSIEND = 0;
  NRF_RADIO->TASKS_RSSISTART = 1;
  while (!NRF_RADIO->EVENTS_RSSIEND) {}
  if (NRF_RADIO->RSSISAMPLE < 65) { return 1; }
  return 0;
    
}

/**********************************************************************************************************/

void nrf_to_nrf::powerUp(){
  NRF_RADIO->POWER = 1;
  NRF_RADIO->EVENTS_TXREADY = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (NRF_RADIO->EVENTS_TXREADY == 0);
  NRF_RADIO->EVENTS_TXREADY = 0;    
}

/**********************************************************************************************************/

void nrf_to_nrf::powerDown(){
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->POWER = 0;
}

/**********************************************************************************************************/
void nrf_to_nrf::setAddressWidth(uint8_t a_width){
  
  addressWidth = a_width;
  
  uint8_t pSize = 0;
  if (!DPL) {
    pSize = staticPayloadSize;
  }
  
  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos) |
                     (a_width - 1                  << RADIO_PCNF1_BALEN_Pos) |
                     (pSize                        << RADIO_PCNF1_STATLEN_Pos) |
                     (staticPayloadSize            << RADIO_PCNF1_MAXLEN_Pos);

}

/**********************************************************************************************************/

void nrf_to_nrf::printDetails(){
    
    Serial.println("================ Radio Configuration ================");
    Serial.print("STATUS\t\t= ");
    Serial.println(NRF_RADIO->STATE);
    
    //Serial.println(addrConv32(NRF_RADIO->PREFIX0);
    Serial.print("RX_ADDR_P0-1\t= 0x");
    uint32_t base = addrConv32(NRF_RADIO->BASE0);
    for(int i=addressWidth-2; i >-1; i--){
      Serial.print( (base >> (i * 8)) & 0xFF,HEX);
    }
    uint32_t prefixes = addrConv32(NRF_RADIO->PREFIX0);
    uint8_t prefix = (prefixes >> 24) & 0xFF;
    Serial.print(prefix,HEX);
    Serial.print(" 0x");
    base = addrConv32(NRF_RADIO->BASE1);
    for(int i=addressWidth-2; i >-1; i--){
      Serial.print( (base >> (i * 8)) & 0xFF,HEX);
    }
    prefix = (prefixes >> 16) & 0xFF;
    Serial.println(prefix,HEX);
      
    Serial.print("RX_ADDR_P2-7\t= 0x");
    prefix = (prefixes >> 8) & 0xFF;
    Serial.print(prefix,HEX);
    Serial.print(" 0x");
    prefix = (prefixes) & 0xFF;
    Serial.print(prefix,HEX);
    Serial.print(" 0x");
    prefixes = addrConv32(NRF_RADIO->PREFIX1);
    prefix = (prefixes >> 24) & 0xFF;
    Serial.print(prefix,HEX);
    Serial.print(" 0x");
    prefix = (prefixes >> 16) & 0xFF;
    Serial.print(prefix,HEX);
    Serial.print(" 0x");
    prefix = (prefixes >> 8) & 0xFF;
    Serial.print(prefix,HEX);
    Serial.print(" 0x");
    prefix = (prefixes) & 0xFF;
    Serial.println(prefix,HEX);

    uint8_t enAA = 0;
    for (int i=0; i<6; i++){
      enAA |= acksPerPipe[i] << i;
    }
    Serial.print("EN_AA\t\t= 0x");
    Serial.println(enAA,HEX);
    Serial.print("EN_RXADDR\t= 0x");
    Serial.println(NRF_RADIO->RXADDRESSES,HEX);
    Serial.print("RF_CH\t\t= 0x");
    Serial.println(NRF_RADIO->FREQUENCY,HEX);
    Serial.println("DYNPD/FEATURE\t= 0x");
    Serial.print("Data Rate\t= ");
    Serial.println(NRF_RADIO->MODE ? "2 MBPS" : "1MBPS");
    Serial.println("Model\t\t= NRF52");
    Serial.print("CRC Length\t= ");
    uint8_t crcLen = getCRCLength();
    if (crcLen == NRF_CRC_16){
      Serial.println("16 bits");
    }else
    if (crcLen == NRF_CRC_8){
      Serial.println("8 bits");
    }else{
      Serial.println("Disabled");
    }
    Serial.print("PA Power\t= ");
    uint8_t paLevel = getPALevel();
    if (paLevel == NRF_PA_MAX){
      Serial.println("PA_MAX");
    }else
    if (paLevel == NRF_PA_HIGH){
      Serial.println("PA_HIGH");
    }else
    if (paLevel == NRF_PA_LOW){
      Serial.println("PA_LOW");
    }else
    if (paLevel == NRF_PA_MIN){
      Serial.println("PA_MIN");
    }else{
      Serial.println("?");
    }
    Serial.print("ARC\t\t= ");
    Serial.println(ARC);
    
}

/**********************************************************************************************************/