

#include "nrf_to_nrf.h"

/* These are set to zero as ShockBurst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      (3UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE  (6UL)  /**< Packet length field size in bits. */

#define PACKET_BASE_ADDRESS_LENGTH  (4UL)                   //!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH        (32UL)                   //!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE      (PACKET_STATIC_LENGTH)  //!< Packet payload maximum size in bytes

/**
 * @brief Function for swapping/mirroring bits in a byte.
 *
 *@verbatim
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *@endverbatim
 *
 * @param[in] inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped/mirrored input byte.
 */
static uint32_t swap_bits(uint32_t inp);

/**
 * @brief Function for swapping bits in a 32 bit word for each byte individually.
 *
 * The bits are swapped as follows:
 * @verbatim
 * output[31:24] = input[24:31]
 * output[23:16] = input[16:23]
 * output[15:8]  = input[8:15]
 * output[7:0]   = input[0:7]
 * @endverbatim
 * @param[in] input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */
// Function to do bytewise bit-swap on an unsigned 32-bit value
static uint32_t bytewise_bit_swap(uint8_t const * p_inp)
{
    uint32_t inp = (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;

}


// Convert a base address from nRF24L format to nRF5 format
static uint32_t addr_conv(uint8_t const* p_addr)
{
    return __REV(bytewise_bit_swap(p_addr)); //lint -esym(628, __rev) -esym(526, __rev) */
}

void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
   /* NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }*/
}


nrf_to_nrf::nrf_to_nrf(){
    //Enable auto ack on all pipes by default
    for(uint8_t i=0; i<8; i++){
      acksPerPipe[i] = true;
    }
};

uint8_t bytes = 0;

bool nrf_to_nrf::begin(){

  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

  NRF_RADIO->POWER = 1;
  NRF_POWER->DCDCEN=1;
  
  NRF_RADIO->PCNF0 = 0x30006;

  NRF_RADIO->PCNF1 = 0x1040020;
  
  NRF_RADIO->BASE0 = 0xE7E7E7E7;                /* Base address 0                                   */
  NRF_RADIO->BASE1 = 0x43434343;
  NRF_RADIO->PREFIX0 = 0x23C343E7;                    /* Prefixes bytes for logical addresses 0           */
  NRF_RADIO->PREFIX1 = 0x13E363A3;
  NRF_RADIO->RXADDRESSES = 0x01;
  NRF_RADIO->TXADDRESS = 0x00; 
  /* Receive address select    */
  
  // Configure CRC for 16-bit
  NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;     /* CRC configuration: 16bit                         */
  NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
  NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
  
  NRF_RADIO->PACKETPTR = (uint32_t)radioData;
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
  NRF_RADIO->MODECNF0  = 0x200;
  NRF_RADIO->TXPOWER   = (0x8 << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->FREQUENCY = 0x4C;
  
  NRF_RADIO->TASKS_TXEN = 1;                    /* Enable RADIO in RX mode*/
  while (!(NRF_RADIO->EVENTS_READY)) {}
  NRF_RADIO->TASKS_START = 1;
  return 1;

}

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
 return (uint8_t)(val>63 ? 255 : val*ED_RSSISCALE); // Convert to IEEE 802.15.4 scale
}

uint8_t lastPacketCounter = 0;
uint8_t lastData = 0;

bool nrf_to_nrf::available(){

  if(NRF_RADIO->EVENTS_CRCOK){
    NRF_RADIO->EVENTS_CRCOK = 0;
    memcpy(&rxBuffer[1],&radioData[2],32);
    rxBuffer[0] = radioData[0];
    rxFifoAvailable = true;
    uint8_t packetCtr = radioData[1];    
    uint8_t packetData = radioData[2];
    // If ack is enabled on this receiving pipe
    if( acksEnabled(NRF_RADIO->RXMATCH) ){
      stopListening();
      delayMicroseconds(25);
      uint32_t txAddress = NRF_RADIO->TXADDRESS;
      NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;
      write(0, 0, 0); //Send an ACK
      NRF_RADIO->TXADDRESS = txAddress;
      startListening();   
    }
    
    //If the packet has the same ID number and data, it is most likely a duplicate
    if(packetCtr == lastPacketCounter && packetData == lastData){
        return 0;
    }
    lastPacketCounter = packetCtr;
    lastData = packetData;
    return 1; 
  }
  return 0;
}

void nrf_to_nrf::read(void* buf, uint8_t len){
  memcpy(buf,&rxBuffer[1],len);
  NRF_RADIO->TASKS_START = 1;
}

bool nrf_to_nrf::write(void* buf, uint8_t len, bool multicast){
  radioData[0] = len;
  radioData[1] = ((radioData[1] + 1) % 4) << 1;
  //radioData[1] |= 1;
  memset(&radioData[2],0,32);
  memcpy(&radioData[2],buf,len);
  
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_START = 1;
  while(NRF_RADIO->EVENTS_END == 0){}
  NRF_RADIO->EVENTS_END = 0;
  //startListening();
  
  
  
  
  return 1;
}

void nrf_to_nrf::startListening(){
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO-> EVENTS_RXREADY = 0;
  NRF_RADIO->EVENTS_CRCOK = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_RXREADY == 0);
  NRF_RADIO-> EVENTS_RXREADY = 0;
  NRF_RADIO->TASKS_START = 1;
}

void nrf_to_nrf::stopListening(){
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TXADDRESS = 0x00;
  NRF_RADIO-> EVENTS_TXREADY = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (NRF_RADIO->EVENTS_TXREADY == 0);
  NRF_RADIO-> EVENTS_TXREADY = 0;
  
  
}


uint8_t nrf_to_nrf::getDynamicPayloadSize(){
  uint8_t size = min(32,rxBuffer[0]);
  return size;
}

bool nrf_to_nrf::isValid(){
  return 1;
}
void nrf_to_nrf::setChannel(uint8_t channel)
{ 
  NRF_RADIO->FREQUENCY = channel;
}

void nrf_to_nrf::setAutoAck(bool enable){}
void nrf_to_nrf::setAutoAck(uint8_t pipe, bool enable){}
void nrf_to_nrf::enableDynamicPayloads(){
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (6 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos) ;

    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                               << RADIO_PCNF1_BALEN_Pos)   |
                       (0                               << RADIO_PCNF1_STATLEN_Pos) |
                       (32                              << RADIO_PCNF1_MAXLEN_Pos);
}
void nrf_to_nrf::disableDynamicPayloads(){
    NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos) |
                       (1 << RADIO_PCNF0_S1LEN_Pos) ;
                       
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                               << RADIO_PCNF1_BALEN_Pos)   |
                       (32                              << RADIO_PCNF1_STATLEN_Pos) |
                       (32                              << RADIO_PCNF1_MAXLEN_Pos);
}




void nrf_to_nrf::setRetries(uint8_t retryVar, uint8_t attempts){}
void nrf_to_nrf::openReadingPipe(uint8_t child, uint64_t address){
   
      child += 1;
      uint32_t base = address >> 8;
      uint32_t prefix = address & 0xFF;
      uint8_t baseArray[5];
      baseArray[0] = address >> 8;
      baseArray[1] = address >> 16;
      baseArray[2] = address >> 24;
      baseArray[3] = address >> 32;
      base = addr_conv(baseArray);
      
      uint8_t prefixArray[5];
      prefixArray[0] = address & 0xFF;
      prefix = addr_conv(prefixArray);
      prefix = prefix >> 24;

    if(child < 4){//prefixes AP1-3 are in prefix0 
      NRF_RADIO->BASE1 = base;
      NRF_RADIO->PREFIX0 &= ~(0xFF << (8 * child));
      NRF_RADIO->PREFIX0 |= prefix << (8 * child);   
    }else{
      NRF_RADIO->BASE1 = base;
      NRF_RADIO->PREFIX1 &= ~(0xFF << (8 * (child - 4)));
      NRF_RADIO->PREFIX1 |= prefix << (8 * (child - 4));  
    }
    NRF_RADIO->RXADDRESSES |= 1 << child;
    
}
void nrf_to_nrf::openWritingPipe(uint64_t address){
      uint32_t base = address >> 8;
      uint32_t prefix = address & 0xFF;
      uint8_t baseArray[5];
      baseArray[0] = address >> 8;
      baseArray[1] = address >> 16;
      baseArray[2] = address >> 24;
      baseArray[3] = address >> 32;
      base = addr_conv(baseArray);
      
      uint8_t prefixArray[5];
      prefixArray[0] = address & 0xFF;
      prefix = addr_conv(prefixArray);
      prefix = prefix >> 24;
      
    NRF_RADIO->BASE0 = base;
    NRF_RADIO->PREFIX0 &= 0xFFFFFF00;
    NRF_RADIO->PREFIX0 |= prefix;    
    NRF_RADIO->TXADDRESS = 0x00;    
}

void nrf_to_nrf::openReadingPipe(uint8_t child, const uint8_t* address){
    
    child +=1;
    
    uint32_t base = addr_conv(&address[1]);
    uint32_t prefix = 0;
    uint8_t prefixArray[5];
    prefixArray[0] = address[0];
    prefix = addr_conv(prefixArray);
    prefix = prefix >> 24;
    
   
    // Using pipes 1-7 for reading pipes, leaving pipe0 for a tx pipe
    if(child < 4){//prefixes AP1-3 are in prefix0 
      NRF_RADIO->BASE1 = base;
      NRF_RADIO->PREFIX0 &= ~(0xFF << (8 * child));
      NRF_RADIO->PREFIX0 |= prefix << (8 * child);   
    }else{
      NRF_RADIO->BASE1 = base;
      NRF_RADIO->PREFIX1 &= ~(0xFF << (8 * (child - 4)));
      NRF_RADIO->PREFIX1 |= prefix << (8 * (child - 4));  
    }
    NRF_RADIO->RXADDRESSES |= 1 << child;

    
}

void nrf_to_nrf::openWritingPipe(const uint8_t* address){
    
    uint32_t base = 0;
    uint32_t prefix = 0;

    base = addr_conv(&address[1]);
    prefix = addr_conv(&address[0]);
    prefix = prefix >> 24;
      
    NRF_RADIO->BASE0 = base;
    NRF_RADIO->PREFIX0 &= 0xFFFFFF00;
    NRF_RADIO->PREFIX0 |= prefix;    
    NRF_RADIO->TXADDRESS = 0x00;
}

bool nrf_to_nrf::txStandBy(){return 1;}
bool nrf_to_nrf::txStandBy(uint32_t timeout, bool startTx){ return 1;}
bool nrf_to_nrf::writeFast(const void* buf, uint8_t len, const bool multicast){
    return write(&buf,len,multicast);
}

bool nrf_to_nrf::acksEnabled(uint8_t pipe){
    
    if(acksPerPipe[pipe]){
        return 1;
    }
    return 0;
}