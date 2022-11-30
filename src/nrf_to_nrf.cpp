

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
static uint32_t bytewise_bitswap(uint32_t inp);


static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
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


nrf_to_nrf::nrf_to_nrf(){};

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
  NRF_RADIO->TXPOWER   = (0x8 << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->SHORTS = 0;
  
  NRF_RADIO->TASKS_RXEN = 1;                    /* Enable RADIO in RX mode*/
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
    uint8_t packetCtr = radioData[1];
    uint8_t packetData = radioData[2];
    //If the packet has the same ID number and data, it is most likely a duplicate
    if(packetCtr == lastPacketCounter && packetData == lastData){
        //Serial.println("drop packet");
        NRF_RADIO->TASKS_RXEN = 1;
        NRF_RADIO->TASKS_START = 1;
        while (!(NRF_RADIO->EVENTS_READY)) {}
        return 0;
    }
    lastPacketCounter = packetCtr;
    lastData = packetData;
    return 1; 
  }
  return 0;
}

void nrf_to_nrf::read(void* buf, uint8_t len){
  memcpy(buf,&radioData[2],len);
  NRF_RADIO->TASKS_RXEN = 1;
  NRF_RADIO->TASKS_START = 1;
  while (!(NRF_RADIO->EVENTS_READY)) {}
}

bool nrf_to_nrf::write(void* buf, uint8_t len, bool multicast){
  radioData[0] = len;
  radioData[1] = 1;
  memcpy(&radioData[2],buf,len);
  NRF_RADIO->EVENTS_PAYLOAD = 0;
  NRF_RADIO->TASKS_START = 1;
  while(NRF_RADIO->EVENTS_PAYLOAD == 0){}
  NRF_RADIO->EVENTS_PAYLOAD = 0;
  return 1;
}

void nrf_to_nrf::startListening(){
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO-> EVENTS_RXREADY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_RXREADY == 0);
  NRF_RADIO-> EVENTS_RXREADY = 0;
  NRF_RADIO->EVENTS_CRCOK = 0;
  NRF_RADIO->TASKS_START = 1;
}

void nrf_to_nrf::stopListening(){
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO-> EVENTS_TXREADY = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (NRF_RADIO->EVENTS_TXREADY == 0);
  NRF_RADIO-> EVENTS_TXREADY = 0;
}


uint8_t nrf_to_nrf::getDynamicPayloadSize(){
  uint8_t size = radioData[0];
  return size;
}

bool nrf_to_nrf::isValid(){
  return 1;
}
void nrf_to_nrf::setChannel(uint8_t channel)
{ 

  Serial.print("Freq: ");
  Serial.println(2400 + NRF_RADIO->FREQUENCY);
  NRF_RADIO->FREQUENCY = channel;
  Serial.print("NewFreq: ");
  Serial.println(2400 + NRF_RADIO->FREQUENCY);
  
}

void nrf_to_nrf::setAutoAck(bool enable){}
void nrf_to_nrf::setAutoAck(uint8_t pipe, bool enable){}
void nrf_to_nrf::enableDynamicPayloads(){}
void nrf_to_nrf::setRetries(uint8_t retryVar, uint8_t attempts){}
void nrf_to_nrf::openReadingPipe(uint8_t child, uint64_t address){}
void nrf_to_nrf::openWritingPipe(uint64_t address){}
bool nrf_to_nrf::txStandBy(){return 1;}
bool nrf_to_nrf::txStandBy(uint32_t timeout, bool startTx){ return 1;}
bool nrf_to_nrf::writeFast(const void* buf, uint8_t len, const bool multicast){
    return write(&buf,len,multicast);
}

