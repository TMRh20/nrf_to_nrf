
/**
 * @file nrf52840_nrf24l01
 *
 * Class declaration for nrf52840_nrf24l01
 */
#ifndef __nrf52840_nrf24l01_H__
#define __nrf52840_nrf24l01_H__
#include <Arduino.h>


class nrf_to_nrf
{

public:

nrf_to_nrf();

uint8_t radioData[32 + 2];

bool begin();
uint8_t sample_ed(void);
bool available();
void read(void* buf, uint8_t len);
bool write(void* buf, uint8_t len, bool multicast = false);
bool writeFast(const void* buf, uint8_t len, const bool multicast =0);
void startListening();
void stopListening();
uint8_t getDynamicPayloadSize();
bool isValid();
void setChannel(uint8_t channel);
void setAutoAck(bool enable);
void setAutoAck(uint8_t pipe, bool enable);
void enableDynamicPayloads();
void disableDynamicPayloads();
void setRetries(uint8_t retryVar, uint8_t attempts);
void openReadingPipe(uint8_t child, uint64_t address);
void openWritingPipe(uint64_t address);
bool failureDetected;
bool txStandBy();
bool txStandBy(uint32_t timeout, bool startTx = 0);

};

#endif //__nrf52840_nrf24l01_H__
