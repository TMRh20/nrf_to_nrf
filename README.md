# nrf_to_nrf
 NRF52840 to NRF24L01 communication library for Arduino
 
 To get the example working with NRF24l01+ the gettingstarted example must be modified as follows:

1. Call radio.setChannel(7);
2. Call radio.enableDynamicPayloads();
3. Comment out the lines for openReadingPipe and openWritingPipe (it uses the default addresses)