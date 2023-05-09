# nrf_to_nrf
 NRF52840 to NRF24L01 communication library for Arduino
 
 To get the gettingstarted example working with NRF24l01+ the RF24 gettingstarted example must be modified to use dynamic payloads
 
 Notes:
 1. There is only a single layer buffer instead of a 3-layer FIFO like the NRF24L01 - 12/4/2022
 2. The enums like `RF24_PA_MAX` are now `NRF_PA_MAX` etc.
 3. To modify RF24 examples to work with this library, just change the following:
     - `#include <nrf_to_nrf.h>` instead of RF24.h
     - Call `nrf_to_nrf radio;` instead of `RF24 radio(7,8);`
     - Modify the enums per note 2
     
The higher layer libs like RF24Network require additional changes, which will be forthcoming soon...

Please log an issue for problems with any of the provided examples.
