# nrf_to_nrf
 NRF52840 to NRF24L01 communication library for Arduino
 
 Notes:
 1. There is only a single layer buffer instead of a 3-layer FIFO like the NRF24L01
 2. The enums like `RF24_PA_MAX` are now `NRF_PA_MAX` etc.
 3. To modify RF24 examples to work with this library, just change the following:
     - `#include <nrf_to_nrf.h>` instead of RF24.h
     - Call `nrf_to_nrf radio;` instead of `RF24 radio(7,8);`
     - Modify the enums per note 2
     
The higher layer libs like RF24Network have been updated to version 2.0 to accommodate this library.

The examples work for communication between NRF52840 and NRF24L01 out of the box.

Please log an issue for problems with any of the provided examples.
