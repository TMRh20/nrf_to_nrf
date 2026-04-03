# nrf_to_nrf

**nRF52x & nRF54x Enhanced ShockBurst (ESB) driver with an RF24-compatible API** (Arduino / PlatformIO).

This library targets nRF52 & nRF54 devices (ex: nRF52840 or nRF54l15) and is designed to feel familiar to users of the `RF24` library (similar function names and workflows).

## Key points / Differences from nRF24L01
1. There is only a **single-layer buffer** (not a 3-layer FIFO like the nRF24L01).
2. Enums like `RF24_PA_MAX` become `NRF_PA_MAX` (etc).
3. Porting RF24 sketches:
   - use `#include <nrf_to_nrf.h>` instead of `RF24.h`
   - create the radio as `nrf_to_nrf radio;` (no CE/CS pins like RF24 modules)
   - update enums (note #2)
   - higher layer libs use the RF52 prefix: `RF52Network`, `RF52Mesh`, etc.

> The higher layer libraries (RF24Network/RF24Mesh/RF24Ethernet) have been updated (v2.x) to accommodate this driver.

---

## Supported environments
- Arduino IDE
- PlatformIO (`framework = arduino`)

## Installation

### Arduino IDE (Library Manager)
Arduino IDE → **Library Manager** → search for `nrf_to_nrf` → **Install**

### PlatformIO
Add to `platformio.ini`:

```ini
[env:nrf52]
platform = nordicnrf52
board = <your_board_id>
framework = arduino
monitor_speed = 115200

lib_deps =
  https://github.com/TMRh20/nrf_to_nrf.git
```

---

## Examples (Receiver / Transmitter)

Use the provided examples:

- RF24-style:
  - Receiver (RX): `examples/RF24/GettingStarted/GettingStarted.ino`
  - Transmitter (TX): `examples/RF24/GettingStarted/GettingStarted.ino` (same sketch; select role via Serial)

- RF24Network “Hello World”:
  - Receiver (RX): `examples/RF24Network/helloworld_rx/helloworld_rx.ino`
  - Transmitter (TX): `examples/RF24Network/helloworld_tx/helloworld_tx.ino`

- Encryption examples:
  - RF24Network RX encryption: `examples/RF24Network/helloworld_rxEncryption/helloworld_rxEncryption.ino`
  - RF24Network TX encryption: `examples/RF24Network/helloworld_txEncryption/helloworld_txEncryption.ino`
  - RF24 GettingStarted encryption: `examples/RF24/GettingStartedEncryption/GettingStartedEncryption.ino`

> Address note (RF24-compatible): for 5-byte addresses, the **first byte is the identifier/prefix byte** (eg: `{'1','N','O','D','E'}`).

---

## Encryption (optional)
There are examples under `examples/` showing encryption usage (setting a 16-byte key, enabling encryption, and increasing max payload sizes):

- `radio.setKey(myKey);`
- `radio.enableEncryption = true;`
- `radio.enableDynamicPayloads(123);` (important so encryption overhead doesn’t reduce usable payload)

---

## Troubleshooting
- **No RX packets:** confirm both sides use the same channel and addresses; start RX with `startListening()` and TX with `stopListening()`.
- **Short/garbled messages:** ensure you’re reading/writing the same payload length; consider enabling dynamic payloads if your lengths vary.
