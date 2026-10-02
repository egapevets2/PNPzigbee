## 9. Hardware-in-the-Loop Test Fixture: TXRXproto

### 9.1 Overview & System Role

The **`TXRXproto`** project (located in `C:\Users\egape\TXRXproto`) serves as the dedicated **Hardware-in-the-Loop (HIL) verification fixture and downstream client** for the Zigbee mesh network. 

Physically wired to the ESP32-C6 node's hardware UART serial bridge, `TXRXproto` provides immediate visual and serial feedback confirming that packets sent from the remote coordinator successfully traverse the wireless Zigbee mesh, get dispatched across the node's serial bridge, and arrive uncorrupted at the end-actuator.

```text
┌─────────────────────────────────┐
│     PuTTY / Host PC Terminal     │
└───────────────┬─────────────────┘
                │ Serial Console ("Kitchen blinkx 5")
                ▼
┌─────────────────────────────────┐
│    Coordinator (ESP32-C6)       │
└───────────────┬─────────────────┘
                │ Zigbee 3.0 Mesh (IEEE 802.15.4 / APS 0xFFC0)
                ▼
┌─────────────────────────────────┐
│       PNPzigbee (ESP32-C6)      │
│     serial_bridge (UART1)       │
└───────────────┬─────────────────┘
                │ Hardware Serial (115200 Baud / 8-N-1)
                ▼
┌─────────────────────────────────┐
│     TXRXproto (SAMD21 / M0)     │
│   • Built-in LED Heartbeat      │
│   • DotStar Blue 'blinkx' LED   │
│   • Bi-directional USB Bridge   │
└─────────────────────────────────┘
```

---

### 9.2 Hardware & Platform Specifications

| Parameter | Specification | Notes |
| :--- | :--- | :--- |
| **Microcontroller** | Microchip / Atmel ATSAMD21E18A | 32-bit ARM Cortex-M0+ @ 48 MHz |
| **Board Profile** | Adafruit Trinket M0 / ItsyBitsy M0 Express | `[env:adafruit_trinket_m0]` |
| **Framework** | Arduino via PlatformIO (`atmelsam`) | Embedded C++ |
| **Primary Libraries** | `Adafruit DotStar`, `SPI` | Fast hardware SPI / bit-bang pixel driver |
| **Operating Voltage** | 3.3V Logic | Direct logic-level compatibility with ESP32-C6 |
| **Interface Ports** | Dual Serial (USB CDC + Hardware UART1) | Concurrent monitoring and bridge routing |

---

### 9.3 Serial Architecture & Transparent Bridging

`TXRXproto` manages two independent serial interfaces concurrently within a non-blocking `loop()` cycle:

1. **Host USB Console (`Serial` @ 9600 Baud):**
   - Connects to the host PC via native USB CDC.
   - Provides an interactive terminal for developers using PuTTY or the PlatformIO Serial Monitor.
   - Any character entered by the user is directly forwarded to the ESP32-C6 via `Serial1.write()`.

2. **Inter-Board Hardware Link (`Serial1` @ 115200 Baud):**
   - Wired directly to the ESP32-C6 UART pins managed by `serial_bridge.c`.
   - Inbound characters from the ESP32-C6 are immediately mirrored back to the host USB console (`Serial.write()`) to preserve transparent bridge visibility.
   - Concurrently, incoming characters are buffered into a 96-byte line accumulator (`processEsp32Byte`) to detect and execute local fixture commands.

### 9.4 Coordinator Command Reference

All commands are submitted through the Coordinator terminal console (PuTTY or Python test runner @ 115200 baud).

#### 1. Network Management Commands (No Target Prefix)
| Command | Parameters | Description |
| :--- | :--- | :--- |
| **`GiveNetworkReport`** | *none* | Initiates live RF ping survey to all online nodes; outputs formatted table with IEEE, short address, local/remote LQI, and RF health status. |
| **`PingNetwork`** | *none* | Broadcasts `PING` to all known devices to evaluate link latency. |
| **`ResetNetwork`** | *none* | Re-opens Zigbee 3.0 permit-joining for 180 seconds (`esp_zb_bdb_open_network(180)`) and starts active rediscovery for known devices. |
| **`ResetCoordinator`** | *none* | Restarts the Coordinator hardware (`esp_restart()`). |

#### 2. Node & Bridge Commands (Prefixed with `<TargetName>`)
| Command | Parameters | Description |
| :--- | :--- | :--- |
| **`<TargetName> PING`** | *none* | Direct node RF ping. Target replies with `< <TargetName>: PONG LQI=<val>`. |
| **`<TargetName> SetupSerialBridge`** | *none* | **Required prerequisite:** Activates UART1 (`GPIO 17 TX` / `GPIO 16 RX`) on the node to enable downstream routing to the Arduino. |
| **`<TargetName> blink [count] [ms]`** | `count`, `ms` | Blinks the ESP32-C6 node's onboard LED locally. |

---

### 9.5 Downstream Fixture Commands (Wired Arduino / `TXRXproto`)

When the Zigbee coordinator sends a text command targeted at the node, the ESP32-C6 serial bridge emits the raw string across UART. `TXRXproto` parses incoming lines and services three dedicated verification commands:

#### 1. Two-Way Serial Round-Trip (`ping` → `GotPing`)
* **Inbound Command:** `<TargetName> ping` (or `ping`, case-insensitive).
* **Execution & Response:**
  * Arduino transmits `GotPing\r\n` back out hardware `Serial1`.
  * The ESP32-C6 `serial_bridge` picks up `GotPing` and transmits it via Zigbee APS message back to the Coordinator.
  * The Coordinator prints `< <TargetName>: GotPing` to the PC host / test runner.
* **Verification Significance:** Confirms 100% two-way communication end-to-end:
  `Host (PC) → Coordinator → Zigbee Over-The-Air → ESP32-C6 Node → UART1 → Arduino → UART1 → ESP32-C6 Node → Zigbee Over-The-Air → Coordinator → Host (PC)`.

#### 2. Visual Actuation (`blinkx <Count>`)
* **Inbound Command:** `<TargetName> blinkx <Count>` (e.g., `Kitchen blinkx 7` or `Garage blinkx 3`).
* **Parser Logic:** Discards the routing prefix `<TargetName>`, bounds-checks `<Count>` (0–50, defaults to 1), and triggers the non-blocking DotStar RGB LED state machine `startBlinkx(count)`.
* **Execution:** Emits confirmation upstream: `[Arduino] blinkx <value>`.

#### 3. Hardware DAC Voltage Generation (`setDAC <Value>`)
* **Inbound Command:** `<TargetName> setDAC <Value>` (e.g., `Kitchen setDAC 512` or `setDAC 1023`).
* **Hardware Output:** Generates true analog DC voltage on Trinket M0 pin **`A0`** via the internal SAMD21 10-bit DAC (`analogWriteResolution(10)`).
  * Range: `0` (0.0V) to `1023` (3.3V). Formula: $V_{out} = \frac{\text{Value}}{1023} \times 3.3\,\text{V}$.
* **Execution & Response:**
  * Arduino sets voltage on pin `A0`.
  * Transmits `GotDAC <Value>\r\n` back across `Serial1` to the ESP32-C6.
  * The Coordinator outputs `< <TargetName>: GotDAC <Value>`.
* **HIL ADC Testing Application:** By wiring Trinket M0 pin **`A0`** to an ESP32-C6 ADC input (e.g., `GPIO2 / A0`), the host test suite can command arbitrary reference voltages, read back the ADC measurement via Zigbee (`ReadADCA0`), and automatically verify the ADC calibration of the ESP32-C6 over the air.

---

### 9.5 Visual Status Indicators

`TXRXproto` employs two independent, non-blocking visual indicators:

#### 1. Liveness Heartbeat (`LED_BUILTIN` - Red LED)
* **Behavior:** A non-blocking asynchronous timing pattern executed every cycle:
  * 1000 ms ON &rarr; 1000 ms OFF &rarr; Three rapid 100 ms strobe pulses.
* **Purpose:** Proves that the SAMD21 scheduler is active, non-starved, and interrupts are functioning normally.

#### 2. Zigbee Verification Indicator (DotStar Addressable RGB LED)
* **Hardware:** Onboard APA102 / DotStar RGB LED driven via `PIN_DOTSTAR_DATA` (D41) and `PIN_DOTSTAR_CLOCK` (D40).
* **Color:** **Vibrant Blue** (`0x0000FF`) at 40/255 brightness.
* **Cadence:** 200 ms ON / 200 ms OFF per pulse.
* **Non-Blocking Operation:** `serviceBlinkx()` runs strictly on `millis()` timestamps, ensuring serial bridge throughput is never delayed or interrupted during long multi-blink sequences.
