# PNPzigbee Firmware: Architecture, Features & Functional Specification
**Target Platform:** Espressif ESP32-C6 (32-bit RISC-V @ 160MHz)  
**Framework:** ESP-IDF v6.0 | FreeRTOS | ESP-Zigbee-SDK (ZBOSS 1.6.x)  
**Role:** Zigbee 3.0 End Device / Router with Distributed Multi-Peripheral Control

---

## 1. Executive Summary

The **PNPzigbee** firmware turns an ESP32-C6 system-on-chip into a versatile, mesh-networked actuator and sensor controller. Communicating via standard **Zigbee 3.0** (IEEE 802.15.4) and a local USB/UART console, it bridges remote mesh coordinator commands to a wide array of physical hardware interfaces:
- **Motion & Motors:** DC motors (DRV8833 & Cytron 10C), Brushless DC (ESC-driven), 4-phase Steppers (Full/Half/Microstep with current-limiting PWM), and 4 independent hardware PWM channels.
- **Sensors:** Time-of-Flight distance (VL53L1X), optical proximity/ambient light (APDS-9930), 12-bit ADC channels with software hysteresis comparators, and dynamic GPIO monitoring.
- **Visuals & Bridging:** Multi-pixel WS2812 addressable RGB LEDs (NeoPixels), onboard indicator LED, and a full-duplex hardware UART serial bridge.
- **Resilience:** Centralized hardware timer management (LEDC arbiter), upstream message rate-limiting (`UpstreamQ`), and dual command routing (direct command or targeted addressing).

---

## 2. System Architecture

### 2.1 High-Level Architecture Diagram

```mermaid
flowchart TD
    subgraph Mesh_and_Host ["Communication Interfaces"]
        ZC["Zigbee Coordinator (Remote)"] <--> |IEEE 802.15.4 / APS| ZB_STACK["ESP Zigbee Stack (ZBOSS)"]
        USB["USB Serial Console"] <--> |Interactive CLI| MS["mesh_serial"]
        UART_DEV["External MCU / Device"] <--> |UART1 115200 Baud| SB["serial_bridge"]
    end

    subgraph Core_Dispatch ["Core Dispatch & Processing"]
        ZB_STACK --> |APS Indication 0xFFC0 / ZCL Attr| APP_MSG["app_msg_task (Message Router)"]
        MS --> |User Input| ZB_STACK
        SB <--> |Bi-directional Tunnel| ZB_STACK
        APP_MSG --> PARSER["parse_text_for_local_action"]
        UQ["UpstreamQ (200ms Rate Limiter)"] --> |APS Data Request| ZB_STACK
    end

    subgraph Resource_Managers ["Hardware Resource Arbiters"]
        LEDC_MGR["LEDC Resource Manager (6 Ch / 4 Timers)"]
        I2C_BUS["Shared I2C Master Bus Handle (400kHz)"]
        ADC_UNIT["ADC1 Oneshot Unit"]
    end

    subgraph Actuator_Subsystems ["Motion & Output Subsystems"]
        PARSER --> PWM["pwm_driver (4 Ch + Slew)"]
        PARSER --> MOTOR["motor_driver (DRV8833 / Cytron)"]
        PARSER --> BLDC["BLDC_driver (50Hz ESC + Arm)"]
        PARSER --> STEPPER["Stepper (Full/Half/Micro + Hold)"]
        PARSER --> NEO["NeoPixel (WS2812 RMT/SPI)"]
        PARSER --> GPIO_OUT["GPIO_handler (Digital Out)"]
        PARSER --> LED["light_driver (Status LED)"]
    end

    subgraph Sensor_Subsystems ["Sensor & Telemetry Subsystems"]
        PARSER --> VL53["VL53L1X ToF Distance"]
        PARSER --> APDS["APDS-9930 Proximity"]
        PARSER --> ADC_DRV["ADC (A0, A1, A2 + Hysteresis)"]
        PARSER --> GPIO_IN["GPIO_handler (Input Change Detect)"]
        PARSER --> I2C_RAW["I2Craw (Direct Register Access)"]
    end

    PWM --> LEDC_MGR
    MOTOR --> LEDC_MGR
    BLDC --> LEDC_MGR
    STEPPER --> LEDC_MGR

    VL53 --> I2C_BUS
    APDS --> I2C_BUS
    I2C_RAW --> I2C_BUS

    ADC_DRV --> ADC_UNIT

    VL53 -.-> |Threshold / Stream| UQ
    APDS -.-> |Threshold / Stream| UQ
    ADC_DRV -.-> |State Change Event| UQ
    GPIO_IN -.-> |Pin Change Event| UQ
```

---

## 3. Communication & Mesh Protocol

### 3.1 Zigbee 3.0 Implementation
- **Profile:** Home Automation (`ESP_ZB_AF_HA_PROFILE_ID`, 0x0104)
- **Device Type:** On/Off Light Bulb (`HA_ESP_LIGHT_ENDPOINT = 10`)
- **Network Role:** Zigbee End Device (`ESP_ZB_DEVICE_TYPE_ED`) or Router (`ESP_ZB_DEVICE_TYPE_ROUTER`), selectable via build flags.
- **Custom Application Cluster (`0xFFC0`):**
  Used for bidirectional ASCII payload streaming and formatted binary packet transmission (`AppMessage` struct with `cmd[18]` and `int16_t value`).
- **Standard ZCL Attribute Handling:**
  Listens for `ESP_ZB_ZCL_CLUSTER_ID_ON_OFF` `ON_OFF` attribute updates to maintain ecosystem compatibility with standard Zigbee coordinators (Home Assistant, Zigbee2MQTT, etc.).

### 3.2 Command Grammar & Routing
Incoming text packets are parsed by `parse_text_for_local_action()` using a dual-mode grammar:

1. **Direct Mode (Unicast / Broadcast to self):**
   ```text
   <Command> [Value1] [Value2]
   Example: MotorSpeed 500
   Example: pwm1 800
   Example: NeoRED 0 255
   ```
2. **Targeted Mode (Sub-device / Multi-node addressing):**
   ```text
   <TargetNode> <Command> [Value1] [Value2]
   Example: NODE_B MotorSpeed 500
   ```
   If `<TargetNode>` does not match a known local command, the router checks if the second token is a recognized command.

### 3.3 Upstream Throttling (`UpstreamQ`)
To prevent saturation of Zigbee transmission buffers and APS acknowledgment queues, all asynchronous event messages (sensor threshold triggers, digital state changes, telemetry streams) pass through `SendTheMessage()`.
- **Queue Depth:** 10 messages of up to 32 bytes each.
- **Throttling Task:** Dedicated FreeRTOS task enforcing a minimum `200ms` transmission window between outgoing mesh packets.

---

## 4. Hardware Resource Management

### 4.1 LEDC Peripheral Arbiter (`ledc_manager`)
The ESP32-C6 features **6 LEDC PWM channels** and **4 hardware timers** in low-speed mode. Multiple drivers (`pwm_driver`, `motor_driver`, `BLDC_driver`, `Stepper`) require PWM generation. The `ledc_manager` prevents peripheral collisions:
- **Timer Sharing:** When a driver requests a timer (e.g., `ledc_manager_alloc_timer(freq_hz, &timer)`), the arbiter first checks if an active timer is already running at that exact frequency. If so, it shares that timer index, saving scarce timer resources.
- **Channel Allocation:** Channels 0 through 5 are allocated dynamically on a first-come, first-served basis with tracking of in-use states.

### 4.2 Shared I2C Bus Architecture
Sensors (`VL53L1X`, `APDS-9930`) and the raw I2C tool (`I2Craw`) utilize the new ESP-IDF v6.0 unified `i2c_master` driver:
- **Bus Speed:** 400 kHz Fast-Mode.
- **Bus Ownership:** Initialized once via `get_shared_i2c_bus()`. Multiple device handles are attached to the same bus handle, preventing bus re-initialization crashes.

---

## 5. Subsystem Detailed Specifications

### 5.1 Motor & Actuator Drivers

#### 1. DC Motor Driver (`motor_driver`)
Supports bidirectional brushed DC motor control with speed ramp slew rate limiting:
- **Operating Modes:**
  - `Mode 0 (DRV8833)`: Dual-PWM H-Bridge control using 2 LEDC channels.
  - `Mode 1 (Cytron 10C)`: Single PWM speed channel + standard GPIO directional pin.
- **Speed Range:** `-1000` (full reverse) to `+1000` (full forward), `0` = dynamic brake/stop.
- **Slew Rate Generator:** FreeRTOS task updates motor output every 20ms according to configurable slew units per second.

#### 2. Brushless DC Motor Driver (`BLDC_driver`)
Controls standard 3-phase BLDC motors via an electronic speed controller (ESC):
- **PWM Waveform:** Standard RC servo signal (50Hz / 20ms period).
- **Pulse Range:** 1000 µs (full reverse) $\rightarrow$ 1500 µs (neutral/stop) $\rightarrow$ 2000 µs (full forward).
- **Safety Arming Sequence:** `BLDCarm` executes a blocking 2-second neutral signal sequence before allowing non-zero throttle.

#### 3. 4-Phase Stepper Controller (`Stepper`)
Direct four-pin phase control for unipolar or bipolar stepper motors:
- **Modes:** Full-Step, Half-Step, and Micro-Step lookup tables.
- **Holding Current:** `HoldPWM` sets a reduced PWM duty cycle when the motor is stationary to prevent overheating while maintaining holding torque.
- **Motion Profiling:** Configurable acceleration and deceleration step profiles with programmable step delay (`msPerStep`).

#### 4. Multi-Channel PWM Driver (`pwm_driver`)
Provides 4 general-purpose PWM channels (1–4):
- **Resolution:** 10-bit hardware resolution (0 to 1023 duty cycle).
- **Dynamic Configuration:** Per-channel frequency, GPIO assignment, and linear slew rate limiting.

#### 5. Addressable RGB Strip (`NeoPixel`)
- **Driver:** ESP-IDF `led_strip` component (RMT / SPI backend).
- **RAM Buffer:** Internal RGB frame buffer supporting indexed color updates (`NeoRED`, `NeoGREEN`, `NeoBLUE`) before atomically committing to hardware with `NeoUpdate`.

---

### 5.2 Sensor & Telemetry Subsystems

#### 1. Time-of-Flight Distance (`VL53L1X`)
- **Range:** Millimeter-accurate optical distance measurement up to 4 meters.
- **Hysteresis Alerting:** Monitors `VLUpperThresh` and `VLLowerThresh`. Triggers an upstream Zigbee notification when an object enters or leaves the defined window.
- **Streaming:** Optional real-time distance streaming (`StreamVL 1`).

#### 2. Proximity & Light (`APDS-9930`)
- **Sensing:** Optical infrared proximity with configurable pulse count (`Npulses`).
- **Hysteresis Comparator:** Evaluates upper and lower proximity thresholds; reports state transitions to the coordinator.

#### 3. 12-Bit Analog Input (`adc`)
- **Channels:** `A0` (GPIO 2), `A1` (GPIO 3), `A2` (GPIO 10).
- **Comparator Engine:** Software Schmitt-trigger comparator. When the raw analog voltage surpasses `UpperThresh`, comparator sets high; when it drops below `LowerThresh`, comparator clears. State transitions automatically enqueue an upstream event.

#### 4. Dynamic Digital I/O (`GPIO_handler`)
- **Features:** Dynamic pin assignment as inputs or outputs.
- **Background Polling:** A 50ms polling task tracks all configured input pins. Any state change instantly generates an upstream Zigbee report: `GPIO <pin> CHANGED TO <0|1>`.

---

### 5.3 Communication Bridges & Consoles

#### 1. Hardware UART Bridge (`serial_bridge`)
- **Peripheral:** `UART_NUM_1` (TX: GPIO 17, RX: GPIO 16 @ 115200 baud).
- **Operation:** Completely transparent bidirectional link between the Zigbee mesh and a connected microcontroller (e.g., Arduino, host PC, or sensor board).

#### 2. USB Serial Console (`mesh_serial`)
- **Peripheral:** ESP32-C6 USB-Serial/JTAG controller.
- **Interactive Shell:** Local terminal interface with line editing, backspace handling, command echoing, and direct message forwarding to the mesh coordinator.

---

## 6. FreeRTOS Task Hierarchy

| Task Name | Core / Pri | Stack Size | Purpose |
| :--- | :--- | :--- | :--- |
| `esp_zb_task` | Core 0 / P5 | 4096 B | ZBOSS Zigbee 3.0 protocol stack & commissioning event loop |
| `app_msg_task` | Core 0 / P5 | 3072 B | Central message dispatcher; parses commands from mesh/serial |
| `serial_console_task`| Core 0 / P5 | 3072 B | Interactive USB console reader and line editor |
| `button_task` | Core 0 / P5 | 2048 B | 20ms debounced monitor for physical button (GPIO 9) |
| `throttle_task` | Core 0 / P5 | 2048 B | Upstream message queue consumer (200ms rate limiter) |
| `gpio_poll_task` | Core 0 / P5 | 2048 B | 50ms periodic digital pin state change scanner |
| `motor_slew_task` | Core 0 / P5 | 2048 B | 20ms motor speed ramp generator |
| `pwm_slew_task` | Core 0 / P5 | 2048 B | 20ms multi-channel PWM duty cycle ramp generator |
| `vl53_task` | Core 0 / P5 | 3072 B | 50ms ToF distance reading, filtering, and threshold check |
| `prox_task` | Core 0 / P5 | 3072 B | 20ms optical proximity reading and threshold evaluation |
| `adc_task` | Core 0 / P5 | 2048 B | Periodic round-robin analog sampling and hysteresis evaluation |
| `serial_bridge_rx_task` | Core 0 / P5 | 2048 B | Hardware UART1 character receiver and line buffer |

---

## 7. Complete Command Reference Dictionary

All commands can be sent as raw Zigbee APS text payloads, ZCL strings, or typed in the USB serial console.

### 7.1 System & Indicators
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `PING` | *none* | Test mesh connectivity and measure link quality | `PONG LQI=<val>` |
| `blink` | `<count> [ms]` | Blink onboard status LED `count` times with `ms` period | `OK` |
| `on` | *none* | Turn standard light state ON | `GOT ON OK` |
| `off` | *none* | Turn standard light state OFF | `GOT OFF OK` |
| `SetupSerialBridge` | *none* | Initialize hardware UART1 serial bridge on GPIO 16/17 | `SETUP SERIAL OK` |

### 7.2 PWM Subsystem
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `SetupPWM<1-4>` | *none* | Allocate hardware timer/channel for PWM channel 1–4 | `SETUP PWM<n> OK` |
| `pwmPin<1-4>` | `<gpio>` | Assign output GPIO pin to PWM channel | `GOT PWMPIN<n> OK` |
| `pwmFreq<1-4>` | `<freq_hz>` | Set frequency in Hz for PWM channel | `GOT PWMFREQ<n> OK` |
| `pwmSlew<1-4>` | `<units/sec>` | Set linear duty ramp rate (`0` = instantaneous) | `GOT PWMSLEW<n> OK` |
| `pwm<1-4>` | `<duty>` | Set target 10-bit duty cycle (`0` to `1023`) | `GOT PWM<n> OK` |

### 7.3 DC & BLDC Motors
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `SetupMotorDriver` | `<mode> <freq>` | Init DC motor (`0`=DRV8833, `1`=Cytron 10C) at `freq` Hz | `GOT MOTORSPEED OK` |
| `MotorSpeed` | `<speed>` | Set motor speed and direction (`-1000` to `+1000`) | `GOT MOTORSPEED OK` |
| `MotorSlew` | `<rate>` | Set speed ramp rate (speed units / second) | `GOT MOTORSLEW OK` |
| `SetupBLDC` | *none* | Initialize 50Hz ESC PWM channel | `SETUP BLDC OK` |
| `BLDCarm` | *none* | Execute 2-second ESC neutral arming sequence | `BLDC ESC Armed` |
| `BLDCspeed` | `<speed>` | Set ESC throttle (`-1000` to `+1000`, `0`=neutral) | `GOT BLDCSPEED OK` |

### 7.4 Stepper Motor
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `SetupStepper` | *none* | Initialize stepper driver subsystem | `SETUP STEPPER OK` |
| `ApGPIO`, `AnGPIO` | `<gpio>` | Set Phase A positive / negative GPIO pins | `GOT APGPIO OK` |
| `BpGPIO`, `BnGPIO` | `<gpio>` | Set Phase B positive / negative GPIO pins | `GOT BPGPIO OK` |
| `FullStep` | *none* | Select Full-Step drive mode | `GOT FULLSTEP OK` |
| `HalfStep` | *none* | Select Half-Step drive mode | `GOT HALFSTEP OK` |
| `MicroStep` | *none* | Select Micro-Step drive mode | `GOT MICROSTEP OK` |
| `msPerStep` | `<ms>` | Set delay between steps in milliseconds | `GOT MSPERSTEP OK` |
| `Step` | `<steps>` | Move stepper specified number of steps (signed) | `GOT STEP OK` |
| `PWMFreq` | `<freq_hz>` | Set current-limiting PWM chopper frequency | `GOT PWMFREQ OK` |
| `PWMDuty` | `<duty_pct>` | Set drive duty cycle percentage | `GOT PWMDUTY OK` |
| `HoldPWM` | `<duty_pct>` | Set stationary holding torque duty cycle | `GOT HOLDPWM OK` |
| `Accel`, `Decel` | `<steps>` | Set ramp acceleration / deceleration step counts | `GOT ACCEL/DECEL OK` |

### 7.5 NeoPixel (Addressable RGB)
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `NeoGPIO` | `<gpio>` | Assign output pin and initialize LED strip | `GOT NEOGPIO OK` |
| `NeoRED` | `<index> <val>` | Set red component (0–255) for pixel index in buffer | `GOT NEORED OK` |
| `NeoGREEN`| `<index> <val>` | Set green component (0–255) for pixel index in buffer | `GOT NEOGREEN OK` |
| `NeoBLUE` | `<index> <val>` | Set blue component (0–255) for pixel index in buffer | `GOT NEOBLUE OK` |
| `NeoUpdate`| *none* | Flush buffer to physical WS2812 strip | `GOT NEOUPDATE OK` |

### 7.6 Analog & Digital I/O
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `SetupADC<A0-A2>`| *none* | Initialize 12-bit ADC channel (A0=GPIO2, A1=GPIO3, A2=GPIO10) | `SETUP ADC<ch> OK` |
| `ReadADC<A0-A2>` | *none* | Read instantaneous raw 12-bit ADC value | `ADCval<ch> <val>` |
| `ADCupperThresh<A0-A2>` | `<val>` | Set upper trigger threshold for hysteresis | `GOT ADCUPPERTHRESH<ch> OK` |
| `ADCLowerThresh<A0-A2>` | `<val>` | Set lower trigger threshold for hysteresis | `GOT ADCLOWERTHRESH<ch> OK` |
| `SetupGPIOhandler` | *none* | Start background GPIO state change detector | `SETUP GPIO OK` |
| `ModeGPIOin` | `<gpio>` | Configure pin as digital input with pull-up | `GOT MODEIN OK` |
| `ModeGPIOout` | `<gpio>` | Configure pin as digital output | `GOT MODEOUT OK` |
| `SetGPIOval` | `<gpio> <val>` | Set digital output level (0 or 1) | `GOT SETGPIO OK` |
| `ReadGPIO` | `<gpio>` | Query digital input level | `GPIO <pin> IS <0\|1>` |

### 7.7 Distance, Proximity & Raw I2C
| Command | Parameters | Description | Response / Event |
| :--- | :--- | :--- | :--- |
| `SetupVL` | *none* | Init VL53L1X ToF sensor & start distance task | `SETUP VL OK` / `FAIL` |
| `StreamVL` | `<0\|1>` | Enable/disable live distance streaming | `VL STREAM ON/OFF` |
| `VLUpperThresh` | `<mm>` | Set distance upper alert threshold | `GOT VLUPPERTHRESH OK` |
| `VLLowerThresh` | `<mm>` | Set distance lower alert threshold | `GOT VLLOWERTHRESH OK` |
| `SetupProximity` | *none* | Init APDS-9930 proximity sensor & start task | `SETUP PROX OK` / `FAIL` |
| `StreamProx` | `<0\|1>` | Enable/disable live proximity streaming | `PROX STREAM ON/OFF` |
| `Npulses` | `<count>` | Set optical IR emitter pulse count | `GOT NPULSES OK` |
| `UpperThresh`, `LowerThresh` | `<val>` | Set proximity alert thresholds | `GOT UPPER/LOWERTHRESH OK` |
| `rawI2CchipADR` | `<7bit_addr>` | Select target I2C slave address | `I2C CHIP ADR OK` |
| `rawI2Cwr` | `<reg> <val>` | Write 1 byte to register over I2C | `I2C WR OK` |
| `rawI2Crd` | `<reg>` | Read 1 byte from register over I2C | `I2Cval <reg> <val>` |

---

## 8. Memory & Partition Layout

The firmware utilizes a custom partition table optimized for Zigbee network credential persistence, OTA application staging, and secure storage:

```csv
# Name,       Type, SubType, Offset,   Size,    Flags
nvs,          data, nvs,     0x9000,   0x6000,
phy_init,     data, phy,     0xf000,   0x1000,
factory,      app,  factory, 0x10000,  900K,
zb_storage,   data, fat,     0xf1000,  16K,
zb_fct,       data, fat,     0xf5000,  1K,
```
- **Application Binary Footprint:** ~600 KB (`0x92910` bytes).
- **Available App Partition Headroom:** 35% free space remaining in the 900 KB factory partition.
- **Non-Volatile Storage (NVS):** Stores Zigbee binding tables, encryption keys, and network PAN credentials across power cycles.

---

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
   - Wired directly to the ESP32-C6 UART pins managed by [`serial_bridge.c`](file:///c:/Users/egape/PNPzigbee/main/serial_bridge.c).
   - Inbound characters from the ESP32-C6 are immediately mirrored back to the host USB console (`Serial.write()`) to preserve transparent bridge visibility.
   - Concurrently, incoming characters are buffered into a 96-byte line accumulator (`processEsp32Byte`) to detect and execute local fixture commands.

---

### 9.4 Verification Protocol Commands

When the Zigbee coordinator sends a text command targeted at the node, the ESP32-C6 serial bridge emits the raw string across UART. `TXRXproto` parses incoming lines and services two dedicated verification commands:

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

