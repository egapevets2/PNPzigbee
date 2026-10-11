# Star Network Stress & Robustness Verification Plan
**Document Version:** 2.0  
**Target Hardware:** ESP32-C6 Unified 6-Node Network (1 Coordinator Hub + 5 End Device Spokes)  
**Architecture:** Star Topology (Centralized Hub-and-Spoke on ESP-NOW Wireless Substrate)  
**Accompanying Tool:** `MeshStressTest.py` (Standalone stress test harness)  
**Date:** October 11, 2026  

---

## 1. Executive Summary & Purpose

The purpose of this specification is to define comprehensive stress and boundary test methodologies designed to push the unified ESP32-C6 wireless network beyond normal operating conditions.

### 1.1 Architectural Model: Star Topology (Hub-and-Spoke)
While the firmware repository leverages ESP-NOW with broadcast fallback mechanics, this project intentionally implements a **Star Topology**:
- **All Downlink Traffic:** Originates exclusively from the **Coordinator (Hub)** to an **End Device (Spoke)**.
- **All Uplink Traffic:** Flows directly from an **End Device (Spoke)** to the **Coordinator (Hub)**.
- **No Peer-to-Peer Multi-Hop Relay:** Spokes do not route or forward packets through neighboring spokes.

### 1.2 Engineering Rationale for the Star Topology
Deploying all nodes within direct radio reach of a central Coordinator is the optimal engineering choice for this deployment:
1. **Deterministic Low Latency:** Single-hop delivery guarantees fast round-trip times (~20–30 ms) with zero intermediate queueing delays or routing re-convergence pauses.
2. **Elimination of Protocol Overhead:** Eliminates multi-hop routing protocol chatter (e.g., RPL/AODV route requests, route replies, neighbor discovery floods).
3. **Fault Isolation:** Failure or reboot of one spoke cannot cascade or sever connectivity to any other spoke.
4. **ESP-NOW Long Range (LR) Envelope:** The ESP32-C6 in Long-Range Mode (`LR=1`, 20 dBm TX power) easily covers 200–400+ meters line-of-sight and penetrates typical residential/workshop interior walls, eliminating the need for mesh relays.
5. **Direct Link Attribution:** LQI and Remote LQI metrics recorded at the Hub represent the exact, unmasked physical link quality to that specific node.

### 1.3 Scope of the Stress Suite
While `test_runner.py` validates peripheral hardware functional correctness (DAC, ADC, GPIO, PWM, Motor, Proximity, Serial Bridge), **`MeshStressTest.py`** evaluates:
- Hub FreeRTOS TX/RX queue backpressure and memory stability under sustained 5-spoke load.
- CSMA/CA air contention and **Hidden Terminal** collision handling.
- Spoke route table eviction, broadcast fallback, and dynamic reverse-route learning.
- Head-of-line blocking resilience when fast bench spokes and marginal edge spokes are polled concurrently.

---

## 2. Network Topology & Deployment Architecture

### 2.1 Physical Node Distribution

| Node Name | Role | Channel | Power Source | Physical Location | Expected RF Status | Host Interface |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Coordinator** | `ROLE_COORDINATOR` (2) | 1 | USB Host | Central Hub Location | N/A (Master Hub) | Serial (COM11, 115200) |
| **Kitchen** | `ROLE_END_DEVICE` (0) | 1 | USB Host / 5V | Proximate Spoke (~50 cm) | `OK` (LQI 200–240) | OTA / Serial (COM5) |
| **Garage** | `ROLE_END_DEVICE` (0) | 1 | USB Host / 5V | Proximate Spoke (~50 cm) | `OK` (LQI 200–240) | OTA / Serial (COM8) |
| **N1** | `ROLE_END_DEVICE` (0) | 1 | External 5V Wall/Bank | Mid-Range Spoke | `OK` / `MARGINAL` (LQI 100–160) | **Pure OTA (No COM)** |
| **N2** | `ROLE_END_DEVICE` (0) | 1 | External 5V Wall/Bank | Perimeter / Obstacle Spoke | `MARGINAL` (LQI 50–99) | **Pure OTA (No COM)** |
| **N3** | `ROLE_END_DEVICE` (0) | 1 | External 5V Wall/Bank | Edge-of-Range Spoke | `WEAK` / `MARGINAL` (LQI 30–60) | **Pure OTA (No COM)** |

### 2.2 Hub-and-Spoke Topology Diagram

```
                                    +-----------------------+
                                    |      Host PC CLI      |
                                    |  (MeshStressTest.py)  |
                                    +-----------------------+
                                                |  USB-UART (COM11)
                                                v
                                    +-----------------------+
                                    |      Coordinator      |
                                    |     (Central Hub)     |
                                    +-----------------------+
                                   /     |     |     |     \
         +------------------------+      |     |     |      +------------------------+
         | (Direct Single-Hop)           |     |     |          (Direct Single-Hop)  |
         | Bench Spoke                   |     |     |          Edge-of-Range Spoke  |
         | LQI 200-240                   |     |     |          LQI 30-100           |
         v                               v     |     v                               v
+-----------------+      +-----------------+   |   +-----------------+      +-----------------+
|     Kitchen     |      |     Garage      |   |   |       N2        |      |       N3        |
|     (Spoke 1)   |      |     (Spoke 2)   |   |   |     (Spoke 4)   |      |     (Spoke 5)   |
+-----------------+      +-----------------+   |   +-----------------+      +-----------------+
                                               v
                                      +-----------------+
                                      |       N1        |
                                      |     (Spoke 3)   |
                                      +-----------------+
```

### 2.3 Physical Realities & Star Stress Dynamics

1. **The "Funnel" Effect at the Hub:** In a star network, 100% of all uplink traffic converges on the Coordinator's single radio receiver. When multiple spokes transmit telemetry or boot announcements at once, they compete directly for the Coordinator's receive buffer and FreeRTOS task processing time.
2. **The "Hidden Terminal" Condition:** Because `Kitchen` and `N3` are on opposite sides of the premises, they may **not be able to hear each other**. When `Kitchen` transmits to the Coordinator, `N3` listens to the air (Clear Channel Assessment - CCA), hears silence, and transmits at the exact same instant—causing an in-air collision at the Coordinator. Testing with edge spokes evaluates how effectively CCA retry backoff resolves hidden terminal collisions.
3. **Head-of-Line Blocking Resilience:** When the Coordinator transmits round-robin, slow acknowledgments or retry timeouts to edge spoke `N3` must never block, delay, or starve rapid communication to proximate bench spokes `Kitchen` and `Garage`.
4. **Complete Wireless Autonomy:** Spokes `N1`, `N2`, and `N3` operate without USB cables, proving that discovery, short address assignment, reverse-route learning, and over-the-air reboots (`N1 reboot`) function seamlessly over the air.

---

## 3. Spoke Provisioning Guide (Nodes N1, N2, and N3)

Because of the **Single-Binary Unified Architecture**, the exact same compiled firmware binary (`PNPzigbee.bin`) is flashed onto all ESP32-C6 modules. Each spoke's role and identity are set in NVS:

### Step 1: Flashing Unified Binary
Connect each new module to PC USB (`COMx`) and execute:
```powershell
cd c:\Users\egape\PNPzigbee
python -m esptool --chip esp32c6 -p COMx -b 460800 write_flash 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\PNPzigbee.bin
```

### Step 2: Workbench CLI NVS Configuration
Open a serial terminal at 115200 baud, press Enter to enter the `mesh-cli>` prompt, and configure:

**For Node N1:**
```text
cfg role 0
cfg name N1
cfg channel 1
cfg power 80
cfg save
```

**For Node N2:**
```text
cfg role 0
cfg name N2
cfg channel 1
cfg power 80
cfg save
```

**For Node N3:**
```text
cfg role 0
cfg name N3
cfg channel 1
cfg power 80
cfg save
```

### Step 3: Deployment
Disconnect each board from PC USB, plug into any standard 5V USB wall adapter or battery bank, and place them across the premises:
- **N1**: Moderate distance (room adjacent to bench).
- **N2**: Farther distance (behind walls / obstacles).
- **N3**: Perimeter / fringe of radio coverage.

---

## 4. Detailed Test Specifications

### Stress Test 1: 5-Spoke High-Frequency Traffic Flood & Hub Queue Backpressure

#### Objective
Evaluate Coordinator Hub and spoke performance under sustained high-rate round-robin message injection across all 5 spokes. Verify Hub FreeRTOS TX queue backpressure handling, CSMA/CA channel backoff under 5-way contention, and memory stability when serving both high-LQI and low-LQI spokes simultaneously.

#### Method
1. Configure traffic bursts at rates of 10 Hz, 20 Hz, 50 Hz, and 100 Hz.
2. Direct traffic in round-robin sequence across all 5 spokes: `Kitchen` -> `Garage` -> `N1` -> `N2` -> `N3` -> `Kitchen`...
3. Transmit burst blocks of 50, 100, and 250 consecutive packets per rate tier.
4. Verify that slow ACKs or retries to edge spokes (`N3`, `N2`) do not stall the Coordinator's TX task or cause head-of-line blocking for proximate spokes (`Kitchen`, `Garage`).
5. Measure:
   - Per-spoke Packet Delivery Ratio (PDR %): `(Successful ACKs / Dispatched Packets) * 100`
   - Overall aggregate PDR across all 5 spokes
   - Hub queue rejection rate (graceful `"error: TX queue full"` vs unhandled drops)
   - Hub and spoke free heap recovery post-burst

#### Pass/Fail Criteria
- **Pass:**
  - Zero task crashes or watchdog resets on Coordinator or any of the 5 spokes.
  - Overall PDR $\ge 95\%$ at 10 Hz; PDR $\ge 85\%$ at 20 Hz across all 5 spokes.
  - Slower ACKs from edge spokes (`N3`) do not starve or block communication to proximate bench spokes (`Kitchen`, `Garage`).
  - When the Hub TX queue fills, Coordinator outputs explicit `TX queue full` rejection and recovers normal throughput within 500 ms of burst cessation.

---

### Stress Test 2: Dynamic Spoke Cache Eviction & Broadcast Discovery Fallback

#### Objective
Verify that the Coordinator Hub seamlessly recovers direct communication with all 5 spokes when its internal device table is completely wiped, validating the Layer 3 broadcast discovery fallback mechanism.

#### Method
1. Confirm initial star health with `GiveNetworkReport` (verifying all 5 spokes are online with valid routes: `count == 5`).
2. Issue `ResetNetwork` over serial to flush the Coordinator's active RAM routing table down to 0 devices (`count = 0`).
3. **Immediately**, without running `GiveNetworkReport` or manual discovery, dispatch operational commands to all 5 spokes in rapid succession:
   - `Kitchen blink 1`
   - `Garage blink 1`
   - `N1 blink 1`
   - `N2 blink 1`
   - `N3 blink 1`
4. Monitor Coordinator serial logs for:
   - Layer 3 Unicast Miss trigger: Spoke MAC/short address not found in table.
   - Broadcast Fallback: Transmission via broadcast short address `0xFFFF` / broadcast MAC.
   - Reverse Route Learning: Automatic insertion of responding spokes into the Coordinator table.
5. Verify that all 5 spokes execute the command and the Coordinator table recovers to `count == 5`.

#### Pass/Fail Criteria
- **Pass:**
  - All 5 spokes successfully receive and execute the dispatched commands despite the cold table.
  - The Coordinator routing table automatically re-populates all 5 spokes (`count == 5`) upon receiving replies.
  - Zero packet drop on initial dispatch via Layer 3 broadcast fallback.

---

### Stress Test 3: RF Boundary & Star Link Budget Asymmetry Survey

#### Objective
Evaluate star link reliability across variable antenna configurations, asymmetric signal strengths, and low LQI boundary conditions across proximate versus edge spokes.

#### Method
1. Query initial baseline LQI and remote LQI across 20 ping cycles via `GiveNetworkReport`.
2. Group spokes into two RF categories:
   - **Proximate Spokes** (`Kitchen`, `Garage`): Expected LQI $\ge 180$ (`OK`).
   - **Edge Spokes** (`N1`, `N2`, `N3`): Expected LQI $30-150$ (`OK`, `MARGINAL`, `WEAK`).
3. Toggle the hardware RF switch on the Coordinator Hub:
   - Ceramic PCB Antenna: `rf 0 0` (PWR=0, SEL=0)
   - External IPEX / Dipole Antenna: `rf 0 1` (PWR=0, SEL=1)
4. For each spoke in both Hub antenna modes:
   - Record Coordinator local `LQI` (uplink quality from spoke to Hub).
   - Record Spoke reported `REMOTE_LQI` (downlink quality from Hub to spoke).
   - Calculate Link Asymmetry: $\Delta LQI = |LQI_{local} - LQI_{remote}|$.
   - Dispatch 25 command cycles to evaluate packet loss correlation with LQI thresholds (`OK` $\ge 100$, `MARGINAL` $50-99$, `WEAK` $< 50$).

#### Pass/Fail Criteria
- **Pass:**
  - Unbroken command dispatch and ACK reception while link status is `OK` or `MARGINAL`.
  - Edge spokes classified as `WEAK` (< 50 LQI) maintain $\ge 80\%$ command success rate via retransmissions.
  - RF status classifications in `GiveNetworkReport` accurately reflect the lower of local and remote LQI.
  - No spurious spoke evictions occur due to intermittent low-LQI packets.

---

### Stress Test 4: Rapid Spoke Churn & Power Flapping Resilience

#### Objective
Verify that repeated, rapid spoke power cycling and reboots do not cause Hub routing table corruption, duplicate device entries, memory leakage, or short address starvation across 5 spokes.

#### Method
1. Execute an automated loop of 20 rapid reboot cycles across spokes:
   - **Phase A (Single Edge Flapping):** Rapid reboot of `N3` (edge-most spoke) every 2.0 seconds $\times 5$ cycles.
   - **Phase B (Mixed Proximate & Edge Flapping):** Alternating rapid reboots of `Garage` and `N2` every 2.5 seconds $\times 5$ cycles.
   - **Phase C (Rolling Spoke Churn):** Reboot `Kitchen` -> `Garage` -> `N1` -> `N2` -> `N3` in 1.5-second sequence.
2. Following each reboot:
   - Log arrival time of Layer 1 announcement (`ANNOUNCE:<name>:<ieee>`).
   - Verify Hub issues `HELLO_ACK`.
   - Verify assigned 16-bit short address remains consistent or updates cleanly without duplicate table entries.
3. At the conclusion of all 20 cycles, verify:
   - Routing table count equals exactly 5 (`routing_table_get_count() == 5`).
   - All 5 spokes report `online == 1` and `route == YES`.
   - FreeRTOS heap memory on Coordinator has not drifted downward from baseline.

#### Pass/Fail Criteria
- **Pass:**
  - Coordinator routing table count remains strictly 5 (zero duplicate MACs or phantom entries).
  - All rebooted spokes achieve route re-establishment within $< 1.5$ seconds of boot.
  - Coordinator heap delta $< 150$ bytes across the entire 20-cycle test (proving zero memory leak).

---

### Stress Test 5: 5-Spoke Concurrent Boot Collision & Hidden-Terminal Jitter

#### Objective
Induce maximum air contention by triggering concurrent boots of all 5 spokes simultaneously, verifying that transmission backoff jitter prevents continuous packet collisions at the Hub receiver even under hidden terminal conditions.

#### Method
1. Issue simultaneous reboot commands to all 5 spokes (`Kitchen`, `Garage`, `N1`, `N2`, `N3`) within a tight 20 ms window.
2. All 5 spokes boot their RF stacks at approximately the same millisecond and attempt to transmit their Layer 1 broadcast announcements (`ANNOUNCE`).
3. Because edge spokes cannot hear bench spokes (hidden terminals), both may transmit simultaneously.
4. Monitor Coordinator RX timeline:
   - Measure arrival times and packet intervals for all 5 announcements.
   - If collisions occur on the air, verify that the Coordinator's automatic Layer 2 boot sweep (`DISCOVER_ALL`) successfully sweeps and recovers any collided spokes.
5. Execute 10 iterations of the 5-spoke concurrent boot race.

#### Pass/Fail Criteria
- **Pass:**
  - In $100\%$ of iterations, all 5 spokes successfully register in the Coordinator table within $< 3.0$ seconds of boot.
  - Zero deadlock or permanent node loss resulting from broadcast packet collisions.

---

### Stress Test 6: Long-Duration 5-Spoke Star Soak & Latency Profiling

#### Objective
Subject the entire 6-node Star Network (Hub + 5 Spokes) to continuous background operation over an extended duration (1 to 12 hours) to identify memory leaks, FreeRTOS task degradation, and environmental RF drift across proximate and edge spokes.

#### Method
1. Continuous round-robin ping loop across all 5 spokes: `Kitchen` -> `Garage` -> `N1` -> `N2` -> `N3` at 1.0 Hz total aggregate rate.
2. Continuous logging of:
   - Per-spoke and aggregate RTT latency (min, average, median, 95th percentile, max).
   - Local LQI and Remote LQI trends over time for proximate versus edge spokes.
   - Cumulative packet count, success count, failure count, and timeout count per spoke.
   - Coordinator free heap memory periodically sampled every 60 seconds.
3. Configurable test durations:
   - Quick Soak: 30 minutes (pre-release sanity check).
   - Standard Soak: 2 hours (regression verification).
   - Extended Soak: 8–12 hours (overnight stability verification).

#### Pass/Fail Criteria
- **Pass:**
  - Aggregate Packet Delivery Ratio (PDR) $\ge 99.0\%$ across all 5 spokes (proximate $\ge 99.8\%$, edge $\ge 98.0\%$).
  - Free heap slope over time is flat (zero continuous downward drift).
  - P95 latency remains $< 60\text{ ms}$ throughout the test duration.
  - Zero permanent spoke disconnects; transient link drops on edge spokes recover automatically.

---

## 5. Architectural Design of `MeshStressTest.py`

### Design Principles
1. **Complete Decoupling:** `MeshStressTest.py` is independent of `test_runner.py`. It does not require test fixtures (e.g., Trinket M0, DAC, ADC, Schmitt shields) and focuses strictly on star radio protocol mechanics.
2. **Dynamic Spoke Auto-Discovery:** Automatically queries `GiveNetworkReport` on startup to discover all active spokes (supporting 2, 3, 5, or more spokes dynamically), or accepts an explicit spoke list via CLI.
3. **Structured Metrics Logging:** Outputs real-time ANSI terminal progress tables and exports structured CSV / JSON run summaries containing latency percentiles, LQI histograms, and PDR statistics.

### Command-Line Interface Specification

```bash
# Run the entire stress test battery across all discovered spokes (Tests 1 through 5)
python MeshStressTest.py -p COM11 --all-stress

# Run specific stress tests individually
python MeshStressTest.py -p COM11 --flood --rate 20 --burst 100
python MeshStressTest.py -p COM11 --eviction
python MeshStressTest.py -p COM11 --rf-margin
python MeshStressTest.py -p COM11 --churn --cycles 20
python MeshStressTest.py -p COM11 --collision-race --iterations 10

# Explicitly specify spoke names (default: auto-discover from GiveNetworkReport)
python MeshStressTest.py -p COM11 --all-stress --nodes Kitchen Garage N1 N2 N3

# Run long-duration soak test
python MeshStressTest.py -p COM11 --soak --duration-min 120 --rate 1.0

# Optional hardware reset COM ports for bench spokes (edge spokes reset purely via OTA)
python MeshStressTest.py -p COM11 --churn --kitchen-port COM5 --garage-port COM8
```

### Module Structure for `MeshStressTest.py`

```
MeshStressTest.py
├── class MeshCoordinatorClient (Serial engine, async reader, auto-reconnect)
├── class StressMetricsCollector (Per-spoke latency, PDR, jitter, LQI history, JSON export)
├── module test_rate_flood (5-spoke round-robin packet injection & backpressure test)
├── module test_route_eviction (ResetNetwork spoke table cache eviction test)
├── module test_rf_margin (Proximate vs edge LQI asymmetry & antenna test)
├── module test_node_churn (Rapid repetitive reboot flapping test across spokes)
├── module test_collision_race (5-spoke concurrent boot announcement race test)
└── module test_soak_runner (Long-duration 1-12h soak & stability profiler)
```

---

## 6. Summary Matrix & Acceptance Criteria

| Stress Test | Command Flag | Nodes Tested | Duration | Key Acceptance Metric |
| :--- | :--- | :--- | :--- | :--- |
| **1. Rate Flood** | `--flood` | All 5 Spokes | ~2–3 min | Aggregate PDR $\ge 95\%$ @ 10Hz, queue full gracefully handled |
| **2. Route Eviction** | `--eviction` | All 5 Spokes | ~30 sec | 100% command delivery to all 5 spokes via broadcast fallback |
| **3. RF Margin** | `--rf-margin` | Proximate vs Edge | ~2 min | LQI asymmetry tracked, zero drops in OK/MARGINAL zones |
| **4. Spoke Churn** | `--churn` | Mixed 5 Spokes | ~4–6 min | Table count strictly 5, zero duplicate MACs, zero heap leak |
| **5. Collision Race** | `--collision-race` | All 5 Spokes | ~3–4 min | 10/10 iterations recover all 5 spokes in $< 3.0$s |
| **6. Soak Profiling** | `--soak` | All 5 Spokes | 30–720 min | PDR $\ge 99.0\%$, P95 RTT $< 60$ms, flat heap profile |

---
*Next Step: Implementation of `MeshStressTest.py` based on this Star Topology specification.*
