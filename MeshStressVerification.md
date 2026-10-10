# Mesh Stress & Robustness Verification Plan
**Document Version:** 1.1  
**Target Hardware:** ESP32-C6 Unified 6-Node Mesh Network (1 Coordinator + 5 End Devices)  
**Accompanying Tool:** `MeshStressTest.py` (Standalone stress test harness)  
**Date:** October 10, 2026  

---

## 1. Executive Summary & Purpose

The purpose of this specification is to define comprehensive stress and boundary test methodologies designed to push the unified ESP32-C6 wireless mesh network beyond normal operating conditions. 

While the standard Hardware-in-the-Loop test suite (`test_runner.py`) validates functional component correctness (DAC, ADC, GPIO, PWM, Motor, Proximity, Serial Bridge) and basic 3-stage reset topology recovery, **`MeshStressTest.py`** is dedicated exclusively to evaluating network survivability, physical-layer robustness, protocol contention, memory stability, and self-healing under adversarial conditions.

This document integrates an expanded **6-node physical topology** consisting of 1 Coordinator, 2 proximate bench nodes, and 3 remote edge-of-range nodes operating purely Over-The-Air (OTA) without PC serial connections.

---

## 2. Network Topology & Deployment Architecture

### 2.1 Physical Node Distribution

| Node Name | Role | Channel | Power Source | Physical Location | Expected RF Status | Host Interface |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Coordinator** | `ROLE_COORDINATOR` (2) | 1 | USB Host | Workbench Bench | N/A (Master Hub) | Serial (COM11, 115200) |
| **Kitchen** | `ROLE_END_DEVICE` (0) | 1 | USB Host / 5V | Workbench (~50 cm) | `OK` (LQI 200–240) | OTA / Serial (COM5) |
| **Garage** | `ROLE_END_DEVICE` (0) | 1 | USB Host / 5V | Workbench (~50 cm) | `OK` (LQI 200–240) | OTA / Serial (COM8) |
| **N1** | `ROLE_END_DEVICE` (0) | 1 | External 5V Wall/Bank | Mid-Range Room | `OK` / `MARGINAL` (LQI 100–160) | **Pure OTA (No COM)** |
| **N2** | `ROLE_END_DEVICE` (0) | 1 | External 5V Wall/Bank | Perimeter / Obstacle | `MARGINAL` (LQI 50–99) | **Pure OTA (No COM)** |
| **N3** | `ROLE_END_DEVICE` (0) | 1 | External 5V Wall/Bank | Edge of Range | `WEAK` / `MARGINAL` (LQI 30–60) | **Pure OTA (No COM)** |

### 2.2 Architectural Topology Diagram

```
                                    +-----------------------+
                                    |      Host PC CLI      |
                                    |  (MeshStressTest.py)  |
                                    +-----------------------+
                                                |  USB-UART (COM11)
                                                v
                                    +-----------------------+
                                    |      Coordinator      |
                                    |     (ESP32-C6 Hub)    |
                                    +-----------------------+
                                         /   |   |   |   \
                  +---------------------+    |   |   |    +---------------------+
                  |                          |   |   |                          |
    2.4GHz Direct | ~50cm Bench              |   |   |             2.4GHz Edge  | Through Walls / Attenuation
    (LQI 200-240) |                          |   |   |             (LQI 30-100) | Pure OTA / Ext 5V Power
                  v                          |   |   |                          v
         +-----------------+                 |   |   |                 +-----------------+
         |     Kitchen     |                 |   |   |                 |       N3        |
         |  (End Device 1) |                 |   |   |                 |  (End Device 5) |
         +-----------------+                 |   |   |                 +-----------------+
                  |                          |   |   |                          |
                  v                          |   |   |                          v
         +-----------------+                 |   |   |                 +-----------------+
         |     Garage      | <---------------+   |   +---------------> |       N2        |
         |  (End Device 2) |                     v                     |  (End Device 4) |
         +-----------------+            +-----------------+            +-----------------+
                                        |       N1        |
                                        |  (End Device 3) |
                                        +-----------------+
```

### 2.3 Why 5 End Devices + Edge-of-Range Nodes Maximize Stress Value

1. **RF Realism & True Path Loss**: Bench nodes within 50 cm experience near-zero path loss, high SNR, and no multipath fading. Placing `N1`, `N2`, and `N3` at the edge introduces attenuation, variable signal-to-noise ratios, and packet error rates that reflect real deployment environments.
2. **True Wireless Independence**: `N1`, `N2`, and `N3` have no USB connection to the test computer. All discovery, short address assignment, routing updates, operational commands, and reboot cycles occur 100% over the air.
3. **Medium Contention & CSMA/CA Scaling**: 5 nodes contending for clear channel assessment (CCA) trigger genuine radio collisions and exponential backoffs, especially during broadcast discovery sweeps.
4. **Queue Fan-Out Stress**: Broadcasting or round-robin streaming across 5 nodes stresses the Coordinator's FreeRTOS TX queue (`COORD_TX_QUEUE_LEN`) with real multi-destination load.
5. **Routing Table Scalability**: Proves the dynamic table scales from 2 to 5 devices (up to the architectural limit of 20 defined in `ROUTING_TABLE_MAX_NODES`), ensuring collision-free short address assignment (`0x0001` through `0x0005`).

---

## 3. Provisioning Procedure for Nodes N1, N2, and N3

Because of the **Single-Binary Unified Architecture**, the exact same compiled firmware binary (`PNPzigbee.bin`) is flashed onto all nodes. Role and identity are set in NVS:

### Step 1: Flashing Unified Binary
Connect module to USB (`COMx`) and execute:
```powershell
cd c:\Users\egape\PNPzigbee
python -m esptool --chip esp32c6 -p COMx -b 460800 write_flash 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\PNPzigbee.bin
```

### Step 2: Workbench CLI NVS Configuration
Open serial terminal at 115200 baud, press Enter to enter the `mesh-cli>` prompt, and configure:

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
Disconnect each board from PC USB, connect to external 5V USB power adapters, and place them at varying distances:
- **N1**: Moderate distance (room adjacent to bench).
- **N2**: Farther distance (behind walls / obstacles).
- **N3**: Perimeter / fringe of radio coverage.

---

## 4. Detailed Test Specifications

### Stress Test 1: 5-Node High-Frequency Traffic Flood & Queue Backpressure

#### Objective
Evaluate coordinator and end-node performance under sustained high-rate message injection across all 5 nodes. Verify FreeRTOS TX queue backpressure handling, CSMA/CA channel backoff under 5-way contention, and memory stability when serving both high-LQI and low-LQI destinations simultaneously.

#### Method
1. Configure traffic bursts at rates of 10 Hz, 20 Hz, 50 Hz, and 100 Hz.
2. Direct traffic in round-robin sequence across all 5 nodes: `Kitchen` -> `Garage` -> `N1` -> `N2` -> `N3` -> `Kitchen`...
3. Transmit burst blocks of 50, 100, and 250 consecutive packets per rate tier.
4. Note that slow ACKs or retries to edge nodes (`N3`, `N2`) will hold the TX queue longer than proximate nodes (`Kitchen`, `Garage`), creating realistic queue congestion.
5. Measure:
   - Per-node Packet Delivery Ratio (PDR %): `(Successful ACKs / Dispatched Packets) * 100`
   - Overall aggregate PDR across all 5 nodes
   - Queue rejection rate (graceful `"error: TX queue full"` vs unhandled drops)
   - Coordinator and node heap recovery post-burst

#### Pass/Fail Criteria
- **Pass:**
  - Zero task crashes or watchdog resets on Coordinator or any of the 5 nodes.
  - Overall PDR $\ge 95\%$ at 10 Hz; PDR $\ge 85\%$ at 20 Hz across all 5 nodes.
  - Slower ACKs from edge nodes (`N3`) do not starve or block communication to proximate nodes (`Kitchen`, `Garage`).
  - When the TX queue fills, Coordinator outputs explicit `TX queue full` rejection and recovers normal throughput within 500 ms of burst cessation.

---

### Stress Test 2: Dynamic Route Eviction & 5-Node Broadcast Fallback

#### Objective
Verify that the Coordinator seamlessly recovers communication with all 5 nodes when its internal routing cache is completely wiped, validating the multi-layer discovery fallback architecture under a full 5-device topology.

#### Method
1. Confirm initial mesh health with `GiveNetworkReport` (verifying all 5 nodes are online with valid routes: `count == 5`).
2. Issue `ResetNetwork` over serial to flush the Coordinator's active RAM routing table down to 0 devices (`count = 0`).
3. **Immediately**, without running `GiveNetworkReport` or manual discovery, dispatch operational commands to all 5 nodes in rapid succession:
   - `Kitchen blink 1`
   - `Garage blink 1`
   - `N1 blink 1`
   - `N2 blink 1`
   - `N3 blink 1`
4. Monitor Coordinator serial logs for:
   - Layer 3 Unicast Miss trigger: Node MAC/short address not found in table.
   - Broadcast Fallback: Transmission via broadcast short address `0xFFFF` / broadcast MAC.
   - Reverse Route Learning: Automatic insertion of responding nodes into the routing table.
5. Verify that all 5 nodes execute the command and the Coordinator table recovers to `count == 5`.

#### Pass/Fail Criteria
- **Pass:**
  - All 5 nodes successfully receive and execute the dispatched commands despite the cold table.
  - The Coordinator routing table automatically re-populates all 5 nodes (`count == 5`) upon receiving replies.
  - Zero packet drop on initial dispatch via Layer 3 broadcast fallback.

---

### Stress Test 3: RF Boundary & Asymmetric Link Margin Survey

#### Objective
Evaluate mesh communication reliability across variable antenna configurations, asymmetric signal strengths, and low LQI boundary conditions across the proximate vs. edge node distribution.

#### Method
1. Query initial baseline LQI and remote LQI across 20 ping cycles via `GiveNetworkReport`.
2. Group nodes into two RF categories:
   - **Proximate Nodes** (`Kitchen`, `Garage`): Expected LQI $\ge 180$ (`OK`).
   - **Edge Nodes** (`N1`, `N2`, `N3`): Expected LQI $30-150$ (`OK`, `MARGINAL`, `WEAK`).
3. Toggle the hardware RF switch on the Coordinator:
   - Ceramic PCB Antenna: `rf 0 0` (PWR=0, SEL=0)
   - External IPEX / Dipole Antenna: `rf 0 1` (PWR=0, SEL=1)
4. For each node in both antenna modes:
   - Record Coordinator local `LQI` (uplink quality).
   - Record Node reported `REMOTE_LQI` (downlink quality).
   - Calculate Link Asymmetry: $\Delta LQI = |LQI_{local} - LQI_{remote}|$.
   - Dispatch 25 command cycles to evaluate packet loss correlation with LQI thresholds (`OK` $\ge 100$, `MARGINAL` $50-99$, `WEAK` $< 50$).

#### Pass/Fail Criteria
- **Pass:**
  - Unbroken command dispatch and ACK reception while link status is `OK` or `MARGINAL`.
  - Edge nodes classified as `WEAK` (< 50 LQI) maintain $\ge 80\%$ command success rate via retransmissions.
  - RF status classifications in `GiveNetworkReport` accurately reflect the lower of local and remote LQI.
  - No spurious route evictions occur due to intermittent low-LQI packets.

---

### Stress Test 4: Rapid Node Churn & Multi-Node Flapping Resilience

#### Objective
Verify that repeated, rapid end-device power cycling and reboots do not cause routing table corruption, duplicate device entries, memory leakage, or short address starvation on the Coordinator across 5 nodes.

#### Method
1. Execute an automated loop of 20 rapid reboot cycles across nodes:
   - **Phase A (Single Edge Flapping)**: Rapid reboot of `N3` (edge-most node) every 2.0 seconds $\times 5$ cycles.
   - **Phase B (Mixed Proximate & Edge Flapping)**: Alternating rapid reboots of `Garage` and `N2` every 2.5 seconds $\times 5$ cycles.
   - **Phase C (Rolling Churn)**: Reboot `Kitchen` -> `Garage` -> `N1` -> `N2` -> `N3` in 1.5-second sequence.
2. Following each reboot:
   - Log arrival time of Layer 1 announcement (`ANNOUNCE:<name>:<ieee>`).
   - Verify Coordinator issues `HELLO_ACK`.
   - Verify assigned 16-bit short address remains consistent or updates cleanly without table entry duplication.
3. At the conclusion of all 20 cycles, verify:
   - Routing table count equals exactly 5 (`routing_table_get_count() == 5`).
   - All 5 nodes report `online == 1` and `route == YES`.
   - FreeRTOS heap memory on Coordinator has not drifted downward from baseline.

#### Pass/Fail Criteria
- **Pass:**
  - Coordinator routing table count remains strictly 5 (zero duplicate MACs or phantom entries).
  - All rebooted nodes achieve route re-establishment within $< 1.5$ seconds of boot.
  - Coordinator heap delta $< 150$ bytes across the entire 20-cycle test (proving zero memory leak).

---

### Stress Test 5: 5-Node Concurrent Announcement Collision & Backoff Jitter

#### Objective
Induce maximum air contention by triggering concurrent boots of all 5 end devices simultaneously, verifying that transmission backoff jitter prevents continuous packet collision.

#### Method
1. Issue simultaneous reboot commands to all 5 nodes (`Kitchen`, `Garage`, `N1`, `N2`, `N3`) within a tight 20 ms window.
2. All 5 nodes boot their RF stacks at approximately the same millisecond and attempt to transmit their Layer 1 broadcast announcements (`ANNOUNCE`).
3. Monitor Coordinator RX timeline:
   - Measure arrival times and packet intervals for all 5 announcements.
   - If collisions occur on the air, verify that the Coordinator's automatic Layer 2 boot sweep (`DISCOVER_ALL`) successfully sweeps and recovers any collided nodes.
4. Execute 10 iterations of the 5-node concurrent boot race.

#### Pass/Fail Criteria
- **Pass:**
  - In $100\%$ of iterations, all 5 nodes successfully register in the Coordinator table within $< 3.0$ seconds of boot.
  - Zero deadlock or permanent node loss resulting from broadcast packet collisions.

---

### Stress Test 6: Long-Duration 5-Node Soak & Stability Profiling

#### Objective
Subject the entire 6-node network (Coordinator + 5 End Devices) to continuous background operation over an extended duration (1 to 12 hours) to identify memory leaks, FreeRTOS task degradation, and environmental RF drift across proximate and edge nodes.

#### Method
1. Continuous round-robin ping loop across all 5 nodes: `Kitchen` -> `Garage` -> `N1` -> `N2` -> `N3` at 1.0 Hz total aggregate rate.
2. Continuous logging of:
   - Per-node and aggregate RTT latency (min, average, median, 95th percentile, max).
   - Local LQI and Remote LQI trends over time for proximate vs edge nodes.
   - Cumulative packet count, success count, failure count, and timeout count per node.
   - Coordinator free heap memory periodically sampled every 60 seconds.
3. Configurable test durations:
   - Quick Soak: 30 minutes (pre-release sanity check).
   - Standard Soak: 2 hours (regression verification).
   - Extended Soak: 8–12 hours (overnight stability verification).

#### Pass/Fail Criteria
- **Pass:**
  - Aggregate Packet Delivery Ratio (PDR) $\ge 99.0\%$ across all 5 nodes (proximate $\ge 99.8\%$, edge $\ge 98.0\%$).
  - Free heap slope over time is flat (zero continuous downward drift).
  - P95 latency remains $< 60\text{ ms}$ throughout the test duration.
  - Zero permanent node disconnects; transient link drops on edge nodes recover automatically.

---

## 5. Architectural Design of `MeshStressTest.py`

### Design Principles
1. **Complete Decoupling:** `MeshStressTest.py` is independent of `test_runner.py`. It does not require test fixtures (e.g., Trinket M0, DAC, ADC, Schmitt shields) and focuses strictly on Zigbee / ESP-NOW radio mesh mechanics.
2. **Dynamic Node Auto-Discovery:** Automatically queries `GiveNetworkReport` on startup to discover all active nodes (supporting 2, 3, 5, or more nodes dynamically), or accepts an explicit node list via CLI.
3. **Structured Metrics Logging:** Outputs real-time ANSI terminal progress tables and exports structured CSV / JSON run summaries containing latency percentiles, LQI histograms, and PDR statistics.

### Command-Line Interface Specification

```bash
# Run the entire stress test battery across all discovered nodes (Tests 1 through 5)
python MeshStressTest.py -p COM11 --all-stress

# Run specific stress tests individually
python MeshStressTest.py -p COM11 --flood --rate 20 --burst 100
python MeshStressTest.py -p COM11 --eviction
python MeshStressTest.py -p COM11 --rf-margin
python MeshStressTest.py -p COM11 --churn --cycles 20
python MeshStressTest.py -p COM11 --collision-race --iterations 10

# Explicitly specify node names (default: auto-discover from GiveNetworkReport)
python MeshStressTest.py -p COM11 --all-stress --nodes Kitchen Garage N1 N2 N3

# Run long-duration soak test
python MeshStressTest.py -p COM11 --soak --duration-min 120 --rate 1.0

# Optional hardware reset COM ports for bench nodes (edge nodes reset purely via OTA)
python MeshStressTest.py -p COM11 --churn --kitchen-port COM5 --garage-port COM8
```

### Module Structure for `MeshStressTest.py`

```
MeshStressTest.py
├── class MeshCoordinatorClient (Serial engine, async reader, auto-reconnect)
├── class StressMetricsCollector (Per-node latency, PDR, jitter, LQI history, JSON export)
├── module test_rate_flood (5-node round-robin packet injection & backpressure test)
├── module test_route_eviction (ResetNetwork route table cache eviction test)
├── module test_rf_margin (Proximate vs edge LQI asymmetry & antenna test)
├── module test_node_churn (Rapid repetitive reboot flapping test across nodes)
├── module test_collision_race (5-node concurrent boot announcement race test)
└── module test_soak_runner (Long-duration 1-12h soak & stability profiler)
```

---

## 6. Summary Matrix & Acceptance Criteria

| Stress Test | Command Flag | Nodes Tested | Duration | Key Acceptance Metric |
| :--- | :--- | :--- | :--- | :--- |
| **1. Rate Flood** | `--flood` | All 5 Nodes | ~2–3 min | Aggregate PDR $\ge 95\%$ @ 10Hz, queue full gracefully handled |
| **2. Route Eviction** | `--eviction` | All 5 Nodes | ~30 sec | 100% command delivery to all 5 nodes via broadcast fallback |
| **3. RF Margin** | `--rf-margin` | Proximate vs Edge | ~2 min | LQI asymmetry tracked, zero drops in OK/MARGINAL zones |
| **4. Node Churn** | `--churn` | Mixed 5 Nodes | ~4–6 min | Table count strictly 5, zero duplicate MACs, zero heap leak |
| **5. Collision Race** | `--collision-race` | All 5 Nodes | ~3–4 min | 10/10 iterations recover all 5 nodes in $< 3.0$s |
| **6. Soak Profiling** | `--soak` | All 5 Nodes | 30–720 min | PDR $\ge 99.0\%$, P95 RTT $< 60$ms, flat heap profile |

---
*Next Step: Implementation of `MeshStressTest.py` based on this 5-node specification.*
