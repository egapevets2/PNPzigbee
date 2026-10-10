# Mesh Stress & Robustness Verification Plan
**Document Version:** 1.0  
**Target Hardware:** ESP32-C6 Unified Mesh Architecture (Coordinator, Kitchen, Garage)  
**Accompanying Tool:** `MeshStressTest.py` (Standalone stress test harness)  
**Date:** October 10, 2026  

---

## 1. Executive Summary & Purpose

The purpose of this specification is to define comprehensive stress and boundary test methodologies designed to push the unified ESP32-C6 wireless mesh network beyond normal operating conditions. 

While the standard Hardware-in-the-Loop test suite (`test_runner.py`) validates functional component correctness (DAC, ADC, GPIO, PWM, Motor, Proximity, Serial Bridge) and basic 3-stage reset topology recovery, **`MeshStressTest.py`** is dedicated exclusively to evaluating network survivability, physical-layer robustness, protocol contention, memory stability, and self-healing under adversarial conditions.

This document serves as the implementation specification for `MeshStressTest.py`.

---

## 2. Test Architecture & Stress Vectors

```
+-----------------------------------------------------------------------------------+
|                            MeshStressTest.py (Host Harness)                       |
+-----------------------------------------------------------------------------------+
       |                                      |                                |
       | Stress Vector 1 & 4                  | Stress Vector 2                | Stress Vector 3
       v (Burst Flood / Flapping)             v (Route Eviction / Wipe)        v (RF Antenna / Margin)
+-----------------------------+        +-----------------------------+  +-----------------------------+
|    FreeRTOS TX/RX Queues    |        | Coordinator Routing Table   |  |   RF Switch & Gain Control  |
| - Backpressure handling     |        | - Table wipe (ResetNetwork) |  | - Ceramic vs External Ant   |
| - CSMA/CA channel backoff   |        | - Layer 3 broadcast fallback|  | - Asymmetric LQI evaluation |
| - Drop & leak prevention    |        | - Dynamic reverse learning  |  | - Marginal LQI (<50) limits |
+-----------------------------+        +-----------------------------+  +-----------------------------+
```

### Core Stress Vectors Under Evaluation

| Vector | Focus Area | Primary Failure Modes Tested |
| :--- | :--- | :--- |
| **V1: Rate Flood & Contention** | FreeRTOS queue depth & 2.4 GHz PHY | Queue overflow, heap depletion, CSMA/CA lockup, packet drop |
| **V2: Dynamic Route Eviction** | Layer 3 fallback & routing discovery | Route table desynchronization, packet loss during cold cache |
| **V3: RF Boundary & Margin** | Antenna switch & TX power margin | Asymmetric links, hidden terminal behavior, link loss at low LQI |
| **V4: Rapid Node Churn** | Continuous power-cycle flapping | Stale table entries, MAC collision, address exhaustion |
| **V5: Collision Race Condition** | Simultaneous boot announcements | Air contention, announcement collision, jitter backoff failure |
| **V6: Long-Duration Soak** | System longevity & memory profiling | Slow heap fragmentation, task starvation, environmental LQI drift |

---

## 3. Detailed Test Specifications

### Stress Test 1: High-Frequency Traffic Flood & Queue Backpressure

#### Objective
Evaluate coordinator and end-node performance under sustained high-rate message injection. Verify that queue backpressure protects system memory and that CSMA/CA MAC backoff functions properly without task deadlock.

#### Method
1. Configure traffic bursts at rates of 10 Hz, 20 Hz, 50 Hz, and 100 Hz.
2. Direct traffic round-robin between `Kitchen` and `Garage` using operational commands (e.g., `blink 1`, GPIO queries, and PING frames).
3. Transmit burst blocks of 50, 100, and 250 consecutive packets per rate tier.
4. Measure:
   - Packet Delivery Ratio (PDR %): `(Successful ACKs / Dispatched Packets) * 100`
   - Queue rejection rate (graceful `"error: TX queue full"` vs unhandled drops)
   - Coordinator and node heap recovery post-burst

#### Pass/Fail Criteria
- **Pass:**
  - Zero task crashes or watchdog resets on Coordinator or remote nodes.
  - PDR $\ge 98\%$ at 10 Hz; PDR $\ge 90\%$ at 20 Hz under contention.
  - When TX queue fills, Coordinator outputs explicit `TX queue full` rejection and recovers normal throughput within 500 ms of burst cessation.

---

### Stress Test 2: Dynamic Route Eviction & Layer 3 Broadcast Fallback

#### Objective
Verify that the Coordinator seamlessly recovers communication with end nodes when its internal routing cache is completely wiped, validating the multi-layer discovery fallback architecture.

#### Method
1. Confirm initial mesh health with `GiveNetworkReport` (verifying `Kitchen` and `Garage` are online with valid routes).
2. Issue `ResetNetwork` over serial to flush the Coordinator's active RAM routing table down to 0 devices (`count = 0`).
3. **Immediately**, without running `GiveNetworkReport` or `scan`, dispatch operational commands to `Kitchen` (`Kitchen blink 1`) and `Garage` (`Garage blink 1`).
4. Monitor Coordinator serial logs for:
   - Layer 3 Unicast Miss trigger: Node MAC/short address not found in table.
   - Broadcast Fallback: Transmission via broadcast short address `0xFFFF` / broadcast MAC.
   - Reverse Route Learning: Automatic insertion of responding node into routing table.
5. Verify command execution latency on the cold-cache dispatch versus warm-cache dispatch.

#### Pass/Fail Criteria
- **Pass:**
  - Both nodes successfully execute the dispatched commands despite the cold table.
  - The Coordinator routing table automatically re-populates both nodes (`count == 2`) upon receiving responses.
  - Zero packet drop on initial dispatch via Layer 3 broadcast fallback.

---

### Stress Test 3: RF Boundary & Asymmetric Link Margin Stress

#### Objective
Evaluate mesh communication reliability across variable antenna configurations, asymmetric signal strengths, and low LQI boundary conditions.

#### Method
1. Query initial baseline LQI and remote LQI across 20 ping cycles via `GiveNetworkReport`.
2. Toggle the hardware RF switch on the Coordinator:
   - Ceramic PCB Antenna: `rf 0 0` (PWR=0, SEL=0)
   - External IPEX / Dipole Antenna: `rf 0 1` (PWR=0, SEL=1)
3. Step through TX power attenuations if supported in runtime configuration (`rf <pwr> <sel>`).
4. In each configuration:
   - Record Coordinator local `LQI` (uplink quality).
   - Record Node reported `REMOTE_LQI` (downlink quality).
   - Calculate Link Asymmetry: $\Delta LQI = |LQI_{local} - LQI_{remote}|$.
   - Dispatch 25 command cycles to evaluate packet loss correlation with LQI thresholds (`OK` $\ge 100$, `MARGINAL` $50-99$, `WEAK` $< 50$).

#### Pass/Fail Criteria
- **Pass:**
  - Unbroken command dispatch and ACK reception while link status is `OK` or `MARGINAL`.
  - RF status classifications in `GiveNetworkReport` accurately transition based on minimum LQI.
  - Dynamic route remains established without erroneous node eviction during momentary LQI dips.

---

### Stress Test 4: Rapid Node Churn & Power Flapping Resilience

#### Objective
Verify that repeated, rapid end-device power cycling and reboots do not cause routing table corruption, duplicate device entries, memory leakage, or short address starvation on the Coordinator.

#### Method
1. Execute an automated loop of 15 rapid reboot cycles:
   - Cycle A: Rapid reboot of `Garage` only every 2.0 seconds.
   - Cycle B: Alternating rapid reboot of `Kitchen` and `Garage` every 2.5 seconds.
2. Following each reboot:
   - Log arrival time of Layer 1 announcement (`ANNOUNCE:<name>:<ieee>`).
   - Verify Coordinator issues `HELLO_ACK`.
   - Verify assigned 16-bit short address remains consistent or updates cleanly without table entry duplication.
3. At the conclusion of 15 cycles, verify:
   - Routing table count equals exactly 2 (`routing_table_get_count() == 2`).
   - FreeRTOS heap memory on Coordinator has not decreased from baseline.

#### Pass/Fail Criteria
- **Pass:**
  - Coordinator routing table count remains strictly equal to the number of physical nodes (no orphan/duplicate entries).
  - All 15 flapping cycles achieve route re-establishment within $< 1.5$ seconds of node boot.
  - Coordinator heap delta $< 100$ bytes across the entire 15-cycle test (proving zero memory leak).

---

### Stress Test 5: Concurrent Announcement Collision & Backoff Jitter

#### Objective
Induce air contention between child nodes by triggering concurrent boots, verifying that transmission backoff jitter prevents continuous packet collision.

#### Method
1. Issue simultaneous reboot commands to `Kitchen` and `Garage` with a minimal artificial host delay ($0-20\text{ ms}$).
2. Both nodes boot their RF stacks at approximately the same millisecond and attempt to transmit their Layer 1 broadcast announcements (`ANNOUNCE`).
3. Monitor Coordinator RX timeline:
   - Measure time delta between `Kitchen` and `Garage` announcement reception.
   - If a collision occurs, verify that the Coordinator's automatic Layer 2 boot sweep (`DISCOVER_ALL`) successfully sweeps and recovers the collided node.
4. Execute 10 iterations of concurrent boot races.

#### Pass/Fail Criteria
- **Pass:**
  - In $100\%$ of iterations, both nodes successfully register in the Coordinator table within $< 2.5$ seconds of boot.
  - Zero deadlock or missing node states resulting from announcement packet collisions.

---

### Stress Test 6: Long-Duration Soak & FreeRTOS Heap Profiling

#### Objective
Subject the mesh to continuous background operation over an extended duration (1 to 12 hours) to identify memory leaks, FreeRTOS task degradation, and environmental RF drift.

#### Method
1. Continuous round-robin ping loop at 1.0 Hz across all nodes.
2. Continuous logging of:
   - Timestamped RTT latency (min, average, median, 95th percentile, max).
   - Local LQI and Remote LQI trends over time.
   - Cumulative packet count, success count, failure count, and timeout count.
   - Coordinator heap high-water mark periodically sampled.
3. Configurable test durations:
   - Quick Soak: 30 minutes (pre-release sanity check).
   - Standard Soak: 2 hours (regression verification).
   - Extended Soak: 8–12 hours (overnight stability verification).

#### Pass/Fail Criteria
- **Pass:**
  - Cumulative Packet Delivery Ratio (PDR) $\ge 99.5\%$.
  - Free heap slope over time is flat (zero continuous downward drift).
  - P95 latency remains $< 50\text{ ms}$ throughout the duration.
  - Zero node disconnects or unrecovered network dropouts.

---

## 4. Architectural Design of `MeshStressTest.py`

### Design Principles
1. **Complete Decoupling:** `MeshStressTest.py` is independent of `test_runner.py`. It does not require test fixtures (e.g., Trinket M0, DAC, ADC, Schmitt shields) and focuses strictly on Zigbee / ESP-NOW radio mesh mechanics.
2. **Modular Test Runners:** Each stress test can be run individually via CLI flags or combined into an automated stress battery.
3. **Structured Metrics Logging:** Outputs real-time ANSI terminal progress tables and exports structured CSV / JSON run summaries containing latency percentiles, LQI histograms, and PDR statistics.

### Command-Line Interface Specification

```bash
# Run the entire stress test battery (Tests 1 through 5)
python MeshStressTest.py -p COM11 --all-stress

# Run specific stress tests individually
python MeshStressTest.py -p COM11 --flood --rate 20 --burst 100
python MeshStressTest.py -p COM11 --eviction
python MeshStressTest.py -p COM11 --rf-margin
python MeshStressTest.py -p COM11 --churn --cycles 15
python MeshStressTest.py -p COM11 --collision-race --iterations 10

# Run long-duration soak test
python MeshStressTest.py -p COM11 --soak --duration-min 120 --rate 1.0

# Optional hardware reset COM ports for end nodes
python MeshStressTest.py -p COM11 --churn --kitchen-port COM5 --garage-port COM8
```

### Module Structure for `MeshStressTest.py`

```
MeshStressTest.py
├── class MeshCoordinatorClient (Serial engine, async reader, auto-reconnect)
├── class StressMetricsCollector (Latency, PDR, jitter, LQI history, JSON export)
├── module test_rate_flood (High-rate packet injection & backpressure test)
├── module test_route_eviction (ResetNetwork route table cache eviction test)
├── module test_rf_margin (RF switch & link asymmetry test)
├── module test_node_churn (Rapid repetitive reboot flapping test)
├── module test_collision_race (Concurrent boot announcement race test)
└── module test_soak_runner (Long-duration 1-12h soak & stability profiler)
```

---

## 5. Summary Matrix & Acceptance Criteria

| Stress Test | Command Flag | Duration | Key Acceptance Metric |
| :--- | :--- | :--- | :--- |
| **1. Rate Flood** | `--flood` | ~1–2 min | PDR $\ge 95\%$ @ 20Hz, queue full gracefully handled |
| **2. Route Eviction** | `--eviction` | ~30 sec | 100% command delivery via Layer 3 broadcast fallback |
| **3. RF Margin** | `--rf-margin` | ~1 min | Asymmetry tracked, zero drop in OK/MARGINAL zones |
| **4. Node Churn** | `--churn` | ~3–5 min | Table count strictly 2, zero duplicate MACs, zero heap leak |
| **5. Collision Race** | `--collision-race` | ~2–3 min | 10/10 iterations recover both nodes in $< 2.5$s |
| **6. Soak Profiling** | `--soak` | 30–720 min | PDR $\ge 99.5\%$, P95 RTT $< 50$ms, flat heap profile |

---
*Next Step: Implementation of `MeshStressTest.py` based on this specification.*
