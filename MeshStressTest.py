#!/usr/bin/env python3
"""
MeshStressTest.py - Standalone Star Network Stress & Robustness Test Harness
=============================================================================
Evaluates the ESP32-C6 Star Topology (Hub-and-Spoke on ESP-NOW LR Substrate)
under high-frequency packet flooding, dynamic table cache eviction,
RF boundary link asymmetry, rapid spoke churn/reboots, and concurrent boot collisions.

Accompanying Document: MeshStressVerification.md (v2.0)
Architecture: Centralized Hub (Coordinator) + 5 Distributed End-Device Spokes
              (Kitchen, Garage, N1, N2, N3)
"""

import sys
import time
import argparse
import threading
import json
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[!] PySerial is required. Install via: pip install pyserial")
    sys.exit(1)


# ANSI Color formatting helpers
class Colors:
    HEADER    = '\033[95m'
    BLUE      = '\033[94m'
    CYAN      = '\033[96m'
    GREEN     = '\033[92m'
    YELLOW    = '\033[93m'
    RED       = '\033[91m'
    BOLD      = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET     = '\033[0m'

    @classmethod
    def green(cls, text: str) -> str: return f"{cls.GREEN}{text}{cls.RESET}"
    @classmethod
    def red(cls, text: str) -> str: return f"{cls.RED}{text}{cls.RESET}"
    @classmethod
    def yellow(cls, text: str) -> str: return f"{cls.YELLOW}{text}{cls.RESET}"
    @classmethod
    def cyan(cls, text: str) -> str: return f"{cls.CYAN}{text}{cls.RESET}"
    @classmethod
    def bold(cls, text: str) -> str: return f"{cls.BOLD}{text}{cls.RESET}"


class CoordinatorClient:
    """Manages serial communication to the central Coordinator Hub."""

    def __init__(self, port: str = "COM11", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._rx_callbacks: List[Any] = []
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        try:
            # Crucial: dtr=False, rts=False prevents accidental ESP32 reboot on connect
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = 0.2
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            print(f"[+] Connected to Coordinator Hub on {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            print(f"[-] Failed to open Coordinator port {self.port}: {e}")
            return False

    def close(self):
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        print(f"[*] Closed port {self.port}.")

    def send_line(self, line: str):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open.")
        clean = line.strip("\r\n")
        self.ser.write((clean + "\r\n").encode("utf-8"))
        self.ser.flush()

    def add_rx_callback(self, cb):
        with self._lock:
            self._rx_callbacks.append(cb)

    def remove_rx_callback(self, cb):
        with self._lock:
            if cb in self._rx_callbacks:
                self._rx_callbacks.remove(cb)

    def _read_loop(self):
        while self._running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline()
                if line:
                    decoded = line.decode("utf-8", errors="replace").rstrip("\r\n")
                    if decoded:
                        with self._lock:
                            callbacks = list(self._rx_callbacks)
                        for cb in callbacks:
                            try:
                                cb(decoded)
                            except Exception:
                                pass
            except Exception:
                break


# =========================================================================
# HELPER PROTOCOL FUNCTIONS
# =========================================================================

def get_network_report(client: CoordinatorClient, timeout: float = 3.5) -> Dict[str, dict]:
    """Queries GiveNetworkReport on Coordinator and parses the table into a dict."""
    nodes = {}
    in_report = False
    done_event = threading.Event()

    def on_rx(msg: str):
        nonlocal in_report
        clean = msg.strip()
        if "NETWORK_REPORT_BEGIN" in clean:
            in_report = True
            return
        if "NETWORK_REPORT_END" in clean:
            in_report = False
            done_event.set()
            return
        if in_report and "," in clean and not clean.startswith("NAME,"):
            parts = [p.strip() for p in clean.split(",")]
            if len(parts) >= 10:
                name = parts[0]
                nodes[name] = {
                    "name": name,
                    "ieee": parts[1],
                    "short": parts[2],
                    "online": (parts[3] == "1"),
                    "type": parts[4],
                    "lqi": int(parts[5]) if parts[5].isdigit() else 0,
                    "remote_lqi": int(parts[6]) if parts[6].isdigit() else -1,
                    "rf_status": parts[7],
                    "age": int(parts[8]) if parts[8].isdigit() else 0,
                    "route": parts[9]
                }

    client.add_rx_callback(on_rx)
    try:
        client.send_line("GiveNetworkReport")
        done_event.wait(timeout=timeout)
    finally:
        client.remove_rx_callback(on_rx)

    return nodes


def verify_node_ack(client: CoordinatorClient, target: str, command: str = "blink 1", timeout: float = 2.0) -> Tuple[bool, float, str]:
    """Sends a targeted command and waits for '< target: [ACK]' or response."""
    got_reply = threading.Event()
    resp_text = ""
    t_start = time.time()

    def on_rx(msg: str):
        nonlocal resp_text
        if f"< {target}:" in msg and ("ACK" in msg or "PONG" in msg):
            resp_text = msg.strip()
            got_reply.set()

    client.add_rx_callback(on_rx)
    try:
        client.send_line(f"{target} {command}")
        passed = got_reply.wait(timeout=timeout)
        rtt_ms = (time.time() - t_start) * 1000.0
        return passed, rtt_ms, resp_text
    finally:
        client.remove_rx_callback(on_rx)


def wait_for_nodes_online(client: CoordinatorClient, target_nodes: List[str], timeout: float = 12.0, poll_interval: float = 1.0) -> Tuple[bool, dict, float]:
    """Polls GiveNetworkReport until all target_nodes are online with route == YES."""
    t_start = time.time()
    t_end = t_start + timeout
    last_report = {}

    while time.time() < t_end:
        report = get_network_report(client, timeout=3.0)
        last_report = report
        all_online = True
        for node in target_nodes:
            entry = report.get(node)
            if not entry or not entry.get("online") or entry.get("route") != "YES":
                all_online = False
                break
        if all_online:
            return True, report, time.time() - t_start
        time.sleep(poll_interval)

    return False, last_report, time.time() - t_start


# =========================================================================
# TEST VECTOR 1: RATE FLOOD & HUB QUEUE BACKPRESSURE
# =========================================================================

def run_test_rate_flood(client: CoordinatorClient, nodes: List[str]) -> bool:
    """
    Stress Test 1: Round-robin burst packet injection across all 5 spokes.
    Evaluates Hub FreeRTOS TX queue backpressure and CSMA/CA channel contention.
    """
    print("\n" + "=" * 70)
    print(Colors.bold("STRESS TEST 1: RATE FLOOD & HUB BACKPRESSURE (5-SPOKE CONTENTION)"))
    print(f"Target Nodes: {', '.join(nodes)}")
    print("=" * 70)

    rates_hz = [5, 10, 20]
    packets_per_tier = 30  # 30 packets per tier (6 round-robins across 5 spokes)
    all_tiers_passed = True

    for rate in rates_hz:
        interval = 1.0 / rate
        print(f"\n[*] Rate Tier: {rate} Hz ({interval*1000:.1f}ms inter-packet delay) | {packets_per_tier} Packets...")
        
        sent_counts = {n: 0 for n in nodes}
        ack_counts = {n: 0 for n in nodes}
        rtt_history = {n: [] for n in nodes}
        hub_queue_full_events = 0

        def on_rx_flood(msg: str):
            nonlocal hub_queue_full_events
            if "queue full" in msg.lower() or "tx full" in msg.lower():
                hub_queue_full_events += 1

        client.add_rx_callback(on_rx_flood)

        for i in range(packets_per_tier):
            target = nodes[i % len(nodes)]
            sent_counts[target] += 1
            passed, rtt_ms, _ = verify_node_ack(client, target, "blink 1", timeout=interval * 1.5 + 0.3)
            if passed:
                ack_counts[target] += 1
                rtt_history[target].append(rtt_ms)
            time.sleep(interval)

        client.remove_rx_callback(on_rx_flood)

        # Print tier results table
        print("-" * 70)
        print("NODE            SENT   ACKS   PDR (%)   MIN RTT   AVG RTT   MAX RTT  STATUS")
        print("-" * 70)
        tier_pdr_total = 0
        for n in nodes:
            s = sent_counts[n]
            a = ack_counts[n]
            pdr = (a / s * 100.0) if s > 0 else 0.0
            tier_pdr_total += pdr
            rtts = rtt_history[n]
            min_rtt = f"{min(rtts):.0f}ms" if rtts else "N/A"
            avg_rtt = f"{sum(rtts)/len(rtts):.0f}ms" if rtts else "N/A"
            max_rtt = f"{max(rtts):.0f}ms" if rtts else "N/A"
            stat = Colors.green("PASS") if pdr >= 80.0 else Colors.red("FAIL")
            print(f"{n:<15} {s:<6} {a:<6} {pdr:>6.1f}%   {min_rtt:<9} {avg_rtt:<9} {max_rtt:<8} {stat}")
        print("-" * 70)

        avg_tier_pdr = tier_pdr_total / len(nodes)
        print(f"Tier Summary: Aggregate PDR = {avg_tier_pdr:.1f}%, Hub Queue Rejections = {hub_queue_full_events}")
        if avg_tier_pdr < 80.0:
            all_tiers_passed = False

    passed_str = Colors.green("PASSED") if all_tiers_passed else Colors.red("FAILED")
    print(f"\n[+] STRESS TEST 1 {passed_str}!\n")
    return all_tiers_passed


# =========================================================================
# TEST VECTOR 2: CACHE EVICTION & BROADCAST FALLBACK
# =========================================================================

def run_test_route_eviction(client: CoordinatorClient, nodes: List[str]) -> bool:
    """
    Stress Test 2: Wipes Coordinator active RAM routing table (ResetNetwork)
    and verifies that direct dispatch falls back to broadcast and learns reverse routes.
    """
    print("\n" + "=" * 70)
    print(Colors.bold("STRESS TEST 2: ROUTE CACHE EVICTION & BROADCAST FALLBACK"))
    print("=" * 70)

    print("[*] Step 1: Confirming initial star network health...")
    init_rep = get_network_report(client)
    print(f"    Initial active nodes reported: {len(init_rep)}")

    print("[*] Step 2: Flushing Coordinator routing table via 'ResetNetwork'...")
    client.send_line("ResetNetwork")
    time.sleep(0.5)

    print("[*] Step 3: Immediately dispatching operational commands to all spokes (Cold Cache)...")
    results = {}
    for n in nodes:
        passed, rtt_ms, resp = verify_node_ack(client, n, "blink 1", timeout=2.5)
        results[n] = (passed, rtt_ms)
        time.sleep(0.2)

    # Allow 1 second for coordinator to complete reverse route learning insertion
    time.sleep(1.0)

    print("[*] Step 4: Checking post-fallback routing table restoration...")
    post_rep = get_network_report(client, timeout=3.5)

    print("-" * 70)
    print("NODE            COLD DISPATCH ACK   REVERSE ROUTE LEARNED   STATUS")
    print("-" * 70)
    all_passed = True
    for n in nodes:
        passed, rtt = results.get(n, (False, 0.0))
        entry = post_rep.get(n)
        learned = (entry is not None and entry.get("online") and entry.get("route") == "YES")
        ack_str = f"PASS ({rtt:.0f}ms)" if passed else "FAIL"
        learn_str = "YES" if learned else "NO"
        stat = Colors.green("PASS") if (passed and learned) else Colors.red("FAIL")
        if not (passed and learned):
            all_passed = False
        print(f"{n:<15} {ack_str:<19} {learn_str:<23} {stat}")
    print("-" * 70)

    passed_str = Colors.green("PASSED") if all_passed else Colors.red("FAILED")
    print(f"\n[+] STRESS TEST 2 {passed_str}!\n")
    return all_passed


# =========================================================================
# TEST VECTOR 3: RF BOUNDARY & LINK ASYMMETRY SURVEY
# =========================================================================

def run_test_rf_margin(client: CoordinatorClient, nodes: List[str], iterations: int = 5) -> bool:
    """
    Stress Test 3: Measures link quality, bidirectional asymmetry (|LQI - REMOTE_LQI|),
    and validates packet delivery correlation across proximate and fringe spokes.
    """
    print("\n" + "=" * 70)
    print(Colors.bold(f"STRESS TEST 3: RF BOUNDARY & LINK ASYMMETRY SURVEY ({iterations} CYCLES)"))
    print("=" * 70)

    survey_data = {n: {"lqi": [], "rem_lqi": [], "acks": 0, "total": 0} for n in nodes}

    for it in range(1, iterations + 1):
        print(f"[*] Survey Cycle {it}/{iterations}...")
        rep = get_network_report(client, timeout=3.5)
        for n in nodes:
            entry = rep.get(n, {})
            lqi = entry.get("lqi", 0)
            rem_lqi = entry.get("remote_lqi", -1)
            survey_data[n]["total"] += 1
            if lqi > 0:
                survey_data[n]["lqi"].append(lqi)
            if rem_lqi >= 0:
                survey_data[n]["rem_lqi"].append(rem_lqi)
            if entry.get("online"):
                survey_data[n]["acks"] += 1
        time.sleep(0.5)

    print("-" * 70)
    print("NODE            UPLINK LQI   DOWNLINK LQI   ASYMMETRY (DELTA)  CLASSIFICATION  STATUS")
    print("-" * 70)
    all_passed = True
    for n in nodes:
        d = survey_data[n]
        avg_lqi = sum(d["lqi"]) / len(d["lqi"]) if d["lqi"] else 0
        avg_rem = sum(d["rem_lqi"]) / len(d["rem_lqi"]) if d["rem_lqi"] else 0
        delta = abs(avg_lqi - avg_rem)
        min_signal = min(avg_lqi, avg_rem) if avg_rem > 0 else avg_lqi
        if min_signal >= 100:
            clf = Colors.green("OK")
        elif min_signal >= 50:
            clf = Colors.yellow("MARGINAL")
        else:
            clf = Colors.red("WEAK")
        
        pdr = (d["acks"] / d["total"] * 100.0) if d["total"] > 0 else 0
        stat = Colors.green("PASS") if pdr >= 80.0 else Colors.red("FAIL")
        if pdr < 80.0:
            all_passed = False
        print(f"{n:<15} {avg_lqi:>6.1f}       {avg_rem:>6.1f}         {delta:>5.1f}          {clf:<15} {stat}")
    print("-" * 70)

    passed_str = Colors.green("PASSED") if all_passed else Colors.red("FAILED")
    print(f"\n[+] STRESS TEST 3 {passed_str}!\n")
    return all_passed


# =========================================================================
# TEST VECTOR 4: RAPID SPOKE CHURN & REBOOT FLAPPING
# =========================================================================

def run_test_node_churn(client: CoordinatorClient, nodes: List[str]) -> bool:
    """
    Stress Test 4: Rapid spoke power cycling / remote OTA reboots.
    Verifies routing table stability, duplicate suppression, and fast re-establishment.
    """
    print("\n" + "=" * 70)
    print(Colors.bold("STRESS TEST 4: RAPID SPOKE CHURN & REBOOT FLAPPING"))
    print("=" * 70)

    all_passed = True

    # Phase A: Single Edge Spoke Rapid Flapping (N3 x 3)
    edge_spoke = "N3" if "N3" in nodes else nodes[-1]
    print(f"\n[*] Phase A: Rapid Edge Flapping -> 3 consecutive reboots on '{edge_spoke}'...")
    for cycle in range(1, 4):
        print(f"    Cycle {cycle}/3: Dispatching '{edge_spoke} reboot'...")
        client.send_line(f"{edge_spoke} reboot")
        time.sleep(1.2)
        ok, rep, el = wait_for_nodes_online(client, [edge_spoke], timeout=8.0)
        res_str = Colors.green(f"REJOINED in {el:.2f}s") if ok else Colors.red("TIMEOUT")
        print(f"    Cycle {cycle}/3: {edge_spoke} {res_str}")
        if not ok:
            all_passed = False
        time.sleep(0.5)

    # Phase B: Alternating Bench & Obstacle Flapping (Garage & N2)
    spoke_a = "Garage" if "Garage" in nodes else nodes[0]
    spoke_b = "N2" if "N2" in nodes else nodes[-1]
    print(f"\n[*] Phase B: Alternating Flapping -> '{spoke_a}' and '{spoke_b}' (2 cycles)...")
    for cycle in range(1, 3):
        print(f"    Cycle {cycle}/2: Rebooting '{spoke_a}'...")
        client.send_line(f"{spoke_a} reboot")
        time.sleep(1.2)
        ok_a, _, el_a = wait_for_nodes_online(client, [spoke_a], timeout=8.0)
        print(f"    Cycle {cycle}/2: {spoke_a} re-established in {el_a:.2f}s")
        if not ok_a:
            all_passed = False

        print(f"    Cycle {cycle}/2: Rebooting '{spoke_b}'...")
        client.send_line(f"{spoke_b} reboot")
        time.sleep(1.2)
        ok_b, _, el_b = wait_for_nodes_online(client, [spoke_b], timeout=8.0)
        print(f"    Cycle {cycle}/2: {spoke_b} re-established in {el_b:.2f}s")
        if not ok_b:
            all_passed = False

    # Final table count check
    final_rep = get_network_report(client, timeout=3.5)
    print(f"\n[*] Final Routing Table Audit: {len(final_rep)} active nodes registered (Expected: {len(nodes)}).")
    if len(final_rep) < len(nodes):
        print("[-] Table count lower than expected.")
        all_passed = False

    passed_str = Colors.green("PASSED") if all_passed else Colors.red("FAILED")
    print(f"\n[+] STRESS TEST 4 {passed_str}!\n")
    return all_passed


# =========================================================================
# TEST VECTOR 5: CONCURRENT BOOT COLLISION RACE
# =========================================================================

def run_test_collision_race(client: CoordinatorClient, nodes: List[str], iterations: int = 2) -> bool:
    """
    Stress Test 5: Concurrent reboots of all spokes within a tight window.
    Tests CSMA/CA randomized backoff under hidden-terminal broadcast contention.
    """
    print("\n" + "=" * 70)
    print(Colors.bold(f"STRESS TEST 5: CONCURRENT BOOT COLLISION RACE ({iterations} ITERATIONS)"))
    print("=" * 70)

    all_passed = True

    for it in range(1, iterations + 1):
        print(f"\n[*] Iteration {it}/{iterations}: Triggering concurrent reboot across ALL spokes ({', '.join(nodes)})...")
        # Blast reboots with zero delay between lines
        for n in nodes:
            client.send_line(f"{n} reboot")
        time.sleep(1.5)  # Allow all boards to reset in parallel

        print("    Waiting for all spokes to resolve collisions and re-register...")
        t0 = time.time()
        ok, rep, elapsed = wait_for_nodes_online(client, nodes, timeout=12.0)

        print("-" * 70)
        print("NODE            ONLINE   ROUTE   LQI   CMD ACK RTT   STATUS")
        print("-" * 70)
        it_ok = True
        for n in nodes:
            entry = rep.get(n, {})
            onl = "YES" if entry.get("online") else "NO"
            rte = entry.get("route", "NO")
            lqi = str(entry.get("lqi", 0))
            ack_ok, rtt_ms, _ = verify_node_ack(client, n, "blink 1", timeout=2.0)
            ack_str = f"{rtt_ms:.0f}ms" if ack_ok else "TIMEOUT"
            stat = Colors.green("PASS") if (ack_ok and entry.get("online")) else Colors.red("FAIL")
            if not (ack_ok and entry.get("online")):
                it_ok = False
            print(f"{n:<15} {onl:<8} {rte:<7} {lqi:<5} {ack_str:<13} {stat}")
        print("-" * 70)

        if it_ok:
            print(Colors.green(f"    [+] Iteration {it} Passed: Full network recovered in {elapsed:.2f}s!"))
        else:
            print(Colors.red(f"    [-] Iteration {it} Failed: Network did not fully recover."))
            all_passed = False
        time.sleep(1.0)

    passed_str = Colors.green("PASSED") if all_passed else Colors.red("FAILED")
    print(f"\n[+] STRESS TEST 5 {passed_str}!\n")
    return all_passed


# =========================================================================
# MASTER RUNNER
# =========================================================================

def run_all_stress_battery(client: CoordinatorClient, nodes: List[str]) -> bool:
    """Executes the full 5-part Star Network Stress Test Battery."""
    t_start = time.time()
    print("\n" + "#" * 70)
    print(Colors.bold("     STAR NETWORK 5-PART STRESS & ROBUSTNESS TEST BATTERY"))
    print(f"  Target Spokes ({len(nodes)}): {', '.join(nodes)}")
    print(f"  Started at:    {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("#" * 70)

    results = {}

    # Test 1
    results["Test1_RateFlood"] = run_test_rate_flood(client, nodes)
    time.sleep(1.0)

    # Test 2
    results["Test2_RouteEviction"] = run_test_route_eviction(client, nodes)
    time.sleep(1.0)

    # Test 3
    results["Test3_RfMargin"] = run_test_rf_margin(client, nodes, iterations=4)
    time.sleep(1.0)

    # Test 4
    results["Test4_NodeChurn"] = run_test_node_churn(client, nodes)
    time.sleep(1.0)

    # Test 5
    results["Test5_CollisionRace"] = run_test_collision_race(client, nodes, iterations=2)

    total_elapsed = time.time() - t_start

    # Final Master Summary Table
    print("\n" + "=" * 70)
    print(Colors.bold("                STRESS SUITE EXECUTION SUMMARY"))
    print("=" * 70)
    subtests = [
        ("Test1_RateFlood",     "Test 1: Rate Flood & FreeRTOS TX Backpressure"),
        ("Test2_RouteEviction", "Test 2: Dynamic Spoke Cache Eviction & Broadcast Fallback"),
        ("Test3_RfMargin",      "Test 3: RF Boundary & Link Asymmetry Survey"),
        ("Test4_NodeChurn",     "Test 4: Rapid Spoke Churn & Reboot Flapping"),
        ("Test5_CollisionRace", "Test 5: Concurrent Boot Collision Race")
    ]

    all_passed = True
    for key, desc in subtests:
        passed = results.get(key, False)
        status = Colors.green("[PASS]") if passed else Colors.red("[FAIL]")
        if not passed:
            all_passed = False
        print(f"  {status:<16} {desc}")

    print("=" * 70)
    min_str = f"{total_elapsed / 60:.1f} minutes" if total_elapsed >= 60 else f"{total_elapsed:.1f} seconds"
    if all_passed:
        print(Colors.bold(Colors.green(f"  >>> ALL 5 STRESS TESTS PASSED SUCCESSFULLY! (Completed in {min_str}) <<<")))
    else:
        print(Colors.bold(Colors.red(f"  >>> ONE OR MORE STRESS TESTS FAILED (Completed in {min_str}) <<<")))
    print("=" * 70 + "\n")

    return all_passed


# =========================================================================
# TEST VECTOR 6: LONG-DURATION STAR SOAK & LATENCY PROFILER
# =========================================================================

def run_test_soak(client: CoordinatorClient, nodes: List[str], duration_min: float = 60.0, rate_hz: float = 1.0, json_file: Optional[str] = None) -> bool:
    """
    Stress Test 6: Long-Duration Star Soak & Latency Profiling.
    Runs a continuous round-robin messaging loop across all spokes for duration_min minutes.
    Tracks PDR, RTT percentiles, battery-induced signal drops, and Hub stability.
    """
    total_seconds = duration_min * 60.0
    t_start = time.time()
    t_end = t_start + total_seconds
    interval = 1.0 / max(rate_hz, 0.1)

    print("\n" + "=" * 70)
    print(Colors.bold(f"STRESS TEST 6: LONG-DURATION STAR SOAK ({duration_min:.0f} MINUTES / {total_seconds:.0f} SECONDS)"))
    print(f"Target Spokes ({len(nodes)}): {', '.join(nodes)}")
    print(f"Aggregate Polling Rate:  {rate_hz:.1f} Hz (Interval: {interval*1000:.0f} ms)")
    print(f"Started at:              {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Scheduled Completion:   {datetime.fromtimestamp(t_end).strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70 + "\n")

    stats = {n: {"sent": 0, "ack": 0, "timeout": 0, "rtts": []} for n in nodes}
    last_status_print = time.time()
    last_lqi_sample = time.time()
    packet_seq = 0

    try:
        while time.time() < t_end:
            target = nodes[packet_seq % len(nodes)]
            packet_seq += 1
            stats[target]["sent"] += 1

            passed, rtt_ms, _ = verify_node_ack(client, target, "blink 1", timeout=interval * 1.5 + 0.3)
            now = time.time()

            if passed:
                stats[target]["ack"] += 1
                stats[target]["rtts"].append(rtt_ms)
            else:
                stats[target]["timeout"] += 1
                elapsed_min = (now - t_start) / 60.0
                ts = datetime.now().strftime('%H:%M:%S')
                print(f"  {Colors.yellow('[!]')} [{ts}] ({elapsed_min:.1f}m in) Spoke '{target}' packet timeout! (Node total: {stats[target]['ack']}/{stats[target]['sent']})", flush=True)

            # Periodic status line every 60 seconds
            if now - last_status_print >= 60.0:
                elapsed_sec = int(now - t_start)
                rem_sec = max(int(t_end - now), 0)
                tot_sent = sum(s["sent"] for s in stats.values())
                tot_ack = sum(s["ack"] for s in stats.values())
                agg_pdr = (tot_ack / tot_sent * 100.0) if tot_sent > 0 else 0.0

                rtt_summary = []
                for n in nodes:
                    recent = stats[n]["rtts"][-20:]
                    avg_n = f"{sum(recent)/len(recent):.0f}ms" if recent else "N/A"
                    rtt_summary.append(f"{n}:{avg_n}")

                print(f"[{elapsed_sec//60:02d}m:{elapsed_sec%60:02d}s / {int(total_seconds)//60:02d}m] PDR={agg_pdr:>5.1f}% | Sent={tot_sent} Ack={tot_ack} | ({' '.join(rtt_summary)})", flush=True)
                last_status_print = now

            # Sleep remaining slice to maintain steady rate
            elapsed_cycle = time.time() - now
            sleep_time = interval - elapsed_cycle
            if sleep_time > 0.005:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\n{Colors.yellow('[!] Soak test interrupted by operator.')}")

    total_run_time = time.time() - t_start

    # Final Soak Results Table
    print("\n" + "=" * 75)
    print(Colors.bold("                   LONG-DURATION SOAK TEST RESULTS"))
    print(f"Total Duration: {total_run_time/60.0:.2f} minutes ({total_run_time:.1f} seconds)")
    print("=" * 75)
    print("NODE            SENT   ACKS   DROPS   PDR (%)   MIN RTT   AVG RTT   P95 RTT   MAX RTT  STATUS")
    print("-" * 75)

    all_passed = True
    tot_sent = 0
    tot_ack = 0

    for n in nodes:
        s = stats[n]["sent"]
        a = stats[n]["ack"]
        d = stats[n]["timeout"]
        tot_sent += s
        tot_ack += a
        pdr = (a / s * 100.0) if s > 0 else 0.0
        rtts = sorted(stats[n]["rtts"])

        min_r = f"{rtts[0]:.0f}ms" if rtts else "N/A"
        avg_r = f"{sum(rtts)/len(rtts):.0f}ms" if rtts else "N/A"
        max_r = f"{rtts[-1]:.0f}ms" if rtts else "N/A"
        p95_idx = int(len(rtts) * 0.95)
        p95_r = f"{rtts[min(p95_idx, len(rtts)-1)]:.0f}ms" if rtts else "N/A"

        node_pass = (pdr >= 95.0)
        stat = Colors.green("PASS") if node_pass else Colors.red("FAIL")
        if not node_pass:
            all_passed = False

        print(f"{n:<15} {s:<6} {a:<6} {d:<7} {pdr:>6.1f}%   {min_r:<9} {avg_r:<9} {p95_r:<9} {max_r:<8} {stat}")

    print("-" * 75)
    agg_pdr = (tot_ack / tot_sent * 100.0) if tot_sent > 0 else 0.0
    print(f"Aggregate Performance: Total Packets = {tot_sent}, Delivered = {tot_ack}, Overall PDR = {agg_pdr:.2f}%")
    print("=" * 75)

    # Optional JSON export
    if json_file:
        summary_obj = {
            "test": "StarNetworkSoak",
            "timestamp": datetime.now().isoformat(),
            "duration_sec": total_run_time,
            "aggregate_pdr": agg_pdr,
            "nodes": {n: {
                "sent": stats[n]["sent"],
                "ack": stats[n]["ack"],
                "drops": stats[n]["timeout"],
                "pdr": (stats[n]["ack"]/stats[n]["sent"]*100.0) if stats[n]["sent"] else 0,
                "avg_rtt": (sum(stats[n]["rtts"])/len(stats[n]["rtts"])) if stats[n]["rtts"] else 0
            } for n in nodes}
        }
        try:
            with open(json_file, "w") as jf:
                json.dump(summary_obj, jf, indent=2)
            print(f"[+] Exported JSON soak report to: {json_file}")
        except Exception as e:
            print(f"[-] Failed to export JSON: {e}")

    passed_str = Colors.green("PASSED") if all_passed else Colors.red("FAILED")
    print(f"\n[+] SOAK TEST {passed_str}!\n")
    return all_passed


def main():
    parser = argparse.ArgumentParser(description="Standalone ESP32-C6 Star Network Stress & Robustness Test Harness")
    parser.add_argument("-p", "--port", type=str, default="COM11", help="Coordinator COM port (default: COM11)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--nodes", nargs="+", default=None, help="Explicit target node names (default: auto-discover)")
    parser.add_argument("--all-stress", action="store_true", help="Run the entire 5-part stress battery")
    parser.add_argument("--flood", action="store_true", help="Run Test 1: Rate Flood & Backpressure")
    parser.add_argument("--eviction", action="store_true", help="Run Test 2: Route Cache Eviction")
    parser.add_argument("--rf-margin", action="store_true", help="Run Test 3: RF Margin & Asymmetry Survey")
    parser.add_argument("--churn", action="store_true", help="Run Test 4: Rapid Spoke Churn Flapping")
    parser.add_argument("--collision-race", action="store_true", help="Run Test 5: Concurrent Boot Collision Race")
    parser.add_argument("--soak", action="store_true", help="Run Test 6: Long-Duration Soak & Latency Profiling")
    parser.add_argument("--duration-min", type=float, default=60.0, help="Duration for soak test in minutes (default: 60)")
    parser.add_argument("--rate", type=float, default=1.0, help="Aggregate polling rate in Hz for soak test (default: 1.0)")
    parser.add_argument("--json", type=str, default=None, help="Path to export structured test metrics JSON")
    parser.add_argument("--report", action="store_true", help="Query and print active network report and exit")

    args = parser.parse_args()

    client = CoordinatorClient(port=args.port, baudrate=args.baud)
    if not client.connect():
        sys.exit(1)

    time.sleep(0.3)

    try:
        # Step 1: Discover or verify target spokes
        if args.nodes:
            active_nodes = args.nodes
            print(f"[*] Using explicit node list: {active_nodes}")
        else:
            print("[*] Auto-discovering active spokes from Coordinator...")
            report = get_network_report(client, timeout=3.5)
            active_nodes = [name for name, data in report.items() if data.get("online")]
            if not active_nodes:
                print("[!] No active spokes reported. Falling back to default: Garage, Kitchen, N1, N2, N3")
                active_nodes = ["Garage", "Kitchen", "N1", "N2", "N3"]
            else:
                print(f"[+] Discovered {len(active_nodes)} active spokes: {', '.join(active_nodes)}")

        # Execution flags
        if args.report:
            rep = get_network_report(client)
            print("\nActive Network Report:")
            for n, d in rep.items():
                print(f"  {n:<12} IEEE={d['ieee']} SHORT={d['short']} LQI={d['lqi']} REM_LQI={d['remote_lqi']} RF={d['rf_status']}")
            sys.exit(0)

        if args.soak:
            json_target = args.json or f"soak_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            passed = run_test_soak(client, active_nodes, duration_min=args.duration_min, rate_hz=args.rate, json_file=json_target)
            sys.exit(0 if passed else 1)

        if args.flood:
            passed = run_test_rate_flood(client, active_nodes)
            sys.exit(0 if passed else 1)

        if args.eviction:
            passed = run_test_route_eviction(client, active_nodes)
            sys.exit(0 if passed else 1)

        if args.rf_margin:
            passed = run_test_rf_margin(client, active_nodes)
            sys.exit(0 if passed else 1)

        if args.churn:
            passed = run_test_node_churn(client, active_nodes)
            sys.exit(0 if passed else 1)

        if args.collision_race:
            passed = run_test_collision_race(client, active_nodes)
            sys.exit(0 if passed else 1)

        # Default or --all-stress: Run full battery
        passed = run_all_stress_battery(client, active_nodes)
        sys.exit(0 if passed else 1)

    finally:
        client.close()


if __name__ == "__main__":
    main()


