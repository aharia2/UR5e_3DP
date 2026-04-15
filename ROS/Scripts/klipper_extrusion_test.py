#!/usr/bin/env python3
"""
klipper_extrusion_test.py

Stress-tests Klipper extrusion by sending many small segments continuously
using threading. Tests whether segments can be pipelined fast enough to
prevent gaps in extrusion.

Four test modes:
  1. sequential  — one segment at a time, measures gap between completion and next start
  2. pingpong    — 2 threads alternate so the next segment is queued before the current finishes
  3. stress      — N threads all sending simultaneously (hammers the queue)
  4. preloaded   — entire profile assembled into ONE G-code script and sent to Klipper in a
                   single call; Klipper buffers everything internally and executes using its
                   own motion clock — most accurate, zero Python timing jitter between segments.
                   Optional G4 dwells enforce exact inter-segment spacing.

Usage:
    python3 klipper_extrusion_test.py [mode]
    python3 klipper_extrusion_test.py sequential
    python3 klipper_extrusion_test.py pingpong
    python3 klipper_extrusion_test.py stress
    python3 klipper_extrusion_test.py preloaded
    python3 klipper_extrusion_test.py waypoints
"""

import json
import os
import queue
import statistics
import sys
import threading
import time
import urllib.request

# ─── Connection settings (matches gcode_executor.py) ─────────────────────────
MOONRAKER_URL = "http://localhost:7125"

# ─── Tunable parameters ───────────────────────────────────────────────────────
SEGMENT_MM   = .1    # mm per segment (smaller = harder to keep up)
TOTAL_MM     = 50.0   # total filament to extrude across all segments
NUM_THREADS  = 4      # number of concurrent threads (stress mode only)
PREHEAT_TEMP = 0      # set > 0 to heat hotend before test (0 = skip)

# ─── Rate profile ─────────────────────────────────────────────────────────────
# Controls how feedrate (mm/min) changes across segments.
#
#   "constant"  — every segment at FEEDRATE_START, ignores FEEDRATE_END
#   "ramp_down" — linear decrease from FEEDRATE_START → FEEDRATE_END
#   "ramp_up"   — linear increase from FEEDRATE_START → FEEDRATE_END
#   "sine"      — sinusoidal oscillation between FEEDRATE_END and FEEDRATE_START
#   "custom"    — use the CUSTOM_FEEDRATES list (repeats if shorter than n_segments)
#
RATE_PROFILE    = "constant"   # choose profile type (see above)
FEEDRATE_START  = 600           # mm/min — starting feedrate (used by all profiles)
FEEDRATE_END    = 100           # mm/min — ending feedrate   (ramp/sine profiles)
SINE_PERIODS    = 2.0           # number of full sine cycles over all segments
CUSTOM_FEEDRATES = [600, 500, 400, 300, 200, 100]  # used only by "custom" profile

# ─── Preloaded mode settings ──────────────────────────────────────────────────
# SEGMENT_INTERVAL_MS controls inter-segment timing in "preloaded" mode.
#
#   0   — pack segments back-to-back with no gaps (fastest, Klipper decides spacing)
#   >0  — enforce a fixed interval (ms) between the START of each segment by inserting
#         a G4 dwell after each move.  If the segment itself takes longer than the
#         interval, the dwell is skipped (clamped to 0) so Klipper never stalls.
#
# Example: SEGMENT_MM=1, FEEDRATE_START=600 → segment takes 100 ms.
#   SEGMENT_INTERVAL_MS=150 → 50 ms dwell after each segment (150 ms cycle time)
#   SEGMENT_INTERVAL_MS=80  → no dwell (segment already takes 100 ms > 80 ms target)
#
SEGMENT_INTERVAL_MS = 0   # ms between segment starts (0 = back-to-back, no dwell)

# ─── Waypoints mode settings ──────────────────────────────────────────────────
# Paste a CSV path (from gcode_interpreter.py output) and optionally limit
# how many rows to execute — useful for testing a small segment first.
#
# CSV columns expected (auto-detected from header):
#   X, Y, Z, QX, QY, QZ, QW, Toolhead_Speed_mm_per_min,
#   Extrusion_Length_mm, Extrusion_Speed_mm_per_min, Move_Type, Segment_ID, GCode_Line
#
# Travel moves (Move_Type == 'travel') become G4 dwells so Klipper holds still
# while the robot arm is repositioning — this is what keeps extrusion in sync.
#
WAYPOINTS_CSV        = "gcode_interpreted files/3DBenchy.csv"  # path to CSV
WAYPOINTS_SEGMENT_ID = 467   # which Segment_ID to execute (single segment, back-to-back)
WAYPOINTS_FIRST_N    = 0     # limit rows within the segment (0 = all rows in segment)

# Chunking — splits the segment into groups whose total motion time >= CHUNK_TARGET_MS.
# Chunk boundaries are placed dynamically based on the actual per-move durations
# (e_delta / e_speed), so fast short moves get more moves per chunk and slow long
# moves get fewer — always just enough to cover the HTTP round-trip.
#
# CHUNK_TARGET_MS should be comfortably above your Moonraker HTTP round-trip latency.
# Measure it by running "sequential" mode and reading the overhead per segment.
# Typical values on localhost: 20–60 ms.  Set this to ~2× that for safety margin.
#
#   0 = send everything in one script (no chunking — risks "Timer too close")
#
CHUNK_TARGET_MS = 800    # ms of motion to buffer per chunk (0 = all at once)
# ─────────────────────────────────────────────────────────────────────────────


# ─── Klipper communication (mirrors gcode_executor.py) ───────────────────────

class KlipperComm:
    """Send G-code to Klipper via the Moonraker HTTP API."""

    def __init__(self, url: str = MOONRAKER_URL):
        self.url = url.rstrip('/')

    def connect(self) -> bool:
        try:
            with urllib.request.urlopen(f'{self.url}/printer/info', timeout=5) as r:
                info = json.loads(r.read())
            state = info.get('result', {}).get('state', 'unknown')
            print(f'[Klipper] Connected — Moonraker at {self.url}  (state: {state})')
            return True
        except Exception as e:
            print(f'[Klipper] WARNING: Could not reach Moonraker: {e}')
            return False

    def send_gcode(self, script: str) -> bool:
        try:
            data = json.dumps({'script': script}).encode()
            req = urllib.request.Request(
                f'{self.url}/printer/gcode/script',
                data=data,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=60) as r:
                r.read()
            return True
        except Exception as e:
            print(f'[Klipper] ERROR sending G-code: {e}')
            return False

    def send_extrusion(self, cumulative_e_mm: float, feedrate: float) -> bool:
        """Send absolute extrusion move (matches gcode_executor.py style)."""
        gcode = f'M82\nG1 E{cumulative_e_mm:.4f} F{feedrate:.4f}'
        return self.send_gcode(gcode)

    def set_temperature(self, temp_c: int) -> bool:
        return self.send_gcode(f'M104 S{temp_c}')

    def reset_extruder(self) -> bool:
        """Zero the extruder position before a test run."""
        return self.send_gcode('G92 E0')


# ─── Rate profile builder ─────────────────────────────────────────────────────

import math as _math

def build_rate_profile(n_segments: int) -> list:
    """
    Returns a list of feedrates (mm/min), one per segment, based on RATE_PROFILE.
    """
    if RATE_PROFILE == "constant":
        return [float(FEEDRATE_START)] * n_segments

    elif RATE_PROFILE == "ramp_down":
        if n_segments == 1:
            return [float(FEEDRATE_START)]
        return [
            FEEDRATE_START + (FEEDRATE_END - FEEDRATE_START) * (i / (n_segments - 1))
            for i in range(n_segments)
        ]

    elif RATE_PROFILE == "ramp_up":
        if n_segments == 1:
            return [float(FEEDRATE_END)]
        return [
            FEEDRATE_END + (FEEDRATE_START - FEEDRATE_END) * (1.0 - i / (n_segments - 1))
            for i in range(n_segments)
        ]

    elif RATE_PROFILE == "sine":
        mid   = (FEEDRATE_START + FEEDRATE_END) / 2.0
        amp   = (FEEDRATE_START - FEEDRATE_END) / 2.0
        return [
            mid + amp * _math.sin(2 * _math.pi * SINE_PERIODS * i / n_segments)
            for i in range(n_segments)
        ]

    elif RATE_PROFILE == "custom":
        src = CUSTOM_FEEDRATES
        return [float(src[i % len(src)]) for i in range(n_segments)]

    else:
        raise ValueError(f"Unknown RATE_PROFILE: '{RATE_PROFILE}'. "
                         f"Choose: constant, ramp_down, ramp_up, sine, custom")


# ─── Helpers ──────────────────────────────────────────────────────────────────

def print_banner(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def print_params(feedrates: list):
    n_segments     = len(feedrates)
    total_duration = sum((SEGMENT_MM / f) * 60.0 for f in feedrates)
    print(f"  Segment size    : {SEGMENT_MM} mm")
    print(f"  Total extrusion : {TOTAL_MM} mm  ({n_segments} segments)")
    print(f"  Rate profile    : {RATE_PROFILE}")
    print(f"  Feedrate range  : {min(feedrates):.1f} – {max(feedrates):.1f} mm/min")
    print(f"  Seg duration    : {min((SEGMENT_MM/f)*60*1000 for f in feedrates):.1f} – "
          f"{max((SEGMENT_MM/f)*60*1000 for f in feedrates):.1f} ms")
    print(f"  Expected total  : {total_duration:.2f} s")


def report_stats(label: str, latencies_ms: list, feedrates: list,
                 total_elapsed: float, n_segments: int):
    # Expected duration per segment based on its feedrate
    expected_ms_list = [(SEGMENT_MM / f) * 60.0 * 1000.0 for f in feedrates]

    print(f"\n--- {label} Results ---")
    print(f"  Segments sent   : {n_segments}")
    print(f"  Total elapsed   : {total_elapsed:.3f} s")
    if latencies_ms and len(latencies_ms) == len(expected_ms_list):
        overheads = [l - e for l, e in zip(latencies_ms, expected_ms_list)]
        print(f"  Per-segment latency (HTTP round-trip + Klipper exec):")
        print(f"    min    : {min(latencies_ms):.1f} ms")
        print(f"    max    : {max(latencies_ms):.1f} ms")
        print(f"    mean   : {statistics.mean(latencies_ms):.1f} ms")
        if len(latencies_ms) > 1:
            print(f"    stdev  : {statistics.stdev(latencies_ms):.1f} ms")
        print(f"  Overhead vs expected (mean): {statistics.mean(overheads):+.1f} ms")
        n_gaps = sum(1 for l, e in zip(latencies_ms, expected_ms_list) if l > e * 1.1)
        print(f"  Segments with >10% gap: {n_gaps}/{n_segments}")


# ─── Mode 1: Sequential ───────────────────────────────────────────────────────

def run_sequential(klipper: KlipperComm):
    print_banner("MODE 1: Sequential (one segment at a time)")
    n_segments = int(TOTAL_MM / SEGMENT_MM)
    feedrates  = build_rate_profile(n_segments)
    print_params(feedrates)
    input("\nPress ENTER to start, or Ctrl+C to abort: ")

    klipper.reset_extruder()
    latencies    = []
    cumulative_e = 0.0
    t_start      = time.perf_counter()

    for i, feedrate in enumerate(feedrates):
        cumulative_e += SEGMENT_MM
        t0 = time.perf_counter()
        ok = klipper.send_extrusion(cumulative_e, feedrate)
        t1 = time.perf_counter()
        elapsed_ms = (t1 - t0) * 1000.0
        latencies.append(elapsed_ms)
        status = "OK" if ok else "FAIL"
        print(f"  seg {i+1:4d}/{n_segments}  E={cumulative_e:7.3f} mm  "
              f"F={feedrate:6.1f}  {elapsed_ms:7.1f} ms  [{status}]")

    total_elapsed = time.perf_counter() - t_start
    report_stats("Sequential", latencies, feedrates, total_elapsed, n_segments)


# ─── Mode 2: Ping-pong (2 threads, double-buffered) ──────────────────────────

def run_pingpong(klipper: KlipperComm):
    """
    Two threads alternate sending segments. While one is blocking inside
    Klipper, the other sends the next segment so it arrives before the
    first finishes — eliminating gaps between segments.
    """
    print_banner("MODE 2: Ping-pong (2 threads, no-gap double-buffer)")
    n_segments = int(TOTAL_MM / SEGMENT_MM)
    feedrates  = build_rate_profile(n_segments)
    print_params(feedrates)
    avg_seg_ms = statistics.mean((SEGMENT_MM / f) * 60.0 * 1000.0 for f in feedrates)
    print(f"  Thread lookahead: ~{avg_seg_ms:.1f} ms avg  (1 segment)")
    input("\nPress ENTER to start, or Ctrl+C to abort: ")

    klipper.reset_extruder()
    latencies = []
    lat_lock  = threading.Lock()
    seg_queue = queue.Queue()
    errors    = []

    # Pre-fill queue with (index, cumulative_e, feedrate) tuples
    for i, feedrate in enumerate(feedrates):
        seg_queue.put((i, (i + 1) * SEGMENT_MM, feedrate))

    def worker(thread_id: int):
        while True:
            try:
                idx, cumulative_e, feedrate = seg_queue.get_nowait()
            except queue.Empty:
                break
            t0 = time.perf_counter()
            ok = klipper.send_extrusion(cumulative_e, feedrate)
            t1 = time.perf_counter()
            elapsed_ms = (t1 - t0) * 1000.0
            with lat_lock:
                latencies.append(elapsed_ms)
                if not ok:
                    errors.append(idx)
            print(f"  [T{thread_id}] seg {idx+1:4d}/{n_segments}  "
                  f"E={cumulative_e:7.3f} mm  F={feedrate:6.1f}  {elapsed_ms:7.1f} ms")
            seg_queue.task_done()

    t_start = time.perf_counter()
    threads = [threading.Thread(target=worker, args=(i,), daemon=True) for i in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    total_elapsed = time.perf_counter() - t_start
    if errors:
        print(f"  ERRORS on segments: {errors}")
    report_stats("Ping-pong", latencies, feedrates, total_elapsed, n_segments)


# ─── Mode 3: Stress (N threads) ───────────────────────────────────────────────

def run_stress(klipper: KlipperComm):
    """
    N threads each pull segments from a shared queue and send them as fast as
    possible. Measures contention, out-of-order risk, and max throughput.
    """
    print_banner(f"MODE 3: Stress ({NUM_THREADS} threads)")
    n_segments = int(TOTAL_MM / SEGMENT_MM)
    feedrates  = build_rate_profile(n_segments)
    print_params(feedrates)
    print(f"  Threads         : {NUM_THREADS}")
    print(f"  NOTE: segments may arrive out of order with many threads.")
    input("\nPress ENTER to start, or Ctrl+C to abort: ")

    klipper.reset_extruder()
    latencies  = []
    lat_lock   = threading.Lock()
    seg_queue  = queue.Queue()
    send_times = []  # (thread_id, seg_idx, t_start, t_end)
    errors     = []

    for i, feedrate in enumerate(feedrates):
        seg_queue.put((i, (i + 1) * SEGMENT_MM, feedrate))

    def worker(thread_id: int):
        while True:
            try:
                idx, cumulative_e, feedrate = seg_queue.get_nowait()
            except queue.Empty:
                break
            t0 = time.perf_counter()
            ok = klipper.send_extrusion(cumulative_e, feedrate)
            t1 = time.perf_counter()
            elapsed_ms = (t1 - t0) * 1000.0
            with lat_lock:
                latencies.append(elapsed_ms)
                send_times.append((thread_id, idx, t0, t1))
                if not ok:
                    errors.append(idx)
            print(f"  [T{thread_id}] seg {idx+1:4d}/{n_segments}  "
                  f"E={cumulative_e:7.3f} mm  F={feedrate:6.1f}  {elapsed_ms:7.1f} ms")
            seg_queue.task_done()

    t_start = time.perf_counter()
    threads = [threading.Thread(target=worker, args=(i,), daemon=True)
               for i in range(NUM_THREADS)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    total_elapsed = time.perf_counter() - t_start

    # Check how many adjacent sends were out of order by start time
    send_times.sort(key=lambda x: x[2])
    ordered_idx = [s[1] for s in send_times]
    out_of_order = sum(1 for a, b in zip(ordered_idx, ordered_idx[1:]) if b < a)

    if errors:
        print(f"  ERRORS on segments: {errors}")
    print(f"  Out-of-order sends: {out_of_order}/{n_segments-1} adjacent pairs")
    report_stats("Stress", latencies, feedrates, total_elapsed, n_segments)


# ─── Mode 4: Preloaded (single batched G-code script) ────────────────────────

def run_preloaded(klipper: KlipperComm):
    """
    Assembles the entire rate profile into one G-code script and sends it to
    Klipper in a single HTTP call.  Klipper buffers all moves in its internal
    lookahead queue and executes them using its own motion clock — there is
    zero Python scheduling jitter between segments.

    Optional G4 dwells (SEGMENT_INTERVAL_MS > 0) let you enforce a precise
    wall-clock interval between segment starts entirely within Klipper.
    """
    print_banner("MODE 4: Preloaded (single batched G-code, Klipper-timed)")
    n_segments = int(TOTAL_MM / SEGMENT_MM)
    feedrates  = build_rate_profile(n_segments)
    print_params(feedrates)

    total_expected_s = sum((SEGMENT_MM / f) * 60.0 for f in feedrates)
    if SEGMENT_INTERVAL_MS > 0:
        # Each segment gets its own fixed interval slot
        total_expected_s = n_segments * SEGMENT_INTERVAL_MS / 1000.0
        print(f"  Interval mode   : {SEGMENT_INTERVAL_MS} ms per segment "
              f"({n_segments * SEGMENT_INTERVAL_MS / 1000.0:.2f} s total)")
    else:
        print(f"  Interval mode   : back-to-back (no dwells)")

    # ── Build the G-code script ───────────────────────────────────────────────
    lines = ["G92 E0", "M82"]  # zero extruder, absolute mode

    cumulative_e = 0.0
    for i, feedrate in enumerate(feedrates):
        cumulative_e += SEGMENT_MM
        lines.append(f"G1 E{cumulative_e:.4f} F{feedrate:.4f}  ; seg {i+1}/{n_segments}")

        if SEGMENT_INTERVAL_MS > 0:
            seg_duration_ms = (SEGMENT_MM / feedrate) * 60.0 * 1000.0
            dwell_ms = int(max(0.0, SEGMENT_INTERVAL_MS - seg_duration_ms))
            if dwell_ms > 0:
                lines.append(f"G4 P{dwell_ms}  ; dwell to fill {SEGMENT_INTERVAL_MS} ms slot")

    script = "\n".join(lines)
    n_lines = len(lines)

    print(f"\n  Script size     : {n_segments} move lines  ({n_lines} total G-code lines)")
    print(f"  Script length   : {len(script)} bytes")
    print(f"  Expected runtime: {total_expected_s:.2f} s")

    # Show a preview of the first and last few lines
    preview = lines[:4] + (["  ..."] if n_lines > 8 else []) + lines[-3:]
    print(f"\n  Preview:")
    for l in preview:
        print(f"    {l}")

    input("\nPress ENTER to send to Klipper, or Ctrl+C to abort: ")

    klipper.reset_extruder()
    t_start = time.perf_counter()
    ok = klipper.send_gcode(script)
    total_elapsed = time.perf_counter() - t_start

    status = "OK" if ok else "FAILED"
    print(f"\n  [{status}] Klipper finished in {total_elapsed:.3f} s  "
          f"(expected ~{total_expected_s:.2f} s)")
    if ok and total_expected_s > 0:
        overhead_ms = (total_elapsed - total_expected_s) * 1000.0
        print(f"  Total overhead  : {overhead_ms:+.1f} ms")


# ─── Mode 5: Waypoints (from gcode_interpreter CSV) ──────────────────────────

def _load_waypoints(csv_path: str) -> list:
    """Parse the gcode_interpreter CSV into a list of row dicts.
    Each row includes motion_time_ms and extrusion_time_ms so they can be compared.
    """
    import csv as _csv
    rows = []
    prev_x, prev_y, prev_z = None, None, None
    with open(csv_path, 'r') as fh:
        for raw in _csv.reader(fh):
            row = [c.strip() for c in raw]
            if not row or row[0].startswith('#'):
                continue
            if row[0].upper() == 'X':
                continue
            if len(row) < 13:
                continue
            try:
                x, y, z        = float(row[0]), float(row[1]), float(row[2])
                tool_speed     = float(row[7])
                e_delta        = float(row[8])
                e_speed        = float(row[9])

                # Motion time: distance from previous waypoint / toolhead speed
                if prev_x is not None and tool_speed > 0:
                    dx = x - prev_x
                    dy = y - prev_y
                    dz = z - prev_z
                    dist_mm = _math.sqrt(dx*dx + dy*dy + dz*dz)
                    motion_time_ms = (dist_mm / tool_speed) * 60.0 * 1000.0
                else:
                    dist_mm = 0.0
                    motion_time_ms = 0.0

                # Extrusion time: e_delta / e_speed
                extrusion_time_ms = (e_delta / e_speed) * 60.0 * 1000.0 if e_speed > 0 else 0.0

                rows.append({
                    'x':                  x,
                    'y':                  y,
                    'z':                  z,
                    'tool_speed':         tool_speed,
                    'e_delta':            e_delta,
                    'e_speed':            e_speed,
                    'move_type':          row[10].strip().lower(),
                    'segment_id':         int(float(row[11])),
                    'gcode_line':         int(float(row[12])),
                    'dist_mm':            dist_mm,
                    'motion_time_ms':     motion_time_ms,
                    'extrusion_time_ms':  extrusion_time_ms,
                })
                prev_x, prev_y, prev_z = x, y, z
            except (ValueError, IndexError):
                pass
    return rows


def run_waypoints(klipper: KlipperComm):
    """
    Reads a single Segment_ID from the gcode_interpreter CSV and sends all
    its extrusion waypoints as one preloaded G-code script.

    Within a segment, all rows are consecutive extrusion moves — no travel,
    no G4 dwells, just back-to-back G1 E commands.  Klipper buffers them all
    and executes using its internal clock.
    """
    print_banner("MODE 5: Waypoints (single segment, preloaded)")

    # ── Resolve CSV path ─────────────────────────────────────────────────────
    csv_path = WAYPOINTS_CSV
    if not os.path.isabs(csv_path):
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), csv_path)
    if not os.path.isfile(csv_path):
        print(f"ERROR: CSV not found: {csv_path}")
        return

    all_rows = _load_waypoints(csv_path)
    if not all_rows:
        print("ERROR: No valid rows found in CSV.")
        return

    # Filter to the requested segment
    seg_rows = [r for r in all_rows
                if r['segment_id'] == WAYPOINTS_SEGMENT_ID
                and r['move_type'] == 'extrusion'
                and r['e_delta'] > 1e-9]

    if not seg_rows:
        print(f"ERROR: No extrusion rows found for Segment_ID={WAYPOINTS_SEGMENT_ID}")
        available = sorted(set(r['segment_id'] for r in all_rows))
        print(f"  Available segment IDs: {available[:20]}{'...' if len(available)>20 else ''}")
        return

    if WAYPOINTS_FIRST_N > 0:
        seg_rows = seg_rows[:WAYPOINTS_FIRST_N]

    e_speeds         = [r['e_speed'] for r in seg_rows if r['e_speed'] > 0]
    total_expected_s = sum((r['e_delta'] / r['e_speed']) * 60.0
                           for r in seg_rows if r['e_speed'] > 0)
    avg_move_ms      = (total_expected_s / len(seg_rows)) * 1000.0 if seg_rows else 0

    # ── Build move list and time-based chunks ─────────────────────────────────
    # Each move carries its duration so chunks are sized by time, not count.
    move_lines = []
    cumulative_e = 0.0
    for r in seg_rows:
        cumulative_e += r['e_delta']
        feedrate      = r['e_speed'] if r['e_speed'] > 0 else FEEDRATE_START
        duration_ms   = (r['e_delta'] / feedrate) * 60.0 * 1000.0
        move_lines.append((cumulative_e, feedrate, r['gcode_line'], duration_ms))

    # Group moves into chunks whose total duration >= CHUNK_TARGET_MS.
    # With CHUNK_TARGET_MS=0, everything goes into one chunk.
    chunks = []
    if CHUNK_TARGET_MS <= 0:
        chunks.append(move_lines)
    else:
        current, current_ms = [], 0.0
        for move in move_lines:
            current.append(move)
            current_ms += move[3]  # duration_ms
            if current_ms >= CHUNK_TARGET_MS:
                chunks.append(current)
                current, current_ms = [], 0.0
        if current:
            chunks.append(current)

    chunk_durations_ms = [sum(m[3] for m in c) for c in chunks]

    print(f"  CSV             : {csv_path}")
    print(f"  Segment ID      : {WAYPOINTS_SEGMENT_ID}")
    print(f"  Waypoints       : {len(seg_rows)}")
    print(f"  Total extrusion : {sum(r['e_delta'] for r in seg_rows):.3f} mm")
    print(f"  Feedrate range  : {min(e_speeds):.1f} – {max(e_speeds):.1f} mm/min")
    print(f"  Avg move time   : {avg_move_ms:.1f} ms/move")
    print(f"  Expected runtime: {total_expected_s:.3f} s")
    if CHUNK_TARGET_MS > 0:
        print(f"  Chunk target    : {CHUNK_TARGET_MS} ms  →  "
              f"{len(chunks)} chunks  "
              f"({min(len(c) for c in chunks)}–{max(len(c) for c in chunks)} moves, "
              f"{min(chunk_durations_ms):.0f}–{max(chunk_durations_ms):.0f} ms each)")
    else:
        print(f"  Chunk mode      : all at once ({len(move_lines)} moves)")

    # ── Per-waypoint timing comparison ───────────────────────────────────────
    diffs_ms = [r['extrusion_time_ms'] - r['motion_time_ms'] for r in seg_rows]
    max_diff  = max(abs(d) for d in diffs_ms) if diffs_ms else 0.0
    print(f"\n  {'GCode':>6}  {'Dist':>7}  {'Motion':>9}  {'Extrude':>9}  {'Diff':>8}")
    print(f"  {'Line':>6}  {'mm':>7}  {'time ms':>9}  {'time ms':>9}  {'ms':>8}")
    print(f"  {'-'*6}  {'-'*7}  {'-'*9}  {'-'*9}  {'-'*8}")
    for r, (_, _, _, dur_ms) in zip(seg_rows, move_lines):
        diff = r['extrusion_time_ms'] - r['motion_time_ms']
        print(f"  {r['gcode_line']:>6}  {r['dist_mm']:>7.3f}  "
              f"{r['motion_time_ms']:>9.3f}  {r['extrusion_time_ms']:>9.3f}  "
              f"{diff:>+8.3f}")

    total_motion_s   = sum(r['motion_time_ms'] for r in seg_rows) / 1000.0
    print(f"\n  Timing summary:")
    print(f"    Total motion time   : {total_motion_s:.3f} s")
    print(f"    Total extrusion time: {total_expected_s:.3f} s")
    print(f"    Max per-waypoint diff: {max_diff:.3f} ms")

    input("\nPress ENTER to send to Klipper, or Ctrl+C to abort: ")

    klipper.reset_extruder()
    t_wall_start = time.perf_counter()
    errors       = 0
    log_lock     = threading.Lock()
    log_entries  = []   # (chunk_idx, n_moves, expected_ms, actual_ms)

    def _send_chunk(chunk_idx, chunk):
        script = "\n".join(
            ["M82"] + [f"G1 E{e:.4f} F{f:.4f}  ; gcode line {gl}"
                       for e, f, gl, _ in chunk]
        )
        expected_ms = sum(m[3] for m in chunk)
        t0 = time.perf_counter()
        ok = klipper.send_gcode(script)
        t1 = time.perf_counter()
        with log_lock:
            log_entries.append((chunk_idx, len(chunk), expected_ms,
                                (t1 - t0) * 1000.0, ok))

    # ── Ping-pong execution ───────────────────────────────────────────────────
    # Two threads alternate.  Thread A sends chunk N while Thread B is still
    # blocking on chunk N-1.  By the time Klipper finishes N-1, chunk N has
    # already been received and is sitting in the move queue — no gap.
    #
    # The key requirement: chunk duration > HTTP round-trip (CHUNK_TARGET_MS).
    # If that holds, chunk N always arrives before N-1 finishes.

    threads = [None, None]
    for chunk_idx, chunk in enumerate(chunks):
        slot = chunk_idx % 2

        # Wait for the previous use of this slot to finish before reusing it
        if threads[slot] is not None:
            threads[slot].join()

        t = threading.Thread(target=_send_chunk, args=(chunk_idx, chunk), daemon=True)
        threads[slot] = t
        t.start()

    # Wait for both slots to drain
    for t in threads:
        if t is not None:
            t.join()

    total_elapsed = time.perf_counter() - t_wall_start

    # Print log in order
    log_entries.sort(key=lambda x: x[0])
    for chunk_idx, n_moves, expected_ms, actual_ms, ok in log_entries:
        if not ok:
            errors += 1
            print(f"  [CHUNK {chunk_idx+1}/{len(chunks)}] FAILED")
        else:
            print(f"  [CHUNK {chunk_idx+1}/{len(chunks)}]  "
                  f"{n_moves:3d} moves  "
                  f"target {expected_ms:5.0f} ms  "
                  f"actual {actual_ms:5.0f} ms  "
                  f"overhead {actual_ms - expected_ms:+.0f} ms")

    overhead_ms = (total_elapsed - total_expected_s) * 1000.0
    print(f"\n  {'OK' if not errors else f'{errors} ERRORS'}  "
          f"Total: {total_elapsed:.3f} s  (expected {total_expected_s:.3f} s  "
          f"overhead {overhead_ms:+.1f} ms)")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    mode = sys.argv[1].lower() if len(sys.argv) > 1 else "pingpong"
    valid_modes = ("sequential", "pingpong", "stress", "preloaded", "waypoints")
    if mode not in valid_modes:
        print(f"Unknown mode '{mode}'. Choose from: {valid_modes}")
        sys.exit(1)

    print_banner("Klipper Extrusion Threading Test")
    print(f"  Mode: {mode}")

    klipper = KlipperComm()
    if not klipper.connect():
        print("ERROR: Could not connect to Moonraker. Is Klipper running?")
        sys.exit(1)

    if PREHEAT_TEMP > 0:
        print(f"\nHeating hotend to {PREHEAT_TEMP}°C...")
        klipper.set_temperature(PREHEAT_TEMP)
        print("  (Not waiting for temp — adjust PREHEAT_TEMP logic if needed)")

    try:
        if mode == "sequential":
            run_sequential(klipper)
        elif mode == "pingpong":
            run_pingpong(klipper)
        elif mode == "stress":
            run_stress(klipper)
        elif mode == "preloaded":
            run_preloaded(klipper)
        elif mode == "waypoints":
            run_waypoints(klipper)
    except KeyboardInterrupt:
        print("\n\nAborted by user.")


if __name__ == "__main__":
    main()
