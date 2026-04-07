"""
Klipper_communcation.py

Sends G-code commands to Klipper via Moonraker's HTTP API.
This works regardless of which user runs the script and gives real error feedback.

Usage as a module (from gcode_executor.py):
    from Klipper_communcation import KlipperComm
    klipper = KlipperComm()
    klipper.send_extrusion(e_mm=1.0, feedrate=600)
    klipper.close()

Usage standalone (quick test):
    python3 Klipper_communcation.py
"""

import urllib.request
import urllib.error
import json
import sys

# ─── Configuration ──────────────────────────────────────────────────────────
MOONRAKER_URL  = "http://localhost:7125"   # Moonraker API base URL
DEFAULT_FEEDRATE = 600                     # mm/min — extrusion feed rate
# ────────────────────────────────────────────────────────────────────────────


class KlipperComm:
    """Send G-code commands to Klipper via Moonraker HTTP API."""

    def __init__(self, moonraker_url: str = MOONRAKER_URL):
        self.url = moonraker_url.rstrip("/")
        self._verify_connection()

    def _verify_connection(self):
        """Check Moonraker is reachable and Klipper is ready."""
        try:
            resp = self._get("/printer/info")
            state = resp.get("result", {}).get("state", "unknown")
            print(f"[Klipper] Connected — Moonraker at {self.url}  (state: {state})")
        except Exception as e:
            print(f"[Klipper] WARNING: Could not reach Moonraker: {e}")
            print(f"[Klipper] Continuing — commands will be retried at send time.")

    def _get(self, path: str) -> dict:
        req = urllib.request.Request(f"{self.url}{path}")
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read().decode())

    def _post(self, path: str, payload: dict) -> dict:
        data = json.dumps(payload).encode()
        req = urllib.request.Request(
            f"{self.url}{path}",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read().decode())

    def send_command(self, gcode: str) -> bool:
        """Send a raw G-code string to Klipper. Returns True on success."""
        try:
            self._post("/printer/gcode/script", {"script": gcode})
            print(f"[Klipper] → {gcode}")
            return True
        except urllib.error.HTTPError as e:
            body = e.read().decode()
            print(f"[Klipper] ERROR {e.code}: {body}")
            return False
        except Exception as e:
            print(f"[Klipper] ERROR sending command: {e}")
            return False

    def send_extrusion(self, e_mm: float, feedrate: int = DEFAULT_FEEDRATE) -> bool:
        """
        Send a linear extrusion move to Klipper.
        e_mm      — amount of filament to push (mm)
        feedrate  — extrusion speed (mm/min)
        """
        cmd = f"_CLIENT_LINEAR_MOVE E={e_mm:.3f} F={feedrate}"
        return self.send_command(cmd)

    def send_extrusion_sequence(self, commands: list) -> bool:
        """
        Send multiple extrusion moves as a single G-code script (one HTTP request).
        commands — list of (e_mm, feedrate) tuples
        All moves are queued atomically into Klipper's move buffer in one round-trip,
        eliminating per-command HTTP overhead.
        Returns True on success.
        """
        if not commands:
            return True
        lines = [f"_CLIENT_LINEAR_MOVE E={e:.3f} F={fr:.0f}" for e, fr in commands]
        script = "\n".join(lines)
        try:
            self._post("/printer/gcode/script", {"script": script})
            print(f"[Klipper] → sequence of {len(commands)} extrusion moves queued")
            return True
        except urllib.error.HTTPError as e:
            body = e.read().decode()
            print(f"[Klipper] ERROR {e.code}: {body}")
            return False
        except Exception as e:
            print(f"[Klipper] ERROR sending sequence: {e}")
            return False

    def set_temperature(self, temp_c: int) -> bool:
        """Set hotend temperature (non-blocking)."""
        return self.send_command(f"M104 S{temp_c}")

    def close(self):
        """No-op — HTTP is stateless, nothing to close."""
        print("[Klipper] Connection closed.")


# ── Standalone test ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    klipper = KlipperComm()
    klipper.send_extrusion(e_mm=1.0, feedrate=600)
    klipper.close()