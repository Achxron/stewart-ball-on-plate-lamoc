#!/usr/bin/env python3
"""
Stewart platform telemetry logger + command sender.

Captures CSV telemetry streaming from the Pico over USB serial, saves it
to a timestamped file, and optionally renders summary plots. Designed for
post-experiment analysis in a paper.

Three modes of operation:

1. CAPTURE ONLY (original behavior)
   python3 telemetry_logger.py --port /dev/ttyACM0 --name idle --duration 30

2. INTERACTIVE — type commands while logging
   python3 telemetry_logger.py --port /dev/ttyACM0 --name tuning --interactive
   ... then type "MODE ATT", "POSE 5 0", "POSE 0 0" while telemetry records.
   Ctrl-C to stop. Commands are sent to the Pico and logged for traceability.

3. SCRIPTED — run a timed sequence of commands (reproducible for papers)
   python3 telemetry_logger.py --port /dev/ttyACM0 --name step_roll5 \\
       --script scripts/step_roll5.txt

   Where scripts/step_roll5.txt contains lines like:
     0.0  MODE ATT
     1.0  POSE 0 0
     3.0  POSE 5 0
     8.0  POSE 0 0
     12.0 END

Usage to replot an existing capture:
   python3 telemetry_logger.py --plot logs/step_roll5_2026-04-20_14-33.csv

Dependencies: pyserial (required), matplotlib/numpy/pandas (for plotting)
   pip install pyserial matplotlib numpy pandas
"""

import argparse
import csv
import os
import re
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from queue import Queue, Empty

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


EXPECTED_COLUMNS = [
    "time_ms", "mode",
    "bx", "by", "bvx", "bvy", "bdet",
    "imu_r", "imu_p",
    "gyro_r", "gyro_p",
    "ref_r", "ref_p",
    "corr_r", "corr_p",
    "pose_r", "pose_p",
    "s1", "s2", "s3", "s4", "s5", "s6",
]


def sanitize_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]", "_", name).strip("_") or "run"


def parse_csv_line(line: str):
    parts = [p.strip() for p in line.strip().split(",")]
    if len(parts) != len(EXPECTED_COLUMNS):
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def parse_script(path: Path):
    """Read a script file; return a list of (time_offset_s, command) tuples.

    Script format (plain text):
      # comments are allowed
      <seconds_from_start>  <command>
    """
    events = []
    with path.open() as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split(None, 1)
            if len(parts) < 2:
                print(f"[script] skipping malformed line {lineno}: {raw!r}")
                continue
            try:
                t = float(parts[0])
            except ValueError:
                print(f"[script] skipping line {lineno} (bad time): {raw!r}")
                continue
            events.append((t, parts[1].strip()))
    events.sort(key=lambda e: e[0])
    return events


def stdin_reader_thread(out_queue: Queue, stop_event: threading.Event):
    """Read lines from stdin and push them into out_queue (daemon thread)."""
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
        except (EOFError, KeyboardInterrupt):
            return
        if not line:
            return
        stripped = line.rstrip("\r\n")
        if stripped:
            out_queue.put(stripped)


def capture(port: str, baud: int, name: str, duration,
            log_dir: Path, interactive: bool = False,
            script_events=None) -> Path:
    """Open the serial port and write telemetry to a CSV file until stopped."""
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    safe_name = sanitize_name(name)
    out_path = log_dir / f"{safe_name}_{stamp}.csv"
    cmd_log_path = log_dir / f"{safe_name}_{stamp}.commands.txt"

    print(f"Opening {port} at {baud} baud ...")
    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print(f"Logging to {out_path}")
    if script_events:
        print(f"Script: {len(script_events)} events over "
              f"{script_events[-1][0]:.1f} s")
    if interactive:
        print("Interactive mode: type commands (e.g. 'MODE ATT', 'POSE 5 0').")
        print("                  Hit Enter to send. Ctrl-C to stop.\n")
    if duration is not None:
        print(f"Duration: {duration:.1f} s (Ctrl-C to stop early)")
    else:
        print("Duration: unlimited (Ctrl-C to stop)\n")

    stopped = {"flag": False}
    stop_event = threading.Event()

    def _handler(signum, frame):
        stopped["flag"] = True
        stop_event.set()
    signal.signal(signal.SIGINT, _handler)

    stdin_q: Queue = Queue()
    if interactive:
        t_stdin = threading.Thread(
            target=stdin_reader_thread, args=(stdin_q, stop_event), daemon=True
        )
        t_stdin.start()

    n_rows = 0
    t_start = time.time()
    last_status = t_start
    next_script_idx = 0

    # Initialize the command log.
    with cmd_log_path.open("w") as cf:
        cf.write("# time_since_start_s\tsource\tcommand\n")

    def send_command(cmd: str, source: str):
        # Pico parser handles '\n' and '\r\n' and ignores empty lines.
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()
        ts = time.time() - t_start
        print(f"  [{ts:6.2f}s] → [{source}] {cmd}")
        with cmd_log_path.open("a") as cf:
            cf.write(f"{ts:.3f}\t{source}\t{cmd}\n")

    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(EXPECTED_COLUMNS)

        while not stopped["flag"]:
            now = time.time()
            elapsed = now - t_start

            if duration is not None and elapsed >= duration:
                break

            if script_events:
                while (next_script_idx < len(script_events)
                       and script_events[next_script_idx][0] <= elapsed):
                    t_evt, cmd = script_events[next_script_idx]
                    next_script_idx += 1
                    if cmd.strip().upper() == "END":
                        print(f"  [{elapsed:6.2f}s] script END reached")
                        stopped["flag"] = True
                        break
                    send_command(cmd, "script")

            if interactive:
                try:
                    cmd = stdin_q.get_nowait()
                    send_command(cmd, "user")
                except Empty:
                    pass

            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").rstrip()
            except Exception:
                continue
            if not line:
                continue

            if not line[0].isdigit():
                print(f"  [info] {line}")
                continue

            row = parse_csv_line(line)
            if row is None:
                print(f"  [skip] malformed: {line!r}")
                continue

            writer.writerow(row)
            n_rows += 1

            if now - last_status >= 1.0:
                rate = n_rows / elapsed if elapsed > 0 else 0
                print(f"  [{elapsed:6.2f}s] rows={n_rows:6d}  "
                      f"rate={rate:5.1f} Hz  last_ms={row[0]:.0f}")
                last_status = now

    stop_event.set()
    ser.close()

    elapsed = time.time() - t_start
    print(f"\nCaptured {n_rows} rows in {elapsed:.2f} s "
          f"(avg {n_rows/max(elapsed, 1e-3):.1f} Hz)")
    print(f"Saved CSV:      {out_path}")
    print(f"Saved commands: {cmd_log_path}")
    return out_path


def plot_file(csv_path: Path):
    try:
        import pandas as pd
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"ERROR: plotting needs pandas, numpy, matplotlib ({e})")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    if df.empty:
        print("No rows to plot.")
        return

    t0 = df["time_ms"].iloc[0]
    t = (df["time_ms"] - t0) / 1000.0

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(t, df["imu_r"], label="imu_r (measured)", lw=1.2)
    axes[0].plot(t, df["ref_r"], label="ref_r (target)", lw=1.0, ls="--", alpha=0.7)
    axes[0].plot(t, df["pose_r"], label="pose_r (commanded)", lw=0.8, alpha=0.5)
    axes[0].set_ylabel("Roll (deg)")
    axes[0].legend(loc="upper right", fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(csv_path.name)

    axes[1].plot(t, df["imu_p"], label="imu_p (measured)", lw=1.2)
    axes[1].plot(t, df["ref_p"], label="ref_p (target)", lw=1.0, ls="--", alpha=0.7)
    axes[1].plot(t, df["pose_p"], label="pose_p (commanded)", lw=0.8, alpha=0.5)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].legend(loc="upper right", fontsize=9)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, df["corr_r"], label="corr_r (PID out)", lw=1.0)
    axes[2].plot(t, df["corr_p"], label="corr_p (PID out)", lw=1.0)
    axes[2].set_ylabel("PID correction (deg)")
    axes[2].legend(loc="upper right", fontsize=9)
    axes[2].grid(True, alpha=0.3)

    for i in range(1, 7):
        axes[3].plot(t, df[f"s{i}"], label=f"s{i}", lw=0.7)
    axes[3].set_ylabel("Servo angles (deg)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right", fontsize=8, ncol=6)
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    out_img = csv_path.with_suffix(".png")
    fig.savefig(out_img, dpi=140)
    print(f"Saved plot: {out_img}")
    plt.show()


def quick_stats(csv_path: Path):
    try:
        import pandas as pd
        import numpy as np
    except ImportError:
        return

    df = pd.read_csv(csv_path)
    if df.empty:
        return

    print("\n=== QUICK STATS ===")
    for axis, meas, ref in [("roll", "imu_r", "ref_r"),
                            ("pitch", "imu_p", "ref_p")]:
        err = df[ref] - df[meas]
        mean_err = err.mean()
        std = err.std()
        p2p = err.max() - err.min()
        rms = np.sqrt((err ** 2).mean())
        print(f"  {axis}: mean_error={mean_err:+.3f}°  std={std:.3f}°  "
              f"p2p={p2p:.3f}°  rms={rms:.3f}°")
    print()


def main():
    ap = argparse.ArgumentParser(
        description="Stewart platform telemetry logger + commander.")
    ap.add_argument("--port", default=None,
                    help="Serial port (e.g. /dev/ttyACM0 or COM4). "
                         "Omit when using --plot.")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--name", default="run",
                    help="Experiment name used in the output filename.")
    ap.add_argument("--duration", type=float, default=None,
                    help="Capture for N seconds, then stop. "
                         "Default: run until Ctrl-C (or script END).")
    ap.add_argument("--log-dir", default="logs",
                    help="Where CSV files go (default: ./logs)")
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="Let you type commands (MODE, POSE, TARGET, TRIM) "
                         "while logging. Commands go to the Pico.")
    ap.add_argument("--script", default=None,
                    help="Run a timed command script. See file header "
                         "for format.")
    ap.add_argument("--plot", default=None,
                    help="Plot an existing CSV file (skip capture).")
    ap.add_argument("--no-plot", action="store_true",
                    help="Skip the plot step after capture.")
    args = ap.parse_args()

    if args.plot is not None:
        plot_file(Path(args.plot))
        quick_stats(Path(args.plot))
        return

    if args.port is None:
        ap.error("--port is required when capturing (or use --plot).")

    script_events = None
    if args.script:
        script_path = Path(args.script)
        if not script_path.exists():
            ap.error(f"Script not found: {script_path}")
        script_events = parse_script(script_path)
        print(f"Loaded {len(script_events)} events from {script_path}")

    if args.interactive and args.script:
        print("Note: interactive commands are also allowed during a script.\n")

    path = capture(args.port, args.baud, args.name,
                   args.duration, Path(args.log_dir),
                   interactive=args.interactive,
                   script_events=script_events)
    quick_stats(path)

    if not args.no_plot:
        try:
            plot_file(path)
        except Exception as e:
            print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()
