#!/usr/bin/env python3
"""Live load monitor for one benchmark process.

Prints one line per second to the terminal while the process is alive and writes the same
numbers to a CSV. At the end writes a small JSON with peaks (temperature, min frequency,
throttling, peak memory). Only the standard library, reads /proc and /sys directly.

usage: monitor.py PID SCANS_CSV N_SCANS OUT_CSV OUT_JSON
"""
import json
import os
import shutil
import subprocess
import sys
import time

pid, scans_csv, n_scans, out_csv, out_json = int(sys.argv[1]), sys.argv[2], int(sys.argv[3]), sys.argv[4], sys.argv[5]
HZ = os.sysconf("SC_CLK_TCK")
VCGENCMD = shutil.which("vcgencmd")
COLOR = sys.stdout.isatty()


def paint(text, code):
    return f"\033[{code}m{text}\033[0m" if COLOR else text


def cpu_times():
    """Per-core (total, idle) jiffies from /proc/stat."""
    res = {}
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("cpu") and line[3].isdigit():
                parts = line.split()
                vals = [int(v) for v in parts[1:9]]
                res[parts[0]] = (sum(vals), vals[3] + vals[4])
    return res


def proc_jiffies():
    try:
        with open(f"/proc/{pid}/stat") as f:
            data = f.read()
        fields = data[data.rindex(")") + 2:].split()
        return int(fields[11]) + int(fields[12])  # utime + stime
    except (OSError, ValueError, IndexError):
        return None


def rss_mb():
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024
    except OSError:
        pass
    return None


def temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read()) / 1000
    except (OSError, ValueError):
        return None


def frequency_mhz():
    freqs = []
    for cpu in range(os.cpu_count() or 1):
        try:
            with open(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq") as f:
                freqs.append(int(f.read()) / 1000)
        except (OSError, ValueError):
            pass
    return max(freqs) if freqs else None


def throttled():
    """Raw hex from `vcgencmd get_throttled` (Raspberry Pi only)."""
    if not VCGENCMD:
        return None
    try:
        out = subprocess.run([VCGENCMD, "get_throttled"], capture_output=True, text=True, timeout=2).stdout
        return int(out.strip().split("=")[-1], 16)
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def throttle_text(value):
    if value is None:
        return "н/д"
    now = []
    if value & 0x1:
        now.append("НИЗЬКА НАПРУГА")
    if value & 0x2:
        now.append("ЧАСТОТУ ОБМЕЖЕНО")
    if value & 0x4:
        now.append("ТРОТЛІНГ")
    if value & 0x8:
        now.append("ТЕМП. ЛІМІТ")
    if now:
        return paint(", ".join(now), "1;31")
    if value & 0xF0000:
        return paint("зараз ні (але було після ввімкнення)", "33")
    return paint("ні", "32")


def progress():
    """(scans done, mean ms per scan over the last 10 scans) from the benchmark CSV."""
    try:
        with open(scans_csv) as f:
            rows = f.read().splitlines()[1:]
    except OSError:
        return 0, None
    rows = [r for r in rows if r.count(",") >= 5]
    last = rows[-10:]
    try:
        ms = sum(float(r.split(",")[5]) for r in last) / len(last) if last else None
    except ValueError:
        ms = None
    return len(rows), ms


def main():
    prev_cpu, prev_proc, t0 = cpu_times(), proc_jiffies(), time.time()
    prev_t = t0
    peak = {"temp_max_c": None, "freq_min_mhz": None, "throttled_or": 0, "rss_max_mb": None,
            "proc_cores_sum": 0.0, "samples": 0, "throttled_raw_available": VCGENCMD is not None}
    with open(out_csv, "w") as log:
        log.write("t_s,scans_done,ms_per_scan_last10,proc_cores,rss_mb,temp_c,freq_mhz,throttled_hex," +
                  ",".join(f"core{i}_pct" for i in range(len(prev_cpu))) + "\n")
        while True:
            time.sleep(1.0)
            alive = os.path.exists(f"/proc/{pid}")
            now_t = time.time()
            cur_cpu, cur_proc = cpu_times(), proc_jiffies()
            dt = now_t - prev_t
            cores_pct = []
            for name in sorted(cur_cpu, key=lambda c: int(c[3:])):
                tot = cur_cpu[name][0] - prev_cpu.get(name, (0, 0))[0]
                idle = cur_cpu[name][1] - prev_cpu.get(name, (0, 0))[1]
                cores_pct.append(100.0 * (tot - idle) / tot if tot > 0 else 0.0)
            proc_cores = None
            if cur_proc is not None and prev_proc is not None and dt > 0:
                proc_cores = (cur_proc - prev_proc) / HZ / dt
            rss, temp, freq, thr = rss_mb(), temperature(), frequency_mhz(), throttled()
            done, ms = progress()

            if temp is not None:
                peak["temp_max_c"] = max(peak["temp_max_c"] or temp, temp)
            if freq is not None:
                peak["freq_min_mhz"] = min(peak["freq_min_mhz"] or freq, freq)
            if thr is not None:
                peak["throttled_or"] |= thr
            if rss is not None:
                peak["rss_max_mb"] = max(peak["rss_max_mb"] or rss, rss)
            if proc_cores is not None:
                peak["proc_cores_sum"] += proc_cores
                peak["samples"] += 1

            elapsed = now_t - t0
            phase = f"скан {done:4d}/{n_scans}" if done else "генерація сцени"
            ms_txt = f"ост.10: {ms:6.0f} мс/скан" if ms is not None else " " * 20
            if ms is not None and ms > 100:
                ms_txt = paint(ms_txt, "33")
            temp_txt = f"{temp:5.1f}°C" if temp is not None else "  н/д"
            if temp is not None and temp >= 80:
                temp_txt = paint(temp_txt, "1;31")
            freq_txt = f"{freq:4.0f} МГц" if freq is not None else "н/д"
            proc_txt = f"{proc_cores:4.2f} ядра" if proc_cores is not None else "   -    "
            rss_txt = f"{rss:5.0f} МБ" if rss is not None else "   - "
            cores_txt = " ".join(f"{c:3.0f}%" for c in cores_pct)
            print(f"[{elapsed:5.0f} с] {phase}  {ms_txt} | процес {proc_txt} {rss_txt} | ядра {cores_txt} | "
                  f"{temp_txt} {freq_txt} | тротлінг: {throttle_text(thr)}", flush=True)
            log.write(f"{elapsed:.1f},{done},{'' if ms is None else round(ms, 2)},"
                      f"{'' if proc_cores is None else round(proc_cores, 3)},{'' if rss is None else round(rss, 1)},"
                      f"{'' if temp is None else temp},{'' if freq is None else freq},{'' if thr is None else hex(thr)},"
                      + ",".join(f"{c:.1f}" for c in cores_pct) + "\n")
            log.flush()
            prev_cpu, prev_proc, prev_t = cur_cpu, cur_proc, now_t
            if not alive:
                break
    peak["proc_cores_mean"] = peak["proc_cores_sum"] / peak["samples"] if peak["samples"] else None
    peak["throttled_or_hex"] = hex(peak["throttled_or"]) if VCGENCMD else None
    with open(out_json, "w") as f:
        json.dump(peak, f, indent=1)


if __name__ == "__main__":
    main()
