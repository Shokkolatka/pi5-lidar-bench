#!/usr/bin/env python3
"""Summary of one run: table in the terminal, summary.md and a short message.txt to send.

usage: summarize.py RESULTS_DIR MODE MINUTES
"""
import csv
import json
import os
import re
import statistics
import sys

res, mode, minutes = sys.argv[1], sys.argv[2], sys.argv[3]

# ms per scan measured on one x86 core (Intel Xeon, ~2.77 GHz effective) for the report, module 7.
# The report converted them to Pi 5 with a factor of 1.6-2.5; this run replaces that guess.
X86_MS = {
    "kiss_default_v0.4_m20": 689.0, "v1.0_m20": 221.5, "v1.0_m5": 143.8, "v1.5_m3": 77.2,
    "v1.0_m5_nocv": 137.3, "v1.0_m5_vib0": 82.1, "inv_v1.0_m5": 43.7, "l2_v1.0_m5": 78.5,
    "sub4_v1.0_m5": 61.4, "long_v1.5_m3": 77.8, "long_inv_v1.0_m5": 44.4,
}
LABELS = {
    "kiss_default_v0.4_m20": "Стандарт KISS-ICP (0,4 м, ≤20)", "v1.0_m20": "1,0 м, ≤20", "v1.0_m5": "1,0 м, ≤5",
    "v1.5_m3": "1,5 м, ≤3", "v1.0_m5_nocv": "1,0 м, ≤5, без прогнозу швидкості", "v1.0_m5_vib0": "1,0 м, ≤5, без вібрацій",
    "inv_v1.0_m5": "Mid-360 перевернутий, 1,0 м, ≤5", "l2_v1.0_m5": "Як Unitree L2, 1,0 м, ≤5",
    "sub4_v1.0_m5": "Кожен 4-й промінь, 1,0 м, ≤5", "long_v1.5_m3": "Довгий прогін 131 м, 1,5 м, ≤3",
    "long_inv_v1.0_m5": "Довгий прогін 131 м, перевернутий",
}
ORDER = list(X86_MS)


def num(text, key):
    m = re.search(rf"{key}=([-\d.e+naif]+)", text)
    try:
        return float(m.group(1)) if m else None
    except ValueError:
        return None


def fmt(x, nd=1, dash="-"):
    if x is None:
        return dash
    return f"{x:.{nd}f}".replace(".", ",")


rows = []
for name in ORDER:
    out = os.path.join(res, f"{name}.out")
    if not os.path.exists(out):
        continue
    text = open(out).read()
    ms = num(text, "ms_per_scan")
    if ms is None:
        continue
    scans_ms = []
    with open(os.path.join(res, f"{name}.scans.csv")) as f:
        for r in csv.DictReader(f):
            try:
                scans_ms.append(float(r["ms_total"]))
            except (KeyError, ValueError):
                pass
    p95 = sorted(scans_ms)[int(0.95 * (len(scans_ms) - 1))] if scans_ms else None
    mon = {}
    mpath = os.path.join(res, f"{name}.monitor.json")
    if os.path.exists(mpath):
        mon = json.load(open(mpath))
    thr = mon.get("throttled_or_hex")
    ref_scans = 600 if name.startswith("long") else 120
    comparable = name in X86_MS and len(scans_ms) == ref_scans  # same run length as in the report
    rows.append(dict(
        name=name, label=LABELS.get(name, name), ms=ms, p95=p95, cores=ms * 10 / 1000,
        drift=num(text, "drift_pct"), map_points=num(text, "map_points"), rss_mb=(num(text, "rss_kb") or 0) / 1024,
        x86=X86_MS.get(name), factor=ms / X86_MS[name] if comparable else None,
        tmax=mon.get("temp_max_c"), fmin=mon.get("freq_min_mhz"),
        throttled=None if thr is None else int(thr, 16) & 0xF,
    ))

if not rows:
    print("Немає жодного завершеного прогону.")
    sys.exit(1)

sysinfo = open(os.path.join(res, "sysinfo.txt")).read()
model = re.search(r"model: (.*)", sysinfo)
model = model.group(1).strip() if model else "невідомо"
cooling = re.search(r"cooling \(зі слів користувача\): (.*)", sysinfo)
cooling = cooling.group(1).strip() if cooling else "не вказано"
ram = re.search(r"Mem:\s+(\d+)", sysinfo)
ram = f"{round(int(ram.group(1)) / 1024)} ГБ" if ram else "?"

factors = [r["factor"] for r in rows if r["factor"]]
factor_med = statistics.median(factors) if factors else None
tmax = max((r["tmax"] for r in rows if r["tmax"] is not None), default=None)
fmin = min((r["fmin"] for r in rows if r["fmin"] is not None), default=None)
any_thr = any(r["throttled"] for r in rows if r["throttled"] is not None)
thr_known = any(r["throttled"] is not None for r in rows)

header = ["Конфігурація", "мс/скан", "p95 мс", "ядра @10 Гц", "% бюджету 2 ядер", "прогноз звіту, мс",
          "Pi/x86", "дрейф %", "макс °C", "мін МГц"]
table = []
for r in rows:
    pred = f"{fmt(r['x86'] * 1.6, 0)}-{fmt(r['x86'] * 2.5, 0)}" if r["x86"] else "-"
    table.append([r["label"], fmt(r["ms"], 0), fmt(r["p95"], 0), fmt(r["cores"], 2), fmt(r["cores"] / 2 * 100, 0),
                  pred, fmt(r["factor"], 2), fmt(r["drift"], 2), fmt(r["tmax"], 1), fmt(r["fmin"], 0)])

widths = [max(len(str(x)) for x in col) for col in zip(header, *table)]
line = lambda cells: "  ".join(str(c).ljust(w) for c, w in zip(cells, widths))
print("Підсумок (кожна конфігурація на одному ядрі, 10 Гц = 100 мс на скан):")
print(line(header))
print(line(["-" * w for w in widths]))
for t in table:
    print(line(t))
print()
print(f"Коефіцієнт Pi 5 / x86 (медіана): {fmt(factor_med, 2)}; у звіті закладено 1,6-2,5.")
print(f"Макс. температура: {fmt(tmax)} °C, мін. частота: {fmt(fmin, 0)} МГц, тротлінг: "
      f"{'так' if any_thr else ('ні' if thr_known else 'невідомо (немає vcgencmd)')}.")

with open(os.path.join(res, "summary.md"), "w") as f:
    f.write(f"# Результати на {model}\n\nРежим: {mode}, тривалість ~{minutes} хв, охолодження: {cooling}, ОЗП: {ram}\n\n")
    f.write("| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n")
    for t in table:
        f.write("| " + " | ".join(t) + " |\n")
    f.write(f"\nКоефіцієнт Pi 5 / x86 (медіана): {fmt(factor_med, 2)} (у звіті 1,6-2,5).\n")
    f.write(f"Макс. температура {fmt(tmax)} °C, мін. частота {fmt(fmin, 0)} МГц, тротлінг: {'так' if any_thr else 'ні'}.\n")

by = {r["name"]: r for r in rows}
mode_ua = "швидкий" if mode == "quick" else "повний"
msg = [f"Прогнав бенчмарк лідарної одометрії (KISS-ICP-подібна, синтетичний ліс під Mid-360) на {model}, {ram} ОЗП, "
       f"охолодження: {cooling}. Набір {mode_ua}, ~{minutes} хв. Головне:"]
if factor_med:
    verdict = "у межах" if 1.6 <= factor_med <= 2.5 else ("нижче за" if factor_med < 1.6 else "вище за")
    msg.append(f"- Коефіцієнт перерахунку Pi 5 / x86 вийшов {fmt(factor_med, 2)} ({verdict} діапазон 1,6-2,5, "
               f"закладений у звіті), тобто стільки разів довше Pi 5 обробляє той самий скан.")
else:
    msg.append("- Коефіцієнт Pi 5 / x86 не порахований: довжина прогонів відрізняється від звіту (120 або 600 сканів).")
for key in ("kiss_default_v0.4_m20", "v1.5_m3", "inv_v1.0_m5"):
    if key in by:
        r = by[key]
        msg.append(f"- {r['label']}: {fmt(r['ms'], 0)} мс/скан, {fmt(r['cores'], 2)} ядра при 10 Гц "
                   f"({fmt(r['cores'] / 2 * 100, 0)}% бюджету), p95 {fmt(r['p95'], 0)} мс.")
msg.append(f"- Температура до {fmt(tmax)} °C, мінімальна частота {fmt(fmin, 0)} МГц, тротлінг: "
           f"{'так' if any_thr else ('ні' if thr_known else 'невідомо')}.")
msg.append(f"Повна таблиця і логи (посекундне навантаження ядер, температура, частота) в архіві {os.path.basename(res)}.tar.gz, "
           f"таблиця у summary.md.")
with open(os.path.join(res, "message.txt"), "w") as f:
    f.write("\n".join(msg) + "\n")
