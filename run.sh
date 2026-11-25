#!/usr/bin/env bash
# Builds the benchmark, runs the configurations one by one with a live load monitor,
# prints a summary table and packs everything into one archive to send back.
#   ./run.sh          quick set, ~6-10 min on Pi 5
#   ./run.sh full     everything from the report, ~30-40 min on Pi 5
# Optional: COOLING="активний кулер" ./run.sh   (goes into the summary text)
set -uo pipefail
cd "$(dirname "$0")"

MODE="${1:-quick}"
SCANS="${SCANS:-120}"         # override only for a smoke test
LONG_SCANS="${LONG_SCANS:-600}"
COOLING="${COOLING:-не вказано}"

if [[ "$MODE" != "quick" && "$MODE" != "full" ]]; then
  echo "Використання: ./run.sh [quick|full]"; exit 1
fi
for tool in g++ python3; do
  command -v "$tool" >/dev/null || { echo "Немає $tool. Встановіть: sudo apt update && sudo apt install -y g++ python3"; exit 1; }
done

STAMP="$(date +%Y%m%d_%H%M)"
NAME="pi5_lidar_${HOSTNAME:-pi}_${STAMP}"
RES="results/$NAME"
mkdir -p build "$RES"

echo "== Збірка"
g++ -O2 -std=c++17 src/icp_bench.cpp -o build/icp_bench || { echo "Збірка не вдалася"; exit 1; }

echo "== Інформація про систему -> $RES/sysinfo.txt"
{
  echo "date: $(date -Is)"
  echo "model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
  echo "os: $(. /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-unknown}")"
  echo "kernel: $(uname -a)"
  echo "cpus: $(nproc)"
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo n/a)"
  echo "max_freq_khz: $(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null || echo n/a)"
  echo "cooling (зі слів користувача): $COOLING"
  echo "compiler: $(g++ --version | head -1)"
  if command -v vcgencmd >/dev/null; then
    echo "temp_start: $(vcgencmd measure_temp)"
    echo "throttled_start: $(vcgencmd get_throttled)"
    echo "arm_clock: $(vcgencmd measure_clock arm)"
    echo "fw: $(vcgencmd version | tr '\n' ' ')"
  fi
  echo; free -m; echo; lscpu 2>/dev/null
} > "$RES/sysinfo.txt"
grep -E "model|governor|temp_start|throttled_start" "$RES/sysinfo.txt"

PIN=""
if command -v taskset >/dev/null && [[ "$(nproc)" -ge 4 ]]; then PIN="taskset -c 3"; fi

BENCH_PID=""
trap '[[ -n "$BENCH_PID" ]] && kill "$BENCH_PID" 2>/dev/null; echo; echo "Перервано."; exit 130' INT TERM

# run_cfg NAME RAYS VOXEL SCANS MAXPTS [KEY=VALUE ...]
run_cfg() {
  local name=$1 rays=$2 vox=$3 scans=$4 mp=$5; shift 5
  echo
  echo "================================================================================"
  echo "== $name: променів $rays, воксель $vox м, до $mp точок, сканів $scans $*"
  echo "================================================================================"
  env MAXPTS="$mp" "$@" $PIN ./build/icp_bench "$rays" "$vox" "$scans" 1 "$RES/$name.scans.csv" > "$RES/$name.out" 2> "$RES/$name.err" &
  BENCH_PID=$!
  python3 tools/monitor.py "$BENCH_PID" "$RES/$name.scans.csv" "$scans" "$RES/$name.monitor.csv" "$RES/$name.monitor.json"
  wait "$BENCH_PID"; local rc=$?
  BENCH_PID=""
  if [[ $rc -ne 0 ]]; then echo "!! $name завершився з кодом $rc (див. $RES/$name.err)"; return; fi
  grep -E "ms_per_scan|drift_pct" "$RES/$name.out" | sed 's/^/   /'
}

T0=$(date +%s)
run_cfg kiss_default_v0.4_m20 20000 0.4 "$SCANS" 20
run_cfg v1.0_m5               20000 1.0 "$SCANS" 5
run_cfg v1.5_m3               20000 1.5 "$SCANS" 3
run_cfg inv_v1.0_m5           20000 1.0 "$SCANS" 5 MOUNT_INV=1
if [[ "$MODE" == "full" ]]; then
  run_cfg v1.0_m20            20000 1.0 "$SCANS" 20
  run_cfg v1.0_m5_nocv        20000 1.0 "$SCANS" 5 NOCV=1
  run_cfg v1.0_m5_vib0        20000 1.0 "$SCANS" 5 VIB_DEG=0
  run_cfg sub4_v1.0_m5         5000 1.0 "$SCANS" 5
  run_cfg l2_v1.0_m5           6400 1.0 "$SCANS" 5
  run_cfg long_v1.5_m3        20000 1.5 "$LONG_SCANS" 3
  run_cfg long_inv_v1.0_m5    20000 1.0 "$LONG_SCANS" 5 MOUNT_INV=1
fi
MINUTES=$(( ($(date +%s) - T0 + 59) / 60 ))
command -v vcgencmd >/dev/null && echo "throttled_end: $(vcgencmd get_throttled)" >> "$RES/sysinfo.txt"

echo
python3 tools/summarize.py "$RES" "$MODE" "$MINUTES"
tar czf "results/$NAME.tar.gz" -C results "$NAME"

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
echo "################################################################################"
echo "#  ГОТОВО за ~$MINUTES хв."
echo "#  Надішли товаришу:"
echo "#    1) архів  $(pwd)/results/$NAME.tar.gz"
echo "#    2) текст із файлу  $RES/message.txt  (він надрукований нижче)"
echo "#  Забрати архів на свій комп'ютер:"
echo "#    scp ${USER:-pi}@${IP:-<ip-малинки>}:$(pwd)/results/$NAME.tar.gz ."
echo "################################################################################"
echo
cat "$RES/message.txt"
