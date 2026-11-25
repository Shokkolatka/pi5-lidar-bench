"""Paper estimates of Raspberry Pi 5 load for each method (all arithmetic in one place).

Two ways to convert operations to time:
  * 'optimistic'  : task statement, 10-30 % of NEON peak per core (38.4 GFLOPS/core) -> 3.84-11.5 Gop/s
  * 'calibrated'  : from FAST-LIO2 on Cortex-A73 (756 pts in 5.23 ms), moved to A76 -> ~0.9 Gop/s/core
Cross-checks use measured times on other ARM/x86 CPUs scaled by single-core speed ratios (assumptions below).
"""
PEAK_CORE = 2.4e9 * 16     # FLOP/s per core: 2 NEON pipes x 4 fp32 lanes x 2 (FMA) = 16/cycle -> 38.4e9
OPT = (0.10 * PEAK_CORE, 0.30 * PEAK_CORE)

# --- calibration on FAST-LIO2 / Khadas VIM3 (Cortex-A73 @2.2 GHz): 756 points/scan, 5.23 ms ---
def fastlio_ops(n_ds, n_raw_kept, k_search=2, k_iter=4, c_knn=1000, c_res=300, c_pre=60, c_ins=400):
    return n_ds * (k_search * c_knn + k_iter * c_res) + n_raw_kept * c_pre + n_ds * c_ins
ops_cal = fastlio_ops(756, 6000)
A73_rate = ops_cal / 5.23e-3
A73_TO_A76 = 1.6           # assumption: Pi 5 core ~1.5-1.8x faster than A73@2.2 GHz
CAL = A73_rate * A73_TO_A76
print(f"calibration: FAST-LIO2 ops for 756 pts = {ops_cal/1e6:.2f} M -> A73 {A73_rate/1e9:.2f} Gop/s -> A76 {CAL/1e9:.2f} Gop/s"
      f" = {100*CAL/PEAK_CORE:.1f}% of core peak")

def cores(ops_per_scan, hz):
    rate = ops_per_scan * hz
    return rate / OPT[1], rate / OPT[0], rate / CAL

def show(name, ops, hz):
    lo, hi, cal = cores(ops, hz)
    print(f"{name:38s} ops/scan={ops/1e6:7.2f} M  @{hz:>4} Hz  optimistic {100*lo:5.1f}-{100*hi:5.1f}% core   calibrated {100*cal:6.1f}% core")

print("\n--- 2D: RPLIDAR S2, 3200 pts/scan, 10 Hz ---")
N2 = 3200
# Hector: 3 levels, ~4 GN iterations/level, ~60 ops/pt/iter (transform, bilinear interp+gradient, 3x3 accum)
hector_match = N2 * 12 * 60
# map update: Bresenham, avg 10 m range: 200+100+50 cells x 5 ops, done every 2nd scan (0.4 m / 2 m/s threshold)
hector_map = N2 * 350 * 5 / 2
show("Hector SLAM", hector_match + hector_map, 10)
# Cartographer local trajectory builder
carto_ceres = 20 * 400 * 150            # 20 iters x 400 pts (adaptive voxel filter) x 150 ops (bicubic + J)
carto_csm = 25 * 201 * 400 * 3          # online correlative: 5x5 xy x 201 angles x 400 pts x 3 ops
carto_insert = 2 * N2 * 200 * 5         # 2 active submaps, ray casting
show("Cartographer local (no loop closure)", carto_ceres + carto_csm + carto_insert, 10)
# SLAM Toolbox / Karto: coarse correlative search 50x50 x 21 angles x 1000 lookups, 4 processed scans/s
karto = 50 * 50 * 21 * 1000
show("SLAM Toolbox scan matching", karto, 4)

print("\n--- 3D: Livox Mid-360, 20 000 pts/scan, 10 Hz ---")
for nds in (1500, 3000):
    show(f"FAST-LIO2 (N_ds={nds})", fastlio_ops(nds, 5000), 10)
for nreg in (2000, 9000):
    # KISS-ICP: 27 voxels x up to 20 points x 8 FLOP per distance + ~200 ops per correspondence, 10-20 iterations
    kiss = nreg * 15 * (27 * 20 * 8 + 200)
    show(f"KISS-ICP (N_reg={nreg}, full voxels)", kiss, 10)
# LOAM-type features: curvature 20k x 11 x 3, odometry kd-tree 2 x 2000 features x 25 iters x 600
loam_odom = 20000 * 11 * 3 + 2000 * 25 * 600
show("LOAM odometry thread", loam_odom, 10)

print("\n--- cross-checks: measured times scaled to Pi 5 (single-core speed ratio assumptions) ---")
ratio = {  # how many times the source core is faster than a Pi 5 core (<1 = slower)
    "Cortex-A73 @2.2 (Khadas VIM3)": (1/1.8, 1/1.5),
    "Cortex-A57 @2.0 (Jetson TX2)": (1/2.5, 1/2.2),
    "Cortex-A77 @2.42 (VOXL 2)": (1.15, 1.25),
    "Cortex-A78AE (Orin NX)": (0.95, 1.15),
    "Intel Atom Z530 @1.6": (1/10, 1/6),
    "Xeon E5-1650 @3.2": (1.25, 1.7),
    "i7-1260P": (2.2, 2.8),
}
def scale(ms, cpu, hz=10):
    a, b = ratio[cpu]
    return ms * a, ms * b, ms * a * hz / 1000, ms * b * hz / 1000
cases = [
    ("FAST-LIO2, 756 pts (paper)", 5.23, "Cortex-A73 @2.2 (Khadas VIM3)", 10),
    # Super-LIO Table II, ARM (Orin NX), mean over the ten Mid-360 sequences se01-se10
    # (Super-LIO: mean of the 8 legible values; iG-LIO ran only on se05: 8.21 ms, X on the other 9)
    ("FAST-LIO2, Mid-360 (Super-LIO tbl, se01-10)", 24.6, "Cortex-A78AE (Orin NX)", 10),
    ("Faster-LIO, Mid-360 (Super-LIO tbl, se01-10)", 13.8, "Cortex-A78AE (Orin NX)", 10),
    ("Super-LIO, Mid-360 (Super-LIO tbl, se01-10)", 5.6, "Cortex-A78AE (Orin NX)", 10),
    ("FAST-LIO2, 96-ch Airy (Matos)", 26.8, "Cortex-A77 @2.42 (VOXL 2)", 10),
    ("Faster-LIO, 96-ch Airy (Matos)", 24.0, "Cortex-A77 @2.42 (VOXL 2)", 10),
    ("DLIO, 96-ch Airy (Matos)", 35.5, "Cortex-A77 @2.42 (VOXL 2)", 10),
    ("KISS-ICP, 96-ch Airy (Matos, picking)", 59.9, "Cortex-A77 @2.42 (VOXL 2)", 10),
    ("F-LOAM, 96-ch Airy (Matos)", 79.4, "Cortex-A77 @2.42 (VOXL 2)", 10),
    ("LeGO-LOAM front-end, VLP-16 (TX2)", 61.0, "Cortex-A57 @2.0 (Jetson TX2)", 10),  # 57.7-61.0 ms in Table IV
    ("LOAM feat+odom, VLP-16 (TX2)", 238.0, "Cortex-A57 @2.0 (Jetson TX2)", 10),
    ("Swarm-LIO2, Mid-360 (paper, lidar at 30 Hz!)", 7.57, "i7-1260P", 30),
]
for name, ms, cpu, hz in cases:
    lo, hi, clo, chi = scale(ms, cpu, hz)
    print(f"{name:45s} {ms:7.2f} ms on {cpu:30s} -> Pi5 {lo:6.1f}-{hi:6.1f} ms/scan = {clo:4.2f}-{chi:4.2f} cores @ {hz} Hz")
# Matos et al. 2026, VOXL 2 (Cortex-A77), 96-ch RoboSense Airy at 10 Hz: cores, lab (Table 7) + warehouse (Table 8)
# KISS-ICP processed only 80-97% of scans in the lab and 33-50% in the warehouse -> its real demand is higher.
for name, c in [("FAST-LIO2", (0.28, 0.40)), ("Faster-LIO", (0.38, 0.62)), ("DLIO", (0.52, 0.62)), ("liorf", (0.76, 0.76)), ("F-LOAM", (0.88, 0.96)), ("KISS-ICP*", (1.92, 2.70))]:
    print(f"Matos cores {name:10s} {c[0]:.2f}-{c[1]:.2f} on A77 -> Pi5 {c[0]*1.15:.2f}-{c[1]*1.25:.2f} cores")
# LeGO-LOAM mapping (TX2 253-278 ms per call), at 1-2 Hz
for hz in (1, 2):
    print(f"LeGO-LOAM mapping @{hz} Hz: TX2 {0.253*hz:.2f}-{0.278*hz:.2f} cores -> Pi5 {0.253*hz/2.5:.2f}-{0.278*hz/2.2:.2f} cores")
print(f"LOAM mapping ~780 ms/s on TX2 -> Pi5 {0.78/2.5:.2f}-{0.78/2.2:.2f} cores")
# Cartographer whole system: 1018 s CPU for 1913 s data on Xeon E5-1650
x = 1018 / 1913
print(f"Cartographer (paper, all threads): {x:.2f} Xeon cores avg -> Pi5 {x*1.25:.2f}-{x*1.7:.2f} cores")
# Hector on Atom Z530: 3x real time on one core
print(f"Hector (paper): <=1/3 Atom core -> Pi5 {100/3/10:.1f}-{100/3/6:.1f}% of a core")
