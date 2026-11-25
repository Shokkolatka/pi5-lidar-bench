// KISS-ICP-like LiDAR odometry benchmark on a synthetic forest (Livox Mid-360-like scans).
// Single-threaded, C++17, no external deps.
// Pipeline per scan: voxel downsample -> constant-velocity prediction -> point-to-point ICP
// against a voxel-hash local map (max N points per voxel, 27-voxel neighbourhood) with
// adaptive threshold + Geman-McClure kernel -> map update -> remove far voxels.
// Build: g++ -O2 -std=c++17 icp_bench.cpp -o icp_bench
// Run:   ./icp_bench <rays_per_scan> <voxel_size> <n_scans> <seed> <csv_out>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include <ctime>

struct V3 { double x, y, z; };
static inline V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 mul(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
static inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline double norm(V3 a) { return std::sqrt(dot(a, a)); }

struct Pose {  // x_world = R * x_body + t
  double R[9];
  V3 t;
};
static inline V3 rot(const double* R, V3 v) {
  return {R[0]*v.x + R[1]*v.y + R[2]*v.z, R[3]*v.x + R[4]*v.y + R[5]*v.z, R[6]*v.x + R[7]*v.y + R[8]*v.z};
}
static inline V3 apply(const Pose& p, V3 v) { return add(rot(p.R, v), p.t); }
static void matmul(const double* A, const double* B, double* C) {
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
    double s = 0; for (int k = 0; k < 3; ++k) s += A[3*i+k] * B[3*k+j]; C[3*i+j] = s; }
}
static void transpose(const double* A, double* B) { for (int i=0;i<3;++i) for (int j=0;j<3;++j) B[3*j+i]=A[3*i+j]; }
// Gram-Schmidt: keep R a proper rotation. Without this, rounding errors make R slightly
// non-orthonormal; inverse() uses the transpose, so the constant-velocity prediction
// R_pred = R1 * R2^T * R1 multiplies that error by ~2.4 every scan and odometry blows up
// after ~35 scans (this was the "divergence near scan 38" in the first version).
static void orthonormalize(double* R) {
  V3 a = {R[0], R[1], R[2]}, b = {R[3], R[4], R[5]};
  a = mul(a, 1.0 / norm(a));
  b = sub(b, mul(a, dot(a, b))); b = mul(b, 1.0 / norm(b));
  V3 c = {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
  R[0]=a.x; R[1]=a.y; R[2]=a.z; R[3]=b.x; R[4]=b.y; R[5]=b.z; R[6]=c.x; R[7]=c.y; R[8]=c.z;
}
static Pose compose(const Pose& a, const Pose& b) {  // a*b
  Pose c; matmul(a.R, b.R, c.R); orthonormalize(c.R); c.t = add(rot(a.R, b.t), a.t); return c;
}
static Pose inverse(const Pose& a) { Pose c; transpose(a.R, c.R); c.t = mul(rot(c.R, a.t), -1.0); return c; }
static void so3exp(V3 w, double* R) {
  double th = norm(w);
  double K[9] = {0, -w.z, w.y, w.z, 0, -w.x, -w.y, w.x, 0};
  double K2[9]; matmul(K, K, K2);
  double a, b;
  if (th < 1e-9) { a = 1.0; b = 0.5; } else { a = std::sin(th) / th; b = (1 - std::cos(th)) / (th * th); }
  for (int i = 0; i < 9; ++i) R[i] = (i % 4 == 0 ? 1.0 : 0.0) + a * K[i] + b * K2[i];
}
static V3 so3log(const double* R) {
  double c = (R[0] + R[4] + R[8] - 1) * 0.5; c = std::max(-1.0, std::min(1.0, c));
  double th = std::acos(c);
  V3 w = {R[7] - R[5], R[2] - R[6], R[3] - R[1]};
  if (th < 1e-9) return mul(w, 0.5);
  return mul(w, th / (2 * std::sin(th)));
}
static Pose from_rpy_xyz(double roll, double pitch, double yaw, V3 t) {
  double Rx[9], Ry[9], Rz[9], T[9];
  so3exp({roll, 0, 0}, Rx); so3exp({0, pitch, 0}, Ry); so3exp({0, 0, yaw}, Rz);
  Pose p; matmul(Rz, Ry, T); matmul(T, Rx, p.R); p.t = t; return p;
}

// ------------------------- synthetic forest -------------------------
struct Tree { double x, y, r, top; double cx, cy, cz, ax, ay, az; double mfp; };
static double ground(double x, double y) { return 0.5 * std::sin(0.05 * x) * std::cos(0.07 * y) + 0.15 * std::sin(0.3 * x + 0.2 * y); }

struct Forest {
  std::vector<Tree> trees;   // trunks + crowns
  std::vector<Tree> bushes;  // undergrowth (sphere-ish crowns only)
};

static double SPEED = 2.0, YAMP = 8.0;
static V3 traj_pos(double t) { return {SPEED * t, YAMP * std::sin(2 * M_PI * t / 40.0), 0}; }

static Forest make_forest(unsigned seed, double density_per_ha) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(0, 1);
  Forest f;
  double x0 = -40, x1 = 170, y0 = -55, y1 = 55;
  int n = int(density_per_ha * (x1 - x0) * (y1 - y0) / 1e4);
  int tries = 0;
  while ((int)f.trees.size() < n && tries < n * 50) {
    ++tries;
    double x = x0 + (x1 - x0) * U(rng), y = y0 + (y1 - y0) * U(rng);
    bool ok = true;
    // keep a 1.8 m corridor around the flight path free of trunks
    for (double tt = std::max(0.0, x / 2.0 - 6); tt < x / 2.0 + 6; tt += 0.25) {
      V3 p = traj_pos(tt); if (std::hypot(p.x - x, p.y - y) < 1.8) { ok = false; break; }
    }
    if (!ok) continue;
    for (auto& t : f.trees) if (std::hypot(t.x - x, t.y - y) < 2.0) { ok = false; break; }
    if (!ok) continue;
    Tree t; t.x = x; t.y = y; t.r = 0.08 + 0.17 * U(rng);
    double h = 14 + 10 * U(rng); double g = ground(x, y);
    t.top = g + h;
    t.cx = x + 0.5 * (U(rng) - 0.5); t.cy = y + 0.5 * (U(rng) - 0.5);
    t.az = 3 + 3 * U(rng); t.ax = 2.5 + 2.0 * U(rng); t.ay = t.ax * (0.8 + 0.4 * U(rng));
    t.az = std::min(t.az, (h - 6.5) / 2.0);  // lowest branches at >= 6.5 m above ground
    t.cz = g + h - t.az;
    t.mfp = 0.8 + 1.2 * U(rng);
    f.trees.push_back(t);
  }
  int nb = int(0.5 * density_per_ha * (x1 - x0) * (y1 - y0) / 1e4);
  for (int i = 0; i < nb; ++i) {
    double x = x0 + (x1 - x0) * U(rng), y = y0 + (y1 - y0) * U(rng);
    bool ok = true;
    for (double tt = std::max(0.0, x / 2.0 - 6); tt < x / 2.0 + 6; tt += 0.25) {
      V3 p = traj_pos(tt); if (std::hypot(p.x - x, p.y - y) < 2.2) { ok = false; break; }
    }
    if (!ok) continue;
    Tree b; b.x = x; b.y = y; b.r = 0; b.top = 0;
    b.ax = 0.5 + 0.7 * U(rng); b.ay = b.ax; b.az = 0.4 + 0.5 * U(rng);
    b.cx = x; b.cy = y; b.cz = ground(x, y) + b.az * 0.8; b.mfp = 0.35;
    f.bushes.push_back(b);
  }
  return f;
}

// ray / ellipsoid: returns entry, exit (t_in < t_out) or false
static inline bool ray_ellipsoid(V3 o, V3 d, const Tree& e, double& tin, double& tout) {
  double ox = (o.x - e.cx) / e.ax, oy = (o.y - e.cy) / e.ay, oz = (o.z - e.cz) / e.az;
  double dx = d.x / e.ax, dy = d.y / e.ay, dz = d.z / e.az;
  double A = dx*dx + dy*dy + dz*dz, B = 2*(ox*dx + oy*dy + oz*dz), C = ox*ox + oy*oy + oz*oz - 1;
  double disc = B*B - 4*A*C; if (disc <= 0) return false;
  double s = std::sqrt(disc); tin = (-B - s) / (2*A); tout = (-B + s) / (2*A);
  return tout > 0;
}


static double LEAF_SCALE = 1.0;
// static foliage: 0.2 m leaf voxels, occupied with probability p (deterministic hash => same leaves every scan)
static inline unsigned long long splitmix(unsigned long long x) { x += 0x9e3779b97f4a7c15ULL; x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL; x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL; return x ^ (x >> 31); }
static inline double leaf_hit(V3 o, V3 d, double tin, double tend, double p) {
  for (double s = tin; s < tend; s += 0.07) {
    double qx = o.x + d.x * s, qy = o.y + d.y * s, qz = o.z + d.z * s;
    long long kx = (long long)std::floor(qx / 0.2), ky = (long long)std::floor(qy / 0.2), kz = (long long)std::floor(qz / 0.2);
    unsigned long long h = splitmix((unsigned long long)(kx * 73856093LL) ^ (unsigned long long)(ky * 19349669LL) ^ (unsigned long long)(kz * 83492791LL));
    if ((h & 0xffffULL) < (unsigned long long)(p * 65536.0)) return s;
  }
  return 1e30;
}

static std::vector<V3> simulate_scan(const Forest& f, const Pose& pose, int n_rays, std::mt19937& rng, double max_range) {
  std::uniform_real_distribution<double> U(0, 1);
  std::normal_distribution<double> N(0, 0.02);  // 2 cm range noise
  std::exponential_distribution<double> Ex(1.0);
  V3 o = pose.t;
  std::vector<const Tree*> near_t, near_b;
  for (auto& t : f.trees) if (std::hypot(t.x - o.x, t.y - o.y) < max_range + 5) near_t.push_back(&t);
  for (auto& b : f.bushes) if (std::hypot(b.x - o.x, b.y - o.y) < max_range + 2) near_b.push_back(&b);
  std::vector<V3> pts; pts.reserve(n_rays);
  static const bool inv = getenv("MOUNT_INV") != nullptr;  // sensor mounted upside-down: FOV -52..+7 deg
  const double el0 = (inv ? -52.0 : -7.0) * M_PI / 180, el1 = (inv ? 7.0 : 52.0) * M_PI / 180;
  for (int i = 0; i < n_rays; ++i) {
    double az = 2 * M_PI * U(rng), el = el0 + (el1 - el0) * U(rng);
    V3 db = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
    V3 d = rot(pose.R, db);
    double best = max_range;
    // trunks: vertical cylinders
    for (auto* t : near_t) {
      double ox = o.x - t->x, oy = o.y - t->y;
      double a = std::max(d.x*d.x + d.y*d.y, 1e-12);
      double b = 2*(ox*d.x + oy*d.y), c = ox*ox + oy*oy - t->r*t->r;
      double disc = b*b - 4*a*c;
      if (disc > 0) {
        double th = (-b - std::sqrt(disc)) / (2*a);
        if (th > 0.05 && th < best) { double z = o.z + th * d.z; if (z < t->top - 2 && z > ground(o.x + th*d.x, o.y + th*d.y) - 0.1) best = th; }
      }
      double tin, tout;
      if (ray_ellipsoid(o, d, *t, tin, tout)) {
        tin = std::max(tin, 0.0);
        double th2 = leaf_hit(o, d, tin, std::min(tout, best), 0.10 * LEAF_SCALE);
        if (th2 < best) best = th2;
      }
    }
    for (auto* b : near_b) {
      double tin, tout;
      if (ray_ellipsoid(o, d, *b, tin, tout)) { tin = std::max(tin, 0.0); double th2 = leaf_hit(o, d, tin, std::min(tout, best), 0.25 * LEAF_SCALE); if (th2 < best) best = th2; }
    }
    // ground: march then bisect
    if (d.z < 0) {
      double prev = 0;
      for (double s = 0.25; s < best; s += 0.25) {
        V3 p = add(o, mul(d, s));
        if (p.z < ground(p.x, p.y)) { double lo = prev, hi = s; for (int k = 0; k < 20; ++k) { double m = 0.5*(lo+hi); V3 q = add(o, mul(d, m)); if (q.z < ground(q.x, q.y)) hi = m; else lo = m; } best = std::min(best, hi); break; }
        prev = s;
      }
    }
    if (best >= max_range || best < 0.1) continue;  // no return / blind zone
    double r = best + N(rng);
    pts.push_back(mul(db, r));  // in sensor/body frame
  }
  return pts;
}

// ------------------------- voxel hash map -------------------------
struct Key { int x, y, z; bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; } };
struct KeyHash { size_t operator()(const Key& k) const { return ((size_t)k.x * 73856093u) ^ ((size_t)k.y * 19349669u) ^ ((size_t)k.z * 83492791u); } };
static inline Key key_of(V3 p, double v) { return {(int)std::floor(p.x / v), (int)std::floor(p.y / v), (int)std::floor(p.z / v)}; }

static std::vector<V3> voxel_downsample(const std::vector<V3>& pts, double v) {
  std::unordered_map<Key, V3, KeyHash> g; g.reserve(pts.size());
  for (auto& p : pts) g.emplace(key_of(p, v), p);  // keep first point per voxel (as KISS-ICP)
  std::vector<V3> out; out.reserve(g.size());
  for (auto& kv : g) out.push_back(kv.second);
  return out;
}

struct VoxelMap {
  double v; int max_pts;
  std::unordered_map<Key, std::vector<V3>, KeyHash> m;
  void add_points(const std::vector<V3>& pts) {
    for (auto& p : pts) { auto& vec = m[key_of(p, v)]; if ((int)vec.size() < max_pts) vec.push_back(p); }
  }
  void remove_far(V3 c, double r) {
    for (auto it = m.begin(); it != m.end();) { V3 q = it->second.front(); if (norm(sub(q, c)) > r) it = m.erase(it); else ++it; }
  }
  size_t n_points() const { size_t s = 0; for (auto& kv : m) s += kv.second.size(); return s; }
};

static int G_SCAN = -1;
static double CONV_EPS = 1e-4;
struct Counters { long long dist = 0, corr = 0, xform = 0, iters = 0; };

// nearest neighbour in 27 voxels
static inline bool nearest(const VoxelMap& M, V3 p, V3& q, double& d2, Counters& C) {
  Key k = key_of(p, M.v); d2 = 1e30; bool found = false;
  for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) for (int dz = -1; dz <= 1; ++dz) {
    auto it = M.m.find({k.x + dx, k.y + dy, k.z + dz}); if (it == M.m.end()) continue;
    for (auto& c : it->second) { V3 e = sub(p, c); double dd = dot(e, e); ++C.dist; if (dd < d2) { d2 = dd; q = c; found = true; } }
  }
  return found;
}

static bool solve6(double A[36], double b[6], double x[6]) {  // Cholesky
  double L[36] = {0};
  for (int i = 0; i < 6; ++i) for (int j = 0; j <= i; ++j) {
    double s = A[6*i+j]; for (int k = 0; k < j; ++k) s -= L[6*i+k] * L[6*j+k];
    if (i == j) { if (s <= 1e-12) return false; L[6*i+i] = std::sqrt(s); } else L[6*i+j] = s / L[6*j+j];
  }
  double y[6]; for (int i = 0; i < 6; ++i) { double s = b[i]; for (int k = 0; k < i; ++k) s -= L[6*i+k]*y[k]; y[i] = s / L[6*i+i]; }
  for (int i = 5; i >= 0; --i) { double s = y[i]; for (int k = i+1; k < 6; ++k) s -= L[6*k+i]*x[k]; x[i] = s / L[6*i+i]; }
  return true;
}

// ICP: source points in body frame, initial guess T; returns refined T (body->world)
static Pose icp(const VoxelMap& M, const std::vector<V3>& src, Pose T, double sigma, Counters& C, int& iters_out) {
  double tau = 3.0 * sigma, ks = sigma / 3.0;
  std::vector<V3> cur(src.size());
  for (size_t i = 0; i < src.size(); ++i) cur[i] = apply(T, src[i]);
  C.xform += src.size();
  int it = 0;
  for (; it < 500; ++it) {
    double A[36] = {0}, b[6] = {0};
    for (auto& p : cur) {
      V3 q; double d2; if (!nearest(M, p, q, d2, C)) continue;
      if (d2 > tau * tau) continue;
      ++C.corr;
      V3 r = sub(p, q);
      double w = (ks * ks) / ((ks + dot(r, r)) * (ks + dot(r, r)));
      // J = [I, -skew(p)]  (3x6)
      double J[3][6] = {{1,0,0, 0, p.z, -p.y}, {0,1,0, -p.z, 0, p.x}, {0,0,1, p.y, -p.x, 0}};
      double rr[3] = {r.x, r.y, r.z};
      for (int i = 0; i < 6; ++i) { for (int j = 0; j <= i; ++j) { double s = J[0][i]*J[0][j] + J[1][i]*J[1][j] + J[2][i]*J[2][j]; A[6*i+j] += w * s; }
        b[i] += w * (J[0][i]*rr[0] + J[1][i]*rr[1] + J[2][i]*rr[2]); }
    }
    for (int i = 0; i < 6; ++i) for (int j = i+1; j < 6; ++j) A[6*i+j] = A[6*j+i];
    double nb[6]; for (int i = 0; i < 6; ++i) nb[i] = -b[i];
    double dx[6]; if (!solve6(A, nb, dx)) break;
    if (getenv("ICP_DEBUG") && G_SCAN == atoi(getenv("ICP_DEBUG"))) { double n2=0; for(int i=0;i<6;++i) n2+=dx[i]*dx[i]; std::fprintf(stderr, "it=%d corr=%lld |dx|=%.4g dx=[%.3g %.3g %.3g | %.3g %.3g %.3g] A00=%.3g A33=%.3g A55=%.3g\n", it, C.corr, std::sqrt(n2), dx[0],dx[1],dx[2],dx[3],dx[4],dx[5], A[0], A[21], A[35]); }
    Pose D; so3exp({dx[3], dx[4], dx[5]}, D.R); D.t = {dx[0], dx[1], dx[2]};
    for (auto& p : cur) p = apply(D, p);
    C.xform += cur.size();
    T = compose(D, T);
    double n2 = 0; for (int i = 0; i < 6; ++i) n2 += dx[i]*dx[i];
    if (std::sqrt(n2) < CONV_EPS) { ++it; break; }
  }
  C.iters += it; iters_out = it;
  return T;
}

static long rss_kb() {
  std::ifstream f("/proc/self/status"); std::string line;
  while (std::getline(f, line)) if (line.rfind("VmRSS:", 0) == 0) return std::atol(line.c_str() + 6);
  return -1;
}

int main(int argc, char** argv) {
  int n_rays = argc > 1 ? std::atoi(argv[1]) : 20000;
  double voxel = argc > 2 ? std::atof(argv[2]) : 0.4;
  int n_scans = argc > 3 ? std::atoi(argv[3]) : 600;
  unsigned seed = argc > 4 ? (unsigned)std::atoi(argv[4]) : 1;
  std::string csv = argc > 5 ? argv[5] : "scans.csv";
  const double max_range = 40.0, dt = 0.1;

  if (getenv("LEAF_SCALE")) LEAF_SCALE = atof(getenv("LEAF_SCALE"));
  if (getenv("SPEED")) SPEED = atof(getenv("SPEED"));
  if (getenv("CONV_EPS")) CONV_EPS = atof(getenv("CONV_EPS"));
  if (getenv("YAMP")) YAMP = atof(getenv("YAMP"));
  std::fprintf(stderr, "generating %d scans...\n", n_scans);
  Forest F = make_forest(seed, 400.0);
  std::mt19937 rng(seed * 7 + 3);
  std::normal_distribution<double> vib(0, (getenv("VIB_DEG") ? atof(getenv("VIB_DEG")) : 0.3) * M_PI / 180);  // attitude jitter vibration-induced attitude jitter

  // ground-truth poses + scans
  std::vector<Pose> gt; std::vector<std::vector<V3>> scans;
  auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; k < n_scans; ++k) {
    double t = k * dt; V3 p = traj_pos(t);
    V3 pn = traj_pos(t + 0.05), pp = traj_pos(t - 0.05);
    double yaw = std::atan2(pn.y - pp.y, pn.x - pp.x) + (getenv("YAWOSC") ? atof(getenv("YAWOSC")) : 0.3) * std::sin(0.7 * t);
    double ay = (YAMP * -std::pow(2*M_PI/40.0, 2) * std::sin(2*M_PI*t/40.0));  // lateral accel
    double roll = std::atan2(ay, 9.81) * 0.5 + vib(rng), pitch = -3.0 * M_PI / 180 + vib(rng);
    static double zamp = getenv("ZAMP") ? atof(getenv("ZAMP")) : 0.5;
    p.z = ground(p.x, p.y) + 3.0 + zamp * std::sin(2 * M_PI * t / 15.0);
    Pose P = from_rpy_xyz(roll, pitch, yaw, p);
    gt.push_back(P);
    scans.push_back(simulate_scan(F, P, n_rays, rng, max_range));
  }
  double gen_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  // odometry
  VoxelMap M; M.v = voxel; M.max_pts = getenv("MAXPTS") ? atoi(getenv("MAXPTS")) : 20;
  Counters C; std::vector<Pose> est; est.reserve(n_scans);
  double sigma2_sum = 0; int sigma_n = 0; const double init_th = 2.0, min_motion = 0.1;
  std::ofstream out(csv);
  out << "scan,raw_pts,reg_pts,map_pts_added,iters,ms_total,ms_icp,map_voxels,map_points,sigma,pos_err,ex,ey,ez,pred_err,gt_step,pred_rot_deg,est_rot_deg\n";
  std::clock_t c0 = std::clock();
  auto w0 = std::chrono::steady_clock::now();
  Pose prev = gt[0], prev2 = gt[0];  // start aligned with ground truth (as SLAM origin)
  for (int k = 0; k < n_scans; ++k) {
    auto s0 = std::chrono::steady_clock::now();
    // range filter: drop points closer than 0.5 m (drone body)
    std::vector<V3> pts; pts.reserve(scans[k].size());
    for (auto& p : scans[k]) if (norm(p) > 0.5) pts.push_back(p);
    auto frame = voxel_downsample(pts, 0.5 * voxel);   // for map
    auto source = voxel_downsample(frame, 1.5 * voxel); // for registration
    // constant-velocity prediction
    Pose delta = compose(inverse(prev2), prev);
    Pose pred = (k < 2) ? gt[0] : compose(prev, delta);
    if (getenv("ORACLE") && k > 0) pred = gt[k];
    if (getenv("NOCV") && k > 0) pred = prev;
    double sigma = sigma_n > 0 ? std::sqrt(sigma2_sum / sigma_n) : init_th;  // adaptive threshold; ICP uses 3*sigma and kernel sigma/3
    int iters = 0;
    Pose T = pred;
    auto i0 = std::chrono::steady_clock::now();
    G_SCAN = k;
    if (getenv("PROBE") && k == atoi(getenv("PROBE"))) {
      double offs[6][3] = {{0.05,0,0},{-0.05,0,0},{0,0.05,0},{0,-0.05,0},{0,0,0.05},{0,0,-0.05}};
      for (auto& o : offs) { Pose st = gt[k]; st.t = add(st.t, {o[0],o[1],o[2]}); Counters c2; int it2=0; Pose r = icp(M, source, st, sigma, c2, it2);
        V3 e = sub(r.t, gt[k].t); std::fprintf(stderr, "start off (%.2f %.2f %.2f) -> err (%.4f %.4f %.4f) iters %d\n", o[0],o[1],o[2], e.x,e.y,e.z, it2); }
      double rots[3][3] = {{0.0087,0,0},{0,0.0087,0},{0,0,0.0087}};
      for (auto& w : rots) { Pose st = gt[k]; double Rd[9]; so3exp({w[0],w[1],w[2]}, Rd); double Rn[9]; matmul(Rd, st.R, Rn); std::memcpy(st.R, Rn, sizeof Rn);
        Counters c2; int it2=0; Pose r = icp(M, source, st, sigma, c2, it2); Pose rel = compose(inverse(gt[k]), r); V3 e = sub(r.t, gt[k].t);
        std::fprintf(stderr, "rot off (%.4f %.4f %.4f) -> rot err %.4f deg, pos err (%.4f %.4f %.4f) iters %d\n", w[0],w[1],w[2], norm(so3log(rel.R))*180/M_PI, e.x,e.y,e.z, it2); }
      { Counters c2; int it2=0; Pose r = icp(M, source, gt[k], sigma, c2, it2); V3 e = sub(r.t, gt[k].t); std::fprintf(stderr, "start gt -> err (%.4f %.4f %.4f) iters %d sigma %.3f\n", e.x,e.y,e.z,it2,sigma); }
    }
    if (k > 0) T = icp(M, source, pred, sigma, C, iters);
    double ms_icp = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - i0).count();
    // adaptive threshold update: model deviation between prediction and estimate
    if (k > 0) {
      Pose dev = compose(inverse(pred), T);
      V3 w = so3log(dev.R); double th = norm(w);
      double md = 2 * max_range * std::sin(th / 2) + norm(dev.t);
      if (md > min_motion) { sigma2_sum += md * md; ++sigma_n; }
    }
    // map update (points in world frame)
    std::vector<V3> wf(frame.size()); for (size_t i = 0; i < frame.size(); ++i) wf[i] = apply(T, frame[i]);
    if (!getenv("FREEZE") || k < atoi(getenv("FREEZE"))) M.add_points(wf);
    if (!getenv("NOREMOVE")) M.remove_far(T.t, max_range);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count();
    out << k << "," << scans[k].size() << "," << source.size() << "," << frame.size() << "," << iters << "," << ms << "," << ms_icp << ","
        << M.m.size() << "," << M.n_points() << "," << sigma << "," << norm(sub(T.t, gt[k].t)) << "," << T.t.x-gt[k].t.x << "," << T.t.y-gt[k].t.y << "," << T.t.z-gt[k].t.z << "," << norm(sub(pred.t, gt[k].t)) << "," << (k>0 ? norm(sub(gt[k].t, gt[k-1].t)) : 0.0) << "," << norm(so3log(compose(inverse(gt[k]), pred).R))*180/M_PI << "," << norm(so3log(compose(inverse(gt[k]), T).R))*180/M_PI << "\n";
    prev2 = prev; prev = T; est.push_back(T);
    if ((k + 1) % 5 == 0) out.flush();  // lets tools/monitor.py show live progress
  }
  double cpu_s = double(std::clock() - c0) / CLOCKS_PER_SEC;
  double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();

  // accuracy
  double path = 0; for (int k = 1; k < n_scans; ++k) path += norm(sub(gt[k].t, gt[k-1].t));
  V3 e_end = sub(est.back().t, gt.back().t);
  double ate2 = 0; for (int k = 0; k < n_scans; ++k) { V3 e = sub(est[k].t, gt[k].t); ate2 += dot(e, e); }
  Pose rel = compose(inverse(gt.back()), est.back()); double rot_err = norm(so3log(rel.R)) * 180 / M_PI;
  double flops = 8.0 * C.dist + 180.0 * C.corr + 18.0 * C.xform;
  size_t mp = M.n_points();
  std::printf("rays/scan=%d voxel=%.2f scans=%d seed=%u trees=%zu bushes=%zu\n", n_rays, voxel, n_scans, seed, F.trees.size(), F.bushes.size());
  std::printf("scan_gen_s=%.1f odom_wall_s=%.3f odom_cpu_s=%.3f ms_per_scan=%.2f\n", gen_s, wall_s, cpu_s, 1000 * wall_s / n_scans);
  std::printf("iters_avg=%.2f dist_evals=%lld corr=%lld xforms=%lld est_flop=%.3e est_GFLOPs=%.3f\n",
              double(C.iters) / std::max(1, n_scans - 1), C.dist, C.corr, C.xform, flops, flops / cpu_s / 1e9);
  std::printf("path_m=%.1f end_err_m=%.3f drift_pct=%.3f ate_rmse_m=%.3f rot_err_deg=%.3f\n", path, norm(e_end), 100 * norm(e_end) / path, std::sqrt(ate2 / n_scans), rot_err);
  std::printf("map_voxels=%zu map_points=%zu map_bytes_points=%zu rss_kb=%ld\n", M.m.size(), mp, mp * sizeof(V3), rss_kb());
  return 0;
}
