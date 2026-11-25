"""Map memory for a 100 x 100 m forest plot: build a voxel model of the same synthetic forest
(400 trees/ha, crowns 5-9 m wide, undergrowth) and measure raw + compressed sizes of several map formats.
Also transfer times over weak radio links."""
import numpy as np, zlib, lzma, json

rng = np.random.default_rng(1)
A, H = 100.0, 30.0                    # plot size (m), height band above lowest ground (m)

def ground(x, y):
    return 0.5*np.sin(0.05*x)*np.cos(0.07*y) + 0.15*np.sin(0.3*x + 0.2*y)

# --- forest parameters (same as the C++ simulator) ---
n_trees = int(400 * A * A / 1e4)
trees = []
while len(trees) < n_trees:
    x, y = rng.uniform(0, A, 2)
    if all((x-t[0])**2 + (y-t[1])**2 > 4.0 for t in trees):
        r = 0.08 + 0.17*rng.random(); h = 14 + 10*rng.random(); g = ground(x, y)
        az = min(3 + 3*rng.random(), (h - 6.5)/2); ax = 2.5 + 2.0*rng.random(); ay = ax*(0.8 + 0.4*rng.random())
        trees.append((x, y, r, g + h, x + 0.5*(rng.random()-0.5), y + 0.5*(rng.random()-0.5), g + h - az, ax, ay, az))
bushes = [(x, y, 0.5+0.7*rng.random(), 0.4+0.5*rng.random()) for x, y in rng.uniform(0, A, (int(0.5*n_trees), 2))]

def build(res):
    nx = ny = int(round(A/res)); nz = int(round(H/res))
    occ = np.zeros((nx, ny, nz), dtype=bool)
    xs = (np.arange(nx) + 0.5)*res; ys = (np.arange(ny) + 0.5)*res; zs = (np.arange(nz) + 0.5)*res - 1.0  # z from -1 m
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    gz = ground(X, Y)
    kz = np.clip(((gz + 1.0)/res).astype(int), 0, nz-1)
    occ[np.arange(nx)[:, None], np.arange(ny)[None, :], kz] = True                       # ground surface
    for (x, y, r, top, cx, cy, cz, ax, ay, az) in trees:
        i0, i1 = int((x-r-res)/res), int((x+r+res)/res)+1; j0, j1 = int((y-r-res)/res), int((y+r+res)/res)+1
        i0, j0 = max(i0, 0), max(j0, 0); i1, j1 = min(i1, nx), min(j1, ny)
        if i0 >= i1 or j0 >= j1: continue
        sx, sy = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
        d = np.hypot(sx-x, sy-y)
        shell = np.abs(d - r) <= res*0.75                                                 # trunk surface
        zk0 = int((ground(x, y)+1.0)/res); zk1 = min(int((top-2+1.0)/res), nz)
        occ[i0:i1, j0:j1, zk0:zk1] |= shell[:, :, None]
        # crown: leaf voxels inside ellipsoid with probability p per 0.2 m leaf cell (scaled to res)
        i0, i1 = max(int((cx-ax)/res), 0), min(int((cx+ax)/res)+1, nx); j0, j1 = max(int((cy-ay)/res), 0), min(int((cy+ay)/res)+1, ny)
        k0, k1 = max(int((cz-az+1.0)/res), 0), min(int((cz+az+1.0)/res)+1, nz)
        if i0 >= i1 or j0 >= j1 or k0 >= k1: continue
        sx, sy, sz = np.meshgrid(xs[i0:i1], ys[j0:j1], zs[k0:k1], indexing='ij')
        inside = ((sx-cx)/ax)**2 + ((sy-cy)/ay)**2 + ((sz-cz)/az)**2 <= 1
        p = 1 - (1 - 0.10)**((res/0.2)**3)                                                 # same leaf density at any res
        occ[i0:i1, j0:j1, k0:k1] |= inside & (rng.random(inside.shape) < p)
    for (x, y, a, c) in bushes:
        g = ground(x, y)
        i0, i1 = max(int((x-a)/res), 0), min(int((x+a)/res)+1, nx); j0, j1 = max(int((y-a)/res), 0), min(int((y+a)/res)+1, ny)
        k0, k1 = max(int((g+1.0)/res), 0), min(int((g+2*c+1.0)/res)+1, nz)
        sx, sy, sz = np.meshgrid(xs[i0:i1], ys[j0:j1], zs[k0:k1], indexing='ij')
        inside = ((sx-x)/a)**2 + ((sy-y)/a)**2 + ((sz-(g+0.8*c))/c)**2 <= 1
        p = 1 - (1 - 0.25)**((res/0.2)**3)
        occ[i0:i1, j0:j1, k0:k1] |= inside & (rng.random(inside.shape) < p)
    return occ, gz

def sz(b): return len(b)
def comp(b): return len(zlib.compress(b, 9)), len(lzma.compress(b, preset=9))

out = {}
for res in (0.2, 0.5):
    occ, gz = build(res)
    nx, ny, nz = occ.shape; n_all = occ.size; n_occ = int(occ.sum())
    bits = np.packbits(occ.ravel()).tobytes()
    idx = np.argwhere(occ).astype(np.uint16)                       # sparse list, sorted (C order)
    lin = np.ravel_multi_index(idx.T, occ.shape).astype(np.int64)
    delta = np.diff(lin, prepend=0).astype(np.uint32).tobytes()   # delta-coded sorted indices
    pts = ((idx.astype(np.float32) + 0.5)*res).astype(np.float32).tobytes()
    r = dict(res=res, grid=f"{nx}x{ny}x{nz}", voxels=n_all, occupied=n_occ, occ_frac=n_occ/n_all,
             dense_uint8_MB=n_all/1e6, dense_bits_MB=len(bits)/1e6,
             dense_bits_zlib_MB=comp(bits)[0]/1e6, dense_bits_lzma_MB=comp(bits)[1]/1e6,
             points_xyz_f32_MB=len(pts)/1e6, sparse_idx_u16_MB=idx.nbytes/1e6,
             sparse_delta_lzma_MB=comp(delta)[1]/1e6,
             hash_map_est_MB=n_occ*32/1e6)  # ~32 B per voxel in a robin-hood hash (key 12 B + value + load factor)
    out[f"3d_{res}"] = r
    # 2D obstacle slice for flight band 2-5 m above ground
    if res == 0.2:
        zrel = (np.arange(nz)*res - 1.0)[None, None, :] - gz[:, :, None]
        band = occ & (zrel >= 2.0) & (zrel <= 5.0)
        slice2d = band.any(axis=2)
        b2 = np.packbits(slice2d.ravel()).tobytes()
        out["2d_slice_0.2"] = dict(cells=slice2d.size, occupied=int(slice2d.sum()), raw_uint8_MB=slice2d.size/1e6,
                                  bits_KB=len(b2)/1e3, zlib_KB=comp(b2)[0]/1e3, lzma_KB=comp(b2)[1]/1e3)
        # 2.5D height map at 0.5 m: max occupied height per cell (uint16 cm)
        top = np.where(occ.any(axis=2), (nz - 1 - np.argmax(occ[:, :, ::-1], axis=2))*res - 1.0, 0.0)
        hm_b = (np.clip(top, 0, 650)*100).astype(np.uint16).tobytes()
        out["2.5d_height_0.2"] = dict(cells=top.size, raw_KB=len(hm_b)/1e3, zlib_KB=comp(hm_b)[0]/1e3, lzma_KB=comp(hm_b)[1]/1e3)

# 2D occupancy grid (uint8 per cell) sizes for the 100 x 100 m plot at several resolutions
for r2 in (0.05, 0.10, 0.20):
    n = int(round(A/r2))**2
    out[f"2d_grid_{r2}"] = dict(cells=n, uint8_MB=n/1e6, float32_MB=4*n/1e6, hector_3levels_uint8_MB=n*(1+1/4+1/16)/1e6)

# weak-link transfer times (s) for representative payloads
links = {"10 kbit/s": 10e3, "50 kbit/s": 50e3, "250 kbit/s": 250e3, "1 Mbit/s": 1e6}
payloads = {
    "pose+cov @10Hz, 1 s (100 B/msg)": 1000,
    "Scan Context 20x60 uint8 (1 keyframe)": 1200,
    "2D slice 0.2 m, lzma": out["2d_slice_0.2"]["lzma_KB"]*1e3,
    "3D 0.5 m bitset, lzma": out["3d_0.5"]["dense_bits_lzma_MB"]*1e6,
    "3D 0.2 m bitset, lzma": out["3d_0.2"]["dense_bits_lzma_MB"]*1e6,
    "3D 0.2 m voxel centres as xyz float32": out["3d_0.2"]["points_xyz_f32_MB"]*1e6,
}
out["transfer_s"] = {p: {l: round(8*b/bw, 2) for l, bw in links.items()} for p, b in payloads.items()}
print(json.dumps(out, indent=1, ensure_ascii=False))
