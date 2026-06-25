#!/usr/bin/env python3
"""Generate SDF XML for smooth body surface in massage.world."""
import math

segments = [
    (0.27, 0.205, 0.090, 0.080),
    (0.33, 0.192, 0.065, 0.045),
    (0.38, 0.197, 0.120, 0.048),
    (0.43, 0.195, 0.160, 0.050),
    (0.49, 0.193, 0.180, 0.052),
    (0.55, 0.189, 0.170, 0.048),
    (0.61, 0.186, 0.155, 0.046),
    (0.67, 0.183, 0.140, 0.044),
    (0.72, 0.179, 0.128, 0.042),
    (0.77, 0.175, 0.130, 0.040),
    (0.82, 0.173, 0.140, 0.038),
    (0.86, 0.171, 0.150, 0.036),
]

xs = [s[0] for s in segments]
zs = [s[1] for s in segments]
hws = [s[2] for s in segments]
ths = [s[3] for s in segments]

def catmull_rom(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)

n = len(segments)
resolution = 0.020
mat_top = 0.14

x_min = xs[0] - 0.04
x_max = xs[-1] + 0.02
num = int((x_max - x_min) / resolution) + 1

pts = []
for i in range(num):
    x = x_min + i * resolution
    si = 0
    for j in range(n - 1):
        if x <= xs[j + 1]:
            si = j
            break
    if x >= xs[-1]:
        si = n - 2

    i0 = max(0, si - 1)
    i1 = si
    i2 = min(n - 1, si + 1)
    i3 = min(n - 1, si + 2)

    if xs[i2] - xs[i1] > 1e-6:
        t = (x - xs[i1]) / (xs[i2] - xs[i1])
    else:
        t = 0.0
    t = max(0.0, min(1.0, t))

    z_s = catmull_rom(zs[i0], zs[i1], zs[i2], zs[i3], t)
    hw = catmull_rom(hws[i0], hws[i1], hws[i2], hws[i3], t)
    th = catmull_rom(ths[i0], ths[i1], ths[i2], ths[i3], t)

    z_s = max(0.16, min(0.22, z_s))
    hw = max(0.05, min(0.18, hw))
    th = max(0.03, min(0.09, th))

    r = th * 0.55
    length = hw * 2.0
    z_c = max(mat_top + r, z_s - 0.008)

    pts.append((x, z_c, z_s, length, r))

print(f'<!-- Generated {len(pts)} surface points from x={x_min:.3f} to x={x_max:.3f} -->')
for idx, (x, z_c, z_s, length, r) in enumerate(pts):
    print(f'      <!-- pt {idx}: x={x:.3f} z_surf={z_s:.3f} r={r:.4f} len={length:.3f} -->')
    print(f'      <link name="back_{idx:02d}"><pose>{x:.3f} 0 {z_c:.3f} 1.5708 0 0</pose>')
    print(f'        <collision name="c"><geometry><cylinder><radius>{r:.4f}</radius><length>{length:.3f}</length></cylinder></geometry></collision>')
    print(f'        <visual name="v"><geometry><cylinder><radius>{r:.4f}</radius><length>{length:.3f}</length></cylinder></geometry>')
    print(f'          <material><ambient>0.86 0.72 0.60 1</ambient><diffuse>0.86 0.72 0.60 1</diffuse></material></visual>')
    print(f'      </link>')

# Also print limb positions
print()
print("<!-- ============ Limb Z-values (all >= mat_top + 0.04 = 0.18) ============ -->")
limb_z = mat_top + 0.04
for name, side, x, y in [
    ("left_upper_arm", -1, 0.49, 0.225),
    ("right_upper_arm", 1, 0.49, 0.225),
    ("left_forearm", -1, 0.64, 0.255),
    ("right_forearm", 1, 0.64, 0.255),
    ("left_hand", -1, 0.76, 0.26),
    ("right_hand", 1, 0.76, 0.26),
    ("left_thigh", -1, 0.935, 0.11),
    ("right_thigh", 1, 0.935, 0.11),
    ("left_calf", -1, 1.12, 0.12),
    ("right_calf", 1, 1.12, 0.12),
    ("left_foot", -1, 1.30, 0.12),
    ("right_foot", 1, 1.30, 0.12),
]:
    print(f"  {name}: z should be >= {limb_z:.3f} (was broken)")
