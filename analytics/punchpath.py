"""Punch paths from a VIZ recording, same rules as the live viewer.

A punch starts on the firmware's punch flag and reaches back to the onset of
acceleration; it ends at full extension (deceleration over, or forward speed
under 12 % of peak). The velocity left at extension is removed as a linear ramp,
because the hand really is stopped there.
"""
import json
import math

G = 9.80665


def qrot(q, v):
    w, x, y, z = q
    # v' = q v q*
    ix = w * v[0] + y * v[2] - z * v[1]
    iy = w * v[1] + z * v[0] - x * v[2]
    iz = w * v[2] + x * v[1] - y * v[0]
    iw = -x * v[0] - y * v[1] - z * v[2]
    return (ix * w + iw * -x + iy * -z - iz * -y,
            iy * w + iw * -y + iz * -x - ix * -z,
            iz * w + iw * -z + ix * -y - iy * -x)


def load(path):
    out = []
    for line in open(path):
        if not line.strip():
            continue
        t, qw, qx, qy, qz, ax, ay, az, rest, flags = json.loads(line)
        n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1
        q = (qw / n, qx / n, qy / n, qz / n)
        aw = qrot(q, (ax / 1000, ay / 1000, az / 1000))
        lin_g = (aw[0], aw[1], aw[2] - rest / 1000)
        out.append(dict(t=t, q=q, lin=tuple(c * G for c in lin_g), g=math.sqrt(sum(c * c for c in lin_g)),
                        raw=math.sqrt(ax * ax + ay * ay + az * az) / 1000, flags=flags))
    return out


def punches(S):
    res, i, armed, last_end = [], 1, True, -1e9
    while i < len(S):
        on = S[i]["flags"] & 2
        if not on:
            armed = True
        if on and armed and S[i]["t"] - last_end > 250:
            j = i
            while j > 0 and S[j - 1]["g"] > 0.5 and S[i]["t"] - S[j - 1]["t"] < 200:
                j -= 1
            j = max(0, j - 1)
            seg = trace(S, j)
            if seg:
                res.append(seg)
                last_end = seg["t_end"]
                i = seg["i_end"]
            armed = False
        i += 1
    return res


def trace(S, j):
    v = [0.0, 0.0, 0.0]
    vs, vpeak, dpk, decel = [tuple(v)], 0.0, (1, 0, 0), False
    k = j + 1
    while k < len(S):
        dt = (S[k]["t"] - S[k - 1]["t"]) / 1000
        a = S[k]["lin"]
        v = [v[c] + a[c] * dt for c in range(3)]
        vs.append(tuple(v))
        sp = math.sqrt(sum(c * c for c in v))
        if sp > vpeak:
            vpeak, dpk = sp, tuple(c / sp for c in v)
        along = sum(v[c] * dpk[c] for c in range(3))
        acc = sum(a[c] * dpk[c] for c in range(3))
        if vpeak > 0.6 and sp < 0.92 * vpeak and acc < -G:
            decel = True
        if (vpeak > 0.6 and sp < 0.92 * vpeak and (along <= 0.12 * vpeak or (decel and acc > -0.3 * G))) or S[k]["t"] - S[j]["t"] > 450:
            break
        k += 1
    k = min(k, len(S) - 1)
    seg = S[j:k + 1]
    if len(seg) < 8:
        return None
    T = seg[-1]["t"] - seg[0]["t"]
    vend = vs[len(seg) - 1]
    p, pts, speeds, length = [0.0, 0.0, 0.0], [], [], 0.0
    for n, s in enumerate(seg):
        f = (s["t"] - seg[0]["t"]) / T if T else 0
        vc = [vs[n][c] - vend[c] * f for c in range(3)]
        if n:
            dt = (s["t"] - seg[n - 1]["t"]) / 1000
            step = [vc[c] * dt for c in range(3)]
            p = [p[c] + step[c] for c in range(3)]
            length += math.sqrt(sum(c * c for c in step))
        pts.append(tuple(p))
        speeds.append(math.sqrt(sum(c * c for c in vc)))
    reach = math.sqrt(sum(c * c for c in p))
    peak_g = max(s["raw"] for s in seg)
    sat = any(s["flags"] & 4 for s in seg)
    pre_quiet = all(x["g"] < 0.35 for x in S[max(0, j - 40):j])          # 200 ms of calm before the onset
    return dict(i0=j, i_end=k, t0=seg[0]["t"], t_end=seg[-1]["t"], ms=T, pts=pts, speeds=speeds,
                t=[s["t"] for s in seg], q=[s["q"] for s in seg], lin=[s["lin"] for s in seg],
                vpeak=max(speeds), reach=reach, length=length, straight=reach / length if length else 0,
                peak_g=peak_g, saturated=sat, pre_quiet=pre_quiet, drift=math.sqrt(sum(c * c for c in vend)))
