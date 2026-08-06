#!/usr/bin/env python3
"""지자기 보정 도구 — 텀블 수집 + 타원체 피팅 + 축 순열 탐색.

이 로봇의 기본 캘리브레이션(btn2 롱프레스 / /calibrate_mag)은 축별 min/max 중점을
쓴다. 손으로 돌려서 각 축의 진짜 극값에 닿아야만 맞는 방식이라, 실제로 2026-08-06에
같은 조건에서 돌린 결과가 **무보정보다 나빴다** (흩어짐 38.1% vs 22.1%). 이 도구는
모든 점을 쓰는 타원체 피팅이라 극값에 안 닿아도 중심을 푼다.

장소가 바뀌어도 그대로 쓸 수 있게 만들었다. 보정이 잡는 하드/소프트아이언은 **로봇 몸에
붙어 있는 것**이라 장소를 따라가지 않는다. 반대로 부품을 붙이거나 떼면 무효다.

사용:
    ros2 run fish_robot_pkg magneto_cal.py                     # 수집 -> 피팅 -> 파일 저장
    ros2 run fish_robot_pkg magneto_cal.py --no-write          # 저장 없이 결과만
    ros2 run fish_robot_pkg magneto_cal.py --csv <파일>        # 이전에 모은 데이터로 다시 계산 (로봇 불필요)
    ros2 run fish_robot_pkg magneto_cal.py --search-axes       # 축 재배치 24가지를 전부 시험
    ros2 run fish_robot_pkg magneto_cal.py --dip 143           # 그 지역의 복각 (기본값 한국 143도)

축 재배치를 모르는 새 하드웨어라면 --search-axes 를 쓴다. 복각(자기장과 중력 사이 각)은
그 지역에서 물리적으로 정해진 값이라, 24가지 오른손 좌표계 후보 중 그 값을 내는 것이
정답이다. 하드아이언이 클 때는 이 판정이 불가능하므로 **반드시 보정 후에** 본다.
"""

import argparse
import math
import os
import shutil
import sys
import time

import numpy as np

# ── 이 로봇 고유의 축 규약 ────────────────────────────────────────────────────
# IMU: nRF52840가 보내는 원시 축 -> FLU (앞-왼쪽-위). 2026-08-04/06 실측 검증 완료.
IMU_TO_FLU = lambda x, y, z: (-z, -x, y)          # noqa: E731
# 지자기: 칩 축 -> FLU. 2026-08-06 텀블로 확인 (복각 144.5도 vs 한국 143도).
MAG_TO_FLU = lambda x, y, z: (y, x, -z)           # noqa: E731

DEFAULT_OUT = os.path.expanduser("~/ros2_ws/log_csv/mag_calib_params.txt")
DEFAULT_LOGDIR = os.path.expanduser("~/ros2_ws/log_csv")

NBIN = 8            # 자세마다 제자리 회전을 8구간(45도)으로 나눈다
NEED_BIN = 6        # 그중 6구간(270도) 이상 채우면 그 자세는 완료
AXIS_MIN = 0.80     # 중력이 한 축에 이만큼 쏠려야 그 자세로 인정
SPIN_MIN = 5.0      # deg/s. 이보다 느리면 회전으로 안 친다 (자이로 바이어스 방지)
MAX_SPHERICITY = 8.0   # 보정 후 |m| 흩어짐이 이 %를 넘으면 저장하지 않는다

POSES = [
    ("+z", "① 평소 자세 (배가 아래)"),
    ("-z", "② 뒤집기 (배가 위)"),
    ("-y", "③ 왼쪽 옆으로 눕히기"),
    ("+y", "④ 오른쪽 옆으로 눕히기"),
    ("+x", "⑤ 코를 위로 세우기"),
    ("-x", "⑥ 코를 아래로"),
]


# ── 피팅 ──────────────────────────────────────────────────────────────────────
def fit_sphere(D):
    """구 피팅(중심3 + 반지름1). 선형이라 거의 항상 풀린다."""
    A = np.column_stack([2 * D[:, 0], 2 * D[:, 1], 2 * D[:, 2], np.ones(len(D))])
    p, *_ = np.linalg.lstsq(A, (D ** 2).sum(axis=1), rcond=None)
    c = p[:3]
    rr = p[3] + float(c @ c)
    return (c, math.sqrt(rr)) if rr > 0 else (None, None)


def fit_ellipsoid(M):
    """축정렬 타원체 피팅. 반환 (center[3], radii[3], 방법이름).

    ※ 원시값을 그대로 넣으면 안 된다. 자기장 덩어리가 원점에서 수십 uT 떨어져 있으면
      x^2 항들이 거의 평행해져 정규방정식이 병들고, 반지름이 음수로 나와 실패한다.
      평균을 먼저 빼서 원점 근처로 옮긴 뒤 풀고, 결과에 다시 더한다.
    """
    m0 = M.mean(axis=0)
    D = M - m0
    x, y, z = D[:, 0], D[:, 1], D[:, 2]
    A = np.column_stack([x * x, y * y, z * z, x, y, z])
    p, *_ = np.linalg.lstsq(A, np.ones(len(x)), rcond=None)
    a, b, c, d, e, f = p

    if a > 0 and b > 0 and c > 0:
        cen = np.array([-d / (2 * a), -e / (2 * b), -f / (2 * c)])
        k = 1.0 + a * cen[0] ** 2 + b * cen[1] ** 2 + c * cen[2] ** 2
        rad = np.sqrt(k / np.array([a, b, c]))
        # 세 반지름이 2배 넘게 차이나면 회전이 부족한 것. 구 피팅이 더 안전하다.
        if rad.max() / rad.min() < 2.0:
            return m0 + cen, rad, "타원체"

    cen, r = fit_sphere(D)
    if cen is None:
        return None, None, None
    return m0 + cen, np.array([r, r, r]), "구 (소프트아이언 미보정)"


# ── 축 순열 탐색 ──────────────────────────────────────────────────────────────
def right_handed_frames():
    """부호 있는 축 순열 중 det=+1 인 24가지. (행렬, 사람이 읽는 이름)."""
    out = []
    for perm in ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)):
        for sx in (1, -1):
            for sy in (1, -1):
                for sz in (1, -1):
                    R = np.zeros((3, 3))
                    R[0, perm[0]] = sx
                    R[1, perm[1]] = sy
                    R[2, perm[2]] = sz
                    if round(np.linalg.det(R)) == 1:
                        sgn = ("+" if s > 0 else "-" for s in (sx, sy, sz))
                        name = ",".join(s + "xyz"[p] for s, p in zip(sgn, perm))
                        out.append((R, f"({name})"))
    return out


def dip_of(M_flu, G_flu):
    """자기장과 중력 사이 각[도]. 회전 불변량이라 자세와 무관하게 일정해야 한다."""
    m = M_flu / np.linalg.norm(M_flu, axis=1, keepdims=True)
    g = G_flu / np.linalg.norm(G_flu, axis=1, keepdims=True)
    return np.degrees(np.arccos(np.clip((m * g).sum(axis=1), -1.0, 1.0)))


# ── 수집 ──────────────────────────────────────────────────────────────────────
def record(mag_topic, imu_topic):
    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class Tumble(Node):
        def __init__(self):
            super().__init__("magneto_cal")
            self.mag = self.acc = self.gyr = None
            self.M, self.A = [], []
            self.bins = {k: set() for k, _ in POSES}
            self.spin = {k: 0.0 for k, _ in POSES}
            self.cur = None
            self.prev_t = None
            self.last_draw = 0.0
            self.drawn = 0
            self.create_subscription(Vector3, mag_topic, self.cb_m, 50)
            self.create_subscription(Imu, imu_topic, self.cb_a, 50)
            self.create_timer(0.05, self.tick)

        def cb_m(self, v):
            self.mag = (v.x, v.y, v.z)

        def cb_a(self, msg):
            l, w = msg.linear_acceleration, msg.angular_velocity
            self.acc = IMU_TO_FLU(l.x, l.y, l.z)
            self.gyr = IMU_TO_FLU(w.x, w.y, w.z)

        def pose_of(self, a):
            g = np.array(a, dtype=float)
            n = np.linalg.norm(g)
            if n < 1e-6:
                return None, None
            g /= n
            i = int(np.argmax(np.abs(g)))
            if abs(g[i]) < AXIS_MIN:
                return None, None          # 어중간하게 기울어짐 - 세지 않는다
            return ("+" if g[i] > 0 else "-") + "xyz"[i], g

        def tick(self):
            if self.mag is None or self.acc is None or self.gyr is None:
                return
            self.M.append(self.mag)
            self.A.append(self.acc)

            now = time.time()
            dt = 0.0 if self.prev_t is None else min(now - self.prev_t, 0.2)
            self.prev_t = now

            key, g = self.pose_of(self.acc)
            self.cur = key
            if key is not None and dt > 0.0:
                # 진행도는 **자이로**로 잰다. 자기장 방향으로 재면 안 된다 - 하드아이언이
                # 지구장보다 크면 한 바퀴 돌려도 방향이 한 바퀴를 돌지 않기 때문이다
                # (실측: 360도 회전에 102도만 움직이고 되돌아옴). 보정하려는 대상으로
                # 진행도를 재는 셈이라, 자세에 따라 즉시 차거나 영영 안 차게 된다.
                rate = float(np.dot(np.array(self.gyr), g))
                if abs(rate) > SPIN_MIN:
                    self.spin[key] += rate * dt
                self.bins[key].add(int((self.spin[key] % 360.0) // (360.0 / NBIN)))

            if now - self.last_draw < 0.3:
                return
            self.last_draw = now
            self.render()
            if all(len(self.bins[k]) >= NEED_BIN for k, _ in POSES):
                print("\n  6가지 자세 모두 완료.")
                raise SystemExit(0)

        def render(self):
            lines, done = [], 0
            for k, label in POSES:
                n = len(self.bins[k])
                done += n >= NEED_BIN
                bar = "".join("█" if i in self.bins[k] else "░" for i in range(NBIN))
                mark = "✅" if n >= NEED_BIN else ("▶ " if k == self.cur else "  ")
                tip = "  ← 이 자세로 계속 도세요" if (k == self.cur and n < NEED_BIN) else ""
                lines.append(f"  {mark}{label:22} {bar} {abs(self.spin[k]):4.0f}도{tip}")
            lines.append(f"     자세 {done}/6   샘플 {len(self.M)}"
                         + ("   (자세를 확실히 취해주세요)" if self.cur is None else ""))
            if self.drawn:
                sys.stdout.write(f"\033[{self.drawn}A")
            sys.stdout.write("\n".join(s.ljust(78) for s in lines) + "\n")
            sys.stdout.flush()
            self.drawn = len(lines)

    rclpy.init()
    node = Tumble()
    print("=" * 78)
    print("  지자기 보정 — 텀블 수집")
    print("    · 아래 6가지 자세를 하나씩 잡고, 그 자세 그대로 제자리에서 한 바퀴 도세요")
    print("      (로봇을 들고 본인이 도는 게 제일 쉽습니다. 책상에 놓고 돌려도 됩니다)")
    print("    · 한자리에서. 들고 돌아다니면 주변 자기장이 바뀌어 피팅이 오염됩니다")
    print("    · 철제 책상·본체·스피커에서 1m 이상 떨어져서")
    print("    · Ctrl-C 로 중단해도 그때까지 데이터로 계산합니다")
    print("=" * 78)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        M, A = node.M, node.A
        node.destroy_node()
        rclpy.try_shutdown()
    return np.array(M), np.array(A)


# ── 분석 ──────────────────────────────────────────────────────────────────────
def analyze(M, A, args):
    print("\n" + "=" * 70)
    print("  결과")
    print("=" * 70)
    print(f"  샘플 {len(M)}점")
    print(f"  축별 변화폭   x {M[:,0].ptp():6.1f}   y {M[:,1].ptp():6.1f}   z {M[:,2].ptp():6.1f} µT"
          "   (셋 다 40 이상이어야 정상)")

    cen, rad, how = fit_ellipsoid(M)
    if cen is None:
        print("\n  ❌ 피팅 실패 — 회전이 한 평면에만 몰려 있습니다.")
        print("     위 '축별 변화폭'에서 작은 축 방향으로 더 뒤집어야 합니다.")
        return None
    scale = rad.mean() / rad
    C = (M - cen) * scale
    n = np.linalg.norm(C, axis=1)
    err = float(n.std() / n.mean() * 100.0)
    raw_n = np.linalg.norm(M, axis=1)

    print(f"  피팅 방법     {how}")
    print(f"\n  하드아이언   {cen[0]:+8.3f} {cen[1]:+8.3f} {cen[2]:+8.3f}  |{np.linalg.norm(cen):.1f}| µT")
    print(f"  세 반지름    {rad[0]:8.3f} {rad[1]:8.3f} {rad[2]:8.3f}  µT")
    print(f"  소프트아이언 {scale[0]:8.4f} {scale[1]:8.4f} {scale[2]:8.4f}")
    print(f"\n  보정 전 |m|  {raw_n.mean():6.2f} ± {raw_n.std():5.2f} µT  "
          f"(흩어짐 {raw_n.std()/raw_n.mean()*100:.1f}%)")
    print(f"  보정 후 |m|  {n.mean():6.2f} ± {n.std():5.2f} µT  (흩어짐 {err:.1f}%)  ← 낮을수록 좋음")

    # 복각 — 정지 구간만 (손 흔들림이 가속도를 오염시키므로)
    still = np.abs(np.linalg.norm(A, axis=1) - np.median(np.linalg.norm(A, axis=1))) < 0.03
    if still.sum() < 50:
        still = np.ones(len(A), dtype=bool)
    m_flu = np.column_stack(MAG_TO_FLU(C[:, 0], C[:, 1], C[:, 2]))
    d = dip_of(m_flu[still], A[still])
    print(f"\n  복각         {d.mean():6.1f} ± {d.std():4.1f}도   (이 지역 기대값 {args.dip:.0f}도)"
          f"   정지 {int(still.sum())}점 기준")

    if args.search_axes:
        print("\n  [축 재배치 24가지 탐색] — 복각이 기대값에 가깝고 흔들림이 작은 것이 정답")
        cand = []
        for R, name in right_handed_frames():
            dd = dip_of((R @ C[still].T).T, A[still])
            cand.append((abs(dd.mean() - args.dip) + dd.std(), dd.mean(), dd.std(), name))
        cand.sort()
        for i, (_, mu, sd, name) in enumerate(cand[:5]):
            cur = "  ← 현재 코드" if name == "(+y,+x,-z)" else ""
            print(f"    {i+1}. {name:14} 복각 {mu:6.1f} ± {sd:4.1f}도{cur}")

    # 가속도 점검 (덤) — 같은 데이터로 공짜
    S = A[still]
    an = np.linalg.norm(S, axis=1)
    print(f"\n  [가속도 점검] |a| {an.mean():.4f} ± {an.std():.4f} g  "
          f"(흩어짐 {an.std()/an.mean()*100:.2f}%)")

    print("\n" + "-" * 70)
    ok = err < MAX_SPHERICITY
    if not ok:
        print(f"  ❌ 보정 후에도 구가 안 됩니다 (흩어짐 {err:.1f}% > {MAX_SPHERICITY}%).")
        print("     회전이 부족했거나 텀블 중 위치를 옮겼을 가능성이 큽니다. 저장하지 않습니다.")
    else:
        print(f"  ✅ 구형도 {err:.1f}% — 쓸 만합니다.")
        if abs(d.mean() - args.dip) > 20.0:
            print(f"  ⚠ 복각 {d.mean():.0f}도가 기대값 {args.dip:.0f}도와 다릅니다.")
            print("     축 재배치(MAG_TO_FLU)가 틀렸을 수 있습니다. --search-axes 로 확인하세요.")
            print("     roll/pitch는 중력이 잡으므로 제어에는 무해하지만 yaw는 못 믿습니다.")
        else:
            print(f"  ✅ 복각 {d.mean():.0f}도 — 축 재배치까지 맞다는 증거입니다.")
    print("=" * 70)
    return (cen, scale) if ok else None


def write_params(cen, scale, out):
    if os.path.exists(out):
        bak = out + ".bak_" + time.strftime("%Y%m%d_%H%M%S")
        shutil.copy2(out, bak)
        print(f"  기존 파일 백업 → {bak}")
    with open(out, "w") as f:
        f.write(f"{cen[0]:.6g} {cen[1]:.6g} {cen[2]:.6g}\n")
        f.write(f"{scale[0]:.6g} {scale[1]:.6g} {scale[2]:.6g}\n")
    print(f"  ✅ 저장 → {out}")
    print("     ※ state_estimation_node 를 재시작해야 반영됩니다 (시작 시 1회만 읽음)")
    print("     ※ btn2 롱프레스는 이 파일을 min/max 방식으로 덮어씁니다. 누르지 마세요")


def main():
    p = argparse.ArgumentParser(description="지자기 보정 — 텀블 + 타원체 피팅")
    p.add_argument("--mag-topic", default="/raw/magnetometer")
    p.add_argument("--imu-topic", default="/raw/imu_6dof")
    p.add_argument("--out", default=DEFAULT_OUT, help="보정 계수 파일 경로")
    p.add_argument("--csv", help="이 CSV로 다시 계산한다 (로봇 없이 오프라인)")
    p.add_argument("--dip", type=float, default=143.0,
                   help="그 지역의 복각[도]. 한국 143, 적도 부근 90, 극지방 0/180 근처")
    p.add_argument("--search-axes", action="store_true", help="축 재배치 24가지를 전부 시험")
    p.add_argument("--no-write", action="store_true", help="파일에 저장하지 않는다")
    args = p.parse_args()

    if args.csv:
        D = np.genfromtxt(args.csv, delimiter=",", names=True)
        M = np.column_stack([D["mx"], D["my"], D["mz"]])
        A = np.column_stack([D["ax_flu"], D["ay_flu"], D["az_flu"]])
        print(f"오프라인 재계산: {args.csv}")
    else:
        M, A = record(args.mag_topic, args.imu_topic)
        if len(M) < 200:
            print("\n  샘플이 너무 적습니다.")
            return 1
        os.makedirs(DEFAULT_LOGDIR, exist_ok=True)
        csv = os.path.join(DEFAULT_LOGDIR, "tumble_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        # 실패하든 말든 원시 데이터는 무조건 남긴다. 다시 돌리는 것보다 훨씬 싸다.
        np.savetxt(csv, np.column_stack([M, A]), delimiter=",",
                   header="mx,my,mz,ax_flu,ay_flu,az_flu", comments="")
        print(f"\n  원시 데이터 → {csv}")

    res = analyze(M, A, args)
    if res is None:
        return 1
    if args.no_write:
        print("  (--no-write 이므로 저장하지 않았습니다)")
        return 0
    write_params(res[0], res[1], args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
