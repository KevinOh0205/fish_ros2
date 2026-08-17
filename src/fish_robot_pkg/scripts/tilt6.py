#!/usr/bin/env python3
# [③] 여러 자세에서 기울기 정확도 — 0.134도를 한 자세 너머로 일반화
#
# 현재 근거는 roll -2.7 / pitch 7.3 딱 한 자세다. 자기장 오차가 기울기로
# 새는 양은 자세에 따라 다르므로 그 숫자는 일반 결과가 아니다. 게다가
# 6.8도가 유령이었던 전례가 있다.
#
# 정답은 중력. 정지 상태에서만 유효하므로 자세마다 정지 판정을 먼저 한다.
# 오일러각은 pitch가 +-90 근처에서 퇴화하므로, 정지 판정도 채점도 전부
# *중력 벡터*로 한다. 오일러각으로 판단하면 안 된다.
import sys, math, time, csv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

HOLD = 20.0        # 자세당 기록 시간
SETTLE = 3.0       # 앞부분 버리는 시간


def upvec(roll, pitch):
    r, p = math.radians(roll), math.radians(pitch)
    return np.array([-math.sin(p), math.cos(p)*math.sin(r), math.cos(p)*math.cos(r)])


def ang(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))


class Tilt(Node):
    def __init__(self):
        super().__init__('tilt6')
        self.acc = None; self.gyr = None; self.mah = None; self.ekf = None
        self.create_subscription(Imu, '/raw/imu_6dof', self.cb_imu, 50)
        self.create_subscription(Vector3, '/filtered/attitude', self.cb_mah, 50)
        self.create_subscription(Vector3, '/filtered/attitude_ekf', self.cb_ekf, 50)

    def cb_imu(self, m):
        self.acc = np.array([-m.linear_acceleration.z, -m.linear_acceleration.x,
                             m.linear_acceleration.y])
        self.gyr = np.array([-m.angular_velocity.z, -m.angular_velocity.x,
                             m.angular_velocity.y])

    def cb_mah(self, a):
        r = a.x
        if abs(r) > 2500:
            r -= math.copysign(5000.0, r)
        self.mah = (r, a.y)

    def cb_ekf(self, a): self.ekf = (a.x, a.y)

    def collect(self, sec):
        rows = []
        t0 = time.monotonic(); last = 0.0
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.acc is None or self.mah is None or self.ekf is None:
                continue
            el = time.monotonic() - t0
            na = np.linalg.norm(self.acc)
            if na < 0.3:
                continue
            up = self.acc / na
            rows.append([el, *up, na, *self.gyr, *self.mah, *self.ekf])
            if el - last >= 0.5:
                last = el
                em = ang(upvec(*self.mah), up); ee = ang(upvec(*self.ekf), up)
                sys.stdout.write(f"\r  {el:4.1f}/{sec:.0f}s  |a|{na:5.3f}g  "
                                 f"자이로{np.abs(self.gyr).max():5.2f}도/초  "
                                 f"Mahony {em:5.3f}도  EKF {ee:5.3f}도   ")
                sys.stdout.flush()
        return np.array(rows)


POSES = [
    "① 평평하게 (기본 자세)",
    "② 코를 위로 (약 45도)",
    "③ 코를 아래로 (약 45도)",
    "④ 왼쪽으로 굴려서 (약 45도)",
    "⑤ 오른쪽으로 굴려서 (약 45도)",
    "⑥ 옆으로 눕혀서 (약 90도)",
]


def main():
    rclpy.init()
    n = Tilt()
    print(f"\n=== [③] 여러 자세 기울기 정확도 · 자세당 {HOLD:.0f}초 ===")
    print("각 자세로 놓고 Enter. 손을 떼고 완전히 정지시킨 뒤 누르세요.")
    print("정확한 각도는 중요하지 않습니다 — 서로 다르기만 하면 됩니다.\n")
    out = []
    for p in POSES:
        input(f"{p}  -> 놓고 Enter: ")
        A = n.collect(HOLD)
        print()
        if len(A) < 100:
            print("    샘플 부족 — 건너뜁니다\n"); continue
        A = A[A[:, 0] > SETTLE]
        UP = A[:, 1:4]
        # 정지 판정: 중력 방향의 흔들림과 자이로. 오일러각으로 하면 안 된다.
        spread = max(ang(UP[i], UP.mean(0)/np.linalg.norm(UP.mean(0)))
                     for i in range(0, len(UP), max(1, len(UP)//200)))
        gmax = np.abs(A[:, 5:8]).max()
        em = np.array([ang(upvec(A[i, 8], A[i, 9]), UP[i]) for i in range(len(A))])
        ee = np.array([ang(upvec(A[i, 10], A[i, 11]), UP[i]) for i in range(len(A))])
        ok = spread < 0.5 and gmax < 2.0
        print(f"    중력 {'정지 OK' if ok else '★ 흔들림 감지'}  "
              f"(방향 흔들림 {spread:.2f}도, 자이로 최대 {gmax:.2f}도/초)")
        print(f"    자세  roll {A[:,8].mean():+7.2f}  pitch {A[:,9].mean():+7.2f}")
        print(f"    Mahony {em.mean():6.3f} +-{em.std():5.3f} 도    "
              f"EKF {ee.mean():6.3f} +-{ee.std():5.3f} 도\n")
        out.append((p, A[:, 8].mean(), A[:, 9].mean(), em.mean(), em.std(),
                    ee.mean(), ee.std(), ok))

    print("=" * 74)
    print(f"{'자세':<26}{'roll':>8}{'pitch':>8}{'Mahony':>11}{'EKF':>11}{'정지':>6}")
    print("-" * 74)
    for p, r, pi, m1, m2, e1, e2, ok in out:
        print(f"{p:<26}{r:+8.2f}{pi:+8.2f}{m1:8.3f}±{m2:.2f}{e1:8.3f}±{e2:.2f}"
              f"{'  OK' if ok else '  ★':>6}")
    if out:
        M = np.array([o[3] for o in out]); E = np.array([o[5] for o in out])
        print("-" * 74)
        print(f"{'전체':<26}{'':>16}{M.mean():8.3f}     {E.mean():8.3f}")
        print(f"{'자세간 최대':<26}{'':>16}{M.max():8.3f}     {E.max():8.3f}")
    print("=" * 74)
    print("기존 근거는 roll -2.7 / pitch 7.3 한 자세에서 Mahony 0.134 / EKF 0.117 였습니다.")
    with open(f"/home/fish/ros2_ws/log_csv/tilt6_{time.strftime('%Y%m%d_%H%M%S')}.csv",
              'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pose', 'roll', 'pitch', 'mah_mean', 'mah_std', 'ekf_mean', 'ekf_std', 'still'])
        w.writerows(out)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
