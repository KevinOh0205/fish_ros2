#!/usr/bin/env python3
# [④] 지자기 상실 -> 6축 폴백 전환 튐
#
# i2c_driver_node를 죽이지 않고 SIGSTOP으로 얼린다. 죽이면 launch가
# respawn해서 회복 시점을 제어할 수 없고, 재시작 중 다른 센서(압력)까지 끊긴다.
# STOP/CONT는 되돌릴 수 있고 커널이 프로세스를 그대로 보존한다.
#
# 보는 것 세 가지:
#   전환 지연  — 마지막 지자기 이후 몇 ms 만에 폴백하는가 (설정 500ms)
#   전환 튐    — 그 순간 자세가 얼마나 뛰는가. 제어 루프에 그대로 들어간다
#   복구 튐    — 지자기가 돌아올 때 다시 뛰는가. 이쪽이 더 클 수 있다
import os, sys, math, time, csv, signal
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

FREEZE_AT = 8.0        # 시작 후 몇 초에 얼릴까
FREEZE_SEC = 8.0       # 얼려둘 시간 (타임아웃 500ms보다 충분히 길게)
DUR = 30.0


def find_i2c():
    """comm은 15자에서 잘리므로 pgrep -x로는 못 찾는다. /proc/*/exe를 훑는다."""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            exe = os.readlink(f'/proc/{pid}/exe')
        except OSError:
            continue
        if exe.endswith('/i2c_driver_node'):
            return int(pid)
    return None


class MagLoss(Node):
    def __init__(self, pid):
        super().__init__('magloss')
        self.pid = pid
        self.t0 = time.monotonic()
        self.rows = []
        self.mah = self.ekf = None
        self.last_mag = None
        self.frozen = False
        self.thawed = False
        self.last = 0.0
        self.create_subscription(Vector3, '/raw/magnetometer', self.cb_mag, 50)
        self.create_subscription(Vector3, '/filtered/attitude', self.cb_mah, 50)
        self.create_subscription(Vector3, '/filtered/attitude_ekf', self.cb_ekf, 50)
        self.create_timer(0.01, self.tick)

    def cb_mag(self, m): self.last_mag = time.monotonic()

    def cb_mah(self, a):
        r = a.x
        if abs(r) > 2500:
            r -= math.copysign(5000.0, r)
        self.mah = (r, a.y, a.z)

    def cb_ekf(self, a): self.ekf = (a.x, a.y, a.z)

    def tick(self):
        if self.mah is None or self.ekf is None or self.last_mag is None:
            return
        el = time.monotonic() - self.t0

        if not self.frozen and el >= FREEZE_AT:
            os.kill(self.pid, signal.SIGSTOP); self.frozen = True
            print(f"\n>>> {el:.1f}s  i2c_driver_node(PID {self.pid}) SIGSTOP — 지자기 정지")
        if self.frozen and not self.thawed and el >= FREEZE_AT + FREEZE_SEC:
            os.kill(self.pid, signal.SIGCONT); self.thawed = True
            print(f"\n>>> {el:.1f}s  SIGCONT — 지자기 복구")

        age = (time.monotonic() - self.last_mag) * 1000.0
        self.rows.append([el, age, *self.mah, *self.ekf])

        if el - self.last >= 0.5:
            self.last = el
            sys.stdout.write(f"\r{el:5.1f}s  지자기 나이 {age:7.1f}ms   "
                             f"Mahony({self.mah[0]:+6.2f},{self.mah[1]:+6.2f},{self.mah[2]:+7.2f})  "
                             f"EKF({self.ekf[0]:+6.2f},{self.ekf[1]:+6.2f},{self.ekf[2]:+7.2f})   ")
            sys.stdout.flush()
        if el >= DUR:
            raise SystemExit


def report(A, name, c0):
    """c0 = 해당 필터의 roll 컬럼 인덱스."""
    t, age = A[:, 0], A[:, 1]
    R, P, Y = A[:, c0], A[:, c0+1], A[:, c0+2]
    # 폴백 시점 = 지자기 나이가 500ms를 처음 넘는 순간
    i = np.argmax(age > 500.0) if (age > 500.0).any() else None
    print(f"\n  [{name}]")
    if i is None or i == 0:
        print("    폴백이 감지되지 않았습니다 (나이가 500ms를 안 넘음)")
        return
    print(f"    폴백 시각 {t[i]:.2f}s, 그때 지자기 나이 {age[i]:.0f} ms")
    for nm, lo, hi in [('폴백 전후 1초', t[i]-1.0, t[i]+1.0),
                       ('복구 전후 1초', FREEZE_AT+FREEZE_SEC-1.0,
                        FREEZE_AT+FREEZE_SEC+1.0),
                       ('전 구간 기준선', 0.0, FREEZE_AT-1.0)]:
        m = (t >= lo) & (t <= hi)
        if m.sum() < 5:
            continue
        print(f"    {nm}:  roll {R[m].max()-R[m].min():5.2f}도  "
              f"pitch {P[m].max()-P[m].min():5.2f}도  yaw {Y[m].max()-Y[m].min():6.2f}도")
    print(f"    NaN {int(np.isnan(A[:, c0:c0+3]).sum())}개")


def main():
    pid = find_i2c()
    if pid is None:
        print("i2c_driver_node를 못 찾았습니다. 돌고 있나요?")
        return
    print(f"\n=== [④] 지자기 상실 -> 6축 폴백 · {DUR:.0f}초 ===")
    print(f"i2c_driver_node PID {pid}")
    print(f"{FREEZE_AT:.0f}초에 얼리고 {FREEZE_SEC:.0f}초 뒤 되살립니다.")
    print("로봇을 가만히 두세요 — 자세가 변하면 전환 튐과 구분이 안 됩니다.\n")
    rclpy.init()
    n = MagLoss(pid)
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        try:
            os.kill(pid, signal.SIGCONT)      # 어떤 경우에도 반드시 되살린다
        except OSError:
            pass
    print("\n" + "=" * 66)
    A = np.array(n.rows)
    report(A, 'Mahony', 2)
    report(A, 'EKF', 5)
    print("=" * 66)
    out = f"/home/fish/ros2_ws/log_csv/magloss_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'mag_age_ms', 'mah_r', 'mah_p', 'mah_y', 'ekf_r', 'ekf_p', 'ekf_y'])
        w.writerows(n.rows)
    print(f"저장: {out}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
