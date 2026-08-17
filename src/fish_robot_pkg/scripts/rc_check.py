#!/usr/bin/env python3
# 조종기 -> 서보 경로 어디가 끊겼는지 가른다.
#
# 경로:  조종기 -> 수신기 -> nRF52840 -> UART -> Pi(/rc/command)
#           -> pid_control_node -> /motor/output -> UART -> nRF52840 -> 서보
#
# Pi가 볼 수 있는 건 가운데 두 개뿐이다. 그래서 이 둘만 보고 범인을 좁힌다:
#   /rc/command 가 안 변하면   -> 조종기/수신기/nRF 입력 쪽
#   /rc/command 는 변하는데
#   /motor/output 이 안 변하면 -> pid_control_node (게인/모드/페일세이프)
#   둘 다 변하는데 서보가 안 움직이면 -> nRF 출력단 또는 서보 전원/배선
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import UInt16MultiArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0


class Check(Node):
    def __init__(self):
        super().__init__('rc_check')
        self.rc = None; self.mo = None; self.att = None
        self.rcs = []; self.mos = []
        self.t0 = time.monotonic(); self.last = 0.0
        self.create_subscription(Quaternion, '/rc/command', self.cb_rc, 20)
        self.create_subscription(UInt16MultiArray, '/motor/output', self.cb_mo, 20)
        self.create_subscription(Vector3, '/filtered/attitude', self.cb_att, 20)
        self.create_timer(0.02, self.tick)

    def cb_rc(self, m): self.rc = (m.x, m.y, m.z, m.w)
    def cb_mo(self, m): self.mo = tuple(m.data)
    def cb_att(self, a): self.att = (a.x, a.y, a.z)

    def tick(self):
        if self.rc is None or self.mo is None:
            return
        el = time.monotonic() - self.t0
        self.rcs.append(self.rc); self.mos.append(self.mo)
        if el - self.last >= 0.2:
            self.last = el
            r, m = self.rc, self.mo
            sys.stdout.write(
                f"\r{el:5.1f}s  조종기 R{r[0]:+7.0f} P{r[1]:+7.0f} Y{r[2]:+7.0f} T{r[3]:+7.0f}"
                f"   서보 [{m[0]:4d} {m[1]:4d} {m[2]:4d}] 꼬리 {m[3]:4d}   ")
            sys.stdout.flush()
        if el >= DUR:
            raise SystemExit


def main():
    rclpy.init(); n = Check()
    print(f"\n=== 조종기 -> 서보 경로 진단 · {DUR:.0f}초 ===")
    print("지금부터 스틱 네 개를 하나씩 끝까지 밀었다 당겼다 하세요.")
    print("스로틀도 꼭 올려보세요.\n")
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    print("\n")
    if len(n.rcs) < 20:
        print("데이터 부족 — /rc/command 또는 /motor/output 이 안 옵니다.")
        rclpy.shutdown(); return

    R = np.array(n.rcs); M = np.array(n.mos, dtype=float)
    nm_r = ['롤', '피치', '요', '스로틀']
    nm_m = ['좌 서보', '우 서보', '요 서보', '꼬리 BLDC']
    print("=" * 62)
    print(f"{'':>10}{'최소':>9}{'최대':>9}{'움직인 폭':>11}")
    print("-" * 62)
    for i in range(4):
        print(f"조종기 {nm_r[i]:<5}{R[:,i].min():9.0f}{R[:,i].max():9.0f}"
              f"{R[:,i].max()-R[:,i].min():11.0f}")
    print("-" * 62)
    for i in range(4):
        print(f"{nm_m[i]:>10}{M[:,i].min():9.0f}{M[:,i].max():9.0f}"
              f"{M[:,i].max()-M[:,i].min():11.0f}")
    print("=" * 62)

    rc_moved = (R[:, :3].max(0) - R[:, :3].min(0)).max()
    thr_moved = R[:, 3].max() - R[:, 3].min()
    servo_moved = (M[:, :3].max(0) - M[:, :3].min(0)).max()
    tail_moved = M[:, 3].max() - M[:, 3].min()

    print("\n판정")
    if rc_moved < 10:
        print("  ★ /rc/command 가 거의 안 움직였습니다 (변동 %.0f)." % rc_moved)
        print("    -> Pi 앞쪽 문제입니다. 조종기 바인딩 / 수신기-nRF 배선 / 채널 매핑.")
        print("    -> 스틱을 실제로 끝까지 밀었는지도 확인하세요.")
    elif servo_moved < 5:
        print("  ★ 조종기는 움직이는데(%.0f) 서보 출력이 안 변합니다(%.0f)." % (rc_moved, servo_moved))
        print("    -> pid_control_node 문제입니다. 페일세이프에 걸렸거나 모드가 엉켰습니다.")
        print("       journalctl -u fish-robot -f | grep PID   로 경고를 보세요.")
    else:
        print("  ✔ Pi 구간은 정상입니다. 조종기 %.0f -> 서보 출력 %.0f 움직임." % (rc_moved, servo_moved))
        print("    -> 그런데 실제 서보가 안 움직이면 Pi 뒤쪽입니다:")
        print("       (1) nRF52840 펌웨어가 /motor/output 을 서보에 반영하지 않음")
        print("           (아밍이 필요하거나, 자체 조종기 패스스루 모드일 수 있음)")
        print("       (2) 서보 전원(BEC)이 안 들어옴 — 신호만으로는 서보가 안 돕니다")
        print("       (3) 서보 신호선 배선")
    if thr_moved < 10:
        print(f"\n  참고: 스로틀이 안 움직였습니다({thr_moved:.0f}). 꼬리 모터는 원래 안 돕니다.")
    elif tail_moved < 5:
        print(f"\n  참고: 스로틀은 움직였는데({thr_moved:.0f}) 꼬리 출력이 안 변했습니다.")
    print()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
