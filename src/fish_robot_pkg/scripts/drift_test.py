#!/usr/bin/env python3
# [②] EKF 6축 yaw 드리프트 — 계획서의 유일한 go/no-go 기준 (<= 2도/분)
#
# 반드시 use_mag:=false 로 띄운 EKF에 대고 돌릴 것. 9축이면 지자기가 yaw를
# 잡아주므로 이 시험은 아무 의미가 없다.
#
# 판정 두 가지:
#   (1) yaw 드리프트 [도/분]  — 선형 회귀 기울기. 기준 <= 2.0
#   (2) yaw 1시그마가 *커지기만* 하는가 — 6축에서는 yaw를 관측하는 것이
#       하나도 없으므로 불확실도는 단조 증가해야 한다. 줄어들면 가속도
#       업데이트가 yaw로 새고 있다는 증거다 (2026-08-06에 실제로 있었던 버그).
import sys, math, time, csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
GATE = 2.0


class Drift(Node):
    def __init__(self):
        super().__init__('drift_test')
        self.t0 = None
        self.rows = []
        self.st = None
        self.g = None
        self.last = 0.0
        self.yaw_prev = None
        self.yaw_acc = 0.0
        self.create_subscription(Vector3, '/filtered/attitude_ekf', self.cb_att, 50)
        self.create_subscription(Float32MultiArray, '/filtered/ekf_status', self.cb_st, 20)
        self.create_subscription(Imu, '/raw/imu_6dof', self.cb_imu, 50)

    def cb_st(self, m): self.st = list(m.data)

    def cb_imu(self, m):
        self.g = (-m.angular_velocity.z, -m.angular_velocity.x, m.angular_velocity.y)

    def cb_att(self, a):
        if self.st is None or self.g is None:
            return
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
            self.yaw_prev = a.z
        el = t - self.t0

        d = a.z - self.yaw_prev                 # -180/+180 경계에서 가짜 점프 방지
        d -= 360.0 * round(d / 360.0)
        self.yaw_acc += d
        self.yaw_prev = a.z

        self.rows.append([el, self.yaw_acc, a.x, a.y] + list(self.st[:12]) + list(self.g))

        if el - self.last >= 5.0:
            self.last = el
            rate = self.yaw_acc / (el / 60.0) if el > 30 else float('nan')
            sys.stdout.write(
                f"\r{el:6.0f}/{DUR:.0f}s  yaw {self.yaw_acc:+8.3f}도  "
                f"({rate:+6.2f} 도/분)  yaw1σ {self.st[5]:6.2f}도  "
                f"bias [{self.st[0]:+6.3f} {self.st[1]:+6.3f} {self.st[2]:+6.3f}]   ")
            sys.stdout.flush()
        if el >= DUR:
            raise SystemExit


def main():
    rclpy.init()
    n = Drift()
    print(f"\n=== [②] EKF 6축 yaw 드리프트 · {DUR/60:.0f}분 ===")
    print("로봇을 완전히 정지시키고 손대지 마세요. 버튼도 누르지 마세요.\n")
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    print("\n")
    if len(n.rows) < 100:
        print("샘플이 너무 적습니다. EKF가 돌고 있는지 확인하세요.")
        rclpy.shutdown(); return

    # 컬럼: [0]t [1]yaw [2]roll [3]pitch [4..15]status0..11 [16..18]원시자이로
    #       status[3..5] = 자세 1시그마 -> yaw 1시그마는 4+5 = 9번
    A = np.array(n.rows)
    t, yaw, s5 = A[:, 0], A[:, 1], A[:, 9]

    # 앞 60초는 바이어스 수렴 구간이라 버린다
    m = t > 60.0
    if m.sum() < 100:
        m = t > t.max() * 0.2
    slope = np.polyfit(t[m], yaw[m], 1)[0] * 60.0

    print("=" * 66)
    print(f"  구간            {t[m].min():.0f} ~ {t[m].max():.0f}초 "
          f"(앞 60초 = 바이어스 수렴 구간 제외)")
    print(f"  yaw 드리프트    {slope:+.3f} 도/분      기준 <= {GATE} 도/분   "
          f"{'합격' if abs(slope) <= GATE else '불합격'}")
    print(f"  총 이동         {yaw[m][-1]-yaw[m][0]:+.2f} 도 / {(t[m][-1]-t[m][0])/60:.1f} 분")

    # yaw 1시그마 — 6축에서는 yaw를 관측하는 게 없으므로 커지기만 해야 한다.
    # 단, 한 샘플짜리 감소로 판정하면 안 된다. Joseph 형태와 대칭화의 부동소수점
    # 오차가 0.5 % 수준의 감소를 늘 만들어낸다. 진짜 누설(2026-08-06)은 40도->7도로
    # *지속* 감소했으므로 누적 감소량으로 판정한다.
    # status는 10 Hz 발행이라 값이 반복된다. 실제 갱신 지점만 뽑는다.
    sv = s5[m]
    v = sv[np.r_[0, np.where(np.diff(sv) != 0)[0] + 1]]
    dv = np.diff(v)
    up, dn = dv[dv > 0].sum(), -dv[dv < 0].sum()
    ratio = dn / up * 100 if up > 0 else float('inf')
    print(f"  yaw 1σ          {sv[0]:.2f} -> {sv[-1]:.2f} 도   "
          f"증가 +{up:.2f} / 감소 −{dn:.2f} 도  (감소분 {ratio:.2f} %)")
    print(f"                  {'정상 — 감소분은 반올림 오차 수준' if ratio < 5 and dv.sum() > 0 else '★ yaw 누설 의심'}")
    print(f"  바이어스 추정    [{A[m][-1,4]:+.4f} {A[m][-1,5]:+.4f} {A[m][-1,6]:+.4f}] 도/초")
    print(f"  원시 자이로 평균 [{A[m][:,16].mean():+.4f} {A[m][:,17].mean():+.4f} "
          f"{A[m][:,18].mean():+.4f}] 도/초")
    print(f"  roll/pitch 이동  {A[m][-1,2]-A[m][0,2]:+.3f} / {A[m][-1,3]-A[m][0,3]:+.3f} 도")
    print("=" * 66)

    out = f"/home/fish/ros2_ws/log_csv/drift_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'yaw', 'roll', 'pitch'] +
                   [f'st{i}' for i in range(12)] + ['gx', 'gy', 'gz'])
        w.writerows(n.rows)
    print(f"저장: {out}  ({len(n.rows)} 샘플)")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
