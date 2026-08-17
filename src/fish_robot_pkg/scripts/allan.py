#!/usr/bin/env python3
# [⑤] Allan 분산 — gyro_noise_sigma 의 근거 만들기
#
#   기록 : python3 ~/allan.py 7200          (초, 2시간 이상 권장)
#   분석 : python3 ~/allan.py --an <csv>
#
# 현재 EKF는 gyro_noise_sigma = 0.05 도/초/√Hz 를 쓰는데 정지 실측은
# 0.0028~0.0069 다. 10배 여유를 "진동과 도착시각 지터 때문"이라고 적어뒀지만
# 한 번도 검증한 적이 없어서 근거가 아니라 짐작이다.
#
# Allan 편차 곡선이 주는 것:
#   기울기 -1/2 구간  -> 각도 랜덤워크 = 진짜 sigma_g  (tau=1s 지점 값)
#   평탄한 최소점     -> 바이어스 불안정성
#   최소점의 tau      -> bias_tau 의 근거
import sys, time, csv
import numpy as np


def record(dur):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class Rec(Node):
        def __init__(self):
            super().__init__('allan_rec')
            self.rows = []; self.t0 = None; self.last = 0.0
            self.create_subscription(Imu, '/raw/imu_6dof', self.cb, 200)

        def cb(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.t0 is None:
                self.t0 = t
            el = t - self.t0
            self.rows.append([el, -m.angular_velocity.z, -m.angular_velocity.x,
                              m.angular_velocity.y])
            if el - self.last >= 30.0:
                self.last = el
                a = np.array(self.rows[-3000:])[:, 1:]
                sys.stdout.write(f"\r{el/60:6.1f}/{dur/60:.0f}분  {len(self.rows)} 샘플  "
                                 f"최근 평균 [{a[:,0].mean():+7.4f} {a[:,1].mean():+7.4f} "
                                 f"{a[:,2].mean():+7.4f}] 도/초   ")
                sys.stdout.flush()
            if el >= dur:
                raise SystemExit

    rclpy.init(); n = Rec()
    print(f"\n=== [⑤] Allan 분산 기록 · {dur/60:.0f}분 ===")
    print("로봇을 완전히 정지시키고 그대로 두세요. 온도가 안정된 뒤가 좋습니다.")
    print("중간에 건드리면 그 구간이 바이어스 불안정성으로 잘못 읽힙니다.\n")
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    out = f"/home/fish/ros2_ws/log_csv/allan_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'gx', 'gy', 'gz']); w.writerows(n.rows)
    print(f"\n저장: {out}  ({len(n.rows)} 샘플)")
    print(f"분석: python3 ~/allan.py --an {out}")
    rclpy.shutdown()


def adev(x, fs):
    """겹치는(overlapping) Allan 편차. 겹치지 않는 버전보다 자유도가 훨씬 크다."""
    N = len(x)
    theta = np.cumsum(x) / fs                       # 각속도 -> 각도 적분
    taus, out = [], []
    m = 1
    while m <= N // 5:
        tau = m / fs
        d = theta[2*m:] - 2*theta[m:-m] + theta[:-2*m]
        if len(d) < 4:
            break
        taus.append(tau)
        out.append(np.sqrt((d**2).mean() / (2 * tau**2)))
        m = max(m + 1, int(m * 1.3))
    return np.array(taus), np.array(out)


def analyze(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    t = d['t']
    fs = len(t) / (t[-1] - t[0])
    print(f"\n{len(t)} 샘플, {(t[-1]-t[0])/3600:.2f} 시간, {fs:.1f} Hz\n")
    print(f"{'축':>4}{'ARW(τ=1s)':>12}{'바이어스불안정':>14}{'그 τ':>9}"
          f"{'설정 0.05 대비':>14}")
    print("-" * 55)
    for nm, k in [('x', 'gx'), ('y', 'gy'), ('z', 'gz')]:
        x = d[k] - d[k].mean()
        tau, s = adev(x, fs)
        arw = np.interp(1.0, tau, s)                # τ=1s 의 편차 = 도/초/√Hz
        i = np.argmin(s)
        print(f"{nm:>4}{arw:12.4f}{s[i]:14.4f}{tau[i]:9.1f}s{0.05/arw:12.1f}배")
    print("-" * 55)
    print("ARW = 각도 랜덤워크 [도/초/√Hz]. 이게 gyro_noise_sigma 의 물리적 근거다.")
    print("바이어스 불안정성 [도/초] 과 그 τ [초] 는 bias_tau 를 정하는 값이다.")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--an':
        analyze(sys.argv[2])
    else:
        record(float(sys.argv[1]) if len(sys.argv) > 1 else 7200.0)
