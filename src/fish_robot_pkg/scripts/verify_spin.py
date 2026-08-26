#!/usr/bin/env python3
# 360도 회전 끝단 검증
#   기준(정답) : 자이로를 중력축 둘레로 적분한 각 (이미 +359.96도로 검증된 축)
#   피검사     : 지자기로 계산한 헤딩 (신규 보정 적용 / 무보정 둘 다)
#   참고       : Mahony yaw (/filtered/attitude)
# 합격 = 물리 360도 회전에 지자기 헤딩도 360도, 되돌아옴 없이 단조 증가
import math, sys, csv, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

CAL = "/home/fish/ros2_ws/config/mag_calib_params.txt"


def load_cal(path):
    with open(path) as f:
        rows = [l.split('#')[0].split() for l in f if l.split('#')[0].strip()]
    b = np.array([float(x) for x in rows[0][:3]])
    s = np.array([float(x) for x in rows[1][:3]])
    return b, s


def unwrap_step(prev, cur):
    d = cur - prev
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


BIAS_SEC = 6.0      # 시작 정지 구간: 자이로 바이어스 측정
RATE_MIN = 5.0      # 역행 판정에 쓸 최소 회전율 [도/초]


class Spin(Node):
    def __init__(self, dur):
        super().__init__('spin360')
        self.bias, self.scale = load_cal(CAL)
        self.dur = dur + BIAS_SEC
        self.gbias = np.zeros(3)
        self.gbuf = []
        self.calibrating = True
        self.mag_raw = None
        self.acc = None
        self.t0 = None
        self.tprev = None
        self.gyro_yaw = 0.0          # 중력축 둘레 자이로 적분 [도]
        self.rows = []
        self.h_cal_prev = None
        self.h_raw_prev = None
        self.h_cal = 0.0             # 누적 언랩 헤딩
        self.h_raw = 0.0
        self.mah_prev = None
        self.mah = 0.0
        self.create_subscription(Imu, '/raw/imu_6dof', self.cb_imu, 50)
        self.create_subscription(Vector3, '/raw/magnetometer', self.cb_mag, 50)
        self.create_subscription(Vector3, '/filtered/attitude', self.cb_att, 50)
        self.mahony_yaw = None
        self.last_print = 0.0

    def cb_mag(self, m):
        self.mag_raw = np.array([m.x, m.y, m.z])

    def cb_att(self, a):
        self.mahony_yaw = a.z

    def heading(self, m_flu, up):
        """up(=중력 반대, body기준) 둘레로 잰 자기장 방향각. CCW+ 로 부호 맞춤."""
        mh = m_flu - np.dot(m_flu, up) * up
        if np.linalg.norm(mh) < 1e-6:
            return None
        fwd = np.array([1.0, 0.0, 0.0])
        f = fwd - np.dot(fwd, up) * up
        if np.linalg.norm(f) < 1e-3:
            return None
        f /= np.linalg.norm(f)
        l = np.cross(up, f)                       # up × forward = 왼쪽
        # 로봇이 CCW로 돌면 자기장은 body에서 CW로 돈다 -> 부호 반전
        return -math.degrees(math.atan2(np.dot(mh, l), np.dot(mh, f)))

    def cb_imu(self, msg):
        if self.mag_raw is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0, self.tprev = t, t
            return
        dt = t - self.tprev
        self.tprev = t
        if dt <= 0 or dt > 0.1:
            return
        el = t - self.t0

        # IMU 축 변환 (검증 완료): FLU = (-z, -x, y)
        g = np.array([-msg.angular_velocity.z, -msg.angular_velocity.x,
                      msg.angular_velocity.y])                      # 도/초
        a = np.array([-msg.linear_acceleration.z, -msg.linear_acceleration.x,
                      msg.linear_acceleration.y])                   # g
        na = np.linalg.norm(a)
        if na < 0.5:
            return
        up = a / na                                                 # body 기준 월드 상방

        # ── 시작 정지 구간: 자이로 바이어스 측정 ──────────────────────────
        # 이 세션의 z 바이어스는 -2.2도/초라 빼지 않으면 30초에 66도가 샌다.
        # (state_estimation_ekf_node도 부팅 때 같은 일을 한다)
        if self.calibrating:
            self.gbuf.append(g)
            if el >= BIAS_SEC:
                B = np.array(self.gbuf)
                self.gbias = B.mean(axis=0)
                sd = B.std(axis=0)
                self.calibrating = False
                self.t0 = t                       # 여기서부터 다시 0초
                print(f"\r자이로 바이어스 = [{self.gbias[0]:+.3f} {self.gbias[1]:+.3f} "
                      f"{self.gbias[2]:+.3f}] 도/초  (표준편차 {sd.max():.3f})")
                if sd.max() > 1.0:
                    print("  ⚠ 정지 상태가 아니었습니다. 바이어스가 오염됐을 수 있습니다.")
                print("→ 지금부터 도세요.\n")
            else:
                sys.stdout.write(f"\r정지 유지... 바이어스 측정 {el:.1f}/{BIAS_SEC:.0f}초  ")
                sys.stdout.flush()
            return
        g = g - self.gbias

        # 자이로: 중력축 성분만 적분 = 정답 기준
        rate = float(np.dot(g, up))
        self.gyro_yaw += rate * dt

        # 지자기: 보정 먼저, 축 재배치 나중
        c = (self.mag_raw - self.bias) * self.scale
        m_cal = np.array([c[1], c[0], -c[2]])
        r = self.mag_raw
        m_raw = np.array([r[1], r[0], -r[2]])

        hc = self.heading(m_cal, up)
        hr = self.heading(m_raw, up)
        if hc is None or hr is None:
            return
        if self.h_cal_prev is None:
            self.h_cal_prev, self.h_raw_prev = hc, hr
        else:
            self.h_cal += unwrap_step(self.h_cal_prev, hc); self.h_cal_prev = hc
            self.h_raw += unwrap_step(self.h_raw_prev, hr); self.h_raw_prev = hr

        if self.mahony_yaw is not None:
            if self.mah_prev is None:
                self.mah_prev = self.mahony_yaw
            else:
                self.mah += unwrap_step(self.mah_prev, self.mahony_yaw)
                self.mah_prev = self.mahony_yaw

        self.rows.append([el, self.gyro_yaw, self.h_cal, self.h_raw, self.mah,
                          float(np.linalg.norm(m_cal)), na, rate])

        if el - self.last_print >= 0.25:
            self.last_print = el
            bar = int(abs(self.gyro_yaw) / 360.0 * 30)
            sys.stdout.write(
                f"\r{el:5.1f}s  자이로(정답){self.gyro_yaw:+7.1f}도 |{'#'*min(bar,30):<30}| "
                f"지자기{self.h_cal:+7.1f}도  Mahony{self.mah:+7.1f}도   ")
            sys.stdout.flush()
        if el >= self.dur:
            raise SystemExit


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    n = Spin(dur)
    print("\n=== 360도 끝단 검증 ===")
    print("로봇을 수평으로 들고, 제자리에서 천천히(20~30초/바퀴) 한 바퀴 도세요.")
    print("가능하면 2바퀴. 자이로 막대가 채워지는 걸 보고 판단하시면 됩니다.\n")
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    print()
    ts = time.strftime('%Y%m%d_%H%M%S')
    out = f"/home/fish/ros2_ws/log_csv/spin360_{ts}.csv"
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'gyro_yaw', 'mag_cal', 'mag_raw', 'mahony', 'mnorm', 'anorm', 'rate'])
        w.writerows(n.rows)
    print(f"저장: {out}  ({len(n.rows)} 샘플)")
    analyze(np.array(n.rows))
    rclpy.shutdown()


def analyze(d):
    if len(d) < 100:
        print("샘플 부족"); return
    t, gy, mc, mr, mh, rate = d[:, 0], d[:, 1], d[:, 2], d[:, 3], d[:, 4], d[:, 7]
    mov = np.abs(rate) > RATE_MIN          # 실제로 돌고 있던 구간에서만 역행을 센다
    print("\n" + "=" * 66)
    print(f"물리 회전량 (자이로 기준) : {gy[-1]:+8.2f} 도   "
          f"(회전 구간 {100*mov.mean():.0f} %)")
    print("-" * 66)
    for name, v in (("지자기 (신규 보정)", mc), ("지자기 (무보정)", mr), ("Mahony yaw", mh)):
        sweep = v[-1] - v[0]
        ratio = sweep / gy[-1] if abs(gy[-1]) > 30 else float('nan')
        err = v - v[0] - (gy - gy[0])                 # 1:1 대응 잔차
        dv, dg = np.diff(v), np.diff(gy)
        sel = mov[1:]
        bad = sel & (np.sign(dv) != np.sign(dg))      # 회전 중인데 헤딩이 역행
        back = float(np.abs(dv[bad]).sum())
        print(f"{name:20s} 회전량 {sweep:+8.2f}도  비율 {ratio:5.2f}  "
              f"최대편차 {np.abs(err).max():6.1f}도  역행 {back:7.1f}도")
    print("-" * 66)
    # 90도 구간별 선형성 — 되돌아옴은 여기서 가장 잘 보인다
    if abs(gy[-1]) > 300:
        s = 1.0 if gy[-1] > 0 else -1.0
        print("물리 90도마다 지자기 헤딩이 움직인 양 (정답 = 90도씩):")
        prev_i, prev_h = 0, mc[0]
        for k in range(1, int(abs(gy[-1]) // 90) + 1):
            idx = np.argmax(s * gy >= 90 * k)
            if idx == 0:
                break
            print(f"   {90*(k-1):4d}~{90*k:4d}도 :  {s*(mc[idx]-prev_h):+7.1f}도")
            prev_i, prev_h = idx, mc[idx]
    print("-" * 66)
    print(f"보정 후 |m| : {d[:,5].mean():.2f} ± {d[:,5].std():.2f} µT "
          f"(흩어짐 {100*d[:,5].std()/d[:,5].mean():.1f} %)")
    print("=" * 66)
    print("합격: 지자기(신규 보정) 비율 ≈ 1.00, 역행 ≈ 0, 90도 구간이 전부 90도 근처")


if __name__ == '__main__':
    main()
