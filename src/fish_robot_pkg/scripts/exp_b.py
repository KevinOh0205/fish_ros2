#!/usr/bin/env python3
# 실험 B — 9축 Mahony vs 9축 EKF, 한 번의 기록으로 두 질문에 답한다
#   ① 360도 헤딩 추종  : 회전 구간에서 두 필터 yaw vs 자이로
#   ② 흔들림 내성      : 흔드는 구간에서 두 필터 기울기 vs 자이로
#
# 정답 기준이 구간마다 다르다:
#   정지  -> 중력 (가속도) 이 정답
#   흔듦  -> 가속도는 비력(중력-선형가속도)이라 정답이 될 수 없다.
#           자이로를 쿼터니언으로 적분한 것이 정답. 짧은 구간에서만 유효하므로
#           분석은 절대값이 아니라 "구간 시작 대비 변화량"으로 채점한다.
import math, sys, csv, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

CAL = "/home/fish/ros2_ws/config/mag_calib_params.txt"
BIAS_SEC = 6.0


def load_cal(p):
    with open(p) as f:
        r = [l.split('#')[0].split() for l in f if l.split('#')[0].strip()]
    return (np.array([float(x) for x in r[0][:3]]),
            np.array([float(x) for x in r[1][:3]]))


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def qexp(v):
    """회전벡터 -> 쿼터니언 (정확한 지수사상)"""
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    a = v / th * math.sin(th/2)
    return np.array([math.cos(th/2), a[0], a[1], a[2]])


def q_up(q):
    """q(body->world)에서 body 기준 월드 상방 = R^T e3 = 회전행렬 3행"""
    w, x, y, z = q
    return np.array([2*(x*z - w*y), 2*(w*x + y*z), w*w - x*x - y*y + z*z])


def q_yaw(q):
    """ZYX yaw [도]. 반시계 = 양수 (좌선회), 노드 규약과 같은 부호."""
    w, x, y, z = q
    return math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def upvec(roll, pitch):
    r, p = math.radians(roll), math.radians(pitch)
    return np.array([-math.sin(p), math.cos(p)*math.sin(r), math.cos(p)*math.cos(r)])


def q_from_up(up):
    """중력만으로 자세 하나 만들기 (yaw는 임의로 0)"""
    roll = math.atan2(up[1], up[2])
    pitch = -math.asin(max(-1.0, min(1.0, up[0])))
    # ZYX(yaw=0): q = qy(pitch) * qx(roll).  이러면 R의 3행 = (-sin p, cos p sin r,
    # cos p cos r) 이 되어 upvec() 과 정확히 같다. pitch를 여기서 또 뒤집으면 안 된다.
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    return qmul(np.array([cp, 0, sp, 0]), np.array([cr, sr, 0, 0]))


class ExpB(Node):
    def __init__(self, dur):
        super().__init__('exp_b')
        self.bias, self.scale = load_cal(CAL)
        self.dur = dur + BIAS_SEC
        self.gbias = np.zeros(3); self.gbuf = []; self.abuf = []
        self.calibrating = True
        self.t0 = self.tprev = None
        self.q = None                      # 자이로만으로 적분한 정답 자세
        self.yaw_t = 0.0; self.yaw_prev = None   # 정답 yaw, 언랩 누적
        self.aref = 1.0
        self.mag = None; self.mah = None; self.ekf = None
        self.rows = []; self.last_print = 0.0
        self.create_subscription(Imu, '/raw/imu_6dof', self.cb_imu, 50)
        self.create_subscription(Vector3, '/raw/magnetometer', self.cb_mag, 50)
        self.create_subscription(Vector3, '/filtered/attitude', self.cb_mah, 50)
        self.create_subscription(Vector3, '/filtered/attitude_ekf', self.cb_ekf, 50)

    def cb_mag(self, m): self.mag = np.array([m.x, m.y, m.z])

    def cb_mah(self, a):
        r = a.x
        if abs(r) > 2500:
            r -= math.copysign(5000.0, r)
        self.mah = (r, a.y, a.z)

    def cb_ekf(self, a): self.ekf = (a.x, a.y, a.z)

    def cb_imu(self, msg):
        if self.mag is None or self.mah is None or self.ekf is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = self.tprev = t; return
        dt = t - self.tprev; self.tprev = t
        if dt <= 0 or dt > 0.1: return
        el = t - self.t0

        g = np.array([-msg.angular_velocity.z, -msg.angular_velocity.x,
                      msg.angular_velocity.y])
        a = np.array([-msg.linear_acceleration.z, -msg.linear_acceleration.x,
                      msg.linear_acceleration.y])
        na = np.linalg.norm(a)
        if na < 0.3: return

        if self.calibrating:
            self.gbuf.append(g); self.abuf.append(a)
            if el >= BIAS_SEC:
                B = np.array(self.gbuf); A = np.array(self.abuf)
                self.gbias = B.mean(axis=0)
                self.aref = float(np.linalg.norm(A, axis=1).mean())
                m = A.mean(axis=0); self.q = q_from_up(m/np.linalg.norm(m))
                self.calibrating = False; self.t0 = t
                print(f"\r자이로 바이어스 [{self.gbias[0]:+.3f} {self.gbias[1]:+.3f} "
                      f"{self.gbias[2]:+.3f}] 도/초 (표준편차 {B.std(axis=0).max():.3f}), "
                      f"|a|기준 {self.aref:.4f} g")
                print("→ ① 한두 바퀴 돌리고  ② 잠깐 멈췄다가  ③ 세게 흔드세요\n")
            else:
                sys.stdout.write(f"\r정지 유지... {el:.1f}/{BIAS_SEC:.0f}초  ")
                sys.stdout.flush()
            return

        gc = np.radians(g - self.gbias)
        self.q = qmul(self.q, qexp(gc * dt))          # 자이로만 적분 = 정답
        self.q /= np.linalg.norm(self.q)
        up_t = q_up(self.q)

        # 정답 yaw: 자이로 적분 쿼터니언에서 뽑아 언랩 누적. 필터 yaw도 같은 방식으로
        # 언랩해야 -180/+180 경계에서 가짜 점프가 안 생긴다.
        yr = q_yaw(self.q)
        if self.yaw_prev is None:
            self.yaw_prev = yr
        d = yr - self.yaw_prev
        d -= 360.0 * round(d / 360.0)
        self.yaw_t += d; self.yaw_prev = yr

        up_a = a / na
        rate = float(np.dot(g - self.gbias, up_a))

        c = (self.mag - self.bias) * self.scale
        m_flu = np.array([c[1], c[0], -c[2]])

        e_mah = ang(upvec(self.mah[0], self.mah[1]), up_t)
        e_ekf = ang(upvec(self.ekf[0], self.ekf[1]), up_t)

        self.rows.append([el, rate, na, na/self.aref - 1.0, self.yaw_t,
                          up_t[0], up_t[1], up_t[2],
                          up_a[0], up_a[1], up_a[2],
                          self.mah[0], self.mah[1], self.mah[2],
                          self.ekf[0], self.ekf[1], self.ekf[2],
                          e_mah, e_ekf, float(np.linalg.norm(m_flu))])

        if el - self.last_print >= 0.25:
            self.last_print = el
            dev = abs(na/self.aref - 1.0)
            ph = "흔듦" if dev > 0.05 else ("회전" if abs(rate) > 5 else "정지")
            sys.stdout.write(
                f"\r{el:5.1f}s [{ph}] 회전{rate:+6.1f}도/초 |a|{na:5.3f}g   "
                f"기울기오차  Mahony{e_mah:5.1f}도   EKF{e_ekf:5.1f}도    ")
            sys.stdout.flush()
        if el >= self.dur:
            raise SystemExit


def ang(u1, u2):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u1, u2))))))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    rclpy.init(); n = ExpB(dur)
    print("\n=== 실험 B : 9축 Mahony vs 9축 EKF ===")
    print("① 처음 6초 정지 (바이어스 측정 — 반드시 가만히)")
    print("② 수평으로 들고 제자리에서 한두 바퀴")
    print("③ 3~5초 정지")
    print("④ 회전 없이 좌우/상하로 세게 흔들기, 몇 차례 반복")
    print("⑤ 마지막에 다시 정지\n")
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    print()
    out = f"/home/fish/ros2_ws/log_csv/expb_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t','rate','anorm','adev','yaw_t','utx','uty','utz','uax','uay','uaz',
                    'mah_r','mah_p','mah_y','ekf_r','ekf_p','ekf_y',
                    'e_mah','e_ekf','mnorm'])
        w.writerows(n.rows)
    print(f"저장: {out}  ({len(n.rows)} 샘플)")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
