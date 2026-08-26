#!/usr/bin/env python3
# 벤치 시험용 — 스틱을 서보에 직결한다 (PID 없음).
#
# 왜 필요한가: 평소에는 스틱이 "목표 자세"라서, 책상 위처럼 로봇이 실제로
# 기울 수 없는 곳에서는 PID가 계속 포화되어 서보 배선/방향/가동범위를
# 확인할 수가 없다. 이 스크립트는 자세를 아예 안 보고 스틱 위치를 그대로
# 서보 위치로 보낸다.
#
# 반드시 pid_control_node 를 먼저 내릴 것. 안 그러면 /motor/output 에
# 발행자가 둘이 되어 100Hz 씩 번갈아 나가고 서보가 떨린다.
#
#   sudo systemctl stop fish-robot.service
#   ros2 run fish_robot_pkg uart_bridge_node &
#   python3 ~/servo_direct.py
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16MultiArray

SERVO_MIN, SERVO_MAX, SERVO_MID = 1325, 1675, 1500   # 혼 약 ±20° (HS-5086WP 0.114°/us) — pid_control_node 와 동일

# 스틱 원시 최대값. 예전 동작하던 Motor_Control.c 의 나눗셈 상수에서 역산했다
# (roll/pitch 를 30/255, yaw 를 30/85 로 나눴다). 실측과 다르면 인자로 바꿀 것.
RC_MAX_RP = float(sys.argv[1]) if len(sys.argv) > 1 else 255.0
RC_MAX_Y = float(sys.argv[2]) if len(sys.argv) > 2 else 85.0


def clamp(v):
    return int(max(SERVO_MIN, min(SERVO_MAX, v)))


class Direct(Node):
    def __init__(self):
        super().__init__('servo_direct')
        self.rc = (0.0, 0.0, 0.0, 0.0)
        self.seen = [0.0, 0.0, 0.0, 0.0]      # 관측된 스틱 최대 절대값
        self.lost = True
        self.pub = self.create_publisher(UInt16MultiArray, '/motor/output', 10)
        self.create_subscription(Quaternion, '/rc/command', self.cb, 20)
        self.create_timer(0.01, self.tick)    # 100 Hz, 원래 제어 주기와 동일
        self.last = 0.0
        self.t0 = time.monotonic()

    def cb(self, m):
        # -9999 는 nRF 가 보내는 링크 두절 센티넬. 그때는 갱신하지 않는다.
        if int(m.w) == -9999:
            self.lost = True
            return
        self.lost = False
        self.rc = (m.x, m.y, m.z, m.w)
        for i, v in enumerate(self.rc):
            self.seen[i] = max(self.seen[i], abs(v))

    def tick(self):
        msg = UInt16MultiArray()
        if self.lost:
            msg.data = [SERVO_MID, SERVO_MID, SERVO_MID, 1000]
        else:
            r, p, y, t = self.rc
            # 스틱을 끝까지 밀면 서보도 끝까지 가도록 사상
            ur = r / RC_MAX_RP * (SERVO_MAX - SERVO_MID)
            up = p / RC_MAX_RP * (SERVO_MAX - SERVO_MID)
            uy = y / RC_MAX_Y * (SERVO_MAX - SERVO_MID)
            # 믹싱은 pid_control_node 와 동일하게 유지 (같은 좌/우 차동 구조)
            msg.data = [clamp(SERVO_MID + up + ur),
                        clamp(SERVO_MID + up - ur),
                        clamp(SERVO_MID + uy),
                        int(max(1000, min(2000, 1000 + t * (1000.0 / 1171.0))))]
        self.pub.publish(msg)

        el = time.monotonic() - self.t0
        if el - self.last >= 0.2:
            self.last = el
            d = msg.data
            s = "링크두절" if self.lost else "정상    "
            sys.stdout.write(
                f"\r[{s}] 스틱 R{self.rc[0]:+6.0f} P{self.rc[1]:+6.0f} "
                f"Y{self.rc[2]:+6.0f} T{self.rc[3]:+6.0f}  ->  "
                f"서보 [{d[0]} {d[1]} {d[2]}] 꼬리 {d[3]}   ")
            sys.stdout.flush()


def main():
    rclpy.init()
    n = Direct()
    print("\n=== 스틱 -> 서보 직결 (PID 없음) ===")
    print(f"스틱 최대값 가정: 롤/피치 ±{RC_MAX_RP:.0f}, 요 ±{RC_MAX_Y:.0f}")
    print("스틱을 끝까지 밀면 서보도 끝까지 가야 합니다.")
    print("Ctrl-C 로 종료하면 중립을 한 번 보내고 끝냅니다.\n")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    m = UInt16MultiArray()
    m.data = [SERVO_MID, SERVO_MID, SERVO_MID, 1000]
    n.pub.publish(m)
    time.sleep(0.2)
    print("\n\n관측된 스틱 최대 절대값:  "
          f"롤 {n.seen[0]:.0f}  피치 {n.seen[1]:.0f}  요 {n.seen[2]:.0f}  스로틀 {n.seen[3]:.0f}")
    print("스틱을 끝까지 밀었는데 롤/피치가 255, 요가 85 근처가 아니면")
    print("가정이 틀린 것이므로 인자로 실제 값을 넣어 다시 돌릴 것:")
    print("  python3 ~/servo_direct.py <롤피치최대> <요최대>\n")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
