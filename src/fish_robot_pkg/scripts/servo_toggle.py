#!/usr/bin/env python3
# 파이 -> nRF -> 서보 경로가 실제로 통하는지 눈으로 확인한다.
#
# 2초마다 1350 <-> 1650 을 번갈아 보낸다. 서보가 그 리듬에 맞춰 좌우로
# 크게 움직이면 경로 전체가 살아 있는 것이다.
#
# 왜 이렇게 느리고 크게 하는가: 떨림과 구분하기 위해서다. 빠르게 조금씩
# 바꾸면 떨림에 묻혀서 "명령을 따르는 것"인지 "그냥 떠는 것"인지 알 수 없다.
#
#   sudo systemctl stop fish-robot.service
#   ros2 run fish_robot_pkg uart_bridge_node &
#   python3 ~/servo_toggle.py
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

LO, HI = 1350, 1650
PERIOD = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0


class Toggle(Node):
    def __init__(self):
        super().__init__('servo_toggle')
        self.pub = self.create_publisher(UInt16MultiArray, '/motor/output', 10)
        self.t0 = time.monotonic()
        self.cur = None
        self.create_timer(0.01, self.tick)     # 송신은 100Hz 유지

    def tick(self):
        el = time.monotonic() - self.t0
        v = LO if int(el / PERIOD) % 2 == 0 else HI
        m = UInt16MultiArray()
        m.data = [v, v, v, 1000]               # 꼬리는 정지 유지
        self.pub.publish(m)
        if v != self.cur:
            self.cur = v
            print(f"  {el:5.1f}s   ->  {v}   "
                  f"{'◀ 왼쪽' if v == LO else '오른쪽 ▶'}")


def main():
    rclpy.init()
    n = Toggle()
    print(f"\n=== 서보 왕복 시험 ===")
    print(f"{PERIOD:.0f}초마다 {LO} <-> {HI} 로 번갈아 보냅니다.")
    print("서보 3개가 그 리듬에 맞춰 좌우로 크게 움직이는지 보세요.\n")
    print("  움직임  -> 파이에서 nRF까지 명령이 잘 갑니다")
    print("  안 움직임 -> 경로가 끊겼거나 서보 전원 문제\n")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    m = UInt16MultiArray()
    m.data = [1500, 1500, 1500, 1000]
    n.pub.publish(m)
    time.sleep(0.2)
    print("\n중립으로 되돌리고 종료했습니다.\n")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
