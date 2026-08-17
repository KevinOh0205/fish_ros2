#!/usr/bin/env python3
# 서보 떨림 원인 가르기 — 완전히 고정된 값을 100Hz로 계속 보낸다.
#
# 파이가 보내는 명령은 이미 매끈한 것으로 측정됐다(표준편차 0.5us). 그래도
# 값이 1us씩 흔들리긴 하므로, 그것마저 없애고 "절대 안 변하는 명령"을 준다.
#
#   그래도 떨린다  -> 명령과 무관. 전원 / PWM 주변장치 충돌 / 서보 자체
#   안 떨린다      -> 명령의 1us 변동에 서보가 반응하는 것 (예상 밖이지만 가능)
#
# 반드시 pid_control_node 를 내리고 쓸 것. 안 그러면 발행자가 둘이 된다.
#   sudo systemctl stop fish-robot.service
#   ros2 run fish_robot_pkg uart_bridge_node &
#   python3 ~/servo_hold.py 1500
#
# 인자로 서보 목표값을 준다. 여러 값을 시험해 볼 것:
#   1500  중립
#   1400  한쪽으로 조금
#   1250  하한 (기구부가 못 가면 여기서 서보가 계속 힘을 쓴다)
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

VAL = int(sys.argv[1]) if len(sys.argv) > 1 else 1500
VAL = max(1250, min(1750, VAL))
TAIL = int(sys.argv[2]) if len(sys.argv) > 2 else 1000   # 1000 = 정지


class Hold(Node):
    def __init__(self):
        super().__init__('servo_hold')
        self.pub = self.create_publisher(UInt16MultiArray, '/motor/output', 10)
        self.msg = UInt16MultiArray()
        self.msg.data = [VAL, VAL, VAL, TAIL]
        self.create_timer(0.01, lambda: self.pub.publish(self.msg))


def main():
    rclpy.init()
    n = Hold()
    print(f"\n=== 고정값 송신 ===")
    print(f"서보 3개 모두 {VAL} us, 꼬리 {TAIL}")
    print("100Hz로 계속 보냅니다. 값은 절대 변하지 않습니다.")
    print("이 상태에서도 서보가 떨리면 명령과 무관한 문제입니다.\n")
    print("Ctrl-C 로 종료\n")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.msg.data = [1500, 1500, 1500, 1000]
    n.pub.publish(n.msg)
    time.sleep(0.2)
    print("\n중립으로 되돌리고 종료했습니다.\n")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
