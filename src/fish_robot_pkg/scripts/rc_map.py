#!/usr/bin/env python3
# 조종기 스틱 -> /rc/command 대응 관계를 확인한다.
#
# 스틱을 하나씩 움직이면서 "어느 칸이 얼마나 움직였는지" 를 본다.
#   다른 칸이 움직이면   -> 채널이 뒤바뀐 것
#   같은 칸인데 부호 반대 -> 부호만 뒤집힌 것
# 둘은 고치는 곳이 다르므로 반드시 구분해야 한다.
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion

NAMES = ['x (roll)', 'y (pitch)', 'z (yaw)', 'w (throttle)']


class Map(Node):
    def __init__(self):
        super().__init__('rc_map')
        self.v = [0.0] * 4
        self.lo = [0.0] * 4
        self.hi = [0.0] * 4
        self.last = 0.0
        self.t0 = time.monotonic()
        self.create_subscription(Quaternion, '/rc/command', self.cb, 20)
        self.create_timer(0.05, self.tick)

    def cb(self, m):
        if int(m.w) == -9999:          # 링크 두절 센티넬은 무시
            return
        self.v = [m.x, m.y, m.z, m.w]
        for i, x in enumerate(self.v):
            self.lo[i] = min(self.lo[i], x)
            self.hi[i] = max(self.hi[i], x)

    def bar(self, x, span=400.0):
        """중앙 기준 좌우 막대. 부호를 눈으로 보기 위한 것."""
        n = 20
        k = int(max(-1.0, min(1.0, x / span)) * n)
        if k >= 0:
            return ' ' * n + '|' + '#' * k + ' ' * (n - k)
        return ' ' * (n + k) + '#' * (-k) + '|' + ' ' * n

    def tick(self):
        el = time.monotonic() - self.t0
        if el - self.last < 0.15:
            return
        self.last = el
        out = []
        for i in range(4):
            out.append(f"{NAMES[i]:>13} {self.v[i]:+7.0f} {self.bar(self.v[i])}")
        sys.stdout.write("\033[H\033[J" + "스틱을 하나씩 끝까지 밀어보세요.\n"
                         "왼쪽으로 뻗으면 음수, 오른쪽이면 양수입니다.\n\n"
                         + "\n".join(out) + "\n")
        sys.stdout.flush()


def main():
    rclpy.init()
    n = Map()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    print("\n\n=== 각 칸이 실제로 움직인 범위 ===")
    for i in range(4):
        print(f"  {NAMES[i]:>13}   최소 {n.lo[i]:+7.0f}   최대 {n.hi[i]:+7.0f}   "
              f"폭 {n.hi[i]-n.lo[i]:7.0f}")
    print("\n스틱 하나만 움직였는데 다른 칸의 폭도 크면 -> 채널이 뒤섞인 것")
    print("폭은 맞는데 방향이 반대면 -> 부호만 뒤집힌 것\n")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
