"""
Unitree Go2 API test.

Roberto Masocco <r.masocco@dotxautomation.com>

June 20, 2024
"""

import sys
import time

import rclpy

from go2_tester.go2_tester import Go2Tester


def main():
    rclpy.init(args=sys.argv)
    node = Go2Tester()

    # Send test requests
    time.sleep(1.0)
    node.heart()
    # node.hello()
    # node.standdown()
    # time.sleep(3.0)
    # node.standup()
    # time.sleep(3.0)
    # node.standdown()
    # time.sleep(3.0)
    # node.recoverystand()

    # init_time = time.time()
    # while True:
    #   node.move(0.0, -0.1, 0.0)
    #   rclpy.spin_once(node, timeout_sec=0.1)
    #   if (time.time() - init_time) > 3.0:
    #       break
    # node.stopmove()

    # Wait for responses
    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
