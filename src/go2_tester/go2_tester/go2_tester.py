"""
Unitree Go2 API tester node.

Roberto Masocco <r.masocco@dotxautomation.com>

June 20, 2024
"""

import json
import time

from rclpy.node import Node

from unitree_api.msg import Request, Response
from unitree_go.msg import WirelessController

# Sport (movement) API IDs
ROBOT_SPORT_API_ID_MOVE = 1008
ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006
ROBOT_SPORT_API_ID_STANDUP = 1004
ROBOT_SPORT_API_ID_STANDDOWN = 1005
ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019
ROBOT_SPORT_API_ID_DAMP = 1001
ROBOT_SPORT_API_ID_HEART = 1036
ROBOT_SPORT_API_ID_HELLO = 1016


class Go2Tester(Node):
    """
    Go2Tester node.
    """

    def __init__(self) -> None:
        """
        Constructor.
        """
        super().__init__('go2_tester')

        # Initialize publishers
        self._sport_req_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10)

        # Initialize subscribers
        self._controller_sub = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.controller_clbk,
            10)

        self.get_logger().info('Node initialized')

    def controller_clbk(self, msg: WirelessController) -> None:
        """
        Wireless controller callback.

        :param msg: Wireless controller message.
        """
        kill_switch_keys = [48]
        if msg.keys in kill_switch_keys:
            self.get_logger().fatal('KILL SWITCH')
            exit(1)

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        """
        Move the robot with velocity control.

        :param vx: Linear velocity in x-axis.
        :param vy: Linear velocity in y-axis.
        :param vyaw: Angular velocity around z-axis.
        """
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE

        # Build request body
        req.parameter = json.dumps({
            'x': vx,
            'y': vy,
            'z': vyaw
        })

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent MOVE request ({req.header.identity.id})')

    def stopmove(self) -> None:
        """
        Stop the robot.
        """
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent STOPMOVE request ({req.header.identity.id})')

    def standdown(self) -> None:
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent STANDDOWN request ({req.header.identity.id})')

    def standup(self) -> None:
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent STANDUP request ({req.header.identity.id})')

    def recoverystand(self) -> None:
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent RECOVERYSTAND request ({req.header.identity.id})')

    def heart(self) -> None:
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_HEART

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent HEART request ({req.header.identity.id})')

    def hello(self) -> None:
        req = Request()

        # Build request header
        req.header.identity.id = int(time.clock_gettime(time.CLOCK_REALTIME))
        req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO

        # Publish request
        self._sport_req_pub.publish(req)
        self.get_logger().warning(
            f'Sent HELLO request ({req.header.identity.id})')
