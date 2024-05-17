#!/usr/bin/env python
# run.py
"""Run a ROS node for network communication.

Successor of 'network_communication'. Unfortunately, this one (because of ROS2)
only sends strings, not the messages.

This one handles odom/speed communication.
"""
######################
# Imports & Globals
######################

import sys

from autopsy.core import Core
from autopsy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)

try:
    from network_zeromq.run import NetworkNode, PARSER
except ImportError:
    from run import NetworkNode, PARSER


from nav_msgs.msg import Odometry

try:
    from vesc_msgs.msg import VescStateStamped
    USE_VESC = True
except ImportError:
    print ("Unable to import 'vesc_msgs.msg'. Speed will not be recorded.")
    USE_VESC = False


PARSER.add_argument(
    "rate",

    help = "frequency of sending the odom/speed data, %%d",
    type = int,
    default = 0,
    metavar = "RATE",
)


######################
# NetworkNode
######################

class StateNetworkNode(NetworkNode):
    """ROS Node for sending states over network."""

    last_odom = None
    last_speed = None


    def __init__(
        self,
        rate,
        *args, **kwargs
    ):
        """Initialize the node.

        This creates a ROS node and defines two sockets; one local to obtain
        data (`ip_address` and `port`) and one to a remote device to send
        data to.

        Arguments
        ---------
        ip_address: str
            IP address of this computer (used to select interface)
        port: int
            port number of the locally opened socket
        remote_ip: str
            IP address of the remote device
        remote_port: int
            port number of the remote device
        rate: int
            frequency to send odom/speed over network
        """
        super(StateNetworkNode, self).__init__(*args, **kwargs)

        self.last_speed = 0.0

        # Create topics for handling data
        self.create_subscription(
            Odometry, "/odom", self.callback_odom,
            qos_profile = QoSProfile(
                depth = 1,
                durability = DurabilityPolicy.VOLATILE,
                reliability = ReliabilityPolicy.BEST_EFFORT,
            )
        )

        if USE_VESC:
            self.create_subscription(
                VescStateStamped, "/sensors/core", self.callback_vesc,
                qos_profile = QoSProfile(
                    depth = 1,
                    durability = DurabilityPolicy.VOLATILE,
                    reliability = ReliabilityPolicy.BEST_EFFORT,
                )
            )

        # Create timer
        self.create_timer(
            rate, timer_send_data
        )

    #
    # Callbacks #
    def callback_send(self, data):
        """Report that send is not available."""
        self.logwarn(
            "Direct send (via '/network/send') is not available, "
            "as the connection is used to send odom/speed data."
        )


    def callback_odom(self, data):
        """Receive odometry data.

        Arguments
        ---------
        data: nav_msgs/Odometry
            current odometry data of the car
        """
        self.last_odom = data


    def callback_vesc(self, data):
        """Receive speed of the car.

        Arguments
        ---------
        data: vesc_msgs/VescStateStamped
            current state of VESC controller
        """
        self.last_speed = data.state.speed / 4105.324277107

    #
    # Timer #
    def timer_send_data(self, *args, **kwargs):
        """Send current states of the network."""
        self.socket_dout.send_string(
            ",".join(["%s" % val for val in [
                self.last_odom.position.x,
                self.last_odom.position.y,
                self.last_odom.orientation.x,
                self.last_odom.orientation.y,
                self.last_odom.orientation.z,
                self.last_odom.orientation.w,
                self.last_speed
            ]])
        )


######################
# Main
######################

def main():
    """Main function."""
    args, other = PARSER.parse_known_args()

    Core.init(args = sys.argv)

    n = StateNetworkNode(
        name = "network_zeromq",
        ip_address = args.ip_address, port = args.port,
        remote_ip = args.remote_ip, remote_port = args.remote_port,
        rate = args.rate
    )

    Core.spin(n)

    Core.shutdown()


if __name__ == "__main__":
    main()
