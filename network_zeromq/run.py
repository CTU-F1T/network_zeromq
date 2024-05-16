#!/usr/bin/env python
# run.py
"""Run a ROS node for network communication.

Successor of 'network_communication'. Unfortunately, this one (because of ROS2)
only sends strings, not the messages.
"""
######################
# Imports & Globals
######################

import sys

from autopsy.core import Core
from autopsy.node import Node
from autopsy.qos import QoSProfile

import zmq

import argparse

import threading

from std_msgs.msg import String

print ("""Loaded zmq.
\tZeroMQ version: %s
\tpyzmq version: %s
""" % (zmq.zmq_version(), zmq.pyzmq_version()))

PARSER = argparse.ArgumentParser(
    prog = "run.py",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description = """
Create a network connection between two devices.
    """,  # noqa: E501
)

PARSER.add_argument(
    "ip_address",

    help = "IP address of this computer, %%s",
    type = str,
    metavar = "LOCAL_IP_ADDRESS",
)

PARSER.add_argument(
    "port",

    help = "port number of local socket, %%d",
    type = int,
    metavar = "LOCAL_PORT",
)

PARSER.add_argument(
    "remote_ip",

    help = "IP address of other computer, %%d",
    type = str,
    metavar = "REMOTE_IP_ADDRESS",
)

PARSER.add_argument(
    "remote_port",

    help = "port number to connect to, %%d",
    type = int,
    metavar = "REMOTE_PORT",
)


######################
# NetworkNode
######################

class NetworkNode(Node):
    """ROS Node for communicating over network."""

    # ZeroMQ variables
    context = None
    socket_din = None
    socket_dout = None


    def __init__(
        self,
        ip_address, port,
        remote_ip, remote_port,
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
        """
        super(NetworkNode, self).__init__(*args, **kwargs)

        # Initialize and set the ZeroMQ
        self.context = zmq.Context()

        self.socket_din = self.context.socket(zmq.PULL)
        self.socket_din.bind(  # One side has to be bound!
            "tcp://%s:%d" % (ip_address, port)
        )

        self.socket_dout = self.context.socket(zmq.PUSH)
        self.socket_dout.connect(
            "tcp://%s:%d" % (remote_ip, remote_port)
        )


        # Create topics for handling data
        self.create_subscription(
            String, "/network/send", self.callback_send,
            qos_profile = QoSProfile(depth = 1)
        )

        self.pub_data = self.create_publisher(
            String, "/network/receive",
            qos_profile = QoSProfile(depth = 1)
        )


        # Initialize and set threading
        self.publisher = threading.Thread(
            target = self.publisher_thread
        )
        self.publisher.start()

    #
    # Callbacks #
    def callback_send(self, data):
        """Send data over network.

        Arguments
        ---------
        data: std_msgs/String
            data to be send over network
        """
        self.loginfo("Sending data: '%s'" % data.data)
        self.socket_dout.send_string(data.data)

    #
    # Thread #
    def publisher_thread(self):
        """Receive data from network and publish them to ROS."""
        while True:
            data = self.socket_din.recv_string()

            self.loginfo("Received data: '%s'" % data)
            self.pub_data.publish(
                String(data = data)
            )


######################
# Main
######################

def main():
    """Main function."""
    args, other = PARSER.parse_known_args()

    Core.init(args = sys.argv)

    n = NetworkNode(
        name = "network_zeromq",
        ip_address = args.ip_address, port = args.port,
        remote_ip = args.remote_ip, remote_port = args.remote_port
    )

    Core.spin(n)

    Core.shutdown()


if __name__ == "__main__":
    main()
