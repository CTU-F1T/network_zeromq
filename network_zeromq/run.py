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

import zmq

import argparse

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
        self.socket_din.connect(
            "tcp://%s:%d" % (ip_address, port)
        )

        self.socket_dout = self.context.socket(zmq.PUSH)
        self.socket_dout.connect(
            "tcp://%s:%d" % (remote_ip, remote_port)
        )


    # TODO: Callback on a topic, send data over network.


    # TODO: Thread reading data from network, publishing to the topic.


######################
# Main
######################

if __name__ == "__main__":
    args, other = PARSER.parse_known_args()

    Core.init(args = sys.argv)

    n = NetworkNode(
        name = "network_zeromq",
        ip_address = args.ip_address, port = args.port,
        remote_ip = args.remote_ip, remote_port = args.remote_port
    )

    Core.spin(n)
