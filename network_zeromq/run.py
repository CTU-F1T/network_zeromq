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

    def __init__(self, *args, **kwargs):
        """Initialize the node."""
        super(NetworkNode, self).__init__(*args, **kwargs)


    # TODO: Callback on a topic, send data over network.


    # TODO: Thread reading data from network, publishing to the topic.


######################
# Main
######################

if __name__ == "__main__":
    args, other = PARSER.parse_known_args()

    Core.init(args = sys.argv)

    n = NetworkNode("network_zeromq")

    Core.spin(n)
