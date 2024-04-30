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

print ("""Loaded zmq.
\tZeroMQ version: %s
\tpyzmq version: %s
""" % (zmq.zmq_version(), zmq.pyzmq_version()))


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
    # TODO: Handle arguments

    Core.init(args = sys.argv)

    n = NetworkNode("network_zeromq")

    Core.spin(n)
