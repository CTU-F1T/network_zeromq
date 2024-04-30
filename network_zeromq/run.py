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


if __name__ == "__main__":
    Core.init(args = sys.argv)

    n = Node("network_zeromq")

    Core.spin(n)
