#!/usr/bin/env python3


import rclpy
from minimal_simulator.minimal_simulator_node import MinimalSimulatorNode


def main(args=None):

    rclpy.init(args=args)

    try:
        node = MinimalSimulatorNode()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()
