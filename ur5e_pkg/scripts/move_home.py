#!/usr/bin/env python3

import rclpy
from command_node import CommandNode

def main(args=None):
    rclpy.init(args=args)

    command = [0,0,0,0,0,0]
    command_node = CommandNode(command)

    rclpy.spin_once(command_node)

    command_node.destroy_node()

if __name__ == '__main__':
    main()