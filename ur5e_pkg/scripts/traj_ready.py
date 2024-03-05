#!/usr/bin/env python3
import rclpy
from command_node import CommandClient

def main(args=None):

    rclpy.init(args=args)

    command = [0,-2,2,-1.57,-1.57,0]
    command_client = CommandClient(command)
    command_client.send_goal()

    rclpy.spin_until_future_complete(command_client, command_client.send_goal())

    command_client.destroy_node()

if __name__ == '__main__':

    main()