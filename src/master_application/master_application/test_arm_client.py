import rclpy
import numpy as np
import time
from igus_moveit_clients.transform import Affine #für 6D Transformation 

#import clients
from igus_moveit_clients.igus_moveit import ARMClient


def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up node with agv clients
    robot = ARMClient()

    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # move robot to home position
    print("move robot to home position")
    robot.home()

    time.sleep(5)

    robot.home()

    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()