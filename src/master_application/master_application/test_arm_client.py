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
    robot.home_position = [-3.0, 0.0, 1.57, 0.0, -0.87, 0.0]
    robot.setVelocity(0.5)

    # move robot to home position
    #robot.home()

    #pose = robot.get_transform('igus_tool0', 'igus_base_link', affine=False)
    affine = robot.get_transform('igus_tool0', 'igus_base_link', affine=True)
 
    world_movement = Affine((0.0,0.0,-0.2))
    new_affine = world_movement * affine

    print('New Affine:', new_affine)
    #print('Pose_tf', pose)
    print('Pose_affine', affine)


    # move robot to a specific position
    feedback = robot.lin(new_affine)
    print('Feedback:', feedback)


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()