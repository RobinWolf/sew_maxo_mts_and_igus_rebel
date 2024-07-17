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
    test_affine = Affine((0.50038992, -0.195618,    0.147), (0.00,0.00,-0.17110917,  0.98525208))

    pose_turn = Affine((0.0,0.0,0.0),(1.0,0.0,0.0,0.0))

    new_affine = world_movement * affine

    #print('New Affine:', new_affine)
    #print('Pose_tf', pose)
    #print('Pose_affine', affine)


    # move robot to a specific position
    print('Test Affine:', test_affine)
    print('Test_Affine_Translation:', test_affine.translation)
    print('Test_Affine_Quaternion:', test_affine.quat)
    feedback = robot.ptp(test_affine)
    print('Feedback:', feedback)

    # turned_affine = test_affine * pose_turn
    # print('Turned Affine:', turned_affine)
    # feedback = robot.ptp(turned_affine)
    # print('Feedback:', feedback)


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()