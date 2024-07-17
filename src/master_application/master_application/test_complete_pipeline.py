import rclpy
import numpy as np
import time
from igus_moveit_clients.transform import Affine 

#import clients
from master_application.storage_position_handling import StorageClient
from sew_agv_clients.agv import AGVClient
from igus_moveit_clients.igus_moveit import ARMClient

def init_nodes():
    # igus robot
    robot = ARMClient()
    robot.home_position = [-3.14, 0.0, 1.57, 0.0, -0.87, 0.0]
    robot.setVelocity(0.5)
    robot.home()
    robot.clear_octomap()

    # sew agv
    agv = AGVClient()

    # storage
    storage = StorageClient()
    storage.positions_config_path = '/home/logilab/ros2_ws/src/master_application/config/storage_positions.yaml' # the path inside the docker container!!!

    return robot, agv, storage

def destroy_nodes(robot, agv, storage):
    robot.destroy_node()
    agv.destroy_node()
    storage.destroy_node()

def navigate_to_park_pose(storage, agv, target_name):
    # get and publish tf of target
    storage.publish_target_tf(target_name)

    # get and publish tf for park position of pos_1
    goal_msg = storage.get_park_poseCmd(target_name)
    if goal_msg == None:
        return None

    # navigate to to the approach pose for the target
    storage.publish_approach_tf(target_name)
    approach_name = storage.get_approach_name(target_name)
    approach_msg = storage.tf_to_navCmd(storage.get_transform(approach_name, 'map', affine=False))
    code_approach = agv.move_to_nav_goal(approach_msg)

    # navigate to to the park pose for the target
    code = agv.move_to_nav_goal(goal_msg)
    return True


def delete_published_tfs(storage, target_name):
    # delete published tfs
    time.sleep(20)
    storage.clear_tf(target_name)
    storage.clear_tf(storage.get_approach_name(target_name))
    storage.clear_tf('park_' + target_name)
    storage.clear_tf('Testpos_0')
    storage.clear_tf('Testpos_1')
    storage.clear_tf('Testpos_2')
    storage.clear_tf('Testpos_3')
    storage.clear_tf('Testpos_4')
    storage.clear_tf('Testpos_5')
    storage.clear_tf('Testpos_6')
    storage.clear_tf('robot_target_after_turn')
    storage.clear_tf('robot_target_before_turn')    


def record_octomap(robot):
    # move to camera record position 1
    robot.ptp_joint([0.0, -1.4, 1.22, 0.0, 1.57, 0.0])
    robot.clear_octomap()
    # camera record position 2 up-middle
    robot.ptp_joint([0.0, -1.4, 1.22, 0.0, 0.7, 0.0])
    # camera record position 3 left
    robot.ptp_joint([0.79, -1.4, 1.22, 0.0, 1.57, 0.0])
    # camera record position 3.1 left down
    robot.ptp_joint([0.79, -1.4, 1.22, 0.0, 0.7, 0.0])
    # camera record position 4 right
    robot.ptp_joint([-0.79, -1.4, 1.22, 0.0, 1.57, 0.0])
    # camera record position 4.1 right down
    robot.ptp_joint([-0.79, -1.4, 1.22, 0.0, 0.7, 0.0])
    time.sleep(1)

  

def move_arm_to_target(storage, robot, target_name):
    # get and publish tf of target
    storage.publish_target_tf(target_name)

    # get transform target pose to robot base frame
    robot_affine_tf = storage.get_transform(target_name, 'igus_base_link', affine=True)
    #print('Robot_tf_to Target:', robot_affine_tf)
    #storage.publish_affine_tf('igus_base_link', 'robot_target_before_turn', robot_affine_tf)

    # turn target tf 180deg round the z axis, to align gripper with target pose
    turn_180 = Affine((0.0,0.0,0.0),(0.0,0.0,1.0,0.0))
    robot_affine_tf = robot_affine_tf * turn_180

    print('Robot_tf_to execute:', robot_affine_tf)
    storage.publish_affine_tf('igus_base_link', 'robot_target_after_turn', robot_affine_tf)

    sucess = robot.ptp(robot_affine_tf)
    print('Success Robot Motion:', sucess)
    return sucess




################################################################################################################################################
####                                                            main()                                                                      ####
################################################################################################################################################             

def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up client nodes
    robot, agv, storage = init_nodes()
    time.sleep(5)

    # target pose (currently hardcoded, maybe in future as user input
    target = 'shelf_test'
    # navigate to park pose
    sucess = navigate_to_park_pose(storage, agv, target)

    # if navigation sucess, record octomap and move arm to target
    if sucess:
        record_octomap(robot)
        robot_sucess = False
        counter = 0
        while not robot_sucess and counter < 3:
            counter += 1 
            robot_sucess = move_arm_to_target(storage, robot, target)


    # destroy all nodes stop execution
    delete_published_tfs(storage, target)
    destroy_nodes(robot, agv, storage)

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()