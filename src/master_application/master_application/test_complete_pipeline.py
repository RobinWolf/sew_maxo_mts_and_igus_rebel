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
    visualize_tf = False
    goalFrame, goalCmd = storage.get_park_poseCmd(target_name, visualize_tf)

    if goalFrame == None:
        print('No park pose found in which the arm can theoretically reach the target: ', target_name)
        return None

    # call method from client class (string frameID, float64 pose[x,y,w])
    code = agv.move_to_nav_goal(goalFrame,goalCmd)

    if visualize_tf:
        time.sleep(20)
        storage.clear_tf(target_name)
        storage.clear_tf(target_name+'_park')
        storage.clear_tf('Testpos_0')
        storage.clear_tf('Testpos_1')
        storage.clear_tf('Testpos_2')
        storage.clear_tf('Testpos_3')
        storage.clear_tf('Testpos_4')
        storage.clear_tf('Testpos_5')
        storage.clear_tf('Testpos_6')

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

  

def move_arm_to_target(robot, target_name):
    eins = 1



def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up client nodes
    robot, agv, storage = init_nodes()
    time.sleep(5)

    record_octomap(robot)

    # navigate to park pose
    # sucess = navigate_to_park_pose(storage, agv, 'shelf_test')

    # #if navigation sucess, record octomap and move arm to target
    # if sucess:
    #     record_octomap(robot)






    # destroyall nodes stop execution
    destroy_nodes(robot, agv, storage)
    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()