import rclpy
import numpy as np
import time
from igus_moveit_clients.transform import Affine #für 6D Transformation 

#import clients
from master_application.storage_position_handling import StorageClient
from sew_agv_clients.agv import AGVClient

def main():
    # init and class setup
    rclpy.init(args=None)
    storage_client = StorageClient()
    agv = AGVClient()
    
    storage_client.positions_config_path = '/home/logilab/ros2_ws/src/master_application/config/storage_positions.yaml' # the path inside the docker container!!!

    time.sleep(5)

    # get and publish tf for pos1
    storage_client.publish_target_tf('shelf_test')

    # get and publish tf for park position of pos_1
    goalFrame, goalCmd = storage_client.get_park_poseCmd('shelf_test', 0.1, True)

    print('goalFrame: ', goalFrame, 'goalCmd: ',goalCmd)

    # call method from client class (string frameID, float64 pose[x,y,w])
    code = agv.move_to_nav_goal(goalFrame,goalCmd)
    print('collision checker return statement: ',code)



    time.sleep(20)

    storage_client.clear_tf('shelf_test')
    storage_client.clear_tf('park_shelf_test')
    storage_client.clear_tf('Testpos_0')
    storage_client.clear_tf('Testpos_1')
    storage_client.clear_tf('Testpos_2')
    storage_client.clear_tf('Testpos_3')
    storage_client.clear_tf('Testpos_4')
    storage_client.clear_tf('Testpos_5')
    storage_client.clear_tf('Testpos_6')


    # class node shutdown
    storage_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
