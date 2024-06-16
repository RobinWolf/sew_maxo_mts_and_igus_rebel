import rclpy
import numpy as np
import time
from igus_moveit_clients.transform import Affine #für 6D Transformation 

#import clients
from master_application.storage_position_handling import StorageClient

def main():
    # init and class setup
    rclpy.init(args=None)
    storage_client = StorageClient()
    
    storage_client.positions_config_path = '/home/logilab/ros2_ws/src/master_application/config/storage_positions.yaml' # the path inside the docker container!!!
    storage_client.robot_base_name = 'igus_base_link'

    time.sleep(5)

    # get and publish tf for pos1
    storage_client.get_target_tf('pos1')

    # get and publish tf for park position of pos_1
    #storage_client.get_park_positions('pos1')

    # get and publish relative tf from robot base to target (to be used for robot arm motion planning)
    affine = storage_client.get_transform('igus_link_2','sew_base_footprint')   # from (child), to (parent)

    print(affine)


    # class node shutdown
    storage_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()