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
    robot_base_frame = 'igus_base_link'

    time.sleep(5)

    # get and publish tf for pos1
    storage_client.publish_target_tf('desk_test')

    # get and publish tf for park position of pos_1
    storage_client.get_park_poseCmd('desk_test', 4, 1, 0.2, True)

    # get and publish relative tf from robot base to target (to be used for robot arm motion planning)
    #affine = storage_client.get_transform('igus_tool0', robot_base_frame)   # from (child), to (parent)
    #print(affine)

    time.wait(20)

    storage_client.clear_tf('desk_test')


    # class node shutdown
    storage_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



########################################### POSES ########################################################
# base to camera view: [-0.36511848  0.00518532  0.56580814] [-0.39106312  0.00596301  0.92033688  0.00375518]
# map to test_pose: [-5.0975369  -2.40226413  0.94048264] [-1.01751061e-02  3.38666639e-04  9.99154612e-01  3.98298179e-02]

# base to test_pose: [-0.61956172  0.01083479  0.36048264] [-1.01807146e-02  2.21394488e-05  9.99909959e-01  8.74223524e-03]


# agv desk park pose pose: (with ros2 topic echo /goal_pose)
#   position:
#     x: -3.9319663047790527
#     y: -2.550388813018799
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: -0.02054567561733093
#     w: 0.9997889153283444
