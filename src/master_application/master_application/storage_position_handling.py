import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import tf_transformations as tft
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time, Duration
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from igus_moveit_clients.transform import Affine
import igus_moveit_clients.util as util
from sew_agv_clients.agv import AgvClient


class StorageClient(Node):
    def __init__(self):
        super().__init__('storage_position_handling_client_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)   # broadcaster to publish by transforms relative to the map frame in tf tree (/tf_static)
        self.positions_config_path = None # default: '/home/ros2_ws/src/master_application/config/positions.yaml'
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)
        self.collisionChecker = AgvClient()
        self.get_logger().info('storage handling node initialized')


    def publish_target_tf(self, target_name):
        """
        string target_name: name of the target to publish its tf

        Returns:
        --------
        None
        """
        # load target data from yaml file
        target_data = self.load_yaml(self.positions_config_path)[target_name]
        target_tf = TransformStamped()
        target_tf.header.frame_id = 'map'
        target_tf.child_frame_id = target_name
        target_tf.header.stamp =  self.get_clock().now().to_msg()
        target_tf.transform.translation.x = target_data['position']['x']
        target_tf.transform.translation.y = target_data['position']['y']
        target_tf.transform.translation.z = target_data['position']['z']
        target_tf.transform.rotation.x = target_data['orientation']['x']
        target_tf.transform.rotation.y = target_data['orientation']['y']  
        target_tf.transform.rotation.z = target_data['orientation']['z']
        target_tf.transform.rotation.w = target_data['orientation']['w']

        # publish tf
        self.tf_static_broadcaster.sendTransform(target_tf)
        self.get_logger().info(f'published tf for {target_name}')
        
        
    def clear_tf(self, tf_name):
        """
        string tf_name: name of the target tf to clear

        Returns:
        --------
        None
        """
        # Create a TransformStamped message with an empty transform
        empty_tf = TransformStamped()
        empty_tf.header.frame_id = 'map'
        empty_tf.child_frame_id = tf_name
        empty_tf.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the empty transform to clear the tf
        self.tf_static_broadcaster.sendTransform(empty_tf)
        self.get_logger().info(f'Cleared tf for {tf_name}')



    def get_park_positions(self, target_name, numTestpoints, armRange, offset):
        """
        string target_name: name of the target to reach

        Returns:
        --------
        bool sucess: whether a park position was found from which the robot can reach the desired target position or not
        """
        park_position_reachable = False

        # get affine between map and target position from (child), to (parent)
        target_tf = self.get_transform(target_name, 'map')
        
        # Calculate the step size for test points
        step_size = (armRange-offset) / numTestpoints

        # interpolate line from target position along local x-axis to find park position
        for testCycle in range (numTestpoints):
            #calculate new test point on x axis (on the ground - z = 0)
            testPoint = Affine()
            testPoint.rotation = np.identity(3)
            testPoint.translation = testCycle * np.array([step_size, 0, -target_tf.translation[2]])

            # publish test tf only for testing purposes
            test_tf = TransformStamped()
            test_tf.header.frame_id = target_name
            test_tf.child_frame_id = 'test_position_' + testCycle + target_name
            test_tf.transform.translation = util.affine_to_transform(testPoint).translation
            test_tf.transform.rotation = util.affine_to_transform(testPoint).rotation

            self.tf_static_broadcaster.sendTransform(test_tf)

            # check if test point is in collision or not (frameID, pose)
            reachable = self.collisionChecker.check_nav_goal()
            if reachable:
                park_position_reachable = True
                break
        
        
        park_tf = TransformStamped()
        park_tf.header.frame_id = target_name
        park_tf.child_frame_id = 'test_position_' + testCycle + target_name
        park_tf.transform.translation = util.affine_to_transform(testPoint).translation
        park_tf.transform.rotation = util.affine_to_transform(testPoint).rotation


        # publish tf for park position
        self.tf_static_broadcaster.sendTransform(park_tf)
        self.get_logger().info(f'published tf for park position of {target_name}')
        return park_position_reachable
    

   
    def get_transform(self, from_frame_rel, to_frame_rel):
        """
        string from_frame_rel: name of the source frame frame to transform from (child)
        string to_frame_rel: name of the target frame to transform into (parent)

        Returns:
        --------
        Affine: transformation matrix from robot base to target position and orientation (4x4 numpy array, can directly passed in motion planning)
        """
        # Calculate the transform between the robot base frame and the target frame
        counter = 0
        has_transform = False
        transform = None
        while (not has_transform) and counter < 200:
            rclpy.spin_once(self)

            counter += 1
    
            #if counter % 10 == 0:
            #    self.get_logger().warn(f'Try to find transform {to_frame_rel} to {from_frame_rel} {counter} times... try again...')

            try:
                has_transform = self.tf_buffer.can_transform(to_frame_rel, from_frame_rel, time=Time(), timeout=Duration(seconds=0.01))
                if has_transform:
                    trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time=Time(), timeout=Duration(seconds=0.01))   # takes latest available tf in buffer
                    trans = trans.transform
                    transform = Affine(
                        [trans.translation.x, trans.translation.y, trans.translation.z],
                        [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
                
            except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().error(f'Exception during transform {from_frame_rel} to {to_frame_rel} after {counter} tries... {ex}')                
                transform = None
            
        if transform != None:    
            self.get_logger().info(f'Found transform {from_frame_rel} to {to_frame_rel} after {counter} tries') 
        else:
            self.get_logger().error(f'Could not transform {from_frame_rel} to {to_frame_rel} after {counter} tries...try again and check passed frame ids...')
        return transform


    


    ##########################################################################################################################################################
    @staticmethod
    def load_yaml(path):
        """
        string path: path to yaml file

        Returns
        -------
        dict data: data from yaml file
        """
        with open(path, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
        return data

        
