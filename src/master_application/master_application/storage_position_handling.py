import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import math
import tf_transformations as tft
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time, Duration
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

from nav2_msgs.action import NavigateToPose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from igus_moveit_clients.transform import Affine
from sew_agv_clients.agv import AGVClient


class StorageClient(Node):
    def __init__(self):
        super().__init__('storage_client_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)   # broadcaster to publish by transforms relative to the map frame in tf tree (/tf_static)
        self.positions_config_path = None # default: '/home/ros2_ws/src/master_application/config/positions.yaml'
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)
        self.collisionChecker = AGVClient()

        # class variables, can be modified for other hardware setups, this are defaults for sew maxo mts and igus rebel (SS24)
        self.armRange = 0.66
        self.agvOffsetX = 0.4
        self.agvOffsetY = -0.19
        self.parkClearanceX = 0.15
        self.stepSize = 0.1
        self.joint1Heigth = 0.86

        self.get_logger().info('storage handling node initialized')

    def publish_affine_tf(self, parent_frame, child_frame, affine):
        """
        string parent_frame: name of the parent frame
        string child_frame: name of the child frame
        Affine() affine: Affine transformation of the tf to publish

        Returns:
        --------
        None
        """
        # load target data from yaml file
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.header.stamp =  self.get_clock().now().to_msg()
        tf.transform.translation.x = affine.translation[0]
        tf.transform.translation.y = affine.translation[1]
        tf.transform.translation.z = affine.translation[2]
        tf.transform.rotation.x = affine.quat[0]
        tf.transform.rotation.y = affine.quat[1]
        tf.transform.rotation.z = affine.quat[2]
        tf.transform.rotation.w = affine.quat[3]

        # publish tf
        self.tf_static_broadcaster.sendTransform(tf)
        self.get_logger().info(f'published tf for {child_frame}')

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

    def get_approach_name(self, target_name):
        """
        string target_name: name of the target to reach

        Returns:
        --------
        string approach_name: name of the approach pose for the given target
        """
        # load target data from yaml file
        target_data = self.load_yaml(self.positions_config_path)[target_name]
        approach_name = target_data['approach']
        return approach_name


    def publish_approach_tf(self, target_name):
        """
        string target_name: name of the target to publish its tf

        Returns:
        --------
        None
        """
        # load approach data from yaml file
        approach_name = self.get_approach_name(target_name)
        approach_data = self.load_yaml(self.positions_config_path)[approach_name]

        approach_tf = TransformStamped()
        approach_tf.header.frame_id = 'map'
        approach_tf.child_frame_id = approach_name
        approach_tf.header.stamp =  self.get_clock().now().to_msg()
        approach_tf.transform.translation.x = approach_data['position']['x']
        approach_tf.transform.translation.y = approach_data['position']['y']
        approach_tf.transform.translation.z = approach_data['position']['z']
        approach_tf.transform.rotation.x = approach_data['orientation']['x']
        approach_tf.transform.rotation.y = approach_data['orientation']['y']  
        approach_tf.transform.rotation.z = approach_data['orientation']['z']
        approach_tf.transform.rotation.w = approach_data['orientation']['w']

        # publish tf
        self.tf_static_broadcaster.sendTransform(approach_tf)
        self.get_logger().info(f'published tf for {approach_name}')
        
        
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


    def get_park_poseCmd(self, target_name):
        """
        string target_name: name of the target to reach

        Returns:
        --------
        nav2_msgs/action/NavigateToPose.action.goal: goal pose         
        if not reachable returns None
        """
        # get geometrymsgs/TransformStamped between map and target position from (child), to (parent)
        target_tf = self.get_transform(target_name, 'map', affine=False)
        
        # set valid search space for park position in relation to the height of the target position
        searchRange = math.sqrt((self.armRange-self.parkClearanceX)**2 - (target_tf.transform.translation.z-self.joint1Heigth)**2)

        # Calculate the step size for test points
        numTestpoints = int(searchRange // self.stepSize) 
        # interpolate line from target position along local x-axis to find park position
        for testCycle in range (numTestpoints):
            #calculate new test point on x axis (on the ground - z = 0) --> child of target_tf
            test_tf = TransformStamped()
            test_tf.header.frame_id = target_name
            test_tf.child_frame_id = f'Testpos_{testCycle}'
            test_tf.transform.translation.x = testCycle * self.stepSize  # only relative translation to target position, no rotation
            test_tf.transform.translation.y = 0.0
            test_tf.transform.translation.z = -target_tf.transform.translation.z

            self.tf_static_broadcaster.sendTransform(test_tf)

            # check if test point is in collision or not (goal_msg)
            goal_msg = self.tf_to_navCmd(test_tf)
            reachable = self.collisionChecker.check_nav_goal(goal_msg)
            if reachable:
                test_tf.transform.translation.x = test_tf.transform.translation.x + self.agvOffsetX + self.parkClearanceX
                test_tf.transform.translation.y = test_tf.transform.translation.y + self.agvOffsetY
                test_tf_to_map = self.get_transform(test_tf.child_frame_id, 'map', affine=False)
                park_tf = test_tf_to_map                                                
                self.get_logger().info(f'Test point {testCycle} is not in collision ...')
                break
            else:
                park_tf = None
                self.get_logger().warn(f'Test point {testCycle} is in collision...')

        # return None if no park position found in range
        if park_tf == None:
            self.get_logger().error(f'No park pose found from which the arm can theoretically reach the target: {target_name}')
            return None
        
        # publish nearest park position                                                         
        park_tf = TransformStamped()
        park_tf.header.frame_id = target_name
        park_tf.child_frame_id = 'park_' + target_name
        park_tf.transform = test_tf.transform

        # publish tf for park position
        self.tf_static_broadcaster.sendTransform(park_tf)
        self.get_logger().info(f'published tf for park position of {target_name}')

        return self.tf_to_navCmd(park_tf)
    

   
    def get_transform(self, from_frame_rel, to_frame_rel, affine=True):
        """
        string from_frame_rel: name of the source frame frame to transform from (child)
        string to_frame_rel: name of the target frame to transform into (parent)

        Returns:
        --------
        Affine: transformation matrix from robot base to target position and orientation (4x4 numpy array, can directly passed in motion planning)
        or
        geometry_msgs/TransformStamped: transformation between the two frames if affine=False
        """
        # Calculate the transform between the robot base frame and the target frame
        counter = 0
        has_transform = False
        transform = None
        while (not has_transform) and counter < 200:
            rclpy.spin_once(self)

            counter += 1

            try:
                has_transform = self.tf_buffer.can_transform(to_frame_rel, from_frame_rel, time=Time(), timeout=Duration(seconds=0.01))
                if has_transform:
                    transform = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, time=Time(), timeout=Duration(seconds=0.01))   # takes latest available tf in buffer
                    trans = transform.transform
                    if affine:
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
    
    @staticmethod
    def tf_to_navCmd(tf):
        """
        TransformStamped() goal tf to navigate agv to (z axis has to be vertical, parent recommendet always 'map' frame for better understanding)

        Returns
        -------
        nav2_msgs/action/NavigateToPose.action.goal: goal pose 
        """
        goal_msg = NavigateToPose.Goal() # goal field of action definition
        goal_msg.pose.header.frame_id = tf.header.frame_id # geometry_msgs/PoseStamped.msg
        goal_msg.pose.pose.position.x = tf.transform.translation.x
        goal_msg.pose.pose.position.y = tf.transform.translation.y
        goal_msg.pose.pose.position.z = tf.transform.translation.z
        goal_msg.pose.pose.orientation.x = tf.transform.rotation.x
        goal_msg.pose.pose.orientation.y = tf.transform.rotation.y
        goal_msg.pose.pose.orientation.z = tf.transform.rotation.z
        goal_msg.pose.pose.orientation.w = tf.transform.rotation.w  
     
        return goal_msg



        
