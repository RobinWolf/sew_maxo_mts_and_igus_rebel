import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
from typing import List
from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

# service and action interfaces/ types
from moveit_wrapper.srv import MoveToPose, MoveToJointPosition, SetVelocity, String  #import the custom service interfaces from the wrapper package
from std_srvs.srv import Empty

# Affine py dependencies
from igus_moveit_clients.transform import Affine
from igus_moveit_clients.util import affine_to_pose


class ARMClient(Node):
     
    def __init__(self):
        super().__init__('arm_client_node')

        # init all needed clients (connect to the servers from moveit_wrapper package --> Wrap MoveIt C++ Interface to Python, because MoveIt Python API has only core capabilities)
        self.move_lin_cli = self.create_client(MoveToPose, "/move_to_pose_lin")    #connects to the services defined in the moveit_wrapper srvs
        while not self.move_lin_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_to_pose_lin service not available, waiting some more ...")
        self.get_logger().info("move_to_pose_lin service available")

        self.move_ptp_cli = self.create_client(MoveToPose, "/move_to_pose_ptp")
        while not self.move_ptp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_to_pose_ptp service not available, waiting some more ...")
        self.get_logger().info("move_to_pose_ptp service available")

        self.move_joint_cli = self.create_client(MoveToJointPosition, "/move_to_joint_position")
        while not self.move_joint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_to_joint_position service not available, waiting some more ...")
        self.get_logger().info("move_to_joint_position service available")

        self.reset_planning_group_cli = self.create_client(String, "/reset_planning_group")
        while not self.reset_planning_group_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("reset_planning_group service not available, waiting some more ...")
        self.get_logger().info("reset_planning_group service available")

        self.set_velocity_cli = self.create_client(SetVelocity, "/setVelocityScaling")
        while not self.set_velocity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("setVelocityScaling service not available, waiting some more ...")
        self.get_logger().info("setVelocityScaling service available")

        self.clear_octomap_cli = self.create_client(Empty, "/clear_octomap")
        while not self.clear_octomap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("clear_octomap service not available, waiting some more ...")
        self.get_logger().info("clear_octomap service available")

        self.home_position = [0.0,0.0,0.0,0.0,0.0,0.0]

    ##############################################################################################################################
    ##           this are the methods which can be called in your application which are communicating to the robot              ##
    ##############################################################################################################################
    def get_transform(self, from_frame_rel, to_frame_rel):
        """
        string to_frame_rel: name of the target frame to transform into
        string from_frame_rel: name of the source frame frame to transform from

        Returns:
        --------
        Affine: transformation matrix from robot base to target position and orientation (4x4 numpy array, can directly passed in motion planning)
        """
        # Calculate the transform between the robot base frame and the target frame
        counter = 0
        has_transform = False
        transform = None
        while (not has_transform) and counter < 10:
            rclpy.spin_once(self)
            counter += 1
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
                trans = trans.transform
                transform = Affine(
                    [trans.translation.x, trans.translation.y, trans.translation.z],
                    [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
                has_transform = self.tf_buffer.can_transform(to_frame_rel, from_frame_rel, now)
                print("Has transform: ", has_transform, counter)
            except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return None
        return transform
    

    def reset_planning_group(self, planning_group) -> bool:
        req = String.Request()
        req.data = planning_group
        future = ARMClient.send_service_request(req, self.reset_planning_group_cli)
        response = self.wait_for_service_response(future)
        return response.success

    def setVelocity(self, fraction) -> bool:
        req = SetVelocity.Request()
        req.velocity_scaling = fraction
        future = ARMClient.send_service_request(req, self.set_velocity_cli)
        response = self.wait_for_service_response(future)
        return response.success
    
    def home(self) -> bool:
        return self.ptp_joint(self.home_position)

    # planning in cartesian space, plans around obstacles and octomap
    def ptp(self, pose: Affine) -> bool:
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = ARMClient.send_service_request(req, self.move_ptp_cli)
        response = self.wait_for_service_response(future)
        return response.success

    # plan in joint_space, move to joint positions directly, but dont plan around obstacles
    def ptp_joint(self, joint_positions: List[float]) -> bool:
        req = MoveToJointPosition.Request()
        req.joint_position = joint_positions
        future = ARMClient.send_service_request(req, self.move_joint_cli)
        response = self.wait_for_service_response(future)
        return response.success

    # plan trajectory along cartesian lin path, but dont plan around obstacles
    def lin(self, pose: Affine) -> bool:
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = ARMClient.send_service_request(req, self.move_lin_cli)
        response = self.wait_for_service_response(future)
        return response.success


    # clear octomap in planning scene, only use current pointcloud for detecting collision geometry in the planning scene
    def clear_octomap(self) -> None:
        req = Empty.Request()
        future = ARMClient.send_service_request(req, self.clear_octomap_cli)
        self.wait_for_service_response(future)
        return





    # service calls for switching planner algorithm from ompl --> needs to be finetuned ! --> maybe introduce stomp as learning algorithm ???


    ##########################################################################################################################################################
    @staticmethod
    def send_service_request(request, client):
        future = client.call_async(request)
        return future
    
    @staticmethod
    def send_action_request(request, client):
        client.wait_for_server()
        future = client.send_goal_async(request)   # callback for ongoing feedback of action servers currently not implemented, buf could be handled here!

        return future

    def wait_for_service_response(self, future):
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response
                

    def wait_for_action_goal_acknowledgment(self, goal_future):
        # handle the process of sending the goal to the action server and receiving the server's acknowledgment
        # this future is completed when the action server processes the goal request
        while rclpy.ok():
            rclpy.spin_once(self)
            if goal_future.done():
                try:
                    goal_handle = goal_future.result()
                except Exception as e:
                    self.get_logger().info(f'Action goal send failed: {e}')
                    return None
                else:
                    if not goal_handle.accepted:
                        self.get_logger().info('Action goal rejected')
                    self.get_logger().info('Action goal accepted')
                    return goal_handle
                
    
    def wait_for_action_result(self, goal_handle):
        result_future = goal_handle.get_result_async()
        # used to handle the process of waiting for the action to complete and retrieving the final result
        # this future is completed when the action server finishes processing the action and sends back the final result
        while rclpy.ok():
            rclpy.spin_once(self)
            if result_future.done():
                try:
                    result = result_future.result()  # access result field from action definition
                except Exception as e:
                    self.get_logger().info(f'Action call failed: {e}')
                    return None
                else:
                    return result
                
    def destroy_node(self) -> None:
        self.destroy_node()
