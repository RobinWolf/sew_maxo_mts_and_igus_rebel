import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class AGVClient(Node):

    def __init__(self):
        super().__init__('agv_client_node')

        # init all needed clients
        self.nav_to_pose_clinet = ActionClient(self, NavigateToPose, 'navigate_to_pose')    #navigate_to_pose (name of topic of type nav2_msgs/action/NavigateToPose
        while not self.nav_to_pose_clinet.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("nav_to_pose_clinet action not available, waiting some more ...")
        self.get_logger().info("nav_to_pose_clinet action available")


    def move_to_nav_goal(self, frameID, pose):
        """
        string frameID: frame where the pose is given in e.g. 'map'
        list pose [x,y,w]: position and quarternion angle (rad) of the goal

        Returns
        -------
        int error_code

        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frameID
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1] 
        goal_msg.pose.pose.position.w = pose[2]  

        goal_future = self.send_action_request(goal_msg, self.nav_to_pose_clinet)   # send goal to action server of initialized client
        goal_handle = self.wait_for_action_goal_acknowledgment(goal_future) # check goal if accepted and return object representing the state of the goal request
        acknowledgement = goal_handle.accepted

        result = self.wait_for_action_result(goal_handle)   # check if goal is reached/ action finished
        error_code = result.error_code

        return error_code
    

    def check_nav_goal(self, frameID, pose):
        """
        string frameID: frame where the pose is given in e.g. 'map'
        list pose [x,y,w]: position and quarternion angle (rad) of the goal

        Returns
        -------
        bool acknowledgement

        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frameID
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1] 
        goal_msg.pose.pose.position.w = pose[2]  

        goal_future = self.send_action_request(goal_msg, self.nav_to_pose_clinet)   # send goal to action server of initialized client
        goal_handle = self.wait_for_action_goal_acknowledgment(goal_future) # check goal if accepted and return object representing the state of the goal request
        acknowledgement = goal_handle.accepted

        return acknowledgement


##########################################################################################################################################################

    @staticmethod
    def send_service_request(request, client):
        future = client.call_async(request)
        return future
    
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
