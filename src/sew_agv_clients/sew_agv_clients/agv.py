import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import ComputePathToPose


class AGVClient(Node):

    def __init__(self):
        super().__init__('agv_client_node')

        # init all needed clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')    # name: navigate_to_pose, type: nav2_msgs/action/NavigateToPose
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("nav_to_pose_client action not available, waiting some more ...")
        self.get_logger().info("nav_to_pose_client action available")
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')    # name: compute_path_to_pose, type: nav2_msgs/action/ComputePathToPose
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("compute_path_to_pose_client action not available, waiting some more ...")
        self.get_logger().info("compute_path_to_pose_client action available")


    def move_to_nav_goal(self, frameID, pose):
        """
        string frameID: frame where the pose is given in e.g. 'map'
        list pose [x,y,w]: position and quarternion angle (rad) of the goal (geometry_msgs/PoseStamped.msg)

        Returns
        -------
        int status (no further response message provided)

        """
        goal_msg = NavigateToPose.Goal() # goal field of acrion definition
        goal_msg.pose.header.frame_id = frameID # geometry_msgs/PoseStamped.msg
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1] 
        goal_msg.pose.pose.orientation.w = pose[2]  

        goal_future = self.send_action_request(goal_msg, self.nav_to_pose_client)   # send goal to action server of initialized client
        goal_handle = self.wait_for_action_goal_acknowledgment(goal_future) # check goal if accepted and return object representing the state of the goal request

        result = self.wait_for_action_result(goal_handle)   # check if goal is reached/ action finished
        status = result.status

        return status
    

    def check_nav_goal(self, frameID, pose): 
        """
        string frameID: frame where the pose is given in e.g. 'map'
        list pose [x,y,w]: position and quarternion angle (rad) of the goal

        Returns
        -------
        bool goal_ok
        """
        goal_ok = None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = frameID
        goal_msg.goal.pose.position.x = pose[0]
        goal_msg.goal.pose.position.y = pose[1] 
        goal_msg.goal.pose.orientation.w = pose[2]
        goal_msg.planner_id = "GridBased"  # default planner_id
        goal_msg.use_start = False  

        goal_future = self.send_action_request(goal_msg, self.compute_path_to_pose_client)   # send goal to action server of initialized client
        goal_handle = self.wait_for_action_goal_acknowledgment(goal_future) # check goal if accepted and return object representing the state of the goal request

        result = self.wait_for_action_result(goal_handle)   # check if goal is reached/ action finished

        if result.status == 6:  # status (6 = ERROR)
            self.get_logger().info('Collision of goal pose detected')
            goal_ok = False
        elif result.status == 4: # status (4 = SUCCESS)
            self.get_logger().info('Goal pose is reachable')
            goal_ok = True
        else:
            self.get_logger().info('Undefined error occured whil checking goal pose')

        return goal_ok


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

