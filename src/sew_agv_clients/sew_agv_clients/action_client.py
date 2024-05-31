import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class NavigateToPoseClient(Node):

    def __init__(self): #overwrites parents __init__
        super().__init__('navigate_to_pose_action_client') #keep inheritance of parents __init__
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')    #navigate_to_pose (name of topic of type nav2_msgs/action/NavigateToPose (oben importiert)

    def send_goal(self, frameID, pose): # sendet goal to server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frameID
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1] 
        goal_msg.pose.pose.position.z = pose[2]  

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)



def main(args=None):
    rclpy.init(args=args)

    navigation_client = NavigateToPoseClient()

    future = navigation_client.send_goal('map',(0.0,-2.0,0.0))

    rclpy.spin_until_future_complete(navigation_client, future)



if __name__ == '__main__':
    main()  