import rclpy
import numpy as np
import time
from nav2_msgs.action import NavigateToPose


#import clients
from sew_agv_clients.agv import AGVClient


def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up node with agv clients
    agv = AGVClient()

    # set goal pose
    goal_msg = NavigateToPose.Goal() # goal field of action definition
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 0.0
    goal_msg.pose.pose.position.y = 0.0
    goal_msg.pose.pose.position.z = 0.0
    goal_msg.pose.pose.orientation.x = 0.0
    goal_msg.pose.pose.orientation.y = 0.0
    goal_msg.pose.pose.orientation.z = 0.0
    goal_msg.pose.pose.orientation.w = 0.0

    # call method from client class (string frameID, float64 pose[x,y,w])
    code = agv.check_nav_goal(goal_msg)
    print('collision checker return statement: ',code)


    # destroy the agv node, stop execution
    agv.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()

if __name__ == '__main__':
    main()