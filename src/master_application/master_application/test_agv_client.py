import rclpy
import numpy as np
import time

#import clients
from sew_agv_clients.agv import AGVClient


def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up node with agv clients
    agv = AGVClient()

    # call method from client class (string frameID, float64 pose[x,y,w])
    goal_accepted = agv.check_nav_goal('map',[1.0,0.0,0.0])
    print('goal accepted: ',goal_accepted)

    error_code = agv.move_to_nav_goal('map',[1.0,0.0,0.0])
    print('Navigation Return Error Code:', error_code)

    # destroy the agv node, stop execution
    agv.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()

if __name__ == '__main__':
    main()