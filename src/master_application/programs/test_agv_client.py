import rclpy
import numpy as np
import time

#import clients
from sew_agv_clients.sew_agv_clients.agv import AGVClient


def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up node with agv clients
    agv = AGVClient()

    # call method from client class
    goal_accepted = agv.check_nav_goal('map',[1,1,0])
    print('goal accepted: ',goal_accepted)

    error_code = agv.move_to_nav_goal('map',[1,1,0])
    print('Navigation Return Error Code:', error_code)



if __name__ == '__main__':
    main()