import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class myActionClient(Node): #child class: myActionClient// parent class: Node -> alle methoden werden übernommen

    def __init__(self): #overwrites parents __init__
        super().__init__('my_navigation_action_client') #keep inheritance of parents __init__
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')    #navigate_to_pose (name of topic of type nav2_msgs/action/NavigateToPose (oben importiert)

    def send_goal(self, frameID, pose): # sendet goal to server, fordert action an
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frameID
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1] 
        goal_msg.pose.pose.position.z = pose[2]  

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)



def main(args=None):    # node gestartet, indem objekt initialisiert wird
    rclpy.init(args=args)

    action_client = myActionClient()

    future = action_client.send_goal('map',(0.0,-2.0,0.0))    ##Werte an send-goal methode übergeben

    rclpy.spin_until_future_complete(action_client, future)



if __name__ == '__main__':  #checkt, ob File direkt ausgeführt wird, oder als Module in ein anderes importiert wird
    main()   # --> wenn first Module in ein anderes Modul importiert wird, läuft dessen main-Methode nicht automatisch ab, erst wenn speziell aufgerufen!!!

#immer für Module, die in andere Module importiert werden
#wenn  file/ first module direkt ausgeführt wird, dann schriebt python __main__ automatisch auf die __name__ Variable
#wenn man das first module in ein weiteres module importiert, wird __name__ Variable mit Name des first Module beschrieben, __name__ vom weiteren Module = __main__



