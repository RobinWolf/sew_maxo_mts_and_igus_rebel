# use the nav2 commander API to control tha navigation out of a python script
# 1: init commander, set initial pose
# 2: goal location and path generation
# 3: goal complete

# https://navigation.ros.org/commander_api/index.html --> webside is down currently
# --> STILL TO FIGURE OUT IF THIS METHODS CALL THE NAV2 BEHAVIOR TREE OR SUB-METHODS --> I WANT THE BEHAVOIR TREE!

from nav2.simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration

# initialize
rclpy.init()
navigator = BasicNavigator()

# set initial pose (0,0,0 0,0,0) is spawningposition when use gazebo
init_pose = PoseStamped()
init_pose.header.frame_id = 'map'
init_pose.header.stamp = navigator.get_clock().now().to_msg()
init_pose.pose.position.x = 0.0
init_pose.pose.position.y = 0.0
init_pose.pose.orientation.w = 0.0

navigator.setInitialPose(init_pose)
navigator.waitUntilNav2Active() # waits till all nav2 lifecycle nodes are active


# calculate plan to goal pose
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 0.0
goal_pose.pose.orientation.w = 1.0

path = navigator.getPath(init_pose, goal_pose)
smoothed_path = navigator.smoothPath(path)


# move along path to goal pose
navigator.goToPose(goal_pose)
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    if feedback.navigation_duration > 200:
        navigator.cancelTask()


# get result after goal is reached
result = navigator.getResult()
if result == TaskResult.SUCCEEDED:
    print("Goal succeeded")
elif result == TaskResult.CANCELED:
    print("Goal canceled")
elif result == TaskResult.FAILED:
    print("Goal failed")