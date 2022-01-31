import time
from utils import *

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
import rclpy
from rclpy.duration import Duration

def main():
    kill_simulator()
    kill_navigation()


if __name__ == '__main__':
    main()