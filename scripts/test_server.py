import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

from utils import *

def main():
    port = 9999
    navigator = None
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(('0.0.0.0', port))
    BUFSIZE = 1024
    while True:
        try:
            data,client_addr = server.recvfrom(BUFSIZE)
            data = data.decode('utf-8')
            print('recv data:', data)
            if data == 'start simulator':
                run_launch_cmd(launch_file='tb3_simulator.launch.py')
                respone = 'success'
                server.sendto(respone.encode('utf-8'), client_addr)
            elif data == 'stop simulator':
                kill_simulator()
                respone = 'success'
                server.sendto(respone.encode('utf-8'), client_addr)
            elif data == 'init navigator':
                if not navigator:
                    rclpy.init()
                    navigator = BasicNavigator()
                    # Set our demo's initial pose
                    initial_pose = PoseStamped()
                    initial_pose.header.frame_id = 'map'
                    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                    initial_pose.pose.position.x = -2.0
                    initial_pose.pose.position.y = -0.5
                    initial_pose.pose.orientation.w = 1.0
                    navigator.setInitialPose(initial_pose)
                    # Wait for navigation to fully activate, since autostarting nav2
                    navigator.waitUntilNav2Active()
                    respone = 'success'
                else:
                    respone = 'fail'
                server.sendto(respone.encode('utf-8'), client_addr)
            elif data == 'destroy navigator':
                if navigator:
                    navigator.nav_through_poses_client.destroy()
                    navigator.nav_to_pose_client.destroy()
                    navigator.follow_waypoints_client.destroy()
                    navigator.compute_path_to_pose_client.destroy()
                    navigator.compute_path_through_poses_client.destroy()
                    navigator.destroy_node()
                    rclpy.shutdown()
                navigator = None
                respone = 'success'
                server.sendto(respone.encode('utf-8'), client_addr)
            elif data == 'publish goal':
                # Go to our demos first goal pose
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = 1.4
                goal_pose.pose.position.y = -1.6
                goal_pose.pose.orientation.w = 1.0
                navigator.goToPose(goal_pose)
                start_time = time.time()
                respone = 'success'
                server.sendto(respone.encode('utf-8'), client_addr)
            elif data == 'check goal':
                respone = 'continue'
                if navigator.isNavComplete():
                    result = navigator.getResult()
                    if result == NavigationResult.SUCCEEDED:
                        print('Goal succeeded!')
                    elif result == NavigationResult.CANCELED:
                        print('Goal was canceled!')
                    elif result == NavigationResult.FAILED:
                        print('Goal failed!')
                    else:
                        print('Goal has an invalid return status!')
                    # send result
                    nav_time = time.time() - start_time
                    if result == NavigationResult.SUCCEEDED:
                        respone = 'success,%f'%nav_time
                    else:
                        respone = 'fail'
                else:
                    feedback = navigator.getFeedback()
                    if feedback:
                        print('Estimated time of arrival: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                            + ' seconds.') 
                        # Some navigation timeout to demo cancellation
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            navigator.cancelNav()
                server.sendto(respone.encode('utf-8'), client_addr)
            else:
                print('invalid cmd')
        except:
            break
    server.close()
    print('exit')
    kill_simulator()

if __name__ == '__main__':
    main()
