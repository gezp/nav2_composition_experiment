#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import time
import math
import csv
import statistics
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses, NavigateToPose

from utils import *

def check_zero_vel(vel:Twist):
    # print('recv', vel)
    epsilon = 0.000001
    if math.fabs(vel.linear.x) < epsilon and \
       math.fabs(vel.linear.y) < epsilon and \
       math.fabs(vel.angular.z) < epsilon:
        return True
    else:
        return False

def test_client_request(cmd, ip = '127.0.0.1', port = 9999):
    BUFSIZE = 1024
    client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    client.sendto(cmd.encode('utf-8'), (ip, port))
    data,server_addr = client.recvfrom(BUFSIZE)
    return data.decode('utf-8')

def test_once(ip, launch_params):
    print('start simulator')
    test_client_request(cmd = 'start simulator', ip = ip)
    time.sleep(2)
    delay = 100
    result_dict = {}
    try:
        rclpy.init()
        print('launch navigation system')
        launch_navigation(launch_params)
        # initialize listener node
        listener_node = Node('listener')
        vel = None
        def listener_callback(msg):
            nonlocal vel
            vel = msg
        subscription = listener_node.create_subscription(Twist, '/cmd_vel', listener_callback, 10)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(listener_node)
        # initialize navigator
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
        print('Waiting for NavigateToPose action server')
        while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            pass
        time.sleep(1.0)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.4
        goal_pose.pose.position.y = -1.6
        goal_pose.pose.orientation.w = 1.0
        print('publish goal')
        vel = None
        start_time = time.time()
        navigator.goToPose(goal_pose)
        # wait to start by checking velocity
        while True:
            executor.spin_once()
            if (vel is not None) and (not check_zero_vel(vel)):
                break
        dt = time.time() - start_time
        print('recieve non-zero velocity, time: ', dt)
        result_dict['start_time'] = dt
        print('run 5s...')
        time.sleep(5)
        print('cancel goal')
        vel = None
        start_time = time.time()
        navigator.cancelNav()
        # wait to stop by checking velocity
        start_time = time.time()
        while True:
            executor.spin_once()
            if vel is not None and check_zero_vel(vel):
                break
        dt = time.time() - start_time
        print('recieve zero velocity, time: ', dt)
        result_dict['cancel_delay'] = dt
        # destroy ros2 node 
        navigator.nav_through_poses_client.destroy()
        navigator.nav_to_pose_client.destroy()
        navigator.follow_waypoints_client.destroy()
        navigator.compute_path_to_pose_client.destroy()
        navigator.compute_path_through_poses_client.destroy()
        navigator.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(e)
    # kill process
    print('kill navigation and simulator')
    kill_navigation()
    time.sleep(1)
    test_client_request('stop simulator', ip = ip)
    time.sleep(1)
    return result_dict

def main():
    # init
    ip = '127.0.0.1'
    repeat_time = 10
    if len(sys.argv) == 2:
        ip = sys.argv[1]
    test_client_request('stop simulator', ip = ip)
    time.sleep(1)
    test_case_names = [
        'normal',
        'dynamic composition with component_container_isolated',
        'dynamic composition with component_container_isolated2',
        'dynamic composition with component_container_mt',
        'manual composition'
    ]
    test_case_params = [
        {'composition_type':'none'},
        {'composition_type':'dynamic', 'container_type':'component_container_isolated'},
        {'composition_type':'dynamic', 'container_type':'component_container_isolated2'},
        {'composition_type':'dynamic', 'container_type':'component_container_mt'},
        {'composition_type':'manual'}
    ]
    f = open('result2.txt','w')
    f.write('#'*80+' raw data:\n')
    total_stats = []
    for i in range(len(test_case_params)):
        start_time_list = []
        cancel_delay_list = []
        # test multiple times
        for k in range(repeat_time):
            print('#'*20+' case %d, repeat %d'%(i+1, k+1))
            result = test_once(ip = ip, launch_params = test_case_params[i])
            start_time_list.append(result['start_time'])
            cancel_delay_list.append(result['cancel_delay'])
            print('result:', result)
        f.write('# '+test_case_names[i]+'\n')
        f.write('start_time:'+str(start_time_list)+'\n')
        f.write('cancel_delay:'+str(cancel_delay_list)+'\n')
        start_time_stats = '%.4f±%.4f'%(statistics.mean(start_time_list), statistics.pstdev(start_time_list))
        cancel_delay_stats = '%.4f±%.4f'%(statistics.mean(cancel_delay_list), statistics.pstdev(cancel_delay_list))
        total_stats.append([start_time_stats, cancel_delay_stats])
        time.sleep(2)
    header = ['case', 'start time(s)', 'cancel delay(s)']
    # store statistics data in txt file
    f.write('#'*80+' statistics data:\n')
    row_str = ''
    for v in header:
        row_str = row_str + '%20s'%v
    f.write(row_str+'\n')
    for i in range(len(test_case_names)):
        row_str = '%20s'%('case '+str(i+1))
        for v in total_stats[i]:
            row_str = row_str + '%20s'%v
        f.write(row_str+'\n')
    f.close()
    # store statistics data in csv file
    f_csv = open('result2.csv','w')
    csv_writer = csv.writer(f_csv)
    csv_writer.writerow(header)
    for i in range(len(test_case_names)):
        csv_writer.writerow([test_case_names[i]]+total_stats[i])
    f_csv.close()

if __name__ == '__main__':
    main()
