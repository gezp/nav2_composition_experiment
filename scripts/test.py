#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import time
import csv
import statistics
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

from utils import *

def nav_case():
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
    # process
    node_processes = get_nav2_processes()
    names = []
    for p in node_processes:
        names.append(p.info['name'])
    print('processes :', names)
    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.4
    goal_pose.pose.position.y = -1.6
    goal_pose.pose.orientation.w = 1.0
    navigator.goToPose(goal_pose)
    total_cpu = []
    total_memory = []
    total_rss = []
    total_pss = []
    total_vms = []
    start_time = time.time()
    while not navigator.isNavComplete():
        # Do something with the feedback
        feedback = navigator.getFeedback()
        if feedback:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                + ' seconds.') 
            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()
        cpu = []
        memory = []
        rss = []
        pss = []
        vms = []
        for p in node_processes:
            cpu.append(p.cpu_percent())
            memory.append(p.memory_percent())
            mem_info = p.memory_full_info()
            rss.append(mem_info.rss/1024/1024)
            pss.append(mem_info.pss/1024/1024)
            vms.append(mem_info.vms/1024/1024)
        # print(cpu)
        # print(memory)
        total_cpu.append(sum(cpu))
        total_memory.append(sum(memory))
        total_rss.append(sum(rss))
        total_pss.append(sum(pss))
        total_vms.append(sum(vms))
        print('cpu:',total_cpu[-1],'memory:',total_memory[-1],',rss:',total_rss[-1],',pss:',total_pss[-1],',vms:',total_vms[-1])
        print('------------------------------------------------')
        time.sleep(0.2)
    # Do something depending on the return code
    nav_time = time.time() - start_time
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!') 
    if result != NavigationResult.SUCCEEDED:
        raise RuntimeError('Navigation was failed')
    # navigator.lifecycleShutdown()
    navigator.nav_through_poses_client.destroy()
    navigator.nav_to_pose_client.destroy()
    navigator.follow_waypoints_client.destroy()
    navigator.compute_path_to_pose_client.destroy()
    navigator.compute_path_through_poses_client.destroy()
    navigator.destroy_node()
    rclpy.shutdown()
    # result
    result_dict = {}
    result_dict['time'] = nav_time
    result_dict['cpu'] = statistics.mean(total_cpu)
    result_dict['memory'] = statistics.mean(total_memory)
    result_dict['rss'] = statistics.mean(total_rss)
    result_dict['pss'] = statistics.mean(total_pss)
    result_dict['vms'] = statistics.mean(total_vms)
    return result_dict

def test_once(ip, launch_params):
    tb3_simulator_client_request('start', ip = ip)
    time.sleep(5)
    test_result = None
    try:
        launch_navigation(launch_params)
        test_result = nav_case()
    except Exception as e:
        print(e)
    # kill process
    kill_navigation()
    tb3_simulator_client_request('stop', ip = ip)
    time.sleep(1)
    return test_result

def main():
    # init
    ip = '127.0.0.1'
    if len(sys.argv) == 2:
        ip = sys.argv[1]
    tb3_simulator_client_request('stop', ip = ip)
    time.sleep(1)
    # test cases
    test_case_names = [
        'normal',
        'dynamic composition with component_container_isolated',
        'dynamic composition with component_container_mt',
        'manual composition'
    ]
    test_case_params = [
        {'composition_type':'none'},
        {'composition_type':'dynamic', 'container_type':'component_container_isolated'},
        {'composition_type':'dynamic', 'container_type':'component_container_mt'},
        {'composition_type':'manual'}
    ]
    total_stats = []
    f = open('result.txt','w')
    f.write('#'*80+' raw data:\n')
    for i in range(len(test_case_params)):
        time_list = []
        cpu_list = []
        memory_list = []
        rss_list = []
        pss_list = []
        vms_list = []
        # test 5 times
        for k in range(5):
            result = test_once(ip = ip, launch_params = test_case_params[i])
            time_list.append(result['time'])
            cpu_list.append(result['cpu'])
            memory_list.append(result['memory'])
            rss_list.append(result['rss'])
            pss_list.append(result['pss'])
            vms_list.append(result['vms'])
            print('result:', result)
        # data: time, cpu, memory, pss, vms
        f.write('# '+test_case_names[i]+'\n')
        f.write('time:'+str(time_list)+'\n')
        f.write('cpu:'+str(cpu_list)+'\n')
        f.write('memory:'+str(memory_list)+'\n')
        f.write('rss:'+str(rss_list)+'\n')
        f.write('pss:'+str(pss_list)+'\n')
        f.write('vms:'+str(vms_list)+'\n')
        time_stats = '%.4f±%.4f'%(statistics.mean(time_list), statistics.pstdev(time_list))
        cpu_stats = '%.4f±%.4f'%(statistics.mean(cpu_list), statistics.pstdev(cpu_list))
        memory_stats = '%.4f±%.4f'%(statistics.mean(memory_list), statistics.pstdev(memory_list))
        rss_stats = '%.4f±%.4f'%(statistics.mean(rss_list), statistics.pstdev(rss_list))
        pss_stats = '%.4f±%.4f'%(statistics.mean(pss_list), statistics.pstdev(pss_list))
        vms_stats = '%.4f±%.4f'%(statistics.mean(vms_list), statistics.pstdev(vms_list))
        total_stats.append([time_stats, cpu_stats, memory_stats, rss_stats, pss_stats, vms_stats])
    f.close()
    # store statistics data in csv
    header = ['case', 'time(s)', 'cpu(%)', 'memory(%)', 'rss(MB)', 'pss(MB)', 'vms(MB)']
    f_csv = open('result.csv','w')
    csv_writer = csv.writer(f_csv)
    csv_writer.writerow(header)
    for i in range(len(test_case_names)):
        csv_writer.writerow([test_case_names[i]]+total_stats[i])
    f_csv.close()


if __name__ == '__main__':
    main()
