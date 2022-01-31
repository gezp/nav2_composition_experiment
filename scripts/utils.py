#!/usr/bin/python3
import subprocess
import time
import socket
import psutil

def get_process(name):
    processes = []
    for p in psutil.process_iter(['name']):
        if p.info['name'] == name:
            processes.append(p)
    return processes

def kill_process(name):
    node_processes = get_process(name)
    for p in node_processes:
        p.kill()

def run_launch_cmd(launch_file,params={}):
    cmd = "ros2 launch nav2_composition_experiment " + launch_file
    for k,v in params.items():
        cmd += " "+k+":="+str(v)
    #print(cmd)
    subprocess.Popen([cmd],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,shell=True)

def kill_simulator():
    process_names = ['gzserver', 'gzclient',  'robot_state_publisher', 'rviz2']
    for name in process_names:
        kill_process(name)
    
def get_nav2_processes():
    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = ['controller_server', 'smoother_server', 'planner_server',
        'recoveries_server', 'bt_navigator', 'waypoint_follower']
    composed_nodes = ['composed_bringup', 'component_container_mt', 'component_container_isolated']
    process_names = localization_nodes + navigation_nodes + ['lifecycle_manager'] + composed_nodes
    node_processes = []
    for name in process_names:
        node_processes.extend(get_process(name))
    return node_processes

def launch_navigation(launch_params):
    composition_type = launch_params['composition_type']
    if composition_type not in ['none', 'dynamic', 'manual']:
        print('invalid composition_type:',composition_type)
        raise RuntimeError('Failed to launch navigation') 
    if composition_type == 'none':
        run_launch_cmd(launch_file='bringup.launch.py',
                       params={'use_composition':False})
    elif composition_type == 'dynamic':
        container_type = launch_params['container_type']
        if container_type not in ['component_container_isolated', 'component_container_mt']:
            print('invalid container_type:', container_type)
            raise RuntimeError('Failed to launch navigation')  
        run_launch_cmd(launch_file='bringup.launch.py',
                       params={'use_composition':True, 
                               'container_type':container_type})
    elif composition_type == 'manual':
        run_launch_cmd(launch_file='composed_bringup.launch.py')

def kill_navigation():
    node_processes = get_nav2_processes()
    for p in node_processes:
        p.kill()

def tb3_simulator_server(port = 9999):
    BUFSIZE = 1024
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(('0.0.0.0', port))
    while True:
        try:
            data,client_addr = server.recvfrom(BUFSIZE)
            if data == b'start':
                print('start')
                run_launch_cmd(launch_file='tb3_simulator.launch.py')
            elif data == b'stop':
                print('stop')
                kill_simulator()
            else:
                print('invalid cmd')
        except:
            break
    server.close()
    print('exit')
    kill_simulator()

def tb3_simulator_client_request(cmd, ip = '127.0.0.1', port = 9999):
    BUFSIZE = 1024
    client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    client.sendto(cmd.encode('utf-8'), (ip, port))
