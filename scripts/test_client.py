#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import time
import csv
import statistics

from utils import *

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
    result_dict = {}
    try:
        print('launch navigation system')
        launch_navigation(launch_params)
        print('initialize navigator')
        test_client_request(cmd = 'init navigator', ip = ip)
        # get process
        node_processes = get_nav2_processes()
        names = []
        for p in node_processes:
            names.append(p.info['name'])
        print('processes :', names)
        total_cpu = []
        total_memory = []
        total_rss = []
        total_pss = []
        total_uss = []
        # start navigation
        print('publish goal')
        test_client_request(cmd = 'publish goal', ip = ip)
        # init cpu
        for p in node_processes:
            p.cpu_percent()
        while True:
            # check
            respone = test_client_request(cmd = 'check goal', ip = ip)
            data = respone.split(',')
            if data[0] == 'success':
                result_dict['time'] = float(data[1])
                result_dict['cpu'] = statistics.mean(total_cpu)
                result_dict['memory'] = statistics.mean(total_memory)
                result_dict['rss'] = statistics.mean(total_rss)
                result_dict['pss'] = statistics.mean(total_pss)
                result_dict['uss'] = statistics.mean(total_uss)
                break
            elif data[0] == 'fail':
                result_dict = None
                break
            cpu = []
            memory = []
            rss = []
            pss = []
            uss = []
            for p in node_processes:
                cpu.append(p.cpu_percent())
                memory.append(p.memory_percent())
                mem_info = p.memory_full_info()
                rss.append(mem_info.rss/1024/1024)
                pss.append(mem_info.pss/1024/1024)
                uss.append(mem_info.uss/1024/1024)
            # print(cpu)
            # print(memory)
            total_cpu.append(sum(cpu))
            total_memory.append(sum(memory))
            total_rss.append(sum(rss))
            total_pss.append(sum(pss))
            total_uss.append(sum(uss))
            print('cpu:',total_cpu[-1],'memory:',total_memory[-1],',rss:',total_rss[-1],',pss:',total_pss[-1],',uss:',total_uss[-1])
            print('------------------------------------------------')
            time.sleep(1.0)
    except Exception as e:
        print(e)
    # kill process
    print('kill navigation and simulator')
    test_client_request(cmd = 'destroy navigator', ip = ip)
    time.sleep(1)
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
    time.sleep(1)
    # test cases
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
    total_stats = []
    f = open('result.txt','w')
    f.write('#'*80+' raw data:\n')
    for i in range(len(test_case_params)):
        time_list = []
        cpu_list = []
        memory_list = []
        rss_list = []
        pss_list = []
        uss_list = []
        # test 5 times
        for k in range(repeat_time):
            print('#'*20+' case %d, repeat %d'%(i+1, k+1))
            result = test_once(ip = ip, launch_params = test_case_params[i])
            time_list.append(result['time'])
            cpu_list.append(result['cpu'])
            memory_list.append(result['memory'])
            rss_list.append(result['rss'])
            pss_list.append(result['pss'])
            uss_list.append(result['uss'])
            print('result:', result)
        # data: time, cpu, memory, pss, uss
        f.write('# '+test_case_names[i]+'\n')
        f.write('time:'+str(time_list)+'\n')
        f.write('cpu:'+str(cpu_list)+'\n')
        f.write('memory:'+str(memory_list)+'\n')
        f.write('rss:'+str(rss_list)+'\n')
        f.write('pss:'+str(pss_list)+'\n')
        f.write('uss:'+str(uss_list)+'\n')
        time_stats = '%.4f±%.4f'%(statistics.mean(time_list), statistics.pstdev(time_list))
        cpu_stats = '%.4f±%.4f'%(statistics.mean(cpu_list), statistics.pstdev(cpu_list))
        memory_stats = '%.4f±%.4f'%(statistics.mean(memory_list), statistics.pstdev(memory_list))
        rss_stats = '%.4f±%.4f'%(statistics.mean(rss_list), statistics.pstdev(rss_list))
        pss_stats = '%.4f±%.4f'%(statistics.mean(pss_list), statistics.pstdev(pss_list))
        uss_stats = '%.4f±%.4f'%(statistics.mean(uss_list), statistics.pstdev(uss_list))
        total_stats.append([time_stats, cpu_stats, memory_stats, rss_stats, pss_stats, uss_stats])
    header = ['case', 'time(s)', 'cpu(%)', 'memory(%)', 'rss(MB)', 'pss(MB)', 'uss(MB)']
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
    f_csv = open('result.csv','w')
    csv_writer = csv.writer(f_csv)
    csv_writer.writerow(header)
    for i in range(len(test_case_names)):
        csv_writer.writerow([test_case_names[i]]+total_stats[i])
    f_csv.close()

if __name__ == '__main__':
    main()
