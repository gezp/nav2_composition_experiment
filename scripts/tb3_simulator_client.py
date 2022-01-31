#!/usr/bin/python
# -*- coding: UTF-8 -*-

from utils import *
import sys

def main():
    ip = '127.0.0.1'
    if len(sys.argv) == 2:
        ip = sys.argv[1]
    while True:
        cmd = input('cmd:')
        tb3_simulator_client_request(cmd, ip = ip)

if __name__ == '__main__':
    main()