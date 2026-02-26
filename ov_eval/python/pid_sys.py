#!/usr/bin/env python

# OpenVINS: An Open Platform for Visual-Inertial Research
# Copyright (C) 2019 Patrick Geneva
# Copyright (C) 2019 OpenVINS Contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  See <https://www.gnu.org/licenses/>.

import os
import psutil
import sys
import time

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    import rospy
elif ROS_VERSION == 2:
    import rclpy
else:
    print("Error: ROS_VERSION not detected. Please source your ROS environment.")
    sys.exit(-1)

def log_info(msg):
    if ROS_VERSION == 1: rospy.loginfo(msg)
    else: print("[INFO] %s" % msg)

def log_err(msg):
    if ROS_VERSION == 1: rospy.logerr(msg)
    else: print("[ERROR] %s" % msg)

def is_shutdown():
    return rospy.is_shutdown() if ROS_VERSION == 1 else not rclpy.ok()

def get_process_name(process_name, doprint=False):
    processes = []
    for proc in psutil.process_iter():
        name, exe, cmdline = "", "", []
        try:
            name = proc.name()
            cmdline = proc.cmdline()
            exe = proc.exe()
        except (psutil.AccessDenied, psutil.ZombieProcess):
            pass
        except psutil.NoSuchProcess:
            continue
        
        match = False
        if name == process_name:
            match = True
        elif len(cmdline) > 0 and cmdline[0] == process_name:
            match = True
        elif exe and os.path.basename(exe) == process_name:
            match = True

        if match:
            if doprint:
                log_info("adding new node monitor (pid %d)" % (proc.pid))
            processes.append(proc)
            
    if len(processes) > 0:
        return processes
    log_err("unable to find process for %s" % (process_name))
    return False

if __name__ == '__main__':

    if ROS_VERSION == 1:
        rospy.init_node("pid_sys")
        rate_val = 2
        sleep_func = rospy.Rate(rate_val).sleep
    else:
        rclpy.init()
        node = rclpy.create_node("pid_sys")
        sleep_func = lambda: time.sleep(0.5)

    if len(sys.argv) < 2:
        log_err("please specify process name")
        log_err("python pid_sys.py <command-name>")
        if ROS_VERSION == 2: rclpy.shutdown()
        sys.exit(-1)

    processes = False
    while processes == False and not is_shutdown():
        processes = get_process_name(sys.argv[1], True)
        if processes == False:
            sleep_func()

    if is_shutdown():
        if ROS_VERSION == 2: rclpy.shutdown()
        sys.exit(-1)

    while not is_shutdown():
        sum_perc_cpu = 0.0
        sum_perc_mem = 0.0
        sum_threads = 0
        for p in processes:
            try:
                perc_cpu = p.cpu_percent(interval=None)
                perc_mem = p.memory_percent()
                threads = p.num_threads()
            except:
                continue
            sum_perc_cpu += perc_cpu
            sum_perc_mem += perc_mem
            sum_threads += threads

        print("cpu percent = %.3f" % sum_perc_cpu)
        print("mem percent = %.3f" % sum_perc_mem)
        print("num threads = %d" % sum_threads)
        processes = False

        while processes == False and not is_shutdown():
            processes = get_process_name(sys.argv[1])
            if not processes == False:
                for p in processes:
                    try:
                        p.cpu_percent(interval=None)
                    except:
                        continue
            sleep_func()

    if ROS_VERSION == 2: rclpy.shutdown()
