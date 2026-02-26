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
import sys
import time
import psutil

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    import rospy
    import rosnode
    try:
        from xmlrpc.client import ServerProxy
    except ImportError:
        from xmlrpclib import ServerProxy
elif ROS_VERSION == 2:
    import rclpy
else:
    print("Error: ROS_VERSION not detected. Please source your ROS environment.")
    sys.exit(-1)

def get_process_ros(node_name, doprint=False):
    if ROS_VERSION == 1:
        node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=True)[2]
        if not node_api:
            rospy.logwarn("could not get api of node %s (%s)" % (node_name, node_api))
            return False
        try:
            response = ServerProxy(node_api).getPid('/NODEINFO')
            process = psutil.Process(response[2])
            if doprint:
                rospy.loginfo("adding new node monitor %s (pid %d)" % (node_name, process.pid))
            return process
        except:
            rospy.logwarn("failed to get of the pid of ros node %s (%s)" % (node_name, node_api))
            return False
    else:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if node_name in proc.info['name'] or (proc.info['cmdline'] and any(node_name in s for s in proc.info['cmdline'])):
                    process = psutil.Process(proc.info['pid'])
                    if doprint:
                        print("[INFO] adding new node monitor %s (pid %d)" % (node_name, process.pid))
                    return process
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return False

if __name__ == '__main__':

    if ROS_VERSION == 1:
        rospy.init_node("pid_ros")
        if not rospy.has_param('~nodes') or not rospy.has_param('~output'):
            rospy.logerr("please specify the nodes and output file for this logger 1")
            rospy.logerr("rosrun ov_eval pid_ros.py _nodes:=<comma,separated,node,names> _output:=<file.txt>")
            sys.exit(-1)
        node_csv = rospy.get_param("~nodes")
        save_path = rospy.get_param("~output")
    else:
        rclpy.init()
        node = rclpy.create_node("pid_ros")
        args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in sys.argv if ':=' in arg}
        node_csv = args.get('_nodes')
        save_path = args.get('_output')
        
        if not node_csv or not save_path:
            print("[ERROR] please specify the nodes and output file for this logger 1")
            print("[ERROR] ros2 run ov_eval pid_ros.py --remap _nodes:=<names> _output:=<file.txt>")
            sys.exit(-1)

    node_list = node_csv.split(',')

    if ROS_VERSION == 1:
        rospy.loginfo("processes: %s (%d in total)" % (node_csv, len(node_list)))
        rospy.loginfo("save path: %s" % save_path)
    else:
        print("[INFO] processes: %s (%d in total)" % (node_csv, len(node_list)))
        print("[INFO] save path: %s" % save_path)

    if not os.path.exists(os.path.dirname(save_path)) and os.path.dirname(save_path) != '':
        try:
            os.makedirs(os.path.dirname(save_path))
        except:
            if ROS_VERSION == 1: rospy.logerr("unable to create the save path!!!!!")
            else: print("[ERROR] unable to create the save path!!!!!")
            sys.exit(-1)

    file = open(save_path, "w")

    header = "# timestamp(s) summed_cpu_perc summed_mem_perc summed_threads"
    for node_name in node_list:
        get_process_ros(node_name, True)  # nice debug print!
        header += " " + str(node_name) + "_cpu_perc " + str(node_name) + "_mem_perc " + str(node_name) + "_threads"
    header += "\n"
    file.write(header)

    def is_shutdown():
        return rospy.is_shutdown() if ROS_VERSION == 1 else not rclpy.ok()

    if is_shutdown():
        file.close()
        sys.exit(-1)

    while not is_shutdown():

        ps_list = []
        for node_name in node_list:
            ps_list.append(get_process_ros(node_name, False))
            try:
                idx = len(ps_list) - 1
                if ps_list[idx]:
                    ps_list[idx].cpu_percent(interval=None)
            except:
                continue

        time.sleep(1.0)

        perc_cpu = []
        perc_mem = []
        threads = []
        for i in range(0, len(node_list)):
            try:
                p_cpu = ps_list[i].cpu_percent(interval=None)
                p_mem = ps_list[i].memory_percent()
                p_threads = ps_list[i].num_threads()
                perc_cpu.append(p_cpu)
                perc_mem.append(p_mem)
                threads.append(p_threads)
            except:
                perc_cpu.append(0)
                perc_mem.append(0)
                threads.append(0)

        log_msg = "cpu%% = %.3f | mem%% = %.3f | threads = %d" % (sum(perc_cpu), sum(perc_mem), sum(threads))
        if ROS_VERSION == 1: rospy.loginfo(log_msg)
        else: print("[INFO] %s" % log_msg)

        data = "%.8f %.3f %.3f %d" % (time.time(), sum(perc_cpu), sum(perc_mem), sum(threads))
        for i in range(0, len(node_list)):
            data += " %.3f %.3f %d" % (perc_cpu[i], perc_mem[i], threads[i])
        data += "\n"
        file.write(data)
        file.flush()

    file.close()
    if ROS_VERSION == 2: rclpy.shutdown()
