#!/usr/bin/env python
import rospy
import subprocess
import argparse
import time
import rosnode
from concurrent.futures import ThreadPoolExecutor
import os

def kill_node(): 
    nodename_array = ["/drone_0_ego_planner_node", "/drone_0_odom_visualization", "/drone_0_pcl_render_node", "/drone_0_poscmd_2_odom", "/drone_0_traj_server"]
    p2=subprocess.Popen(['rosnode','list'],stdout=subprocess.PIPE) 
    p2.wait() 
    nodelist=p2.communicate() 
    nd=nodelist[0] 
    nd = nd.decode("utf-8") # Decoding bytes to a string
    nd=nd.split("\n") 
    
    # Print currently running nodes
    print("Currently running nodes:")
    for node in nd:
        print(str(node))
        print(str(node) in nodename_array)
        if str(node) in nodename_array:
            subprocess.call(['rosnode','kill',node]) 

init_time = time.time()

data_dict = {
    "0":{"init_x":-15.0, "init_y":-9.0, "init_z":0.1, "prev_time": init_time},
    "1":{"init_x":-15.0, "init_y":-7.0, "init_z":0.1, "prev_time": init_time},
    "2":{"init_x":-15.0, "init_y":-5.0, "init_z":0.1, "prev_time": init_time},
    "3":{"init_x":-15.0, "init_y":-3.0, "init_z":0.1, "prev_time": init_time},
    "4":{"init_x":-15.0, "init_y":-1.0, "init_z":0.1, "prev_time": init_time},
    "5":{"init_x":-15.0, "init_y":1.0, "init_z":0.1, "prev_time": init_time},
    "6":{"init_x":-15.0, "init_y":3.0, "init_z":0.1, "prev_time": init_time},
    "7":{"init_x":-15.0, "init_y":5.0, "init_z":0.1, "prev_time": init_time},
    "8":{"init_x":-15.0, "init_y":7.0, "init_z":0.1, "prev_time": init_time},
    "9":{"init_x":-15.0, "init_y":9.0, "init_z":0.1, "prev_time": init_time},
    "10":{"init_x":-15.0, "init_y":-9.0, "init_z":1.0, "prev_time": init_time},
    "11":{"init_x":-15.0, "init_y":-7.0, "init_z":1.0, "prev_time": init_time},
    "12":{"init_x":-15.0, "init_y":-5.0, "init_z":1.0, "prev_time": init_time},
    "13":{"init_x":-15.0, "init_y":-3.0, "init_z":1.0, "prev_time": init_time},
    "14":{"init_x":-15.0, "init_y":-1.0, "init_z":1.0, "prev_time": init_time},
    "15":{"init_x":-15.0, "init_y":1.0, "init_z":1.0, "prev_time": init_time},
    "16":{"init_x":-15.0, "init_y":3.0, "init_z":1.0, "prev_time": init_time},
    "17":{"init_x":-15.0, "init_y":5.0, "init_z":1.0, "prev_time": init_time},
    "18":{"init_x":-15.0, "init_y":7.0, "init_z":1.0, "prev_time": init_time},
    "19":{"init_x":-15.0, "init_y":9.0, "init_z":1.0, "prev_time": init_time},
}

drones_num = 3

def launch_node(drone_id):
    print("launch node " + str(drone_id))
    cmd = "roslaunch ego_planner drone.launch drone_id:="+str(drone_id)+" init_x:="+str(data_dict[str(drone_id)]["init_x"])+" init_y:="+str(data_dict[str(drone_id)]["init_y"])+" init_z:="+str(data_dict[str(drone_id)]["init_z"])
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    data_dict[str(drone_id)]["prev_time"] = time.time()
    return p

def check_node(node_name):
    node_list = rosnode.get_node_names()
    for n in node_list:
        if node_name in n:
            return True
    return False

def print_output(process, drone_id):
    for line in iter(process.stdout.readline, b''):
        print(line.decode())

def monitor_node(drones_num):
    rospy.init_node('node_monitor', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    p = subprocess.Popen(['roslaunch', 'ego_planner', 'random_forest.launch'], stdout=subprocess.PIPE)
    print("launch random forest")
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
        while not rospy.is_shutdown():
            for i in range(drones_num):
                node_name = "/drone_" + str(i) + "_ego_planner_node"
                if not check_node(node_name) and time.time() - data_dict[str(i)]["prev_time"] > 5:
                    rospy.logwarn("Node %s does not exist" % node_name)
                    process = launch_node(i)
                    executor.submit(print_output, process, i)
            rate.sleep()

if __name__ == '__main__':
    try:
        monitor_node(drones_num)
    except rospy.ROSInterruptException:
        pass