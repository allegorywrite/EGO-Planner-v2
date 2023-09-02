import rospy
import argparse
import subprocess
import time
import rosnode
from concurrent.futures import ThreadPoolExecutor
import os

data_dict = {
    "0":{"init_x":-15.0, "init_y":-9.0, "init_z":0.1},
    "1":{"init_x":-15.0, "init_y":-7.0, "init_z":0.1},
    "2":{"init_x":-15.0, "init_y":-5.0, "init_z":0.1},
    "3":{"init_x":-15.0, "init_y":-3.0, "init_z":0.1},
    "4":{"init_x":-15.0, "init_y":-1.0, "init_z":0.1},
    "5":{"init_x":-15.0, "init_y":1.0, "init_z":0.1},
    "6":{"init_x":-15.0, "init_y":3.0, "init_z":0.1},
    "7":{"init_x":-15.0, "init_y":5.0, "init_z":0.1},
    "8":{"init_x":-15.0, "init_y":7.0, "init_z":0.1},
    "9":{"init_x":-15.0, "init_y":9.0, "init_z":0.1},
    "10":{"init_x":-15.0, "init_y":-9.0, "init_z":1.0},
    "11":{"init_x":-15.0, "init_y":-7.0, "init_z":1.0},
    "12":{"init_x":-15.0, "init_y":-5.0, "init_z":1.0},
    "13":{"init_x":-15.0, "init_y":-3.0, "init_z":1.0},
    "14":{"init_x":-15.0, "init_y":-1.0, "init_z":1.0},
    "15":{"init_x":-15.0, "init_y":1.0, "init_z":1.0},
    "16":{"init_x":-15.0, "init_y":3.0, "init_z":1.0},
    "17":{"init_x":-15.0, "init_y":5.0, "init_z":1.0},
    "18":{"init_x":-15.0, "init_y":7.0, "init_z":1.0},
    "19":{"init_x":-15.0, "init_y":9.0, "init_z":1.0},
}

prev_time = time.time()

def check_node(node_name):
    node_list = rosnode.get_node_names()
    for n in node_list:
        if node_name in n:
            return True
    return False

def print_output(process, drone_id):
    for line in iter(process.stdout.readline, b''):
        print(line.decode())

def launch_node(drone_id):
    print("launch node " + str(drone_id))
    prev_time = time.time()
    cmd = "roslaunch ego_planner drone.launch drone_id:="+str(drone_id)+" init_x:="+str(data_dict[str(drone_id)]["init_x"])+" init_y:="+str(data_dict[str(drone_id)]["init_y"])+" init_z:="+str(data_dict[str(drone_id)]["init_z"])
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    data_dict[str(drone_id)]["prev_time"] = time.time()
    return p

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone_id', type=int, default=0)
    args = parser.parse_args()
    rospy.init_node('drone_manager', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
        while not rospy.is_shutdown():
            node_name = "/drone_" + str(args.drone_id) + "_ego_planner_node"
            if not check_node(node_name) and time.time() - data_dict[str(args.drone_id)]["prev_time"] > 5:
                rospy.logwarn("Node %s does not exist" % node_name)
                process = launch_node(args.drone_id)
                executor.submit(print_output, process, args.drone_id)
            rate.sleep()    