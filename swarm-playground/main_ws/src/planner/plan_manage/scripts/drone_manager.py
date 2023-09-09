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

def node_is_arrive(node_name):
    node_list = rosnode.get_node_names()
    # print(node_list)
    for n in node_list:
        # print(n, node_name, str(node_name) == str(n))
        if node_name in n:
            return True
    return False

def check_nodes(drone_id):
    node_list = [
        "/drone_"+str(drone_id)+"_ego_planner_node", 
        "/drone_"+str(drone_id)+"_odom_visualization", 
        "/drone_"+str(drone_id)+"_pcl_render_node", 
        "/drone_"+str(drone_id)+"_poscmd_2_odom", 
        "/drone_"+str(drone_id)+"_traj_server"
    ]
    node_list_copy = list(node_list)
    for node_name in node_list:
        if node_is_arrive(node_name):
            node_list_copy.remove(node_name)
    if len(node_list_copy) == 0:
        print("All nodes are running")
        return True
    elif len(node_list_copy) > 0 and len(node_list_copy) < 5:
        print("Some nodes are not running")
        print(node_list_copy)
        kill_node(node_list)
        return False
    else:
        print("All nodes are not running")
        return False
    
def kill_node(node_list):
    for node in node_list:
        subprocess.call(['rosnode','kill',node])
    subprocess.call(['rosnode','cleanup','-y'])

def print_output(process, drone_id):
    for line in iter(process.stdout.readline, b''):
        print(line.decode())

def launch_node(drone_id):
    global prev_time
    print("launch node " + str(drone_id))
    prev_time = time.time()
    cmd = "roslaunch ego_planner drone.launch drone_id:="+str(drone_id)+" init_x:="+str(data_dict[str(drone_id)]["init_x"])+" init_y:="+str(data_dict[str(drone_id)]["init_y"])+" init_z:="+str(data_dict[str(drone_id)]["init_z"])
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    data_dict[str(drone_id)]["prev_time"] = time.time()
    return p

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone_id', type=int, default=0)
    args, unknown = parser.parse_known_args()
    rospy.init_node('drone_manager', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
        while not rospy.is_shutdown():
            if time.time() - prev_time > 5:
                if not check_nodes(args.drone_id):
                    process = launch_node(args.drone_id)
                    executor.submit(print_output, process, args.drone_id)
            rate.sleep()    