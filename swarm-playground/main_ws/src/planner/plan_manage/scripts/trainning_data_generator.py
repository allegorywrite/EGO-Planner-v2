#!/usr/bin/env python

import rospy
import numpy as np
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import argparse
from quadrotor_msgs.msg import GoalSet
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing import Process
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class DroneDataGenerator:
    def __init__(self, total_drones, visualize):
        self.total_drones = total_drones
        self.dim_per_drone = 13
        self.prev_time = 0
        self.time_interval = 0.05   # 100Hz
        self.agents_reached_goal = [False] * total_drones
        # self.snapshot = np.empty(shape=(total_drones, self.dim_per_drone))
        self.snapshot = [None] * total_drones
        self.snapshooted = [False] * total_drones
        # self.data_seq = np.empty(shape=(self.total_drones * self.dim_per_drone + 1, 0))
        self.data_seq = []
        self.obstacle_cloud_seq = []
        self.goal_pub = rospy.Publisher("/data_generator/goal_with_id", GoalSet, queue_size=10)
        self.goal_generated = False
        self.visualize = visualize
        self.time_goal_generated = rospy.Time.now()
        self.waypoint_safe_radius_A2O = 0.1
        self.waypoint_safe_radius_A2A = 0.5
        self.waypoint_array = np.empty(shape=(0, 3))
        self.obstacle_cloud = [None] * self.total_drones

        self.make_cloud_array()

        for i in range(0, self.total_drones):
          rospy.Subscriber("drone_{}_visual_slam/odom".format(i), Odometry, self.odom_callback, i)
          rospy.Subscriber("drone_{}_planning/reach_goal".format(i), Bool, self.goal_callback, i)
          rospy.Subscriber("/drone_{}_ego_planner_node/grid_map/occupancy".format(i), PointCloud2, self.inflate_callback, i)

        print("Data generator initialized")

    def addsnapshoot(self, msg_time):
        # row = np.array([msg_time])
        row = [msg_time]
        for i in range(0, len(self.snapshot)):
            # print("row:",row,"position:",self.snapshot[i])
            row.append(self.snapshot[i])
            # row = np.concatenate((row, self.snapshot[i]))
        print("Data seq shape:", self.data_seq.shape, "in:", row.shape)
        self.data_seq.append(row)
        # self.data_seq = np.hstack([self.data_seq, np.reshape(row, (-1,1))])
        print("Data seq shape:", self.data_seq.shape)
        self.obstacle_cloud_seq.append([msg_time, self.obstacle_cloud])
        print("Obstacle cloud seq shape:", len(self.obstacle_cloud_seq))
        # print("data_seq:",self.data_seq, self.data_seq[0][1])

    def odom_callback(self, data, drone_id):
        msg_time = data.header.stamp.to_sec()
        if(msg_time - self.prev_time > self.time_interval and self.goal_generated):
            # if self.prev_time != 0:
            #     if msg_time - self.prev_time > 2*self.time_interval:
            #         print("Error: time interval is too large")
            #     if msg_time - self.prev_time < 0:
            #         print("Error: invalid time interval")

            position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                                 data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w,
                                 data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z,
                                 data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])
            self.snapshot[drone_id] = position
            self.snapshooted[drone_id] = True
            if(all(self.snapshooted)):
                self.snapshooted = [False] * self.total_drones
                self.prev_time = msg_time
                self.addsnapshoot(msg_time)

    def goal_callback(self, data, drone_id):
        print("Drone {} reached goal".format(drone_id))
        if(not self.goal_generated):
            return
        self.agents_reached_goal[drone_id] = data.data
        if(all(self.agents_reached_goal)):
            self.goal_generated = False
            self.save_data()
            self.reset()

    def make_cloud_array(self):
        data = rospy.wait_for_message("/map_generator/global_cloud", PointCloud2, timeout=None)
        # PointCloud2 to array
        gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        self.cloud_arr = np.array(list(gen))

    def is_occupied(self, x, y, z):
        # Create arrays for lower and upper bounds
        lower_bound_A2O = [x - self.waypoint_safe_radius_A2O, y - self.waypoint_safe_radius_A2O, z - self.waypoint_safe_radius_A2O]
        upper_bound_A2O = [x + self.waypoint_safe_radius_A2O, y + self.waypoint_safe_radius_A2O, z + self.waypoint_safe_radius_A2O]
        lower_bound_A2A = [x - self.waypoint_safe_radius_A2A, y - self.waypoint_safe_radius_A2A, z - self.waypoint_safe_radius_A2A]
        upper_bound_A2A = [x + self.waypoint_safe_radius_A2A, y + self.waypoint_safe_radius_A2A, z + self.waypoint_safe_radius_A2A]
        # Check if there are any points within the specified bounds
        A2O_occupied = np.any((self.cloud_arr >= lower_bound_A2O).all(axis=1) & (self.cloud_arr <= upper_bound_A2O).all(axis=1))
        A2A_occupied = np.any((self.waypoint_array >= lower_bound_A2A).all(axis=1) & (self.waypoint_array <= upper_bound_A2A).all(axis=1))
        return A2O_occupied or A2A_occupied
    
    def inflate_callback(self, data, drone_id):
        point_cloud = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        # print("droneid:{}, pointcloud:{}".format(drone_id, len(point_cloud)))
        self.obstacle_cloud[drone_id] = point_cloud
        # print("droneid:{}, pointcloud:{}".format(drone_id, len(self.obstacle_cloud[drone_id])))
        
    def generate_goal(self):
        for i in range(0, self.total_drones):
            goal_msg = GoalSet()
            while(True):
                x = np.random.rand() * 20 - 10
                y = np.random.rand() * 20 - 10
                z = np.random.rand() * 2
                if(not self.is_occupied(x, y, z)):
                    goal_msg.drone_id = i
                    goal_msg.goal[0] = x
                    goal_msg.goal[1] = y
                    goal_msg.goal[2] = z
                    self.waypoint_array = np.vstack([self.waypoint_array, np.array([x,y,z])])
                    print("Drone {} goal: {}".format(i, goal_msg.goal))        
                    break
                else:
                    print("Drone {} goal is occupied".format(i))
            self.goal_pub.publish(goal_msg)
            self.goal_pub.publish(goal_msg)
        self.goal_generated = True
        self.time_goal_generated = rospy.Time.now()
        print("Goal generated")

    def save_data(self):
        obstacle_cloud_array = np.array(self.obstacle_cloud_seq, dtype=object)
        print(len(self.obstacle_cloud_seq), len(obstacle_cloud_array))
        print("save data of shape, replay", len(self.data_seq), "map", len(obstacle_cloud_array))
        random_number = np.random.randint(10000000, 99999999)
        replay_file_path = os.path.expanduser("~/polka_dot/data/training/replay/agents{}_{}.npy".format(self.total_drones, random_number))
        map_file_path = os.path.expanduser("~/polka_dot/data/training/map/agents{}_{}.npy".format(self.total_drones, random_number))
        os.makedirs(os.path.dirname(replay_file_path), exist_ok=True)
        os.makedirs(os.path.dirname(map_file_path), exist_ok=True)
        np.save(replay_file_path, self.data_seq)
        np.save(map_file_path, obstacle_cloud_array)
        if self.visualize:
            self.visualize_data(replay_file_path)
    
    def visualize_data(self, file_path):
        # matplotlibを使って3Dプロット
        data = np.load(file_path)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        for i in range(0, self.total_drones):
            ax.plot(data[1+i*self.dim_per_drone], data[2+i*self.dim_per_drone], data[3+i*self.dim_per_drone], label="drone{}".format(i))
        plt.legend()
        plt.show()

    def is_proceeding(self):
        return self.agents_reached_goal.count(True) < self.total_drones *0.8
    
    def reset(self):
        self.agents_reached_goal = [False] * self.total_drones
        self.snapshooted = [False] * self.total_drones
        self.goal_generated = False
        self.prev_time = 0
        np.empty(shape=(self.total_drones, self.dim_per_drone))
        self.data_seq = np.empty(shape=(self.total_drones * self.dim_per_drone + 1, 0))
        self.obstacle_cloud_seq = []
        self.time_goal_generated = rospy.Time.now()
        self.obstacle_cloud = [None] * self.total_drones
        self.waypoint_array = np.empty(shape=(0, 3))

# def execute_node(args):
#     rospy.init_node("drone_data_generator", anonymous=True)

#     drone_data_saver = DroneDataGenerator(args.total_drones, args.visualize)
#     # １秒待つ
#     rospy.sleep(1)
#     drone_data_saver.generate_goal()

#     rospy.spin()

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--total_drones', type=int, default=3)
#     parser.add_argument('--visualize', action='store_true')
#     args = parser.parse_args()
    
#     # Set the timeout in seconds
#     timeout = 10 # 5 minutes

#     # Create the process
#     p = Process(target=execute_node, args=(args,))

#     p.start()

#     # Allow the process to finish or kill it after the timeout
#     p.join(timeout)

#     if p.is_alive():
#         print("The process did not finish on time.")
#         p.terminate()
#         p.join()


if __name__ == '__main__':
    iteration_num = 10
    time_out = 60 # 60 seconds
    parser = argparse.ArgumentParser()
    parser.add_argument('--total_drones', type=int, default=3)
    parser.add_argument('--visualize', action='store_true')
    args = parser.parse_args()

    rospy.init_node("drone_data_generator", anonymous=True)

    drone_data_saver = DroneDataGenerator(args.total_drones, args.visualize)
    rospy.sleep(1)

    for itr in range(0, iteration_num):
        print("Iteration:{}".format(itr))
        drone_data_saver.generate_goal()
        while not rospy.is_shutdown():
            if not drone_data_saver.is_proceeding():
                print("Predetermined amount of agent reached goal")
                drone_data_saver.save_data()
                drone_data_saver.reset()
                break
            if(rospy.Time.now() - drone_data_saver.time_goal_generated > rospy.Duration(time_out)):
                print("Timeout")
                drone_data_saver.save_data()
                drone_data_saver.reset()
                break
            rospy.sleep(1)

    # while drone_data_saver.is_proceeding() and not rospy.is_shutdown():
    #     rospy.sleep(1)
    #     if(rospy.Time.now() - init_time > rospy.Duration(time_out)):
    #         print("Timeout")
    #         drone_data_saver.save_data()
    #         break
    