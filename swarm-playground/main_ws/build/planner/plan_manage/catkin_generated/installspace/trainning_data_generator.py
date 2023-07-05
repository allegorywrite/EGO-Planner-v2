#!/usr/bin/env python3

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
        self.time_interval = 0.1   # 100Hz
        self.agents_reached_goal = [False] * total_drones
        self.snapshot = np.empty(shape=(total_drones, self.dim_per_drone))
        self.snapshooted = [False] * total_drones
        self.data_seq = np.empty(shape=(self.total_drones * self.dim_per_drone + 1, 0))
        self.goal_pub = rospy.Publisher("/data_generator/goal_with_id", GoalSet, queue_size=10)
        self.goal_generated = False
        self.visualize = visualize

        for i in range(0, self.total_drones):
          rospy.Subscriber("drone_{}_visual_slam/odom".format(i), Odometry, self.odom_callback, i)
          rospy.Subscriber("drone_{}_planning/reach_goal".format(i), Bool, self.goal_callback, i)
          rospy.Subscriber("/drone_{}_ego_planner_node/grid_map/occupancy_inflate".format(i), PointCloud2, self.inflate_callback, i)

        print("Data generator initialized")

    def odom_callback(self, data, drone_id):
        msg_time = data.header.stamp.to_sec()
        if(msg_time - self.prev_time > self.time_interval and self.goal_generated):
            if(self.prev_time != 0 and (msg_time - self.prev_time > 2*self.time_interval or msg_time - self.prev_time < 0)):
                # エラー処理
                print("Error: invalid asynchronus process")

            position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                                 data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w,
                                 data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z,
                                 data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])
            self.snapshot[drone_id] = position
            self.snapshooted[drone_id] = True
            if(all(self.snapshooted)):
                row = np.array([msg_time])
                for i in range(0, len(self.snapshot)):
                    # print("row:",row,"position:",self.snapshot[i])
                    row = np.concatenate((row, self.snapshot[i]))
                # print("msg_time:",msg_time,"row:",row)
                self.data_seq = np.hstack([self.data_seq, np.reshape(row, (-1,1))])
                # print("data_seq:",self.data_seq, self.data_seq[0][1])
                self.snapshooted = [False] * self.total_drones
                self.prev_time = msg_time

    def goal_callback(self, data, drone_id):
        print("Drone {} reached goal".format(drone_id))
        self.agents_reached_goal[drone_id] = data.data
        if(all(self.agents_reached_goal)):
            self.goal_generated = False
            self.save_data()
    
    def inflate_callback(self, data, drone_id):
        point_cloud = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        print("droneid:{}, pointcloud:{}".format(drone_id, len(point_cloud)))
        
    def generate_goal(self):
        for i in range(0, self.total_drones):
            goal_msg = GoalSet()
            goal_msg.drone_id = i
            goal_msg.goal[0] = np.random.rand() * 10 - 5
            goal_msg.goal[1] = np.random.rand() * 10 - 5
            goal_msg.goal[2] = np.random.rand()
            print("Drone {} goal: {}".format(i, goal_msg.goal))
            self.goal_pub.publish(goal_msg)
            self.goal_pub.publish(goal_msg)
        self.goal_generated = True
        print("Goal generated")

    def save_data(self):
        print("Saving data")
        random_number = np.random.randint(100000, 999999)
        file_path = os.path.expanduser("~/polka_dot/data/training/test/agents{}_{}.npy".format(self.total_drones, random_number))
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        np.save(file_path, self.data_seq)
        self.data_seq = np.empty(shape=(self.total_drones * self.dim_per_drone + 1, 0))
        if self.visualize:
            self.visualize_data(file_path)
    
    def visualize_data(self, file_path):
        # matplotlibを使って3Dプロット
        data = np.load(file_path)
        print("data shape:",data.shape)
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
    time_out = 60 # 60 seconds
    parser = argparse.ArgumentParser()
    parser.add_argument('--total_drones', type=int, default=3)
    parser.add_argument('--visualize', action='store_true')
    args = parser.parse_args()

    rospy.init_node("drone_data_generator", anonymous=True)
    init_time = rospy.Time.now()

    drone_data_saver = DroneDataGenerator(args.total_drones, args.visualize)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        drone_data_saver.generate_goal()
        rospy.sleep(1)

    # while drone_data_saver.is_proceeding() and not rospy.is_shutdown():
    #     rospy.sleep(1)
    #     if(rospy.Time.now() - init_time > rospy.Duration(time_out)):
    #         print("Timeout")
    #         drone_data_saver.save_data()
    #         break
    