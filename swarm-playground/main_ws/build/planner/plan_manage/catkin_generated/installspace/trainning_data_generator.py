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
import tf

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
        plane_array = []
        self.obstacle_cloud = [plane_array] * self.total_drones
        self.make_cloud_array()
        self.tf_listener = tf.TransformListener()

        for i in range(0, self.total_drones):
          rospy.Subscriber("drone_{}_visual_slam/odom".format(i), Odometry, self.odom_callback, i)
          rospy.Subscriber("drone_{}_planning/reach_goal".format(i), Bool, self.goal_callback, i)
          rospy.Subscriber("/drone_{}_ego_planner_node/grid_map/occupancy".format(i), PointCloud2, self.inflate_callback, i)
        print("Data generator initialized")

    def addsnapshoot(self, msg_time):
        # print("Adding snapshoot")
        row = [msg_time]
        # print("msg_time: {}".format(msg_time))
        for i in range(0, len(self.snapshot)):
            row.append(self.snapshot[i])
            # print("map time of drone{}: {}".format(i, self.obstacle_cloud[i][-1][0]))
        self.data_seq.append(row)

    def odom_callback(self, data, drone_id):
        msg_time = data.header.stamp.to_sec()
        if(msg_time - self.prev_time > self.time_interval and self.goal_generated):
            # if self.prev_time != 0:
            #     if msg_time - self.prev_time > 2*self.time_interval:
            #         print("Error: time interval is too large")
            #     if msg_time - self.prev_time < 0:
            #         print("Error: invalid time interval")
            position = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                                 data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w,
                                 data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z,
                                 data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]
            self.snapshot[drone_id] = position
            self.snapshooted[drone_id] = True
            if(all(self.snapshooted) and self.goal_generated):
                self.snapshooted = [False] * self.total_drones
                self.prev_time = msg_time
                self.addsnapshoot(msg_time)

    def goal_callback(self, data, drone_id):
        # frames = self.tf_listener.getFrameStrings()
        # print(frames)
        if(not self.goal_generated):
            return
        print("Drone {} reached goal".format(drone_id))
        self.agents_reached_goal[drone_id] = data.data
        if(all(self.agents_reached_goal)):
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
        print("Inflate callback of drone {}".format(drone_id))   
        point_cloud = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        self.obstacle_cloud[drone_id].append([data.header.stamp.to_sec(), point_cloud])

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
        self.agents_reached_goal = [False] * self.total_drones
        self.goal_generated = False
        obstacle_cloud_seq = []
        replay_data = np.array(self.data_seq, dtype=object)
        if(self.obstacle_cloud[0] == self.obstacle_cloud[1]):
            print("Error")
        for t in range(0, replay_data.shape[0]):
            msg_time = replay_data[t][0]
            obstacle_cloud_array = []
            for agent_id in range(0, self.total_drones):
                # obstacle_cloud_eachからobstacle_cloud_each[i][0]の値がmsg_timeに最も近い時刻の障害物点群を取得
                obstacle_cloud_each = self.obstacle_cloud[agent_id]
                i = np.abs(np.array([obstacle_cloud_each[j][0] for j in range(0, len(obstacle_cloud_each))]) - msg_time).argmin()
                obstacle_cloud_array.append(obstacle_cloud_each[i][1])
            obstacle_cloud_seq.append([msg_time, obstacle_cloud_array])

        obstacle_cloud_np = np.array(obstacle_cloud_seq, dtype=object)
        print("save data of shape, replay", replay_data.shape, "map", obstacle_cloud_np.shape)
        random_number = np.random.randint(10000000, 99999999)
        replay_file_path = os.path.expanduser("~/polka_dot/data/training/replay/agents{}_{}.npy".format(self.total_drones, random_number))
        map_file_path = os.path.expanduser("~/polka_dot/data/training/map/agents{}_{}.npy".format(self.total_drones, random_number))
        os.makedirs(os.path.dirname(replay_file_path), exist_ok=True)
        os.makedirs(os.path.dirname(map_file_path), exist_ok=True)
        np.save(replay_file_path, replay_data)
        # np.save(map_file_path, obstacle_cloud_array)
        if self.visualize:
            self.visualize_data(replay_file_path, replay_data, obstacle_cloud_seq)

    def visualize_data(self, file_path, replay_data, map_data):
        # replay_data = np.load(file_path, allow_pickle=True)
        # map_file = "{}/../map/{}".format(os.path.dirname(file_path), os.path.basename(file_path))
        # map_data = np.load(map_file, allow_pickle=True)

        # ワールド座標系
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
        ax.set_zlim(0, 15)
        colors = ["r", "g", "b", "c", "m", "y", "k"]
        for agent_id in range(0, replay_data.shape[1]-1):
            # エージェントの軌跡の描画
            trajectory_x = []
            trajectory_y = []
            trajectory_z = []
            for t in range(0, replay_data.shape[0]):
                trajectory_x.append(replay_data[t,agent_id+1][0])
                trajectory_y.append(replay_data[t,agent_id+1][1])
                trajectory_z.append(replay_data[t,agent_id+1][2])
            ax.plot(trajectory_x, trajectory_y, trajectory_z, 
                    color=colors[agent_id % len(colors)], label="drone{}".format(agent_id))
            # 点群の描画
            if(map_data[0][1][agent_id] == map_data[0][1][agent_id-1]):
                print("same")
            point_cloud = np.array(map_data[0][1][agent_id])
            if(point_cloud.shape[0] > 0):
                ax.scatter(point_cloud[:,0], point_cloud[:,1], point_cloud[:,2], 
                            color=colors[agent_id % len(colors)], s=1)
        plt.legend()
        plt.show()

    def is_proceeding(self):
        return self.agents_reached_goal.count(True) < self.total_drones *0.8
    
    def reset(self):
      # チェックポイント変数
        self.agents_reached_goal = [False] * self.total_drones
        self.snapshooted = [False] * self.total_drones
        self.goal_generated = False
        self.time_goal_generated = rospy.Time.now()
        self.prev_time = 0
      # データを格納する配列
        self.data_seq = []
        plane_array = []
        self.obstacle_cloud = [plane_array] * self.total_drones
      # ゴールを格納する配列
        self.waypoint_array = np.empty(shape=(0, 3))

if __name__ == "__main__":
    iteration_num = 1
    time_out = 60 # 60 seconds
    parser = argparse.ArgumentParser()
    parser.add_argument("--total_drones", type=int, default=3)
    parser.add_argument("--visualize", action="store_true")
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