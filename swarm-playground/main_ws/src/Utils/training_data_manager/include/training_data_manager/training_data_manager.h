#ifndef TRAINING_DATA_MANAGER_H
#define TRAINING_DATA_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <filesystem>

namespace training_data_manager
{
    class TrainingDataManager
    {
    public:
        TrainingDataManager(ros::NodeHandle &nh);
        ~TrainingDataManager();
        void generateGoal();

    private:
        ros::NodeHandle nh_;

        ros::Publisher goal_pub_;
        ros::Timer snapshot_timer_;
        // tf::TransformListener tf_listener_;
        std::vector<ros::Subscriber> odom_subs_;
        std::vector<ros::Subscriber> reach_goal_subs_;
        std::vector<ros::Subscriber> local_point_cloud_subs_;

        int total_drones_;
        int dim_per_drone_;
        double snapshot_interval_;
        bool goal_generated_;
        double waypoint_safe_radius_A2O_;
        double waypoint_safe_radius_A2A_;
        std::vector<bool> reached_goal_flag_;
        std::vector<Eigen::Vector3d> goal_of_all_drones_;

        std::vector<Eigen::VectorXd> state_snapshot_of_all_drones_;
        std::vector<std::vector<Eigen::VectorXd>> state_seq_of_all_drones_;
        // std::vector<bool> state_snapshot_flag_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> map_snapshot_of_all_drones_;
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>>> map_seq_of_all_drones_;

        pcl::PointCloud<pcl::PointXYZ> global_point_cloud_;
        
        bool getGlobalPointCloud();

        void addSnapshot(const ros::TimerEvent &e);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int drone_id);
        void reachGoalCallback(const std_msgs::Bool::ConstPtr &msg, int drone_id);
        void localPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int drone_id);
        bool goalIsOccupied(Eigen::Vector3d &goal);
        void saveData();
        void visualizeTrajectoryies();
        void reset();
    };
} // namespace training_data_manager

#endif