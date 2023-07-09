#include <training_data_manager/training_data_manager.h>

namespace training_data_manager
{
    TrainingDataManager::TrainingDataManager(ros::NodeHandle &nh)
    {
        ROS_INFO("Initializing TrainingDataManager...");
        nh_ = nh;
        dim_per_drone_ = 14;
        goal_generated_ = false;
        episode_count_ = 0;
        waypoint_safe_radius_A2O_ = 0.1;
        waypoint_safe_radius_A2A_ = 0.5;
        /* parameter */
        nh.param("total_drones", total_drones_, 3);
        nh.param("snapshot_interval", snapshot_interval_, 0.05);
        nh.param("max_episode_num", max_episode_num_, 3);
        /* flag vector */
        reached_goal_flag_.resize(total_drones_, false);
        goal_of_all_drones_.resize(total_drones_, Eigen::Vector3d::Zero());
        /* snapshot vector */
        state_snapshot_of_all_drones_.resize(total_drones_, Eigen::VectorXd::Zero(dim_per_drone_));
        map_snapshot_of_all_drones_.resize(total_drones_, pcl::PointCloud<pcl::PointXYZ>());
        /* sequence vector */
        state_seq_of_all_drones_.clear();
        map_seq_of_all_drones_.clear();
        /* global map */
        if(getGlobalPointCloud())
        {
            ROS_INFO("Global point cloud received!");
        }else{
            ROS_ERROR("Failed to get global point cloud!");
            exit(-1);
        }
        /* timer */
        snapshot_timer_ = nh.createTimer(ros::Duration(snapshot_interval_), &TrainingDataManager::addSnapshot, this);
        /* publisher */
        goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/data_generator/goal_with_id", 1);
        /* subscriber */
        for(int i = 0; i < total_drones_; i++) 
        {
            std::string odom_topic = "/drone_" + std::to_string(i) + "_visual_slam/odom";
            std::string goal_topic = "/drone_" + std::to_string(i) + "_planning/reach_goal";
            std::string inflate_topic = "/drone_" + std::to_string(i) + "_ego_planner_node/grid_map/occupancy";

            odom_subs_.push_back(nh.subscribe<nav_msgs::Odometry>(odom_topic, 1000, boost::bind(&TrainingDataManager::odomCallback, this, _1, i)));
            reach_goal_subs_.push_back(nh.subscribe<std_msgs::Bool>(goal_topic, 1000, boost::bind(&TrainingDataManager::reachGoalCallback, this, _1, i)));
            local_point_cloud_subs_.push_back(nh.subscribe<sensor_msgs::PointCloud2>(inflate_topic, 1000, boost::bind(&TrainingDataManager::localPointCloudCallback, this, _1, i)));
        }
    }

    TrainingDataManager::~TrainingDataManager()
    { std::cout << "TrainingDataManager destructor called" << std::endl; }

    void TrainingDataManager::addSnapshot(const ros::TimerEvent &event)
    {
        if (!goal_generated_)
            return;
        state_seq_of_all_drones_.push_back(state_snapshot_of_all_drones_);
        map_seq_of_all_drones_.push_back(map_snapshot_of_all_drones_);
        // ROS_INFO("Add snapshot, total snapshots: %d", (int)state_seq_of_all_drones_.size());
    }

    void TrainingDataManager::localPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int drone_id)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        Eigen::Vector3d point;
        map_snapshot_of_all_drones_[drone_id] = cloud;
    }

    void TrainingDataManager::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int drone_id)
    {
        Eigen::VectorXd state(dim_per_drone_);
        state << msg->header.stamp.toSec(), msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                 msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                 msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
        state_snapshot_of_all_drones_[drone_id] = state;
        // state_snapshot_flag_[drone_id] = true;
    }

    void TrainingDataManager::reachGoalCallback(const std_msgs::Bool::ConstPtr &msg, int drone_id)
    {
        if (!goal_generated_)
            return;
        reached_goal_flag_[drone_id] = msg->data;
        ROS_INFO("Reached goal drones: %d", (int)std::count(reached_goal_flag_.begin(), reached_goal_flag_.end(), true));
        // 事前に設定した割合のドローンがゴールに到達したらデータを保存する
        if (std::count(reached_goal_flag_.begin(), reached_goal_flag_.end(), true) >= total_drones_ * 0.8)
        {
            ROS_INFO("Save training data");
            goal_generated_ = false;
            snapshot_timer_.stop();
            saveData();
            reset();
            if(episode_count_ >= max_episode_num_)
            {
                ROS_INFO("Finished training data generation");
                ros::shutdown();
            }else{
                ROS_INFO("Start episode %d", episode_count_);
                generateGoal();
            }
        }
    }

    bool TrainingDataManager::EpisodeIsFinished()
    {
        if (!goal_generated_)
            return false;
        if (std::count(reached_goal_flag_.begin(), reached_goal_flag_.end(), true) >= total_drones_ * 0.8)
        {
            ROS_INFO("Save training data");
            goal_generated_ = false;
            snapshot_timer_.stop();
            saveData();
            reset();
            return true;
        }
        return false;
    }

    void TrainingDataManager::generateGoal()
    {
        ROS_INFO("Generating goal...");
        // total_drones_個のゴールをランダムに生成する
        for (int i = 0; i < total_drones_; i++)
        {
            quadrotor_msgs::GoalSet goal_msg;
            int itr = 0;
            while(true)
            {
                ROS_INFO("Generating goal for drone %d", i);
                if (itr > 100)
                {
                    ROS_ERROR("Failed to generate goal");
                    return;
                }
                Eigen::Vector3d goal;
                goal << 20.0 * (double)rand() / RAND_MAX - 10.0, 
                        20.0 * (double)rand() / RAND_MAX - 10.0, 
                        2.0 * (double)rand() / RAND_MAX;
                if (!goalIsOccupied(goal))
                {
                    ROS_INFO("goal: [%f, %f, %f]", goal.x(), goal.y(), goal.z());
                    goal_of_all_drones_[i] = goal;
                    goal_msg.drone_id = i;
                    goal_msg.goal[0] = goal.x();
                    goal_msg.goal[1] = goal.y();
                    goal_msg.goal[2] = goal.z();
                    goal_pub_.publish(goal_msg);
                    ROS_INFO("Drone %d goal: [%f, %f, %f]", i, goal.x(), goal.y(), goal.z());
                    break;
                }
                itr++;
            }
        }
        goal_generated_ = true;
        episode_count_++;
        snapshot_timer_.start();
    }

    bool TrainingDataManager::goalIsOccupied(Eigen::Vector3d &goal)
    {
        // 生成したゴールが占有されているかどうかを判定する
        Eigen::Vector3d lower_bound_A2O = goal - Eigen::Vector3d(waypoint_safe_radius_A2O_, waypoint_safe_radius_A2O_, waypoint_safe_radius_A2O_);
        Eigen::Vector3d upper_bound_A2O = goal + Eigen::Vector3d(waypoint_safe_radius_A2O_, waypoint_safe_radius_A2O_, waypoint_safe_radius_A2O_);
        Eigen::Vector3d lower_bound_A2A = goal - Eigen::Vector3d(waypoint_safe_radius_A2A_, waypoint_safe_radius_A2A_, waypoint_safe_radius_A2A_);
        Eigen::Vector3d upper_bound_A2A = goal + Eigen::Vector3d(waypoint_safe_radius_A2A_, waypoint_safe_radius_A2A_, waypoint_safe_radius_A2A_);
        // global_point_cloud_の中でlower_bound_A2O~upper_bound_A2Oの範囲にある点があるかどうかを判定する
        for (int i = 0; i < global_point_cloud_.size(); i++)
        {
            Eigen::Vector3d point(global_point_cloud_.points[i].x, global_point_cloud_.points[i].y, global_point_cloud_.points[i].z);
            if (point.x() > lower_bound_A2O.x() && point.x() < upper_bound_A2O.x() &&
                point.y() > lower_bound_A2O.y() && point.y() < upper_bound_A2O.y() &&
                point.z() > lower_bound_A2O.z() && point.z() < upper_bound_A2O.z())
            {
                return true;
            }
        }
        // goal_of_all_drones_の中でlower_bound_A2A~upper_bound_A2Aの範囲にある点があるかどうかを判定する
        for (int i = 0; i < total_drones_; i++)
        {
            if (goal_of_all_drones_[i] == Eigen::Vector3d::Zero())
                continue;
            Eigen::Vector3d point(goal_of_all_drones_[i].x(), goal_of_all_drones_[i].y(), goal_of_all_drones_[i].z());
            if (point.x() > lower_bound_A2A.x() && point.x() < upper_bound_A2A.x() &&
                point.y() > lower_bound_A2A.y() && point.y() < upper_bound_A2A.y() &&
                point.z() > lower_bound_A2A.z() && point.z() < upper_bound_A2A.z())
            {
                return true;
            }
        }
        return false;
    }

    bool TrainingDataManager::getGlobalPointCloud()
    {
        // マップ全体の点群を取得する
        sensor_msgs::PointCloud2::ConstPtr global_cloud_msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/map_generator/global_cloud", ros::Duration(3.0));
        if (global_cloud_msg == NULL)
        {
            ROS_ERROR("Failed to get global point cloud");
            return false;
        }
        pcl::fromROSMsg(*global_cloud_msg, global_point_cloud_);
        return true;
    }

    void TrainingDataManager::reset()
    {
        goal_generated_ = false;
        snapshot_timer_.stop();

        /* flag vector */
        reached_goal_flag_.clear();
        reached_goal_flag_.resize(total_drones_, false);
        goal_of_all_drones_.clear();
        goal_of_all_drones_.resize(total_drones_, Eigen::Vector3d::Zero());
        /* snapshot vector */
        state_snapshot_of_all_drones_.clear();
        state_snapshot_of_all_drones_.resize(total_drones_, Eigen::VectorXd::Zero(dim_per_drone_));
        map_snapshot_of_all_drones_.clear();
        map_snapshot_of_all_drones_.resize(total_drones_, pcl::PointCloud<pcl::PointXYZ>());
        /* sequence vector */
        state_seq_of_all_drones_.clear();
        map_seq_of_all_drones_.clear();
    }

    void TrainingDataManager::saveData()
    {
        int random_number = rand() % (99999999 + 1 - 10000000) + 10000000;
        std::string home = std::getenv("HOME");
        std::string replay_file_path 
            = home + "/drone/polka_dot/data/training/replay/agents" + std::to_string(total_drones_) + "_" + std::to_string(random_number) + ".csv";
        std::string vision_file_path 
            = home + "/drone/polka_dot/data/training/vision/agents" + std::to_string(total_drones_) + "_" + std::to_string(random_number) + ".pcd";
        std::string map_file_path
            = home + "/drone/polka_dot/data/training/map/agents" + std::to_string(total_drones_) + "_" + std::to_string(random_number) + ".yaml";

        // リプレイデータをcsvで保存
        std::ofstream state_file(replay_file_path);
        for (const auto& snapshot : state_seq_of_all_drones_) {
            for (int t = 0; t < snapshot.size(); ++t) {
                Eigen::VectorXd state_of_each_drone = snapshot[t];
                for (int i = 0; i < state_of_each_drone.size(); ++i) {
                    state_file << state_of_each_drone[i];
                    if (!(t == snapshot.size() - 1 && i == state_of_each_drone.size() - 1))
                        state_file << ",";
                }
            }
            state_file << "\n";
        }
        state_file.close();
        
        // 点群データをpcdで保存
        for (int t = 0; t < map_seq_of_all_drones_.size(); ++t) {
            for (int i = 0; i < map_seq_of_all_drones_[t].size(); ++i) {
                if(map_seq_of_all_drones_[t][i].size() == 0)
                    // 点群がない場合は特殊な値を入れる
                    map_seq_of_all_drones_[t][i].push_back(pcl::PointXYZ(0, 0, 0));
                pcl::io::savePCDFileASCII(vision_file_path + "_agent" + std::to_string(i) + "_timestep" + std::to_string(t) + ".pcd", map_seq_of_all_drones_[t][i]);
            }
        }

        // マップデータをyamlで保存
        YAML::Node agents = YAML::Node();
        for (int i = 0; i < total_drones_; ++i) {
            YAML::Node agent = YAML::Node();
            agent["name"] = "agent" + std::to_string(i);
            std::vector<double> goal = {goal_of_all_drones_[i].x(), goal_of_all_drones_[i].y(), goal_of_all_drones_[i].z()};
            agent["goal"] = goal;
            agents.push_back(agent);
        }
        YAML::Node map = YAML::Node();
        map["agents"] = agents;
        std::ofstream map_file(map_file_path);
        map_file << map;
        map_file.close();

        ROS_INFO("Saved data to %s, %s and %s", replay_file_path.c_str(), vision_file_path.c_str(), map_file_path.c_str());
    }
}