#include <training_data_manager/training_data_manager.h>

namespace training_data_manager
{
    TrainingDataManager::TrainingDataManager(ros::NodeHandle &nh)
    {
        ROS_INFO("Initializing TrainingDataManager...");
        nh_ = nh;
        dim_per_drone_ = 14;
        goal_generated_ = false;
        test_mode_ = false;
        episode_count_ = 0;
        prev_restart_time_ = ros::Time::now();
        node_reset_episode_count_ = 0;
        waypoint_safe_radius_A2O_ = 0.1;
        waypoint_safe_radius_A2A_ = 0.5;
        /* parameter */
        nh.param("total_drones", total_drones_, 3);
        nh.param("snapshot_interval", snapshot_interval_, 0.05);
        nh.param("max_episode_num", max_episode_num_, 3);
        nh.param("test_mode", test_mode_, false);
        /* flag vector */
        reached_goal_flag_.resize(total_drones_, false);
        reached_goal_time_.resize(total_drones_, ros::Time::now());
        goal_of_all_drones_.resize(total_drones_, Eigen::Vector3d::Zero());
        /* snapshot vector */
        state_snapshot_of_all_drones_.resize(total_drones_, Eigen::VectorXd::Zero(dim_per_drone_));
        state_snapshot_of_all_drones_last_.resize(total_drones_, Eigen::VectorXd::Zero(dim_per_drone_));
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
        freeze_check_timer_ = nh.createTimer(ros::Duration(1.0), &TrainingDataManager::checkFreeze, this);
        emergency_restart_timer_ = nh.createTimer(ros::Duration(120.0), &TrainingDataManager::emergencyRestart, this);
        /* publisher */
        goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/data_generator/goal_with_id", 1);
        // goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal", 1);
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

    void TrainingDataManager::checkFreeze(const ros::TimerEvent &event)
    {
        for(int i = 0; i < total_drones_; i++)
        {
            Eigen::Vector3f old_state = Eigen::Vector3f(state_snapshot_of_all_drones_last_[i][1], state_snapshot_of_all_drones_last_[i][2], state_snapshot_of_all_drones_last_[i][3]);
            Eigen::Vector3f current_state = Eigen::Vector3f(state_snapshot_of_all_drones_[i][1], state_snapshot_of_all_drones_[i][2], state_snapshot_of_all_drones_[i][3]);
            bool is_frozen = (current_state - old_state).norm() < 0.001;
            bool is_reached_goal = reached_goal_flag_[i];
            if((ros::Time::now() - reached_goal_time_[i]).toSec() > 20.0 && is_frozen && !is_reached_goal)
            {
                ROS_ERROR("Drone %d is frozen!", i);
                killNodes(i);
                for (int j = 0; j < total_drones_; j++){
                    reached_goal_time_[j] = ros::Time::now();
                }
                // node_reset_episode_count_++;
                // reset();
                // generateGoal();
            }
        }
        state_snapshot_of_all_drones_last_ = state_snapshot_of_all_drones_;
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
                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                 msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
                 msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
        state_snapshot_of_all_drones_[drone_id] = state;
        // state_snapshot_flag_[drone_id] = true;
    }

    void TrainingDataManager::reachGoalCallback(const std_msgs::Bool::ConstPtr &msg, int drone_id)
    {
        if (!goal_generated_)
            return;
        reached_goal_flag_[drone_id] = msg->data;
        reached_goal_time_[drone_id] = ros::Time::now();
        ROS_INFO("Reached goal drones: %d", (int)std::count(reached_goal_flag_.begin(), reached_goal_flag_.end(), true));
        // 事前に設定した割合のドローンがゴールに到達したらデータを保存する
        if (std::count(reached_goal_flag_.begin(), reached_goal_flag_.end(), true) >= total_drones_ * 0.6)
        {
            goal_generated_ = false;
            snapshot_timer_.stop();
            emergency_restart_timer_.stop();
            if(!test_mode_){
                ROS_INFO("Save training data");
                saveData();
            }
            // prev_restart_time_
            if ((ros::Time::now() - prev_restart_time_).toSec() > 1000.0)
            {
                for (int i = 0; i < total_drones_; i++){
                    killNodes(i);
                }
                    
                prev_restart_time_ = ros::Time::now();
                reset(true);
            }else{
                reset(false);
            }

            // if(node_reset_episode_count_ >= 3){
            //     for (int i = 0; i < total_drones_; i++)
            //         if (!reached_goal_flag_[i])
            //              killNodes(i);
            //     node_reset_episode_count_ = 0;
            //     ros::Duration(5.0).sleep();
            // }
            // for (int i = 0; i < total_drones_; i++)
            //     killNodes(i);
            // node_reset_episode_count_ = 0;
            // ros::Duration(5.0).sleep();

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
        if (std::count(reached_goal_flag_.begin(), reached_goal_flag_.end(), true) >= total_drones_ * 0.6)
        {
            ROS_INFO("Save training data");
            goal_generated_ = false;
            snapshot_timer_.stop();
            emergency_restart_timer_.stop();
            saveData();
            reset(false);
            return true;
        }
        return false;
    }

    void TrainingDataManager::emergencyRestart(const ros::TimerEvent &event)
    {
        ROS_INFO("Emergency restart");
        goal_generated_ = false;
        snapshot_timer_.stop();
        reset(true);
        generateGoal();
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
        node_reset_episode_count_++;
        snapshot_timer_.start();
        emergency_restart_timer_.start();
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

    void TrainingDataManager::reset(bool reset_monitor)
    {
        if(reset_monitor){
            reached_goal_time_.resize(total_drones_, ros::Time::now());
        }
            
        goal_generated_ = false;
        snapshot_timer_.stop();
        emergency_restart_timer_.stop();

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
        // for (int t = 0; t < map_seq_of_all_drones_.size(); ++t) {
        //     for (int i = 0; i < map_seq_of_all_drones_[t].size(); ++i) {
        //         if(map_seq_of_all_drones_[t][i].size() == 0)
        //             // 点群がない場合は特殊な値を入れる
        //             map_seq_of_all_drones_[t][i].push_back(pcl::PointXYZ(0, 0, 0));
        //         pcl::io::savePCDFileASCII(vision_file_path + "_agent" + std::to_string(i) + "_timestep" + std::to_string(t) + ".pcd", map_seq_of_all_drones_[t][i]);
        //     }
        // }
        pcl::io::savePCDFileASCII(vision_file_path, global_point_cloud_);

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

    std::string TrainingDataManager::exec(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }
    
    bool TrainingDataManager::checkNodes(int drone_id)
    {
        std::vector<std::string> node_array = {
            "/drone_"+std::to_string(drone_id)+"_ego_planner_node", 
            "/drone_"+std::to_string(drone_id)+"_odom_visualization", 
            "/drone_"+std::to_string(drone_id)+"_pcl_render_node", 
            "/drone_"+std::to_string(drone_id)+"_poscmd_2_odom", 
            "/drone_"+std::to_string(drone_id)+"_traj_server"};
        std::string node_list = exec("rosnode list");
        std::istringstream node_stream(node_list);
        std::string node;

        while (std::getline(node_stream, node)) {
            auto itr = std::find(node_array.begin(), node_array.end(), node);
            if (itr != node_array.end()) {
                node_array.erase(itr);
            }
            if (node_array.empty()) {
                return true;
            }
        }
        return false;
    }

    void TrainingDataManager::killNodes(int drone_id)
    {
        std::vector<std::string> node_array = {
            "/drone_"+std::to_string(drone_id)+"_ego_planner_node", 
            "/drone_"+std::to_string(drone_id)+"_odom_visualization", 
            "/drone_"+std::to_string(drone_id)+"_pcl_render_node", 
            "/drone_"+std::to_string(drone_id)+"_poscmd_2_odom", 
            "/drone_"+std::to_string(drone_id)+"_traj_server"};
        std::string node_list = exec("rosnode list");
        std::istringstream node_stream(node_list);
        std::string node;
        
        // Print currently running nodes
        std::cout << "Currently running nodes: \n";
        while (std::getline(node_stream, node)) {
            if(std::find(node_array.begin(), node_array.end(), node) != node_array.end()){
                std::string kill_cmd = "rosnode kill " + node;
                std::cout << "kill node: " << node << "\n";
                system(kill_cmd.c_str()); // Execute shell command to kill node
            }
        }
        // ros::Duration(0.1).sleep();
        // std::string rebuild_cmd = "roslaunch ego_planner autoflight.launch";
        // system(rebuild_cmd.c_str()); // Execute shell command to kill node
        // std::cout << "Do something else..." << "\n";
    }
}