#include <training_data_manager/training_data_manager.h>

using namespace training_data_manager;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "training_data_manager");
  ros::NodeHandle nh("~");
  TrainingDataManager training_data_manager(nh);
  // 1秒待つ
  ros::Duration(1.0).sleep();
  training_data_manager.generateGoal();
  ros::spin();
  return 0;
}