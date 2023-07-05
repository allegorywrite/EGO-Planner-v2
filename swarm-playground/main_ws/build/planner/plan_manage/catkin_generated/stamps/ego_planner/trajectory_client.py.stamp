#!/usr/bin/env python
import rospy
from quadrotor_msgs.msg import GoalSet
from traj_utils.srv import Trajectory

def call_trajectory_service():
    drone_id = 1
    rospy.init_node('trajectory_service_client') # ノードの初期化
    service_name = "/drone_" + str(drone_id) + "_planning/trajectory_service"
    rospy.wait_for_service(service_name) # サービスが利用可能になるまで待つ
    try:
        trajectory_service = rospy.ServiceProxy(service_name, Trajectory) # サービスのプロキシを作成
        req = GoalSet()
        req.drone_id = drone_id
        req.goal[0] = 0
        req.goal[1] = 0
        req.goal[2] = 0
        resp = trajectory_service(req) # サービスを呼び出す
        print(resp.path) # pathの中身を表示
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    call_trajectory_service()