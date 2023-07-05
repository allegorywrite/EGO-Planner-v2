;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::Trajectory)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'Trajectory (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::TRAJECTORY")
  (make-package "TRAJ_UTILS::TRAJECTORY"))
(unless (find-package "TRAJ_UTILS::TRAJECTORYREQUEST")
  (make-package "TRAJ_UTILS::TRAJECTORYREQUEST"))
(unless (find-package "TRAJ_UTILS::TRAJECTORYRESPONSE")
  (make-package "TRAJ_UTILS::TRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "QUADROTOR_MSGS"))
  (ros::roseus-add-msgs "quadrotor_msgs"))


(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass traj_utils::TrajectoryRequest
  :super ros::object
  :slots (_goalset ))

(defmethod traj_utils::TrajectoryRequest
  (:init
   (&key
    ((:goalset __goalset) (instance quadrotor_msgs::GoalSet :init))
    )
   (send-super :init)
   (setq _goalset __goalset)
   self)
  (:goalset
   (&rest __goalset)
   (if (keywordp (car __goalset))
       (send* _goalset __goalset)
     (progn
       (if __goalset (setq _goalset (car __goalset)))
       _goalset)))
  (:serialization-length
   ()
   (+
    ;; quadrotor_msgs/GoalSet _goalset
    (send _goalset :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; quadrotor_msgs/GoalSet _goalset
       (send _goalset :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; quadrotor_msgs/GoalSet _goalset
     (send _goalset :deserialize buf ptr-) (incf ptr- (send _goalset :serialization-length))
   ;;
   self)
  )

(defclass traj_utils::TrajectoryResponse
  :super ros::object
  :slots (_path ))

(defmethod traj_utils::TrajectoryResponse
  (:init
   (&key
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(defclass traj_utils::Trajectory
  :super ros::object
  :slots ())

(setf (get traj_utils::Trajectory :md5sum-) "daf3e1c19fb97921316eec6ffd7ff9ed")
(setf (get traj_utils::Trajectory :datatype-) "traj_utils/Trajectory")
(setf (get traj_utils::Trajectory :request) traj_utils::TrajectoryRequest)
(setf (get traj_utils::Trajectory :response) traj_utils::TrajectoryResponse)

(defmethod traj_utils::TrajectoryRequest
  (:response () (instance traj_utils::TrajectoryResponse :init)))

(setf (get traj_utils::TrajectoryRequest :md5sum-) "daf3e1c19fb97921316eec6ffd7ff9ed")
(setf (get traj_utils::TrajectoryRequest :datatype-) "traj_utils/TrajectoryRequest")
(setf (get traj_utils::TrajectoryRequest :definition-)
      "quadrotor_msgs/GoalSet goalset

================================================================================
MSG: quadrotor_msgs/GoalSet
int16 drone_id
float32[3] goal
---
nav_msgs/Path path

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get traj_utils::TrajectoryResponse :md5sum-) "daf3e1c19fb97921316eec6ffd7ff9ed")
(setf (get traj_utils::TrajectoryResponse :datatype-) "traj_utils/TrajectoryResponse")
(setf (get traj_utils::TrajectoryResponse :definition-)
      "quadrotor_msgs/GoalSet goalset

================================================================================
MSG: quadrotor_msgs/GoalSet
int16 drone_id
float32[3] goal
---
nav_msgs/Path path

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :traj_utils/Trajectory "daf3e1c19fb97921316eec6ffd7ff9ed")


