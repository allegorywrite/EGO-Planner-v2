; Auto-generated. Do not edit!


(cl:in-package traj_utils-srv)


;//! \htmlinclude Trajectory-request.msg.html

(cl:defclass <Trajectory-request> (roslisp-msg-protocol:ros-message)
  ((goalset
    :reader goalset
    :initarg :goalset
    :type quadrotor_msgs-msg:GoalSet
    :initform (cl:make-instance 'quadrotor_msgs-msg:GoalSet)))
)

(cl:defclass Trajectory-request (<Trajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-srv:<Trajectory-request> is deprecated: use traj_utils-srv:Trajectory-request instead.")))

(cl:ensure-generic-function 'goalset-val :lambda-list '(m))
(cl:defmethod goalset-val ((m <Trajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-srv:goalset-val is deprecated.  Use traj_utils-srv:goalset instead.")
  (goalset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trajectory-request>) ostream)
  "Serializes a message object of type '<Trajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goalset) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trajectory-request>) istream)
  "Deserializes a message object of type '<Trajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goalset) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trajectory-request>)))
  "Returns string type for a service object of type '<Trajectory-request>"
  "traj_utils/TrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trajectory-request)))
  "Returns string type for a service object of type 'Trajectory-request"
  "traj_utils/TrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trajectory-request>)))
  "Returns md5sum for a message object of type '<Trajectory-request>"
  "daf3e1c19fb97921316eec6ffd7ff9ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trajectory-request)))
  "Returns md5sum for a message object of type 'Trajectory-request"
  "daf3e1c19fb97921316eec6ffd7ff9ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trajectory-request>)))
  "Returns full string definition for message of type '<Trajectory-request>"
  (cl:format cl:nil "quadrotor_msgs/GoalSet goalset~%~%================================================================================~%MSG: quadrotor_msgs/GoalSet~%int16 drone_id~%float32[3] goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trajectory-request)))
  "Returns full string definition for message of type 'Trajectory-request"
  (cl:format cl:nil "quadrotor_msgs/GoalSet goalset~%~%================================================================================~%MSG: quadrotor_msgs/GoalSet~%int16 drone_id~%float32[3] goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goalset))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Trajectory-request
    (cl:cons ':goalset (goalset msg))
))
;//! \htmlinclude Trajectory-response.msg.html

(cl:defclass <Trajectory-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass Trajectory-response (<Trajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-srv:<Trajectory-response> is deprecated: use traj_utils-srv:Trajectory-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <Trajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-srv:path-val is deprecated.  Use traj_utils-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trajectory-response>) ostream)
  "Serializes a message object of type '<Trajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trajectory-response>) istream)
  "Deserializes a message object of type '<Trajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trajectory-response>)))
  "Returns string type for a service object of type '<Trajectory-response>"
  "traj_utils/TrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trajectory-response)))
  "Returns string type for a service object of type 'Trajectory-response"
  "traj_utils/TrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trajectory-response>)))
  "Returns md5sum for a message object of type '<Trajectory-response>"
  "daf3e1c19fb97921316eec6ffd7ff9ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trajectory-response)))
  "Returns md5sum for a message object of type 'Trajectory-response"
  "daf3e1c19fb97921316eec6ffd7ff9ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trajectory-response>)))
  "Returns full string definition for message of type '<Trajectory-response>"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trajectory-response)))
  "Returns full string definition for message of type 'Trajectory-response"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Trajectory-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Trajectory)))
  'Trajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Trajectory)))
  'Trajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trajectory)))
  "Returns string type for a service object of type '<Trajectory>"
  "traj_utils/Trajectory")