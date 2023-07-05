
(cl:in-package :asdf)

(defsystem "traj_utils-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :quadrotor_msgs-msg
)
  :components ((:file "_package")
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
  ))