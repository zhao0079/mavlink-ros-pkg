
(in-package :asdf)

(defsystem "lcm_mavlink_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Mavlink" :depends-on ("_package"))
    (:file "_package_Mavlink" :depends-on ("_package"))
    ))
