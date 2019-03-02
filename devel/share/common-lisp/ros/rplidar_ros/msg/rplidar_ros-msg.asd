
(cl:in-package :asdf)

(defsystem "rplidar_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "coordinate_msg" :depends-on ("_package_coordinate_msg"))
    (:file "_package_coordinate_msg" :depends-on ("_package"))
  ))