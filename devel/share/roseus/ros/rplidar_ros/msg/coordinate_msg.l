;; Auto-generated. Do not edit!


(when (boundp 'rplidar_ros::coordinate_msg)
  (if (not (find-package "RPLIDAR_ROS"))
    (make-package "RPLIDAR_ROS"))
  (shadow 'coordinate_msg (find-package "RPLIDAR_ROS")))
(unless (find-package "RPLIDAR_ROS::COORDINATE_MSG")
  (make-package "RPLIDAR_ROS::COORDINATE_MSG"))

(in-package "ROS")
;;//! \htmlinclude coordinate_msg.msg.html


(defclass rplidar_ros::coordinate_msg
  :super ros::object
  :slots (_coordinate_x _coordinate_y _angle ))

(defmethod rplidar_ros::coordinate_msg
  (:init
   (&key
    ((:coordinate_x __coordinate_x) 0.0)
    ((:coordinate_y __coordinate_y) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _coordinate_x (float __coordinate_x))
   (setq _coordinate_y (float __coordinate_y))
   (setq _angle (float __angle))
   self)
  (:coordinate_x
   (&optional __coordinate_x)
   (if __coordinate_x (setq _coordinate_x __coordinate_x)) _coordinate_x)
  (:coordinate_y
   (&optional __coordinate_y)
   (if __coordinate_y (setq _coordinate_y __coordinate_y)) _coordinate_y)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64 _coordinate_x
    8
    ;; float64 _coordinate_y
    8
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _coordinate_x
       (sys::poke _coordinate_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _coordinate_y
       (sys::poke _coordinate_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _coordinate_x
     (setq _coordinate_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _coordinate_y
     (setq _coordinate_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rplidar_ros::coordinate_msg :md5sum-) "7e3aeddc9dae72f8917acd183d2e2c53")
(setf (get rplidar_ros::coordinate_msg :datatype-) "rplidar_ros/coordinate_msg")
(setf (get rplidar_ros::coordinate_msg :definition-)
      "float64 coordinate_x
float64 coordinate_y
float64 angle
")



(provide :rplidar_ros/coordinate_msg "7e3aeddc9dae72f8917acd183d2e2c53")


