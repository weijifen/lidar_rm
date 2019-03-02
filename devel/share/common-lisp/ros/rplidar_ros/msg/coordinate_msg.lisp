; Auto-generated. Do not edit!


(cl:in-package rplidar_ros-msg)


;//! \htmlinclude coordinate_msg.msg.html

(cl:defclass <coordinate_msg> (roslisp-msg-protocol:ros-message)
  ((coordinate_x
    :reader coordinate_x
    :initarg :coordinate_x
    :type cl:float
    :initform 0.0)
   (coordinate_y
    :reader coordinate_y
    :initarg :coordinate_y
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass coordinate_msg (<coordinate_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <coordinate_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'coordinate_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rplidar_ros-msg:<coordinate_msg> is deprecated: use rplidar_ros-msg:coordinate_msg instead.")))

(cl:ensure-generic-function 'coordinate_x-val :lambda-list '(m))
(cl:defmethod coordinate_x-val ((m <coordinate_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rplidar_ros-msg:coordinate_x-val is deprecated.  Use rplidar_ros-msg:coordinate_x instead.")
  (coordinate_x m))

(cl:ensure-generic-function 'coordinate_y-val :lambda-list '(m))
(cl:defmethod coordinate_y-val ((m <coordinate_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rplidar_ros-msg:coordinate_y-val is deprecated.  Use rplidar_ros-msg:coordinate_y instead.")
  (coordinate_y m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <coordinate_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rplidar_ros-msg:angle-val is deprecated.  Use rplidar_ros-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <coordinate_msg>) ostream)
  "Serializes a message object of type '<coordinate_msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'coordinate_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'coordinate_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <coordinate_msg>) istream)
  "Deserializes a message object of type '<coordinate_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'coordinate_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'coordinate_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<coordinate_msg>)))
  "Returns string type for a message object of type '<coordinate_msg>"
  "rplidar_ros/coordinate_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'coordinate_msg)))
  "Returns string type for a message object of type 'coordinate_msg"
  "rplidar_ros/coordinate_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<coordinate_msg>)))
  "Returns md5sum for a message object of type '<coordinate_msg>"
  "7e3aeddc9dae72f8917acd183d2e2c53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'coordinate_msg)))
  "Returns md5sum for a message object of type 'coordinate_msg"
  "7e3aeddc9dae72f8917acd183d2e2c53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<coordinate_msg>)))
  "Returns full string definition for message of type '<coordinate_msg>"
  (cl:format cl:nil "float64 coordinate_x~%float64 coordinate_y~%float64 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'coordinate_msg)))
  "Returns full string definition for message of type 'coordinate_msg"
  (cl:format cl:nil "float64 coordinate_x~%float64 coordinate_y~%float64 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <coordinate_msg>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <coordinate_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'coordinate_msg
    (cl:cons ':coordinate_x (coordinate_x msg))
    (cl:cons ':coordinate_y (coordinate_y msg))
    (cl:cons ':angle (angle msg))
))
