; Auto-generated. Do not edit!


(cl:in-package serial_common-msg)


;//! \htmlinclude wind_mill_msg.msg.html

(cl:defclass <wind_mill_msg> (roslisp-msg-protocol:ros-message)
  ((horizonal
    :reader horizonal
    :initarg :horizonal
    :type cl:float
    :initform 0.0)
   (vertical
    :reader vertical
    :initarg :vertical
    :type cl:float
    :initform 0.0))
)

(cl:defclass wind_mill_msg (<wind_mill_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wind_mill_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wind_mill_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_common-msg:<wind_mill_msg> is deprecated: use serial_common-msg:wind_mill_msg instead.")))

(cl:ensure-generic-function 'horizonal-val :lambda-list '(m))
(cl:defmethod horizonal-val ((m <wind_mill_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_common-msg:horizonal-val is deprecated.  Use serial_common-msg:horizonal instead.")
  (horizonal m))

(cl:ensure-generic-function 'vertical-val :lambda-list '(m))
(cl:defmethod vertical-val ((m <wind_mill_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_common-msg:vertical-val is deprecated.  Use serial_common-msg:vertical instead.")
  (vertical m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wind_mill_msg>) ostream)
  "Serializes a message object of type '<wind_mill_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'horizonal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vertical))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wind_mill_msg>) istream)
  "Deserializes a message object of type '<wind_mill_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizonal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wind_mill_msg>)))
  "Returns string type for a message object of type '<wind_mill_msg>"
  "serial_common/wind_mill_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wind_mill_msg)))
  "Returns string type for a message object of type 'wind_mill_msg"
  "serial_common/wind_mill_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wind_mill_msg>)))
  "Returns md5sum for a message object of type '<wind_mill_msg>"
  "cac8e6416acc3c4967ad84a4b158da62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wind_mill_msg)))
  "Returns md5sum for a message object of type 'wind_mill_msg"
  "cac8e6416acc3c4967ad84a4b158da62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wind_mill_msg>)))
  "Returns full string definition for message of type '<wind_mill_msg>"
  (cl:format cl:nil "float32 horizonal~%float32 vertical~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wind_mill_msg)))
  "Returns full string definition for message of type 'wind_mill_msg"
  (cl:format cl:nil "float32 horizonal~%float32 vertical~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wind_mill_msg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wind_mill_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'wind_mill_msg
    (cl:cons ':horizonal (horizonal msg))
    (cl:cons ':vertical (vertical msg))
))
