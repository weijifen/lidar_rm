;; Auto-generated. Do not edit!


(when (boundp 'serial_common::wind_mill_msg)
  (if (not (find-package "SERIAL_COMMON"))
    (make-package "SERIAL_COMMON"))
  (shadow 'wind_mill_msg (find-package "SERIAL_COMMON")))
(unless (find-package "SERIAL_COMMON::WIND_MILL_MSG")
  (make-package "SERIAL_COMMON::WIND_MILL_MSG"))

(in-package "ROS")
;;//! \htmlinclude wind_mill_msg.msg.html


(defclass serial_common::wind_mill_msg
  :super ros::object
  :slots (_horizonal _vertical ))

(defmethod serial_common::wind_mill_msg
  (:init
   (&key
    ((:horizonal __horizonal) 0.0)
    ((:vertical __vertical) 0.0)
    )
   (send-super :init)
   (setq _horizonal (float __horizonal))
   (setq _vertical (float __vertical))
   self)
  (:horizonal
   (&optional __horizonal)
   (if __horizonal (setq _horizonal __horizonal)) _horizonal)
  (:vertical
   (&optional __vertical)
   (if __vertical (setq _vertical __vertical)) _vertical)
  (:serialization-length
   ()
   (+
    ;; float32 _horizonal
    4
    ;; float32 _vertical
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _horizonal
       (sys::poke _horizonal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vertical
       (sys::poke _vertical (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _horizonal
     (setq _horizonal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vertical
     (setq _vertical (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get serial_common::wind_mill_msg :md5sum-) "cac8e6416acc3c4967ad84a4b158da62")
(setf (get serial_common::wind_mill_msg :datatype-) "serial_common/wind_mill_msg")
(setf (get serial_common::wind_mill_msg :definition-)
      "float32 horizonal
float32 vertical

")



(provide :serial_common/wind_mill_msg "cac8e6416acc3c4967ad84a4b158da62")


