;; Auto-generated. Do not edit!


(when (boundp 'serial_manager::distance_msg)
  (if (not (find-package "SERIAL_MANAGER"))
    (make-package "SERIAL_MANAGER"))
  (shadow 'distance_msg (find-package "SERIAL_MANAGER")))
(unless (find-package "SERIAL_MANAGER::DISTANCE_MSG")
  (make-package "SERIAL_MANAGER::DISTANCE_MSG"))

(in-package "ROS")
;;//! \htmlinclude distance_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass serial_manager::distance_msg
  :super ros::object
  :slots (_header _grafic ))

(defmethod serial_manager::distance_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:grafic __grafic) (make-array 57 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _grafic __grafic)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:grafic
   (&optional __grafic)
   (if __grafic (setq _grafic __grafic)) _grafic)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[57] _grafic
    (* 8    57)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[57] _grafic
     (dotimes (i 57)
       (sys::poke (elt _grafic i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[57] _grafic
   (dotimes (i (length _grafic))
     (setf (elt _grafic i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get serial_manager::distance_msg :md5sum-) "f8328157916dc7d8e983a8a56463b89a")
(setf (get serial_manager::distance_msg :datatype-) "serial_manager/distance_msg")
(setf (get serial_manager::distance_msg :definition-)
      "Header header
float64[57] grafic


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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :serial_manager/distance_msg "f8328157916dc7d8e983a8a56463b89a")


