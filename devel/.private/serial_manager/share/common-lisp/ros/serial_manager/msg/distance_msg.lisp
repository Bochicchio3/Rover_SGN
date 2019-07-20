; Auto-generated. Do not edit!


(cl:in-package serial_manager-msg)


;//! \htmlinclude distance_msg.msg.html

(cl:defclass <distance_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (grafic
    :reader grafic
    :initarg :grafic
    :type (cl:vector cl:float)
   :initform (cl:make-array 57 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass distance_msg (<distance_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <distance_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'distance_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_manager-msg:<distance_msg> is deprecated: use serial_manager-msg:distance_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <distance_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_manager-msg:header-val is deprecated.  Use serial_manager-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'grafic-val :lambda-list '(m))
(cl:defmethod grafic-val ((m <distance_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_manager-msg:grafic-val is deprecated.  Use serial_manager-msg:grafic instead.")
  (grafic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <distance_msg>) ostream)
  "Serializes a message object of type '<distance_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'grafic))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <distance_msg>) istream)
  "Deserializes a message object of type '<distance_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'grafic) (cl:make-array 57))
  (cl:let ((vals (cl:slot-value msg 'grafic)))
    (cl:dotimes (i 57)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<distance_msg>)))
  "Returns string type for a message object of type '<distance_msg>"
  "serial_manager/distance_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'distance_msg)))
  "Returns string type for a message object of type 'distance_msg"
  "serial_manager/distance_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<distance_msg>)))
  "Returns md5sum for a message object of type '<distance_msg>"
  "f8328157916dc7d8e983a8a56463b89a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'distance_msg)))
  "Returns md5sum for a message object of type 'distance_msg"
  "f8328157916dc7d8e983a8a56463b89a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<distance_msg>)))
  "Returns full string definition for message of type '<distance_msg>"
  (cl:format cl:nil "Header header~%float64[57] grafic~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'distance_msg)))
  "Returns full string definition for message of type 'distance_msg"
  (cl:format cl:nil "Header header~%float64[57] grafic~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <distance_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'grafic) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <distance_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'distance_msg
    (cl:cons ':header (header msg))
    (cl:cons ':grafic (grafic msg))
))
