; Auto-generated. Do not edit!


(in-package lcm_mavlink_ros-msg)


;//! \htmlinclude Mavlink.msg.html

(defclass <Mavlink> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (len
    :reader len-val
    :initarg :len
    :type fixnum
    :initform 0)
   (seq
    :reader seq-val
    :initarg :seq
    :type fixnum
    :initform 0)
   (sysid
    :reader sysid-val
    :initarg :sysid
    :type fixnum
    :initform 0)
   (compid
    :reader compid-val
    :initarg :compid
    :type fixnum
    :initform 0)
   (msgid
    :reader msgid-val
    :initarg :msgid
    :type fixnum
    :initform 0)
   (fromlcm
    :reader fromlcm-val
    :initarg :fromlcm
    :type boolean
    :initform nil)
   (payload
    :reader payload-val
    :initarg :payload
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0)))
)
(defmethod serialize ((msg <Mavlink>) ostream)
  "Serializes a message object of type '<Mavlink>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'len)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'seq)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'sysid)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'compid)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'msgid)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'fromlcm) 1 0)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'payload))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'payload))
)
(defmethod deserialize ((msg <Mavlink>) istream)
  "Deserializes a message object of type '<Mavlink>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'len)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'seq)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'sysid)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'compid)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'msgid)) (read-byte istream))
  (setf (slot-value msg 'fromlcm) (not (zerop (read-byte istream))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'payload) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'payload)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Mavlink>)))
  "Returns string type for a message object of type '<Mavlink>"
  "lcm_mavlink_ros/Mavlink")
(defmethod md5sum ((type (eql '<Mavlink>)))
  "Returns md5sum for a message object of type '<Mavlink>"
  "cd609ddc2b0642cb6f5fb5ca07f1fb77")
(defmethod message-definition ((type (eql '<Mavlink>)))
  "Returns full string definition for message of type '<Mavlink>"
  (format nil "Header header~%uint8 len~%uint8 seq~%uint8 sysid~%uint8 compid~%uint8 msgid~%bool fromlcm~%uint8[] payload~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Mavlink>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     4 (reduce #'+ (slot-value msg 'payload) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
))
(defmethod ros-message-to-list ((msg <Mavlink>))
  "Converts a ROS message object to a list"
  (list '<Mavlink>
    (cons ':header (header-val msg))
    (cons ':len (len-val msg))
    (cons ':seq (seq-val msg))
    (cons ':sysid (sysid-val msg))
    (cons ':compid (compid-val msg))
    (cons ':msgid (msgid-val msg))
    (cons ':fromlcm (fromlcm-val msg))
    (cons ':payload (payload-val msg))
))
