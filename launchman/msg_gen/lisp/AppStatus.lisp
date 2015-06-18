; Auto-generated. Do not edit!


(cl:in-package launchman-msg)


;//! \htmlinclude AppStatus.msg.html

(cl:defclass <AppStatus> (roslisp-msg-protocol:ros-message)
  ((active
    :reader active
    :initarg :active
    :type (cl:vector launchman-msg:Application)
   :initform (cl:make-array 0 :element-type 'launchman-msg:Application :initial-element (cl:make-instance 'launchman-msg:Application))))
)

(cl:defclass AppStatus (<AppStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AppStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AppStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-msg:<AppStatus> is deprecated: use launchman-msg:AppStatus instead.")))

(cl:ensure-generic-function 'active-val :lambda-list '(m))
(cl:defmethod active-val ((m <AppStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:active-val is deprecated.  Use launchman-msg:active instead.")
  (active m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AppStatus>) ostream)
  "Serializes a message object of type '<AppStatus>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'active))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'active))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AppStatus>) istream)
  "Deserializes a message object of type '<AppStatus>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'active) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'active)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'launchman-msg:Application))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AppStatus>)))
  "Returns string type for a message object of type '<AppStatus>"
  "launchman/AppStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AppStatus)))
  "Returns string type for a message object of type 'AppStatus"
  "launchman/AppStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AppStatus>)))
  "Returns md5sum for a message object of type '<AppStatus>"
  "8a2d696a9a11a9fb77b5416e878dcad0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AppStatus)))
  "Returns md5sum for a message object of type 'AppStatus"
  "8a2d696a9a11a9fb77b5416e878dcad0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AppStatus>)))
  "Returns full string definition for message of type '<AppStatus>"
  (cl:format cl:nil "Application[] active~%~%================================================================================~%MSG: launchman/Application~%string taskid~%string name~%string status~%string icon~%string provides~%string depends~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AppStatus)))
  "Returns full string definition for message of type 'AppStatus"
  (cl:format cl:nil "Application[] active~%~%================================================================================~%MSG: launchman/Application~%string taskid~%string name~%string status~%string icon~%string provides~%string depends~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AppStatus>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'active) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AppStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'AppStatus
    (cl:cons ':active (active msg))
))
