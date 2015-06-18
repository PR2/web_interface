; Auto-generated. Do not edit!


(cl:in-package launchman-srv)


;//! \htmlinclude ListTasks-request.msg.html

(cl:defclass <ListTasks-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ListTasks-request (<ListTasks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListTasks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListTasks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<ListTasks-request> is deprecated: use launchman-srv:ListTasks-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListTasks-request>) ostream)
  "Serializes a message object of type '<ListTasks-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListTasks-request>) istream)
  "Deserializes a message object of type '<ListTasks-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListTasks-request>)))
  "Returns string type for a service object of type '<ListTasks-request>"
  "launchman/ListTasksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListTasks-request)))
  "Returns string type for a service object of type 'ListTasks-request"
  "launchman/ListTasksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListTasks-request>)))
  "Returns md5sum for a message object of type '<ListTasks-request>"
  "8e23a0a55cd0f3fcb9989be6ceff0145")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListTasks-request)))
  "Returns md5sum for a message object of type 'ListTasks-request"
  "8e23a0a55cd0f3fcb9989be6ceff0145")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListTasks-request>)))
  "Returns full string definition for message of type '<ListTasks-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListTasks-request)))
  "Returns full string definition for message of type 'ListTasks-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListTasks-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListTasks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ListTasks-request
))
;//! \htmlinclude ListTasks-response.msg.html

(cl:defclass <ListTasks-response> (roslisp-msg-protocol:ros-message)
  ((tasks
    :reader tasks
    :initarg :tasks
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ListTasks-response (<ListTasks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListTasks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListTasks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<ListTasks-response> is deprecated: use launchman-srv:ListTasks-response instead.")))

(cl:ensure-generic-function 'tasks-val :lambda-list '(m))
(cl:defmethod tasks-val ((m <ListTasks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:tasks-val is deprecated.  Use launchman-srv:tasks instead.")
  (tasks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListTasks-response>) ostream)
  "Serializes a message object of type '<ListTasks-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'tasks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListTasks-response>) istream)
  "Deserializes a message object of type '<ListTasks-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tasks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tasks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListTasks-response>)))
  "Returns string type for a service object of type '<ListTasks-response>"
  "launchman/ListTasksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListTasks-response)))
  "Returns string type for a service object of type 'ListTasks-response"
  "launchman/ListTasksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListTasks-response>)))
  "Returns md5sum for a message object of type '<ListTasks-response>"
  "8e23a0a55cd0f3fcb9989be6ceff0145")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListTasks-response)))
  "Returns md5sum for a message object of type 'ListTasks-response"
  "8e23a0a55cd0f3fcb9989be6ceff0145")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListTasks-response>)))
  "Returns full string definition for message of type '<ListTasks-response>"
  (cl:format cl:nil "string[] tasks~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListTasks-response)))
  "Returns full string definition for message of type 'ListTasks-response"
  (cl:format cl:nil "string[] tasks~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListTasks-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListTasks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ListTasks-response
    (cl:cons ':tasks (tasks msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ListTasks)))
  'ListTasks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ListTasks)))
  'ListTasks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListTasks)))
  "Returns string type for a service object of type '<ListTasks>"
  "launchman/ListTasks")