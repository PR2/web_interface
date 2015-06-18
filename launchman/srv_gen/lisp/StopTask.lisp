; Auto-generated. Do not edit!


(cl:in-package launchman-srv)


;//! \htmlinclude StopTask-request.msg.html

(cl:defclass <StopTask-request> (roslisp-msg-protocol:ros-message)
  ((taskid
    :reader taskid
    :initarg :taskid
    :type cl:string
    :initform "")
   (username
    :reader username
    :initarg :username
    :type cl:string
    :initform ""))
)

(cl:defclass StopTask-request (<StopTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<StopTask-request> is deprecated: use launchman-srv:StopTask-request instead.")))

(cl:ensure-generic-function 'taskid-val :lambda-list '(m))
(cl:defmethod taskid-val ((m <StopTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:taskid-val is deprecated.  Use launchman-srv:taskid instead.")
  (taskid m))

(cl:ensure-generic-function 'username-val :lambda-list '(m))
(cl:defmethod username-val ((m <StopTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:username-val is deprecated.  Use launchman-srv:username instead.")
  (username m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopTask-request>) ostream)
  "Serializes a message object of type '<StopTask-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'taskid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'taskid))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'username))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'username))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopTask-request>) istream)
  "Deserializes a message object of type '<StopTask-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'taskid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'taskid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'username) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'username) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopTask-request>)))
  "Returns string type for a service object of type '<StopTask-request>"
  "launchman/StopTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopTask-request)))
  "Returns string type for a service object of type 'StopTask-request"
  "launchman/StopTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopTask-request>)))
  "Returns md5sum for a message object of type '<StopTask-request>"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopTask-request)))
  "Returns md5sum for a message object of type 'StopTask-request"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopTask-request>)))
  "Returns full string definition for message of type '<StopTask-request>"
  (cl:format cl:nil "string taskid~%string username~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopTask-request)))
  "Returns full string definition for message of type 'StopTask-request"
  (cl:format cl:nil "string taskid~%string username~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopTask-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'taskid))
     4 (cl:length (cl:slot-value msg 'username))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopTask-request
    (cl:cons ':taskid (taskid msg))
    (cl:cons ':username (username msg))
))
;//! \htmlinclude StopTask-response.msg.html

(cl:defclass <StopTask-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass StopTask-response (<StopTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<StopTask-response> is deprecated: use launchman-srv:StopTask-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <StopTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:status-val is deprecated.  Use launchman-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopTask-response>) ostream)
  "Serializes a message object of type '<StopTask-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopTask-response>) istream)
  "Deserializes a message object of type '<StopTask-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopTask-response>)))
  "Returns string type for a service object of type '<StopTask-response>"
  "launchman/StopTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopTask-response)))
  "Returns string type for a service object of type 'StopTask-response"
  "launchman/StopTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopTask-response>)))
  "Returns md5sum for a message object of type '<StopTask-response>"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopTask-response)))
  "Returns md5sum for a message object of type 'StopTask-response"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopTask-response>)))
  "Returns full string definition for message of type '<StopTask-response>"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopTask-response)))
  "Returns full string definition for message of type 'StopTask-response"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopTask-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopTask-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopTask)))
  'StopTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopTask)))
  'StopTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopTask)))
  "Returns string type for a service object of type '<StopTask>"
  "launchman/StopTask")