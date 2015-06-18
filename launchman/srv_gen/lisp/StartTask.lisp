; Auto-generated. Do not edit!


(cl:in-package launchman-srv)


;//! \htmlinclude StartTask-request.msg.html

(cl:defclass <StartTask-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass StartTask-request (<StartTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<StartTask-request> is deprecated: use launchman-srv:StartTask-request instead.")))

(cl:ensure-generic-function 'taskid-val :lambda-list '(m))
(cl:defmethod taskid-val ((m <StartTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:taskid-val is deprecated.  Use launchman-srv:taskid instead.")
  (taskid m))

(cl:ensure-generic-function 'username-val :lambda-list '(m))
(cl:defmethod username-val ((m <StartTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:username-val is deprecated.  Use launchman-srv:username instead.")
  (username m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTask-request>) ostream)
  "Serializes a message object of type '<StartTask-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTask-request>) istream)
  "Deserializes a message object of type '<StartTask-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTask-request>)))
  "Returns string type for a service object of type '<StartTask-request>"
  "launchman/StartTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTask-request)))
  "Returns string type for a service object of type 'StartTask-request"
  "launchman/StartTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTask-request>)))
  "Returns md5sum for a message object of type '<StartTask-request>"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTask-request)))
  "Returns md5sum for a message object of type 'StartTask-request"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTask-request>)))
  "Returns full string definition for message of type '<StartTask-request>"
  (cl:format cl:nil "string taskid~%string username~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTask-request)))
  "Returns full string definition for message of type 'StartTask-request"
  (cl:format cl:nil "string taskid~%string username~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTask-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'taskid))
     4 (cl:length (cl:slot-value msg 'username))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTask-request
    (cl:cons ':taskid (taskid msg))
    (cl:cons ':username (username msg))
))
;//! \htmlinclude StartTask-response.msg.html

(cl:defclass <StartTask-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass StartTask-response (<StartTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<StartTask-response> is deprecated: use launchman-srv:StartTask-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <StartTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:status-val is deprecated.  Use launchman-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTask-response>) ostream)
  "Serializes a message object of type '<StartTask-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTask-response>) istream)
  "Deserializes a message object of type '<StartTask-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTask-response>)))
  "Returns string type for a service object of type '<StartTask-response>"
  "launchman/StartTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTask-response)))
  "Returns string type for a service object of type 'StartTask-response"
  "launchman/StartTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTask-response>)))
  "Returns md5sum for a message object of type '<StartTask-response>"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTask-response)))
  "Returns md5sum for a message object of type 'StartTask-response"
  "a7f7c2a0ff94dc94508b68c526bc0b69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTask-response>)))
  "Returns full string definition for message of type '<StartTask-response>"
  (cl:format cl:nil "string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTask-response)))
  "Returns full string definition for message of type 'StartTask-response"
  (cl:format cl:nil "string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTask-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTask-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartTask)))
  'StartTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartTask)))
  'StartTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTask)))
  "Returns string type for a service object of type '<StartTask>"
  "launchman/StartTask")