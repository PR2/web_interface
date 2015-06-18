; Auto-generated. Do not edit!


(cl:in-package launchman-srv)


;//! \htmlinclude StatusUpdate-request.msg.html

(cl:defclass <StatusUpdate-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StatusUpdate-request (<StatusUpdate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StatusUpdate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StatusUpdate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<StatusUpdate-request> is deprecated: use launchman-srv:StatusUpdate-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StatusUpdate-request>) ostream)
  "Serializes a message object of type '<StatusUpdate-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StatusUpdate-request>) istream)
  "Deserializes a message object of type '<StatusUpdate-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StatusUpdate-request>)))
  "Returns string type for a service object of type '<StatusUpdate-request>"
  "launchman/StatusUpdateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusUpdate-request)))
  "Returns string type for a service object of type 'StatusUpdate-request"
  "launchman/StatusUpdateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StatusUpdate-request>)))
  "Returns md5sum for a message object of type '<StatusUpdate-request>"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StatusUpdate-request)))
  "Returns md5sum for a message object of type 'StatusUpdate-request"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StatusUpdate-request>)))
  "Returns full string definition for message of type '<StatusUpdate-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StatusUpdate-request)))
  "Returns full string definition for message of type 'StatusUpdate-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StatusUpdate-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StatusUpdate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StatusUpdate-request
))
;//! \htmlinclude StatusUpdate-response.msg.html

(cl:defclass <StatusUpdate-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass StatusUpdate-response (<StatusUpdate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StatusUpdate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StatusUpdate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-srv:<StatusUpdate-response> is deprecated: use launchman-srv:StatusUpdate-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <StatusUpdate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-srv:status-val is deprecated.  Use launchman-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StatusUpdate-response>) ostream)
  "Serializes a message object of type '<StatusUpdate-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StatusUpdate-response>) istream)
  "Deserializes a message object of type '<StatusUpdate-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StatusUpdate-response>)))
  "Returns string type for a service object of type '<StatusUpdate-response>"
  "launchman/StatusUpdateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusUpdate-response)))
  "Returns string type for a service object of type 'StatusUpdate-response"
  "launchman/StatusUpdateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StatusUpdate-response>)))
  "Returns md5sum for a message object of type '<StatusUpdate-response>"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StatusUpdate-response)))
  "Returns md5sum for a message object of type 'StatusUpdate-response"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StatusUpdate-response>)))
  "Returns full string definition for message of type '<StatusUpdate-response>"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StatusUpdate-response)))
  "Returns full string definition for message of type 'StatusUpdate-response"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StatusUpdate-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StatusUpdate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StatusUpdate-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StatusUpdate)))
  'StatusUpdate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StatusUpdate)))
  'StatusUpdate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusUpdate)))
  "Returns string type for a service object of type '<StatusUpdate>"
  "launchman/StatusUpdate")