; Auto-generated. Do not edit!


(cl:in-package web_msgs-msg)


;//! \htmlinclude WebEvent.msg.html

(cl:defclass <WebEvent> (roslisp-msg-protocol:ros-message)
  ((source
    :reader source
    :initarg :source
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass WebEvent (<WebEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WebEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WebEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name web_msgs-msg:<WebEvent> is deprecated: use web_msgs-msg:WebEvent instead.")))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <WebEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader web_msgs-msg:source-val is deprecated.  Use web_msgs-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <WebEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader web_msgs-msg:type-val is deprecated.  Use web_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <WebEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader web_msgs-msg:data-val is deprecated.  Use web_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WebEvent>) ostream)
  "Serializes a message object of type '<WebEvent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WebEvent>) istream)
  "Deserializes a message object of type '<WebEvent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WebEvent>)))
  "Returns string type for a message object of type '<WebEvent>"
  "web_msgs/WebEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WebEvent)))
  "Returns string type for a message object of type 'WebEvent"
  "web_msgs/WebEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WebEvent>)))
  "Returns md5sum for a message object of type '<WebEvent>"
  "4f05e2e1608bf7d788fa5e654f805aeb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WebEvent)))
  "Returns md5sum for a message object of type 'WebEvent"
  "4f05e2e1608bf7d788fa5e654f805aeb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WebEvent>)))
  "Returns full string definition for message of type '<WebEvent>"
  (cl:format cl:nil "string source		# who or what triggered the event~%string type		# type of event (e.g. login, logout)~%string data		# any data about the event~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WebEvent)))
  "Returns full string definition for message of type 'WebEvent"
  (cl:format cl:nil "string source		# who or what triggered the event~%string type		# type of event (e.g. login, logout)~%string data		# any data about the event~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WebEvent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'source))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WebEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'WebEvent
    (cl:cons ':source (source msg))
    (cl:cons ':type (type msg))
    (cl:cons ':data (data msg))
))
