; Auto-generated. Do not edit!


(cl:in-package launchman-msg)


;//! \htmlinclude Application.msg.html

(cl:defclass <Application> (roslisp-msg-protocol:ros-message)
  ((taskid
    :reader taskid
    :initarg :taskid
    :type cl:string
    :initform "")
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform "")
   (icon
    :reader icon
    :initarg :icon
    :type cl:string
    :initform "")
   (provides
    :reader provides
    :initarg :provides
    :type cl:string
    :initform "")
   (depends
    :reader depends
    :initarg :depends
    :type cl:string
    :initform ""))
)

(cl:defclass Application (<Application>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Application>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Application)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name launchman-msg:<Application> is deprecated: use launchman-msg:Application instead.")))

(cl:ensure-generic-function 'taskid-val :lambda-list '(m))
(cl:defmethod taskid-val ((m <Application>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:taskid-val is deprecated.  Use launchman-msg:taskid instead.")
  (taskid m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Application>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:name-val is deprecated.  Use launchman-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Application>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:status-val is deprecated.  Use launchman-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'icon-val :lambda-list '(m))
(cl:defmethod icon-val ((m <Application>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:icon-val is deprecated.  Use launchman-msg:icon instead.")
  (icon m))

(cl:ensure-generic-function 'provides-val :lambda-list '(m))
(cl:defmethod provides-val ((m <Application>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:provides-val is deprecated.  Use launchman-msg:provides instead.")
  (provides m))

(cl:ensure-generic-function 'depends-val :lambda-list '(m))
(cl:defmethod depends-val ((m <Application>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader launchman-msg:depends-val is deprecated.  Use launchman-msg:depends instead.")
  (depends m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Application>) ostream)
  "Serializes a message object of type '<Application>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'taskid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'taskid))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'icon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'icon))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'provides))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'provides))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'depends))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'depends))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Application>) istream)
  "Deserializes a message object of type '<Application>"
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
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'icon) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'icon) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'provides) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'provides) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depends) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'depends) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Application>)))
  "Returns string type for a message object of type '<Application>"
  "launchman/Application")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Application)))
  "Returns string type for a message object of type 'Application"
  "launchman/Application")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Application>)))
  "Returns md5sum for a message object of type '<Application>"
  "f6a16a9c297a883b8eb15bf869d26eca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Application)))
  "Returns md5sum for a message object of type 'Application"
  "f6a16a9c297a883b8eb15bf869d26eca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Application>)))
  "Returns full string definition for message of type '<Application>"
  (cl:format cl:nil "string taskid~%string name~%string status~%string icon~%string provides~%string depends~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Application)))
  "Returns full string definition for message of type 'Application"
  (cl:format cl:nil "string taskid~%string name~%string status~%string icon~%string provides~%string depends~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Application>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'taskid))
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'status))
     4 (cl:length (cl:slot-value msg 'icon))
     4 (cl:length (cl:slot-value msg 'provides))
     4 (cl:length (cl:slot-value msg 'depends))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Application>))
  "Converts a ROS message object to a list"
  (cl:list 'Application
    (cl:cons ':taskid (taskid msg))
    (cl:cons ':name (name msg))
    (cl:cons ':status (status msg))
    (cl:cons ':icon (icon msg))
    (cl:cons ':provides (provides msg))
    (cl:cons ':depends (depends msg))
))
