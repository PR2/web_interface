
(cl:in-package :asdf)

(defsystem "web_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WebEvent" :depends-on ("_package_WebEvent"))
    (:file "_package_WebEvent" :depends-on ("_package"))
  ))