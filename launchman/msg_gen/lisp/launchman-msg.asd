
(cl:in-package :asdf)

(defsystem "launchman-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AppUpdate" :depends-on ("_package_AppUpdate"))
    (:file "_package_AppUpdate" :depends-on ("_package"))
    (:file "AppStatus" :depends-on ("_package_AppStatus"))
    (:file "_package_AppStatus" :depends-on ("_package"))
    (:file "Application" :depends-on ("_package_Application"))
    (:file "_package_Application" :depends-on ("_package"))
  ))