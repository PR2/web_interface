
(cl:in-package :asdf)

(defsystem "launchman-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StartTask" :depends-on ("_package_StartTask"))
    (:file "_package_StartTask" :depends-on ("_package"))
    (:file "ListTasks" :depends-on ("_package_ListTasks"))
    (:file "_package_ListTasks" :depends-on ("_package"))
    (:file "StatusUpdate" :depends-on ("_package_StatusUpdate"))
    (:file "_package_StatusUpdate" :depends-on ("_package"))
    (:file "StopTask" :depends-on ("_package_StopTask"))
    (:file "_package_StopTask" :depends-on ("_package"))
  ))