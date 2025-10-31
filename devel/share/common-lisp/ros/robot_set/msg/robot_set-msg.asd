
(cl:in-package :asdf)

(defsystem "robot_set-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TCPState" :depends-on ("_package_TCPState"))
    (:file "_package_TCPState" :depends-on ("_package"))
  ))