
(cl:in-package :asdf)

(defsystem "serial_manager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Param" :depends-on ("_package_Param"))
    (:file "_package_Param" :depends-on ("_package"))
    (:file "distance_msg" :depends-on ("_package_distance_msg"))
    (:file "_package_distance_msg" :depends-on ("_package"))
    (:file "pippo" :depends-on ("_package_pippo"))
    (:file "_package_pippo" :depends-on ("_package"))
  ))