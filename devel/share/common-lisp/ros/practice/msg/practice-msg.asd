
(cl:in-package :asdf)

(defsystem "practice-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "information" :depends-on ("_package_information"))
    (:file "_package_information" :depends-on ("_package"))
  ))