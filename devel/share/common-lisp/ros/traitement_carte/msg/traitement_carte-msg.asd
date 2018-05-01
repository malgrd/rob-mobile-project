
(cl:in-package :asdf)

(defsystem "traitement_carte-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "point_init" :depends-on ("_package_point_init"))
    (:file "_package_point_init" :depends-on ("_package"))
  ))