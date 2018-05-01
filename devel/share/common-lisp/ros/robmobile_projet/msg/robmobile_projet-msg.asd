
(cl:in-package :asdf)

(defsystem "robmobile_projet-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Tab_point" :depends-on ("_package_Tab_point"))
    (:file "_package_Tab_point" :depends-on ("_package"))
  ))