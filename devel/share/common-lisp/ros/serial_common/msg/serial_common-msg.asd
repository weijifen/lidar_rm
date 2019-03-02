
(cl:in-package :asdf)

(defsystem "serial_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "wind_mill_msg" :depends-on ("_package_wind_mill_msg"))
    (:file "_package_wind_mill_msg" :depends-on ("_package"))
  ))