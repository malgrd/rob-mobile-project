; Auto-generated. Do not edit!


(cl:in-package traitement_carte-msg)


;//! \htmlinclude point_init.msg.html

(cl:defclass <point_init> (roslisp-msg-protocol:ros-message)
  ((point_init
    :reader point_init
    :initarg :point_init
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass point_init (<point_init>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <point_init>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'point_init)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traitement_carte-msg:<point_init> is deprecated: use traitement_carte-msg:point_init instead.")))

(cl:ensure-generic-function 'point_init-val :lambda-list '(m))
(cl:defmethod point_init-val ((m <point_init>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traitement_carte-msg:point_init-val is deprecated.  Use traitement_carte-msg:point_init instead.")
  (point_init m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <point_init>) ostream)
  "Serializes a message object of type '<point_init>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point_init) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <point_init>) istream)
  "Deserializes a message object of type '<point_init>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point_init) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<point_init>)))
  "Returns string type for a message object of type '<point_init>"
  "traitement_carte/point_init")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'point_init)))
  "Returns string type for a message object of type 'point_init"
  "traitement_carte/point_init")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<point_init>)))
  "Returns md5sum for a message object of type '<point_init>"
  "b1418312b1601ee4616c36827ab2854a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'point_init)))
  "Returns md5sum for a message object of type 'point_init"
  "b1418312b1601ee4616c36827ab2854a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<point_init>)))
  "Returns full string definition for message of type '<point_init>"
  (cl:format cl:nil "geometry_msgs/Point point_init~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'point_init)))
  "Returns full string definition for message of type 'point_init"
  (cl:format cl:nil "geometry_msgs/Point point_init~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <point_init>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point_init))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <point_init>))
  "Converts a ROS message object to a list"
  (cl:list 'point_init
    (cl:cons ':point_init (point_init msg))
))
