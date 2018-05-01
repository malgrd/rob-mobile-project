; Auto-generated. Do not edit!


(cl:in-package robmobile_projet-msg)


;//! \htmlinclude Tab_point.msg.html

(cl:defclass <Tab_point> (roslisp-msg-protocol:ros-message)
  ((tab_points_X
    :reader tab_points_X
    :initarg :tab_points_X
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (tab_points_Y
    :reader tab_points_Y
    :initarg :tab_points_Y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (size
    :reader size
    :initarg :size
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Tab_point (<Tab_point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tab_point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tab_point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robmobile_projet-msg:<Tab_point> is deprecated: use robmobile_projet-msg:Tab_point instead.")))

(cl:ensure-generic-function 'tab_points_X-val :lambda-list '(m))
(cl:defmethod tab_points_X-val ((m <Tab_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robmobile_projet-msg:tab_points_X-val is deprecated.  Use robmobile_projet-msg:tab_points_X instead.")
  (tab_points_X m))

(cl:ensure-generic-function 'tab_points_Y-val :lambda-list '(m))
(cl:defmethod tab_points_Y-val ((m <Tab_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robmobile_projet-msg:tab_points_Y-val is deprecated.  Use robmobile_projet-msg:tab_points_Y instead.")
  (tab_points_Y m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <Tab_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robmobile_projet-msg:size-val is deprecated.  Use robmobile_projet-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tab_point>) ostream)
  "Serializes a message object of type '<Tab_point>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tab_points_X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tab_points_X))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tab_points_Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tab_points_Y))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tab_point>) istream)
  "Deserializes a message object of type '<Tab_point>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tab_points_X) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tab_points_X)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tab_points_Y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tab_points_Y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tab_point>)))
  "Returns string type for a message object of type '<Tab_point>"
  "robmobile_projet/Tab_point")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tab_point)))
  "Returns string type for a message object of type 'Tab_point"
  "robmobile_projet/Tab_point")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tab_point>)))
  "Returns md5sum for a message object of type '<Tab_point>"
  "0c3ab8d762f1957274b3be318659dca4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tab_point)))
  "Returns md5sum for a message object of type 'Tab_point"
  "0c3ab8d762f1957274b3be318659dca4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tab_point>)))
  "Returns full string definition for message of type '<Tab_point>"
  (cl:format cl:nil "float32[] tab_points_X~%float32[] tab_points_Y~%uint8 size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tab_point)))
  "Returns full string definition for message of type 'Tab_point"
  (cl:format cl:nil "float32[] tab_points_X~%float32[] tab_points_Y~%uint8 size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tab_point>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tab_points_X) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tab_points_Y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tab_point>))
  "Converts a ROS message object to a list"
  (cl:list 'Tab_point
    (cl:cons ':tab_points_X (tab_points_X msg))
    (cl:cons ':tab_points_Y (tab_points_Y msg))
    (cl:cons ':size (size msg))
))
