; Auto-generated. Do not edit!


(cl:in-package practice-msg)


;//! \htmlinclude information.msg.html

(cl:defclass <information> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass information (<information>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <information>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'information)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name practice-msg:<information> is deprecated: use practice-msg:information instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-msg:point-val is deprecated.  Use practice-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-msg:vel-val is deprecated.  Use practice-msg:vel instead.")
  (vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <information>) ostream)
  "Serializes a message object of type '<information>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <information>) istream)
  "Deserializes a message object of type '<information>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<information>)))
  "Returns string type for a message object of type '<information>"
  "practice/information")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'information)))
  "Returns string type for a message object of type 'information"
  "practice/information")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<information>)))
  "Returns md5sum for a message object of type '<information>"
  "8f9c1875cae7e508fddc892d11872297")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'information)))
  "Returns md5sum for a message object of type 'information"
  "8f9c1875cae7e508fddc892d11872297")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<information>)))
  "Returns full string definition for message of type '<information>"
  (cl:format cl:nil "geometry_msgs/Point point~%geometry_msgs/Vector3 vel~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'information)))
  "Returns full string definition for message of type 'information"
  (cl:format cl:nil "geometry_msgs/Point point~%geometry_msgs/Vector3 vel~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <information>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <information>))
  "Converts a ROS message object to a list"
  (cl:list 'information
    (cl:cons ':point (point msg))
    (cl:cons ':vel (vel msg))
))
