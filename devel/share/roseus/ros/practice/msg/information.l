;; Auto-generated. Do not edit!


(when (boundp 'practice::information)
  (if (not (find-package "PRACTICE"))
    (make-package "PRACTICE"))
  (shadow 'information (find-package "PRACTICE")))
(unless (find-package "PRACTICE::INFORMATION")
  (make-package "PRACTICE::INFORMATION"))

(in-package "ROS")
;;//! \htmlinclude information.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass practice::information
  :super ros::object
  :slots (_point _vel ))

(defmethod practice::information
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:vel __vel) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _point __point)
   (setq _vel __vel)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:vel
   (&rest __vel)
   (if (keywordp (car __vel))
       (send* _vel __vel)
     (progn
       (if __vel (setq _vel (car __vel)))
       _vel)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; geometry_msgs/Vector3 _vel
    (send _vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; geometry_msgs/Vector3 _vel
       (send _vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; geometry_msgs/Vector3 _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;;
   self)
  )

(setf (get practice::information :md5sum-) "8f9c1875cae7e508fddc892d11872297")
(setf (get practice::information :datatype-) "practice/information")
(setf (get practice::information :definition-)
      "geometry_msgs/Point point
geometry_msgs/Vector3 vel


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :practice/information "8f9c1875cae7e508fddc892d11872297")


