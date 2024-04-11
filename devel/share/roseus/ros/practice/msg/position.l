;; Auto-generated. Do not edit!


(when (boundp 'practice::position)
  (if (not (find-package "PRACTICE"))
    (make-package "PRACTICE"))
  (shadow 'position (find-package "PRACTICE")))
(unless (find-package "PRACTICE::POSITION")
  (make-package "PRACTICE::POSITION"))

(in-package "ROS")
;;//! \htmlinclude position.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass practice::position
  :super ros::object
  :slots (_header _uav0_px _uav0_py _uav0_pz _uav1_px _uav1_py _uav1_pz _uav2_px _uav2_py _uav2_pz _g0_px _g0_py _g0_pz _g1_px _g1_py _g1_pz _g2_px _g2_py _g2_pz _time _uav0_pid_vx _uav0_pid_vy _uav0_pid_vz _uav0_aS _uav0_a_vx _uav0_a_vy _uav0_a_vz _uav0_cS _uav0_c_vx _uav0_c_vy _uav0_c_vz _uav1_pid_vx _uav1_pid_vy _uav1_pid_vz _uav1_aS _uav1_a_vx _uav1_a_vy _uav1_a_vz _uav1_cS _uav1_c_vx _uav1_c_vy _uav1_c_vz _uav2_pid_vx _uav2_pid_vy _uav2_pid_vz _uav2_aS _uav2_a_vx _uav2_a_vy _uav2_a_vz _uav2_cS _uav2_c_vx _uav2_c_vy _uav2_c_vz ))

(defmethod practice::position
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uav0_px __uav0_px) 0.0)
    ((:uav0_py __uav0_py) 0.0)
    ((:uav0_pz __uav0_pz) 0.0)
    ((:uav1_px __uav1_px) 0.0)
    ((:uav1_py __uav1_py) 0.0)
    ((:uav1_pz __uav1_pz) 0.0)
    ((:uav2_px __uav2_px) 0.0)
    ((:uav2_py __uav2_py) 0.0)
    ((:uav2_pz __uav2_pz) 0.0)
    ((:g0_px __g0_px) 0.0)
    ((:g0_py __g0_py) 0.0)
    ((:g0_pz __g0_pz) 0.0)
    ((:g1_px __g1_px) 0.0)
    ((:g1_py __g1_py) 0.0)
    ((:g1_pz __g1_pz) 0.0)
    ((:g2_px __g2_px) 0.0)
    ((:g2_py __g2_py) 0.0)
    ((:g2_pz __g2_pz) 0.0)
    ((:time __time) 0.0)
    ((:uav0_pid_vx __uav0_pid_vx) 0.0)
    ((:uav0_pid_vy __uav0_pid_vy) 0.0)
    ((:uav0_pid_vz __uav0_pid_vz) 0.0)
    ((:uav0_aS __uav0_aS) 0.0)
    ((:uav0_a_vx __uav0_a_vx) 0.0)
    ((:uav0_a_vy __uav0_a_vy) 0.0)
    ((:uav0_a_vz __uav0_a_vz) 0.0)
    ((:uav0_cS __uav0_cS) 0.0)
    ((:uav0_c_vx __uav0_c_vx) 0.0)
    ((:uav0_c_vy __uav0_c_vy) 0.0)
    ((:uav0_c_vz __uav0_c_vz) 0.0)
    ((:uav1_pid_vx __uav1_pid_vx) 0.0)
    ((:uav1_pid_vy __uav1_pid_vy) 0.0)
    ((:uav1_pid_vz __uav1_pid_vz) 0.0)
    ((:uav1_aS __uav1_aS) 0.0)
    ((:uav1_a_vx __uav1_a_vx) 0.0)
    ((:uav1_a_vy __uav1_a_vy) 0.0)
    ((:uav1_a_vz __uav1_a_vz) 0.0)
    ((:uav1_cS __uav1_cS) 0.0)
    ((:uav1_c_vx __uav1_c_vx) 0.0)
    ((:uav1_c_vy __uav1_c_vy) 0.0)
    ((:uav1_c_vz __uav1_c_vz) 0.0)
    ((:uav2_pid_vx __uav2_pid_vx) 0.0)
    ((:uav2_pid_vy __uav2_pid_vy) 0.0)
    ((:uav2_pid_vz __uav2_pid_vz) 0.0)
    ((:uav2_aS __uav2_aS) 0.0)
    ((:uav2_a_vx __uav2_a_vx) 0.0)
    ((:uav2_a_vy __uav2_a_vy) 0.0)
    ((:uav2_a_vz __uav2_a_vz) 0.0)
    ((:uav2_cS __uav2_cS) 0.0)
    ((:uav2_c_vx __uav2_c_vx) 0.0)
    ((:uav2_c_vy __uav2_c_vy) 0.0)
    ((:uav2_c_vz __uav2_c_vz) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _uav0_px (float __uav0_px))
   (setq _uav0_py (float __uav0_py))
   (setq _uav0_pz (float __uav0_pz))
   (setq _uav1_px (float __uav1_px))
   (setq _uav1_py (float __uav1_py))
   (setq _uav1_pz (float __uav1_pz))
   (setq _uav2_px (float __uav2_px))
   (setq _uav2_py (float __uav2_py))
   (setq _uav2_pz (float __uav2_pz))
   (setq _g0_px (float __g0_px))
   (setq _g0_py (float __g0_py))
   (setq _g0_pz (float __g0_pz))
   (setq _g1_px (float __g1_px))
   (setq _g1_py (float __g1_py))
   (setq _g1_pz (float __g1_pz))
   (setq _g2_px (float __g2_px))
   (setq _g2_py (float __g2_py))
   (setq _g2_pz (float __g2_pz))
   (setq _time (float __time))
   (setq _uav0_pid_vx (float __uav0_pid_vx))
   (setq _uav0_pid_vy (float __uav0_pid_vy))
   (setq _uav0_pid_vz (float __uav0_pid_vz))
   (setq _uav0_aS (float __uav0_aS))
   (setq _uav0_a_vx (float __uav0_a_vx))
   (setq _uav0_a_vy (float __uav0_a_vy))
   (setq _uav0_a_vz (float __uav0_a_vz))
   (setq _uav0_cS (float __uav0_cS))
   (setq _uav0_c_vx (float __uav0_c_vx))
   (setq _uav0_c_vy (float __uav0_c_vy))
   (setq _uav0_c_vz (float __uav0_c_vz))
   (setq _uav1_pid_vx (float __uav1_pid_vx))
   (setq _uav1_pid_vy (float __uav1_pid_vy))
   (setq _uav1_pid_vz (float __uav1_pid_vz))
   (setq _uav1_aS (float __uav1_aS))
   (setq _uav1_a_vx (float __uav1_a_vx))
   (setq _uav1_a_vy (float __uav1_a_vy))
   (setq _uav1_a_vz (float __uav1_a_vz))
   (setq _uav1_cS (float __uav1_cS))
   (setq _uav1_c_vx (float __uav1_c_vx))
   (setq _uav1_c_vy (float __uav1_c_vy))
   (setq _uav1_c_vz (float __uav1_c_vz))
   (setq _uav2_pid_vx (float __uav2_pid_vx))
   (setq _uav2_pid_vy (float __uav2_pid_vy))
   (setq _uav2_pid_vz (float __uav2_pid_vz))
   (setq _uav2_aS (float __uav2_aS))
   (setq _uav2_a_vx (float __uav2_a_vx))
   (setq _uav2_a_vy (float __uav2_a_vy))
   (setq _uav2_a_vz (float __uav2_a_vz))
   (setq _uav2_cS (float __uav2_cS))
   (setq _uav2_c_vx (float __uav2_c_vx))
   (setq _uav2_c_vy (float __uav2_c_vy))
   (setq _uav2_c_vz (float __uav2_c_vz))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:uav0_px
   (&optional __uav0_px)
   (if __uav0_px (setq _uav0_px __uav0_px)) _uav0_px)
  (:uav0_py
   (&optional __uav0_py)
   (if __uav0_py (setq _uav0_py __uav0_py)) _uav0_py)
  (:uav0_pz
   (&optional __uav0_pz)
   (if __uav0_pz (setq _uav0_pz __uav0_pz)) _uav0_pz)
  (:uav1_px
   (&optional __uav1_px)
   (if __uav1_px (setq _uav1_px __uav1_px)) _uav1_px)
  (:uav1_py
   (&optional __uav1_py)
   (if __uav1_py (setq _uav1_py __uav1_py)) _uav1_py)
  (:uav1_pz
   (&optional __uav1_pz)
   (if __uav1_pz (setq _uav1_pz __uav1_pz)) _uav1_pz)
  (:uav2_px
   (&optional __uav2_px)
   (if __uav2_px (setq _uav2_px __uav2_px)) _uav2_px)
  (:uav2_py
   (&optional __uav2_py)
   (if __uav2_py (setq _uav2_py __uav2_py)) _uav2_py)
  (:uav2_pz
   (&optional __uav2_pz)
   (if __uav2_pz (setq _uav2_pz __uav2_pz)) _uav2_pz)
  (:g0_px
   (&optional __g0_px)
   (if __g0_px (setq _g0_px __g0_px)) _g0_px)
  (:g0_py
   (&optional __g0_py)
   (if __g0_py (setq _g0_py __g0_py)) _g0_py)
  (:g0_pz
   (&optional __g0_pz)
   (if __g0_pz (setq _g0_pz __g0_pz)) _g0_pz)
  (:g1_px
   (&optional __g1_px)
   (if __g1_px (setq _g1_px __g1_px)) _g1_px)
  (:g1_py
   (&optional __g1_py)
   (if __g1_py (setq _g1_py __g1_py)) _g1_py)
  (:g1_pz
   (&optional __g1_pz)
   (if __g1_pz (setq _g1_pz __g1_pz)) _g1_pz)
  (:g2_px
   (&optional __g2_px)
   (if __g2_px (setq _g2_px __g2_px)) _g2_px)
  (:g2_py
   (&optional __g2_py)
   (if __g2_py (setq _g2_py __g2_py)) _g2_py)
  (:g2_pz
   (&optional __g2_pz)
   (if __g2_pz (setq _g2_pz __g2_pz)) _g2_pz)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:uav0_pid_vx
   (&optional __uav0_pid_vx)
   (if __uav0_pid_vx (setq _uav0_pid_vx __uav0_pid_vx)) _uav0_pid_vx)
  (:uav0_pid_vy
   (&optional __uav0_pid_vy)
   (if __uav0_pid_vy (setq _uav0_pid_vy __uav0_pid_vy)) _uav0_pid_vy)
  (:uav0_pid_vz
   (&optional __uav0_pid_vz)
   (if __uav0_pid_vz (setq _uav0_pid_vz __uav0_pid_vz)) _uav0_pid_vz)
  (:uav0_aS
   (&optional __uav0_aS)
   (if __uav0_aS (setq _uav0_aS __uav0_aS)) _uav0_aS)
  (:uav0_a_vx
   (&optional __uav0_a_vx)
   (if __uav0_a_vx (setq _uav0_a_vx __uav0_a_vx)) _uav0_a_vx)
  (:uav0_a_vy
   (&optional __uav0_a_vy)
   (if __uav0_a_vy (setq _uav0_a_vy __uav0_a_vy)) _uav0_a_vy)
  (:uav0_a_vz
   (&optional __uav0_a_vz)
   (if __uav0_a_vz (setq _uav0_a_vz __uav0_a_vz)) _uav0_a_vz)
  (:uav0_cS
   (&optional __uav0_cS)
   (if __uav0_cS (setq _uav0_cS __uav0_cS)) _uav0_cS)
  (:uav0_c_vx
   (&optional __uav0_c_vx)
   (if __uav0_c_vx (setq _uav0_c_vx __uav0_c_vx)) _uav0_c_vx)
  (:uav0_c_vy
   (&optional __uav0_c_vy)
   (if __uav0_c_vy (setq _uav0_c_vy __uav0_c_vy)) _uav0_c_vy)
  (:uav0_c_vz
   (&optional __uav0_c_vz)
   (if __uav0_c_vz (setq _uav0_c_vz __uav0_c_vz)) _uav0_c_vz)
  (:uav1_pid_vx
   (&optional __uav1_pid_vx)
   (if __uav1_pid_vx (setq _uav1_pid_vx __uav1_pid_vx)) _uav1_pid_vx)
  (:uav1_pid_vy
   (&optional __uav1_pid_vy)
   (if __uav1_pid_vy (setq _uav1_pid_vy __uav1_pid_vy)) _uav1_pid_vy)
  (:uav1_pid_vz
   (&optional __uav1_pid_vz)
   (if __uav1_pid_vz (setq _uav1_pid_vz __uav1_pid_vz)) _uav1_pid_vz)
  (:uav1_aS
   (&optional __uav1_aS)
   (if __uav1_aS (setq _uav1_aS __uav1_aS)) _uav1_aS)
  (:uav1_a_vx
   (&optional __uav1_a_vx)
   (if __uav1_a_vx (setq _uav1_a_vx __uav1_a_vx)) _uav1_a_vx)
  (:uav1_a_vy
   (&optional __uav1_a_vy)
   (if __uav1_a_vy (setq _uav1_a_vy __uav1_a_vy)) _uav1_a_vy)
  (:uav1_a_vz
   (&optional __uav1_a_vz)
   (if __uav1_a_vz (setq _uav1_a_vz __uav1_a_vz)) _uav1_a_vz)
  (:uav1_cS
   (&optional __uav1_cS)
   (if __uav1_cS (setq _uav1_cS __uav1_cS)) _uav1_cS)
  (:uav1_c_vx
   (&optional __uav1_c_vx)
   (if __uav1_c_vx (setq _uav1_c_vx __uav1_c_vx)) _uav1_c_vx)
  (:uav1_c_vy
   (&optional __uav1_c_vy)
   (if __uav1_c_vy (setq _uav1_c_vy __uav1_c_vy)) _uav1_c_vy)
  (:uav1_c_vz
   (&optional __uav1_c_vz)
   (if __uav1_c_vz (setq _uav1_c_vz __uav1_c_vz)) _uav1_c_vz)
  (:uav2_pid_vx
   (&optional __uav2_pid_vx)
   (if __uav2_pid_vx (setq _uav2_pid_vx __uav2_pid_vx)) _uav2_pid_vx)
  (:uav2_pid_vy
   (&optional __uav2_pid_vy)
   (if __uav2_pid_vy (setq _uav2_pid_vy __uav2_pid_vy)) _uav2_pid_vy)
  (:uav2_pid_vz
   (&optional __uav2_pid_vz)
   (if __uav2_pid_vz (setq _uav2_pid_vz __uav2_pid_vz)) _uav2_pid_vz)
  (:uav2_aS
   (&optional __uav2_aS)
   (if __uav2_aS (setq _uav2_aS __uav2_aS)) _uav2_aS)
  (:uav2_a_vx
   (&optional __uav2_a_vx)
   (if __uav2_a_vx (setq _uav2_a_vx __uav2_a_vx)) _uav2_a_vx)
  (:uav2_a_vy
   (&optional __uav2_a_vy)
   (if __uav2_a_vy (setq _uav2_a_vy __uav2_a_vy)) _uav2_a_vy)
  (:uav2_a_vz
   (&optional __uav2_a_vz)
   (if __uav2_a_vz (setq _uav2_a_vz __uav2_a_vz)) _uav2_a_vz)
  (:uav2_cS
   (&optional __uav2_cS)
   (if __uav2_cS (setq _uav2_cS __uav2_cS)) _uav2_cS)
  (:uav2_c_vx
   (&optional __uav2_c_vx)
   (if __uav2_c_vx (setq _uav2_c_vx __uav2_c_vx)) _uav2_c_vx)
  (:uav2_c_vy
   (&optional __uav2_c_vy)
   (if __uav2_c_vy (setq _uav2_c_vy __uav2_c_vy)) _uav2_c_vy)
  (:uav2_c_vz
   (&optional __uav2_c_vz)
   (if __uav2_c_vz (setq _uav2_c_vz __uav2_c_vz)) _uav2_c_vz)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _uav0_px
    8
    ;; float64 _uav0_py
    8
    ;; float64 _uav0_pz
    8
    ;; float64 _uav1_px
    8
    ;; float64 _uav1_py
    8
    ;; float64 _uav1_pz
    8
    ;; float64 _uav2_px
    8
    ;; float64 _uav2_py
    8
    ;; float64 _uav2_pz
    8
    ;; float64 _g0_px
    8
    ;; float64 _g0_py
    8
    ;; float64 _g0_pz
    8
    ;; float64 _g1_px
    8
    ;; float64 _g1_py
    8
    ;; float64 _g1_pz
    8
    ;; float64 _g2_px
    8
    ;; float64 _g2_py
    8
    ;; float64 _g2_pz
    8
    ;; float64 _time
    8
    ;; float64 _uav0_pid_vx
    8
    ;; float64 _uav0_pid_vy
    8
    ;; float64 _uav0_pid_vz
    8
    ;; float64 _uav0_aS
    8
    ;; float64 _uav0_a_vx
    8
    ;; float64 _uav0_a_vy
    8
    ;; float64 _uav0_a_vz
    8
    ;; float64 _uav0_cS
    8
    ;; float64 _uav0_c_vx
    8
    ;; float64 _uav0_c_vy
    8
    ;; float64 _uav0_c_vz
    8
    ;; float64 _uav1_pid_vx
    8
    ;; float64 _uav1_pid_vy
    8
    ;; float64 _uav1_pid_vz
    8
    ;; float64 _uav1_aS
    8
    ;; float64 _uav1_a_vx
    8
    ;; float64 _uav1_a_vy
    8
    ;; float64 _uav1_a_vz
    8
    ;; float64 _uav1_cS
    8
    ;; float64 _uav1_c_vx
    8
    ;; float64 _uav1_c_vy
    8
    ;; float64 _uav1_c_vz
    8
    ;; float64 _uav2_pid_vx
    8
    ;; float64 _uav2_pid_vy
    8
    ;; float64 _uav2_pid_vz
    8
    ;; float64 _uav2_aS
    8
    ;; float64 _uav2_a_vx
    8
    ;; float64 _uav2_a_vy
    8
    ;; float64 _uav2_a_vz
    8
    ;; float64 _uav2_cS
    8
    ;; float64 _uav2_c_vx
    8
    ;; float64 _uav2_c_vy
    8
    ;; float64 _uav2_c_vz
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _uav0_px
       (sys::poke _uav0_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_py
       (sys::poke _uav0_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_pz
       (sys::poke _uav0_pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_px
       (sys::poke _uav1_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_py
       (sys::poke _uav1_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_pz
       (sys::poke _uav1_pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_px
       (sys::poke _uav2_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_py
       (sys::poke _uav2_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_pz
       (sys::poke _uav2_pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g0_px
       (sys::poke _g0_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g0_py
       (sys::poke _g0_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g0_pz
       (sys::poke _g0_pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g1_px
       (sys::poke _g1_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g1_py
       (sys::poke _g1_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g1_pz
       (sys::poke _g1_pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g2_px
       (sys::poke _g2_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g2_py
       (sys::poke _g2_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g2_pz
       (sys::poke _g2_pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_pid_vx
       (sys::poke _uav0_pid_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_pid_vy
       (sys::poke _uav0_pid_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_pid_vz
       (sys::poke _uav0_pid_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_aS
       (sys::poke _uav0_aS (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_a_vx
       (sys::poke _uav0_a_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_a_vy
       (sys::poke _uav0_a_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_a_vz
       (sys::poke _uav0_a_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_cS
       (sys::poke _uav0_cS (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_c_vx
       (sys::poke _uav0_c_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_c_vy
       (sys::poke _uav0_c_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav0_c_vz
       (sys::poke _uav0_c_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_pid_vx
       (sys::poke _uav1_pid_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_pid_vy
       (sys::poke _uav1_pid_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_pid_vz
       (sys::poke _uav1_pid_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_aS
       (sys::poke _uav1_aS (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_a_vx
       (sys::poke _uav1_a_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_a_vy
       (sys::poke _uav1_a_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_a_vz
       (sys::poke _uav1_a_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_cS
       (sys::poke _uav1_cS (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_c_vx
       (sys::poke _uav1_c_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_c_vy
       (sys::poke _uav1_c_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav1_c_vz
       (sys::poke _uav1_c_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_pid_vx
       (sys::poke _uav2_pid_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_pid_vy
       (sys::poke _uav2_pid_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_pid_vz
       (sys::poke _uav2_pid_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_aS
       (sys::poke _uav2_aS (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_a_vx
       (sys::poke _uav2_a_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_a_vy
       (sys::poke _uav2_a_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_a_vz
       (sys::poke _uav2_a_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_cS
       (sys::poke _uav2_cS (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_c_vx
       (sys::poke _uav2_c_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_c_vy
       (sys::poke _uav2_c_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uav2_c_vz
       (sys::poke _uav2_c_vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _uav0_px
     (setq _uav0_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_py
     (setq _uav0_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_pz
     (setq _uav0_pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_px
     (setq _uav1_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_py
     (setq _uav1_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_pz
     (setq _uav1_pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_px
     (setq _uav2_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_py
     (setq _uav2_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_pz
     (setq _uav2_pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g0_px
     (setq _g0_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g0_py
     (setq _g0_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g0_pz
     (setq _g0_pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g1_px
     (setq _g1_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g1_py
     (setq _g1_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g1_pz
     (setq _g1_pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g2_px
     (setq _g2_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g2_py
     (setq _g2_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g2_pz
     (setq _g2_pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_pid_vx
     (setq _uav0_pid_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_pid_vy
     (setq _uav0_pid_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_pid_vz
     (setq _uav0_pid_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_aS
     (setq _uav0_aS (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_a_vx
     (setq _uav0_a_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_a_vy
     (setq _uav0_a_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_a_vz
     (setq _uav0_a_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_cS
     (setq _uav0_cS (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_c_vx
     (setq _uav0_c_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_c_vy
     (setq _uav0_c_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav0_c_vz
     (setq _uav0_c_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_pid_vx
     (setq _uav1_pid_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_pid_vy
     (setq _uav1_pid_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_pid_vz
     (setq _uav1_pid_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_aS
     (setq _uav1_aS (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_a_vx
     (setq _uav1_a_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_a_vy
     (setq _uav1_a_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_a_vz
     (setq _uav1_a_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_cS
     (setq _uav1_cS (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_c_vx
     (setq _uav1_c_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_c_vy
     (setq _uav1_c_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav1_c_vz
     (setq _uav1_c_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_pid_vx
     (setq _uav2_pid_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_pid_vy
     (setq _uav2_pid_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_pid_vz
     (setq _uav2_pid_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_aS
     (setq _uav2_aS (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_a_vx
     (setq _uav2_a_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_a_vy
     (setq _uav2_a_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_a_vz
     (setq _uav2_a_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_cS
     (setq _uav2_cS (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_c_vx
     (setq _uav2_c_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_c_vy
     (setq _uav2_c_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uav2_c_vz
     (setq _uav2_c_vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get practice::position :md5sum-) "efbb7ed7259937659d1ca1eefb10b7e9")
(setf (get practice::position :datatype-) "practice/position")
(setf (get practice::position :definition-)
      "std_msgs/Header header


float64 uav0_px
float64 uav0_py
float64 uav0_pz

float64 uav1_px
float64 uav1_py
float64 uav1_pz

float64 uav2_px
float64 uav2_py
float64 uav2_pz

float64 g0_px
float64 g0_py
float64 g0_pz

float64 g1_px
float64 g1_py
float64 g1_pz

float64 g2_px
float64 g2_py
float64 g2_pz

float64 time

float64 uav0_pid_vx
float64 uav0_pid_vy
float64 uav0_pid_vz

float64 uav0_aS
float64 uav0_a_vx
float64 uav0_a_vy
float64 uav0_a_vz

float64 uav0_cS
float64 uav0_c_vx
float64 uav0_c_vy
float64 uav0_c_vz

float64 uav1_pid_vx
float64 uav1_pid_vy
float64 uav1_pid_vz

float64 uav1_aS
float64 uav1_a_vx
float64 uav1_a_vy
float64 uav1_a_vz

float64 uav1_cS
float64 uav1_c_vx
float64 uav1_c_vy
float64 uav1_c_vz

float64 uav2_pid_vx
float64 uav2_pid_vy
float64 uav2_pid_vz

float64 uav2_aS
float64 uav2_a_vx
float64 uav2_a_vy
float64 uav2_a_vz

float64 uav2_cS
float64 uav2_c_vx
float64 uav2_c_vy
float64 uav2_c_vz



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :practice/position "efbb7ed7259937659d1ca1eefb10b7e9")


