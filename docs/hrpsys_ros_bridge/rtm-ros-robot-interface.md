### rtm-ros-robot-interface
- :super **robot-interface**
- :slots nil



#### :init
&nbsp;&nbsp;&nbsp;*&rest* *args* 

- :init method. This method should be overriden in subclass. <br>


#### :force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns :force-vector [N] list for all limbs obtained by :state. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns :moment-vector [Nm] list for all limbs obtained by :state. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :off-force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :force-vector [N] list for all limbs obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :off-moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :moment-vector [Nm] list for all limbs obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :reference-force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns reference force-vector [N] list for all limbs obtained by :state. <br>
    This value corresponds to StateHolder and SequencePlayer RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :reference-moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns reference moment-vector [Nm] list for all limbs obtained by :state. <br>
    This value corresponds to StateHolder and SequencePlayer RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :absolute-force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :force-vector [N] list for all limbs in world frame obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :absolute-moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :moment-vector [Nm] list for all limbs in world frame obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :zmp-vector
&nbsp;&nbsp;&nbsp;*&optional* *(wrt :local)* 

- Returns zmp vector [mm]. <br>
    If wrt is :local, returns zmp in the base-link frame. If wrt is :world, returns zmp in the world frame. <br>


#### :ref-capture-point-vector
&nbsp;&nbsp;&nbsp;*&optional* *(wrt :local)* 

- Returns ref-capture-point vector [mm]. <br>
    If wrt is :local, returns ref-capture-point in the base-link frame. If wrt is :world, returns ref-capture-point in the world frame. <br>


#### :act-capture-point-vector
&nbsp;&nbsp;&nbsp;*&optional* *(wrt :local)* 

- Returns act-capture-point vector [mm]. <br>
    If wrt is :local, returns act-capture-point in the base-link frame. If wrt is :world, returns act-capture-point in the world frame. <br>


#### :ref-contact-states


- Returns contact states from AutoBalancer. <br>


#### :act-contact-states


- Returns contact states from Stabilizer. <br>


#### :temperature-vector


- Returns temperature vector. <br>


#### :motor-extra-data


- Returns motor extra data. Please see iob definition for each system. <br>


#### :imucoords


- Returns robot's coords based on imu measurement. <br>


#### :accel-vector


- Returns acceleration [m/s2] of the acceleration sensor. <br>


#### :gyro-vector


- Returns angular velocity [rad/s] of the gyro sensor. <br>


#### :state
&nbsp;&nbsp;&nbsp;*&rest* *args* 

- Obtains sensor and robot command topics using spin-once. <br>


#### :set-interpolation-mode
&nbsp;&nbsp;&nbsp;*interpolation-mode* 

- Set interpolation mode for SequencePlayer. <br>


#### :calc-zmp-from-state
&nbsp;&nbsp;&nbsp;*&key* *(wrt :world)* 

- Calculate zmp from state [mm]. <br>
    For example ;; (progn (send *ri* :go-velocity 0 0 0) (objects (list (*ri* . robot))) (do-until-key (let ((zmp (send *ri* :calc-zmp-from-state))) (send *irtviewer* :draw-objects :flush nil) (send zmp :draw-on :flush t :size 300)))) <br>
    :wrt is :local => calc local zmp for (*ri* . robot)'s root-link coords <br>
    :wrt is :world => calc world zmp for (*ri* . robot) <br>


#### :get-robot-date-string


- Get string including robot name and date. <br>
    For example, "SampleRobot_20160412163151". <br>


#### :def-limb-controller-method
&nbsp;&nbsp;&nbsp;*limb* *&key* *(debugp nil)* 

- Method to add limb controller action by default setting. <br>
     Currently, FollowJointTrajectoryAction is used. <br>
     This method calls defmethod. If :debugp t, we can see defmethod s-expressions. <br>


#### :set-base-coords
&nbsp;&nbsp;&nbsp;*base-coords* *tm* 

- Set base coordinates in the world frame. <br>
    base-coords is Euslisp coords and tm is [ms]. <br>


#### :set-base-pos
&nbsp;&nbsp;&nbsp;*base-pos* *tm* 

- Set base pos in the world frame. <br>
    base-pos is [mm] and tm is [ms]. <br>


#### :set-base-rpy
&nbsp;&nbsp;&nbsp;*base-rpy* *tm* 

- Set base rpy in the world frame. <br>
    base-rpy is [rad] and tm is [ms]. <br>


#### :wait-interpolation-of-group
&nbsp;&nbsp;&nbsp;*groupname* 

- Wait interpolation of group. <br>
    !!This method is not recommended, please use :wait-interpolation method like (send *ri* :wait-interpolation :head-controller).!! <br>


#### :add-joint-group
&nbsp;&nbsp;&nbsp;*groupname* *&optional* *(jnames (if (find-method self (read-from-string (format nil :~A-controller (string-downcase groupname)))) (cdr (assoc :joint-names (car (send self (read-from-string (format nil :~A-controller (string-downcase groupname)))))))))* 

- Add joint group for SequencePlayer. <br>
    groupname is joint group name such as rarm or lleg. <br>
    jnames is list of joint name. <br>


#### :remove-joint-group
&nbsp;&nbsp;&nbsp;*groupname* 

- Remove joint group for SequencePlayer. <br>
    groupname is joint group name such as rarm or lleg. <br>


#### :set-joint-angles-of-group
&nbsp;&nbsp;&nbsp;*groupname* *av* *tm* 

- Set joint angles of group. <br>
    !!This method is not recommended, please use :angle-vector method like (send *ri* :angle-vector (send *robot* :angle-vector) 2000 :head-controller).!! <br>


#### :load-pattern
&nbsp;&nbsp;&nbsp;*basename* *&optional* *(tm 0.0)* 

- Load pattern files, such as xx.pos and xx.waist. <br>
    For pattern file definitions, please see loadPattern in SequencePlayer documentation in hrpsys-base API Doc. <br>


#### :wait-interpolation-seq


- Directly call SequencePlayer waitInterpolation. <br>
    This can be used for force/moment interpolation. <br>


#### :set-ref-forces-moments
&nbsp;&nbsp;&nbsp;*force-list* *moment-list* *tm* 

- Set reference wrenches. wrench-list is list of wrench ([N],[Nm]) for all end-effectors. tm is interpolation time [ms]. <br>


#### :set-ref-forces
&nbsp;&nbsp;&nbsp;*force-list* *tm* *&key* *(update-robot-state t)* 

- Set reference forces. force-list is list of force ([N]) for all end-effectors. tm is interpolation time [ms]. <br>


#### :set-ref-moments
&nbsp;&nbsp;&nbsp;*moment-list* *tm* *&key* *(update-robot-state t)* 

- Set reference moments. moment-list is list of moment ([Nm]) for all end-effectors. tm is interpolation time [ms]. <br>


#### :set-ref-force-moment
&nbsp;&nbsp;&nbsp;*force* *moment* *tm* *&optional* *(limb :arms)* *&key* *(update-robot-state t)* 

- Set reference force [N] and moment [Nm]. tm is interpolation time [ms]. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-ref-force
&nbsp;&nbsp;&nbsp;*force* *tm* *&optional* *(limb :arms)* *&key* *(update-robot-state t)* 

- Set reference force [N]. tm is interpolation time [ms]. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-ref-moment
&nbsp;&nbsp;&nbsp;*moment* *tm* *&optional* *(limb :arms)* *&key* *(update-robot-state t)* 

- Set reference moment [Nm]. tm is interpolation time [ms]. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :angle-vector-sequence-full
&nbsp;&nbsp;&nbsp;*jpos* *tm* *&key* *(sequence-length (length jpos))* <br>&nbsp;&nbsp;&nbsp;*(joint-length (length (car jpos)))* <br>&nbsp;&nbsp;&nbsp;*(fsensor-length (length (send robot :force-sensors)))* <br>&nbsp;&nbsp;&nbsp;*(vel (make-list sequence-length :initial-element (instantiate float-vector joint-length)))* <br>&nbsp;&nbsp;&nbsp;*(torque (make-list sequence-length :initial-element (instantiate float-vector joint-length)))* <br>&nbsp;&nbsp;&nbsp;*(root-coords (make-list sequence-length :initial-element (make-coords)))* <br>&nbsp;&nbsp;&nbsp;*(acc (make-list sequence-length :initial-element (instantiate float-vector 3)))* <br>&nbsp;&nbsp;&nbsp;*(zmp (make-list sequence-length :initial-element (instantiate float-vector 3)))* <br>&nbsp;&nbsp;&nbsp;*(wrench (make-list sequence-length :initial-element (instantiate float-vector (* 6 fsensor-length))))* <br>&nbsp;&nbsp;&nbsp;*(optional (make-list sequence-length :initial-element (instantiate float-vector (* 2 fsensor-length))))* <br>&nbsp;&nbsp;&nbsp;*(pos (send-all root-coords :worldpos))* <br>&nbsp;&nbsp;&nbsp;*(rpy (mapcar #'(lambda (x) (reverse (car (rpy-angle (send x :worldrot))))) root-coords))* <br>&nbsp;&nbsp;&nbsp;*(root-local-zmp (mapcar #'(lambda (zz cc) (send cc :inverse-transform-vector zz)) zmp root-coords))* 

- Call service for setJointAnglesSequenceFull. Definition of each sequence is similar to sequence file of loadPattern. <br>
    Arguments type: <br>
     Required <br>
      jpos: sequence of joint angles(float-vector) [deg],  (list av0 av1 ... avn) <br>
      tm: sequence of duration(float) [ms],  (list tm0 tm1 ... tmn) <br>
     Key <br>
      vel: sequence of joint angular velocities(float-vector) [deg/s],  (list vel0 vel1 ... veln) <br>
      torque: sequence of torques(float-vector) [Nm],  (list torque0 torque1 ... torquen) <br>
      root-coords: sequence of waist(root-link) coords in the world frame. (list rc0 rc1 ... rcn). Origin coords by default. <br>
      acc: sequence of waist acc(float-vector) [m/s^2],  (list acc0 acc1 ... accn) <br>
      zmp: sequence of zmp in the world frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). Zero by default. <br>
      wrench: sequence of wrench(float-vector) [N, Nm] for all fsensors,  (list wrench0 wrench1 ... wrenchn) <br>
      optional: sequence of optional(float-vector) [],  (list optional0 optional1 ... optionaln) <br>
     Not required (calculated from other arguments by default), therefore users need not to use these arguments. <br>
      pos: sequence of waist pos(float-vector) [mm] in the world frame,  (list pos0 pos1 ... posn). If root-coords is specified, calculated from root-coords and do not set pos. <br>
      rpy: sequence of waist rpy(float-vector) [rad] in the world frame,  (list rpy0 rpy1 ... rpyn). If root-coords is specified, calculated from root-coords and do not set rpy. <br>
      root-local-zmp: sequence of zmp in the waist(root-link) frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). If root-coords and zmp are specified, calculated from root-coords and zmp and do not set root-local-zmp. <br>


#### :set-tolerance
&nbsp;&nbsp;&nbsp;*&key* *(tolerance 0.1)* <br>&nbsp;&nbsp;&nbsp;*(link-pair-name all)* 

- Set tolerance [m] of collision detection with given link-pair-name (all by default). <br>


#### :start-collision-detection


- Enable collision detection. <br>


#### :stop-collision-detection


- Disable collision detection. <br>


#### :get-collision-status


- Get collision status. <br>


#### :save-log
&nbsp;&nbsp;&nbsp;*fname* *&key* *(set-robot-date-string t)* <br>&nbsp;&nbsp;&nbsp;*(make-directory nil)* 

- Save log files as [fname].[component_name]_[dataport_name]. <br>
    This method corresponds to DataLogger save(). <br>
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t. <br>
    If make-directory is t and fname is /foo/bar/basename, make directory of /foo/bar/basename and log files will be saved with /foo/bar/basename/basename <br>


#### :start-log


- Start logging. <br>
    This method corresponds to DataLogger clear(). <br>


#### :set-log-maxlength
&nbsp;&nbsp;&nbsp;*&optional* *(maxlength 4000)* 

- Set max log length. <br>
    This method corresponds to DataLogger maxLength(). <br>


#### :set-servo-gain-percentage
&nbsp;&nbsp;&nbsp;*name* *percentage* 

- Set servo gain percentage [0-100] with given name. <br>


#### :remove-force-sensor-offset


- Remove force sensor offset. <br>
    This function takes 10[s]. Please keep the robot static and make sure that robot's sensors do not contact with any objects. <br>


#### :set-servo-error-limit
&nbsp;&nbsp;&nbsp;*name* *limit* 

- Set RobotHardware servo error limit [rad] with given name. <br>


#### :calibrate-inertia-sensor


- Calibrate inetria sensor. <br>
    This function takes 10[s]. Please keep the robot static. <br>


#### :get-impedance-controller-param-arguments


- Get arguments of :raw-set-impedance-controller-param <br>


#### :start-impedance
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Start impedance controller mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :stop-impedance
&nbsp;&nbsp;&nbsp;*limb* 

- Stop impedance controller mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-impedance-controller-param
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Set impedance controller parameter like (send *ri* :set-impedance-controller-param :rarm :K-p 400). <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>
    For arguments, please see (send *ri* :get-impedance-controller-param-arguments). <br>


#### :get-impedance-controller-param
&nbsp;&nbsp;&nbsp;*limb* 

- Get impedance controller parameter. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :get-impedance-controller-controller-mode
&nbsp;&nbsp;&nbsp;*name* 

- Get ImpedanceController ControllerMode as Euslisp symbol. <br>


#### :get-object-turnaround-detector-param


- Get objectcontactturnarounddetectorparam. <br>


#### :get-object-turnaround-detector-param-arguments


- Get arguments of :raw-set-object-turnaround-detector-param <br>


#### :start-object-turnaround-detection
&nbsp;&nbsp;&nbsp;*&key* *(ref-diff-wrench)* <br>&nbsp;&nbsp;&nbsp;*(max-time)* <br>&nbsp;&nbsp;&nbsp;*(limbs)* 

- Start ObjectContactTurnaroundDetection mode. <br>
    ref-diff-wrench is final reference wrench (scalar). <br>
    max-time is max time [msec]. <br>
    limbs is limb list to be used. <br>


#### :check-object-turnaround-detection


- Check object contact turnaround detection. <br>
   If t, detected. Otherwise, not detected. <br>


#### :get-otd-object-forces-moments


- Get ObjectContactTurnaroundDetector's curernt forces and moments for used limbs. <br>
    Return value is (list force-list moment-list). <br>


#### :set-object-turnaround-ref-force
&nbsp;&nbsp;&nbsp;*&key* *(limbs '(:rarm :larm))* <br>&nbsp;&nbsp;&nbsp;*(axis (float-vector 0 0 -1))* <br>&nbsp;&nbsp;&nbsp;*(max-time 4000.0)* <br>&nbsp;&nbsp;&nbsp;*(max-ref-force)* <br>&nbsp;&nbsp;&nbsp;*(detect-time-offset 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-time 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-p t)* <br>&nbsp;&nbsp;&nbsp;*(periodic-time 200)* <br>&nbsp;&nbsp;&nbsp;*(detector-total-wrench :total-force)* <br>&nbsp;&nbsp;&nbsp;*(return-value-mode :forces)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-linear-p)* 

- Set object turnaround reference force. <br>
    axis is resultant force direction. <br>
    max-time is max time [msec] and max-ref-force is max reference resultant force. <br>
    max-ref-force is max resultant reference force [N]. max-ref-force is added to original ref forces. <br>
    detect-time-offset is additional checking time [msec]. <br>
    set-ref-force-time is time to set reference force [msec]. <br>
    If set-ref-force-p is t, set estimated reference force. Otherwise, set reference force to zero. <br>
    periodic-time is loop wait time[msec]. <br>
    return-value-mode is used to select return value type. <br>
    If :all, returns the results of :get-otd-object-forces-moments itself. If :forces, returns forces results. <br>
    If set-ref-force-linear-p t, set linear interpolation first and set hoffarbib interpolation in return. <br>


#### :set-object-turnaround-ref-moment
&nbsp;&nbsp;&nbsp;*&key* *(limbs '(:rarm :larm))* <br>&nbsp;&nbsp;&nbsp;*(axis (float-vector 0 0 1))* <br>&nbsp;&nbsp;&nbsp;*(max-ref-moment 10)* <br>&nbsp;&nbsp;&nbsp;*(func)* <br>&nbsp;&nbsp;&nbsp;*(moment-center)* <br>&nbsp;&nbsp;&nbsp;*(max-time 4000.0)* <br>&nbsp;&nbsp;&nbsp;*(detect-time-offset 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-time 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-p t)* <br>&nbsp;&nbsp;&nbsp;*(periodic-time 200)* <br>&nbsp;&nbsp;&nbsp;*(detector-total-wrench :total-moment)* <br>&nbsp;&nbsp;&nbsp;*(return-value-mode :all)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-linear-p)* 

- Set object turnaround reference force. <br>
    axis is axis which resultant moment is around. <br>
    max-time is max time [msec] and max-ref-moment is max reference resultant moment. <br>
    func is function to distribute estimated resultant moment to hands' forces/moments. <br>
    moment-center is moment center [mm]. <br>
    detect-time-offset is additional checking time [msec]. <br>
    set-ref-force-time is time to set reference force [msec]. <br>
    If set-ref-force-p is t, set estimated reference force. Otherwise, set reference force to zero. <br>
    periodic-time is loop wait time[msec]. <br>
    return-value-mode is used to select return value type. <br>
    If :all, returns the results of :get-otd-object-forces-moments itself. If :forces, returns forces results. <br>
    If set-ref-force-linear-p t, set linear interpolation first and set hoffarbib interpolation in return. <br>


#### :set-object-turnaround-detector-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *detector-total-wrench* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set Object Turnaround Detector. <br>
    For arguments, please see (send *ri* :get-object-turnaround-detector-param-arguments). <br>


#### :get-object-turnaround-detector-detector-total-wrench


- Get Object Turnadound Detector as Euslisp symbol. <br>


#### :get-forcemoment-offset-param-arguments


- Get arguments of :raw-set-forcemoment-offset-param <br>


#### :zero-set-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*limb* 

- Set RemoveForceSensorLinkOffset's params offset to zero. <br>


#### :set-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Set RemoveForceSensorLinkOffset params for given limb. <br>
    For arguments, please see (send *ri* :get-forcemoment-offset-param-arguments). <br>


#### :get-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*limb* 

- Get RemoveForceSensorLinkOffset params for given limb. <br>


#### :load-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*fname* *&key* *(set-offset t)* 

- Load RemoveForceSensorLinkOffset params from fname (file path). <br>


#### :load-forcemoment-offset-params
&nbsp;&nbsp;&nbsp;*filename* 

- Load RMFO offset parameters from parameter file. <br>
    This method corresponds to RemoveForceSensorLinkOffset loadForceMomentOffsetParams(). <br>


#### :dump-forcemoment-offset-params
&nbsp;&nbsp;&nbsp;*filename* *&key* *(set-robot-date-string t)* 

- Save all RMFO offset parameters. <br>
    This method corresponds to RemoveForceSensorLinkOffset dumpForceMomentOffsetParams(). <br>
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t. <br>


#### :remove-force-sensor-offset-rmfo
&nbsp;&nbsp;&nbsp;*&key* *(limbs)* <br>&nbsp;&nbsp;&nbsp;*((:time tm) 8.0)* 

- remove offsets on sensor outputs form force/torque sensors. <br>
    Sensor offsets (force_offset and moment_offset in ForceMomentOffsetParam) are calibrated. <br>
    Please keep the robot static and make sure that robot's sensors do not contact with any objects. <br>
    Argument: <br>
      limbs is list of sensor names to be calibrated. <br>
      If not specified, all sensors are calibrated by default. <br>
      time is duration of calibration[s]. 8.0[s] by default. <br>
    Return: <br>
      t if set successfully, nil otherwise <br>


#### :remove-force-sensor-offset-rmfo-arms
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 8.0)* 

- Remove force and moment offset for :rarm and :larm. <br>
    time is duration of calibration[s]. 8.0[s] by default. <br>


#### :remove-force-sensor-offset-rmfo-legs
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 8.0)* 

- Remove force and moment offset for :rleg and :lleg. <br>
    time is duration of calibration[s]. 8.0[s] by default. <br>


#### :reset-force-moment-offset-arms
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 0.1)* 

- time[s] <br>


#### :reset-force-moment-offset-legs
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 0.1)* 

- time[s] <br>


#### :reset-force-moment-offset
&nbsp;&nbsp;&nbsp;*limbs* 

- Remove force and moment offsets. limbs should be list of limb symbol name. <br>


#### :get-gait-generator-param


- Get gaitgeneratorparam. <br>


#### :get-gait-generator-param-arguments


- Get arguments of :raw-set-gait-generator-param <br>


#### :get-auto-balancer-param


- Get AutoBalancer param. <br>


#### :get-auto-balancer-param-arguments


- Get arguments of :raw-set-auto-balancer-param <br>


#### :start-auto-balancer
&nbsp;&nbsp;&nbsp;*&key* *(limbs (if (not (every #'null (send robot :arms))) '(:rleg :lleg :rarm :larm) '(:rleg :lleg)))* 

- startAutoBalancer. <br>
    If robot with arms, start auto balancer with legs and arms ik by default. <br>
    Otherwise, start auto balancer with legs ik by default. <br>


#### :stop-auto-balancer


- Stop auto balancer mode <br>


#### :go-pos-no-wait
&nbsp;&nbsp;&nbsp;*xx* *yy* *th* 

- Call goPos without wait. <br>


#### :go-pos
&nbsp;&nbsp;&nbsp;*xx* *yy* *th* 

- Call goPos with wait. <br>


#### :get-foot-step-params


- Get AutoBalancer foot step params. <br>


#### :get-foot-step-param
&nbsp;&nbsp;&nbsp;*param-name* 

- Get AutoBalancer foot step param by given name. <br>
    param-name is key word for parameters defined in IDL. <br>
    param-name should be :rleg-coords :lleg-coords :support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords :support-leg :support-leg-with-both. <br>


#### :set-foot-steps-no-wait
&nbsp;&nbsp;&nbsp;*foot-step-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step by default parameters and do not wait for step finish. <br>
    foot-step-list is list of footstep (only biped) or list of list of footstep. <br>
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking. <br>


#### :set-foot-steps
&nbsp;&nbsp;&nbsp;*foot-step-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step by default parameters and wait for step finish. <br>
    foot-step-list is list of footstep (only biped) or list of list of footstep. <br>
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking. <br>


#### :set-foot-steps-with-param-no-wait
&nbsp;&nbsp;&nbsp;*foot-step-list* *step-height-list* *step-time-list* *toe-angle-list* *heel-angle-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step with step parameter and do not wait for step finish. <br>
    foot-step-list is list of footstep (only biped) or list of list of footstep. <br>
    step-height-list is list of step height (only biped) or list of list of step height. <br>
    step-time-list is list of step time (only biped) or list of list of step time. <br>
    toe-angle-list is list of toe angle (only biped) or list of list of toe angle. <br>
    heel-angle-list is list of heel angle (only biped) or list of list of heel angle. <br>


#### :set-foot-steps-with-param
&nbsp;&nbsp;&nbsp;*foot-step-list* *step-height-list* *step-time-list* *toe-angle-list* *heel-angle-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step with step parameter and wait for step finish. <br>
    For arguments, please see :set-foot-steps-with-param-no-wait documentation. <br>


#### :set-foot-steps-roll-pitch
&nbsp;&nbsp;&nbsp;*angle* *&key* *(axis :x)* 

- Set foot steps with roll or pitch orientation. <br>
    angle is roll or pitch angle [deg]. <br>
    axis is :x (roll) or :y (pitch). <br>


#### :set-foot-steps-with-base-height
&nbsp;&nbsp;&nbsp;*fs* *av-list* *time-list* 

- Set foot steps with sending angle-vector. <br>


#### :set-foot-steps-with-param-and-base-height
&nbsp;&nbsp;&nbsp;*fs-params* *av-list* *time-list* 

- Set foot steps and params with sending angle-vector. <br>


#### :adjust-foot-steps
&nbsp;&nbsp;&nbsp;*rfoot-coords* *lfoot-coords* 

- Adjust current footsteps during autobalancer mode and not walking. <br>
    rfoot-coords and lfoot-coords are end-coords for new foot steps. <br>


#### :adjust-foot-steps-roll-pitch
&nbsp;&nbsp;&nbsp;*angle* *&key* *(axis :x)* 

- Adjust foot steps with roll or pitch orientation. <br>
    angle is roll or pitch angle [deg]. <br>
    axis is :x (roll) or :y (pitch). <br>


#### :get-remaining-foot-step-sequence-current-index


- Get remaining foot steps from GaitGenerator and current index. <br>
   Return is (list (list current-support-foot-coords remaining-swing-dst-coords-0 ... ) current-index). <br>


#### :get-current-footstep-index


- Get current footstep index. <br>


#### :get-remaining-foot-step-sequence


- Get remaining foot steps from GaitGenerator. <br>
   Return is (list current-support-foot-coords remaining-swing-dst-coords-0 ... ). <br>


#### :get-go-pos-footsteps-sequence
&nbsp;&nbsp;&nbsp;*xx* *yy* *th* 

- Get foot steps of go-pos without executing them. <br>
   Return is list of list of footstep. <br>


#### :draw-remaining-foot-step-sequence
&nbsp;&nbsp;&nbsp;*vwer* *&key* *(flush)* <br>&nbsp;&nbsp;&nbsp;*(rleg-color #f(1.0 0.0 0.0))* <br>&nbsp;&nbsp;&nbsp;*(lleg-color #f(0.0 1.0 0.0))* <br>&nbsp;&nbsp;&nbsp;*(change-support-leg-color t)* <br>&nbsp;&nbsp;&nbsp;*(support-leg-color #f(1.0 1.0 1.0))* 

- Draw remaining foot steps. <br>


#### :go-velocity
&nbsp;&nbsp;&nbsp;*vx* *vy* *vth* 

- Call goVelocity. <br>


#### :go-stop


- Stop stepping. <br>


#### :emergency-walking-stop


- Stop stepping immediately. <br>


#### :calc-go-velocity-param-from-velocity-center-offset
&nbsp;&nbsp;&nbsp;*ang* *velocity-center-offset* 

- Calculate go-velocity velocities from rotation center and rotation angle. <br>
    ang is rotation angle [rad]. velocity-center-offset is velocity center offset [mm] from foot mid coords. <br>


#### :wait-foot-steps


- Wait for whole footsteps are executed. <br>


#### :set-gait-generator-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *default-orbit-type* <br>&nbsp;&nbsp;&nbsp;*leg-default-translate-pos* <br>&nbsp;&nbsp;&nbsp;*stride-parameter* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set gait generator param. <br>
    For arguments, please see (send *ri* :get-gait-generator-param-arguments). <br>


#### :print-gait-generator-orbit-type


- Print GaitGenerator orbit types. <br>


#### :get-gait-generator-orbit-type


- Get GaitGenerator Orbit Type as Euslisp symbol. <br>


#### :calc-toe-heel-phase-ratio
&nbsp;&nbsp;&nbsp;*toe-angle* *heel-angle* *double-support-ratio-half* 

- Calculate vector for toe heel phase ratio. <br>
    Arguments: <br>
      toe-angle, heel-angle : max toe/heel angle [deg]. <br>
      double-support-ratio-half : half of double support ratio in terms of toe heel phase ratio. <br>


#### :set-gait-generator-toe-heel-angles
&nbsp;&nbsp;&nbsp;*toe-angle* *heel-angle* *double-support-ratio-half* 

- Set toe-angle, heel-angle, and toe-heel-phase-ratio. <br>
    toe-heel-phase-ratio is automatically calculated. <br>
    Arguments: <br>
      toe-angle, heel-angle : max toe/heel angle [deg]. <br>
      double-support-ratio-half : half of double support ratio in terms of toe heel phase ratio. <br>


#### :get-auto-balancer-controller-mode


- Get AutoBalancer ControllerMode as Euslisp symbol. <br>


#### :set-auto-balancer-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *leg-names* <br>&nbsp;&nbsp;&nbsp;*default-zmp-offsets* <br>&nbsp;&nbsp;&nbsp;*graspless-manip-arm* <br>&nbsp;&nbsp;&nbsp;*graspless-manip-reference-trans-pos* <br>&nbsp;&nbsp;&nbsp;*graspless-manip-reference-trans-rot* <br>&nbsp;&nbsp;&nbsp;*use-force-mode* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set AutoBalancer param. <br>
    For arguments, please see (send *ri* :get-auto-balancer-param-arguments). <br>


#### :print-auto-balancer-use-force-mode


- Print AutoBalancer UseForceMode. <br>


#### :get-auto-balancer-use-force-mode


- Get AutoBalancer UseForceMode as Euslisp symbol. <br>


#### :cmd-vel-mode


- Walk with subscribing /cmd_vel topic. <br>


#### :calc-dvel-with-velocity-center-offset
&nbsp;&nbsp;&nbsp;*ang* *velocity-center-offset* 

- Calculate velocity params for rotating with given velocity center offset. <br>
    Ang : [deg], offset vector [mm] <br>


#### :set-default-step-time-with-the-same-swing-time
&nbsp;&nbsp;&nbsp;*default-step-time* 

- Set default step time with the same swing time. <br>


#### :start-graspless-manip-mode
&nbsp;&nbsp;&nbsp;*robot* *arm* 

- Start graspless manip mode while walking. <br>
    robot is robot instance which angle-vector is for graspless manip. <br>
    arm is used arm (:rarm, :larm, :arms). <br>


#### :stop-graspless-manip-mode


- Stop graspless manip mode while walking. <br>


#### :calc-hand-trans-coords-dual-arms
&nbsp;&nbsp;&nbsp;*robot* 

- Calculate foot->hand coords transformation for dual-arm graspless manipulation. <br>


#### :calc-hand-trans-coords-single-arm
&nbsp;&nbsp;&nbsp;*robot* *arm* 

- Calculate foot->hand coords transformation for single-arm graspless manipulation. <br>


#### :set-soft-error-limit
&nbsp;&nbsp;&nbsp;*name* *limit* 

- Set SoftErrorLimiter servo error limit [rad] with given name. <br>


#### :get-st-param


- Get stparam. <br>


#### :get-st-param-arguments


- Get arguments of :raw-set-st-param <br>


#### :set-st-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *st-algorithm* <br>&nbsp;&nbsp;&nbsp;*eefm-pos-damping-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-rot-damping-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-pos-time-const-support* <br>&nbsp;&nbsp;&nbsp;*eefm-rot-time-const* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-pos-spring-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-pos-time-const* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-rot-spring-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-rot-time-const* <br>&nbsp;&nbsp;&nbsp;*eefm-ee-forcemoment-distribution-weight* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set Stabilizer parameters. <br>
    For arguments, please see (send *ri* :get-st-param-arguments). <br>


#### :set-st-param-for-non-feedback-lip-mode


- Set Stabilizer parameters to make robot Linear Inverted Pendulum mode without state feedback. <br>
    By using this mode, robot feets adapt to ground surface. <br>


#### :set-default-st-param


- Set Stabilzier parameter by default parameters. <br>


#### :set-st-param-by-ratio
&nbsp;&nbsp;&nbsp;*state-feedback-gain-ratio* *damping-control-gain-ratio* *attitude-control-gain-ratio* 

- Set Stabilzier parameter by ratio from default parameters. <br>
    state-feedback-gain-ratio is ratio for state feedback gain. <br>
    damping-control-gain-ratio is ratio for damping control gain. <br>
    attitude-control-gain-ratio is ratio for attitude control gain. <br>
    When ratio = 1, parameters are same as default parameters. <br>
    For state-feedback-gain-ratio and attitude-control-gain-ratio, <br>
    when ratio < 1 Stabilzier works less feedback control and less oscillation and when ratio > 1 more feedback and more oscillation. <br>
    For dampnig-control-gain-ratio, <br>
    when ratio > 1 feet behavior becomes hard and safe and when ratio < 1 soft and dangerous. <br>


#### :get-st-controller-mode


- Get Stabilizer ControllerMode as Euslisp symbol. <br>


#### :get-st-algorithm


- Get Stabilizer Algorithm as Euslisp symbol. <br>


#### :start-st


- Start Stabilizer Mode. <br>


#### :stop-st


- Stop Stabilizer Mode. <br>


#### :get-kalman-filter-param


- Get kalmanfilterparam. <br>


#### :set-kalman-filter-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *q-angle* <br>&nbsp;&nbsp;&nbsp;*q-rate* <br>&nbsp;&nbsp;&nbsp;*r-angle* <br>&nbsp;&nbsp;&nbsp;*kf-algorithm* <br>&nbsp;&nbsp;&nbsp;*acc-offset* <br>&nbsp;&nbsp;&nbsp;*sensorrpy-offset* 

- Set kalmanfilterparam. For arguments, please see (send *ri* :get-kalman-filter-param-arguments). <br>


#### :get-kalman-filter-param-arguments


- Get arguments of :set-kalman-filter-param <br>


#### :get-kalman-filter-algorithm


- Get KalmanFilter Algorithm as Euslisp symbol. <br>


#### :get-emergency-stopper-param


- Get emergencystopperparam. <br>


#### :set-emergency-stopper-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *default-recover-time* <br>&nbsp;&nbsp;&nbsp;*default-retrieve-time* <br>&nbsp;&nbsp;&nbsp;*is-stop-mode* 

- Set emergencystopperparam. For arguments, please see (send *ri* :get-emergency-stopper-param-arguments). <br>


#### :get-emergency-stopper-param-arguments


- Get arguments of :set-emergency-stopper-param <br>


#### :emergency-stop-motion


- Stop motion emergently in EmergencyStopper. <br>


#### :hard-emergency-stop-motion


- Stop motion emergently in hard EmergencyStopper. <br>


#### :emergency-release-motion


- Release emergency motion stopping in EmergencyStopper. <br>


#### :hard-emergency-release-motion


- Release emergency motion stopping in hard EmergencyStopper. <br>


#### :emergency-mode


- Returns emergency mode. <br>


#### :start-default-unstable-controllers
&nbsp;&nbsp;&nbsp;*&key* *(ic-limbs '(:rarm :larm))* <br>&nbsp;&nbsp;&nbsp;*(abc-limbs (if (not (every #'null (send robot :arms))) '(:rleg :lleg :rarm :larm) '(:rleg :lleg)))* 

- Start default unstable RTCs controller mode. <br>
    Currently Stabilzier, AutoBalancer, and ImpedanceController are started. <br>


#### :stop-default-unstable-controllers
&nbsp;&nbsp;&nbsp;*&key* *(ic-limbs '(:rarm :larm))* 

- Stop default unstable RTCs controller mode. <br>
    Currently Stabilzier, AutoBalancer, and ImpedanceController are stopped. <br>


#### :get-reference-force-updater-param-arguments


- Get arguments of :raw-set-reference-force-updater-param <br>


#### :get-supported-reference-force-updater-name-list


- Get supported reference force updater names (:rarm, :larm, ...) <br>


#### :start-reference-force-updater
&nbsp;&nbsp;&nbsp;*limb* 

- Start reference force updater mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :stop-reference-force-updater
&nbsp;&nbsp;&nbsp;*limb* 

- Stop reference force updater mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :start-reference-force-updater-no-wait
&nbsp;&nbsp;&nbsp;*limb* 

- Start reference force updater mode without wait. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :stop-reference-force-updater-no-wait
&nbsp;&nbsp;&nbsp;*limb* 

- Start reference force updater mode without wait. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :set-reference-force-updater-param
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Set reference force updater parameter like (send *ri* :set-reference-force-updater-param :rarm :p-gain 0.02). <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>
    For arguments, please see (send *ri* :get-reference-force-updater-param-arguments) <br>


#### :get-reference-force-updater-param
&nbsp;&nbsp;&nbsp;*limb* 

- Get reference force updater parameter. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :get-torque-controller-param
&nbsp;&nbsp;&nbsp;*jname* 

- Get torquecontrollerparam. <br>


#### :set-torque-controller-param
&nbsp;&nbsp;&nbsp;*jname* *&rest* *args* *&key* *tc* <br>&nbsp;&nbsp;&nbsp;*ke* <br>&nbsp;&nbsp;&nbsp;*kd* <br>&nbsp;&nbsp;&nbsp;*ki* <br>&nbsp;&nbsp;&nbsp;*alpha* <br>&nbsp;&nbsp;&nbsp;*beta* 

- Set torquecontrollerparam. For arguments, please see (send *ri* :get-torque-controller-param-arguments). <br>


#### :get-torque-controller-param-arguments


- Get arguments of :set-torque-controller-param <br>


#### :enable-torque-control
&nbsp;&nbsp;&nbsp;*jname* 

- enable torque controller for specified joint (only start torque control when tau exceeds tauMax) <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>


#### :disable-torque-control
&nbsp;&nbsp;&nbsp;*jname* 

- disable torque control in specified joint (torque controller do nothing and pass qRefIn when disabeld) <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>


#### :start-torque-control
&nbsp;&nbsp;&nbsp;*jname* *&optional* *(tauref nil)* 

- start torque control in specified joint. set reference torque before start if tauref is given. <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>
    tauref is reference torque [N/m], should be a number (for one joint) or list of numbers (for multiple joints) <br>


#### :stop-torque-control
&nbsp;&nbsp;&nbsp;*jname* 

- stop torque controller for specified joint (this method does not disable torque control) <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>


#### :set-reference-torque
&nbsp;&nbsp;&nbsp;*jname* *tauref* 

- set reference torque in specified joint <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>
    tauref (reference torque [N/m]) should be a number (for one joint) or list of numbers (for multiple joints) <br>


#### :print-rtc-param-all


- Print RTC parameter struct probided by :get-xx method in more readable style for all recommended RTCs. <br>
    Check all possible RTCs by checking existence of ROSBridge nodes. <br>


:rtmros-motor-states-callback *msg* 

:rtmros-zmp-callback *msg* 

:rtmros-imu-callback *msg* 

:rtmros-force-sensor-callback *fsensor-name* *msg* 

:rtmros-emergency-mode-callback *msg* 

:rtmros-capture-point-callback *ref-act* *msg* 

:rtmros-contact-states-callback *ref-act* *msg* 

:tmp-force-moment-vector-for-limb *f/m* *fsensor-name* *&optional* *(topic-name-prefix nil)* 

:tmp-force-moment-vector *f/m* *&optional* *(limb)* *(topic-name-prefix nil)* 

:define-all-rosbridge-srv-methods *&key* *(debug-view nil)* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-fnames-from-type *type-name* *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-idl-fnames *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-srv-fnames *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-method-def-macro *rtc-name* *srv-name* *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-idl-enum-values *value* *enum-type-string* 

:set-base-pose *&optional* *base-coords* *(tm 0.1)* 

:sync-controller *controller* *&optional* *(interpolation-time 1000)* *(blockp t)* 

:raw-get-impedance-controller-param *name* 

:raw-set-impedance-controller-param *name* *&rest* *args* *&key* *m-p* *d-p* *k-p* *m-r* *d-r* *k-r* *force-gain* *moment-gain* *sr-gain* *avoid-gain* *reference-gain* *manipulability-limit* *controller-mode* *ik-optional-weight-vector* *use-sh-base-pos-rpy* 

:raw-start-impedance *limb* 

:start-impedance-no-wait *limb* 

:stop-impedance-no-wait *limb* 

:wait-impedance-controller-transition *limb* 

:force-sensor-method *limb* *method-func* *method-name* *&rest* *args* 

:raw-set-object-turnaround-detector-param *&rest* *args* *&key* *wrench-cutoff-freq* *dwrench-cutoff-freq* *detect-ratio-thre* *start-ratio-thre* *detect-time-thre* *start-time-thre* *axis* *moment-center* *detector-total-wrench* 

:raw-get-forcemoment-offset-param *name* 

:raw-set-forcemoment-offset-param *name* *&rest* *args* *&key* *force-offset* *moment-offset* *link-offset-centroid* *link-offset-mass* 

:_reset-force-moment-offset *limbs* *f/m* *&key* *(itr 10)* 

:raw-set-gait-generator-param *&rest* *args* *&key* *default-step-time* *default-step-height* *default-double-support-ratio* *default-double-support-ratio-before* *default-double-support-ratio-after* *default-double-support-static-ratio* *default-double-support-static-ratio-before* *default-double-support-static-ratio-after* *default-double-support-ratio-swing-before* *default-double-support-ratio-swing-after* *stride-parameter* *stride-limitation-for-circle-type* *default-orbit-type* *swing-trajectory-delay-time-offset* *swing-trajectory-final-distance-weight* *stair-trajectory-way-point-offset* *cycloid-delay-kick-point-offset* *swing-trajectory-time-offset-xy2z* *gravitational-acceleration* *toe-pos-offset-x* *heel-pos-offset-x* *toe-zmp-offset-x* *heel-zmp-offset-x* *toe-angle* *heel-angle* *toe-check-thre* *heel-check-thre* *toe-heel-phase-ratio* *use-toe-joint* *use-toe-heel-transition* *use-toe-heel-auto-set* *zmp-weight-map* *leg-default-translate-pos* *optional-go-pos-finalize-footstep-num* *overwritable-footstep-index-offset* *overwritable-stride-limitation* *use-stride-limitation* *stride-limitation-type* *leg-margin* *footstep-modification-gain* *modify-footsteps* *cp-check-margin* *margin-time-ratio* 

:raw-set-auto-balancer-param *&rest* *args* *&key* *default-zmp-offsets* *move-base-gain* *controller-mode* *use-force-mode* *graspless-manip-mode* *graspless-manip-arm* *graspless-manip-p-gain* *graspless-manip-reference-trans-pos* *graspless-manip-reference-trans-rot* *transition-time* *zmp-transition-time* *adjust-footstep-transition-time* *leg-names* *has-ik-failed* *pos-ik-thre* *rot-ik-thre* *is-hand-fix-mode* *end-effector-list* *default-gait-type* *ik-limb-parameters* *use-limb-stretch-avoidance* *limb-stretch-avoidance-time-const* *limb-stretch-avoidance-vlimit* *limb-length-margin* *additional-force-applied-link-name* *additional-force-applied-point-offset* 

:raw-get-foot-step-param 

:abc-footstep->eus-footstep *f* 

:eus-footstep->abc-footstep *f* 

:cmd-vel-cb *msg* *&key* *(vel-x-ratio 1.0)* *(vel-y-ratio 1.0)* *(vel-th-ratio 1.0)* 

:start-cmd-vel-mode 

:stop-cmd-vel-mode 

:raw-set-st-param *&rest* *args* *&key* *k-tpcc-p* *k-tpcc-x* *k-brot-p* *k-brot-tc* *eefm-k1* *eefm-k2* *eefm-k3* *eefm-zmp-delay-time-const* *eefm-ref-zmp-aux* *eefm-rot-damping-gain* *eefm-rot-time-const* *eefm-pos-damping-gain* *eefm-pos-time-const-support* *eefm-swing-rot-spring-gain* *eefm-swing-rot-time-const* *eefm-swing-pos-spring-gain* *eefm-swing-pos-time-const* *eefm-ee-moment-limit* *eefm-pos-compensation-limit* *eefm-rot-compensation-limit* *eefm-pos-time-const-swing* *eefm-pos-transition-time* *eefm-pos-margin-time* *eefm-leg-inside-margin* *eefm-leg-outside-margin* *eefm-leg-front-margin* *eefm-leg-rear-margin* *eefm-body-attitude-control-gain* *eefm-body-attitude-control-time-const* *eefm-cogvel-cutoff-freq* *eefm-wrench-alpha-blending* *eefm-alpha-cutoff-freq* *eefm-gravitational-acceleration* *eefm-ee-pos-error-p-gain* *eefm-ee-rot-error-p-gain* *eefm-ee-error-cutoff-freq* *eefm-support-polygon-vertices-sequence* *eefm-use-force-difference-control* *eefm-use-swing-damping* *eefm-swing-damping-force-thre* *eefm-swing-damping-moment-thre* *eefm-swing-rot-damping-gain* *eefm-swing-pos-damping-gain* *eefm-ee-forcemoment-distribution-weight* *st-algorithm* *controller-mode* *transition-time* *is-ik-enable* *is-feedback-control-enable* *is-zmp-calc-enable* *cop-check-margin* *cp-check-margin* *tilt-margin* *ref-capture-point* *act-capture-point* *cp-offset* *contact-decision-threshold* *foot-origin-offset* *emergency-check-mode* *end-effector-list* *is-estop-while-walking* *ik-limb-parameters* *use-limb-stretch-avoidance* *limb-stretch-avoidance-time-const* *limb-stretch-avoidance-vlimit* *limb-length-margin* *detection-time-to-air* *root-rot-compensation-limit* *use-zmp-truncation* 

:get-hand-config-list 

:start-grasp *limb* *&key* *(target-error)* *(gain-percentage)* 

:stop-grasp *limb* 

:raw-get-reference-force-updater-param *name* 

:raw-set-reference-force-updater-param *name* *&rest* *args* *&key* *motion-dir* *frame* *update-freq* *update-time-ratio* *p-gain* *d-gain* *i-gain* *is-hold-value* 


#### :def-limb-controller-method
&nbsp;&nbsp;&nbsp;*limb* *&key* *(debugp nil)* 

- Method to add limb controller action by default setting. <br>
     Currently, FollowJointTrajectoryAction is used. <br>
     This method calls defmethod. If :debugp t, we can see defmethod s-expressions. <br>


#### :get-robot-date-string


- Get string including robot name and date. <br>
    For example, "SampleRobot_20160412163151". <br>


#### :calc-zmp-from-state
&nbsp;&nbsp;&nbsp;*&key* *(wrt :world)* 

- Calculate zmp from state [mm]. <br>
    For example ;; (progn (send *ri* :go-velocity 0 0 0) (objects (list (*ri* . robot))) (do-until-key (let ((zmp (send *ri* :calc-zmp-from-state))) (send *irtviewer* :draw-objects :flush nil) (send zmp :draw-on :flush t :size 300)))) <br>
    :wrt is :local => calc local zmp for (*ri* . robot)'s root-link coords <br>
    :wrt is :world => calc world zmp for (*ri* . robot) <br>


#### :set-interpolation-mode
&nbsp;&nbsp;&nbsp;*interpolation-mode* 

- Set interpolation mode for SequencePlayer. <br>


#### :state
&nbsp;&nbsp;&nbsp;*&rest* *args* 

- Obtains sensor and robot command topics using spin-once. <br>


#### :gyro-vector


- Returns angular velocity [rad/s] of the gyro sensor. <br>


#### :accel-vector


- Returns acceleration [m/s2] of the acceleration sensor. <br>


#### :imucoords


- Returns robot's coords based on imu measurement. <br>


#### :motor-extra-data


- Returns motor extra data. Please see iob definition for each system. <br>


#### :temperature-vector


- Returns temperature vector. <br>


#### :act-contact-states


- Returns contact states from Stabilizer. <br>


#### :ref-contact-states


- Returns contact states from AutoBalancer. <br>


#### :act-capture-point-vector
&nbsp;&nbsp;&nbsp;*&optional* *(wrt :local)* 

- Returns act-capture-point vector [mm]. <br>
    If wrt is :local, returns act-capture-point in the base-link frame. If wrt is :world, returns act-capture-point in the world frame. <br>


#### :ref-capture-point-vector
&nbsp;&nbsp;&nbsp;*&optional* *(wrt :local)* 

- Returns ref-capture-point vector [mm]. <br>
    If wrt is :local, returns ref-capture-point in the base-link frame. If wrt is :world, returns ref-capture-point in the world frame. <br>


#### :zmp-vector
&nbsp;&nbsp;&nbsp;*&optional* *(wrt :local)* 

- Returns zmp vector [mm]. <br>
    If wrt is :local, returns zmp in the base-link frame. If wrt is :world, returns zmp in the world frame. <br>


#### :absolute-moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :moment-vector [Nm] list for all limbs in world frame obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :absolute-force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :force-vector [N] list for all limbs in world frame obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :reference-moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns reference moment-vector [Nm] list for all limbs obtained by :state. <br>
    This value corresponds to StateHolder and SequencePlayer RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :reference-force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns reference force-vector [N] list for all limbs obtained by :state. <br>
    This value corresponds to StateHolder and SequencePlayer RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :off-moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :moment-vector [Nm] list for all limbs obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :off-force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns offset-removed :force-vector [N] list for all limbs obtained by :state. <br>
    This value corresponds to RemoveForceSensorLinkOffset RTC. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :moment-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns :moment-vector [Nm] list for all limbs obtained by :state. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :force-vector
&nbsp;&nbsp;&nbsp;*&optional* *(limb)* 

- Returns :force-vector [N] list for all limbs obtained by :state. <br>
    If a limb argument is specified, returns a vector for the limb. <br>


#### :init
&nbsp;&nbsp;&nbsp;*&rest* *args* 

- :init method. This method should be overriden in subclass. <br>


:get-idl-enum-values *value* *enum-type-string* 

:get-rosbridge-method-def-macro *rtc-name* *srv-name* *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-srv-fnames *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-idl-fnames *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:get-rosbridge-fnames-from-type *type-name* *&optional* *(ros-pkg-name hrpsys_ros_bridge)* 

:define-all-rosbridge-srv-methods *&key* *(debug-view nil)* *(ros-pkg-name hrpsys_ros_bridge)* 

:tmp-force-moment-vector *f/m* *&optional* *(limb)* *(topic-name-prefix nil)* 

:tmp-force-moment-vector-for-limb *f/m* *fsensor-name* *&optional* *(topic-name-prefix nil)* 

:rtmros-contact-states-callback *ref-act* *msg* 

:rtmros-capture-point-callback *ref-act* *msg* 

:rtmros-emergency-mode-callback *msg* 

:rtmros-force-sensor-callback *fsensor-name* *msg* 

:rtmros-imu-callback *msg* 

:rtmros-zmp-callback *msg* 

:rtmros-motor-states-callback *msg* 


#### :angle-vector-sequence-full
&nbsp;&nbsp;&nbsp;*jpos* *tm* *&key* *(sequence-length (length jpos))* <br>&nbsp;&nbsp;&nbsp;*(joint-length (length (car jpos)))* <br>&nbsp;&nbsp;&nbsp;*(fsensor-length (length (send robot :force-sensors)))* <br>&nbsp;&nbsp;&nbsp;*(vel (make-list sequence-length :initial-element (instantiate float-vector joint-length)))* <br>&nbsp;&nbsp;&nbsp;*(torque (make-list sequence-length :initial-element (instantiate float-vector joint-length)))* <br>&nbsp;&nbsp;&nbsp;*(root-coords (make-list sequence-length :initial-element (make-coords)))* <br>&nbsp;&nbsp;&nbsp;*(acc (make-list sequence-length :initial-element (instantiate float-vector 3)))* <br>&nbsp;&nbsp;&nbsp;*(zmp (make-list sequence-length :initial-element (instantiate float-vector 3)))* <br>&nbsp;&nbsp;&nbsp;*(wrench (make-list sequence-length :initial-element (instantiate float-vector (* 6 fsensor-length))))* <br>&nbsp;&nbsp;&nbsp;*(optional (make-list sequence-length :initial-element (instantiate float-vector (* 2 fsensor-length))))* <br>&nbsp;&nbsp;&nbsp;*(pos (send-all root-coords :worldpos))* <br>&nbsp;&nbsp;&nbsp;*(rpy (mapcar #'(lambda (x) (reverse (car (rpy-angle (send x :worldrot))))) root-coords))* <br>&nbsp;&nbsp;&nbsp;*(root-local-zmp (mapcar #'(lambda (zz cc) (send cc :inverse-transform-vector zz)) zmp root-coords))* 

- Call service for setJointAnglesSequenceFull. Definition of each sequence is similar to sequence file of loadPattern. <br>
    Arguments type: <br>
     Required <br>
      jpos: sequence of joint angles(float-vector) [deg],  (list av0 av1 ... avn) <br>
      tm: sequence of duration(float) [ms],  (list tm0 tm1 ... tmn) <br>
     Key <br>
      vel: sequence of joint angular velocities(float-vector) [deg/s],  (list vel0 vel1 ... veln) <br>
      torque: sequence of torques(float-vector) [Nm],  (list torque0 torque1 ... torquen) <br>
      root-coords: sequence of waist(root-link) coords in the world frame. (list rc0 rc1 ... rcn). Origin coords by default. <br>
      acc: sequence of waist acc(float-vector) [m/s^2],  (list acc0 acc1 ... accn) <br>
      zmp: sequence of zmp in the world frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). Zero by default. <br>
      wrench: sequence of wrench(float-vector) [N, Nm] for all fsensors,  (list wrench0 wrench1 ... wrenchn) <br>
      optional: sequence of optional(float-vector) [],  (list optional0 optional1 ... optionaln) <br>
     Not required (calculated from other arguments by default), therefore users need not to use these arguments. <br>
      pos: sequence of waist pos(float-vector) [mm] in the world frame,  (list pos0 pos1 ... posn). If root-coords is specified, calculated from root-coords and do not set pos. <br>
      rpy: sequence of waist rpy(float-vector) [rad] in the world frame,  (list rpy0 rpy1 ... rpyn). If root-coords is specified, calculated from root-coords and do not set rpy. <br>
      root-local-zmp: sequence of zmp in the waist(root-link) frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). If root-coords and zmp are specified, calculated from root-coords and zmp and do not set root-local-zmp. <br>


#### :set-ref-moment
&nbsp;&nbsp;&nbsp;*moment* *tm* *&optional* *(limb :arms)* *&key* *(update-robot-state t)* 

- Set reference moment [Nm]. tm is interpolation time [ms]. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-ref-force
&nbsp;&nbsp;&nbsp;*force* *tm* *&optional* *(limb :arms)* *&key* *(update-robot-state t)* 

- Set reference force [N]. tm is interpolation time [ms]. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-ref-force-moment
&nbsp;&nbsp;&nbsp;*force* *moment* *tm* *&optional* *(limb :arms)* *&key* *(update-robot-state t)* 

- Set reference force [N] and moment [Nm]. tm is interpolation time [ms]. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-ref-moments
&nbsp;&nbsp;&nbsp;*moment-list* *tm* *&key* *(update-robot-state t)* 

- Set reference moments. moment-list is list of moment ([Nm]) for all end-effectors. tm is interpolation time [ms]. <br>


#### :set-ref-forces
&nbsp;&nbsp;&nbsp;*force-list* *tm* *&key* *(update-robot-state t)* 

- Set reference forces. force-list is list of force ([N]) for all end-effectors. tm is interpolation time [ms]. <br>


#### :set-ref-forces-moments
&nbsp;&nbsp;&nbsp;*force-list* *moment-list* *tm* 

- Set reference wrenches. wrench-list is list of wrench ([N],[Nm]) for all end-effectors. tm is interpolation time [ms]. <br>


#### :wait-interpolation-seq


- Directly call SequencePlayer waitInterpolation. <br>
    This can be used for force/moment interpolation. <br>


#### :load-pattern
&nbsp;&nbsp;&nbsp;*basename* *&optional* *(tm 0.0)* 

- Load pattern files, such as xx.pos and xx.waist. <br>
    For pattern file definitions, please see loadPattern in SequencePlayer documentation in hrpsys-base API Doc. <br>


#### :set-joint-angles-of-group
&nbsp;&nbsp;&nbsp;*groupname* *av* *tm* 

- Set joint angles of group. <br>
    !!This method is not recommended, please use :angle-vector method like (send *ri* :angle-vector (send *robot* :angle-vector) 2000 :head-controller).!! <br>


#### :remove-joint-group
&nbsp;&nbsp;&nbsp;*groupname* 

- Remove joint group for SequencePlayer. <br>
    groupname is joint group name such as rarm or lleg. <br>


#### :add-joint-group
&nbsp;&nbsp;&nbsp;*groupname* *&optional* *(jnames (if (find-method self (read-from-string (format nil :~A-controller (string-downcase groupname)))) (cdr (assoc :joint-names (car (send self (read-from-string (format nil :~A-controller (string-downcase groupname)))))))))* 

- Add joint group for SequencePlayer. <br>
    groupname is joint group name such as rarm or lleg. <br>
    jnames is list of joint name. <br>


#### :wait-interpolation-of-group
&nbsp;&nbsp;&nbsp;*groupname* 

- Wait interpolation of group. <br>
    !!This method is not recommended, please use :wait-interpolation method like (send *ri* :wait-interpolation :head-controller).!! <br>


#### :set-base-rpy
&nbsp;&nbsp;&nbsp;*base-rpy* *tm* 

- Set base rpy in the world frame. <br>
    base-rpy is [rad] and tm is [ms]. <br>


#### :set-base-pos
&nbsp;&nbsp;&nbsp;*base-pos* *tm* 

- Set base pos in the world frame. <br>
    base-pos is [mm] and tm is [ms]. <br>


#### :set-base-coords
&nbsp;&nbsp;&nbsp;*base-coords* *tm* 

- Set base coordinates in the world frame. <br>
    base-coords is Euslisp coords and tm is [ms]. <br>


:sync-controller *controller* *&optional* *(interpolation-time 1000)* *(blockp t)* 

:set-base-pose *&optional* *base-coords* *(tm 0.1)* 


#### :get-collision-status


- Get collision status. <br>


#### :stop-collision-detection


- Disable collision detection. <br>


#### :start-collision-detection


- Enable collision detection. <br>


#### :set-tolerance
&nbsp;&nbsp;&nbsp;*&key* *(tolerance 0.1)* <br>&nbsp;&nbsp;&nbsp;*(link-pair-name all)* 

- Set tolerance [m] of collision detection with given link-pair-name (all by default). <br>



#### :set-log-maxlength
&nbsp;&nbsp;&nbsp;*&optional* *(maxlength 4000)* 

- Set max log length. <br>
    This method corresponds to DataLogger maxLength(). <br>


#### :start-log


- Start logging. <br>
    This method corresponds to DataLogger clear(). <br>


#### :save-log
&nbsp;&nbsp;&nbsp;*fname* *&key* *(set-robot-date-string t)* <br>&nbsp;&nbsp;&nbsp;*(make-directory nil)* 

- Save log files as [fname].[component_name]_[dataport_name]. <br>
    This method corresponds to DataLogger save(). <br>
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t. <br>
    If make-directory is t and fname is /foo/bar/basename, make directory of /foo/bar/basename and log files will be saved with /foo/bar/basename/basename <br>



#### :calibrate-inertia-sensor


- Calibrate inetria sensor. <br>
    This function takes 10[s]. Please keep the robot static. <br>


#### :set-servo-error-limit
&nbsp;&nbsp;&nbsp;*name* *limit* 

- Set RobotHardware servo error limit [rad] with given name. <br>


#### :remove-force-sensor-offset


- Remove force sensor offset. <br>
    This function takes 10[s]. Please keep the robot static and make sure that robot's sensors do not contact with any objects. <br>


#### :set-servo-gain-percentage
&nbsp;&nbsp;&nbsp;*name* *percentage* 

- Set servo gain percentage [0-100] with given name. <br>



:raw-get-impedance-controller-param *name* 


:raw-set-impedance-controller-param *name* *&rest* *args* *&key* *m-p* *d-p* *k-p* *m-r* *d-r* *k-r* *force-gain* *moment-gain* *sr-gain* *avoid-gain* *reference-gain* *manipulability-limit* *controller-mode* *ik-optional-weight-vector* *use-sh-base-pos-rpy* 


#### :get-impedance-controller-param-arguments


- Get arguments of :raw-set-impedance-controller-param <br>



#### :get-impedance-controller-controller-mode
&nbsp;&nbsp;&nbsp;*name* 

- Get ImpedanceController ControllerMode as Euslisp symbol. <br>


#### :get-impedance-controller-param
&nbsp;&nbsp;&nbsp;*limb* 

- Get impedance controller parameter. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :set-impedance-controller-param
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Set impedance controller parameter like (send *ri* :set-impedance-controller-param :rarm :K-p 400). <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>
    For arguments, please see (send *ri* :get-impedance-controller-param-arguments). <br>


#### :stop-impedance
&nbsp;&nbsp;&nbsp;*limb* 

- Stop impedance controller mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


#### :start-impedance
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Start impedance controller mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs. <br>


:force-sensor-method *limb* *method-func* *method-name* *&rest* *args* 

:wait-impedance-controller-transition *limb* 

:stop-impedance-no-wait *limb* 

:start-impedance-no-wait *limb* 

:raw-start-impedance *limb* 


#### :get-object-turnaround-detector-param


- Get objectcontactturnarounddetectorparam. <br>



:raw-set-object-turnaround-detector-param *&rest* *args* *&key* *wrench-cutoff-freq* *dwrench-cutoff-freq* *detect-ratio-thre* *start-ratio-thre* *detect-time-thre* *start-time-thre* *axis* *moment-center* *detector-total-wrench* 


#### :get-object-turnaround-detector-param-arguments


- Get arguments of :raw-set-object-turnaround-detector-param <br>



#### :get-object-turnaround-detector-detector-total-wrench


- Get Object Turnadound Detector as Euslisp symbol. <br>


#### :set-object-turnaround-detector-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *detector-total-wrench* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set Object Turnaround Detector. <br>
    For arguments, please see (send *ri* :get-object-turnaround-detector-param-arguments). <br>


#### :set-object-turnaround-ref-moment
&nbsp;&nbsp;&nbsp;*&key* *(limbs '(:rarm :larm))* <br>&nbsp;&nbsp;&nbsp;*(axis (float-vector 0 0 1))* <br>&nbsp;&nbsp;&nbsp;*(max-ref-moment 10)* <br>&nbsp;&nbsp;&nbsp;*(func)* <br>&nbsp;&nbsp;&nbsp;*(moment-center)* <br>&nbsp;&nbsp;&nbsp;*(max-time 4000.0)* <br>&nbsp;&nbsp;&nbsp;*(detect-time-offset 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-time 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-p t)* <br>&nbsp;&nbsp;&nbsp;*(periodic-time 200)* <br>&nbsp;&nbsp;&nbsp;*(detector-total-wrench :total-moment)* <br>&nbsp;&nbsp;&nbsp;*(return-value-mode :all)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-linear-p)* 

- Set object turnaround reference force. <br>
    axis is axis which resultant moment is around. <br>
    max-time is max time [msec] and max-ref-moment is max reference resultant moment. <br>
    func is function to distribute estimated resultant moment to hands' forces/moments. <br>
    moment-center is moment center [mm]. <br>
    detect-time-offset is additional checking time [msec]. <br>
    set-ref-force-time is time to set reference force [msec]. <br>
    If set-ref-force-p is t, set estimated reference force. Otherwise, set reference force to zero. <br>
    periodic-time is loop wait time[msec]. <br>
    return-value-mode is used to select return value type. <br>
    If :all, returns the results of :get-otd-object-forces-moments itself. If :forces, returns forces results. <br>
    If set-ref-force-linear-p t, set linear interpolation first and set hoffarbib interpolation in return. <br>


#### :set-object-turnaround-ref-force
&nbsp;&nbsp;&nbsp;*&key* *(limbs '(:rarm :larm))* <br>&nbsp;&nbsp;&nbsp;*(axis (float-vector 0 0 -1))* <br>&nbsp;&nbsp;&nbsp;*(max-time 4000.0)* <br>&nbsp;&nbsp;&nbsp;*(max-ref-force)* <br>&nbsp;&nbsp;&nbsp;*(detect-time-offset 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-time 2000.0)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-p t)* <br>&nbsp;&nbsp;&nbsp;*(periodic-time 200)* <br>&nbsp;&nbsp;&nbsp;*(detector-total-wrench :total-force)* <br>&nbsp;&nbsp;&nbsp;*(return-value-mode :forces)* <br>&nbsp;&nbsp;&nbsp;*(set-ref-force-linear-p)* 

- Set object turnaround reference force. <br>
    axis is resultant force direction. <br>
    max-time is max time [msec] and max-ref-force is max reference resultant force. <br>
    max-ref-force is max resultant reference force [N]. max-ref-force is added to original ref forces. <br>
    detect-time-offset is additional checking time [msec]. <br>
    set-ref-force-time is time to set reference force [msec]. <br>
    If set-ref-force-p is t, set estimated reference force. Otherwise, set reference force to zero. <br>
    periodic-time is loop wait time[msec]. <br>
    return-value-mode is used to select return value type. <br>
    If :all, returns the results of :get-otd-object-forces-moments itself. If :forces, returns forces results. <br>
    If set-ref-force-linear-p t, set linear interpolation first and set hoffarbib interpolation in return. <br>


#### :get-otd-object-forces-moments


- Get ObjectContactTurnaroundDetector's curernt forces and moments for used limbs. <br>
    Return value is (list force-list moment-list). <br>


#### :check-object-turnaround-detection


- Check object contact turnaround detection. <br>
   If t, detected. Otherwise, not detected. <br>


#### :start-object-turnaround-detection
&nbsp;&nbsp;&nbsp;*&key* *(ref-diff-wrench)* <br>&nbsp;&nbsp;&nbsp;*(max-time)* <br>&nbsp;&nbsp;&nbsp;*(limbs)* 

- Start ObjectContactTurnaroundDetection mode. <br>
    ref-diff-wrench is final reference wrench (scalar). <br>
    max-time is max time [msec]. <br>
    limbs is limb list to be used. <br>



:raw-get-forcemoment-offset-param *name* 


:raw-set-forcemoment-offset-param *name* *&rest* *args* *&key* *force-offset* *moment-offset* *link-offset-centroid* *link-offset-mass* 


#### :get-forcemoment-offset-param-arguments


- Get arguments of :raw-set-forcemoment-offset-param <br>



#### :reset-force-moment-offset
&nbsp;&nbsp;&nbsp;*limbs* 

- Remove force and moment offsets. limbs should be list of limb symbol name. <br>


#### :reset-force-moment-offset-legs
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 0.1)* 

- time[s] <br>


#### :reset-force-moment-offset-arms
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 0.1)* 

- time[s] <br>


#### :remove-force-sensor-offset-rmfo-legs
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 8.0)* 

- Remove force and moment offset for :rleg and :lleg. <br>
    time is duration of calibration[s]. 8.0[s] by default. <br>


#### :remove-force-sensor-offset-rmfo-arms
&nbsp;&nbsp;&nbsp;*&key* *((:time tm) 8.0)* 

- Remove force and moment offset for :rarm and :larm. <br>
    time is duration of calibration[s]. 8.0[s] by default. <br>


#### :remove-force-sensor-offset-rmfo
&nbsp;&nbsp;&nbsp;*&key* *(limbs)* <br>&nbsp;&nbsp;&nbsp;*((:time tm) 8.0)* 

- remove offsets on sensor outputs form force/torque sensors. <br>
    Sensor offsets (force_offset and moment_offset in ForceMomentOffsetParam) are calibrated. <br>
    Please keep the robot static and make sure that robot's sensors do not contact with any objects. <br>
    Argument: <br>
      limbs is list of sensor names to be calibrated. <br>
      If not specified, all sensors are calibrated by default. <br>
      time is duration of calibration[s]. 8.0[s] by default. <br>
    Return: <br>
      t if set successfully, nil otherwise <br>


#### :dump-forcemoment-offset-params
&nbsp;&nbsp;&nbsp;*filename* *&key* *(set-robot-date-string t)* 

- Save all RMFO offset parameters. <br>
    This method corresponds to RemoveForceSensorLinkOffset dumpForceMomentOffsetParams(). <br>
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t. <br>


#### :load-forcemoment-offset-params
&nbsp;&nbsp;&nbsp;*filename* 

- Load RMFO offset parameters from parameter file. <br>
    This method corresponds to RemoveForceSensorLinkOffset loadForceMomentOffsetParams(). <br>


#### :load-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*fname* *&key* *(set-offset t)* 

- Load RemoveForceSensorLinkOffset params from fname (file path). <br>


#### :get-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*limb* 

- Get RemoveForceSensorLinkOffset params for given limb. <br>


#### :set-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Set RemoveForceSensorLinkOffset params for given limb. <br>
    For arguments, please see (send *ri* :get-forcemoment-offset-param-arguments). <br>


#### :zero-set-forcemoment-offset-param
&nbsp;&nbsp;&nbsp;*limb* 

- Set RemoveForceSensorLinkOffset's params offset to zero. <br>


:_reset-force-moment-offset *limbs* *f/m* *&key* *(itr 10)* 


#### :get-gait-generator-param


- Get gaitgeneratorparam. <br>



:raw-set-gait-generator-param *&rest* *args* *&key* *default-step-time* *default-step-height* *default-double-support-ratio* *default-double-support-ratio-before* *default-double-support-ratio-after* *default-double-support-static-ratio* *default-double-support-static-ratio-before* *default-double-support-static-ratio-after* *default-double-support-ratio-swing-before* *default-double-support-ratio-swing-after* *stride-parameter* *stride-limitation-for-circle-type* *default-orbit-type* *swing-trajectory-delay-time-offset* *swing-trajectory-final-distance-weight* *stair-trajectory-way-point-offset* *cycloid-delay-kick-point-offset* *swing-trajectory-time-offset-xy2z* *gravitational-acceleration* *toe-pos-offset-x* *heel-pos-offset-x* *toe-zmp-offset-x* *heel-zmp-offset-x* *toe-angle* *heel-angle* *toe-check-thre* *heel-check-thre* *toe-heel-phase-ratio* *use-toe-joint* *use-toe-heel-transition* *use-toe-heel-auto-set* *zmp-weight-map* *leg-default-translate-pos* *optional-go-pos-finalize-footstep-num* *overwritable-footstep-index-offset* *overwritable-stride-limitation* *use-stride-limitation* *stride-limitation-type* *leg-margin* *footstep-modification-gain* *modify-footsteps* *cp-check-margin* *margin-time-ratio* 


#### :get-gait-generator-param-arguments


- Get arguments of :raw-set-gait-generator-param <br>



#### :get-auto-balancer-param


- Get autobalancerparam. <br>



:raw-set-auto-balancer-param *&rest* *args* *&key* *default-zmp-offsets* *move-base-gain* *controller-mode* *use-force-mode* *graspless-manip-mode* *graspless-manip-arm* *graspless-manip-p-gain* *graspless-manip-reference-trans-pos* *graspless-manip-reference-trans-rot* *transition-time* *zmp-transition-time* *adjust-footstep-transition-time* *leg-names* *has-ik-failed* *pos-ik-thre* *rot-ik-thre* *is-hand-fix-mode* *end-effector-list* *default-gait-type* *ik-limb-parameters* *use-limb-stretch-avoidance* *limb-stretch-avoidance-time-const* *limb-stretch-avoidance-vlimit* *limb-length-margin* *additional-force-applied-link-name* *additional-force-applied-point-offset* 


#### :get-auto-balancer-param-arguments


- Get arguments of :raw-set-auto-balancer-param <br>



#### :calc-hand-trans-coords-single-arm
&nbsp;&nbsp;&nbsp;*robot* *arm* 

- Calculate foot->hand coords transformation for single-arm graspless manipulation. <br>


#### :calc-hand-trans-coords-dual-arms
&nbsp;&nbsp;&nbsp;*robot* 

- Calculate foot->hand coords transformation for dual-arm graspless manipulation. <br>


#### :stop-graspless-manip-mode


- Stop graspless manip mode while walking. <br>


#### :start-graspless-manip-mode
&nbsp;&nbsp;&nbsp;*robot* *arm* 

- Start graspless manip mode while walking. <br>
    robot is robot instance which angle-vector is for graspless manip. <br>
    arm is used arm (:rarm, :larm, :arms). <br>


#### :set-default-step-time-with-the-same-swing-time
&nbsp;&nbsp;&nbsp;*default-step-time* 

- Set default step time with the same swing time. <br>


#### :calc-dvel-with-velocity-center-offset
&nbsp;&nbsp;&nbsp;*ang* *velocity-center-offset* 

- Calculate velocity params for rotating with given velocity center offset. <br>
    Ang : [deg], offset vector [mm] <br>


#### :cmd-vel-mode


- Walk with subscribing /cmd_vel topic. <br>


#### :get-auto-balancer-use-force-mode


- Get AutoBalancer UseForceMode as Euslisp symbol. <br>


#### :print-auto-balancer-use-force-mode


- Print AutoBalancer UseForceMode. <br>


#### :set-auto-balancer-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *leg-names* <br>&nbsp;&nbsp;&nbsp;*default-zmp-offsets* <br>&nbsp;&nbsp;&nbsp;*graspless-manip-arm* <br>&nbsp;&nbsp;&nbsp;*graspless-manip-reference-trans-pos* <br>&nbsp;&nbsp;&nbsp;*graspless-manip-reference-trans-rot* <br>&nbsp;&nbsp;&nbsp;*use-force-mode* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set AutoBalancer param. <br>
    For arguments, please see (send *ri* :get-auto-balancer-param-arguments). <br>


#### :get-auto-balancer-param


- Get AutoBalancer param. <br>


#### :get-auto-balancer-controller-mode


- Get AutoBalancer ControllerMode as Euslisp symbol. <br>


#### :set-gait-generator-toe-heel-angles
&nbsp;&nbsp;&nbsp;*toe-angle* *heel-angle* *double-support-ratio-half* 

- Set toe-angle, heel-angle, and toe-heel-phase-ratio. <br>
    toe-heel-phase-ratio is automatically calculated. <br>
    Arguments: <br>
      toe-angle, heel-angle : max toe/heel angle [deg]. <br>
      double-support-ratio-half : half of double support ratio in terms of toe heel phase ratio. <br>


#### :calc-toe-heel-phase-ratio
&nbsp;&nbsp;&nbsp;*toe-angle* *heel-angle* *double-support-ratio-half* 

- Calculate vector for toe heel phase ratio. <br>
    Arguments: <br>
      toe-angle, heel-angle : max toe/heel angle [deg]. <br>
      double-support-ratio-half : half of double support ratio in terms of toe heel phase ratio. <br>


#### :get-gait-generator-orbit-type


- Get GaitGenerator Orbit Type as Euslisp symbol. <br>


#### :print-gait-generator-orbit-type


- Print GaitGenerator orbit types. <br>


#### :set-gait-generator-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *default-orbit-type* <br>&nbsp;&nbsp;&nbsp;*leg-default-translate-pos* <br>&nbsp;&nbsp;&nbsp;*stride-parameter* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set gait generator param. <br>
    For arguments, please see (send *ri* :get-gait-generator-param-arguments). <br>


#### :wait-foot-steps


- Wait for whole footsteps are executed. <br>


#### :calc-go-velocity-param-from-velocity-center-offset
&nbsp;&nbsp;&nbsp;*ang* *velocity-center-offset* 

- Calculate go-velocity velocities from rotation center and rotation angle. <br>
    ang is rotation angle [rad]. velocity-center-offset is velocity center offset [mm] from foot mid coords. <br>


#### :emergency-walking-stop


- Stop stepping immediately. <br>


#### :go-stop


- Stop stepping. <br>


#### :go-velocity
&nbsp;&nbsp;&nbsp;*vx* *vy* *vth* 

- Call goVelocity. <br>


#### :draw-remaining-foot-step-sequence
&nbsp;&nbsp;&nbsp;*vwer* *&key* *(flush)* <br>&nbsp;&nbsp;&nbsp;*(rleg-color #f(1.0 0.0 0.0))* <br>&nbsp;&nbsp;&nbsp;*(lleg-color #f(0.0 1.0 0.0))* <br>&nbsp;&nbsp;&nbsp;*(change-support-leg-color t)* <br>&nbsp;&nbsp;&nbsp;*(support-leg-color #f(1.0 1.0 1.0))* 

- Draw remaining foot steps. <br>


#### :get-go-pos-footsteps-sequence
&nbsp;&nbsp;&nbsp;*xx* *yy* *th* 

- Get foot steps of go-pos without executing them. <br>
   Return is list of list of footstep. <br>


#### :get-remaining-foot-step-sequence


- Get remaining foot steps from GaitGenerator. <br>
   Return is (list current-support-foot-coords remaining-swing-dst-coords-0 ... ). <br>


#### :get-current-footstep-index


- Get current footstep index. <br>


#### :get-remaining-foot-step-sequence-current-index


- Get remaining foot steps from GaitGenerator and current index. <br>
   Return is (list (list current-support-foot-coords remaining-swing-dst-coords-0 ... ) current-index). <br>


#### :adjust-foot-steps-roll-pitch
&nbsp;&nbsp;&nbsp;*angle* *&key* *(axis :x)* 

- Adjust foot steps with roll or pitch orientation. <br>
    angle is roll or pitch angle [deg]. <br>
    axis is :x (roll) or :y (pitch). <br>


#### :adjust-foot-steps
&nbsp;&nbsp;&nbsp;*rfoot-coords* *lfoot-coords* 

- Adjust current footsteps during autobalancer mode and not walking. <br>
    rfoot-coords and lfoot-coords are end-coords for new foot steps. <br>


#### :set-foot-steps-with-param-and-base-height
&nbsp;&nbsp;&nbsp;*fs-params* *av-list* *time-list* 

- Set foot steps and params with sending angle-vector. <br>


#### :set-foot-steps-with-base-height
&nbsp;&nbsp;&nbsp;*fs* *av-list* *time-list* 

- Set foot steps with sending angle-vector. <br>


#### :set-foot-steps-roll-pitch
&nbsp;&nbsp;&nbsp;*angle* *&key* *(axis :x)* 

- Set foot steps with roll or pitch orientation. <br>
    angle is roll or pitch angle [deg]. <br>
    axis is :x (roll) or :y (pitch). <br>


#### :set-foot-steps-with-param
&nbsp;&nbsp;&nbsp;*foot-step-list* *step-height-list* *step-time-list* *toe-angle-list* *heel-angle-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step with step parameter and wait for step finish. <br>
    For arguments, please see :set-foot-steps-with-param-no-wait documentation. <br>


#### :set-foot-steps-with-param-no-wait
&nbsp;&nbsp;&nbsp;*foot-step-list* *step-height-list* *step-time-list* *toe-angle-list* *heel-angle-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step with step parameter and do not wait for step finish. <br>
    foot-step-list is list of footstep (only biped) or list of list of footstep. <br>
    step-height-list is list of step height (only biped) or list of list of step height. <br>
    step-time-list is list of step time (only biped) or list of list of step time. <br>
    toe-angle-list is list of toe angle (only biped) or list of list of toe angle. <br>
    heel-angle-list is list of heel angle (only biped) or list of list of heel angle. <br>


#### :set-foot-steps
&nbsp;&nbsp;&nbsp;*foot-step-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step by default parameters and wait for step finish. <br>
    foot-step-list is list of footstep (only biped) or list of list of footstep. <br>
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking. <br>


#### :set-foot-steps-no-wait
&nbsp;&nbsp;&nbsp;*foot-step-list* *&key* *(overwrite-footstep-index 0)* 

- Set foot step by default parameters and do not wait for step finish. <br>
    foot-step-list is list of footstep (only biped) or list of list of footstep. <br>
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking. <br>


#### :get-foot-step-param
&nbsp;&nbsp;&nbsp;*param-name* 

- Get AutoBalancer foot step param by given name. <br>
    param-name is key word for parameters defined in IDL. <br>
    param-name should be :rleg-coords :lleg-coords :support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords :support-leg :support-leg-with-both. <br>


#### :get-foot-step-params


- Get AutoBalancer foot step params. <br>


#### :go-pos
&nbsp;&nbsp;&nbsp;*xx* *yy* *th* 

- Call goPos with wait. <br>


#### :go-pos-no-wait
&nbsp;&nbsp;&nbsp;*xx* *yy* *th* 

- Call goPos without wait. <br>


#### :stop-auto-balancer


- Stop auto balancer mode <br>


#### :start-auto-balancer
&nbsp;&nbsp;&nbsp;*&key* *(limbs (if (not (every #'null (send robot :arms))) '(:rleg :lleg :rarm :larm) '(:rleg :lleg)))* 

- startAutoBalancer. <br>
    If robot with arms, start auto balancer with legs and arms ik by default. <br>
    Otherwise, start auto balancer with legs ik by default. <br>


:stop-cmd-vel-mode 

:start-cmd-vel-mode 

:cmd-vel-cb *msg* *&key* *(vel-x-ratio 1.0)* *(vel-y-ratio 1.0)* *(vel-th-ratio 1.0)* 

:eus-footstep->abc-footstep *f* 

:abc-footstep->eus-footstep *f* 

:raw-get-foot-step-param 


#### :set-soft-error-limit
&nbsp;&nbsp;&nbsp;*name* *limit* 

- Set SoftErrorLimiter servo error limit [rad] with given name. <br>



#### :get-st-param


- Get stparam. <br>



:raw-set-st-param *&rest* *args* *&key* *k-tpcc-p* *k-tpcc-x* *k-brot-p* *k-brot-tc* *eefm-k1* *eefm-k2* *eefm-k3* *eefm-zmp-delay-time-const* *eefm-ref-zmp-aux* *eefm-rot-damping-gain* *eefm-rot-time-const* *eefm-pos-damping-gain* *eefm-pos-time-const-support* *eefm-swing-rot-spring-gain* *eefm-swing-rot-time-const* *eefm-swing-pos-spring-gain* *eefm-swing-pos-time-const* *eefm-ee-moment-limit* *eefm-pos-compensation-limit* *eefm-rot-compensation-limit* *eefm-pos-time-const-swing* *eefm-pos-transition-time* *eefm-pos-margin-time* *eefm-leg-inside-margin* *eefm-leg-outside-margin* *eefm-leg-front-margin* *eefm-leg-rear-margin* *eefm-body-attitude-control-gain* *eefm-body-attitude-control-time-const* *eefm-cogvel-cutoff-freq* *eefm-wrench-alpha-blending* *eefm-alpha-cutoff-freq* *eefm-gravitational-acceleration* *eefm-ee-pos-error-p-gain* *eefm-ee-rot-error-p-gain* *eefm-ee-error-cutoff-freq* *eefm-support-polygon-vertices-sequence* *eefm-use-force-difference-control* *eefm-use-swing-damping* *eefm-swing-damping-force-thre* *eefm-swing-damping-moment-thre* *eefm-swing-rot-damping-gain* *eefm-swing-pos-damping-gain* *eefm-ee-forcemoment-distribution-weight* *st-algorithm* *controller-mode* *transition-time* *is-ik-enable* *is-feedback-control-enable* *is-zmp-calc-enable* *cop-check-margin* *cp-check-margin* *tilt-margin* *ref-capture-point* *act-capture-point* *cp-offset* *contact-decision-threshold* *foot-origin-offset* *emergency-check-mode* *end-effector-list* *is-estop-while-walking* *ik-limb-parameters* *use-limb-stretch-avoidance* *limb-stretch-avoidance-time-const* *limb-stretch-avoidance-vlimit* *limb-length-margin* *detection-time-to-air* *root-rot-compensation-limit* *use-zmp-truncation* 


#### :get-st-param-arguments


- Get arguments of :raw-set-st-param <br>



#### :stop-st


- Stop Stabilizer Mode. <br>


#### :start-st


- Start Stabilizer Mode. <br>


#### :get-st-algorithm


- Get Stabilizer Algorithm as Euslisp symbol. <br>


#### :get-st-controller-mode


- Get Stabilizer ControllerMode as Euslisp symbol. <br>


#### :set-st-param-by-ratio
&nbsp;&nbsp;&nbsp;*state-feedback-gain-ratio* *damping-control-gain-ratio* *attitude-control-gain-ratio* 

- Set Stabilzier parameter by ratio from default parameters. <br>
    state-feedback-gain-ratio is ratio for state feedback gain. <br>
    damping-control-gain-ratio is ratio for damping control gain. <br>
    attitude-control-gain-ratio is ratio for attitude control gain. <br>
    When ratio = 1, parameters are same as default parameters. <br>
    For state-feedback-gain-ratio and attitude-control-gain-ratio, <br>
    when ratio < 1 Stabilzier works less feedback control and less oscillation and when ratio > 1 more feedback and more oscillation. <br>
    For dampnig-control-gain-ratio, <br>
    when ratio > 1 feet behavior becomes hard and safe and when ratio < 1 soft and dangerous. <br>


#### :set-default-st-param


- Set Stabilzier parameter by default parameters. <br>


#### :set-st-param-for-non-feedback-lip-mode


- Set Stabilizer parameters to make robot Linear Inverted Pendulum mode without state feedback. <br>
    By using this mode, robot feets adapt to ground surface. <br>


#### :set-st-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *st-algorithm* <br>&nbsp;&nbsp;&nbsp;*eefm-pos-damping-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-rot-damping-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-pos-time-const-support* <br>&nbsp;&nbsp;&nbsp;*eefm-rot-time-const* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-pos-spring-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-pos-time-const* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-rot-spring-gain* <br>&nbsp;&nbsp;&nbsp;*eefm-swing-rot-time-const* <br>&nbsp;&nbsp;&nbsp;*eefm-ee-forcemoment-distribution-weight* <br>&nbsp;&nbsp;&nbsp;*&allow-other-keys* 

- Set Stabilizer parameters. <br>
    For arguments, please see (send *ri* :get-st-param-arguments). <br>



#### :get-kalman-filter-param


- Get kalmanfilterparam. <br>



#### :set-kalman-filter-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *q-angle* <br>&nbsp;&nbsp;&nbsp;*q-rate* <br>&nbsp;&nbsp;&nbsp;*r-angle* <br>&nbsp;&nbsp;&nbsp;*kf-algorithm* <br>&nbsp;&nbsp;&nbsp;*acc-offset* <br>&nbsp;&nbsp;&nbsp;*sensorrpy-offset* 

- Set kalmanfilterparam. For arguments, please see (send *ri* :get-kalman-filter-param-arguments). <br>



#### :get-kalman-filter-param-arguments


- Get arguments of :set-kalman-filter-param <br>



#### :get-kalman-filter-algorithm


- Get KalmanFilter Algorithm as Euslisp symbol. <br>



#### :get-emergency-stopper-param


- Get emergencystopperparam. <br>



#### :set-emergency-stopper-param
&nbsp;&nbsp;&nbsp;*&rest* *args* *&key* *default-recover-time* <br>&nbsp;&nbsp;&nbsp;*default-retrieve-time* <br>&nbsp;&nbsp;&nbsp;*is-stop-mode* 

- Set emergencystopperparam. For arguments, please see (send *ri* :get-emergency-stopper-param-arguments). <br>



#### :get-emergency-stopper-param-arguments


- Get arguments of :set-emergency-stopper-param <br>



#### :emergency-mode


- Returns emergency mode. <br>


#### :hard-emergency-release-motion


- Release emergency motion stopping in hard EmergencyStopper. <br>


#### :emergency-release-motion


- Release emergency motion stopping in EmergencyStopper. <br>


#### :hard-emergency-stop-motion


- Stop motion emergently in hard EmergencyStopper. <br>


#### :emergency-stop-motion


- Stop motion emergently in EmergencyStopper. <br>



:stop-grasp *limb* 

:start-grasp *limb* *&key* *(target-error)* *(gain-percentage)* 

:get-hand-config-list 


#### :stop-default-unstable-controllers
&nbsp;&nbsp;&nbsp;*&key* *(ic-limbs '(:rarm :larm))* 

- Stop default unstable RTCs controller mode. <br>
    Currently Stabilzier, AutoBalancer, and ImpedanceController are stopped. <br>


#### :start-default-unstable-controllers
&nbsp;&nbsp;&nbsp;*&key* *(ic-limbs '(:rarm :larm))* <br>&nbsp;&nbsp;&nbsp;*(abc-limbs (if (not (every #'null (send robot :arms))) '(:rleg :lleg :rarm :larm) '(:rleg :lleg)))* 

- Start default unstable RTCs controller mode. <br>
    Currently Stabilzier, AutoBalancer, and ImpedanceController are started. <br>



:raw-get-reference-force-updater-param *name* 


:raw-set-reference-force-updater-param *name* *&rest* *args* *&key* *motion-dir* *frame* *update-freq* *update-time-ratio* *p-gain* *d-gain* *i-gain* *is-hold-value* 


#### :get-reference-force-updater-param-arguments


- Get arguments of :raw-set-reference-force-updater-param <br>



#### :get-reference-force-updater-param
&nbsp;&nbsp;&nbsp;*limb* 

- Get reference force updater parameter. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :set-reference-force-updater-param
&nbsp;&nbsp;&nbsp;*limb* *&rest* *args* 

- Set reference force updater parameter like (send *ri* :set-reference-force-updater-param :rarm :p-gain 0.02). <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>
    For arguments, please see (send *ri* :get-reference-force-updater-param-arguments) <br>


#### :stop-reference-force-updater-no-wait
&nbsp;&nbsp;&nbsp;*limb* 

- Start reference force updater mode without wait. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :start-reference-force-updater-no-wait
&nbsp;&nbsp;&nbsp;*limb* 

- Start reference force updater mode without wait. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :stop-reference-force-updater
&nbsp;&nbsp;&nbsp;*limb* 

- Stop reference force updater mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :start-reference-force-updater
&nbsp;&nbsp;&nbsp;*limb* 

- Start reference force updater mode. <br>
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list). <br>


#### :get-supported-reference-force-updater-name-list


- Get supported reference force updater names (:rarm, :larm, ...) <br>



#### :get-torque-controller-param
&nbsp;&nbsp;&nbsp;*jname* 

- Get torquecontrollerparam. <br>



#### :set-torque-controller-param
&nbsp;&nbsp;&nbsp;*jname* *&rest* *args* *&key* *tc* <br>&nbsp;&nbsp;&nbsp;*ke* <br>&nbsp;&nbsp;&nbsp;*kd* <br>&nbsp;&nbsp;&nbsp;*ki* <br>&nbsp;&nbsp;&nbsp;*alpha* <br>&nbsp;&nbsp;&nbsp;*beta* 

- Set torquecontrollerparam. For arguments, please see (send *ri* :get-torque-controller-param-arguments). <br>



#### :get-torque-controller-param-arguments


- Get arguments of :set-torque-controller-param <br>



#### :set-reference-torque
&nbsp;&nbsp;&nbsp;*jname* *tauref* 

- set reference torque in specified joint <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>
    tauref (reference torque [N/m]) should be a number (for one joint) or list of numbers (for multiple joints) <br>


#### :stop-torque-control
&nbsp;&nbsp;&nbsp;*jname* 

- stop torque controller for specified joint (this method does not disable torque control) <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>


#### :start-torque-control
&nbsp;&nbsp;&nbsp;*jname* *&optional* *(tauref nil)* 

- start torque control in specified joint. set reference torque before start if tauref is given. <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>
    tauref is reference torque [N/m], should be a number (for one joint) or list of numbers (for multiple joints) <br>


#### :disable-torque-control
&nbsp;&nbsp;&nbsp;*jname* 

- disable torque control in specified joint (torque controller do nothing and pass qRefIn when disabeld) <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>


#### :enable-torque-control
&nbsp;&nbsp;&nbsp;*jname* 

- enable torque controller for specified joint (only start torque control when tau exceeds tauMax) <br>
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints) <br>



#### :print-rtc-param-all


- Print RTC parameter struct probided by :get-xx method in more readable style for all recommended RTCs. <br>
    Check all possible RTCs by checking existence of ROSBridge nodes. <br>



### print-end-effector-parameter-conf-from-robot ###
&nbsp;&nbsp;&nbsp;*rb* 

- Print end effector setting for hrpsys conf file. <br>


### dump-seq-pattern-file ###
&nbsp;&nbsp;&nbsp;*rs-list* *output-basename* *&key* *(initial-sync-time 3.0)* <br>&nbsp;&nbsp;&nbsp;*(robot)* 

- Dump pattern file for SequencePlayer. <br>
     rs-list : list of (list :time time0 :angle-vector av :root-coords rc ...). <br>
               Fields other than :time and :angle-vector are optional. <br>
     output-basename : output file (output-basename.pos, ...). <br>
     root-coords : worldcoords for root link. <br>
     zmp : world zmp[mm]. <br>
     wrench-list : world (list force-list moment-list) at end effector. <br>


### add-optional-data-from-rs-list ###
&nbsp;&nbsp;&nbsp;*rs-list* *robot* *&key* *(all-limbs (mapcar #'(lambda (fs) (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) '(:rleg :lleg :rarm :larm))) (send robot :force-sensors)))* <br>&nbsp;&nbsp;&nbsp;*(limbs (remove nil (mapcar #'(lambda (fs) (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) (case (length (cadr (memq :contact-state (car rs-list)))) (2 '(:rleg :lleg)) (4 '(:rleg :lleg :rarm :larm))))) (send robot :force-sensors))))* <br>&nbsp;&nbsp;&nbsp;*(add-optional-data-p t)* 

- Add optionalData from rs-list. <br>
   all-limbs is all limbs with ForceSensor in VRML. <br>
   limbs is all limbs included in rs-list. <br>
   If add-optional-data-p is nil, return contact state and swing support time list. <br>
   :contact-state is required in rs-list. <br>
   :support = 1.0, :swing = 0.0 <br>


### load-from-seq-pattern-file ###
&nbsp;&nbsp;&nbsp;*input-basename* 

- Load from seq pattern file and generate robot state list. <br>


### calculate-eefm-st-state-feedback-gain ###
&nbsp;&nbsp;&nbsp;*default-cog-height* *&key* *(alpha -13.0)* <br>&nbsp;&nbsp;&nbsp;*(beta -4.0)* <br>&nbsp;&nbsp;&nbsp;*((:time-constant tp) 0.04)* <br>&nbsp;&nbsp;&nbsp;*((:gravitational-acceleration ga) (* 0.001 (elt *g-vec* 2)))* <br>&nbsp;&nbsp;&nbsp;*((:print-mode pm) :euslisp)* 

- Calculate EEFMe st state feedback gain (k1, k2, k3) and print them. <br>
   default-cog-height is default COG height[mm]. <br>
   alpha and beta are poles. -13.0 and -4.0 by default. <br>
   time-constant is time constant of ZMP tracking delay [s]. 0.04[s] by default. <br>
   gravitational-acceleration is gravitational acceleration [m/s^2]. <br>
   When print-mode is :euslisp, print k1, k2, and k3 in Euslisp manner. <br>
   When print-mode is :python, print k1, k2, and k3 in python hrpsys_config manner. <br>


### calculate-eefm-st-state-feedback-default-gain-from-robot ###
&nbsp;&nbsp;&nbsp;*robot* *&key* *(alpha -13.0)* <br>&nbsp;&nbsp;&nbsp;*(beta -4.0)* <br>&nbsp;&nbsp;&nbsp;*((:time-constant tp) 0.04)* <br>&nbsp;&nbsp;&nbsp;*((:gravitational-acceleration ga) (* 0.001 (elt *g-vec* 2)))* <br>&nbsp;&nbsp;&nbsp;*((:print-mode pm) :euslisp)* <br>&nbsp;&nbsp;&nbsp;*(exec-reset-pose-p t)* 

- Calculate EEFMe st state feedback gain (k1, k2, k3) from robot model and print them. <br>
   robot is robot model and calculate gains from reset-pose by default. <br>


### calculate-toe-heel-pos-offsets ###
&nbsp;&nbsp;&nbsp;*robot* *&key* *((:print-mode pm) :euslisp)* 

- Calculate toe and heel position offset in ee frame for toe heel contact used in GaitGenerator in AutoBalancer. <br>
   robot is robot model. <br>
   When print-mode is :euslisp, print parameters in Euslisp style. <br>
   When print-mode is :python, print parameters in Python hrpsys_config style. <br>


### calculate-toe-heel-zmp-offsets ###
&nbsp;&nbsp;&nbsp;*robot* *&key* *((:print-mode pm) :euslisp)* 

- Calculate toe and heel zmp offset in ee frame for toe heel contact used in GaitGenerator in AutoBalancer. <br>
   robot is robot model. <br>
   When print-mode is :euslisp, print parameters in Euslisp style. <br>
   When print-mode is :python, print parameters in Python hrpsys_config style. <br>


### calculate-sole-margin-params ###
&nbsp;&nbsp;&nbsp;*robot* *&key* *((:print-mode pm) :euslisp)* <br>&nbsp;&nbsp;&nbsp;*(margin-ratio 1.0)* 

- Calculate sole edge margin in Stabilizer. <br>
   robot is robot model. <br>
   margin-ratio is multiplied by values (1.0 by default). <br>
   When print-mode is :euslisp, print parameters in Euslisp style. <br>
   When print-mode is :python, print parameters in Python hrpsys_config style. <br>


### dump-project-file-by-cloning-euslisp-models ###
&nbsp;&nbsp;&nbsp;*robot* *robot-file-path* *&key* *(object-models)* <br>&nbsp;&nbsp;&nbsp;*(object-models-file-path)* <br>&nbsp;&nbsp;&nbsp;*(timestep 0.005)* <br>&nbsp;&nbsp;&nbsp;*(dt 0.005)* <br>&nbsp;&nbsp;&nbsp;*(use-highgain-mode t)* <br>&nbsp;&nbsp;&nbsp;*(integrate t)* <br>&nbsp;&nbsp;&nbsp;*(method :euler)* <br>&nbsp;&nbsp;&nbsp;*(output-fname (format nil /tmp/~A (send robot :name)))* <br>&nbsp;&nbsp;&nbsp;*(debug)* 

- Clone euslisp robot and objects to OpenHRP3 project file. <br>
   Add euslisp model + locate euslisp model in OpenHRP3 world. <br>
   Arguments (required) <br>
      robot  : Robot model <br>
      robot-file-path : Path to the robot euslisp model, robot VRML(or Collada) file path. <br>
   Arguments (key word) <br>
      object-models : list of object euslisp model <br>
      object-models-file-path) ;; list of object VRML(or Collada) file path <br>
      debug : print openhrp3 <br>
   Arguments (key word + openhrp-project-generator's argument) <br>
      Please see https://github.com/fkanehiro/openhrp3/blob/master/server/ModelLoader/README.md <br>
      timestep, dt : [s] <br>
      output-fname : output file name '[output-fname].xml'. By default, '[robot'sname].xml' <br>
      integrate, use-highgain-mode, method : See above README <br>
   <br>


### gen-projectgenerator-joint-properties-string ###
&nbsp;&nbsp;&nbsp;*robot* 

- Generate joint properties setting for openhrp-project-generator. <br>
   Object (cascaded-link) is required as an argument. <br>


### gen-projectgenerator-model-root-coords-string ###
&nbsp;&nbsp;&nbsp;*obj* 

- Generate root-coords offset setting for openhrp-project-generator. <br>
   Object (cascaded-link) is required as an argument. <br>


def-set-get-param-method *param-class* *set-param-method-name* *get-param-method-name* *getarg-method-name* *set-param-idl-name* *get-param-idl-name* *&key* *(optional-args)* *(debug nil)* 

calculate-toe-heel-offsets *robot* *&key* *((:print-mode pm) :euslisp)* *(pos-or-zmp :pos)* 

