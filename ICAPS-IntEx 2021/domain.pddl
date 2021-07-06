(define (domain offshore_energy)
(:requirements :strips :typing :fluents :negative-preconditions :disjunctive-preconditions :durative-actions :duration-inequalities :universal-preconditions )
(:types
  robot
  poi
  robot_sensor
  actuator
)

(:predicates (at ?r - robot ?wp - poi)
             (valve_at ?wp - poi)
             (recharge_point ?r - robot ?wp - poi)

             (robot_can_act ?r - robot ?wp - poi)
             (available ?r - robot)
             (related ?wpv ?wps - poi)
             (can_inspect_valve ?r - robot)
             (can_supervise_process ?r - robot)

             (equipped_for_temperature_analysis ?r - robot ?s - robot_sensor)
             (equipped_for_pression_analysis ?r - robot ?s - robot_sensor)
             (equipped_for_camera_imaging ?r - robot ?s - robot_sensor)
             (equipped_with_actuator ?r - robot ?a - actuator)


             (pressure_sensed  ?wp - poi)
             (temperature_sensed  ?wp - poi)
             (image_captured  ?wp - poi)
             (valve_inspected  ?wp - poi)
             (valve_turned  ?wp - poi)
             (explored ?wp - poi)
             (valve_manip_recorded ?wp - poi)


)
(:functions (energy ?r - robot)
            (consumption ?r - robot)
            (speed ?r - robot)
            (recharge_rate ?r - robot)
            (data_acquired ?r - robot)
            (data_capacity ?r - robot)

            (distance_intime ?wpi ?wpf - poi)
            (comm_data_dur ?r - robot)
            (take_image_dur ?r - robot)
            (check_pressure_dur ?r - robot)
            (check_temperature_dur ?r - robot)
            (valve_inspection_dur ?r - robot)
            (manipulate_valve_dur ?r - robot)



)
; domain action for terrestrial and aerial vehicles
(:durative-action navigation
:parameters (?r - robot ?wpi  ?wpf - poi)
:duration ( = ?duration (distance_intime ?wpi ?wpf))
:condition (and
           (over all (robot_can_act ?r  ?wpf))
           (at start (available ?r))
           (at start (at ?r ?wpi))
           ;(at start (>= (energy ?r) (* (/ (distance_intime ?wpi ?wpf) (speed ?r)) (consumption ?r))))
           )
:effect (and
        ;(at start (decrease (energy ?r) (* (/ (distance_intime ?wpi ?wpf) (speed ?r)) (consumption ?r))))
        (at start (not (available ?r)))
        (at start (not (at ?r ?wpi)))
        (at end (at ?r ?wpf))
        (at end (explored ?wpf))
        (at end (available ?r))
        )
)
; domain action for terrestrial and aerial vehicles
(:durative-action comm_data
:parameters (?r - robot   ?wp - poi)
:duration (= ?duration (comm_data_dur ?r))
:condition (and
           (over all (robot_can_act ?r  ?wp))
           (over all (at ?r ?wp))
           (at start (available ?r))
           (at start (>= (data_acquired ?r) (data_capacity ?r)))
           (at start (>= (energy ?r) 2))
           )
:effect (and
        (at start (not (available ?r)))
        (at end (available ?r))
        (at end (decrease (energy ?r) 2))
        (at end (assign (data_acquired ?r) 0))
	 )
)
; domain action for terrestrial and aerial vehicles
(:durative-action recharge_battery
:parameters (?r - robot  ?wp - poi)
:duration (= ?duration (/ (- 100 (energy ?r)) (recharge_rate ?r)))
:condition (and
          (over all (robot_can_act ?r  ?wp))
          (over all (at ?r ?wp))
          (over all (recharge_point ?r ?wp))
          (at start (at ?r ?wp))
          (at start (available ?r))
          (at start (<= (energy ?r) 80))
           )
:effect (and
        (at start (not (available ?r)))
        (at end (available ?r))
        (at end (increase (energy ?r) (* ?duration (recharge_rate ?r))))
        )
)

; domain action for  aerial vehicles
(:durative-action take_image
 :parameters (?r - robot ?s - robot_sensor  ?wp - poi)
 :duration (= ?duration (take_image_dur ?r))
 :condition (and
            (over all (robot_can_act ?r  ?wp))
            (over all (equipped_for_camera_imaging ?r ?s))
            (over all (at ?r ?wp))
            (over all (can_supervise_process ?r))
            (at start (at ?r ?wp))
            (at start (available ?r))
            (at start (>= (energy ?r) 1))
            (at start (< (data_acquired ?r) (data_capacity ?r)))
            )
 :effect (and
         (at start (not (available ?r)))
         (at start (decrease (energy ?r) 1))
         (at end (image_captured ?wp))
         (at end (available ?r))
         (at end (increase (data_acquired ?r) 1))
         )
)
; domain action for terrestrial vehicles
(:durative-action check_temperature
:parameters (?r - robot ?s - robot_sensor ?wp - poi)
:duration (= ?duration (check_temperature_dur ?r))
:condition (and
           (over all (robot_can_act ?r  ?wp))
           (over all (at ?r ?wp))
           (over all (equipped_for_temperature_analysis ?r ?s))
           (at start (at ?r ?wp))
           (at start (available ?r))
           (at start (>= (energy ?r) 3))
           (at start (< (data_acquired ?r) (data_capacity ?r)))
           )
:effect (and
        (at start (not (available ?r)))
        (at start (decrease (energy ?r) 3))
        (at end (temperature_sensed ?wp))
        (at end (available ?r))
        (at end (increase (data_acquired ?r) 1))
        )
)
; domain action for terrestrial vehicles
(:durative-action check_pressure
:parameters (?r - robot ?s - robot_sensor  ?wp - poi)
:duration (= ?duration (check_pressure_dur ?r))
:condition (and
           (over all (robot_can_act ?r  ?wp))
           (over all (at ?r ?wp))
           (at start (at ?r ?wp))
           (at start (available ?r))
           (at start (equipped_for_pression_analysis ?r ?s))
           (at start (>= (energy ?r) 3))
           (at start (< (data_acquired ?r) (data_capacity ?r)))
           )
:effect (and
        (at start (not (available ?r)))
        (at start (decrease (energy ?r) 5))
        (at end (pressure_sensed ?wp))
        (at end (available ?r))
        (at end (increase (data_acquired ?r) 1))
        )
)
; domain action for terrestrial vehicles
(:durative-action valve_inspection
 :parameters (?r - robot   ?s - robot_sensor ?wp - poi)
 :duration ( = ?duration (valve_inspection_dur ?r))
 :condition (and
             (over all (robot_can_act ?r  ?wp))
             (over all (at ?r ?wp))
             (over all (can_inspect_valve ?r))
             (over all (equipped_for_camera_imaging ?r ?s))
             (at start (at ?r ?wp))
             (at start (available ?r))
             (at start (>= (energy ?r) 2))
             (at start (< (data_acquired ?r) (data_capacity ?r)))
            )
  :effect (and
          (at start (not (available ?r)))
          (at end (valve_inspected ?wp))
          (at end (decrease (energy ?r) 2))
          (at end (available ?r))
          (at end (increase (data_acquired ?r) 1))
          )
)
; domain action for terrestrial and aerial vehicles (action with collaboration)
(:durative-action manipulate_valve
 :parameters (?rh ?rd - robot  ?s - robot_sensor ?a - actuator ?wpv ?wps - poi)
 :duration ( = ?duration (manipulate_valve_dur ?rh))
 :condition (and
             (over all (robot_can_act ?rd  ?wps))
             (over all (robot_can_act ?rh  ?wpv))
             (over all (at ?rh ?wpv))
             (over all (at ?rd ?wps))
             (over all (valve_at ?wpv))
             (over all (related ?wpv ?wps))
             (over all (can_supervise_process ?rd))
             (over all (equipped_for_camera_imaging ?rd ?s))
             (over all (equipped_with_actuator ?rh ?a))
             (at start (valve_inspected ?wpv))
             (at start (available ?rh))
             (at start (available ?rd))
             (at start (>= (energy ?rh) 2))
             (at start (>= (energy ?rd) 2))
             (at start (< (data_acquired ?rd) (data_capacity ?rd)))
            )
  :effect (and
           (at start (not (available ?rh)))
           (at start (not (available ?rd)))
           (at end (valve_turned ?wpv))
           (at end (valve_manip_recorded ?wps))
           (at end (decrease (energy ?rh) 2))
           (at end (decrease (energy ?rd) 2))
           (at end (available ?rh))
           (at end (available ?rd))
           (at end (increase (data_acquired ?rd) 1))
           )

)
)
