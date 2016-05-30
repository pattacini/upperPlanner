# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: NGUYEN Dong Hai Phuong
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# reachingSupevisor.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
* reachingSupervisor_IDL
*
* IDL Interface to \ref reachingSupervisor services.
*/
service reachingSupervisor_IDL
{
  /**
  * Sets tolerance.
  * @param _tol the solver exits if norm(x_d-x)<tol.
  * @return true/false on success/failure.
  */
  bool set_tol(1:double _tol);

  /**
  * Gets the tolerance.
  * @return the current tolerance value.
  */
  double get_tol();

  /**
  * Sets End-Effector Speed.
  * @param _speedEE  the speed of the End-Effector
  * @return true/false on success/failure.
  */
  bool set_speedEE(1:double _speedEE);

  /**
  * Gets the speed of End-Effector movement
  * @return the current setting speed of End-Effector
  */
  double get_speedEE();

  /**
  * Sets verbosity.
  * @param _verbosity: the verbosity of the controller
  * @return true/false on success/failure.
  */
  bool set_verbosity(1:i32 _verbosity);  

  /**
  * Gets verbosity.
  * @return the verbosity of the controller
  */
  i32 get_verbosity();  

  /**
  * Sets planning time for each local planner
  * @param _deadline: planning time 
  * @return true/false on success/failure.
  */
  bool set_deadline(1:double _deadline);

  /**
  * Gets the planning time of each local planner
  * @return the current setting planning time of each local planner
  */
  double get_deadline();

  /**
  * Sets global planning time for the planner
  * @param _globDeadline: global planning time 
  * @return true/false on success/failure.
  */
  bool set_glob_deadline(1:double _globDeadline);

  /**
  * Gets the global planning time of the planner
  * @return the current setting global planning time of the planner
  */
  double get_glob_deadline();

  /**
  * Sets target for the planner
  * @param _target: string value target
  * @return true/false on success/failure.
  */
  bool set_target(1:string _target);

  /**
  * Gets the setting target of the planner
  * @return the current setting target of the planner
  */
  string get_target();

  /**
  * Send planning request to reaching-planner
  * @param _deadline  planning time for each local planner
  * @return true/false on success/failure.
  */
  bool run_planner(1:double _deadline);
   
  /**
  * Disables the controller and switches to position mode
  * @return true/false on success/failure.
  **/
  bool stop();


}
