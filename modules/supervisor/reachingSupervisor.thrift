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

  double get_speedEE();

  /**
  * Sets verbosity.
  * @param _verbosity  the verbosity of the controller
  * @return true/false on success/failure.
  */
  bool set_verbosity(1:i32 _verbosity);  

  /**
  * Gets verbosity.
  * @return the verbosity of the controller
  */
  i32 get_verbosity();  
   
  /**
  * Disables the controller and switches to position mode
  * @return true/false on success/failure.
  **/
  bool stop();


}
