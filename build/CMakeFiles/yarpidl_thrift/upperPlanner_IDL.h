// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_upperPlanner_IDL
#define YARP_THRIFT_GENERATOR_upperPlanner_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class upperPlanner_IDL;


/**
 * upperPlanner_IDL
 * IDL Interface to \ref upperPlanner services.
 */
class upperPlanner_IDL : public yarp::os::Wire {
public:
  upperPlanner_IDL();
  /**
   * Require replan with new time constraint
   * @param _deadline Time constraint
   *            (put it between brackets if asking for it through rpc).
   * @return true/false on success/failure.
   */
  virtual bool re_plan(const double _deadline);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
