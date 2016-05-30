/*
 * Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Nguyen Dong Hai Phuong <phuong.nguyen@iit.it>
 * website: www.robotcub.org
 * author website: https://github.com/towardthesea
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
*/

#ifndef REACHINGSUPERVISOR_H
#define REACHINGSUPERVISOR_H

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/common.h>

#include <stdarg.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <math.h>

#include "motionPlan.h"
#include "particleThread.h"
#include "particleWaypointThread.h"
#include "multipleParticleThread.h"
#include "reachingSupervisor_IDL.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace iCub::iKin;


class reachingSupervisor : public RFModule, public reachingSupervisor_IDL
{
protected:
    // Internal variables
    string                      name;
    int                         nDim;
    deque<waypointTrajectory>   listTrajectories;
    int                         numberWaypoint;
    vector<string>              ctrlPointsNames;

    int         rate;
    int         verbosity;
    double      tol;

    // Port variables
    motionPlan  planPortIn;
    motionPlan  planPortIn1;
    RpcServer   rpcSrvr;
    RpcClient   rpc2Planner;



    // Planning variables
    string      targetName;
    double      localPlanningTime;
    double      globalPlanningTime;

    int                 indexCurSegment;
    bool             finishedCurSegment;
//    particleThread        *tempWaypoint;
//    particleThread        *tempWaypointEE;
//    particleThread        *tempWaypointEB;

    multipleParticleThread          *tempWaypoint;

    particleWaypointThread        *tempWaypointEE;
    particleWaypointThread        *tempWaypointEB;
    double          timeToFinishSegment;    // moving time between 2 sucessive waypoints, should be the same for every controlled points in one link
    double                      speedEE;    // velocity magnitude of End-Effector
    double                      speedEB;    // velocity magnitude of Elbow
    vector<double>            speedLink;    // std vector contains all velocity magnituds of a link, e.g for Forearm, it includes End-effector and Elbow velocity
public:
    reachingSupervisor();

    /************************************************************************/
    // Inherit methods
    virtual bool configure(ResourceFinder&);

    virtual bool close();

    virtual bool attach (RpcServer &source);

    virtual double getPeriod();

    virtual bool updateModule();


    /************************************************************************/
    /**
     * @brief Computes the velocity vector for the motion between 2 waypoints
     * @param speed: double value of controlled point speed
     * @param wp1: yarp Vector of waypoint coordinate of the waypoint 1 of the segment
     * @param wp2: yarp Vector of waypoint coordinate of the waypoint 2 of the segment
     * @return: yarp Vector of the velocity of controlled point
     */
    Vector computeVelFromSegment(const double& speed, const Vector& wp1, const Vector& wp2);

    /**
     * @brief Computes the speed of other controlled point known speed of one controlled point
     * @param speed: double value of controlled point speed
     * @param wp1: yarp Vector of waypoint coordinate of the waypoint 1 of this segment
     * @param wp2: yarp Vector of waypoint coordinate of the waypoint 2 of this segment
     * @param wp1Other: yarp Vector of waypoint coordinate of the waypoint 1 of other segment
     * @param wp2Other: yarp Vector of waypoint coordinate of the waypoint 2 of other segment
     * @return: double value of the speed of other controlled point
     */
    double computeOtherCtrlPtSpeed(const double& speed, const Vector &wp1, const Vector &wp2,
                                                        const Vector &wp1Other, const Vector &wp2Other);

    /**
    * @brief Find the distance between 2 waypoints
    * @param wp1: 3D yarp Vector of 3D coordinate of a waypoint 1
    * @param wp2: 3D yarp Vector of 3D coordinate of a waypoint 2
    * @return: a double value of the distance.
    */
    double distWpWp(const Vector &wp1, const Vector &wp2);

    /**
     * @brief Sets the tolerance of the internal particles generation
     * @param _tol: double value of the tolerance
     * @return true/false on success/failure.
     */
    bool setTol(const double &_tol);

    /**
     * @brief Gets the tolerance of the internal particles generation
     * @return double value of the tolerance
     */
    double getTol();

    /**
     * @brief Sets the speed of the End-Effector motion
     * @param _speedEE: double value of the End-Effector speed
     * @return true/false on success/failure.
     * @see speedEE
     */
    bool setSpeedEE(const double &_speedEE);

    /**
     * @brief Gets the speed of the End-Effector motion
     * @return double value of the End-Effector speed
     * @see speedEE
     */
    double getSpeedEE();

    /**
     * @brief Sets the verbosity value
     * @param _verbosity: integer value of the verbosity
     * @return true/false on success/failure.
     */
    bool setVerbosity(const int &_verbosity);

    /**
     * @brief Gets the verbosity value
     * @return Integer value of the verbosity
     */
    int getVerbosity();

    bool setDeadline(const double &_deadline);

    double getDeadline();

    bool setGlobDeadline(const double &_globDeadline);

    double getGlobDeadline();

    bool setTarget(const string &_target);

    string getTarget();

    bool sendCmd2Planner();

    /************************************************************************/
    // Thrift methods
    bool set_tol(const double _tol)
    {
        return setTol(_tol);
    }

    double get_tol()
    {
        return getTol();
    }

    bool set_speedEE(const double _speedEE)
    {
        return setSpeedEE(_speedEE);
    }

    double get_speedEE()
    {
        return getSpeedEE();
    }

    bool set_verbosity(const int32_t _verbosity)
    {
        return setVerbosity(_verbosity);
    }

    int get_verbosity()
    {
        return getVerbosity();
    }

    bool set_deadline(const double _deadline)
    {
        return setDeadline(_deadline);
    }

    double get_deadline()
    {
        return getDeadline();
    }

    bool set_glob_deadline(const double _globDeadline)
    {
        return setGlobDeadline(_globDeadline);
    }

    double get_glob_deadline()
    {
        return getGlobDeadline();
    }

    bool set_target(const string &_target)
    {
        return setTarget(_target);
    }

    string get_target()
    {
        return getTarget();
    }

    bool run_planner(const double _deadline)
    {
        setDeadline(_deadline);
        return sendCmd2Planner();
    }
};

#endif // REACHINGSUPERVISOR_H
