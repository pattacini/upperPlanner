/*
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef SINGLEPLANNER_H
#define SINGLEPLANNER_H

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/RpcServer.h>
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


#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace std;
using namespace RRTstar;
using namespace SingleIntegrator;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace iCub::iKin;


typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

class singlePlanner
{
protected:
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string robot;       // Name of the robot
    string  name;       // Name of the module
    string  part;       // Name of part to use
    string controlPoint;// Name of Control point

    // Variable for planner
    planner_t rrts;

    // Dynamic system
    System system;
    int nDim;

    // Number of iteration for planning
    int nIter;

    // Deadline
    double deadline;

    // Time to solve planning problem
    double solveTime;
    //Time solveTime1;

    // Best trajectory for End Effector
    vector<Vector> bestTraj;
    vector<Vector> bestTrajRoot;

    // Set of Obstacles
    vector<Vector> obsSet;

    // Goal
    Vector target;
    Vector startPos;

    Bottle cmd;
    Port portToSimWorld;
    Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR
    Matrix T_root_world; //homogenous transf. matrix expressing the rotation and translation of FoR from robot (Root) to from world (simulator) FoR

    int printMessage(const int l, const char *f, ...) const;

public:
    singlePlanner(const int&, const string&, const string&, const string&);

    void init();

    void setIteration(const int&);

    void setDeadline(const double&);

    void setRegionOperating(const Vector&);

    void setGoal(const Vector&);

    void setStart(const Vector&);

    // Obstacle is a Vector of six with 3 for coordinates and 3 for size
    void setObstacles(const std::vector<Vector>&);

    void updatePlanner(void);

    vector<Vector> generateTrajectory(void);

    vector<Vector> getBestTrajRoot(void);

    void executeTrajectory(vector<Vector> &_bestTraj, vector<Vector> &_bestTrajRoot, const string &color);

    void printTrajectory(void);

    void logVertices(void);

    void logTrajectory();

    void displayPlan(const string &color);

//    void sendTrajectory(void);

    void convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos);

    void convertPosFromSimToRootFoR(const Vector &pos, Vector &outPos);

    //*** visualizations in icub simulator
    /**
    * Creates a sphere (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param radius
    * @param pos
    */
    void createStaticSphere(double radius, const yarp::sig::Vector &pos, const string &color);

    void moveSphere(int index, const yarp::sig::Vector &pos);

    void createStaticBox(const yarp::sig::Vector &pos, const string &type);

    void moveBox(int index, const yarp::sig::Vector &pos);
};

#endif // SINGLEPLANNER_H
