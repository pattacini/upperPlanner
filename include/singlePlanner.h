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
    string running_mode;// Name for running_mode

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
    /**
     * @brief constructor
     * @param _verbosity
     * @param _name: string value of the whole planner name
     * @param _robot: string value of robot name, i.e icubSim or icub
     * @param _running_mode: string value of running_mode, i.e single or batch
     * @param _controlPoint: string value of the controlled point name, e.g End-Effector, Elbow
     */
    singlePlanner(const int&, const string&, const string&, const string&, const string&);

    void init();

    /**
     * @brief Set number of interation to generate the planning
     * @param nIteration: integer value of the maximum interations
     */
    void setIteration(const int&);

    /**
     * @brief Set deadline for the computation of a planner
     * @param _deadline: double value of the maximum computation time
     */
    void setDeadline(const double&);

    /**
     * @brief Set the working space
     * @param regionOperation: a 6 parameter yarp Vector of the working space, the first 3 are the coordinate of the center, the last 3 are the size
     */
    void setRegionOperating(const Vector&);

    /**
     * @brief Set the final state of the planner
     * @param goal: a 6 parameter yarp Vector of the goal, the first 3 are the coordinate of the center, the last 3 are the size
     */
    void setGoal(const Vector&);

    /**
     * @brief Set the beginning state of the planner
     * @param start: a 3D yarp Vector of the coordinate the state
     */
    void setStart(const Vector&);

    // Obstacle is a Vector of six with 3 for coordinates and 3 for size
    /**
     * @brief Set the list of obstacles in the environment
     * @param obstacles: a standard vector of 6 parameter yarp Vector of obstacles, the first 3 are the coordinate of the center, the last 3 are the size
     */
    void setObstacles(const std::vector<Vector>&);

    /**
     * @brief Initialize the RRT tree and set the gamma value
     */
    void updatePlanner(void);

    /**
     * @brief Run repeatedly random sampling to generate the tree according to the maximum iteration and the deadline
     * @return A standard vector of 3D yarp vector of coordinate of waypoints of trajectory in Simulator frame
     */
    vector<Vector> generateTrajectory(void);

    /**
     * @brief Get the best trajectory in Robot frame
     * @return A standard vector of 3D yarp vector of coordinate of waypoints of trajectory in Robot frame
     */
    vector<Vector> getBestTrajRoot(void);

    /**
     * @brief Run the procedure to get the trajectory
     * @param _bestTraj: A standard vector of 3D yarp vector of coordinate of waypoints of trajectory in Simulator frame
     * @param _bestTrajRoot: A standard vector of 3D yarp vector of coordinate of waypoints of trajectory in Robot frame
     * @param color: string value of the trajectory color when displayed in Simulator
     */
    void executeTrajectory(vector<Vector> &_bestTraj, vector<Vector> &_bestTrajRoot, const string &color);

    /**
     * @brief Print out in the terminal the results of planning
     */
    void printTrajectory(void);

    /**
     * @brief Log the vertices of the RRT tree into files
     */
    void logVertices(void);

    /**
     * @brief Log the results of planning into files
     */
    void logTrajectory();

    /**
     * @brief Display all objects, i.e obstacles and target, and trajectory in Simulator
     * @param color: String value of color fo the trajectory.
     */
    void displayPlan(const string &color);

//    void sendTrajectory(void);

    /**
    * @brief Convert an 3D coordinate from Robot frame to Simulator frame
    * @param obj: 3D yarp Vector of an coordinate in Robot frame
    * @param outObj: 3D yarp Vector of an coordinate in Simulator frame
    */
    void convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos);

    /**
    * @brief Convert an 3D coordinate from Simulator frame to Robot frame
    * @param obj: 3D yarp Vector of an coordinate in Simulator frame
    * @param outObj: 3D yarp Vector of an coordinate in Robot frame
    */
    void convertPosFromSimToRootFoR(const Vector &pos, Vector &outPos);

    //*** visualizations in icub simulator
    /**
    * @brief Creates a sphere (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param radius: double value of radius of the sphere
    * @param pos: 3D yarp vector of coordinate of sphere center
    * @param color: string value of color of the sphere
    */
    void createStaticSphere(double radius, const yarp::sig::Vector &pos, const string &color);

    void moveSphere(int index, const yarp::sig::Vector &pos);

    /**
    * @brief Creates a 3D box (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param pos: 6 parameters yarp vector of coordinate of center of the box and 3D dimension respectively
    * @param type: string value of box type, i.e. obstacle or goal
    */
    void createStaticBox(const yarp::sig::Vector &pos, const string &type);

    void moveBox(int index, const yarp::sig::Vector &pos);
};

#endif // SINGLEPLANNER_H
