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

#ifndef __UPPERPLANNER_H__
#define __UPPERPLANNER_H__

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

//#include "object.h"
#include "rrts.hpp"
#include "system_single_integrator.h"
#include "singlePlanner.h"
#include "upperPlanner_IDL.h"

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

class rpcDataProcessor : public PortReader
{
    virtual bool read(ConnectionReader& connection)
    {
        Bottle in, out;
        bool ok = in.read(connection);
        if (!ok) return false;
        // process data "in", prepare "out"
        if (in.get(0).asString() == "replan")
        {
            printf("receive replan cmd");
        }

        ConnectionWriter *returnToSender = connection.getWriter();
        if (returnToSender!=NULL)
        {
            out.write(*returnToSender);
        }
        return true;
    }
};

class upperPlanner: public yarp::os::RFModule//, public upperPlanner_IDL
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string robot;       // Name of the robot
    string  name;       // Name of the module
    string  part;       // Part to use
    string  part_short;

    RpcServer rpcSrvr;
    bool replan;        // Flag to run the planner
    double planningTime;// Deadline for planner

    // Flag to know if the torso shall be used or not
    bool useTorso;

    bool disableTorso;  // flag to know if the torso has to be used or not
    bool visualizeObjectsInSim; // using yarp rpc /icubSim/world to visualize Objects, i.e. obstacles, target
    /***************************************************************************/
    // INTERNAL VARIABLES:
//    // Variable for planner
//    planner_t rrts;

//    // Dynamic system
//    System system;
    int nDim;

//    // Number of iteration for planning
//    int nIter;

//    // Deadline
//    double deadline;

//    // Time to solve planning problem
//    double solveTime;
//    //Time solveTime1;

    // Best trajectory for End Effector
    vector<Vector> bestTrajEE;
    vector<Vector> bestTrajRootEE;

    // Set of Obstacles
    vector<Vector> obsSet;

    // Goal
    Vector target;
//    Vector startPos;

    // Manipulator
    iCubArm *arm;
    //iKinChain &chain;
    Vector x0;   // End-Effector initial position

    // Ports for exchange data
    Stamp ts;   // Stamp for the setEnvelope for the ports

    BufferedPort<Bottle> upperTrajectPortOut;   // Output for Trajectory of upper body


//    Bottle cmd;
//    Port portToSimWorld;
    Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR
    Matrix T_root_world; //homogenous transf. matrix expressing the rotation and translation of FoR from robot (Root) to from world (simulator) FoR

    // Driver for "classical" interfaces
    PolyDriver       ddA;
    PolyDriver       ddT;

    // "Classical" interfaces for the arm
    IEncoders            *iencsA;
    IVelocityControl2     *ivelA;
    IControlMode2         *imodA;
    IControlLimits        *ilimA;
    yarp::sig::Vector     *encsA;

    int jntsA;

    // "Classical" interfaces for the torso
    IEncoders            *iencsT;
    IVelocityControl2     *ivelT;
    IControlMode2         *imodT;
    IControlLimits        *ilimT;
    yarp::sig::Vector     *encsT;
    int jntsT;

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR: rate, name, verbosity,
    //upperPlanner(int , const string & , int );
    // CONSTRUCTOR: rate, name, verbosity, regionOperating (6), regionGoal (6)
    //upperPlanner(int , const string & , int , const Vector&, const Vector&);

    // CONSTRUCTOR:
    upperPlanner();
    /************************************************************************/
    // Inherit methods
    virtual bool configure(ResourceFinder&);

    virtual bool close();

    virtual bool attach (RpcServer&);

    virtual double getPeriod();

    virtual bool updateModule();

    virtual bool respond(const Bottle&, Bottle&);

    /************************************************************************/
    // Planner methods
//    bool re_plan(const double& );

//    bool restartPlanner(void);

    void updateArmChain();

    void processRpcCommand();

//    void setIteration(const int&);

//    void setDeadline(const double&);

//    void setRegionOperating(const Vector&);

//    void setGoal(const Vector&);

//    void setStart(const Vector&);

//    // Obstacle is a Vector of six with 3 for coordinates and 3 for size
//    void setObstacles(const std::vector<Vector>&);

//    void updatePlanner(void);

//    vector<Vector> generateTrajectory(void);

//    vector<Vector> getBestTrajRoot(void);

//    void executeTrajectory(void);

//    void printTrajectory(void);

//    void logTrajectory();

//    void displayPlan(void);

    void sendTrajectory(void);

    void convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos);

    void convertPosFromSimToRootFoR(const Vector &pos, Vector &outPos);

//    //*** visualizations in icub simulator
//    /**
//    * Creates a sphere (not affected by gravity) in the iCub simulator through the /icubSim/world port
//    * @param radius
//    * @param pos
//    */
//    void createStaticSphere(double radius, const yarp::sig::Vector &pos);

//    void moveSphere(int index, const yarp::sig::Vector &pos);

//    void createStaticBox(const yarp::sig::Vector &pos, const string &type);

//    void moveBox(int index, const yarp::sig::Vector &pos);
};

#endif

