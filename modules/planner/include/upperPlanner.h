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

#ifndef __UPPERPLANNER_H__
#define __UPPERPLANNER_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/all.h>

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

// Created message to communicate with ROS
#include "SharedData_new.h"
#include "object.h"

#include <stdarg.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>

#include "rrts.hpp"
#include "system_single_integrator.h"
#include "singlePlanner.h"
//#include "upperPlanner_IDL.h"

#include "motionPlan.h"

using namespace std;
using namespace RRTstar;
using namespace SingleIntegrator;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace yarp_msg;


typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

/**
 * @brief The trajOutPort class to send the trajectory out
 */
class trajOutPort : public BufferedPort<Bottle>
{
protected:
    string controlPoint;
    vector<Vector> trajectory;
    Stamp ts;
    int nDim;

public:
    void setControlPoint(const string &ctrPoint){
        controlPoint = ctrPoint;
    }

    void setTrajectory(const vector<Vector> &traject)
    {
        trajectory = traject;
        if (trajectory.size()>0)
            nDim = trajectory[0].size();
    }

    void sendTrajectory()
    {
        Bottle &outTraj = prepare();
        outTraj.clear();
        Bottle traj, viaPoint;
        traj.clear();
        viaPoint.clear();

        ts.update();

        traj.addString(controlPoint);   // Added on 21/03/2016 for sending trajectory of different control points

        traj.addInt(trajectory.size());
        traj.addInt(nDim);
        for (int j=0; j<trajectory.size(); j++)
        {
            viaPoint.clear();
            for (int i=0; i<nDim; i++)
                viaPoint.addDouble(trajectory[j][i]);
            traj.append(viaPoint);
        }
        outTraj.addList().read(traj);
        setEnvelope(ts);
        write();
    }
};

class rpcDataProcessor : public PortReader
{
    virtual bool read(ConnectionReader& connection)
    {
        Bottle in, out;
        bool ok = in.read(connection);
        if (!ok) return false;
        // process data "in", prepare "out"
        if (in.get(0).asString() == "re_plan")
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

/**
 * @brief The upperPlanner class to generate motion planner for multiple controlled points, e.g the Forearm with End-Effector, half Elbow and Elbow
 */
class upperPlanner: public yarp::os::RFModule//, public upperPlanner_IDL
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int     verbosity;
    // Name of the module (to change port names accordingly):
    string  robot;          // Name of the robot
    string  name;           // Name of the module
    string  part;           // Part to use
    string  part_short;
    string  running_mode;   // To set running mode as "Single" or "Batch"
    string  targetName;     // Name of the object considered as target
    int     targetID;       // ID of the object considered as target in OPC (at /memory/rpc)

    RpcServer   rpcSrvr;
    bool        replan;         // Flag to run the planner
    RpcClient   rpc2OPC;        // rpc client to connect to port /OPC/rpc
    RpcClient   rpc2ARE;        // rpc client to connect to port /actionsRenderingEngine/get:io
    RpcClient   rpc2calib;      // rpc client to connect to port /iolReachingCalibration/rpc

    // Variables for Batch operation
    unsigned int countReplan;
    unsigned int maxReplan;
    bool    success;            // Flag to indicate the plan is sucessful or not (Batch Summary only)
    double  solvingTime;        // Total time for each whole planner of all control points
    double  planningTime;       // Deadline for local planner
    double  planningTimeGlob;   // Global deadline for the whole planner
    bool    interruptFlag;      // Flag to interrup planner if the time is over the planningTimeGlob
    double  costEE;
    double  costElbow;
    bool    fixEnv;             // Flag to set environment fixed for cost-vs-time benchmark
    bool    generateObstacles;  // Flag to generate obstacle randomly

    // Flag to know if the torso shall be used or not
    bool    useTorso;

    bool    disableTorso;  // flag to know if the torso has to be used or not
    bool    visualizeObjectsInSim; // using yarp rpc /icubSim/world to visualize Objects, i.e. obstacles, target
    bool    visualizeObjectsInGui;
    /***************************************************************************/
    // INTERNAL VARIABLES:

    // Variable for planner
    int nDim;
    Vector workspace;       // World frame
    Vector workspaceRoot;   // Root frame

    // Best trajectory for End Effector
    vector<Vector> bestTrajEE;
    vector<Vector> bestTrajRootEE;

    // Best trajectory for Elbow
    vector<Vector> bestTrajElbow;
    vector<Vector> bestTrajRootElbow;
    bool planForElbow;

    // Best trajectory for Half of Elbow
    vector<Vector> bestTrajHalfElbow;
    vector<Vector> bestTrajRootHalfElbow;

    // Set of Obstacles
    vector<Vector> obsSet;
    vector<Vector> obsSetRoot;
    vector<Vector> obsSetExpandedElbow; // Expanded obstacle set of the Elbow,
                                        // generated from obstacle set of End-Effector
                                        // Obstacle set of the Elbow is combination of
                                        // obsSet and obsSetExpandedElbow
    vector<Vector> obsSetExpandedElbow_fromHalf;

    vector<Vector> obsSetExpandedHalfElbow;

    // Goal
    Vector target;
    Vector rpcCmdPos;   // Position received from rpc command <-- planPos
//    Vector startPos;

    // Received Command
    string rpcCmd;      // String store rpcCmd

    // Manipulator
    iCubArm *arm;
    Vector x0;   // End-Effector initial position before planning
                 // will be read through the Dynamic chain

    // Links' length
    double lShoulder, lArm, lForearm;
    unsigned int indexElbow;


    // Ports for exchange data
    Stamp ts;   // Stamp for the setEnvelope for the ports
    BufferedPort<Bottle> upperTrajectPortOut;   // Output for Trajectory of upper body

    trajOutPort EEPortOut, HalfElbowPortOut;

    motionPlan planPortOut;

    Bottle cmd;
    Bottle cmdGui;          // Bottle to display object on iCubGui
    Port portToSimWorld;
    Port portToGui;
    Matrix T_world_root;    //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR
    Matrix T_root_world;    //homogenous transf. matrix expressing the rotation and translation of FoR from robot (Root) to from world (simulator) FoR

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

    /***************************************************************************/
    // ROS COMMUNICATION VARIABLES:
    bool    useROS;             // Flag to communicate with ROS
    string  nodeName;           // Name of YARP node in ROS network
    string  sharedTopicName;    // Name of topic of acknowledment about the status of ROS planner
public:
    yarp::os::Publisher<SharedData_new> port;   // changed Port to Publisher
    yarp::os::Publisher<object> port_object;    // changed Port to Publisher
    yarp::os::Subscriber<SharedData_new> sub_signalFromROS;

public:
    // CONSTRUCTOR: rate, name, verbosity,
    //upperPlanner(int , const string & , int );
    // CONSTRUCTOR: rate, name, verbosity, regionOperating (6), regionGoal (6)
    //upperPlanner(int , const string & , int , const Vector&, const Vector&);

    // CONSTRUCTOR:
    /**
     * @brief Constructor
     */
    upperPlanner();
    /************************************************************************/
    // Inherit methods
    virtual bool configure(ResourceFinder&);

    virtual bool close();

//    virtual bool attach (RpcServer&);

    virtual double getPeriod();

    virtual bool updateModule();

    virtual bool respond(const Bottle&, Bottle&);

    /************************************************************************/
    // Planner methods
//    bool re_plan(const double& );

//    bool restartPlanner(void);

    double computeMotionDistance(vector<Vector>& traj);

    /**
     * @brief To send an object to ROS
     * @param typeObj: String value to describe the type of object, target or obstacle
     * @param obj: YARP Vector of 3D object, including 3D position and 3D dimension
     */
    void sendObj2ROS(string typeObj, Vector obj);

    /**
    * @brief Initialize the log process in batch running_mode
    */
    void initBatchSummary();

    /**
     * @brief Log processing time in batch running_mode
     */
    void logBatchSummary();

    /**
    * @brief Add another waypoint between two waypoint 1 and 2, to form two control points' path with the same numbers of waypoints
    * @param point1: 3D yarp Vector of waypoint 1
    * @param point2: 3D yarp Vector of waypoint 2
    * @param pointj: 3D yarp Vector of waypoint j, in the path of another control point
    * @param lengthLimb: geometric distance of 2 control points
    * @param newPoint: 3D yarp Vector of the output padded waypoint
    * @return Output is a boolean value to indicate if it is successful in padding a waypoint (true) or not (false)
    */
    bool padWaypoint(const Vector &point1, const Vector &point2,
                     const Vector &pointj, const double &lengthLimb,
                     Vector &newPoint);

    /**
     * @brief Update the information of Kinematic  chain to make sure the joints' value is right
     */
    void updateArmChain();

    /**
     * @brief processRpcCommand
     */
    void processRpcCommand();

    /**
    * @brief Get information (coordinate and dimension) of partner's hands from /OPC/rpc port of WYSIWYD application
    * @param handRoot is the 6 parameter yarp Vector of hand's 3D coordinate and 3D dimension
    * @return Output is a boolean value indicating if a hand is available (true) or not (false)
    */
    bool getHandFromOPC(vector<Vector> &handsRoot);

    /**
    * @brief Get information (coordinate and dimension) of 3D object from /OPC/rpc port of WYSIWYD application
    * @param objectName: string value of name of the object wanted to obtain information, i.e targetName (Octopus, box, etc.)
    * @param idObject: int value output of the id of the object wanted to obtain information, which is stored in /OPC
    * @param objectRoot: is the 6 parameter yarp Vector of object 3D coordinate and 3D dimension
    * @return Output is a boolean value indicating if the object is available (true) or not (false)
    */
    bool getObjFromOPC_Name(const string &objectName, int &idObject, Vector &objectRoot);

    /**
    * @brief Get list of id of obstacles (excluding target) from /OPC/rpc port of WYSIWYD
    * Output is a standard integer vector of id of all obtacles
    */
    vector<int> getObsIDFromOPC_Name();

    /**
    * @brief Get information (coordinate and dimension) of 3D obstacle from /OPC/rpc port of WYSIWYD application
    * @param idObject: integer value of the id of the obstacle wanted to obtain information, which is stored in /OPC and has been obtained by @see getObsIDFromOPC_Name()
    * @param obstacle: is the 6 parameter yarp Vector of object 3D coordinate and 3D dimension
    * @return Output is a boolean value indicating if the obstacle is available (true) or not (false)
    */
    bool getObsFromOPC(const int &idObject, Vector &obstacle);


    /**
     * @brief Get table height from /actionsRenderingEngine/get:io port of **`actionsRenderingEngine`** module
     * @param tableHeight: is a double value of table height in the Robot frame
     * @return Output is a boolean value indicating if the table is available (true) or not (false). If not, it means that you forgot to calibrate the table. Please refer documentation for more information.
     */
    bool getTableHeightFromOPC(double &tableHeight);

    /**
     * @brief Get calibrated position of object from iolReachingCalibration
     * @param object: Vector of 3D position of object
     */
    void getCalibObj(Vector &object);

    /**
    * @brief Expanding the size of all obstacles closed to any waypoints of a trajectory to prevent waypoints of the trajectory of the next controlled point to be assigned in "bad" position
    * @param traject: a standard vector of 3D yarp Vectors of waypoints
    * @param obstacles: std vector of 6 parameter yarp Vector, the first 3 are 3D coordinate of the center of obstacle, the last 3 are sizes of obstacle
    * @param lengthLimb: a double value for the length of link connected two controlled point
    * @return Output is the standard vector of 6 parameter yarp Vector of new obstacles' list
    */
    vector<Vector> expandObstacle(const vector<Vector> &traject, const vector<Vector> &obstacles,
                                  const double &lengthLimb);

    /**
    * @brief Find the other ending point in a link, like Elbow in the Forearm when knowing position of End-Effector and another control point
    * @param oneEndPoint: 3D yarp Vector of 3D coordinate of one control point (e.g. End-effector)
    * @param halfPoint: 3D yarp Vector of 3D coordinate of another control point (e.g. middle point in the Forearm)
    * @param lengthLimb: geometric distance of 2 control points (e.g. length of Forearm)
    * @return Output is the D yarp Vector of the other Ending point of link (e.g. Elbow)
    */
    Vector findOtherEndPoint(const Vector &oneEndPoint, const Vector &halfPoint,
                             const double &lengthLimb);

    /**
    * @brief Find the distance between a waypoint and an obstacle
    * @param waypoint: 3D yarp Vector of 3D coordinate of a waypoint
    * @param obstacles: std vector of 6 parameter yarp Vector, the first 3 are 3D coordinate of the center of obstacle, the last 3 are sizes of obstacle
    * @return Output is a double value of the distance.
    */
    double distWpObs(const Vector &waypoint, const Vector &obstacle);

    /**
    * @brief Find the distance between 2 waypoints
    * @param wp1: 3D yarp Vector of 3D coordinate of a waypoint 1
    * @param wp2: 3D yarp Vector of 3D coordinate of a waypoint 2
    * @return Output is a double value of the distance.
    */
    double distWpWp(const Vector &wp1, const Vector &wp2);

    /**
    * @brief Check if the new waypoint is collided with set of obstacles
    * @param waypoint: 3D yarp Vector of 3D coordinate
    * @param obstacles: std vector of 6 parameter yarp Vector, the first 3 are 3D coordinate of the center of obstacle, the last 3 are sizes of obstacle
    * @return Output is a boolean value indicate the collision (true) and no collision (false)
    */
    bool collisionCheck(const Vector &waypoint, const vector<Vector> &obstacles);

    /**
    * @brief Find the center points of all surfaces of a box obstacle and the norm vectors of all those surfaces
    * @param obstacles: std vector of 6 parameter yarp Vector, the first 3 are 3D coordinate of the center of obstacle, the last 3 are sizes of obstacle
    * @return centers: is a standard vector of 3D coordinate of centers
    * @return normVetors: 2 is a standard vector of 3D parameters normal vector
    */
    void findCenterOnSurfaces(const Vector &obstacle,
                         vector<Vector> &centers, vector<Vector> &normVectors);

    /**
    * @brief Find dot product (or inner product) of 2 vector
    * @param v1: 3D yarp Vector 1
    * @param v2: 3D yarp Vector 2
    * @return Output is a double result of dot product calculation
    */
    double dotProduct(const Vector &v1, const Vector &v2);

    /**
    * @brief Find intersections between a segment of a straight line and a plane
    * @param point1: 3D yarp Vector of 3D coordinate of 1 ending point of the segment
    * @param point2: 3D yarp Vector of 3D coordinate of another ending point of the segment
    * @param pointOnPlane: 3D yarp Vector of 3D coordinate of a point in the plane
    * @param normalVector: 3D yarp Vector of the normal vector of the plane
    * @see findCenterOnSurfaces()
    * @return Output is a double result of dot product calculation
    */
    bool intersectionSegmentPlane(const Vector &point1, const Vector &point2,
                                  const Vector &pointOnPlane, const Vector &normalVector,
                                  Vector &intersection);

    /**
    * @brief Check if a segment of the path is collided with obstacles
    * @param point1: 3D yarp Vector of 3D coordinate of 1 ending point of the segment
    * @param point2: 3D yarp Vector of 3D coordinate of another ending point of the segment
    * @param obstacles: standard vector of list of obstacles, in which each obstacle is a 6 parameter yarp Vector
    * @return Output is a boolean vector indicate if the path segment collides with any obstacle (true) or not (false)
    */
    bool collisionCheckPathSegment(const Vector &point1, const Vector &point2,
                            const vector<Vector> &obstacles);

    /**
    * @brief Display a trajectory on Sim
    * @param trajectory: standard vector of list of waypoint presented a controlled point trajectory, each waypoint is a 3D yarp Vector
    * @param color: string value of color to display the trajectory, i.e. red, green, blue, yellow, magenta, purple.
    */
    void displayTraj(const vector<Vector> &trajectory, const string &color);

    void logTrajectory(const string &trajectFilename, const double &time);

    void sendTrajectory(void);


    void sendTrajectory(const string &ctrPoint,
                        const vector<Vector> &trajectory);

    void sendTrajectory(const string &ctrPoint,
                        BufferedPort<Bottle> trajectPortOut,
                        const vector<Vector> &trajectory);

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

    /**
    * @brief Creates a 3D box (not affected by gravity) in the iCub simulator through the /icubSim/world port
    * @param pos: 6 parameters yarp vector of coordinate of center of the box and 3D dimension respectively
    * @param type: string value of box type, i.e. obstacle or goal
    */
    void createStaticBox(const yarp::sig::Vector &pos, const string &type);

    /**
    * @brief Display all of objects, i.e. obstacles and target in the workspace
    */
    void displayWorkspaceGui();

    void createObsGui(const Vector &pos, const string &type, const int &order);

    void displayWpsGui(const vector<Vector> &trajectory, const string &ctrlPoint, const string &color);

    void createWpGui(const Vector &pos, const string &ctrlPoint, const int &order, const string &color);

    /**
    * @brief Convert an object from Simulator frame to Robot frame
    * @param obj: 6 parameter yarp Vector of an object in Simulator frame, including 3D coordinate of center and 3D dimesion
    * @param outObj: 6 parameter yarp Vector of an object in Robot frame, including 3D coordinate of center and 3D dimesion
    */
    void convertObjFromSimToRootFoR(const Vector &obj, Vector &outObj);

    /**
    * @brief Convert an object from Robot frame to Simulator frame
    * @param obj: 6 parameter yarp Vector of an object in Robot frame, including 3D coordinate of center and 3D dimesion
    * @param outObj: 6 parameter yarp Vector of an object in Simulator frame, including 3D coordinate of center and 3D dimesion
    */
    void convertObjFromRootToSimFoR(const Vector &obj, Vector &outObj);

    /**
    * @brief Initialize the action of display trajectory of a controlled point in Gui
    * @param ctrlPoint: string value of controlled point name
    * @param color: string value of trajectory color
    */
    void initShowTrajGui(const string &ctrlPoint, const string &color);

    /**
    * @brief Display the trajectory of a controlled point in Gui
    * @param trajectory: a standard vector of 3D yarp vectors of waypoints
    * @param ctrlPoint: string value of controlled point name
    */
    void updateTrajGui(const vector<Vector> &trajectory, const string &ctrlPoint);
};

#endif

