#ifndef MOTIONPLAN_H
#define MOTIONPLAN_H

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <vector>
#include <deque>
#include <string>
#include <fstream>
#include <stdarg.h>
#include <math.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

//namespace iCub {
//   namespace motionPlan {
//       class motionPlan;
//       class waypointTrajectory;
//    }
//}

/**
 * @brief The waypointTrajectory class contains some useful information of a generated motion path to be easily sent through different modules
 */
class waypointTrajectory
{
private:
    string  controlPointName;
    int     numberDimension;
    int     numberWaypoints;
    vector<Vector> waypoints;   // Delete it later
public:

    /**
     * @brief Empty constructor of waypointTrajectory
     */
    waypointTrajectory()
    {
        numberDimension = 3;
        numberWaypoints = 0;
        waypoints.clear();
    }

    /**
     * @brief Constructor of waypointTrajectory
     * @param ctrlPtName: string value for the name of the controlled point
     * @param trajectory: standard vector of 3D yarp vector of motion path
     */
    waypointTrajectory(const string &ctrlPtName,
                       const vector<Vector> &trajectory)
    {
        controlPointName = ctrlPtName;
        waypoints = trajectory;
        numberWaypoints = trajectory.size();
        if (trajectory.size()>0)
            numberDimension = waypoints[0].size();
    }

    /**
     * @brief Sets name for the controlled point
     * @param ctrlPtName: string value for the name of the controlled point
     */
    void setCtrlPointName(const string& ctrlPtName)
    {
        controlPointName = ctrlPtName;
    }

    /**
     * @brief Sets trajectory for the waypointTrajectory
     * @param trajectory: standard vector of 3D yarp vector of motion path
     */
    void setWaypoints(const vector<Vector>& trajectory)
    {
        waypoints = trajectory;
        numberWaypoints = trajectory.size();
        if (trajectory.size()>0)
            numberDimension = waypoints[0].size();
    }

    /**
     * @brief Gets the name of the controlled point
     * @return a string value of the name
     */
    string getCtrlPointName()
    {
        return controlPointName;
    }

    /**
     * @brief Gets the dimension of the waypoints
     * @return an integer value of the dimension
     */
    int getDimension()
    {
        return numberDimension;
    }

    /**
     * @brief Gets number of waypoints in the motion path
     * @return an integer value of the number of waypoints
     */
    int getNbWaypoint()
    {
        return numberWaypoints;
    }

    /**
     * @brief Gets the sets of waypoints of the motion path
     * @return a standard vector of 3D yarp Vector, containing the 3D coordinate of waypoints
     */
    vector<Vector> getWaypoints()
    {
        return waypoints;
    }

    /**
     * @brief Find the distance between 2 waypoints
     * @param wp1: 3D yarp Vector of 3D coordinate of a waypoint 1
     * @param wp2: 3D yarp Vector of 3D coordinate of a waypoint 2
     * @return: a double value of the distance.
     */
    double distWpWp(const Vector &wp1, const Vector &wp2)
    {
        double distance = 1000;
        if ((wp1.size()==numberDimension) && (wp2.size()==numberDimension))
        {
            double sqSum = 0;
            for (int i=0; i<numberDimension; i++)
                sqSum += pow((wp1[i]-wp2[i]),2.0);
            distance = sqrt(sqSum);
        }
        return distance;
    }


    /**
     * @brief Compute the length of the trajectory
     * @return Double value of the length
     */
    double getLengthTraj()
    {
        double cost=0.0;
        for (int i=0; i<waypoints.size()-1; i++)
            cost += distWpWp(waypoints[i],waypoints[i+1]);

        return cost;
    }

};

/**
 * @brief The motionPlan class to create a BufferedPort to receive or send waypointTrajectory
 * @see waypointTrajectory
 */
class motionPlan : public BufferedPort<Bottle>
{
    using BufferedPort<Bottle>::onRead;

    /**
     * @brief Reads waypointTrajectory through port set to plan in non-blocking mode
     * @param inPlan: Bottle value containing the waypointTrajectory
     * @see waypointTrajectory
     */
    virtual void onRead(Bottle &inPlan);

private:
    bool haveNewMsg;
    deque<waypointTrajectory> listTrajectory;  // Delete later
    Stamp ts;

public:
    /**
     * @brief Constructor motionPlan
     */
    motionPlan();

    ~motionPlan();

    /**
     * @brief Adds a waypointTrajectory to the plan
     * @param ctrlPtTrajectory: a waypointTrajectory instance
     * @see waypointTrajectory
     */
    void addTrajectory(const waypointTrajectory &ctrlPtTrajectory);

    /**
     * @brief Clears all waypointTrajectory in the plan
     * @see waypointTrajectory
     */
    void clearTrajectory();

    /**
     * @brief Gets waypointTrajectory in the plan
     * @return A standard deque of set of waypointTrajectory
     * @see waypointTrajectory
     */
    deque<waypointTrajectory>& getListTrajectory(); //Change to reference

    /**
     * @brief Sets waypointTrajectory for the plan
     * @param _listTraject: a standard deque of a set of waypointTrajectory
     */
    void setListTrajectory(const deque<waypointTrajectory>& _listTraject);
    /**
     * @brief Sends waypointTrajectory through the port set to plan
     * @see waypointTrajectory
     */
    void sendPlan();

    /**
     * @brief Receives waypointTrajectory through the port  set to plan in blocking mode
     * @see waypointTrajectory
     */
    void receivePlan();

    /**
     * @brief Checks if there is a new processed message in the port
     * @return A boolean value to indicate if there is a new message(true) or not(false)
     */
    bool& gotNewMsg();

    /**
     * @brief Sets value to haveNewMsg variable
     * @param value A boolean value
     * @see haveNewMsg
     */
    void setNewMsg(const bool &value);


};

//class motionPlan : public Portable
//{
//private:
//    vector<waypointTrajectory> listTrajectory;
//public:
//    motionPlan();

//    void addTrajectory(const waypointTrajectory &ctrlPtTrajectory);

//    virtual bool read(ConnectionReader &connection);

//    virtual bool write(ConnectionWriter &connection);
//};

#endif // MOTIONPLAN_H
