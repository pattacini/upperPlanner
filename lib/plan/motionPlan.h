#ifndef MOTIONPLAN_H
#define MOTIONPLAN_H

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <string>
#include <fstream>
#include <stdarg.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

class waypointTrajectory
{
private:
    string  controlPoint;
    int     numberDimension;
    int     numberWaypoints;
    vector<Vector> waypoints;
public:

    waypointTrajectory()
    {
        numberDimension = 0;
        numberWaypoints = 0;
    }

    waypointTrajectory(const string &ctrlPt,
                       const vector<Vector> &trajectory)
    {
        controlPoint = ctrlPt;
        waypoints = trajectory;
        numberWaypoints = trajectory.size();
        if (trajectory.size()>0)
            numberDimension = waypoints[0].size();
    }

    void setCtrlPoint(const string& ctrlPt)
    {
        controlPoint = ctrlPt;
    }


    void setWaypoints(const vector<Vector>& trajectory)
    {
        waypoints = trajectory;
        numberWaypoints = trajectory.size();
        if (trajectory.size()>0)
            numberDimension = waypoints[0].size();
    }

    string getCtrlPoint()
    {
        return controlPoint;
    }

    int getDimension()
    {
        return numberDimension;
    }

    int getNbWaypoint()
    {
        return numberWaypoints;
    }

    vector<Vector> getWaypoints()
    {
        return waypoints;
    }

};

class motionPlan : public BufferedPort<Bottle>
{
private:
    vector<waypointTrajectory> listTrajectory;
    Stamp ts;
public:
    motionPlan();

    void addTrajectory(const waypointTrajectory &ctrlPtTrajectory);

    void clearTrajectory();

    vector<waypointTrajectory> getListTrajectory();

    void sendPlan();

    void receivePlan();


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
