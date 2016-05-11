#ifndef MOTIONPLAN_H
#define MOTIONPLAN_H

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <deque>
#include <string>
#include <fstream>
#include <stdarg.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

namespace iCub {
   namespace motionPlan {
       class motionPlan;
       class waypointTrajectory;
    }
}

class waypointTrajectory
{
private:
    string  controlPointName;
    int     numberDimension;
    int     numberWaypoints;
    vector<Vector> waypoints;   // Delete it later
public:

    waypointTrajectory()
    {
        numberDimension = 0;
        numberWaypoints = 0;
        waypoints.clear();
    }

    waypointTrajectory(const string &ctrlPt,
                       const vector<Vector> &trajectory)
    {
        controlPointName = ctrlPt;
        waypoints = trajectory;
        numberWaypoints = trajectory.size();
        if (trajectory.size()>0)
            numberDimension = waypoints[0].size();
    }

    void setCtrlPointName(const string& ctrlPt)
    {
        controlPointName = ctrlPt;
    }


    void setWaypoints(const vector<Vector>& trajectory)
    {
        waypoints = trajectory;
        numberWaypoints = trajectory.size();
        if (trajectory.size()>0)
            numberDimension = waypoints[0].size();
    }

    string getCtrlPointName()
    {
        return controlPointName;
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
    using BufferedPort<Bottle>::onRead;
    virtual void onRead(Bottle &inPlan);

private:
    bool haveNewMsg;
    deque<waypointTrajectory> listTrajectory;  // Delete later
    Stamp ts;

public:
    motionPlan();
    ~motionPlan();




    void addTrajectory(const waypointTrajectory &ctrlPtTrajectory);

    void clearTrajectory();

    deque<waypointTrajectory>& getListTrajectory(); //Change to reference

    void sendPlan();

    void receivePlan();



    bool& gotNewMsg();

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
