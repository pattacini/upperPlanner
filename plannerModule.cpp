#include <yarp/os/all.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <iostream>
#include <string.h>

#include "upperPlanner.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;
using namespace iCub::iKin;

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("upper-planner");
    rf.setDefaultConfigFile("upperPlanner.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("");
        yInfo("Options:");
        yInfo("");
        yInfo("   --context     path:  where to find the called resource");
        yInfo("   --from        from:  the name of the .ini file.");
        yInfo("   --name        name:  the name of the module (default reactController).");
        yInfo("   --robot       robot: the name of the robot. Default icubSim.");
        yInfo("   --part        part:  the arm to use. Default left_arm.");
        yInfo("   --rate        rate:  the period used by the thread. Default 100ms.");
        yInfo("   --verbosity   int:   verbosity level (default 0).");
        yInfo("");
        return 0;
    }

    if (!yarp.checkNetwork())
    {
        yError("No Network!!!");
        return -1;
    }

//    // Test=======================================================
//    Vector workspace(6,0.0);
//    workspace[3]=30.0/10.0;
//    workspace[4]=30.0/10.0;
//    workspace[5]=30.0/10.0;

//    double scale = 10.0;
//    double sizeObject = 1.0/scale;  //1.0/10.0;
//    double sizeGoal = .15; //1.4/scale;

//    Vector goal(6,sizeGoal);
//    goal[0]=-1.0/scale; //-3.0/scale; //-10.0/scale;
//    goal[1]=5.64/scale; //6.0/scale; //7.0/scale;
//    goal[2]=3.7/scale;  //1.0/scale;

//    srand(time(NULL));
//    vector<Vector> obsSet;
//    for (int i=0; i<=10; i++)
//    {
//        Vector obs(6, sizeObject);
//        obs[0] = (double)(rand()%8-4)/scale;
//        //obs[1] = 0.564;
//        obs[1] = 0.614;
//        obs[4] = 0.2;
//        obs[2] = (double)(rand()%4+3)/scale;
//        if ((abs(10*obs[0]-10*goal[0])>=abs(10*obs[3]-10*goal[3]))&&
//                (abs(10*obs[2]-10*goal[2])>=abs(10*obs[5]-10*goal[5]))&&
//                (abs(10*obs[2]-10*goal[2])>=abs(10*obs[5]-10*goal[5])))
//        {
//            obsSet.push_back(obs);
//        }

//    }
//    //============================================================

    upperPlanner planner;
    planner.configure(rf);

//    planner.setRegionOperating(workspace);
//    planner.setGoal(goal);
//    planner.setDeadline(0.65);
//    planner.setObstacles(obsSet);



    return planner.runModule();
    //return planner.runModule(rf);   // runModule(rf) = configure() + runModule()

}
