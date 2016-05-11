#include "reachingSupervisor.h"

reachingSupervisor::reachingSupervisor()
{
    name = "reaching-supervisor";
}

bool reachingSupervisor::configure(ResourceFinder &rf)
{
    printf("reaching-supervisor: starting...\n");
    //******************* NAME ******************
    if (rf.check("name"))
    {
        name = rf.find("name").asString();
        yInfo("[reaching-supervisor] Module name set to %s", name.c_str());
    }
    else yInfo("[reaching-supervisor] Module name set to default, i.e. %s", name.c_str());
    setName(name.c_str());

    //****Normal communication port *******************************************************
    planPortIn.useCallback();

    string port2planner = "/"+name+"/bestCartesianTrajectory:i";
    if (!planPortIn.open(port2planner.c_str()))
        yError("[reaching-supervisor] Unable to open port << port2planner << endl");

    string portPlanner = "/reaching-planner/bestCartesianTrajectory:o";
    if(Network::connect(portPlanner.c_str(),port2planner.c_str()))
        printf("[reaching-supervisor] can connect to receive motion plan\n");

}

double reachingSupervisor::getPeriod()
{
    return 1.0;
}

bool reachingSupervisor::updateModule()
{


//    planPortIn.receivePlan();
    printf("updateModule reachingSupervisor\n");
    if (planPortIn.gotNewMsg())
    {
        printf("check haveNewMsg\n");
        planPortIn.haveNewMsg = false;



        vector<waypointTrajectory> &listTraject = planPortIn.getListTrajectory();
        if (listTraject.size()>0)
        {

            printf("Got some thing. listTrajectory.size()= %d\n",listTraject.size());

//            for (int i=0; i<listTraject.size(); i++)
//            {
//                printf("i= %d\n",i);
//                vector<Vector> tempTrajectory = listTraject[i].getWaypoints();
//                for (int j=0; j<tempTrajectory.size(); j++)
//                {
//                    printf("Waypoint[%d] = %f, %f, %f\n",j,tempTrajectory[j][0],tempTrajectory[j][1],tempTrajectory[j][2]);
//                }
//            }
        }

    }
//    if (planPortIn.gotNewMsg())
//    {

//        printf("===============================\n");
//        planPortIn.setNewMsg(false);
//        printf("Get list of Trajectory!!!\n");
//        vector<waypointTrajectory> &listTraject = planPortIn.getListTrajectory();
//        if (listTraject.size()>0)
//        {

//            printf("Got some thing. listTrajectory.size()= %d\n",listTraject.size());

//    //        for (int i=0; i<listTraject.size(); i++)
//    //        {
//    //            printf("i= %d\n",i);
//    //            vector<Vector> tempTrajectory = listTraject[i].getWaypoints();
//    //            for (int j=0; j<tempTrajectory.size(); j++)
//    //            {
//    //                printf("Waypoint[%d] = %f, %f, %f\n",j,tempTrajectory[j][0],tempTrajectory[j][1],tempTrajectory[j][2]);
//    //            }
//    //        }
//        }
//    }

}

bool reachingSupervisor::close()
{
    planPortIn.interrupt();
    planPortIn.close();
    listTrajectory.clear();
}
