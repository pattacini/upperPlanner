#include "reachingSupervisorThread.h"

reachingSupervisorThread::reachingSupervisorThread()
{
    name = "reaching-supervisor";
}

bool reachingSupervisorThread::configure(ResourceFinder &rf)
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

    string port2planner = "/"+name+"/bestCartesianTrajectory:i";
    if (!planPortIn.open(port2planner.c_str()))
        yError("[reaching-supervisor] Unable to open port << port2planner << endl");

    string portPlanner = "/reaching-planner/bestCartesianTrajectory:o";
    if(Network::connect(portPlanner.c_str(),port2planner.c_str()))
        printf("[reaching-supervisor] can connect to receive motion plan\n");
}

double reachingSupervisorThread::getPeriod()
{
    return 1.0;
}

bool reachingSupervisorThread::updateModule()
{
//    vector<waypointTrajectory> &listTraject;
    printf("check updateModule reachingSupervisorThread\n");
//    planPortIn.receivePlan();
    printf("===============================\n");
    printf("Get list of Trajectory!!!\n");
    vector<waypointTrajectory> &listTraject = planPortIn.getListTrajectory();
    if (listTraject.size()>0)
    {

        printf("Got some thing. listTrajectory.size()= %d\n",listTraject.size());

//        for (int i=0; i<listTraject.size(); i++)
//        {
//            printf("i= %d\n",i);
//            vector<Vector> tempTrajectory = listTraject[i].getWaypoints();
//            for (int j=0; j<tempTrajectory.size(); j++)
//            {
//                printf("Waypoint[%d] = %f, %f, %f\n",j,tempTrajectory[j][0],tempTrajectory[j][1],tempTrajectory[j][2]);
//            }
//        }
    }

}

bool reachingSupervisorThread::close()
{
    planPortIn.interrupt();
    planPortIn.close();
    listTrajectory.clear();
}
