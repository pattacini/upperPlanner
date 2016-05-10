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

    string port2planner = "/"+name+"/bestCartesianTrajectory:i";
    if (!planPortIn.open(port2planner.c_str()))
        yError("[reaching-supervisor] Unable to open port << port2planner << endl");

    string portPlanner = "/reaching-planner/bestCartesianTrajectory:o";
    Network::connect(portPlanner.c_str(),port2planner.c_str());
    printf("[reaching-supervisor] can connect to receive motion plan");
}

double reachingSupervisor::getPeriod()
{
    return 1.0;
}

bool reachingSupervisor::updateModule()
{
    printf("check updateModule reachingSupervisor");
    planPortIn.receivePlan();
    printf("===============================\n");
    printf("Get list of Trajectory!!!\n");
    listTrajectory = planPortIn.getListTrajectory();
    if (listTrajectory.size()>0)
    {
        printf("Got some thing\n");
        for (int i=0; i<listTrajectory.size(); i++)
        {
    //        listTrajectory[i]
        }
    }

}

bool reachingSupervisor::close()
{
    planPortIn.interrupt();
    planPortIn.close();
}
