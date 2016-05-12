#include "reachingSupervisor.h"

reachingSupervisor::reachingSupervisor()
{
    name        =  "reaching-supervisor";
    rate        =                     10;
    verbosity   =                      0;
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

    //****Particle Thread******************************************************************
    tempWaypoint = new particleThread(rate,name,verbosity);

    return true;
}

double reachingSupervisor::getPeriod()
{
    return 1.0;
}

bool reachingSupervisor::updateModule()
{

//    printf("updateModule reachingSupervisor\n");
    if (planPortIn.gotNewMsg())
    {
        printf("===============================\n");
        printf("updateModule() reachingSupervisor\n");
        planPortIn.setNewMsg(false);



        deque<waypointTrajectory> &listTraject = planPortIn.getListTrajectory();
        if (listTraject.size()>0)
        {

            printf("Got some thing. listTrajectory.size()= %d\n",(int) listTraject.size());

            for (int i=0; i<listTraject.size(); i++)
            {
                string tempCtrlPtName = listTraject[i].getCtrlPointName();
                printf("CtrlPointName = %s\n",tempCtrlPtName.c_str());
                vector<Vector> tempTrajectory = listTraject[i].getWaypoints();
                for (int j=0; j<tempTrajectory.size(); j++)
                {
                    printf("\tWaypoint[%d] = %f, %f, %f\n",j,tempTrajectory[j][0],tempTrajectory[j][1],tempTrajectory[j][2]);
                }
            }
        }

    }


    return true;
}

bool reachingSupervisor::close()
{
    planPortIn.interrupt();
    planPortIn.close();
    listTrajectories.clear();

    return true;
}
