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

#include "reachingSupervisor.h"

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

reachingSupervisor::reachingSupervisor()
{
    name        =  "reaching-supervisor";
    nDim        =                      3;
    rate        =                    100;   // in milisecond
    verbosity   =                      0;
    timeToFinishSegment =              1;
    tol         =                  0.005;   //0.001 with 10ms

    speedEE     =                    0.1;
    speedLink.clear();
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
//    tempWaypoint = new particleThread(rate,name,verbosity);

    // hard coded
//    tempWaypointEE = new particleThread(rate,name,verbosity);
//    tempWaypointEB = new particleThread(rate,name,verbosity);

//    tempWaypointEE = new particleWaypointThread(rate,"EE",verbosity,tol);
//    tempWaypointEB = new particleWaypointThread(rate,"EB",verbosity,tol);

    tempWaypoint = new multipleParticleThread(rate,name,verbosity,tol);

    finishedCurSegment = true;
    numberWaypoint = 0;


    return true;
}

double reachingSupervisor::getPeriod()
{
    return .10; // in second
}

bool reachingSupervisor::updateModule()
{

//    printf("updateModule reachingSupervisor\n");
    if (planPortIn.gotNewMsg())
    {
        listTrajectories.clear();
        ctrlPointsNames.clear();
        indexCurSegment=0;
        finishedCurSegment = true;
//        tempWaypointEE->stop();
//        tempWaypointEB->stop();

        //multi-waypoints
        tempWaypoint->stop();

        printf("===============================\n");
        printf("updateModule() reachingSupervisor\n");
        planPortIn.setNewMsg(false);



        deque<waypointTrajectory> &listTraject = planPortIn.getListTrajectory();
        if (listTraject.size()>0)
        {

            printf("Got some thing. listTrajectory.size()= %d\n",(int) listTraject.size());

            for (int i=0; i<listTraject.size(); i++)
            {

                printf("numberWaypoint = %d\n",numberWaypoint);
                string tempCtrlPtName = listTraject[i].getCtrlPointName();
                printf("CtrlPointName = %s\n",tempCtrlPtName.c_str());
                vector<Vector> tempTrajectory = listTraject[i].getWaypoints();
                for (int j=0; j<tempTrajectory.size(); j++)
                {
                    printf("\tWaypoint[%d] = %f, %f, %f\n",j,tempTrajectory[j][0],tempTrajectory[j][1],tempTrajectory[j][2]);
                }
            }
        }

        listTrajectories = planPortIn.getListTrajectory();
    }

    if (listTrajectories.size()>0)
    {
        // For each trajectory of each controlled point:
        // 1 Get 2 sucessive waypoints of trajectory and put to Segment
        // 2 Generate temporal waypoints between them
        // 3 Integrate after each period
        // 4 Check new particle is close to the 2nd waypoint
        //  a YES:
        //      - Set 2nd waypoint and next waypoint in trajectory as Segment
        //      - Come back to 1
        //  b NO: come back to 2 and continue
        //
//        for (int i=0; i<listTrajectories.size(); i++)
//        {
//            vector<Vector> tempTrajectory = listTrajectories[i].getWaypoints();
//            int indexSegment=0;
//            x_0 = tempTrajectory[indexSegment];
//            x_d = tempTrajectory[indexSegment+1];

//        }

        ctrlPointsNames.clear();
        for (int i=0; i<listTrajectories.size(); i++)
        {
            string tempCtrlPtName = listTrajectories[i].getCtrlPointName();
            ctrlPointsNames.push_back(tempCtrlPtName);
        }
        tempWaypoint->setCtrlPointsNames(ctrlPointsNames);

        numberWaypoint = listTrajectories[0].getNbWaypoint();
        if (indexCurSegment<numberWaypoint-1)
        {
            printf("indexCurSegment =%d\n", indexCurSegment);
            vector<Vector> tempTrajectoryEE = listTrajectories[0].getWaypoints();
            vector<Vector> tempTrajectoryEB = listTrajectories[1].getWaypoints();

            Vector x_0EE(nDim,0.0), x_0EB(nDim,0.0), x_dEE(nDim,0.0), x_dEB(nDim,0.0);
            Vector velEE(nDim,0.0), velEB(nDim,0.0);

            vector<Vector> x_0, vel;
            vector<Vector> x_d;

            x_0EE = tempTrajectoryEE[indexCurSegment];
            x_dEE = tempTrajectoryEE[indexCurSegment+1];

            x_0EB = tempTrajectoryEB[indexCurSegment];
            x_dEB = tempTrajectoryEB[indexCurSegment+1];

            speedEB = computeOtherCtrlPtSpeed(speedEE,x_0EE,x_dEE,x_0EB,x_dEB);

            velEE = computeVelFromSegment(speedEE,x_0EE,x_dEE);
            velEB = computeVelFromSegment(speedEB,x_0EB,x_dEB);

            //multi-waypoints
            x_0.push_back(x_0EE);
            x_0.push_back(x_0EB);
            vel.push_back(velEE);
            vel.push_back(velEB);
            x_d.push_back(x_dEE);
            x_d.push_back(x_dEB);

//            printf("x_0EE = %s\n", x_0EE.toString().c_str());
//            printf("x_dEE = %s\n", x_dEE.toString().c_str());
//            printf("x_0EB = %s\n", x_0EB.toString().c_str());
//            printf("x_dEB = %s\n", x_dEB.toString().c_str());

//            printf("speedEE = %f \t speedEB = %f \n", speedEE, speedEB);

//            printf("velEE = %s\n", velEE.toString().c_str());
//            printf("velEB = %s\n", velEB.toString().c_str());


            printf("finishedCurSegment =%d \n",finishedCurSegment);
            if (finishedCurSegment)
            {
                printf("check finishedCurSegment 1\n");

//                tempWaypointEE->stop();
//                tempWaypointEB->stop();

                //multi-waypoints
                tempWaypoint->stop();

                printf("check finishedCurSegment 2\n");
//                tempWaypointEE->setupNewParticle(x_0EE,velEE);
                printf("check finishedCurSegment 2a\n");
//                tempWaypointEB->setupNewParticle(x_0EB,velEB);

                //multi-waypoints
                tempWaypoint->setupNewParticle(x_0,vel);


                printf("check finishedCurSegment 3\n");
//                tempWaypointEE->setLastWaypoint(x_dEE);
//                tempWaypointEB->setLastWaypoint(x_dEB);

                //multi-waypoints
                tempWaypoint->setLastWaypoint(x_d);

                printf("check finishedCurSegment 4\n");
//                tempWaypointEE->start();
//                tempWaypointEB->start();

                //multi-waypoints
                tempWaypoint->start();
                printf("check finishedCurSegment 5\n");
            }

            printf("getParticle()\n");
//            Vector x_nEE = tempWaypointEE->getParticle();
//            Vector x_nEB = tempWaypointEB->getParticle();

            //multi-waypoints
            vector<Vector> x_n = tempWaypoint->getParticle();
            printf("check getParticle()\n");

    //        printf("x_nEE = %s\n", x_nEE.toString().c_str());
    //        printf("x_nEB = %s\n", x_nEB.toString().c_str());

//            printf("norm(x_nEE-x_dEE) = %f\t norm(x_nEB-x_dEB) = %f\n", norm(x_nEE-x_dEE), norm(x_nEB-x_dEB));
            printf("norm(x_nEE-x_dEE) = %f\t norm(x_nEB-x_dEB) = %f\n", norm(x_n[0]-x_d[0]), norm(x_n[1]-x_d[1]));

    //        if (finishedCurSegment = ((norm(x_nEE-x_dEE)<=tol) && (norm(x_nEB-x_dEB)<=tol)))
    //        if (finishedCurSegment = ((distWpWp(x_nEE,x_dEE)<=tol) && (distWpWp(x_nEB,x_dEB)<=tol)))
    //        if (finishedCurSegment=(!tempWaypointEE->checkRunning()&&!tempWaypointEB->checkRunning()))

//            if ((tempWaypointEE->checkFinished()&&tempWaypointEB->checkFinished()))
//            {
//                finishedCurSegment = true;
//                printf("reaching waypoint %d-th & finish current segment \n",indexCurSegment+1);
//                indexCurSegment++;
//            }
//            else
//                finishedCurSegment = false;


            if ((tempWaypoint->checkFinished()))
            {
                finishedCurSegment = true;
                printf("reaching waypoint %d-th & finish current segment \n",indexCurSegment+1);
                indexCurSegment++;
            }
            else
                finishedCurSegment = false;
            printf("check checkFinished()\n");
            printf("==========================\n");
        }
        else if (indexCurSegment>=numberWaypoint-1)
        {
//            tempWaypointEE->stop();
//            tempWaypointEB->stop();

            tempWaypoint->stop();
            listTrajectories.clear();
            printf("Finish trajectory. Waiting...\n");
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

Vector reachingSupervisor::computeVelFromSegment(const double &speed, const Vector &wp1, const Vector &wp2)
{
    Vector vel(nDim,0.0);
    vel = speed*(wp2-wp1)/norm(wp2-wp1);
    return vel;
}

double reachingSupervisor::computeOtherCtrlPtSpeed(const double &speed, const Vector &wp1, const Vector &wp2,
                                                                        const Vector &wp1Other, const Vector &wp2Other)
{
    double dist = distWpWp(wp1,wp2);
    double distOther = distWpWp(wp1Other,wp2Other);

    return (speed*distOther/dist);
}


double reachingSupervisor::distWpWp(const Vector &wp1, const Vector &wp2)
{
    double distance = 1000;
    if ((wp1.size()==nDim) && (wp2.size()==nDim))
    {
        double sqSum = 0;
        for (int i=0; i<nDim; i++)
            sqSum += pow((wp1[i]-wp2[i]),2.0);
        distance = sqrt(sqSum);
    }
    return distance;
}
