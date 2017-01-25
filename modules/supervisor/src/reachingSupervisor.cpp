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
    rate        =                     50;   // in milisecond
    verbosity   =                      0;
    timeToFinishSegment =              1;
    tol         =                  0.005;   //0.001 with 10ms

    speedEE     =                    0.1;
    speedLink.clear();

    localPlanningTime   =            1.0;
    globalPlanningTime  =           10.0;
    targetName          =      "Octopus";

    gotPlan             =          false;
}

bool reachingSupervisor::configure(ResourceFinder &rf)
{
    printf("reaching-supervisor: starting...\n");
    //******************* NAME ******************
    if (rf.check("name"))
    {
        name = rf.find("name").asString();
        yInfo("[%s] Module name set to %s", name.c_str(),name.c_str());
    }
    else yInfo("[%s] Module name set to default, i.e. %s", name.c_str(),name.c_str());
    setName(name.c_str());

    //******************* VERBOSE ******************
    if (rf.check("verbosity"))
    {
        verbosity = rf.find("verbosity").asInt();
        yInfo("[%s] verbosity set to %i", name.c_str(), verbosity);
    }
    else yInfo("[%s] Could not find verbosity option in the config file; using %i as default", name.c_str(), verbosity);

    //******************* RATE OF THREAD ******************
    if (rf.check("rate"))
    {
        rate = rf.find("rate").asDouble();
        yInfo("[%s] rate set to %i ms", name.c_str(), rate);
    }
    else yInfo("[%s] Could not find rate option in the config file; using %i as default", name.c_str(), rate);

    //******************* TOLERANCE ******************
    if (rf.check("tolerance"))
    {
        tol = rf.find("tolerance").asDouble();
        yInfo("[%s] tolerance set to %f", name.c_str(), tol);
    }
    else yInfo("[%s] Could not find tolerance option in the config file; using %f as default", name.c_str(), tol);

    //******************* SPEED OF END-EFFECTOR*******
    if (rf.check("speedEE"))
    {
        speedEE = rf.find("speedEE").asDouble();
        yInfo("[%s] speedEE set to %f m/s", name.c_str(), speedEE);
    }
    else yInfo("[%s] Could not find speedEE option in the config file; using %f as default", name.c_str(), speedEE);

    //****Normal communication port *******************************************************
    planPortIn.useCallback();

    // Output port to send command to Planner
    string portToPlanner = "/"+name+"/reaching-planner/rpc:o";
    if (!rpc2Planner.open(portToPlanner.c_str()))
        yError("[%s] Unable to open port << portToPlanner << endl",name.c_str());

    // Output port to send command to reactCtroller
    string portToReactCtrl = "/"+name+"/reactController/rpc:o";
    if (!rpc2reactCtrl.open(portToReactCtrl.c_str()))
        yError("[%s] Unable to open port << portToReactCtrl << endl",name.c_str());

    // Input port from the Planner
    string portFromPlanner = "/"+name+"/bestCartesianTrajectory:i";
    if (!planPortIn.open(portFromPlanner.c_str()))
        yError("[%s] Unable to open port << portFromPlanner << endl",name.c_str());

    string portOfPlanner = "/reaching-planner/bestCartesianTrajectory:o";
    if(Network::connect(portOfPlanner.c_str(),portFromPlanner.c_str()))
        yInfo("[%s] can connect to receive motion plan\n",name.c_str());

    // Output port to Controller, used to transfer information to multiple particle generation
    string portParticle = "/"+name+"/particlesCartesianTrajectory:o";

    string port2planner1 = "/"+name+"/bestCartesianTrajectory1:i";
    if (!planPortIn1.open(port2planner1.c_str()))
        yError("[%s] Unable to open port << port2planner1 << endl",name.c_str());

    //****RPC Service port ****************************************************************
    rpcSrvr.open(("/"+name+"/rpc:i").c_str());
    attach(rpcSrvr);

    //****Particle Thread******************************************************************

    // hard coded
    tempWaypoint = new multipleParticleThread(rate,name,verbosity,tol,portParticle);

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
    if (planPortIn.gotNewMsg())
    {
        listTrajectories.clear();
        ctrlPointsNames.clear();
        indexCurSegment=0;
        finishedCurSegment = true;

        tempWaypoint->stop();

        printf("===============================\n");
        printf("[%s] updateModule()\n", name.c_str());
        gotPlan = true;
        planPortIn.setNewMsg(false);



        deque<waypointTrajectory> &listTraject = planPortIn.getListTrajectory();
        if (listTraject.size()>0)
        {

            printf("\tGot some thing. listTrajectory.size()= %d\n",(int) listTraject.size());

            for (int i=0; i<listTraject.size(); i++)
            {

                printf("\tnumberWaypoint = %d\n",listTraject[0].getNbWaypoint());
                string tempCtrlPtName = listTraject[i].getCtrlPointName();
                printf("\tCtrlPointName = %s\n",tempCtrlPtName.c_str());

                if (tempCtrlPtName=="End-Effector")
                    lengthEE = listTraject[i].getLengthTraj();

                vector<Vector> tempTrajectory = listTraject[i].getWaypoints();
                for (int j=0; j<tempTrajectory.size(); j++)
                {
                    printf("\t\tWaypoint[%d] = %f, %f, %f\n",j,tempTrajectory[j][0],tempTrajectory[j][1],tempTrajectory[j][2]);
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
            printf("\tindexCurSegment =%d\n", indexCurSegment);
            vector<Vector> tempTrajectoryEE = listTrajectories[0].getWaypoints();


            Vector x_0EE(nDim,0.0), x_0EB(nDim,0.0), x_dEE(nDim,0.0), x_dEB(nDim,0.0);
            Vector velEE(nDim,0.0), velEB(nDim,0.0);

            vector<Vector> x_0, vel;
            vector<Vector> x_d;

            x_0EE = tempTrajectoryEE[indexCurSegment];
            x_dEE = tempTrajectoryEE[indexCurSegment+1];

            velEE = computeVelFromSegment(speedEE,x_0EE,x_dEE);

            //multi-waypoints
            x_0.push_back(x_0EE);
            vel.push_back(velEE);
            x_d.push_back(x_dEE);

            // Elbow setting
            if (listTrajectories.size()==2)
            {
                vector<Vector> tempTrajectoryEB = listTrajectories[1].getWaypoints();
                x_0EB = tempTrajectoryEB[indexCurSegment];
                x_dEB = tempTrajectoryEB[indexCurSegment+1];

                speedEB = computeOtherCtrlPtSpeed(speedEE,x_0EE,x_dEE,x_0EB,x_dEB);

                velEB = computeVelFromSegment(speedEB,x_0EB,x_dEB);

                x_0.push_back(x_0EB);
                vel.push_back(velEB);
                x_d.push_back(x_dEB);
            }

            printf("\tfinishedCurSegment =%d \n",finishedCurSegment);
            if (finishedCurSegment)
            {
                printf("check finishedCurSegment 1\n");
                tempWaypoint->stop();

                printf("check finishedCurSegment 2\n");
                printf("check finishedCurSegment 2a\n");
                tempWaypoint->setupNewParticle(x_0,vel);

                printf("check finishedCurSegment 3\n");
                tempWaypoint->setLastWaypoint(x_d);

                printf("check finishedCurSegment 4\n");
                tempWaypoint->start();

                printf("check finishedCurSegment 5\n");
            }

            vector<Vector> x_n = tempWaypoint->getParticle();
            printf("check getParticle()\n");

            for (int i=0; i<x_n.size(); i++)
            {
                printf("norm(x_n[%d]-x_d[%d]) = %f\n", i, i, norm(x_n[i]-x_d[i]));
            }

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
            tempWaypoint->stop();
            listTrajectories.clear();
            printf("Finish trajectory. Waiting for new plan...\n");
        }
    }



    return true;
}

bool reachingSupervisor::close()
{
    planPortIn.interrupt();
    planPortIn.close();
    listTrajectories.clear();

    rpcSrvr.interrupt();
    rpcSrvr.close();

    rpc2Planner.interrupt();
    rpc2Planner.close();

    delete tempWaypoint;

    return true;
}

bool reachingSupervisor::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
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

bool reachingSupervisor::setTol(const double &_tol)
{
    if (_tol>=0.0)
    {
        tol=_tol;
        return true;
    }
    return false;
}

double reachingSupervisor::getTol()
{
    return tol;
}

bool reachingSupervisor::setSpeedEE(const double &_speedEE)
{
    if (_speedEE>=0.0)
    {
        speedEE = _speedEE;
        return true;
    }
    else
        return false;
}

double reachingSupervisor::getSpeedEE()
{
    return speedEE;
}

bool reachingSupervisor::setVerbosity(const int &_verbosity)
{
    if (_verbosity>=0)
        verbosity=_verbosity;
    else
        verbosity=0;
    return true;
}

int reachingSupervisor::getVerbosity()
{
    return verbosity;
}

bool reachingSupervisor::setDeadline(const double &_deadline)
{
    if (_deadline >= 0.0)
    {
        localPlanningTime = _deadline;
        return true;
    }
    else
        return false;
}

double reachingSupervisor::getDeadline()
{
    return localPlanningTime;
}

bool reachingSupervisor::setGlobDeadline(const double &_globDeadline)
{
    if (_globDeadline >= 0.0)
    {
        globalPlanningTime = _globDeadline;
        return true;
    }
    else
        return false;
}

double reachingSupervisor::getGlobDeadline()
{
    return globalPlanningTime;
}

bool reachingSupervisor::setTarget(const string &_target)
{
    targetName = _target;
    return true;
}

string reachingSupervisor::getTarget()
{
    return targetName;
}

bool reachingSupervisor::sendCmd2Planner()
{
    Bottle cmd;
    cmd.addString("replan");
    cmd.addDouble(localPlanningTime);
    cmd.addString(targetName);
    return rpc2Planner.write(cmd);
}

bool reachingSupervisor::sendCmd2PlannerPos(const Vector &targetPos)
{
    Bottle cmd;
    cmd.addString("planPos");
    for (int i=0; i<targetPos.size(); i++)
        cmd.addDouble(targetPos[i]);
    cmd.addDouble(localPlanningTime);
    return rpc2Planner.write(cmd);
}

bool reachingSupervisor::sendCmd2ReactCtrl_stop()
{
    Bottle cmd;
    cmd.addString("stop");
    return rpc2reactCtrl.write(cmd);
}

bool reachingSupervisor::resumeCtrl()
{
    return tempWaypoint->resumeParticle();
}

bool reachingSupervisor::stopCtrl()
{
    sendCmd2ReactCtrl_stop();
    return tempWaypoint->stopParticle();
}
