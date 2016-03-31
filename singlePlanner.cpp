/*
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

#include "singlePlanner.h"

singlePlanner::singlePlanner(const int& _verbosity, const string& _name,
                             const string& _robot, const string& _controlPoint)
{
    verbosity   =        _verbosity;
    name        =             _name;
    robot       =            _robot;
    controlPoint=             _controlPoint;
    nDim        =                 3;
    nIter       =             50000;
    deadline    =               1.0;
    system.setNumDimensions(nDim);
    init();
}

void singlePlanner::init()
{
    //**** visualizing targets and collision points in simulator ***************************
    string port2icubsim = "/" + name + "/" + controlPoint + "/sim:o";
    //cout<<port2icubsim<<endl;
    if (!portToSimWorld.open(port2icubsim.c_str())) {
        yError("Unable to open port << port2icubsim << endl");
    }
    std::string port2world = "/icubSim/world";
    yarp::os::Network::connect(port2icubsim, port2world.c_str());

    if (controlPoint != "local-Half-Elbow")
    {
        cmd.clear();
        cmd.addString("world");
        cmd.addString("del");
        cmd.addString("all");
        portToSimWorld.write(cmd);
    }

    T_world_root = zeros(4,4);
    T_world_root(0,1)=-1;
    T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
    T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
    T_world_root(3,3)=1;
    T_root_world=SE3inv(T_world_root);
}

int singlePlanner::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
}

void singlePlanner::setIteration(const int &nIteration)
{
    nIter = nIteration;
}

void singlePlanner::setDeadline(const double &_deadline)
{
    deadline = _deadline;
}

void singlePlanner::setRegionOperating(const Vector &regionOperating)
{
    system.regionOperating.setNumDimensions(nDim);
    for (int i=0; i<nDim; i++)
    {
        system.regionOperating.center[i]=regionOperating[i];
        system.regionOperating.size[i]=regionOperating[i+3];
    }
}

void singlePlanner::setGoal(const Vector &goal)
{
    system.regionGoal.setNumDimensions(nDim);
    for (int i=0; i<nDim; i++)
    {
        system.regionGoal.center[i]=goal[i];
        system.regionGoal.size[i] = goal[i+3];
    }
    //target = new Vector(2*nDim,0.0);
    target = goal;
}

void singlePlanner::setStart(const Vector &start)
{
  rrts.setSystem(system);

  vertex_t &root = rrts.getRootVertex();
  State &rootState = root.getState();
  for (int i=0; i<nDim; i++)
    {
      rootState[i] = start[i];
    }
  startPos = start;
}

void singlePlanner::setObstacles(const std::vector<Vector> &obstacles)
{
  //obsSet.resize(obstacles.size());
  region *obstacle;
  for(int j=0; j<obstacles.size(); j++)
    {
      obstacle = new region;
      obstacle->setNumDimensions(nDim);
      Vector obs(2*nDim,0.0);
      for (int i=0; i<nDim; i++)
        {
          obstacle->center[i] = obstacles[j][i];
          obstacle->size[i] = obstacles[j][i+3];
          obs[i]=obstacles[j][i];
          obs[i+3]=obstacles[j][i+3];
        }
      system.obstacles.push_front(obstacle);
      obsSet.push_back(obs);
    }
//  cout<<"data in obsSet "<<endl;
//  for (int j=0; j<obsSet.size(); j++)
//    {
//      printf("Obstacle [%i] = [%g,%g,%g,%g,%g,%g]\n",
//         j, obsSet[j][0], obsSet[j][1], obsSet[j][2],
//      obsSet[j][3], obsSet[j][4], obsSet[j][5]);
//    }
//    cout<<endl;
//  rrts.setSystem(system);
}

void singlePlanner::updatePlanner()
{
  rrts.initialize ();

  // This parameter should be larger than 1.5 for asymptotic
  //   optimality. Larger values will weigh on optimization
  //   rather than exploration in the RRT* algorithm. Lower
  //   values, such as 0.1, should recover the RRT.
  rrts.setGamma (1.5);
}

vector<Vector> singlePlanner::generateTrajectory()
//void singlePlanner::generateTrajectory(vector<Vector> newBest)
{
    Vector vertexBest(nDim,0.0);
    Vector vertexBestRoot(nDim,0.0);
//    int stateIndex = 0;
    list<double*> traject;


    clock_t start = clock();
    //Time startTime = Time::now();
    //*******************************************************************//
    // Main loop to generate a plan
    for (int i = 0; i < nIter; i++)
    {
        rrts.iteration ();

        // added on 01/03/2016 for testin time deadline of planner
        clock_t finish = clock();
        //Time finishTime = Time::now();
        solveTime = ((double)(finish-start))/CLOCKS_PER_SEC;
        if (solveTime>=deadline)
          break;
        // end of added part
    }
    //*******************************************************************//

    // commented on 01/03/2016 for testing time deadline of planner
//    clock_t finish = clock();
//    //Time finishTime = Time::now();
//    solveTime = ((double)(finish-start))/CLOCKS_PER_SEC;
    // end of commented part

    //solveTime1 = ((finishTime-startTime))/CLOCKS_PER_SEC;
//    cout << "Time inside : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;


    rrts.getBestTrajectory(traject);
    for (list<double*>::const_iterator itt = traject.begin(); itt!=traject.end();itt++)
    {
        double* stateRef = *itt;
        for (int i=0; i<nDim; i++)
          {
            vertexBest[i] = stateRef[i];
          }
        bestTraj.push_back(vertexBest);
        convertPosFromSimToRootFoR(vertexBest,vertexBestRoot);
        bestTrajRoot.push_back(vertexBestRoot);
//        cout<<"state ["<<stateIndex<<"]: "<<stateRef[0]<<","<<stateRef[1]<<","<<stateRef[2]<<endl;
//        stateIndex++;
    }
    // Add the goal state to bestTraj
//    vertexBest = target.subVector(0,2);
//    convertPosFromSimToRootFoR(vertexBest,vertexBestRoot);
//    bestTraj.push_back(vertexBest);
//    bestTrajRoot.push_back(vertexBestRoot);

    cout<<"\nTrajectory is generated in "<<solveTime<<" seconds !!!"<< endl;
    cout<<endl;
    return bestTraj;
}

vector<Vector> singlePlanner::getBestTrajRoot()
{
  return bestTrajRoot;
}

void singlePlanner::executeTrajectory(vector<Vector> &_bestTraj,
                                      vector<Vector> &_bestTrajRoot,
                                      const string &color)
{

    updatePlanner();
    _bestTraj = generateTrajectory();
    _bestTrajRoot = getBestTrajRoot();
    printTrajectory();
    displayPlan(color);
    logTrajectory();
}

void singlePlanner::printTrajectory()
{
    vertex_t &root = rrts.getRootVertex();
    State &rootState = root.getState();
    printf("===============================\n");
    printf("PLAN PROBLEM\n");
    cout<<"Workspace center: "<<rrts.system->regionOperating.center[0]<<","<<rrts.system->regionOperating.center[1]<<","<<rrts.system->regionOperating.center[2]<<endl;
    cout<<"Workspace size: "<<rrts.system->regionOperating.size[0]<<","<<rrts.system->regionOperating.size[1]<<","<<rrts.system->regionOperating.size[2]<<endl;
    cout<<"Start: "<<rootState[0]<<","<<rootState[1]<<","<<rootState[2]<<endl;
    cout<<"Target center: "<<rrts.system->regionGoal.center[0]<<","<<rrts.system->regionGoal.center[1]<<","<<rrts.system->regionGoal.center[2]<<endl;
    cout<<"Target size: "<<rrts.system->regionGoal.size[0]<<","<<rrts.system->regionGoal.size[1]<<","<<rrts.system->regionGoal.size[2]<<endl;
    int nObs = 0;
    list<region*> obs = rrts.system->obstacles;
    for (list<region*>::iterator obsItt = obs.begin(); obsItt!=obs.end(); obsItt++)
      {
        region* obsState = *obsItt;
        cout<<"obstacle ["<<nObs++<<"]: "<<obsState->center[0]<<","<<obsState->center[1]<<","<<obsState->center[2];
        cout<<"; size: "<<obsState->size[0]<<","<<obsState->size[1]<<","<<obsState->size[2]<<endl;
      }
    printf("\n===============================\n");
    printf("PLANNER RESULTS\n");

    int nVertices = rrts.numVertices;
    cout<<"Numbers of vertics: "<< nVertices<<endl;

    vertex_t& vertexBest = rrts.getBestVertex ();
    if (&vertexBest == NULL) {
        cout << "No best vertex" << endl;
        //return 0;
    }
    else
      {
        cout << "There is a list of best vertices" <<endl;

      }
    for (int j=0; j<bestTraj.size(); j++)
      //for (int i=0; i<nDim; i++)
      {
        printf("vertex [%i] = [%g,%g,%g]\n",
               j, bestTraj[j][0], bestTraj[j][1], bestTraj[j][2]);
      }

    for (int j=0; j<bestTrajRoot.size(); j++)
      //for (int i=0; i<nDim; i++)
    {
        printf("vertex in root [%i] = [%g,%g,%g]\n",
           j, bestTrajRoot[j][0], bestTrajRoot[j][1], bestTrajRoot[j][2]);
    }

}

void singlePlanner::logVertices()
{
    char logNameVertices[50], logNameWPs[50];
    sprintf(logNameVertices,"%s_listVertices.txt",controlPoint.c_str());
    sprintf(logNameWPs,"%s_waypoints.txt",controlPoint.c_str());
    ofstream logfile(logNameVertices);
    ofstream logfile1(logNameWPs);

    if (logfile.is_open())
    {
        for (list<vertex_t*>::iterator iter=rrts.listVertices.begin(); iter!= rrts.listVertices.end(); iter++)
        {
            vertex_t &vertexCurr = **iter;
            vertex_t &vertexParent = vertexCurr.getParent();
            if (&vertexParent == NULL)
                continue;
            State &stateCurr = vertexCurr.getState();
            State &stateParent = vertexParent.getState();
            //cout<<"vertex ["<<vertexIndex<<"]: "<<stateCurr[0]<<","<<stateCurr[1]<<","<<stateCurr[2]<<endl;
            logfile<< stateCurr[0] << "\t";
            logfile<< stateCurr[1] << "\t";
            logfile<< stateCurr[2] << "\t";
            logfile<< stateParent[0] << "\t";
            logfile<< stateParent[1] << "\t";
            logfile<< stateParent[2] << "\n";
        }

    }
    logfile.close();

    if (logfile1.is_open())
    {
        for (int j=0; j<bestTraj.size(); j++)
        {
            for (int i=0; i<nDim; i++)
              logfile1<< bestTraj[j][i] <<"\t";
            logfile1<<"\n";
        }
    }
    logfile1.close();
}

void singlePlanner::logTrajectory()
{
    char logNameWorkspace[50];
    char logNameResult[50];
    sprintf(logNameWorkspace,"%s_workspacePlanner.txt",controlPoint.c_str());
    sprintf(logNameResult,"%s_resultPlanner.txt",controlPoint.c_str());
    ofstream logfile(logNameWorkspace);
    ofstream logfile1(logNameResult);

//    ofstream logfile("workspacePlanner.txt");
//    ofstream logfile1("resultPlanner.txt");

    if (logfile.is_open())
      {
        logfile<< "Workspace\t";
        logfile<< rrts.system->regionOperating.center[0] <<"\t";
        logfile<< rrts.system->regionOperating.center[1] <<"\t";
        logfile<< rrts.system->regionOperating.center[2] <<"\t";
        //logfile<< "Space_size\t";
        logfile<< rrts.system->regionOperating.size[0] <<"\t";
        logfile<< rrts.system->regionOperating.size[1] <<"\t";
        logfile<< rrts.system->regionOperating.size[2] <<"\n";
        logfile<< "Target\t";
        logfile<< rrts.system->regionGoal.center[0]<<"\t";
        logfile<< rrts.system->regionGoal.center[1]<<"\t";
        logfile<< rrts.system->regionGoal.center[2]<<"\t";
        //logfile<< "Target_size\t";
        logfile<< rrts.system->regionGoal.size[0]<<"\t";
        logfile<< rrts.system->regionGoal.size[1]<<"\t";
        logfile<< rrts.system->regionGoal.size[2]<<"\n";
        logfile<< "Start\t";
        logfile<< startPos[0]<<"\t";
        logfile<< startPos[1]<<"\t";
        logfile<< startPos[2]<<"\t";
        logfile<< "0" <<"\t";
        logfile<< "0" <<"\t";
        logfile<< "0" <<"\n";
        //logfile<< "==============================="<<"\n";
        for (int j=0; j<obsSet.size(); j++)
          {
            logfile<< "Obs"<<j<<"\t";
            for (int i=0; i<2*nDim; i++)
              logfile<< obsSet[j][i] <<"\t";
            logfile<<"\n";
          }
      }
    else
      cerr << "File not open!"<< endl;

    if (logfile1.is_open())
      {
        logfile1<<"Solving_time:\t"<<solveTime<<"\n";
        for (int j=0; j<bestTraj.size(); j++)
          {
            logfile1<< "Way-point"<<j<<"\t";
            for (int i=0; i<nDim; i++)
              logfile1<< bestTraj[j][i] <<"\t";
            logfile1<<"\n";
          }

      }
    logfile.close();
    logfile.close();
}

void singlePlanner::displayPlan(const string &color)
{
    printf("\n===============================\n");
    cout<<"DISPLAY PLAN"<<endl;
    int indexFirstObsSet = 0;
    if (robot == "icubSim")
        indexFirstObsSet = 2;   // Do not display robot and table
    else
        indexFirstObsSet = 1;   // Do not display robot

    if (controlPoint == "End-effector")
    {
        for (int j=indexFirstObsSet; j<obsSet.size(); j++) // Do not display the robot itself
          {
            createStaticBox(obsSet[j],"obstacle");
          }
    }
//    for (int i=0; i<bestTraj.size(); i++)
//      {
//        createStaticSphere(0.03, bestTraj[i], color);
//      }
    if (controlPoint == "End-effector")
        createStaticBox(target,"goal");
}

void singlePlanner::convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos)
{
    Vector pos_temp = pos;
    pos_temp.resize(4);
    pos_temp(3) = 1.0;

    //printf("convertPosFromRootToSimFoR: need to convert %s in icub root FoR to simulator FoR.\n",pos.toString().c_str());
    //printf("convertPosFromRootToSimFoR: pos in icub root resized to 4, with last value set to 1:%s\n",pos_temp.toString().c_str());

    outPos.resize(4,0.0);
    outPos = T_world_root * pos_temp;
    //printf("convertPosFromRootToSimFoR: outPos in simulator FoR:%s\n",outPos.toString().c_str());
    outPos.resize(3);
    //printf("convertPosFromRootToSimFoR: outPos after resizing back to 3 values:%s\n",outPos.toString().c_str());
    return;
}

void singlePlanner::convertPosFromSimToRootFoR(const Vector &pos, Vector &outPos)
{
    Vector pos_temp = pos;
    pos_temp.resize(4);
    pos_temp(3) = 1.0;

    //printf("convertPosFromRootToSimFoR: need to convert %s in icub root FoR to simulator FoR.\n",pos.toString().c_str());
    //printf("convertPosFromRootToSimFoR: pos in icub root resized to 4, with last value set to 1:%s\n",pos_temp.toString().c_str());

    outPos.resize(4,0.0);
    outPos = T_root_world * pos_temp;
    //printf("convertPosFromRootToSimFoR: outPos in simulator FoR:%s\n",outPos.toString().c_str());
    outPos.resize(3);
    //printf("convertPosFromRootToSimFoR: outPos after resizing back to 3 values:%s\n",outPos.toString().c_str());
    return;
}

void singlePlanner::createStaticSphere(double radius, const Vector &pos, const string &color)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("ssph");
    cmd.addDouble(radius);

    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color
    if (color=="red")
    {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(0);  //red
    }
    else if (color == "green")
    {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(0);  //green
    }
    else if (color == "blue")
    {
        cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    }
    else if (color == "purple")
    {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(1);  //purple
    }
    else if (color == "yellow")
    {
        cmd.addInt(1);cmd.addInt(1);cmd.addInt(0);  //yellow
    }
    else if (color == "magenta")
    {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(1);  //magenta
    }
    else
    {
        cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    }

//    cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    cmd.addString("false"); //no collisions
    printMessage(5,"createSphere(): sending %s \n",cmd.toString().c_str());
    portToSimWorld.write(cmd);
}

void singlePlanner::moveSphere(int index, const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("ssph");
    cmd.addInt(index);
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    portToSimWorld.write(cmd);
}

void singlePlanner::createStaticBox(const Vector &pos, const string &type)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(pos(3)); cmd.addDouble(pos(4)); cmd.addDouble(pos(5)); //fixed size

    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color
    if (type == "obstacle")
      {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(0); //red
//        cmd.addString("true"); //collisions
      }
    else if (type == "goal")
      {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(0); //green
//        cmd.addString("false"); //no collisions
      }
    cmd.addString("false"); //no collisions
    printMessage(5,"createBox(): sending %s \n",cmd.toString().c_str());
    portToSimWorld.write(cmd);
}

void singlePlanner::moveBox(int index, const Vector &pos)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("sbox");
    cmd.addInt(index);
    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    portToSimWorld.write(cmd);
}
