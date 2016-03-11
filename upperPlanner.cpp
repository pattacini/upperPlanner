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

#include "upperPlanner.h"


using namespace yarp;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

upperPlanner::upperPlanner()
{
    robot       =         "icubSim";
    name        =    "upperPlanner";
    part        =        "left_arm";
    verbosity   =                 0;
    disableTorso=             false;

    nDim        =                 3;
    lShoulder   =              0.05;
    lArm        =              0.22;
    lForearm    =              0.16;

    if (robot == "icubSim")
    {
        visualizeObjectsInSim = true;
    }
    else
    {
        visualizeObjectsInSim = false;
    }
}

//bool upperPlanner::re_plan(const double& _deadline)
//{
//    yInfo("");
//    yInfo("[upperPlanner] received 'replan' command at new deadline of %f",_deadline);
//    printf("[upperPlanner] received 'replan' command at new deadline of %f",_deadline);
//    return replan = true;
////    return restartPlanner();
//}

//bool upperPlanner::restartPlanner()
//{
//    return replan = true;

//}

int upperPlanner::printMessage(const int l, const char *f, ...) const
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

bool upperPlanner::configure(ResourceFinder &rf){

    printf("upperPlanner: starting...\n");
    //******************* NAME ******************
    if (rf.check("name"))
    {
        name = rf.find("name").asString();
        yInfo("[upperPlanner] Module name set to %s", name.c_str());
    }
    else yInfo("[upperPlanner] Module name set to default, i.e. %s", name.c_str());
    setName(name.c_str());

    //******************* ROBOT ******************
    if (rf.check("robot"))
    {
        robot = rf.find("robot").asString();
        yInfo("[upperPlanner] Robot is: %s", robot.c_str());
    }
    else yInfo("[upperPlanner] Could not find robot option in the config file; using %s as default",robot.c_str());

    //******************* VERBOSE ******************
    if (rf.check("verbosity"))
    {
        verbosity = rf.find("verbosity").asInt();
        yInfo("[upperPlanner] verbosity set to %i", verbosity);
    }
    else yInfo("[upperPlanner] Could not find verbosity option in the config file; using %i as default",verbosity);

    //******************* PART ******************
    if (rf.check("part"))
    {
        part = rf.find("part").asString();
        part_short = part;
        if (part=="left")
        {
            part="left_arm";
//            arm = iCubArm("left");
        }
        else if (part=="right")
        {
            part="right_arm";
//            arm = iCubArm("right");
        }
        else if (part!="left_arm" && part!="right_arm")
        {
            part="left_arm";
//            arm = iCubArm("left");
            yWarning("[upperPlanner] part was not in the admissible values. Using %s as default.",part.c_str());
        }
        yInfo("[upperPlanner] part to use is: %s", part.c_str());
    }
    else yInfo("[upperPlanner] Could not find part option in the config file; using %s as default",part.c_str());

    //********************** CONFIGS ***********************
    if (rf.check("disableTorso"))
    {
        if(rf.find("disableTorso").asString()=="on"){
            disableTorso = true;
            useTorso = false;
            yInfo("[upperPlanner] disableTorso flag set to on.");
        }
        else{
            disableTorso = false;
            useTorso = true;
            yInfo("[upperPlanner] disableTorso flag set to off.");
        }
    }
    else
    {
         yInfo("[upperPlanner] Could not find disableTorso flag (on/off) in the config file; using %d as default",disableTorso);
    }
    //********************** Visualizations in simulator ***********************
   if (robot == "icubSim"){
       if (rf.check("visualizeObjectsInSim"))
       {
           if(rf.find("visualizeObjectstInSim").asString()=="on"){
               visualizeObjectsInSim = true;
               yInfo("[upperPlanner] visualizeObjectsInSim flag set to on.");
           }
           else{
               visualizeObjectsInSim = false;
               yInfo("[upperPlanner] visualizeObjectsInSim flag set to off.");
           }
       }
       else
       {
           yInfo("[upperPlanner] Could not find visualizeObjectsInSim flag (on/off) in the config file; using %d as default",visualizeObjectsInSim);
       }
   }

   /******** iKin chain and variables, and transforms init *************************/

    arm = new iCubArm(part_short.c_str());
    // Release / block torso links (blocked by default)
    for (int i = 0; i < 3; i++)
    {
        if (useTorso)
        {
            arm->releaseLink(i);
        }
        else
        {
            arm->blockLink(i,0.0);
        }
    }

    /*****  Drivers, interfaces, control boards etc. ***********************************************************/

    yarp::os::Property OptA;
    OptA.put("robot",  robot.c_str());
    OptA.put("part",   part.c_str());
    OptA.put("device", "remote_controlboard");
    OptA.put("remote",("/"+robot+"/"+part).c_str());
    OptA.put("local", ("/"+name +"/"+part).c_str());
    if (!ddA.open(OptA))
    {
        yError("[reactCtrlThread]Could not open %s PolyDriver!",part.c_str());
        return false;
    }

    bool okA = 1;

    if (ddA.isValid())
    {
        okA = okA && ddA.view(iencsA);
        okA = okA && ddA.view(ivelA);
        okA = okA && ddA.view(imodA);
        okA = okA && ddA.view(ilimA);
    }
    iencsA->getAxes(&jntsA);
    encsA = new yarp::sig::Vector(jntsA,0.0);

    if (!okA)
    {
        yError("[reactCtrlThread]Problems acquiring %s interfaces!!!!",part.c_str());
        return false;
    }

    yarp::os::Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());
    if (!ddT.open(OptT))
    {
        yError("[reactCtrlThread]Could not open torso PolyDriver!");
        return false;
    }

    bool okT = 1;

    if (ddT.isValid())
    {
        okT = okT && ddT.view(iencsT);
        okT = okT && ddT.view(ivelT);
        okT = okT && ddT.view(imodT);
        okT = okT && ddT.view(ilimT);
    }
    iencsT->getAxes(&jntsT);
    encsT = new yarp::sig::Vector(jntsT,0.0);

    if (!okT)
    {
        yError("[reactCtrlThread]Problems acquiring torso interfaces!!!!");
        return false;
    }

//   //**** visualizing targets and collision points in simulator ***************************
//   string port2icubsim = "/" + name + "/sim:o";
//   //cout<<port2icubsim<<endl;
//   if (!portToSimWorld.open(port2icubsim.c_str())) {
//       yError("[upperPlanner] Unable to open port << port2icubsim << endl");
//   }
//   std::string port2world = "/icubSim/world";
//   yarp::os::Network::connect(port2icubsim, port2world.c_str());

//   cmd.clear();
//   cmd.addString("world");
//   cmd.addString("del");
//   cmd.addString("all");
//   portToSimWorld.write(cmd);

   T_world_root = zeros(4,4);
   T_world_root(0,1)=-1;
   T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
   T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
   T_world_root(3,3)=1;
   T_root_world=SE3inv(T_world_root);


   //****RPC Port ************************************************************************
   rpcSrvr.open(("/"+name+"/rpc:i").c_str());
//   attach(rpcSrvr); // Cannot work withooouuuu thrift

//   rpcDataProcessor processor;
//   rpcSrvr.setReader(processor);

   //****Normal communication port *******************************************************
   ts.update();
   upperTrajectPortOut.open(("/"+name+"/bestCartesianTrajectory:o").c_str());


   //****Planning*************************************************************************
   replan = true;   // Remember to clear after running planner
   planningTime = 1.0;
   bestTrajEE.clear();
   bestTrajRootEE.clear();
   bestTrajElbow.clear();
   bestTrajRootElbow.clear();

   return true;
}

bool upperPlanner::attach(RpcServer &source)
{
//    return this->yarp().attachAsServer(source);
}

double upperPlanner::getPeriod()
{
    return 1.0;
}

bool upperPlanner::updateModule()
{
    processRpcCommand();


    // Check if required re-plan and do if necessary; remember to clear "replan" after running
    if (replan==true)
    {
        singlePlanner plannerEE(verbosity,name,"End-effector");
        singlePlanner plannerElbow(verbosity,name, "Elbow");
        // 1.Reading message of Trajectory through port

        // 2.Perception part: obtaining information of Target, Workspace, Obstacles
        // Information should be in <root> FoR


        // Test=======================================================
        Vector workspace(6,0.0);
        workspace[3]=8.0/10.0;
        workspace[4]=20.0/10.0;
        workspace[5]=8.0/10.0;

        double scale = 10.0;
        double sizeObject = 1.0/scale;  //1.0/10.0;
        double sizeGoal = .15; //1.4/scale;

        Vector goal(6,sizeGoal);
        goal[0]=-1.0/scale; //-3.0/scale; //-10.0/scale;
        goal[1]=5.64/scale; //6.0/scale; //7.0/scale;
        goal[2]=3.7/scale;  //1.0/scale;

        target = goal;

        srand(time(NULL));
//        vector<Vector> obsSet;
        obsSet.clear();
        for (int i=0; i<=10; i++)
        {
            Vector obs(6, sizeObject);
            obs[0] = (double)(rand()%8-4)/scale;
            //obs[1] = 0.564;
            obs[1] = 0.614;
            obs[4] = 0.2;
            obs[2] = (double)(rand()%4+3)/scale;
            if ((abs(10*obs[0]-10*goal[0])>=abs(10*obs[3]-10*goal[3]))&&
                    (abs(10*obs[2]-10*goal[2])>=abs(10*obs[5]-10*goal[5]))&&
                    (abs(10*obs[2]-10*goal[2])>=abs(10*obs[5]-10*goal[5])))
            {
                obsSet.push_back(obs);
            }

        }
        //============================================================
        plannerEE.setRegionOperating(workspace);
        plannerEE.setGoal(goal);
        plannerEE.setDeadline(planningTime);
        plannerEE.setObstacles(obsSet);

        // 3.Manipulator position
        Vector xCur(3,0.0), startPose(3,0.0), xElbow(3,0.0), startPoseElbow(3,0.0);

        updateArmChain();
        iKinChain &chain = *arm->asChain();
        xCur = chain.EndEffPosition();

        xElbow = chain.Position(indexElbow);

        printf("useTorso: %d \n", useTorso);
        printf("EE Position: %f, %f, %f\n",xCur[0], xCur[1], xCur[2]);
        printf("Elbow (link %d-th) Position: %f, %f, %f\n",indexElbow, xElbow[0], xElbow[1], xElbow[2]);

//
//        for (int i=0; i<10; i++)
//        {
//            xElbow = chain.Position(i);
//            printf("Link %d-th Position: %f, %f, %f\n",i,xElbow[0], xElbow[1], xElbow[2]);
//        }

        convertPosFromRootToSimFoR(xCur,startPose); // This test using Objects in World of Sim
        printf("startPose: %f, %f, %f\n",startPose[0], startPose[1], startPose[2]);
        plannerEE.setStart(startPose);

        // 4.Planning
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf("!!!                      !!!\n");
        printf("!!! END-EFFECTOR PLANNER !!!\n");
        printf("!!!                      !!!\n");
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        plannerEE.executeTrajectory(bestTrajEE, bestTrajRootEE, "blue");

        // 4b. Planning for Elbow
        double sizeGoalElbow = 2*lForearm;
        Vector goalElbow(6,sizeGoalElbow);
        int indexLastTrajEE = bestTrajEE.size()-1;
        for (int i=0; i<nDim; i++)
            goalElbow[i] = bestTrajEE[indexLastTrajEE][i];

        printf("!!!!!!!!!!!!!!!!!!!!!\n");
        printf("!!!               !!!\n");
        printf("!!! ELBOW PLANNER !!!\n");
        printf("!!!               !!!\n");
        printf("!!!!!!!!!!!!!!!!!!!!!\n");

//        Vector workspace(6,0.0);
        workspace[3]=.8-lForearm;
        workspace[4]=20.0/10.0;
        workspace[5]=.8-lForearm;

        plannerElbow.setRegionOperating(workspace);
        plannerElbow.setGoal(goalElbow);
        plannerElbow.setDeadline(planningTime);
        plannerElbow.setObstacles(obsSet);

        convertPosFromRootToSimFoR(xElbow,startPoseElbow);
        plannerElbow.setStart(startPoseElbow);

        plannerElbow.executeTrajectory(bestTrajElbow,bestTrajRootElbow, "yellow");

        // 5.Sending message of Trajectory through port
        sendTrajectory();

        // 6.Clearing all planners to make sure they won't effect the next run
        bestTrajEE.clear();
        bestTrajRootEE.clear();

        bestTrajElbow.clear();
        bestTrajRootElbow.clear();

        replan = false;
    }

    return true;
}

bool upperPlanner::respond(const Bottle &command, Bottle &reply)
{
    cout<<"Got something, echo is on"<< endl;
    if (command.get(0).asString()=="re_plan")
    {
        reply.addString("starting");
        replan = true;
//        return true;
    }
    else
    {
        reply.addString("waiting...");
        replan = false;
    }
    return true;
}


bool upperPlanner::close()
{
    printf("upperPlanner: stopping...\n");

    delete encsA; encsA = NULL;
    delete encsT; encsT = NULL;
    delete   arm;   arm = NULL;
    // Close any opened ports

    upperTrajectPortOut.interrupt();
    upperTrajectPortOut.close();
    rpcSrvr.interrupt();
    rpcSrvr.close();
    printf("Done, goodbye from upperPlanner\n");
    return true;
}

void upperPlanner::updateArmChain()
{
    iencsA->getEncoders(encsA->data());
    Vector qA=encsA->subVector(0,6);

    if (useTorso)
    {
        iencsT->getEncoders(encsT->data());
        Vector qT(3,0.0);
        qT[0]=(*encsT)[2];
        qT[1]=(*encsT)[1];
        qT[2]=(*encsT)[0];

        Vector q(10,0.0);
        q.setSubvector(0,qT);
        q.setSubvector(3,qA);
        arm->setAng(q*CTRL_DEG2RAD);

        indexElbow = 6;
    }
    else
    {
        arm->setAng(qA*CTRL_DEG2RAD);

        indexElbow = 3;
    }


}

void upperPlanner::processRpcCommand()
{
    Bottle in, out;
    rpcSrvr.read(in,true);
    // Process data
    if (in.get(0).asString()=="re_plan")
    {
        out.addString("restarting_planner with time constraint of");
        replan = true;
        if (in.size()==2)
            planningTime = in.get(1).asDouble();
        else
            planningTime = 1.0;
        out.addDouble(planningTime);
        out.addString("seconds!!!");
    }
    else
    {
        out.addString("waiting...");
        replan = false;
    }
    rpcSrvr.reply(out);

}



void upperPlanner::sendTrajectory()
{
    Bottle &outTraj = upperTrajectPortOut.prepare();
    outTraj.clear();
    Bottle traj, viaPoint;
    traj.clear();
    viaPoint.clear();

    ts.update();

    traj.addInt(bestTrajRootEE.size());
    traj.addInt(nDim);
    for (int j=0; j<bestTrajRootEE.size(); j++)
    {
        viaPoint.clear();
        for (int i=0; i<nDim; i++)
            viaPoint.addDouble(bestTrajRootEE[j][i]);
        traj.append(viaPoint);
    }
    outTraj.addList().read(traj);
    upperTrajectPortOut.setEnvelope(ts);
    upperTrajectPortOut.write();
}

void upperPlanner::convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos)
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

void upperPlanner::convertPosFromSimToRootFoR(const Vector &pos, Vector &outPos)
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

//void upperPlanner::setIteration(const int &nIteration)
//{
//  nIter = nIteration;
//}

//void upperPlanner::setDeadline(const double &_deadline)
//{
//  deadline = _deadline;
//}

//void upperPlanner::setRegionOperating(const Vector &regionOperating)
//{
//  system.regionOperating.setNumDimensions(nDim);
//  for (int i=0; i<nDim; i++)
//    {
//      system.regionOperating.center[i]=regionOperating[i];
//      system.regionOperating.size[i]=regionOperating[i+3];
//    }
//}

//void upperPlanner::setGoal(const Vector &goal)
//{
//  system.regionGoal.setNumDimensions(nDim);
//  for (int i=0; i<nDim; i++)
//    {
//      system.regionGoal.center[i]=goal[i];
//      system.regionGoal.size[i] = goal[i+3];
//    }
//  //target = new Vector(2*nDim,0.0);
//  target = goal;
//}

//void upperPlanner::setStart(const Vector &start)
//{
//  rrts.setSystem(system);

//  vertex_t &root = rrts.getRootVertex();
//  State &rootState = root.getState();
//  for (int i=0; i<nDim; i++)
//    {
//      rootState[i] = start[i];
//    }
//  startPos = start;
//}

//void upperPlanner::setObstacles(const std::vector<Vector> &obstacles)
//{
//  //obsSet.resize(obstacles.size());
//  region *obstacle;
//  for(int j=0; j<obstacles.size(); j++)
//    {
//      obstacle = new region;
//      obstacle->setNumDimensions(nDim);
//      Vector obs(2*nDim,0.0);
//      for (int i=0; i<nDim; i++)
//        {
//          obstacle->center[i] = obstacles[j][i];
//          obstacle->size[i] = obstacles[j][i+3];
//          obs[i]=obstacles[j][i];
//          obs[i+3]=obstacles[j][i+3];
//        }
//      system.obstacles.push_front(obstacle);
//      obsSet.push_back(obs);
//    }
////  cout<<"data in obsSet "<<endl;
////  for (int j=0; j<obsSet.size(); j++)
////    {
////      printf("Obstacle [%i] = [%g,%g,%g,%g,%g,%g]\n",
////         j, obsSet[j][0], obsSet[j][1], obsSet[j][2],
////      obsSet[j][3], obsSet[j][4], obsSet[j][5]);
////    }
////    cout<<endl;
////  rrts.setSystem(system);
//}

//void upperPlanner::updatePlanner()
//{
//  rrts.initialize ();

//  // This parameter should be larger than 1.5 for asymptotic
//  //   optimality. Larger values will weigh on optimization
//  //   rather than exploration in the RRT* algorithm. Lower
//  //   values, such as 0.1, should recover the RRT.
//  rrts.setGamma (1.5);
//}

//vector<Vector> upperPlanner::generateTrajectory()
////void upperPlanner::generateTrajectory(vector<Vector> newBest)
//{
//    Vector vertexBest(nDim,0.0);
//    Vector vertexBestRoot(nDim,0.0);
////    int stateIndex = 0;
//    list<double*> traject;


//    clock_t start = clock();
//    //Time startTime = Time::now();
//    //*******************************************************************//
//    // Main loop to generate a plan
//    for (int i = 0; i < nIter; i++)
//    {
//        rrts.iteration ();

//        // added on 01/03/2016 for testin time deadline of planner
//        clock_t finish = clock();
//        //Time finishTime = Time::now();
//        solveTime = ((double)(finish-start))/CLOCKS_PER_SEC;
//        if (solveTime>=deadline)
//          break;
//        // end of added part
//    }
//    //*******************************************************************//

//    // commented on 01/03/2016 for testing time deadline of planner
////    clock_t finish = clock();
////    //Time finishTime = Time::now();
////    solveTime = ((double)(finish-start))/CLOCKS_PER_SEC;
//    // end of commented part

//    //solveTime1 = ((finishTime-startTime))/CLOCKS_PER_SEC;
////    cout << "Time inside : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;


//    rrts.getBestTrajectory(traject);
//    for (list<double*>::const_iterator itt = traject.begin(); itt!=traject.end();itt++)
//    {
//        double* stateRef = *itt;
//        for (int i=0; i<nDim; i++)
//          {
//            vertexBest[i] = stateRef[i];
//          }
//        bestTraj.push_back(vertexBest);
//        convertPosFromSimToRootFoR(vertexBest,vertexBestRoot);
//        bestTrajRoot.push_back(vertexBestRoot);
////        cout<<"state ["<<stateIndex<<"]: "<<stateRef[0]<<","<<stateRef[1]<<","<<stateRef[2]<<endl;
////        stateIndex++;
//    }
//    // Add the goal state to bestTraj
////    vertexBest = target.subVector(0,2);
////    convertPosFromSimToRootFoR(vertexBest,vertexBestRoot);
////    bestTraj.push_back(vertexBest);
////    bestTrajRoot.push_back(vertexBestRoot);

//    cout<<"\nTrajectory is generated in "<<solveTime<<" seconds !!!"<< endl;
//    cout<<endl;
//    return bestTraj;
//}

//vector<Vector> upperPlanner::getBestTrajRoot()
//{
//  return bestTrajRoot;
//}

//void upperPlanner::executeTrajectory()
//{

//    updatePlanner();
//    generateTrajectory();
//    getBestTrajRoot();
//    printTrajectory();
//    displayPlan();
//    logTrajectory();
//}

//void upperPlanner::printTrajectory()
//{
//    vertex_t &root = rrts.getRootVertex();
//    State &rootState = root.getState();
//    printf("===============================\n");
//    printf("PLAN PROBLEM\n");
//    cout<<"Workspace center: "<<rrts.system->regionOperating.center[0]<<","<<rrts.system->regionOperating.center[1]<<","<<rrts.system->regionOperating.center[2]<<endl;
//    cout<<"Workspace size: "<<rrts.system->regionOperating.size[0]<<","<<rrts.system->regionOperating.size[1]<<","<<rrts.system->regionOperating.size[2]<<endl;
//    cout<<"Start: "<<rootState[0]<<","<<rootState[1]<<","<<rootState[2]<<endl;
//    cout<<"Target center: "<<rrts.system->regionGoal.center[0]<<","<<rrts.system->regionGoal.center[1]<<","<<rrts.system->regionGoal.center[2]<<endl;
//    cout<<"Target size: "<<rrts.system->regionGoal.size[0]<<","<<rrts.system->regionGoal.size[1]<<","<<rrts.system->regionGoal.size[2]<<endl;
//    int nObs = 0;
//    list<region*> obs = rrts.system->obstacles;
//    for (list<region*>::iterator obsItt = obs.begin(); obsItt!=obs.end(); obsItt++)
//      {
//        region* obsState = *obsItt;
//        cout<<"obstacle ["<<nObs++<<"]: "<<obsState->center[0]<<","<<obsState->center[1]<<","<<obsState->center[2];
//        cout<<"; size: "<<obsState->size[0]<<","<<obsState->size[1]<<","<<obsState->size[2]<<endl;
//      }
//    printf("\n===============================\n");
//    printf("PLANNER RESULTS\n");

//    int nVertices = rrts.numVertices;
//    cout<<"Numbers of vertics: "<< nVertices<<endl;

//    vertex_t& vertexBest = rrts.getBestVertex ();
//    if (&vertexBest == NULL) {
//        cout << "No best vertex" << endl;
//        //return 0;
//    }
//    else
//      {
//        cout << "There is a list of best vertices" <<endl;

//      }
//    for (int j=0; j<bestTraj.size(); j++)
//      //for (int i=0; i<nDim; i++)
//      {
//        printf("vertex [%i] = [%g,%g,%g]\n",
//               j, bestTraj[j][0], bestTraj[j][1], bestTraj[j][2]);
//      }

//    for (int j=0; j<bestTrajRoot.size(); j++)
//      //for (int i=0; i<nDim; i++)
//      {
//        printf("vertex in root [%i] = [%g,%g,%g]\n",
//               j, bestTrajRoot[j][0], bestTrajRoot[j][1], bestTrajRoot[j][2]);
//      }

//}

//void upperPlanner::logTrajectory()
//{
//    ofstream logfile("workspacePlanner.txt");
//    ofstream logfile1("resultPlanner.txt");

//    if (logfile.is_open())
//      {
//        logfile<< "Workspace\t";
//        logfile<< rrts.system->regionOperating.center[0] <<"\t";
//        logfile<< rrts.system->regionOperating.center[1] <<"\t";
//        logfile<< rrts.system->regionOperating.center[2] <<"\t";
//        //logfile<< "Space_size\t";
//        logfile<< rrts.system->regionOperating.size[0] <<"\t";
//        logfile<< rrts.system->regionOperating.size[1] <<"\t";
//        logfile<< rrts.system->regionOperating.size[2] <<"\n";
//        logfile<< "Target\t";
//        logfile<< rrts.system->regionGoal.center[0]<<"\t";
//        logfile<< rrts.system->regionGoal.center[1]<<"\t";
//        logfile<< rrts.system->regionGoal.center[2]<<"\t";
//        //logfile<< "Target_size\t";
//        logfile<< rrts.system->regionGoal.size[0]<<"\t";
//        logfile<< rrts.system->regionGoal.size[1]<<"\t";
//        logfile<< rrts.system->regionGoal.size[2]<<"\n";
//        logfile<< "Start\t";
//        logfile<< startPos[0]<<"\t";
//        logfile<< startPos[1]<<"\t";
//        logfile<< startPos[2]<<"\t";
//        logfile<< "0" <<"\t";
//        logfile<< "0" <<"\t";
//        logfile<< "0" <<"\n";
//        //logfile<< "==============================="<<"\n";
//        for (int j=0; j<obsSet.size(); j++)
//          {
//            logfile<< "Obs"<<j<<"\t";
//            for (int i=0; i<2*nDim; i++)
//              logfile<< obsSet[j][i] <<"\t";
//            logfile<<"\n";
//          }
//      }
//    else
//      cerr << "File not open!"<< endl;

//    if (logfile1.is_open())
//      {
//        logfile1<<"Solving_time:\t"<<solveTime<<"\n";
//        for (int j=0; j<bestTraj.size(); j++)
//          {
//            logfile1<< "Way-point"<<j<<"\t";
//            for (int i=0; i<nDim; i++)
//              logfile1<< bestTraj[j][i] <<"\t";
//            logfile1<<"\n";
//          }

//      }
//}

//void upperPlanner::displayPlan()
//{
//    printf("\n===============================\n");
//    cout<<"DISPLAY PLAN"<<endl;
//    for (int j=1; j<obsSet.size(); j++)
//      {
//        createStaticBox(obsSet[j],"obstacle");
//      }
//    for (int i=0; i<bestTrajEE.size(); i++)
//      {
//        createStaticSphere(0.05, bestTrajEE[i]);
//      }
//    createStaticBox(target,"goal");
//}

//void upperPlanner::createStaticSphere(double radius, const Vector &pos)
//{
//    cmd.clear();
//    cmd.addString("world");
//    cmd.addString("mk");
//    cmd.addString("ssph");
//    cmd.addDouble(radius);

//    cmd.addDouble(pos(0));
//    cmd.addDouble(pos(1));
//    cmd.addDouble(pos(2));
//    // color
//    cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
//    cmd.addString("false"); //no collisions
//    printMessage(5,"createSphere(): sending %s \n",cmd.toString().c_str());
//    portToSimWorld.write(cmd);
//}

//void upperPlanner::moveSphere(int index, const Vector &pos)
//{
//    cmd.clear();
//    cmd.addString("world");
//    cmd.addString("set");
//    cmd.addString("ssph");
//    cmd.addInt(index);
//    cmd.addDouble(pos(0));
//    cmd.addDouble(pos(1));
//    cmd.addDouble(pos(2));
//    portToSimWorld.write(cmd);
//}

//void upperPlanner::createStaticBox(const Vector &pos, const string &type)
//{
//    cmd.clear();
//    cmd.addString("world");
//    cmd.addString("mk");
//    cmd.addString("sbox");
//    cmd.addDouble(pos(3)); cmd.addDouble(pos(4)); cmd.addDouble(pos(5)); //fixed size

//    cmd.addDouble(pos(0));
//    cmd.addDouble(pos(1));
//    cmd.addDouble(pos(2));
//    // color
//    if (type == "obstacle")
//      {
//        cmd.addInt(1);cmd.addInt(0);cmd.addInt(0); //red
////        cmd.addString("true"); //collisions
//      }
//    else if (type == "goal")
//      {
//        cmd.addInt(0);cmd.addInt(1);cmd.addInt(0); //green
////        cmd.addString("false"); //no collisions
//      }
//    cmd.addString("false"); //no collisions
//    printMessage(5,"createBox(): sending %s \n",cmd.toString().c_str());
//    portToSimWorld.write(cmd);
//}

//void upperPlanner::moveBox(int index, const Vector &pos)
//{
//    cmd.clear();
//    cmd.addString("world");
//    cmd.addString("set");
//    cmd.addString("sbox");
//    cmd.addInt(index);
//    cmd.addDouble(pos(0));
//    cmd.addDouble(pos(1));
//    cmd.addDouble(pos(2));
//    portToSimWorld.write(cmd);
//}

// empty line to make gcc happy
