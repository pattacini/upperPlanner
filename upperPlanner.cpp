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
    name        ="reaching-planner";
    part        =        "left_arm";
    running_mode=          "single";
    verbosity   =                 0;
    disableTorso=             false;
    maxReplan   =              1000;
    replan      =             false;

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

    printf("reaching-planner: starting...\n");
    //******************* NAME ******************
    if (rf.check("name"))
    {
        name = rf.find("name").asString();
        yInfo("[reaching-planner] Module name set to %s", name.c_str());
    }
    else yInfo("[reaching-planner] Module name set to default, i.e. %s", name.c_str());
    setName(name.c_str());

    //******************* ROBOT ******************
    if (rf.check("robot"))
    {
        robot = rf.find("robot").asString();
        yInfo("[reaching-planner] Robot is: %s", robot.c_str());
    }
    else yInfo("[reaching-planner] Could not find robot option in the config file; using %s as default",robot.c_str());

    //******************* VERBOSE ******************
    if (rf.check("verbosity"))
    {
        verbosity = rf.find("verbosity").asInt();
        yInfo("[reaching-planner] verbosity set to %i", verbosity);
    }
    else yInfo("[reaching-planner] Could not find verbosity option in the config file; using %i as default",verbosity);

    //******************* PART ******************
    if (rf.check("part"))
    {
        part = rf.find("part").asString();
        part_short = part;
        if (part=="left")
        {
            part="left_arm";
        }
        else if (part=="right")
        {
            part="right_arm";
        }
        else if (part!="left_arm" && part!="right_arm")
        {
            part="left_arm";
            yWarning("[reaching-planner] part was not in the admissible values. Using %s as default.",part.c_str());
        }
        yInfo("[reaching-planner] part to use is: %s", part.c_str());
    }
    else yInfo("[reaching-planner] Could not find part option in the config file; using %s as default",part.c_str());
    //********************** MODE ***********************
    if (rf.check("running_mode"))
    {
        running_mode = rf.find("running_mode").asString();
        yInfo("[reaching-planner] running_mode set to %s", running_mode.c_str());
    }
    else yInfo("[reaching-planner] Could not find running_mode option in the config file; using %s as default",running_mode.c_str());
    //********************** NUMBER OF REPLAN ***********************
    if (rf.check("maxReplan"))
    {
        maxReplan = rf.find("maxReplan").asInt();
        yInfo("[reaching-planner] maxReplan set to %i", maxReplan);
    }
    else yInfo("[reaching-planner] Could not find maxReplan option in the config file; using %i as default",maxReplan);
    //********************** CONFIGS ***********************
    if (rf.check("disableTorso"))
    {
        if(rf.find("disableTorso").asString()=="on"){
            disableTorso = true;
            useTorso = false;
            yInfo("[reaching-planner] disableTorso flag set to on.");
        }
        else{
            disableTorso = false;
            useTorso = true;
            yInfo("[reaching-planner] disableTorso flag set to off.");
        }
    }
    else
    {
         yInfo("[reaching-planner] Could not find disableTorso flag (on/off) in the config file; using %d as default",disableTorso);
    }
    //********************** Visualizations in simulator ***********************
   if (robot == "icubSim"){
       if (rf.check("visualizeObjectsInSim"))
       {
           if(rf.find("visualizeObjectstInSim").asString()=="on"){
               visualizeObjectsInSim = true;
               yInfo("[reaching-planner] visualizeObjectsInSim flag set to on.");
           }
           else{
               visualizeObjectsInSim = false;
               yInfo("[reaching-planner] visualizeObjectsInSim flag set to off.");
           }
       }
       else
       {
           yInfo("[reaching-planner] Could not find visualizeObjectsInSim flag (on/off) in the config file; using %d as default",visualizeObjectsInSim);
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
        yError("[reaching-planner]Could not open %s PolyDriver!",part.c_str());
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
        yError("[reaching-planner]Problems acquiring %s interfaces!!!!",part.c_str());
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
        yError("[reaching-planner]Could not open torso PolyDriver!");
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

    //**** visualizing targets and collision points in simulator ***************************
    string port2icubsim = "/" + name + "/sim:o";
    //cout<<port2icubsim<<endl;
    if (!portToSimWorld.open(port2icubsim.c_str())) {
       yError("[upperPlanner] Unable to open port << port2icubsim << endl");
    }
    std::string port2world = "/icubSim/world";
    yarp::os::Network::connect(port2icubsim, port2world.c_str());

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
    attach(rpcSrvr);

    //   rpcDataProcessor processor;
    //   rpcSrvr.setReader(processor);

    //****Normal communication port *******************************************************
    ts.update();
    //   upperTrajectPortOut.open(("/"+name+"/bestCartesianTrajectory:o").c_str());

    EEPortOut.setControlPoint("End-Effector");
    EEPortOut.open(("/"+name+"/End-Effector/bestCartesianTrajectory:o").c_str());

    HalfElbowPortOut.setControlPoint("Half-Elbow");
    HalfElbowPortOut.open(("/"+name+"/Half-Elbow/bestCartesianTrajectory:o").c_str());

    //****Planning*************************************************************************
    if (running_mode == "batch")
    {
        replan = true;   // Remember to clear after running planner
        planningTime = .10;
        initBatchSummary();
    }
    else if (running_mode == "sigle")
    {
        replan = false;
        planningTime = 1.0;
    }

    bestTrajEE.clear();
    bestTrajRootEE.clear();
    bestTrajElbow.clear();
    bestTrajRootElbow.clear();
    bestTrajHalfElbow.clear();
    bestTrajRootHalfElbow.clear();

    workspaceRoot.resize(6);        // Root frame
    workspaceRoot[0] = -0.024500;   // x coordinate
    workspaceRoot[1] = -0.074500;   // y coordinate
    workspaceRoot[2] = 0.051000;    // z coordinate
    workspaceRoot[3] = 1.05100;     // size of x coordinate
    workspaceRoot[4] = 0.84900;     // size of y coordinate
    workspaceRoot[5] = 1.00000;     // size of z coordinate

    workspace.resize(6);        // World frame
    workspace[0] = 0.074500;   // x coordinate
    workspace[1] = 0.051000;   // y coordinate
    workspace[2] = 0.024500;    // z coordinate
    workspace[3] = 0.84900;     // size of x coordinate
    workspace[4] = 1.00000+1;     // size of y coordinate
    workspace[5] = 1.05100;     // size of z coordinate

    countReplan = 0;

    return true;
}

//bool upperPlanner::attach(RpcServer &source)
//{
////    return this->yarp().attachAsServer(source);
//}

double upperPlanner::getPeriod()
{
    return 1.0;
}

bool upperPlanner::updateModule()
{
    if (running_mode == "single")
    {
//        processRpcCommand();
    }

    clock_t start = clock();


    // Check if required re-plan and do if necessary; remember to clear "replan" after running
    if (replan)
    {
        success = false;    //Batch Summary only

        singlePlanner plannerEE(verbosity,name,robot,"End-effector");
        singlePlanner plannerElbow(verbosity,name, robot, "Elbow");
        singlePlanner plannerHalfElbow(verbosity,name, robot,"Half-Elbow");


        // 1.Reading message of Trajectory through port

        // 2.Perception part: obtaining information of Target, Workspace, Obstacles
        // Information should be in <root> FoR

        double xReaching = 0.7, zReaching = 0.7;
        // Test=======================================================
//        Vector workspace(6,0.0);
//        workspace[3]= xReaching;    //8.0/10.0;
//        workspace[4]=20.0/10.0;
//        workspace[5]= zReaching;    //8.0/10.0;

        double scale = 10.0;
        double sizeObject = 1.0/scale;  //1.0/10.0;
        double sizeGoal = .15; //1.4/scale;

        Vector goal(6,sizeGoal);    // World frame for the table

        if (robot == "icubSim")
        {

            goal[0]=-1.0/scale; //-3.0/scale; //-10.0/scale;
            goal[1]=5.64/scale; //6.0/scale; //7.0/scale;
            goal[2]=3.7/scale;  //1.0/scale;

//            target = goal;
            convertPosFromSimToRootFoR(goal,target);

            srand(time(NULL));
    //        vector<Vector> obsSet;
            obsSet.clear();
            obsSetExpandedElbow.clear();
            obsSetExpandedHalfElbow.clear();
            Vector obs0(6,0.2); // robot itself in World frame
            obs0[0]=0.0;
            obs0[1]=0.5;
            obs0[2]=0.0;
            obs0[4]=1.0;

            obsSet.push_back(obs0);

            Vector obs1(6,0.5); // Table in World frame
            obs1[0] = 0.0;
            obs1[1] = 0.5;
            obs1[2] = 0.35;
            obs1[3] = 0.7;
            obs1[4] = 0.025;

            obsSet.push_back(obs1);

            for (int i=0; i<=10; i++)
            {
                Vector obs(6, sizeObject);  // World frame
                obs[0] = (double)(rand()%8-4)/scale;
                //obs[1] = 0.564;
                obs[1] = 0.614;
                obs[4] = 0.2;
                obs[2] = (double)(rand()%4+3)/scale;
                if ((abs(10*obs[0]-10*goal[0])>=abs(10*obs[3]-10*goal[3]))&&
                        (abs(10*obs[2]-10*goal[2])>=abs(10*obs[5]-10*goal[5]))&&
                        (abs(10*obs[2]-10*goal[2])>=abs(10*obs[5]-10*goal[5])))
                {
//                    convertPosFromSimToRootFoR(obs,obs);    // Root frame
                    obsSet.push_back(obs);
                }

            }
            //============================================================
        }
        plannerEE.setRegionOperating(workspace);    // World frame
        plannerEE.setGoal(goal);                  // World frame
        plannerEE.setDeadline(planningTime);        // World frame
        plannerEE.setObstacles(obsSet);             // World frame

        // 3.Manipulator position
        Vector xCur(3,0.0), startPose(3,0.0),
                xElbow(3,0.0), startPoseElbow(3,0.0),
                xHalfElbow(3,0.0), startPoseHalfElbow(3,0.0),
                xLocalHalfElbow(3,0.0), startPoseLocalHalfElbow(3,0.0);

        updateArmChain();
        iKinChain &chain = *arm->asChain();
        xCur = chain.EndEffPosition();              // Root frame
        xElbow = chain.Position(indexElbow);        // Root frame

        for (int i=0; i<nDim; i++)
            xHalfElbow[i]=(xCur[i]+xElbow[i])/2.0;// Root frame

        printf("useTorso: %d \n", useTorso);
        printf("EE Position: %f, %f, %f\n",xCur[0], xCur[1], xCur[2]);
        printf("Elbow (link %d-th) Position: %f, %f, %f\n",indexElbow, xElbow[0], xElbow[1], xElbow[2]);

        convertPosFromRootToSimFoR(xCur,startPose); // This test using Objects in World of Sim
        printf("startPose: %f, %f, %f\n",startPose[0], startPose[1], startPose[2]);
        plannerEE.setStart(startPose);              // World frame
//        plannerEE.setStart(xCur);                   // Root frame

        // 4.Planning
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf("!!!                      !!!\n");
        printf("!!! END-EFFECTOR PLANNER !!!\n");
        printf("!!!                      !!!\n");
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        plannerEE.executeTrajectory(bestTrajEE, bestTrajRootEE, "blue");

        // 4b. Planning for Half-Elbow

//        // Calculating goal region of the Elbow based on the last reaching point of the End-Effector
//        double sizeGoalElbow = 2*lForearm;
//        Vector goalElbow(6,sizeGoalElbow);
//        int indexLastTrajEE = bestTrajEE.size()-1;
//        for (int i=0; i<nDim; i++)
//            goalElbow[i] = bestTrajEE[indexLastTrajEE][i];

        if (bestTrajEE.size()>1)
        {
            bool replanLocal = true;
            xLocalHalfElbow = xHalfElbow;   // Root frame
            printf("Half Elbow Position: %f, %f, %f\n", xLocalHalfElbow[0], xLocalHalfElbow[1], xLocalHalfElbow[2]);
            Vector lastHalfElbow(3,0.0), lastRootHalfElbow(3,0.0);

            obsSetExpandedHalfElbow = expandObstacle(bestTrajEE,obsSet,lForearm/2.0);
            // Creating a Control point in the Forearm at middle of EE and Elbow

            for (int indexGoalLocal=1; indexGoalLocal< bestTrajEE.size(); indexGoalLocal++)
            {

                //singlePlanner localPlannerHalfElbow(verbosity,name,"local-Half-Elbow");
                vector<Vector> bestTrajLocalHalfElbow;
                vector<Vector> bestTrajRootLocalHalfElbow;

                double sizeGoalLocalHalfElbow = lForearm;
                Vector localGoalHalfElbow(6,sizeGoalLocalHalfElbow);
                for (int i=0; i<nDim; i++)
                    localGoalHalfElbow[i] = bestTrajEE[indexGoalLocal][i];  // World frame

//                obsSetExpandedHalfElbow = expandObstacle(bestTrajEE,obsSet,lForearm/2.0);

                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                printf("!!!                          !!!\n");
                printf("!!! LOCAL HALF-ELBOW PLANNER !!!\n");
                printf("!!!                          !!!\n");
                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

                Vector workspaceHalfElbow = workspace;
                workspaceHalfElbow[3]=workspace[3]-lForearm/2.0;
//                workspaceHalfElbow[4]=workspace[4]-lForearm/2.0;
                workspaceHalfElbow[5]=workspace[5]-lForearm/2.0;

//                singlePlanner localPlannerHalfElbow(verbosity,name,"local-Half-Elbow");
//                localPlannerHalfElbow.setRegionOperating(workspace);
//                localPlannerHalfElbow.setGoal(localGoalHalfElbow);
//                localPlannerHalfElbow.setDeadline(planningTime);
//                localPlannerHalfElbow.setObstacles(obsSet);  //Add "real" obstacle set to the planner of Elbow

//                localPlannerHalfElbow.setObstacles(obsSetExpandedHalfElbow);


//                convertPosFromRootToSimFoR(xLocalHalfElbow,startPoseLocalHalfElbow);
//                localPlannerHalfElbow.setStart(startPoseLocalHalfElbow);

//                if (replanLocal == true)
//                {




                bool replanPadWaypoint = false;
                vector<Vector> paddingWaypoints, paddingRootWaypoints;
//                while (bestTrajLocalHalfElbow.size()==0 || replanPadWaypoint)
                do
                {
                    singlePlanner localPlannerHalfElbow(verbosity,name,robot,"local-Half-Elbow");
                    localPlannerHalfElbow.setRegionOperating(workspaceHalfElbow);
                    localPlannerHalfElbow.setGoal(localGoalHalfElbow);
                    localPlannerHalfElbow.setDeadline(planningTime);
                    localPlannerHalfElbow.setObstacles(obsSet);  //Add "real" obstacle set to the planner of Elbow

                    localPlannerHalfElbow.setObstacles(obsSetExpandedHalfElbow);


                    convertPosFromRootToSimFoR(xLocalHalfElbow,startPoseLocalHalfElbow);
                    localPlannerHalfElbow.setStart(startPoseLocalHalfElbow);    // World frame (sim frame)

                    localPlannerHalfElbow.executeTrajectory(bestTrajLocalHalfElbow,bestTrajRootLocalHalfElbow, "yellow");
//                }
                    if (bestTrajLocalHalfElbow.size()==0)
                    {
                        replanPadWaypoint = true;
                        printf("===============================\n");
                        printf("Replanning cause no best local path!!!\n");
                        printf("===============================\n");
                    }
                    else
                    {
                        // Padding the End-Effector's tracjectory to have same number of waypoints as of the Half-Elbow
                        if (bestTrajLocalHalfElbow.size()>2)
                        {
                            for (int i=1; i<bestTrajLocalHalfElbow.size()-1; i++)
                            {
                                printf("===============================\n");
                                printf("Padding EE's trajectory: %d-th\n",i);
                                Vector wpj = bestTrajLocalHalfElbow[i];         // World frame
//                                Vector wp1 = startPoseLocalHalfElbow;           // World frame, of EE Trajectory
                                Vector wp1 = bestTrajEE[indexGoalLocal-1];      // World frame, of EE Trajectory
                                Vector wp2 = localGoalHalfElbow.subVector(0,2); // World frame, of EE Trajectory
                                Vector wpNew(3,0.0), wpNewRoot(3,0.0);
                                if (padWaypoint(wp1,wp2,wpj,lForearm/2.0,wpNew))
                                {
                                    // Insert "wpNew" to the trajectory of the End-Effector
                                    printf("\tPadding waypoint!!! \n");
                                    printf("\t\twpj  = %f, %f, %f\n", wpj[0], wpj[1], wpj[2]);
                                    printf("\t\twp1  = %f, %f, %f\n", wp1[0], wp1[1], wp1[2]);
                                    printf("\t\twp2  = %f, %f, %f\n", wp2[0], wp2[1], wp2[2]);
                                    printf("\t\twpNew= %f, %f, %f\n", wpNew[0], wpNew[1], wpNew[2]);

                                    convertPosFromSimToRootFoR(wpNew,wpNewRoot);
                                    paddingWaypoints.push_back(wpNew);
                                    paddingRootWaypoints.push_back(wpNewRoot);

                                    cout<<"\tCheck padding"<<endl;

//                                    vector<Vector>::iterator it, itRoot;
//                                    it = bestTrajEE.begin();
//                                    bestTrajEE.insert(it+indexGoalLocal,1,wpNew);
//                                    convertPosFromSimToRootFoR(wpNew,wpNewRoot);

//                                    itRoot = bestTrajRootEE.begin();
//                                    bestTrajRootEE.insert(itRoot+indexGoalLocal,1,wpNewRoot);

//                                    indexGoalLocal++;

                                    replanPadWaypoint = false;
                                }
                                else
                                {
                                    // replanning

                                    printf("\tReplanning for padding waypoint!!!\n");

                                    paddingWaypoints.clear();
                                    paddingRootWaypoints.clear();
                                    replanPadWaypoint = true;

                                }
                                printf("===============================\n");
                                if (replanPadWaypoint)
                                {

                                    break;
                                }
                            }

                        }
//                    bestTrajLocalHalfElbow.clear();
//                    bestTrajRootLocalHalfElbow.clear();
                    }


//                if (bestTrajRootLocalHalfElbow.size()>1)
//                {
//                    xLocalHalfElbow = bestTrajRootLocalHalfElbow[bestTrajRootLocalHalfElbow.size()-1];
//                    printf("Half Elbow Position: %f, %f, %f\n", xLocalHalfElbow[0], xLocalHalfElbow[1], xLocalHalfElbow[2]);

//                    for (int i=0; i<bestTrajLocalHalfElbow.size()-1; i++)
//                    {
//                        bestTrajHalfElbow.push_back(bestTrajLocalHalfElbow[i]);
//                        bestTrajRootHalfElbow.push_back(bestTrajRootLocalHalfElbow[i]);
//                    }
//                    lastHalfElbow = bestTrajLocalHalfElbow[bestTrajLocalHalfElbow.size()-1];
//                    lastRootHalfElbow = bestTrajRootLocalHalfElbow[bestTrajRootLocalHalfElbow.size()-1];

////                        replanLocal = false;
//                }


//                    else
//                    {
//                        replanLocal = true;
//                    }
                }
                while (replanPadWaypoint);
                if (paddingWaypoints.size()>0)
                {
                    printf("===============================\n");
                    cout<<"Insert waypoints to the End-Effector's trajectory"<<endl;
                    vector<Vector>::iterator it, itRoot;
                    it = bestTrajEE.begin();
                    bestTrajEE.insert(it+indexGoalLocal,paddingWaypoints.begin(),paddingWaypoints.end());
                    cout<<"\tCheck insert World frame"<<endl;

                    itRoot = bestTrajRootEE.begin();
                    bestTrajRootEE.insert(itRoot+indexGoalLocal,paddingRootWaypoints.begin(),paddingRootWaypoints.end());
                    cout<<"\tCheck insert Root frame"<<endl;
                    printf("===============================\n");


                    indexGoalLocal +=paddingWaypoints.size();
                }

                if (bestTrajRootLocalHalfElbow.size()>1)
                {
                    xLocalHalfElbow = bestTrajRootLocalHalfElbow[bestTrajRootLocalHalfElbow.size()-1];
                    printf("Half Elbow Position: %f, %f, %f\n", xLocalHalfElbow[0], xLocalHalfElbow[1], xLocalHalfElbow[2]);

                    for (int i=0; i<bestTrajLocalHalfElbow.size()-1; i++)
                    {
                        bestTrajHalfElbow.push_back(bestTrajLocalHalfElbow[i]);
                        bestTrajRootHalfElbow.push_back(bestTrajRootLocalHalfElbow[i]);
                    }
                    lastHalfElbow = bestTrajLocalHalfElbow[bestTrajLocalHalfElbow.size()-1];
                    lastRootHalfElbow = bestTrajRootLocalHalfElbow[bestTrajRootLocalHalfElbow.size()-1];

    //                        replanLocal = false;
                }
            }

            bestTrajHalfElbow.push_back(lastHalfElbow);
            bestTrajRootHalfElbow.push_back(lastRootHalfElbow);
            printf("Number of waypoints of End-effector 's path: %d\n", (int)bestTrajEE.size());
            printf("Number of waypoints of Half Elbow 's path: %d\n", (int)bestTrajHalfElbow.size());

            /*
//            // Creating a Control point in the Forearm at middle of EE and Elbow
//            double sizeGoalHalfElbow = lForearm;
//            Vector goalHalfElbow(6,sizeGoalHalfElbow);
//            int indexLastTrajEE = bestTrajEE.size()-1;
//            for (int i=0; i<nDim; i++)
//                goalHalfElbow[i] = bestTrajEE[indexLastTrajEE][i];

//            // Calculating expanded obstacle set for the Elbow
//    //        obsSetExpandedElbow = expandObstacle(bestTrajEE,obsSet,lForearm);

//            obsSetExpandedHalfElbow = expandObstacle(bestTrajEE,obsSet,lForearm/2.0);


//            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//            printf("!!!                    !!!\n");
//            printf("!!! HALF-ELBOW PLANNER !!!\n");
//            printf("!!!                    !!!\n");
//            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

//    //        Vector workspace(6,0.0);
//            workspace[3]=xReaching-lForearm/2.0;
//            workspace[4]=20.0/10.0;
//            workspace[5]=zReaching-lForearm/2.0;

//            plannerHalfElbow.setRegionOperating(workspace);
//            plannerHalfElbow.setGoal(goalHalfElbow);
//            plannerHalfElbow.setDeadline(planningTime);
//            plannerHalfElbow.setObstacles(obsSet);  //Add "real" obstacle set to the planner of Elbow

//            plannerHalfElbow.setObstacles(obsSetExpandedHalfElbow);


//            convertPosFromRootToSimFoR(xHalfElbow,startPoseHalfElbow);
//            plannerHalfElbow.setStart(startPoseHalfElbow);

//            plannerHalfElbow.executeTrajectory(bestTrajHalfElbow,bestTrajRootHalfElbow, "yellow");

//            // 4.c Planning for Elbow

//            if (bestTrajHalfElbow.size()>0)
//            {
//                // Calculating goal region of the Elbow based on the last reaching point of the End-Effector
//                double sizeGoalElbow = lForearm;
//                Vector goalElbow(6,sizeGoalElbow);
//                int indexLastTrajHalfElbow = bestTrajHalfElbow.size()-1;
//                for (int i=0; i<nDim; i++)
//                    goalElbow[i] = bestTrajHalfElbow[indexLastTrajHalfElbow][i];

//                obsSetExpandedElbow = expandObstacle(bestTrajHalfElbow,obsSet,lForearm/2.0);
//                obsSetExpandedElbow_fromHalf = expandObstacle(bestTrajHalfElbow,obsSetExpandedHalfElbow,lForearm/2.0);

//                printf("!!!!!!!!!!!!!!!!!!!!!\n");
//                printf("!!!               !!!\n");
//                printf("!!! ELBOW PLANNER !!!\n");
//                printf("!!!               !!!\n");
//                printf("!!!!!!!!!!!!!!!!!!!!!\n");

////                Vector workspace(6,0.0);
//                workspace[3]=xReaching-lForearm;
//                workspace[4]=20.0/10.0;
//                workspace[5]=zReaching-lForearm;

//                plannerElbow.setRegionOperating(workspace);
//                plannerElbow.setGoal(goalElbow);
//                plannerElbow.setDeadline(planningTime);
//                plannerElbow.setObstacles(obsSet);  //Add "real" obstacle set to the planner of Elbow

//                plannerElbow.setObstacles(obsSetExpandedElbow);
//                plannerElbow.setObstacles(obsSetExpandedElbow_fromHalf);

//                convertPosFromRootToSimFoR(xElbow,startPoseElbow);
//                plannerElbow.setStart(startPoseElbow);

//                plannerElbow.executeTrajectory(bestTrajElbow,bestTrajRootElbow, "magenta");

            */
            // 5.Sending message of Trajectory through port
            EEPortOut.setTrajectory(bestTrajRootEE);
            EEPortOut.sendTrajectory();

            HalfElbowPortOut.setTrajectory(bestTrajRootHalfElbow);
            HalfElbowPortOut.sendTrajectory();

            // 6. Display Trajectory
            if (running_mode == "single")
            {
                displayTraj(bestTrajEE,"blue");
                displayTraj(bestTrajHalfElbow,"yellow");
            }
        }
        else
        {
            // Replan
        }

        clock_t finish = clock();

        solvingTime = ((double)(finish-start))/CLOCKS_PER_SEC;
\
        printf("===============================\n");
        printf("SUMMARY\n");

        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf("!!!  Solving time : %f  !!!\n", solvingTime);
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

        if (bestTrajEE.size() == bestTrajHalfElbow.size())
        {
            bestTrajElbow.clear();
            printf("Distance of waypoint in trajectory of EE and Half-elbow are: \n");
            for (int i=0; i<bestTrajEE.size(); i++)
            {
                printf ("\twaypoint %d: %f \n", i, distWpWp(bestTrajEE[i],bestTrajHalfElbow[i]));

                Vector elbow(3,0.0);
                elbow = findOtherEndPoint(bestTrajEE[i],bestTrajHalfElbow[i],lForearm);
                bestTrajElbow.push_back(elbow);

            }


            printf("Length of Half Forearm: %f \n", lForearm/2.0);
            if (bestTrajEE.size()!=0)
                success = true;

//            displayTraj(bestTrajElbow,"purple");
            for (int i=0; i<bestTrajElbow.size(); i++)
            {
                if (collisionCheck(bestTrajElbow[i],obsSet))
                {
                    success = false;
                    printf("Elbow %d collided\n", i);
                }
            }
        }

//        logTrajectory(name, solvingTime);

        // Batch operation for statistic purpose
        if (running_mode == "batch")
        {
            countReplan++;

            if(countReplan>=maxReplan)
            {
                replan = false;
                running_mode = "single";
                countReplan = 0;
            }
            else
                replan = true;
            logBatchSummary();
        }
        else if (running_mode== "single")
        {
            logTrajectory(name, solvingTime);
            plannerEE.logVertices();
            displayTraj(bestTrajElbow,"purple");
            replan = false;
        }

        // 7.Clearing all planners to make sure they won't effect the next run


        bestTrajEE.clear();
        bestTrajRootEE.clear();

        bestTrajElbow.clear();
        bestTrajRootElbow.clear();

        bestTrajHalfElbow.clear();
        bestTrajRootHalfElbow.clear();

        printf("THE END %d\n", countReplan);
        printf("===============================\n");


    }
    return true;
}

bool upperPlanner::respond(const Bottle &command, Bottle &reply)
{
    cout<<"Got something, echo is on"<< endl;
    if (command.get(0).asString()=="replan")
    {
        reply.addString("Starting planner with deadline of ");
        replan = true;
        if (command.size()==2)
            planningTime = command.get(1).asDouble();
        else
            planningTime = 1.0;
        reply.addDouble(planningTime);
        reply.addString("seconds!!!");
    }
    else
    {
        reply.addString("Command does not exist. Waiting a command...");
        replan = false;
    }
    return true;
}


bool upperPlanner::close()
{
    printf("[reaching-planner]: stopping...\n");

    delete encsA; encsA = NULL;
    delete encsT; encsT = NULL;
    delete   arm;   arm = NULL;
    // Close any opened ports

    upperTrajectPortOut.interrupt();
    upperTrajectPortOut.close();
    rpcSrvr.interrupt();
    rpcSrvr.close();
    EEPortOut.interrupt();
    EEPortOut.close();
    HalfElbowPortOut.interrupt();
    HalfElbowPortOut.close();

    if (arm)
    {
        delete arm;
        arm=0;
    }

    printf("Done, goodbye from [reaching-planner]\n");
    return true;
}

void upperPlanner::initBatchSummary()
{
    ofstream logfile1("batchSummary.txt",ofstream::out | ofstream::trunc);
    if (logfile1.is_open())
    {
        logfile1<<"countReplan"<<"\t";
        logfile1<<"solvingTime"<<"\t";
        logfile1<<"waypoints"<<"\t";
        logfile1<<"success"<<"\n";


    }
}

void upperPlanner::logBatchSummary()
{
    ofstream logfile1("batchSummary.txt",ofstream::out | ofstream::app);

    if (logfile1.is_open())
    {
        logfile1<<countReplan<<"\t";
        logfile1<<solvingTime<<"\t";
        logfile1<<bestTrajEE.size()<<"\t";
        logfile1<<success<<"\n";


    }
//    logfile1.close();
}

bool upperPlanner::padWaypoint(const Vector &point1, const Vector &point2,
                               const Vector &pointj, const double &lengthLimb,
                               Vector &newPoint)
{
    Vector v(nDim,0.0), a(nDim,0.0);
    Vector Xt1(nDim,0.0), Xt2(nDim,0.0);
    double t1,t2;
    double coefA, coefB, coefC, delta;
    double eps = 0.001;

    bool result = false;

    for (int i=0; i<nDim; i++)
    {
        v[i]=point2[i]-point1[i];
        a[i]=pointj[i]-point1[i];
    }
    coefA = pow(v[0],2.0) + pow(v[1],2.0) + pow(v[2],2.0);
    coefB = -2*(a[0]*v[0] + a[1]*v[1] + a[2]*v[2]);
    coefC = pow(a[0],2.0) + pow(a[1],2.0) + pow(a[2],2.0) - pow(lengthLimb,2.0);

    delta = pow(coefB,2.0)-4*coefA*coefC;

    if (delta>=0)
    {
        t1 = (-coefB+sqrt(delta))/(2*coefA);
        t2 = (-coefB-sqrt(delta))/(2*coefA);
        for (int i=0; i<nDim; i++)
        {
            Xt1[i] = point1[i] + v[i]*t1;
            Xt2[i] = point1[i] + v[i]*t2;
        }
        double dist_Xt1_point1 = distWpWp(Xt1,point1);
        double dist_Xt1_point2 = distWpWp(Xt1,point2);
        double dist_Xt2_point1 = distWpWp(Xt2,point1);
        double dist_Xt2_point2 = distWpWp(Xt2,point2);
        double dist_point1_point2 = distWpWp(point1,point2);

        if((abs(10*(dist_Xt1_point1+dist_Xt1_point2-dist_point1_point2))/10.0)<=eps)
        {
            newPoint = Xt1;
            result = true;
        }

        if((abs(10*(dist_Xt2_point1+dist_Xt2_point2-dist_point1_point2))/10.0)<=eps)
        {
            newPoint = Xt2;
            result = true;
        }
    }

    return result;

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
        out.addString("restarting planner with time constraint of");
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

vector<Vector> findCorner(const Vector &obstacle)
{
    vector<Vector> vertices;
    Vector vertice(3,0.0);
    vertice[0] = obstacle[0] - obstacle[3]/2.0;
    vertice[1] = obstacle[1] - obstacle[4]/2.0;
    vertice[2] = obstacle[2] - obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] + obstacle[3]/2.0;
    vertice[1] = obstacle[1] - obstacle[4]/2.0;
    vertice[2] = obstacle[2] - obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] - obstacle[3]/2.0;
    vertice[1] = obstacle[1] + obstacle[4]/2.0;
    vertice[2] = obstacle[2] - obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] + obstacle[3]/2.0;
    vertice[1] = obstacle[1] + obstacle[4]/2.0;
    vertice[2] = obstacle[2] - obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] - obstacle[3]/2.0;
    vertice[1] = obstacle[1] - obstacle[4]/2.0;
    vertice[2] = obstacle[2] + obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] + obstacle[3]/2.0;
    vertice[1] = obstacle[1] - obstacle[4]/2.0;
    vertice[2] = obstacle[2] + obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] - obstacle[3]/2.0;
    vertice[1] = obstacle[1] + obstacle[4]/2.0;
    vertice[2] = obstacle[2] + obstacle[5]/2.0;
    vertices.push_back(vertice);


    vertice[0] = obstacle[0] + obstacle[3]/2.0;
    vertice[1] = obstacle[1] + obstacle[4]/2.0;
    vertice[2] = obstacle[2] + obstacle[5]/2.0;
    vertices.push_back(vertice);

    return vertices;
}

void findMinMax(const vector<Vector> &corners, Vector &minVertices, Vector &maxVertices)
{
    Vector min(3,0.0), max(3,0.0);
    for (int j=0; j<3; j++)
    {
        min[j] = corners[0][j];
        max[j] = corners[0][j];
    }

    for (int i=0; i<corners.size(); i++)
    {
        for (int j=0; j<3; j++)
        {
            if (corners[i][j]<min[j])
                min[j] = corners[i][j];
            if (corners[i][j]>max[j])
                max[j] = corners[i][j];
        }
    }
    minVertices = min;
    maxVertices = max;
}

vector<Vector> upperPlanner::expandObstacle(const vector<Vector> &traject, const vector<Vector> &obstacles
                                            , const double &lengthLimb)
{
    vector<Vector> obsSetExpanded;
    for (int i=0; i<traject.size(); i++)
    {
        for (int j=1; j<obstacles.size(); j++)
        {
            if(distWpObs(traject[i],obstacles[j])<=lengthLimb)
            {
                Vector waypoint = traject[i];
                Vector obstacle = obstacles[j];

                // Find 8 corners of obstacle
                vector<Vector> vertices=findCorner(obstacle);
                Vector minVertices(3,0.0), maxVertices(3,0.0);
                findMinMax(vertices,minVertices,maxVertices);

                // Find intersection points of normal vector with the admisible surface (sphere) of the next control point
                Vector Px1 = waypoint, Px2 = waypoint;
                Vector Py1 = waypoint, Py2 = waypoint;
                Vector Pz1 = waypoint, Pz2 = waypoint;

                Px1[0] = waypoint[0] + lengthLimb;
                Px2[0] = waypoint[0] - lengthLimb;

                Py1[1] = waypoint[1] + lengthLimb;
                Py2[1] = waypoint[1] - lengthLimb;

                Pz1[2] = waypoint[2] + lengthLimb;
                Pz2[2] = waypoint[2] - lengthLimb;

                // Find distaces from Waypoint to vertices of obstacle
                Vector distWpVer(8,0.0), scaleVerIts(8,1.0);

                for (int i=0; i<8; i++)
                {
                    distWpVer[i] = distWpObs(waypoint,vertices[i]);
                    scaleVerIts[i] = lengthLimb/distWpVer[i];
                }

                // Find intersection points of vectors from Waypoint to vertices of obstacle, with the admissible surface (sphere) of the next control point
                vector<Vector> intersectPts;
                for (int i=0; i<8; i++)
                {
                    Vector intersectPt(3,0.0);
                    for (int j=0; j<nDim; j++)
                        intersectPt[j] = waypoint[j] + (vertices[i][j]-waypoint[j])*scaleVerIts[i];
                    intersectPts.push_back(intersectPt);

                }


                // Approximate new obstacles by using Aligned-Angle Bounding Box for the set of intersection points.
                Vector minVerticesNew(3,0.0), maxVerticesNew(3,0.0);
                findMinMax(intersectPts,minVerticesNew,maxVerticesNew);


                if ((((waypoint[0]<maxVertices[0])&&(waypoint[0]>minVertices[0]))
                     &&((waypoint[1]<maxVertices[1])&&(waypoint[1]>minVertices[1]))))
                {
                    if (waypoint[2]>maxVertices[2])
//                        zMin = Pz2(3);
                        minVerticesNew[2] = Pz2[2];
                    else if(waypoint[2]<maxVertices[2])
//                        zMax = Pz1(3);
                        maxVerticesNew[2] = Pz1[2];
//                    else
//                    {
//                        zMin = Pz2(3);
//                        zMax = Pz1(3);

//                    }
                }
                else if ((((waypoint[0]<maxVertices[0])&&(waypoint[0]>minVertices[0]))
                          &&((waypoint[2]<maxVertices[2])&&(waypoint[2]>minVertices[2]))))
                {
                    if (waypoint[1]>maxVertices[1])
//                        yMin = Py2(2);
                        minVerticesNew[1] = Py2[1];
                    else if(waypoint[1]<maxVertices[1])
//                        yMax = Py1(2);
                        maxVerticesNew[1] = Py1[1];
//                    else
//                    {
//                        yMin = Py2(2);
//                        yMax = Py1(2);
//                    }
                }

                else if ((((waypoint[1]<maxVertices[1])&&(waypoint[1]>minVertices[1]))
                          &&((waypoint[2]<maxVertices[2])&&(waypoint[2]>minVertices[2]))))
                {
                    if (waypoint[0]>maxVertices[0])
//                        xMin = Px2(1);
                        minVerticesNew[0] = Px2[0];
                    else if(waypoint[0]<maxVertices[0])
//                        xMax = Px1(1);
                        maxVerticesNew[0] = Px1[0];
//                    else
//                        xMin = Px2(1);
//                        xMax = Px1(1);
                }




                Vector obsExpanded(6,0.0);
                for (int i=0; i<nDim; i++)
                {
                    obsExpanded[i] = (maxVerticesNew[i]+minVerticesNew[i])/2.0;
                    obsExpanded[i+nDim] = maxVerticesNew[i]-minVerticesNew[i];
                }
                obsSetExpanded.push_back(obsExpanded);

            }
        }
    }

    return obsSetExpanded;
}

Vector upperPlanner::findOtherEndPoint(const Vector &oneEndPoint, const Vector &halfPoint,
                                       const double &lengthLimb)
{
    // Find distaces from one Ending part to a Mid point in a limb
    double distEnd2Mid, scale;

    distEnd2Mid = distWpWp(oneEndPoint,halfPoint);
    scale = lengthLimb/distEnd2Mid;


    // Find intersection points of vectors from Waypoint to vertices of obstacle, with the admissible surface (sphere) of the next control point
    Vector otherEndPoint(nDim,0.0);
    for (int j=0; j<nDim; j++)
        otherEndPoint[j] = oneEndPoint[j]+(halfPoint[j]-oneEndPoint[j])*scale;

    return otherEndPoint;
}


bool upperPlanner::collisionCheck(const Vector &waypoint, const vector<Vector> &obstacles)
{
    bool collided;
    for(int i=0; i<obstacles.size(); i++)
    {
        collided = true;
        for(int j=0; j<nDim; j++)
        {
            if (fabs(waypoint[j]-obstacles[i][j])>obstacles[i][j+3]/2.0)
            {
                collided = false;
                break;
            }
        }
    }
    return collided;
}

double upperPlanner::distWpObs(const Vector &waypoint, const Vector &obstacle)
{
    double distance = 1000;
    if ((waypoint.size()==nDim) && (obstacle.size()>=nDim))
    {
        double sqSum = 0;
        for (int i=0; i<nDim; i++)
            sqSum += pow((waypoint[i]-obstacle[i]),2.0);
        distance = sqrt(sqSum);
    }
    return distance;
}

double upperPlanner::distWpWp(const Vector &wp1, const Vector &wp2)
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

void upperPlanner::sendTrajectory()
{
    Bottle &outTraj = upperTrajectPortOut.prepare();
    outTraj.clear();
    Bottle traj, viaPoint;
    traj.clear();
    viaPoint.clear();

    ts.update();

    traj.addString("End-Effector");   // Added on 21/03/2016 for sending trajectory of different control points

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

void upperPlanner::sendTrajectory(const string &ctrPoint,
                                  const vector<Vector> &trajectory)
{
    Bottle &outTraj = upperTrajectPortOut.prepare();
    outTraj.clear();
    Bottle traj, viaPoint;
    traj.clear();
    viaPoint.clear();

    ts.update();

    traj.addString(ctrPoint);   // Added on 21/03/2016 for sending trajectory of different control points

    traj.addInt(trajectory.size());
    traj.addInt(nDim);
    for (int j=0; j<trajectory.size(); j++)
    {
        viaPoint.clear();
        for (int i=0; i<nDim; i++)
            viaPoint.addDouble(trajectory[j][i]);
        traj.append(viaPoint);
    }
    outTraj.addList().read(traj);
    upperTrajectPortOut.setEnvelope(ts);
    upperTrajectPortOut.write();
}

void upperPlanner::sendTrajectory(const string &ctrPoint,
                                  BufferedPort<Bottle> trajectPortOut,
                                  const vector<Vector> &trajectory)
{
    Bottle &outTraj = trajectPortOut.prepare();
    outTraj.clear();
    Bottle traj, viaPoint;
    traj.clear();
    viaPoint.clear();

    ts.update();

    traj.addString(ctrPoint);   // Added on 21/03/2016 for sending trajectory of different control points

    traj.addInt(trajectory.size());
    traj.addInt(nDim);
    for (int j=0; j<trajectory.size(); j++)
    {
        viaPoint.clear();
        for (int i=0; i<nDim; i++)
            viaPoint.addDouble(trajectory[j][i]);
        traj.append(viaPoint);
    }
    outTraj.addList().read(traj);
    trajectPortOut.setEnvelope(ts);
    trajectPortOut.write();
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

void upperPlanner::logTrajectory(const string &trajectFilename,
                                 const double &time)
{
    char logNameResult[50];
    sprintf(logNameResult,"%s_resultPlanner.txt",trajectFilename.c_str());
    ofstream logfile1(logNameResult);

    if (logfile1.is_open())
      {
        logfile1<<"Solving_time:\t"<<time<<"\n";
        logfile1<<"End-Effector"<<"\n";
        for (int j=0; j<bestTrajEE.size(); j++)
          {
            logfile1<< "Way-point"<<j<<"\t";
            for (int i=0; i<nDim; i++)
              logfile1<< bestTrajEE[j][i] <<"\t";
            logfile1<<"\n";
          }
        logfile1<<"Half-Elbow"<<"\n";
        for (int j=0; j<bestTrajHalfElbow.size(); j++)
          {
            logfile1<< "Way-point"<<j<<"\t";
            for (int i=0; i<nDim; i++)
              logfile1<< bestTrajHalfElbow[j][i] <<"\t";
            logfile1<<"\n";
          }
      }
    logfile1.close();
}

void upperPlanner::displayTraj(const vector<Vector> &trajectory,
                               const string &color)
{
    printf("\n===============================\n");
    cout<<"DISPLAY TRAJECTORY"<<endl;
    for (int i=0; i<trajectory.size(); i++)
    {
        createStaticSphere(0.03, trajectory[i], color);
    }

}

void upperPlanner::createStaticSphere(double radius, const Vector &pos, const string &color)
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

void upperPlanner::createStaticBox(const Vector &pos, const string &type)
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
