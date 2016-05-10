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
    targetName  =         "Octopus";
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
    //********************** NUMBER OF REPEAT PLANNING IN BATCH MODE************
    if (rf.check("maxReplan"))
    {
        maxReplan = rf.find("maxReplan").asInt();
        yInfo("[reaching-planner] maxReplan set to %i", maxReplan);
    }
    else yInfo("[reaching-planner] Could not find maxReplan option in the config file; using %i as default",maxReplan);
    //********************** TARGET************
    if (rf.check("targetName"))
    {
        targetName = rf.find("targetName").asString();
        yInfo("[reaching-planner] targetName set to %s", targetName.c_str());
    }
    else yInfo("[reaching-planner] Could not find targetName option in the config file; using %s as default",targetName.c_str());

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
        yError("[reaching-planner]Problems acquiring torso interfaces!!!!");
        return false;
    }

    //**** visualizing targets and collision points in simulator ***************************
    if (robot == "icubSim")
    {
        string port2icubsim = "/" + name + "/sim:o";
        //cout<<port2icubsim<<endl;
        if (!portToSimWorld.open(port2icubsim.c_str())) {
           yError("[reaching-planner] Unable to open port << port2icubsim << endl");
        }
        std::string port2world = "/icubSim/world";
        yarp::os::Network::connect(port2icubsim, port2world.c_str());
    }
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

    //****Synchronise icubSim and iCubGui *************************************************
    string armSimState = "/"+ robot +"/" + part + "/state:o";
    string armGui = "/iCubGui/" + part + ":i";
    if (yarp::os::Network::connect(armSimState.c_str(),armGui.c_str()))
        visualizeObjectsInGui = true;
    else
        visualizeObjectsInGui = false;

    if (!disableTorso)
    {
        string torsoSimState = "/"+ robot +"/torso/state:o";
        string torsoGui = "/iCubGui/torso:i";
        yarp::os::Network::connect(torsoSimState.c_str(),torsoGui.c_str());
    }

    //**** visualizing targets and collision points in iCubGui ***************************
    string port2iCubGui = "/" + name + "/gui:o";
    if (!portToGui.open(port2iCubGui.c_str())) {
       yError("[reaching-planner] Unable to open port << port2iCubGui << endl");
    }
    std::string portGuiObject = "/iCubGui/objects";     // World frame
    yarp::os::Network::connect(port2iCubGui.c_str(), portGuiObject.c_str());
    cmdGui.clear();
    cmdGui.addString("reset");
    portToGui.write(cmdGui);

//    initShowTrajGui("EE","blue");
//    initShowTrajGui("End-Effector","blue");
//    initShowTrajGui("Half-Elbow","yellow");
//    initShowTrajGui("Elbow","purple");


    //****RPC Port ************************************************************************
    rpcSrvr.open(("/"+name+"/rpc:i").c_str());
    attach(rpcSrvr);

    string port2OPC = "/" + name + "/OPC/rpc:io";
    if (!rpc2OPC.open(port2OPC.c_str()))
    {
        yError("[reaching-planner] Unable to open port << port2OPC << endl");
    }
//    string portOPCrpc = "/objectsPropertiesCollector/rpc";
//    string portOPCrpc = "/memory/rpc";
    string portOPCrpc = "/OPC/rpc"; // For WYSIWYD
    Network::connect(port2OPC.c_str(), portOPCrpc.c_str());

    string port2actReEn = "/" + name + "/actionsRenderingEngine/get:io";
    if (!rpc2actReEn.open(port2actReEn.c_str()))
    {
        yError("[reaching-planner] Unable to open port << port2actReEn << endl");
    }
    string port_actReEnrpc = "/actionsRenderingEngine/get:io"; // For WYSIWYD
    Network::connect(port2actReEn.c_str(), port_actReEnrpc.c_str());

    //   rpcDataProcessor processor;
    //   rpcSrvr.setReader(processor);

    //****Normal communication port *******************************************************
    ts.update();
    //   upperTrajectPortOut.open(("/"+name+"/bestCartesianTrajectory:o").c_str());

    EEPortOut.setControlPoint("End-Effector");
    EEPortOut.open(("/"+name+"/End-Effector/bestCartesianTrajectory:o").c_str());

    HalfElbowPortOut.setControlPoint("Half-Elbow");
    HalfElbowPortOut.open(("/"+name+"/Half-Elbow/bestCartesianTrajectory:o").c_str());

    planPortOut.open(("/"+name+"/bestCartesianTrajectory:o").c_str());
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

    clock_t start;// = clock();


    // Check if required re-plan and do if necessary; remember to clear "replan" after running
    if (replan)
    {
        bool haveTarget = false;
        success = false;    //Batch Summary only

//        singlePlanner plannerEE(verbosity,name,robot,"End-effector");
//        singlePlanner plannerElbow(verbosity,name, robot, "Elbow");
//        singlePlanner plannerHalfElbow(verbosity,name, robot,"Half-Elbow");


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

        // Old codes for obstacles and target


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

        // New codes for obstacles and target
        if (robot == "icubSim")
        {

            haveTarget = true;
            goal[0]=-1.0/scale; //-3.0/scale; //-10.0/scale;
            goal[1]=5.64/scale; //6.0/scale; //7.0/scale;
            goal[2]=3.7/scale;  //1.0/scale;

//            target = goal;
//            convertPosFromSimToRootFoR(goal,target);

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
        else if(robot == "icub")
        {
            obsSet.clear();
            printf("===============================\n");
            printf("Get target from OPC!!!\n");

            Vector targetCenterRoot(nDim,0.0), targetCenterSim(nDim,0.0);
            Vector targetRoot(2*nDim,0.0), targetSim(2*nDim,0.0);

            // Obtain target
            haveTarget = getObjFromOPC_Name(targetName,targetID, targetRoot);
            convertObjFromRootToSimFoR(targetRoot,targetSim);
            goal = targetSim;
            printf("targetRoot: %f, %f, %f, %f, %f, %f\n",targetRoot[0], targetRoot[1], targetRoot[2],targetRoot[3], targetRoot[4], targetRoot[5]);
//            printf("goal: %f, %f, %f, %f, %f, %f\n",goal[0], goal[1], goal[2],goal[3], goal[4], goal[5]);

            // Obtain table
            printf("Get table from actionsRenderingEngine!!!\n");
            double tableHeightUp;


            if (getTableHeightFromOPC(tableHeightUp))
            {
                Vector table(6,0.8), tableSim(6,0.0);
                table[0] = -0.5;
                table[1] = 0.0;
                table[4] = 1.6;
                table[5] = 0.02;

                table[2] = tableHeightUp-table[5]/2;
                convertObjFromRootToSimFoR(table,tableSim);
                obsSet.push_back(tableSim);
                printf("table: %f, %f, %f, %f, %f, %f\n",table[0], table[1], table[2], table[3], table[4], table[5]);
            }

            // Obtain obstacles
            vector<int> obstacleIDsOPC = getObsIDFromOPC_Name();
            for (int i=0; i<obstacleIDsOPC.size(); i++)
            {
                printf ("\tid [%d]=%d\n",i,obstacleIDsOPC[i]);
//                Vector obstacle = getObsFromOPC(obstacleIDsOPC[i]);
                Vector obstacle(6,0.0), obstacleSim(6,0.0);
                if (getObsFromOPC(obstacleIDsOPC[i],obstacle))
                {
                    convertObjFromRootToSimFoR(obstacle,obstacleSim);
                    obsSet.push_back(obstacleSim);
                    printf("obstacle: %f, %f, %f, %f, %f, %f\n",obstacle[0], obstacle[1], obstacle[2],obstacle[3], obstacle[4], obstacle[5]);
                }

            }

//            targetCenterRoot = targetRoot.subVector(0,2);
//            convertPosFromRootToSimFoR(targetCenterRoot,targetCenterSim);
//            goal.setSubvector(0,targetCenterSim);
//            goal[3] = targetRoot[4];
//            goal[4] = targetRoot[5];
//            goal[5] = targetRoot[3];

//            goal.setSubvector(0,startPose);

//            printf("targetCenterRoot: %f, %f, %f\n",targetCenterRoot[0], targetCenterRoot[1], targetCenterRoot[2]);
//            printf("targetCenterSim: %f, %f, %f\n",targetCenterSim[0], targetCenterSim[1], targetCenterSim[2]);


            // Get objects from messages, information in Root frame

            // Convert to World (simulator) frame

        }

        if (haveTarget)
        {

            target = goal;
            start = clock();

            // 4.Planning for End-Effector
            do
            {
                singlePlanner plannerEE(verbosity,name,robot,running_mode,"End-effector");
                plannerEE.setRegionOperating(workspace);    // World frame
                plannerEE.setGoal(goal);                  // World frame
                plannerEE.setDeadline(planningTime);        // World frame
                plannerEE.setObstacles(obsSet);             // World frame


                plannerEE.setStart(startPose);              // World frame
        //        plannerEE.setStart(xCur);                   // Root frame


                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                printf("!!!                      !!!\n");
                printf("!!! END-EFFECTOR PLANNER !!!\n");
                printf("!!!                      !!!\n");
                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                plannerEE.executeTrajectory(bestTrajEE, bestTrajRootEE, "blue");

                if(running_mode=="single")
                    plannerEE.logVertices();

            }
            while (bestTrajEE.size()==0);

            vector<Vector> bestTrajEE_temp = bestTrajEE;
            vector<Vector> bestTrajRootEE_temp = bestTrajRootEE;

            // Repeat planning for Half-Elbow if there is collision in Elbow
            do
            {
                // 4b. Planning for Half-Elbow
                if (bestTrajEE.size()>1)
                {
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

                        bool replanPadWaypoint = false;
                        vector<Vector> paddingWaypoints, paddingRootWaypoints;
        //                while (bestTrajLocalHalfElbow.size()==0 || replanPadWaypoint)
                        do
                        {
                            singlePlanner localPlannerHalfElbow(verbosity,name,robot,running_mode,"local-Half-Elbow");
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
                                    // From i=1 rather than i=0 to prevent replication when forming the whole trajectory
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
                        }
                        while (replanPadWaypoint);

                        // Insert waypoints to the End-Effector's trajectory
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

                        // Store last waypoint of local planner for Half Elbow's trajectory later
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

                    // Insert the last waypoint of last local planner for Half Elbow to form the whole trajectory of the Half Elbow control point
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

    //                if (running_mode == "single")
    //                {
    //                    // 5.Sending message of Trajectory through port
    //                    EEPortOut.setTrajectory(bestTrajRootEE);
    //                    EEPortOut.sendTrajectory();

    //                    HalfElbowPortOut.setTrajectory(bestTrajRootHalfElbow);
    //                    HalfElbowPortOut.sendTrajectory();

    //                    // 6. Display Trajectory

    //                    displayTraj(bestTrajEE,"blue");
    //                    displayTraj(bestTrajHalfElbow,"yellow");
    //                }

        //            // Replan
        //            replan = false;
                }


    //            if (robot =="icubSim")  // Remove this and else when finishing debug with Elbow checking
    //            {

                // ELBOW Checking
                if (bestTrajEE.size() == bestTrajHalfElbow.size())
                {
                    bestTrajElbow.clear();
                    printf("Distance of waypoint in trajectory of EE and Half-elbow are: \n");
                    for (int i=0; i<bestTrajEE.size(); i++)
                    {
                        printf ("\twaypoint %d: %f \n", i, distWpWp(bestTrajEE[i],bestTrajHalfElbow[i]));

                        Vector elbow(3,0.0), elbowRoot(3,0.0);
                        elbow = findOtherEndPoint(bestTrajEE[i],bestTrajHalfElbow[i],lForearm);
                        bestTrajElbow.push_back(elbow);

                        convertPosFromSimToRootFoR(elbow,elbowRoot);
                        bestTrajRootElbow.push_back(elbowRoot);

                    }


                    printf("Length of Half Forearm: %f \n", lForearm/2.0);
                    if (bestTrajEE.size()!=0)
                        success = true;

        //            displayTraj(bestTrajElbow,"purple");
                    printf("=============================\n");
                    printf("ELBOW CHECKING\n");
                    printf("\tCheck elbow position \n");
                    for (int i=0; i<bestTrajElbow.size(); i++)
                    {
                        if (collisionCheck(bestTrajElbow[i],obsSet))
                        {
                            success = false;
                            printf("\t\tElbow %d is collided\n", i);
                        }
                    }
                    printf("\tCheck elbow path \n");
                    for (int i=1; i<bestTrajElbow.size(); i++)
                    {
                        if (collisionCheckPathSegment(bestTrajElbow[i-1],bestTrajElbow[i],obsSet))
                        {
                            success = false;
                            printf("\t\tSegment %d of the Elbow's path is collided\n", i);
                        }
                    }
                }

                // Replan if there is collision in Elbow
                if (!success)
                {
                    printf("===============================\n");
                    printf("Replanning cause collision in Elbow!!!\n");
                    printf("===============================\n");
                    bestTrajEE.clear();
                    bestTrajRootEE.clear();
                    bestTrajEE = bestTrajEE_temp;
                    bestTrajRootEE = bestTrajRootEE_temp;
                    bestTrajHalfElbow.clear();
                    bestTrajRootHalfElbow.clear();
                    bestTrajElbow.clear();
                    bestTrajRootElbow.clear();
                    obsSetExpandedHalfElbow.clear();
                }

    //            }
    //            else
    //            {
    //                success = true;
    //            }
            }
            while (!success);

            clock_t finish = clock();

            solvingTime = ((double)(finish-start))/CLOCKS_PER_SEC;

            printf("===============================\n");
            printf("SUMMARY\n");

            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            printf("!!!  Solving time : %f  !!!\n", solvingTime);
            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

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
            // 5.Sending message of Trajectory through port
            EEPortOut.setTrajectory(bestTrajRootEE);
            EEPortOut.sendTrajectory();

            HalfElbowPortOut.setTrajectory(bestTrajRootHalfElbow);
            HalfElbowPortOut.sendTrajectory();

            planPortOut.clearTrajectory();
            waypointTrajectory EE("End-Effector",bestTrajRootEE);
            waypointTrajectory HE("Half-Elbow",bestTrajRootHalfElbow);
            planPortOut.addTrajectory(EE);
            planPortOut.addTrajectory(HE);
            planPortOut.sendPlan();

            // Using class motionPlan


            // 6. Display Trajectory

            displayTraj(bestTrajEE,"blue");
            displayTraj(bestTrajHalfElbow,"yellow");
            displayTraj(bestTrajElbow,"purple");

            if (visualizeObjectsInGui)
            {
                displayWorkspaceGui();
    //            displayWpsGui(bestTrajRootEE, "End-Effector", "blue");
    //            displayWpsGui(bestTrajRootHalfElbow, "Half-Elbow", "yellow");
    //            displayWpsGui(bestTrajRootElbow, "Elbow", "purple");

    //            initShowTrajGui("End","blue");
                initShowTrajGui("EE","blue");
                initShowTrajGui("HE","yellow");
                initShowTrajGui("E","purple");

    //            updateTrajGui(bestTrajRootEE, "End");
                updateTrajGui(bestTrajRootEE, "EE");
                updateTrajGui(bestTrajRootHalfElbow, "HE");
                updateTrajGui(bestTrajRootElbow, "E");
            }


            replan = false;
        }

        }
        else
        {
            printf("Cannot find the target, waiting for the new one\n");
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

bool upperPlanner::getObjFromOPC_Name(const string &objectName, int &idObject, Vector &objectRoot)
{
    Vector objectIn3D(6,0.0);
    Vector centerIn3D(3,0.0);
    bool isPresent = false;

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content=cmd.addList();
    Bottle &cond_1=content.addList();
    cond_1.addString("entity");
    cond_1.addString("==");
    cond_1.addString("object");
    content.addString("&&");
    Bottle &cond_2=content.addList();
    cond_2.addString("name");
    cond_2.addString("==");
    cond_2.addString(objectName);

    rpc2OPC.write(cmd,reply);
    if(reply.size()>1)
    {
        if(reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            if (Bottle *b=reply.get(1).asList())
            {
                if (Bottle *b1=b->get(1).asList())
                {
                    cmd.clear();
//                    int id=b1->get(0).asInt();
                    idObject=b1->get(0).asInt();
                    cmd.addVocab(Vocab::encode("get"));
                    Bottle &info=cmd.addList();
                    Bottle &info2=info.addList();
                    info2.addString("id");
//                    info2.addInt(id);
                    info2.addInt(idObject);

                }
                else
                    yError("no object id provided by OPC!");
            }
            else
                yError("uncorrect reply from OPC!");

            Bottle reply;
            rpc2OPC.write(cmd,reply);
            if (reply.size()>1)
            {
                if (reply.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *b=reply.get(1).asList())
                    {
                        if ((b->find("isPresent").asInt())==1)
                        {
                            isPresent = true;

                            if (Bottle *b1=b->find("position_3d").asList())
                            {
                                for (int i=0; i<3; i++)
                                {
    //                                centerIn3D[i]=b1->get(i).asDouble();
                                    objectIn3D[i]=b1->get(i).asDouble();
                                }

                            }
                            else
                                yError("position_3d field not found in the OPC reply!");

                            if (double b1=b->find("rt_dim_x").asDouble())
                            {
                                objectIn3D[3]=b1;
                                printf("\tdimX = %f\n", b1);
                            }
                            else
                                yError("rt_dim_x field not found in the OPC reply!");
                            if (double b1=b->find("rt_dim_y").asDouble())
                            {
                                objectIn3D[4]=b1;
                                printf("\tdimY = %f\n", b1);
                            }
                            else
                                yError("rt_dim_y field not found in the OPC reply!");
                            if (double b1=b->find("rt_dim_z").asDouble())
                            {
                                objectIn3D[5]=b1;
                                printf("\tdimZ = %f\n", b1);
                            }
                            else
                                yError("rt_dim_z field not found in the OPC reply!");
                        }
                    }
                    else
                        yError("uncorrect reply structure received!");
                }
                else
                    yError("Failure in reply for object 2D point!");
            }
            else
                yError("reply size for 3D point less than 1!");
        }
        else
            yError("Failure in reply for object id!");
    }
    else
        yError("reply size for object id less than 1!");

    objectRoot = objectIn3D;

    return isPresent;

}

vector<int> upperPlanner::getObsIDFromOPC_Name()
{
    vector<int> obstacleIDs;
    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content=cmd.addList();
//    Bottle &cond_1=content.addList();
    content.addString("all");

    rpc2OPC.write(cmd,reply);
    if(reply.size()>1)
    {
        if(reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            if (Bottle *b=reply.get(1).asList())
            {
                if (Bottle *b1=b->get(1).asList())
                {
                    for (int i=0; i<b1->size();i++)
                    {
                        int idObs = b1->get(i).asInt();
                        if (idObs != targetID)
                        {
                            obstacleIDs.push_back(idObs);
                        }
                    }
                }
                else
                    yError("no object id provided by OPC!");
            }
            else
                yError("uncorrect reply from OPC!");
        }
        else
            yError("Failure in reply for object id!");
    }
    else
        yError("reply size for object id less than 1!");

    return obstacleIDs;
}

bool upperPlanner::getObsFromOPC(const int &idObject, Vector &obstacle)
{
    Vector objectIn3D(6,0.0);
    bool isPresent = false;

    Bottle cmd,reply;


    cmd.addVocab(Vocab::encode("get"));
    Bottle &info=cmd.addList();
    Bottle &info2=info.addList();
    info2.addString("id");

    info2.addInt(idObject);


    rpc2OPC.write(cmd,reply);
    if (reply.size()>1)
    {
        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            if (Bottle *b=reply.get(1).asList())
            {
                if ((b->find("isPresent").asInt())==1)
                {
                    isPresent = true;

                    if (Bottle *b1=b->find("position_3d").asList())
                    {
                        for (int i=0; i<3; i++)
                        {
                            objectIn3D[i]=b1->get(i).asDouble();
                        }

                    }
                    else
                        yError("position_3d field not found in the OPC reply!");

                    if (double b1=b->find("rt_dim_x").asDouble())
                    {
                        objectIn3D[3]=b1;
                        printf("\tdimX = %f\n", b1);
                    }
                    else
                        yError("rt_dim_x field not found in the OPC reply!");
                    if (double b1=b->find("rt_dim_y").asDouble())
                    {
                        objectIn3D[4]=b1;
                        printf("\tdimY = %f\n", b1);
                    }
                    else
                        yError("rt_dim_y field not found in the OPC reply!");
                    if (double b1=b->find("rt_dim_z").asDouble())
                    {
                        objectIn3D[5]=b1;
                        printf("\tdimZ = %f\n", b1);
                    }
                    else
                        yError("rt_dim_z field not found in the OPC reply!");
                }
            }
            else
                yError("uncorrect reply structure received!");
        }
        else
            yError("Failure in reply for object 3D point!");
    }
    else
        yError("reply size for 3D point less than 1!");


    obstacle = objectIn3D;
    return isPresent;
}

bool upperPlanner::getTableHeightFromOPC(double &tableHeight)
{
    bool isPresent = false;

    Bottle cmd,reply;

    cmd.addVocab(Vocab::encode("get"));
    cmd.addVocab(Vocab::encode("table"));
//    cmd.addString("table");

//    cmd.addString("get table");

    rpc2actReEn.write(cmd,reply);
//    cout << reply.toString()<<endl;
    if (reply.size()>=1)
    {
        if (isPresent=reply.check("table_height"))
            tableHeight=reply.find("table_height").asDouble();
        else
            yError("table_height field not found in the actionsRenderingEngine reply!");
    }
    else
        yError("reply size for table height less than 1!");

    return isPresent;

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
            if (fabs(waypoint[j]-obstacles[i][j])>=obstacles[i][j+3]/2.0)   // Remove "=" sign for more strict checking
            {
                collided = false;
                break;
            }
        }
    }
    return collided;
}

void upperPlanner::findCenterOnSurfaces(const Vector &obstacle,
                                   vector<Vector> &centers, vector<Vector> &normVectors)
{
    Vector center(nDim,0.0);

    centers.clear();
    normVectors.clear();

    for (int i=0; i<nDim; i++)
    {
        center = obstacle.subVector(0,2);
        center[i] = obstacle[i]-obstacle[i+3]/2.0;
        centers.push_back(center);

        center[i] = obstacle[i]+obstacle[i+3]/2.0;
        centers.push_back(center);
    }

    for (int i=0; i<centers.size(); i++)
    {
        Vector normVector(nDim,0.0);
        for (int j=0; j<nDim; j++)
        {
            normVector[j] = centers[i][j] - obstacle[j];
        }
        normVectors.push_back(normVector);
    }

}

double upperPlanner::dotProduct(const Vector &v1, const Vector &v2)
{
    double out = 0;

    if (v1.size() == v2.size())
    {
        for (int i=0; i<v1.size(); i++)
        {
            out += v1[i]*v2[i];
        }
    }
    return out;
}

bool upperPlanner::intersectionSegmentPlane(const Vector &point1, const Vector &point2,
                                       const Vector &pointOnPlane, const Vector &normalVector,
                                       Vector &intersection)
{
    bool intersect = false;
    intersection.resize(nDim);

    Vector u(nDim,0.0), w(nDim,0.0);
    double valueD, valueN, scale;

    for (int i=0; i<nDim; i++)
    {
        u[i] = point2[i]-point1[i];
        w[i] = point1[i]-pointOnPlane[i];
    }

    valueD = dotProduct(normalVector,u);
    valueN = -dotProduct(normalVector,w);

    if (fabs(valueD)>=0.000001)
    {
        scale = valueN/valueD;

        if (scale>=0.0 && scale <=1.0)
        {
            for (int i=0; i<nDim; i++)
            {
                intersection[i] = point1[i] + scale*u[i];
            }
            intersect = true;
        }
    }
    return intersect;
}

bool upperPlanner::collisionCheckPathSegment(const Vector &point1, const Vector &point2,
                                        const vector<Vector> &obstacles)
{
    bool collided = false;
    for (int i=0; i<obstacles.size(); i++)
    {
        vector<Vector> centers;
        vector<Vector> normVects;
        // Find center and normal vector of 6 surface of each obstacle
//        printf("\tFind center and norm vector for obstacle %d-th \n", i);
        findCenterOnSurfaces(obstacles[i],centers,normVects);
        for (int j=0; j<centers.size(); j++)
        {
            Vector intersectPoint(nDim,0.0);
            // Find intersection point between two sucessive waypoint and the surface of obstacle
//            printf("\tFind intersection point between path segment and surface %d-th\n",j);
            if (intersectionSegmentPlane(point1,point2,centers[j],normVects[j],intersectPoint))
            {
                // Check if the intersection point is in inside or on obstacle
                // If YES: path collides with obstacle
                // IF NO: no collision

                vector<Vector> subObsSet;
                subObsSet.push_back(obstacles[i]);
//                printf("\tCollision check for explored intersection \n");
                if (collisionCheck(intersectPoint,subObsSet))
                {
                    collided = true;
                    break;
                }
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
    if (trajectory.size()>0)
    {
        printf("\n===============================\n");
        cout<<"DISPLAY TRAJECTORY"<<endl;
        for (int i=0; i<trajectory.size(); i++)
        {
            createStaticSphere(0.03, trajectory[i], color);
        }
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

void upperPlanner::displayWorkspaceGui()
{

    printf("\n===============================\n");
    cout<<"DISPLAY WORKSPACE GUI"<<endl;

    cmdGui.clear();
    cmdGui.addString("reset");
    portToGui.write(cmdGui);

    Vector obsGui(6,0.0);
    if (obsSet.size()>0)
    {
        int firstIndex;
        if (robot == "icubSim")
            firstIndex = 1;
        else if (robot == "icub")
            firstIndex = 0;

        for (int i=firstIndex; i<obsSet.size(); i++)
        {

            obsGui.resize(obsSet[i].size());
            convertObjFromSimToRootFoR(obsSet[i],obsGui);

            if (i==firstIndex)
                createObsGui(obsGui,"table",i);
            else
                createObsGui(obsGui,"obstacle",i);
        }
    }
    convertObjFromSimToRootFoR(target,obsGui);
    createObsGui(obsGui,"goal",0);

}

template <typename T>
string NumberToString ( T Number )
{
    stringstream ss;
    ss << Number;
    return ss.str();
}

void upperPlanner::createObsGui(const Vector &pos, const string &type, const int &order)
{
    if (pos.size()==6)
    {
        string objectName;
        if (type == "obstacle")
            objectName = "o" + NumberToString(order);
        else if (type == "goal")
            objectName = "g";
        else if (type == "table")
            objectName = "t";


        cmdGui.clear();
        cmdGui.addString("object");
        cmdGui.addString(objectName);
//        cmdGui.addString("");
        for (int i=3; i<pos.size(); i++)    // size
        {
            cmdGui.addDouble(pos[i]*1000.0);
        }
        for (int i=0; i<3; i++)             // position
        {
            cmdGui.addDouble(pos[i]*1000.0);
        }
        for (int i=0; i<3; i++)             // orientation
        {
            cmdGui.addDouble(0.0);
        }
        if (type == "obstacle")             // color
        {
            cmdGui.addInt(255);cmdGui.addInt(0);cmdGui.addInt(0); //red
        }
        else if (type == "goal")
        {
            cmdGui.addInt(0);cmdGui.addInt(255);cmdGui.addInt(0); //green
        }
        else if (type == "table")
        {
            cmdGui.addInt(210);cmdGui.addInt(105);cmdGui.addInt(30); //chocolate
        }
        cmdGui.addDouble(.9);

        portToGui.write(cmdGui);

    }


}

void upperPlanner::displayWpsGui(const vector<Vector> &trajectory, const string &ctrlPoint, const string &color)
{
    printf("\n===============================\n");
    cout<<"DISPLAY WAYPOINTS GUI"<<endl;
    for (int i=0; i<trajectory.size(); i++)
    {
        createWpGui(trajectory[i], ctrlPoint, i, color);
    }
}

void upperPlanner::createWpGui(const Vector &pos, const string &ctrlPoint,
                               const int &order, const string &color)
{
    if (pos.size()==3)
    {
        string objectName;
        if (ctrlPoint == "End-Effector")
            objectName = "ee" + NumberToString(order);
        else if (ctrlPoint == "Half-Elbow")
            objectName = "h" + NumberToString(order);
        else if (ctrlPoint == "Elbow")
            objectName = "e" + NumberToString(order);


        cmdGui.clear();
        cmdGui.addString("object");
        cmdGui.addString(objectName);
//        cmdGui.addString("");
        // size
        cmdGui.addDouble(30);
        cmdGui.addDouble(30);
        cmdGui.addDouble(30);

        for (int i=0; i<3; i++)             // position
        {
            cmdGui.addDouble(pos[i]*1000.0);
        }
        for (int i=0; i<3; i++)             // orientation
        {
            cmdGui.addDouble(0.0);
        }
        if (color =="red")             // color
        {
            cmdGui.addInt(255);cmdGui.addInt(0);cmdGui.addInt(0); //red
        }
        else if (color =="green")
        {
            cmdGui.addInt(0);cmdGui.addInt(255);cmdGui.addInt(0); //green
        }
        else if (color =="blue")
        {
            cmdGui.addInt(0);cmdGui.addInt(0);cmdGui.addInt(255); //blue
        }
        else if (color =="purple")
        {
            cmdGui.addInt(255);cmdGui.addInt(0);cmdGui.addInt(255); //purple
        }
        else if (color =="yellow")
        {
            cmdGui.addInt(255);cmdGui.addInt(255);cmdGui.addInt(0); //yellow
        }
        cmdGui.addDouble(.9);

        portToGui.write(cmdGui);

    }
}

void upperPlanner::convertObjFromSimToRootFoR(const Vector &obj, Vector &outObj)
{
    Vector object(3,0.0);

    convertPosFromSimToRootFoR(obj.subVector(0,2),object);
    outObj[0] = object[0];
    outObj[1] = object[1];
    outObj[2] = object[2];
    outObj[3] = obj[5];
    outObj[4] = obj[3];
    outObj[5] = obj[4];
}

void upperPlanner::convertObjFromRootToSimFoR(const Vector &obj, Vector &outObj)
{
    Vector object(3,0.0);

    convertPosFromRootToSimFoR(obj.subVector(0,2),object);
    outObj[0] = object[0];
    outObj[1] = object[1];
    outObj[2] = object[2];
    outObj[3] = obj[4];
    outObj[4] = obj[5];
    outObj[5] = obj[3];
}

void upperPlanner::initShowTrajGui(const string &ctrlPoint, const string &color)
{
    printf("\n===============================\n");
    cout<<"INIT TRAJECTORY GUI"<<endl;

    cmdGui.clear();

    cmdGui.addString("trajectory");
    cmdGui.addString(ctrlPoint.c_str());    // trajectory identifier
//    cmdGui.addString(ctrlPoint.c_str());              // trajectory name
    cmdGui.addString("");              // trajectory name
    cmdGui.addInt(512);                // max samples in circular queue
    cmdGui.addDouble(30.0);             // lifetime of samples
    if (color =="red")             // color
    {
        cmdGui.addInt(255);cmdGui.addInt(0);cmdGui.addInt(0); //red
    }
    else if (color =="green")
    {
        cmdGui.addInt(0);cmdGui.addInt(255);cmdGui.addInt(0); //green
    }
    else if (color =="blue")
    {
        cmdGui.addInt(0);cmdGui.addInt(0);cmdGui.addInt(255); //blue
    }
    else if (color =="purple")
    {
        cmdGui.addInt(255);cmdGui.addInt(0);cmdGui.addInt(255); //purple
    }
    else if (color =="yellow")
    {
        cmdGui.addInt(255);cmdGui.addInt(255);cmdGui.addInt(0); //yellow
    }
//    cmdGui.addInt(0);cmdGui.addInt(0);cmdGui.addInt(255);
    cmdGui.addDouble(1.0);             // alpha [0,1]
    cmdGui.addDouble(5.0);             // line width

    portToGui.write(cmdGui);

}

void upperPlanner::updateTrajGui(const vector<Vector> &trajectory, const string &ctrlPoint)
{
    if (trajectory.size()>0)
    {
        printf("\n===============================\n");
        cout<<"DISPLAY TRAJECTORY GUI"<<endl;


        for (int i=0; i<trajectory.size(); i++)
        {
            cmdGui.clear();

            cmdGui.addString("addpoint");
            cmdGui.addString(ctrlPoint.c_str());                    // trajectory identifier
            cmdGui.addDouble(1000.0*trajectory[i][0]);      // posX [mm]
            cmdGui.addDouble(1000.0*trajectory[i][1]);      // posY [mm]
            cmdGui.addDouble(1000.0*trajectory[i][2]);      // posZ [mm]

            portToGui.write(cmdGui);
        }
    }

}

// empty line to make gcc happy
