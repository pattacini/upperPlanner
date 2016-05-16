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

#include <yarp/os/all.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <iostream>
#include <string.h>

#include "upperPlanner.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;
using namespace iCub::iKin;

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("reaching-with-avoidance");
    rf.setDefaultConfigFile("reaching-planner.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("");
        yInfo("Options:");
        yInfo("");
        yInfo("   --context                 path:   where to find the called resource");
        yInfo("   --from                    from:   the name of the .ini file.");
        yInfo("   --name                    name:   the name of the module (default reaching-planner).");
        yInfo("   --robot                   robot:  the name of the robot. Default icubSim.");
        yInfo("   --part                    part:   the arm to use. Default left_arm.");
        yInfo("   --verbosity               int:    verbosity level (default 0).");
        yInfo("   --running_mode            string: operation mode (default Single), other is Batch.");
        yInfo("   --maxReplan               int:    maxiumum repeat planning in Batch mode (default 0).");
        yInfo("   --targetName              string: reaching object (default Octopus).");
        yInfo("   --disableTorso            string: on/off the torso (default off), other is on.");
        yInfo("   --visualizeObjectsInSim   string: on/off visualizing objects in simulator (default on), other is off.");
        yInfo("");
        return 0;
    }

    if (!yarp.checkNetwork())
    {
        yError("No Network!!!");
        return -1;
    }


    upperPlanner planner;
    planner.configure(rf);

    return planner.runModule();
    //return planner.runModule(rf);   // runModule(rf) = configure() + runModule()

}
