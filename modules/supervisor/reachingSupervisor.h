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


#ifndef REACHINGSUPERVISOR_H
#define REACHINGSUPERVISOR_H

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/common.h>

#include <stdarg.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>

#include "motionPlan.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace iCub::iKin;


class reachingSupervisor : public RFModule
{
protected:
    string name;
    vector<waypointTrajectory> listTrajectory;

    motionPlan planPortIn;

public:
    reachingSupervisor();

    /************************************************************************/
    // Inherit methods
    virtual bool configure(ResourceFinder&);

    virtual bool close();

//    virtual bool attach (RpcServer&);

    virtual double getPeriod();

    virtual bool updateModule();


    /************************************************************************/
};

#endif // REACHINGSUPERVISOR_H
