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

#include "particleWaypointThread.h"

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

particleWaypointThread::particleWaypointThread(int _rate, const string &_name, int _verbosity,
                                               const double& tolerence) :
    particleThread(_rate,_name,_verbosity)
{
    lastWaypoint.clear();
    tol = tolerence;
    rate = _rate;
    nDim = 3;
    isFinished = false;
}

void particleWaypointThread::run()
{
    if (isRunning)
    {
//        printf("check\n");
//        LockGuard lg(mutex);
        integrator->integrate(vel);
        Vector x_n(nDim,0.0), x_d(nDim,0.0);
        x_n = getParticle();
        x_d = lastWaypoint;
        if (distWpWp(x_n,x_d)<=tol)
        {
            isRunning = false;
            printf("distance of x_n, x_d: %f\n",distWpWp(x_n,x_d));
            isFinished = true;
        }

    }
}

bool particleWaypointThread::setupNewParticle(const Vector &_x_0, const Vector &_vel)
{
    LockGuard lg(mutex);

    printf("%s particleThread:\n\tcheck 1\n",name.c_str());
    if (_x_0.size()==nDim && _vel.size()==nDim)
    {
        printf("\tcheck 2\n");
        isRunning=true;
        printf("\tcheck 3\n");
        vel=_vel;
        printf("\tcheck 4, _x_0.size: %d \n", (int)_x_0.size());
//        integrator = new Integrator(rate/1000,_x_0);
        integrator=new Integrator(rate/1000.0,Vector(3,0.0));

        integrator->reset(_x_0);
        printf("\tcheck 5\n");
        return true;
    }

    return false;
}

void particleWaypointThread::setLastWaypoint(const Vector& waypoint)
{
    lastWaypoint = waypoint;
    isFinished = false;
}

void particleWaypointThread::setTolerence(const double& tolerence)
{
    tol = tolerence;
}


double particleWaypointThread::distWpWp(const Vector &wp1, const Vector &wp2)
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

bool particleWaypointThread::checkFinished()
{
    return isFinished;
}
