#ifndef PARTICLEWAYPOINTTHREAD_H
#define PARTICLEWAYPOINTTHREAD_H

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

#include <stdarg.h>
#include <math.h>

#include "particleThread.h"


using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

class particleWaypointThread : public particleThread
{
protected:
    Vector lastWaypoint;
    double tol;             //tolerence
    int nDim;
    bool isFinished;
    double rate;
public:
    particleWaypointThread(int _rate, const string &_name, int _verbosity, const double& tolerence);

    virtual void run();

//    virtual void threadRelease();

    virtual bool setupNewParticle(const yarp::sig::Vector &, const yarp::sig::Vector &);

//    virtual bool stopParticle();

//    virtual bool resetParticle(const yarp::sig::Vector &_x_0);

//    virtual yarp::sig::Vector getParticle();

    void setLastWaypoint(const Vector& waypoint);

    void setTolerence(const double& tolerence);

    double distWpWp(const Vector &wp1, const Vector &wp2);

    bool checkFinished();


};

#endif // PARTICLEWAYPOINTTHREAD_H
