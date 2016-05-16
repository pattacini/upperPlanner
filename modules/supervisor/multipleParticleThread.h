#ifndef MULTIPLEPARTICLETHREAD_H
#define MULTIPLEPARTICLETHREAD_H

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

#include <vector>
#include <stdarg.h>
#include <math.h>

#include "motionPlan.h"

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

class multipleParticleThread : public yarp::os::RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    // Integrator to get the particle trajectory
    vector<iCub::ctrl::Integrator>  mIntegrator;
//    iCub::ctrl::Integrator *integrator;

    // Speed of the particle in 3D
    vector<yarp::sig::Vector>       vel;

    // Mutex for handling things correctly
    yarp::os::Mutex                 mutex;

    // Variable to turn the particleThread on or off
    bool                            isRunning;

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;


    vector<Vector>                  lastWaypoints;
    bool                            isFinished;
    vector<bool>                    isParticlesFinished;    // Variable to stop each particle
    int                             numberCtrlPoints;
    vector<string>                  ctrlPointsNames;

    double                          tol;                    //tolerence
    int                             nDim;
    double                          rate;

    motionPlan                      particlesPortOut;        // port to send particle to controller

public:
    multipleParticleThread(int _rate, const string &_name, int _verbosity, const double& tolerence);

    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
    * Initializes the integrator to a new particle to track.
    * @return true/false on success/failure.
    **/
    bool setupNewParticle(const vector<Vector> &_x_0, const vector<Vector> &_vel);

    /**
    * Stops the particle motion.
    * @return true/false on success/failure.
    **/
    bool stopParticle();

    /**
    * Resets the particle at a given state
    * @param  _x_0 the state
    * @return true/false on success/failure.
    **/
    bool resetParticle(const vector<Vector> &_x_0);

    /**
    * Gets the current state of the particle.
    * @return the particle 3D position.
    **/
    vector<Vector> getParticle();

    void setLastWaypoint(const vector<Vector>& waypoint);

    void setTolerence(const double& tolerence);

    double distWpWp(const Vector &wp1, const Vector &wp2);

    bool checkFinished();

    void setParticlesFinished(bool _value);

    void setCtrlPointsNames(const vector<string> &names);

};

#endif // MULTIPLEPARTICLETHREAD_H
