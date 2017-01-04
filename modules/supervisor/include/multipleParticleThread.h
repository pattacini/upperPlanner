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

/**
 * @brief The multipleParticleThread class to generate multiple particles for the multiple Cartesian point controller
 */
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

    // Velocities of the particles in 3D
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
    string                          particlesPortName;       // name of port to send particle to controller

public:
    multipleParticleThread(int _rate, const string &_name, int _verbosity, const double &tolerence,
                           const string &portName);

    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
    * @brief Initializes the integrator to a new particle to track.
    * @param _x_0: a standard vector of 3D yarp Vectors of particles' positions
    * @param _vel: a standard vector of 3D yarp Vectors of particles' velocities
    * @return true/false on success/failure.
    **/
    bool setupNewParticle(const vector<Vector> &_x_0, const vector<Vector> &_vel);

    /**
    * @brief Stops the particle motion.
    * @return true/false on success/failure.
    **/
    bool stopParticle();

    /**
    * @brief Resume the particle motion after stopParticle().
    * @return true/false on success/failure.
    **/
    bool resumeParticle();

    /**
    * @brief Resets the particle at a given state
    * @param  _x_0: a standard vector of 3D yarp Vectors of particles' positions
    * @return true/false on success/failure.
    **/
    bool resetParticle(const vector<Vector> &_x_0);

    /**
    * @brief Gets the current state of the particle.
    * @return the standard vector of particles' 3D position.
    **/
    vector<Vector> getParticle();

    /**
     * @brief Sets last waypoints in motion path segments generated from the planner for particles generator
     * @param waypoint: a standard vector of 3D yarp Vectors of particles' positions,
     */
    void setLastWaypoint(const vector<Vector>& waypoint);

    /**
     * @brief Sets tolerance for the checking if the generated particles reaching the last waypoints or not
     * @param tolerence: double value of the tolerance for the particles generator
     */
    void setTolerence(const double& tolerence);

    /**
    * @brief Finds the distance between 2 waypoints
    * @param wp1: 3D yarp Vector of 3D coordinate of a waypoint 1
    * @param wp2: 3D yarp Vector of 3D coordinate of a waypoint 2
    * @return Output is a double value of the distance.
    */
    double distWpWp(const Vector &wp1, const Vector &wp2);

    /**
     * @brief Checks if the current segments of motion paths are finished or not
     * @return Output is a boolean value to indicate the current segments are finished(true) or not(false)
     */
    bool checkFinished();

    /**
     * @brief Sets value to isParticlesFinished variable
     * @param _value: boolean value
     * @see: isParticlesFinished
     */
    void setParticlesFinished(bool _value);

    /**
     * @brief Sets names for controlled points to publish to Controller by motionPlan
     * @param names: standard vector of string values containing names of controlled points
     * @see motionPlan
     */
    void setCtrlPointsNames(const vector<string> &names);

};

#endif // MULTIPLEPARTICLETHREAD_H
