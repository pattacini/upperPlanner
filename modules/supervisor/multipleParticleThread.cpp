#include "multipleParticleThread.h"

multipleParticleThread::multipleParticleThread(int _rate, const string &_name, int _verbosity, const double &tolerence) :
    RateThread(_rate), name(_name), verbosity(_verbosity)
{
//    numberCtrlPoints    = numberCtrlPts;
    tol                 = tolerence;
    rate                = _rate;
    nDim                = 3;

    lastWaypoints.clear();
    isFinished          = false;
    isRunning           = false;

    // *****Port for communication***********************************
    particlesPortOut.open(("/"+name+"/particlesCartesianTrajectory:o").c_str());
}

void multipleParticleThread::run()
{
    if (isRunning)
    {
        vector<Vector> x_n = getParticle();
        vector<Vector> x_d = lastWaypoints;

        int countRunning = 1, countFinished = 1;
        for (int i=0; i< numberCtrlPoints; i++)
        {
//        printf("check\n");
//        LockGuard lg(mutex);
//            Integrator integrator = mIntegrator[i];
//            integrator.integrate(vel[i]);

//            if (!isParticlesFinished[i])
//                mIntegrator[i].integrate(vel[i]);
            if (distWpWp(x_n[i],x_d[i])<=tol)
            {
//                isRunning = false;
                isParticlesFinished[i] = true;
                printf("i = %d,\tdistance of x_n, x_d: %f\n",i,distWpWp(x_n[i],x_d[i]));
                countRunning++;
                countFinished++;
//                isFinished = true;
            }
            if (!isParticlesFinished[i])
                mIntegrator[i].integrate(vel[i]);

        }

        if (countRunning >= numberCtrlPoints)
            isRunning = false;
        if (countFinished >= numberCtrlPoints)
            isFinished = true;

        // Send particles of controlled waypoints to controller
        x_n = getParticle();
        particlesPortOut.clearTrajectory();


        for (int i=0; i<ctrlPointsNames.size();i++)
        {
//            printf("ctrlPointsNames[%d] = %s\n",i,ctrlPointsNames[i].c_str());
            vector<Vector> trajectory;
            trajectory.push_back(x_n[i]);
            waypointTrajectory wpTraj(ctrlPointsNames[i],trajectory);
            particlesPortOut.addTrajectory(wpTraj);
        }
        particlesPortOut.sendPlan();

    }
}

void multipleParticleThread::threadRelease()
{
    mIntegrator.clear();

}

bool multipleParticleThread::setupNewParticle(const vector<Vector> &_x_0, const vector<Vector> &_vel)
{
    LockGuard lg(mutex);

    printf("%s multipleParticleThread: setupNewParticle\n\tcheck 1\n",name.c_str());

//    for (int i=0; i<ctrlPointsNames.size();i++)
//    {
//        printf("ctrlPointsNames[%d] = %s\n",i,ctrlPointsNames[i].c_str());
//    }

    if (_x_0.size()==_vel.size())
    {
        isRunning=true;
        isFinished = false;
        numberCtrlPoints = _x_0.size();
        vel = _vel;
        setParticlesFinished(false);
        for (int i=0; i<_x_0.size(); i++)
        {
            if (_x_0[i].size()==nDim && _vel[i].size()==nDim)
            {
                printf("\tcheck 2\n");
                printf("\tcheck 3\n");
                printf("\tcheck 4, _x_0.size: %d \n", (int)_x_0[i].size());
                Integrator integrator=Integrator(rate/1000.0,Vector(3,0.0));

                integrator.reset(_x_0[i]);
                mIntegrator.push_back(integrator);
                printf("\tcheck 5\n");

            }

        }
        return true;
    }

    return false;
}

bool multipleParticleThread::stopParticle()
{
    LockGuard lg(mutex);
    isRunning=false;
    return true;
}

bool multipleParticleThread::resetParticle(const vector<Vector> &_x_0)
{
    LockGuard lg(mutex);
    setParticlesFinished(false);
    for (int i=0; i<_x_0.size(); i++)
    {
        if (_x_0[i].size()==nDim)
        {
            mIntegrator[i].reset(_x_0[i]);
        }
    }
    isRunning=false;
    return true;
}

vector<Vector> multipleParticleThread::getParticle()
{
    LockGuard lg(mutex);
    vector<Vector> particles;
    particles.clear();
    for (int i=0; i<mIntegrator.size(); i++)
    {
        Vector x_n= mIntegrator[i].get();
        particles.push_back(x_n);
    }
    return particles;
}

void multipleParticleThread::setLastWaypoint(const vector<Vector> &waypoints)
{
    lastWaypoints = waypoints;
    isFinished = false;

}

double multipleParticleThread::distWpWp(const Vector &wp1, const Vector &wp2)
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

bool multipleParticleThread::checkFinished()
{
    return isFinished;
}

int multipleParticleThread::printMessage(const int l, const char *f, ...) const
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

void multipleParticleThread::setParticlesFinished(bool _value)
{
    isParticlesFinished.resize(numberCtrlPoints);
    for (int i=0; i<numberCtrlPoints; i++)
    {
        isParticlesFinished[i]=_value;
    }
}

void multipleParticleThread::setCtrlPointsNames(const vector<string> &names)
{
    ctrlPointsNames = names;
}
