#include "motionPlan.h"

//using namespace waypointTrajectory;

template <typename T>
string NumberToString ( T Number )
{
    stringstream ss;
    ss << Number;
    return ss.str();
}

motionPlan::motionPlan()
{

}

void motionPlan::addTrajectory(const waypointTrajectory &ctrlPtTrajectory)
{
    listTrajectory.push_back(ctrlPtTrajectory);
}

void motionPlan::clearTrajectory()
{
    listTrajectory.clear();
}

vector<waypointTrajectory> motionPlan::getListTrajectory()
{
    return listTrajectory;
}

void motionPlan::sendPlan()
{
    Bottle &outPlan = prepare();
    outPlan.clear();
    for (int i=0; i<listTrajectory.size(); i++)
    {
        waypointTrajectory trajectory = listTrajectory[i];
        Bottle trajectPack, ctrlPt, nWp, nDim, listWaypoints, wp, coordinate;
        vector<Vector> waypoints = trajectory.getWaypoints();

        ctrlPt.addString("control-point");
        ctrlPt.addString(trajectory.getCtrlPoint());
        trajectPack.addList().read(ctrlPt);

        nWp.addString("number-waypoints");
        nWp.addInt(trajectory.getNbWaypoint());
        trajectPack.addList().read(nWp);

        nDim.addString("number-dimension");
        nDim.addInt(trajectory.getDimension());
        trajectPack.addList().read(nDim);


        for (int j=0; j<waypoints.size(); j++)
        {
            wp.clear(); coordinate.clear();
            string wpName = "waypoint_" + NumberToString(j);
            wp.addString(wpName);
            for (int k=0; k<trajectory.getDimension(); k++)
                coordinate.addDouble(waypoints[j][k]);
            wp.addList().read(coordinate);
            listWaypoints.addList().read(wp);
        }

        trajectPack.append(listWaypoints);


        outPlan.addList().read(trajectPack);
    }

    ts.update();
    setEnvelope(ts);
    write();


}

void motionPlan::receivePlan()
{
    Bottle* inPlan = read();

    if (inPlan != NULL)
    {
        Bottle* inListTrajectories = inPlan->get(0).asList();
        int numberCtrlPoints = inListTrajectories->size();
        if (inListTrajectories != NULL)
        {
            waypointTrajectory wpTraject;
            vector<Vector> trajectory;
            int numberWaypoint, numberDimension;

            for (int i=0; i<numberCtrlPoints; i++)
            {
                if (Bottle* inListTrajectory = inListTrajectories->get(i).asList())
                {
                    if (inListTrajectory->find("control-point") != NULL)
                        wpTraject.setCtrlPoint(inListTrajectory->find("control-point").asString());

                    if (int tempMsg = inListTrajectory->find("number-waypoints").asInt())
                        numberWaypoint = tempMsg;

                    if (int tempMsg = inListTrajectory->find("number-dimension").asInt())
                        numberDimension = tempMsg;

                    for (int j=0; j<numberWaypoint; j++)
                    {
                        Vector waypoint(numberDimension,0.0);
                        string wpName = "waypoint_" + NumberToString(j);
                        if (Bottle* coordinate = inListTrajectory->find(wpName).asList())
                        {
                            if (coordinate->size()==numberDimension)
                            {
                                for (int k=0; k<numberDimension; k++)
                                {
                                    waypoint[k]=coordinate->get(k).asDouble();
                                }
                                trajectory.push_back(waypoint);
                            }
                        }

                    }
                }

            }
            wpTraject.setWaypoints(trajectory);
            listTrajectory.push_back(wpTraject);
        }
    }
}



//virtual bool motionPlan::write(ConnectionWriter &connection)
//{
//    connection.appendInt(BOTTLE_TAG_LIST);
//    connection.appendInt(listTrajectory.size());
//    for (int i=0; i<listTrajectory.size(); i++)
//    {
//        connection.appendInt(BOTTLE_TAG_LIST);
//        connection.appendInt(3+listTrajectory[i].getNbWaypoint());
//    }

//    connection.convertTextMode();

//    return !connection.isError();

//}
