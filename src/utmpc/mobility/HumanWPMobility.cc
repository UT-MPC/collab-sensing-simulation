#include "HumanWPMobility.h"

#include <fstream>
#include <iostream>

#include "../ndlayer/BlendAgent.h"
#include "inet/common/geometry/common/Quaternion.h"

using inet::Coord;
using inet::GeoCoord;
using inet::IGeographicCoordinateSystem;
using inet::INITSTAGE_LOCAL;
using inet::INITSTAGE_PHYSICAL_ENVIRONMENT;
using inet::physicalenvironment::IGround;
using inet::Quaternion;
using omnetpp::cStringTokenizer;
using std::endl;

namespace scentssim {

Define_Module(HumanWPMobility);

void HumanWPMobility::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);
    EV_TRACE << "initializing HumanWPMobility stage " << stage << endl;
    if (stage == INITSTAGE_LOCAL) {
        speed = par("speed");
        waypointProximity = par("waypointProximity");
        targetPointIndex = 0;
        heading = 0;
        angularSpeed = 0;
    }
    else if (stage == INITSTAGE_PHYSICAL_ENVIRONMENT) {
        readWaypointsFromFile(par("waypointFile"));
    }
}

void HumanWPMobility::setInitialPosition()
{
    lastPosition.x = waypoints[targetPointIndex].x;
    lastPosition.y = waypoints[targetPointIndex].y;
    lastVelocity.x = speed * cos(M_PI * heading / 180);
    lastVelocity.y = speed * sin(M_PI * heading / 180);
}

void HumanWPMobility::readWaypointsFromFile(const char *fileName)
{
    EV_DEBUG << "readWaypointsFromFile:" << fileName << endl;
    geographic_coordinate_system_ = inet::getModuleFromPar<IGeographicCoordinateSystem>(par("coordinateSystemModule"), this, false);
    char line[256];
    std::ifstream inputFile(fileName);
    while (true) {
        inputFile.getline(line, 256);
        if (!inputFile.fail()) {
            cStringTokenizer tokenizer(line, ",");
            Coord playgroundCoordinate;
            double value1 = atof(tokenizer.nextToken());
            double value2 = atof(tokenizer.nextToken());
            double value3 = 0.5;
            double x;
            double y;
            double z;
            if (geographic_coordinate_system_ == nullptr) {
                x = value1;
                y = value2;
                z = value3;
            }
            else {
                Coord playgroundCoordinate = geographic_coordinate_system_->computeSceneCoordinate(GeoCoord(inet::deg(value1), inet::deg(value2), inet::m(value3)));
                x = playgroundCoordinate.x;
                y = playgroundCoordinate.y;
                z = playgroundCoordinate.z;
            }
            waypoints.push_back(Waypoint(x, y, z));
        }
        else
            break;
    }
}

void HumanWPMobility::move()
{
    Waypoint target = waypoints[targetPointIndex];
    double dx = target.x - lastPosition.x;
    double dy = target.y - lastPosition.y;
    if (dx * dx + dy * dy < waypointProximity * waypointProximity)  // reached so change to next (within the predefined proximity of the waypoint)
        targetPointIndex = (targetPointIndex + 1) % waypoints.size();
    double targetDirection = atan2(dy, dx) / M_PI * 180;
    double diff = targetDirection - heading;
    while (diff < -180)
        diff += 360;
    while (diff > 180)
        diff -= 360;
    angularSpeed = diff * 5;
    double timeStep = (omnetpp::simTime() - lastUpdate).dbl();
    heading += angularSpeed * timeStep;

    Coord tempSpeed = Coord(cos(M_PI * heading / 180), sin(M_PI * heading / 180)) * speed;
    Coord tempPosition = lastPosition + tempSpeed * timeStep;

    lastVelocity = tempPosition - lastPosition;
    lastPosition = tempPosition;
}

} // namespace scentssim
