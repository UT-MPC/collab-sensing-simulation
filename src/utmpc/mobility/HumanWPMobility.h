#ifndef UTMPC_MOBILITY_HUMANWPMOBILITY_H_
#define UTMPC_MOBILITY_HUMANWPMOBILITY_H_

#include "inet/common/geometry/common/GeographicCoordinateSystem.h"
#include "inet/environment/contract/IGround.h"
#include "inet/mobility/base/MovingMobilityBase.h"

namespace scentssim {

class HumanWPMobility: public inet::MovingMobilityBase {
protected:
    struct Waypoint {
        double x;
        double y;
        double timestamp;

        Waypoint(double x, double y, double timestamp) :
                x(x), y(y), timestamp(timestamp) {
        }
    };
    std::vector<Waypoint> waypoints;

    double speed;
    double heading;
    double waypointProximity;
    double angularSpeed;
    int targetPointIndex;

    virtual void initialize(int stage) override;
    virtual void setInitialPosition() override;
    virtual void move() override;

    virtual void readWaypointsFromFile(const char *fileName);

public:
    virtual double getMaxSpeed() const override {
        return speed;
    }
    inet::IGeographicCoordinateSystem* get_geographic_coordinate_system() const {
        return geographic_coordinate_system_;
    }
private:
    inet::IGeographicCoordinateSystem* geographic_coordinate_system_;

};

} // End of namespace scentssim

#endif /* UTMPC_MOBILITY_HUMANWPMOBILITY_H_ */
