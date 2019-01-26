package scentssim.utmpc.mobility;

import inet.mobility.base.MovingMobilityBase;

simple HumanWPMobility extends MovingMobilityBase {
        @class(HumanWPMobility);
        @signal[mobilityStateChanged](type=scentssim::HumanWPMobility);
        string waypointFile;
        double waypointProximity @unit(m);
        double speed @unit(mps);        
        double startTime @unit("s") = default(0s);   // time when the movement starts
        double initialX @unit(m) = default(uniform(constraintAreaMinX, constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(constraintAreaMinY, constraintAreaMaxY));
        double initialZ @unit(m) = default(nanToZero(uniform(constraintAreaMinZ, constraintAreaMaxZ)));
        bool initFromDisplayString = default(false);
}