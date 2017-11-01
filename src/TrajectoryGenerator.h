#ifndef _TRAJECTORY_GENERATOR_H_ 
#define _TRAJECTORY_GENERATOR_H_ 

#include "HighwayMap.h"
#include "Vehicle.h"

#include <vector>

using std::vector;

class TrajectoryGenerator{

private:

    vector<vector<double>> t; // previous generated trajectory

public:

    TrajectoryGenerator() : t(2) {}

    ~TrajectoryGenerator() {}

    // ---
    // Generate a new trajectory for a vehicle along a route
    // - 'car' contains the current location information
    //   and target speed and lane
    // - 'prev_path' contains the unused points of the previous trajectory
    //   TODO: we do not need this because member variable t 
    //   has the same information. Only need to know how many points
    //   we have left of the previous trajectory
    // - 'route' contains map waypoints that the vehicle should follow
    //   and road information.
    //
    // - vectors of x- and y-coordinates (global) that car should follow
    //   spacing of the points determine the speed of the vehicle.
    vector<vector<double>> new_trajectory(
        const Vehicle &car,
        vector<vector<double>> prev_path,
        const HighwayMap *route);
};

#endif

