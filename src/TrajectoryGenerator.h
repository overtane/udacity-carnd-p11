#ifndef _TRAJECTORY_GENERATOR_H_ 
#define _TRAJECTORY_GENERATOR_H_ 

#include "HighwayMap.h"
#include "Vehicle.h"

#include <vector>

using std::vector;

class TrajectoryGenerator{

private:

    vector<vector<double>> t; // generated trajectory

public:

    TrajectoryGenerator() : t(2) {}

    ~TrajectoryGenerator() {}

    vector<vector<double>> new_trajectory(
        const Vehicle &car, vector<vector<double>> prev_path, const HighwayMap *route);
};

#endif

