#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "HighwayMap.h"

#include <vector>

using std::vector;

vector<vector<double>> plan(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const HighwayMap &hwmap);                   // IN

//vector<vector<double>> plan(
//        const Telemetry &car,                        // IN
//        const vector<vector<double>> &prev_path,     // IN
//        const vector<double> &end_path,              // IN
//        const vector<vector<double>> &sensor_fusion, // IN
//        const vector<vector<double>> &waypoints);    // IN

#endif
