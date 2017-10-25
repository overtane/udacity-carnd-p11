
#include "helpers.h"
#include "path_planner.h"

#include <iostream>

// Note:
// 1 MPH  = 0.447 m/s
// 1 m/s  = 2.2369 MPH
// 50 MPH = 22.352 m/s 


// Sensor fusion data:
// The data format for each car is: [ id, x, y, vx, vy, s, d]. 
// - id is a unique identifier for that car. 
// - x, y values are in global map coordinates,
// - vx, vy values are the velocity components, also in reference to the global map. 
// - s,d are the Frenet coordinates for that car.

using namespace std;

// ----
// Test planner a from  classroom
// Generate a straight 50 points path with 0.5m increments (both x and y directions)
static vector<vector<double>> planA(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<double> &end_path,              // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const vector<vector<double>> &waypoints)     // IN
{
    vector<vector<double>> next_path(2);

    double dist_inc = 0.43;
    for(int i = 0; i < 50; i++)
    {
          next_path[0].push_back(car.x+(dist_inc*i)*cos(deg2rad(car.yaw)));
          next_path[1].push_back(car.y+(dist_inc*i)*sin(deg2rad(car.yaw)));
    }
 
    return next_path;
}

// ----
// Test planner B from classroom
// Generate a circular 50 points path with 0.5m increments (both x and y directions)
// use the tail of remaining previous path as starting point and generate only
// the very tail of the path
static vector<vector<double>> planB(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<double> &end_path,              // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const vector<vector<double>> &waypoints)     // IN
{
    const int X = 0;
    const int Y = 1;

    vector<vector<double>> next_path {prev_path[X], prev_path[Y]};

    double pos_x;
    double pos_y;
    double angle;
    int path_size = prev_path[X].size();

    if (path_size == 0) {
        pos_x = car.x;
        pos_y = car.y;
        angle = deg2rad(car.yaw);
    } else {
        pos_x = prev_path[X][path_size-1];
        pos_y = prev_path[Y][path_size-1];

        double pos_x2 = prev_path[X][path_size-2];
        double pos_y2 = prev_path[Y][path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    cout << path_size << endl;

    double dist_inc = 0.42;
    for (int i = 0; i < 50-path_size; i++) {    
        next_path[X].push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
        next_path[Y].push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
        pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
        pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    }

    return next_path;
    
}

vector<vector<double>> plan(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<double> &end_path,              // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const vector<vector<double>> &waypoints)     // IN
{
    return planB(car, prev_path, end_path, sensor_fusion, waypoints);
}
