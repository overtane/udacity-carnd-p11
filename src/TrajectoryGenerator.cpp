
#include "TrajectoryGenerator.h"
#include "Vehicle.h"
#include "HighwayMap.h"

#include "spline.h"

#include <algorithm>
#include <iostream>

using std::cout;
using std::endl;
using std::min;
using tk::spline;

vector<vector<double>> TrajectoryGenerator::new_trajectory(
    const Vehicle &car,
    vector<vector<double>> prev_path, 
    const HighwayMap *route)
{
    const int X = 0;
    const int Y = 1;
    const int YAW = 2;

    // initialize next path with remaining points from previous path
    vector<vector<double>> next_path {prev_path[X], prev_path[Y]};

    Pos ref; // reference position for coordinate transformation
    //int path_size = min(50-remaining+10, remaining);
    int path_size = prev_path[X].size();

    cout << path_size << endl;

    vector<vector<double>> spts(2); // spline points

    // points for spline generation
    if (path_size < 2) {
        ref.push_back(car.xy[X]);
        ref.push_back(car.xy[Y]);
        ref.push_back(deg2rad(car.yaw));

        double x2 = car.xy[X] - cos(car.yaw);
        double y2 = car.xy[Y] - sin(car.yaw);
        vector<double> prev = global2car( {x2,y2}, ref );
        spts[X].push_back(prev[X]);
        spts[Y].push_back(prev[Y]);
        spts[X].push_back(0.0);
        spts[Y].push_back(0.0);
    } else {
        ref.push_back(prev_path[X][path_size-1]);
        ref.push_back(prev_path[Y][path_size-1]);

        double x2 = prev_path[X][path_size-2];
        double y2 = prev_path[Y][path_size-2];
        ref.push_back(atan2(ref[Y]-y2,ref[X]-x2)); // ref point yaw

        vector<double> prev = global2car( {x2, y2}, ref);
        spts[X].push_back(prev[X]);
        spts[Y].push_back(prev[Y]);
        spts[X].push_back(0.0);
        spts[Y].push_back(0.0);
    }

    double center = route->get_lane_center(car.lane); // current lane
    cout << "car:" << car.sd[0] << ", " << car.sd[1] << endl;

    vector<double> next0 = route->frenet2cartesian( {car.sd[0]+30, center} );
    cout << next0[X] << " " << next0[Y] << endl;
    next0 = global2car(next0, ref);
    cout << next0[X] << " " << next0[Y] << endl;
    spts[X].push_back(next0[X]);
    spts[Y].push_back(next0[Y]);

    vector<double> next1 = route->frenet2cartesian( {car.sd[0]+60, center});
    cout << next1[X] << " " << next1[Y] << endl;
    next1 = global2car(next1, ref);
    cout << next1[X] << " " << next1[Y] << endl;
    spts[X].push_back(next1[X]);
    spts[Y].push_back(next1[Y]);

    vector<double> next2 = route->frenet2cartesian( {car.sd[0]+90, center} );
    cout << next2[X] << " " << next2[Y] << endl;
    next2 = global2car(next2, ref);
    cout << next2[X] << " " << next2[Y] << endl;
    spts[X].push_back(next2[X]);
    spts[Y].push_back(next2[Y]);

    spline sp;
    sp.set_points(spts[X], spts[Y]);

    cout << "---" << endl;

    // spline spacing
    double target_x = 30.0;
    double target_y = sp(target_x);
        // note: straight line distance
    double target_dist = sqrt(target_x*target_x + target_y*target_y);  

    //for (int i = 50-remaining; i<path_size; i++) {
    //    next_path[X].push_back(this->t[X][i]);
    //    next_path[Y].push_back(this->t[Y][i]);
    //    cout << "old point: " << this->t[X][i] << " " << this->t[Y][i] << endl;
    //} 

    // add new points to next path
    double prev_x = 0.0;
    for (int i = 0; i < 50-path_size; i++) {    
        double N = target_dist/(.02*car.target_speed); // distance for 20ms at 48 MPH
        double x = prev_x + target_x/N;
        double y = sp(x);

        prev_x = x;

        //cout << "spline point: " << x << " " << y << endl;
        vector<double> pt = car2global( {x,y}, ref);
        cout << "path point: " << pt[X] << " " << pt[Y] << endl;

        next_path[X].push_back(pt[X]);
        next_path[Y].push_back(pt[Y]);
    }

    this->t[X].clear();
    this->t[Y].clear();
    this->t[X] = next_path[X];
    this->t[Y] = next_path[Y];

    cout << this->t[X].size() << endl;

    return next_path;

}

