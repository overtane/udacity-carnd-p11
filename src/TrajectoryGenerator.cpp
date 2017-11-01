
#include "TrajectoryGenerator.h"
#include "Vehicle.h"
#include "HighwayMap.h"

#include "spline.h"

#include <algorithm>
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;
using std::min;
using std::max;
using std::log;
using tk::spline;


// logarithmic acceleration, with max acc in the beginning
static double acc1(double max_acc, double speed, double target_speed)
{
  // TODO: rework this
  if (fabs(speed) < 1)
      return max_acc;
  else
      return max_acc - max_acc * log(speed)/log(target_speed);
}

vector<vector<double>> TrajectoryGenerator::new_trajectory(
    const Vehicle &car,
    vector<vector<double>> prev_path, 
    const HighwayMap *route)
{
    const int X = 0;
    const int Y = 1;

    // initialize next path with remaining points from previous path
    vector<vector<double>> next_path(2);

    Pos ref; // reference position for coordinate transformation
    int path_size = prev_path[X].size();
    // amount to reuse from previous trajectory
    int reuse_size = min(10,path_size);

    // We reuse max 10 points of previous path
    // - This gives reaction time of 0.2s
    // - This is at max speed about 4 meters
    int t_size = this->t[X].size();
    int first_reuse = max(0, t_size - path_size);
    for (int i = first_reuse; i<t_size && i<first_reuse+reuse_size; i++) {
        next_path[X].push_back(this->t[X][i]);
        next_path[Y].push_back(this->t[Y][i]);
    } 

    vector<vector<double>> spts(2); // spline points

    // starting points for spline generation
    if (path_size < 2) {
        ref.push_back(car.xy[X]);
        ref.push_back(car.xy[Y]);
        ref.push_back(deg2rad(car.yaw));
        spts[X].push_back(0.0);
        spts[Y].push_back(0.0);
    } else {
        ref.push_back(next_path[X][reuse_size-1]);
        ref.push_back(next_path[Y][reuse_size-1]);

        double x2 = next_path[X][reuse_size-2];
        double y2 = next_path[Y][reuse_size-2];
        ref.push_back(atan2(ref[Y]-y2,ref[X]-x2)); // ref point yaw

        vector<double> prev = global2car( {x2, y2}, ref);
        spts[X].push_back(prev[X]);
        spts[Y].push_back(prev[Y]);
        spts[X].push_back(0.0);
        spts[Y].push_back(0.0);
    }
 
    int target_lane = car.target_lane;

    double center = route->get_lane_center(target_lane); 

    vector<double> next0 = route->frenet2cartesian( {car.sd[0]+40, center} );
    next0 = global2car(next0, ref);
    spts[X].push_back(next0[X]);
    spts[Y].push_back(next0[Y]);

    vector<double> next1 = route->frenet2cartesian( {car.sd[0]+80, center});
    next1 = global2car(next1, ref);
    spts[X].push_back(next1[X]);
    spts[Y].push_back(next1[Y]);

    // generate spline here
    spline sp;
    sp.set_points(spts[X], spts[Y]);

    cout << "Target speed:" << car.target_speed << endl;

    // spline spacing
    double target_x = 40.0;
    double target_y = sp(target_x);
    // note: straight line distance
    double target_dist = sqrt(target_x*target_x + target_y*target_y);  

    // add new points to next path
    double max_acc = 5.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    double prev_speed = (path_size<2) ? car.speed : 
                        sqrt(spts[X][0]*spts[X][0]+spts[Y][0]*spts[Y][0])/0.02;
    // TODO: calculate prev_acc
    double acc = 0.0;

    for (int i = 0; i < 50-reuse_size; i++) {    
        double x, y;

        if (car.state != Vehicle::VehicleState::START) {
            // accelerate/decelerate  
            acc = acc1(max_acc, prev_speed, car.target_speed);
            if (prev_speed > car.target_speed && acc > 0.0)
                acc = 0.0;
            x = prev_x + prev_speed * .02 + .5 * .004 * acc;
            y = sp(x);
            prev_speed = sqrt((x-prev_x)*(x-prev_x) + (y-prev_y)*(y-prev_y))/.02;
        } else {
            //constant speed
            double N = target_dist/(.02*car.target_speed);
            x = prev_x + target_x/N;
            y = sp(x);
        }

        prev_x = x;
        prev_y = y;

        vector<double> pt = car2global( {x,y}, ref);
        next_path[X].push_back(pt[X]);
        next_path[Y].push_back(pt[Y]);
    }

    this->t[X].clear();
    this->t[Y].clear();
    this->t[X] = next_path[X];
    this->t[Y] = next_path[Y];

    return next_path;
}

