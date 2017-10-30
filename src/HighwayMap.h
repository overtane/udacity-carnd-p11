#ifndef _HIGHWAY_MAP_H_
#define _HIGHWAY_MAP_H_

#include "spline.h"

#include <vector>
#include <string>

using std::vector;
using std::string;
using tk::spline;


class HighwayMap {

private:

    // number of lanes
    int n_lanes;
    
    // lane width in meters
    double lane_width;

    // speed limit in meters per second
    double speed_limit;

    // max s value for a circular track
    double wrap;


    // Indices to waypoints and spl vectors
    enum WP_Indices { Xi, Yi, Si, Dxi, Dyi };
 
    // Loaded waypoints defining the road curve
    // Waypoints represent the center of the road. Drivable lanes are to the right
    vector<vector<double>> waypoints;

    // Smoothed road
    vector<spline> spl;

    // default constructor not used
    HighwayMap() {};


public:
    
    // Constructor: set road parameters and load map waypoints
    HighwayMap(string filename, int n_lanes, double lane_width, double speed_limit);
    
    ~HighwayMap() {};

    // Load highway map waypoints from a file
    bool load_map(string filename);

    void set_n_lanes(int n) { n_lanes = n; }
    void set_speed_limit_mph(double mph) { speed_limit = mph / 2.239; }
    void set_circular(double max_s) { wrap = wrap; }

    int get_n_lanes() const { return n_lanes; }
    int get_lane(double d) const { return int(d/this->lane_width); }
    double get_lane_center(int lane) const { return lane * this->lane_width + this->lane_width/2; }
    double get_speed_limit() const { return speed_limit; }
    double get_safety_distance() const { return speed_limit * .75; }
    bool is_circular() const { return fabs(wrap) < 1; }
  
    // Convert a Frenet (s,d) position to Cartesian (x,y) position 
    vector<double> frenet2cartesian(const vector<double> frenet_position) const;

};

#endif

