
#include "helpers.h"

#include "HighwayMap.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>

using std::ifstream;
using std::istringstream; 

HighwayMap::HighwayMap(string filename, int n_lanes, double lane_width, double speed_limit) : 
    n_lanes(n_lanes),
    lane_width(lane_width),
    speed_limit(speed_limit),
    circular(0.0),
    wrap(0.0),
    waypoints(5),
    spl(5)
{
    load_map(filename);
}
 
bool HighwayMap::load_map(string filename)
{
    ifstream in_map_(filename.c_str(), ifstream::in);

    // TODO check file status 

    string line;
    while (getline(in_map_, line)) {

        istringstream iss(line);
        double x, y, s, d_x, d_y;

        iss >> x >> y >> s >> d_x >> d_y;

        waypoints[Xi].push_back(x);
        waypoints[Yi].push_back(y);
        waypoints[Si].push_back(s);
        waypoints[Dxi].push_back(d_x);
        waypoints[Dyi].push_back(d_y);
    }

    // Fit all other waypoint parameters with s-coordinate values
    // This gives us a smooth curve for each position 
    // for (s,d) -> (x,y) conversion
    //
    // In reality this would be done in shorter segments
    spl[Xi].set_points(waypoints[Si], waypoints[Xi]);
    spl[Yi].set_points(waypoints[Si], waypoints[Yi]);
    spl[Dxi].set_points(waypoints[Si], waypoints[Dxi]);
    spl[Dyi].set_points(waypoints[Si], waypoints[Dyi]);

    return true;
}

vector<double> HighwayMap::frenet2cartesian(const vector<double> sd) const
{
    double s = (this->circular) ? fmod(sd[0],this->wrap) : sd[0];
    double d = sd[1];

    //std::cout << s << " " << d << " " << this->wrap <<std::endl;

    //double x = spl[Xi](s) + spl[Dxi](s) * d; 
    //double y = spl[Yi](s) + spl[Dyi](s) * d; 
    //return { x, y };

    return getXY(s, d, waypoints[Si], waypoints[Xi], waypoints[Yi]);


}


