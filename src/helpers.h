#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <math.h>
#include <vector>

using std::vector;

typedef vector<double> Pos;

struct Telemetry {
    double x;
    double y;
    double yaw;
    double s;
    double d;
    double speed;
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph2ms(double x) { return x / 2.2369; }
inline double ms2mph(double x) { return x * 2.2369; }

// euclidian distance between two points
inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// convert (x,y) from global coordinate space to car's coordinate space
// ref is car's position with global yaw angle (rad)
inline Pos global2car(Pos pos, Pos &ref)
{

    double cos_yaw = cos(-ref[2]);
    double sin_yaw = sin(-ref[2]);
    double x = pos[0] - ref[0]; 
    double y = pos[1] - ref[1]; 

    return { x*cos_yaw - y*sin_yaw, x*sin_yaw + y*cos_yaw };
}

// convert (x,y) from car's coordinate space to global coordinate space
// ref is car's position with global yaw angle (rad)
inline Pos car2global(Pos pos, Pos &ref)
{
    double cos_yaw = cos(ref[2]);
    double sin_yaw = sin(ref[2]);
    double x = pos[0]*cos_yaw - pos[1]*sin_yaw; 
    double y = pos[0]*sin_yaw + pos[1]*cos_yaw; 

    return { x + ref[0], y + ref[1] };
}

int ClosestWaypoint(double x, double y,
        const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
        const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
        const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d,
        const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);



// Get car's lane
inline int getLane(double d) { return int(d/4); }

// Get d-coordinate of the center of lane 
inline double getLaneCenter(double d) { return 2 + 4*getLane(d); }

// Get global position N meters ahead
inline vector<double> getXYAhead(double s, double n, double d, vector<vector<double>> waypoints) 
{ 
    return getXY(s+n, d, waypoints[2], waypoints[0], waypoints[1]);
}

// Evaluate quintic polynomial
inline double quintic_eval(double t, vector<double> c) 
{
    return c[0] + c[1]*t + c[2]*t*t + c[3]*t*t*t + c[4]*t*t*t*t + c[5]*t*t*t*t*t;
}  


// Jerk Minimizing Trajectory
vector<double> JMT(vector< double> start, vector <double> end, double T);

#endif
