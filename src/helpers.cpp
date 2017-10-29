#include "helpers.h"

#include "Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// Find closest waypoint in the map to point (x,y)
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

// Find next map waypoint from point (x,y) to direction theta
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_dot_dot]

        equals to [ s(0), s_dot(0), s_dot_dot(0) ]
    
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

       equals to [ s(T), s_dot(T), s_dot_dot(T) ]

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1) -> [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    // s_dot(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t**2 + 4 * a_4 * t**3 + 5 * a_5 * t**4  
    // s_dot_dot(t) = 2 * a_2 * t + 6 * a_3 * t + 12 * a_4 * t**2 + 20 * a_5 * t**3
    //
    // [ a_0, a_1, a_2 ] = [ s(0), s_dot(0), 0.5 * s_dot_dot(0) ]
    //  
    // Ax = b --> x = A**-1 * b:
    //
    //     [ T**3,   T**4,   T**5
    // A =  3T**2,  4T**3,  5T**4
    //      6T,    12T**2, 20T**3 ]
    //
    // x = [ a_3, a_4, a_5 ] ** -1
    // 
    //     [ s(T) - (s(0) + s_dot(0) * T + 0.5 * s_dot_dot(0) * T**2)
    // b =   s_dot(T) - (s_dot(0) + s_dot_dot(0) * T)
    //       s_dot_dot(T) - s_dot_dot(0) ]
    //

    double s_0 = start[0];
    double s_dot_0 = start[1];
    double s_dot_dot_0 = start[2];

    double s_T = end[0];
    double s_dot_T = end[1];
    double s_dot_dot_T = end[2];

    MatrixXd A(3,3);
    VectorXd b(3);

    vector<double> a(6, 0.0);
    a[0] = s_0;
    a[1] = s_dot_0;
    a[2] = 0.5 * s_dot_dot_0;

    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T, 5*T*T*T*T,
           6*T,  12*T*T,  20*T*T*T;
    b << s_T - (s_0 + s_dot_0 * T + 0.5 * s_dot_dot_0 * T * T),
         s_dot_T - (s_dot_0 + s_dot_dot_0 * T),
         s_dot_dot_T - s_dot_dot_0;

    //  In practice the line below corresponsd to: 
    //    MatrixXd Ai = A.inverse();
    //    VectorXd x = Ai*b;
    //  
    VectorXd x = A.colPivHouseholderQr().solve(b);

    a[3] = x[0];
    a[4] = x[1];
    a[5] = x[2];

    return a ;
    
}


