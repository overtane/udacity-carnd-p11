
#include "spline.h"
#include "helpers.h"
#include "path_planner.h"
#include "HighwayMap.h"

#include <iostream>
#include <tuple>

// Note:
// 1 MPH  = 0.447 m/s
// 1 m/s  = 2.2369 MPH
// 50 MPH = 22.352 m/s -> 0.447 m / 20ms
// 48 MPH = 0.429 m / 20 m/s (this is the target velocity

// Sensor fusion data:
// The data format for each car is: [ id, x, y, vx, vy, s, d]. 
// - id is a unique identifier for that car. 
// - x, y values are in global map coordinates,
// - vx, vy values are the velocity components, also in reference to the global map. 
// - s,d are the Frenet coordinates for that car.

using std::vector;
using std::cout;
using std::endl;

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

    double dist_inc = 0.429;
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

    double dist_inc = 0.429;
    for (int i = 0; i < 50-path_size; i++) {    
        next_path[X].push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
        next_path[Y].push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
        pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
        pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    }

    return next_path;
    
}

// ----
// Test planner C (from walkthrough)
// Keep the current lane at constant speed 

static vector<vector<double>> planC(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<double> &end_path,              // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const vector<vector<double>> &waypoints)     // IN
{
    const int X = 0;
    const int Y = 1;
    const int YAW = 2;

    // initialize next path with remaining points from previous path
    vector<vector<double>> next_path {prev_path[X], prev_path[Y]};

    Pos ref; // reference position for coordinate transformation
    int path_size = prev_path[X].size();

    cout << path_size << endl;

    vector<vector<double>> spts(2); // spline points

    // points for spline generation
    if (path_size < 2) {
        ref.push_back(car.x);
        ref.push_back(car.y);
        ref.push_back(deg2rad(car.yaw));

        double x2 = car.x - cos(car.yaw);
        double y2 = car.y - sin(car.yaw);
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

    double center = getLaneCenter(car.d); // current lane
    cout << "lane center: " << center << ", car d: " << car.d << endl;

    vector<double> next0 = getXYAhead(car.s, 30, center, waypoints);
    cout << next0[X] << " " << next0[Y] << endl;
    next0 = global2car(next0, ref);
    cout << next0[X] << " " << next0[Y] << endl;
    spts[X].push_back(next0[X]);
    spts[Y].push_back(next0[Y]);

    vector<double> next1 = getXYAhead(car.s, 60, center, waypoints);
    cout << next1[X] << " " << next1[Y] << endl;
    next1 = global2car(next1, ref);
    cout << next1[X] << " " << next1[Y] << endl;
    spts[X].push_back(next1[X]);
    spts[Y].push_back(next1[Y]);

    vector<double> next2 = getXYAhead(car.s, 90, center, waypoints);
    cout << next2[X] << " " << next2[Y] << endl;
    next2 = global2car(next2, ref);
    cout << next2[X] << " " << next2[Y] << endl;
    spts[X].push_back(next2[X]);
    spts[Y].push_back(next2[Y]);

    tk::spline sp;
    sp.set_points(spts[X], spts[Y]);

    cout << "---" << endl;

    // spline spacing
    double target_x = 30.0;
    double target_y = sp(target_x);
        // note: straight line distance
    double target_dist = sqrt(target_x*target_x + target_y*target_y);  

    // add new points to next path
    double prev_x = 0.0;
    for (int i = 0; i < 50-path_size; i++) {    
        double N = target_dist/(.02*48/2.24); // distance for 20ms at 48 MPH
        double x = prev_x + target_x/N;
        double y = sp(x);

        prev_x = x;

        cout << "spline point: " << x << " " << y << endl;
        vector<double> pt = car2global( {x,y}, ref);
        cout << "path point: " << pt[X] << " " << pt[Y] << endl;

        next_path[X].push_back(pt[X]);
        next_path[Y].push_back(pt[Y]);
    }

    return next_path;
}

// ----
// Test planner D 
// Keep the current lane at constant speed.
// Use quintic polynomial trajectories

static vector<vector<double>> planD(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const HighwayMap &hwmap)                     // IN
{
    // Point indices
    const int X = 0;
    const int Y = 1;

    static double last_s = 0.0;
    static double last_d = 0.0;
    static double last_v = 0.0;

    // initialize next path with remaining points from previous path
    vector<vector<double>> next_path {prev_path[X], prev_path[Y]};

    int prev_size = prev_path[X].size();

    //cout << prev_size << endl;

    double s;
    double d;
    double v; 
    if (prev_size < 2) {
        s = car.s;
        d = car.d;
        v = car.speed;
    } else {
        s = last_s;
        d = last_d;
        //double x1 = prev_path[X][prev_size-1];
        //double x2 = prev_path[X][prev_size-2];
        //double y1 = prev_path[Y][prev_size-1];
        //double y2 = prev_path[Y][prev_size-2];
        //v = std::min(mph2ms(49), distance(x1,y1,x2,y2)/0.02); // velocity 
        v = std::min(last_v, mph2ms(49));
    }

    vector<double> s_start = { s, v, 0.0 };
    vector<double> s_end   = { s+75, mph2ms(49), 0.0 };    
    vector<double> s_coeffs = JMT(s_start, s_end, 4); 

    //cout << v << " - " 
    //     << s_coeffs[0] << ", " 
    //     << s_coeffs[1] << ", "
    //     << s_coeffs[2] << ", "
    //     << s_coeffs[3] << ", "
    //     << s_coeffs[4] << ", "
    //     << s_coeffs[5] << ", "
    //     << endl;

    vector<double> d_start = { d, 0.0, 0.0 };
    vector<double> d_end   = { getLaneCenter(d), 0.0, 0.0 };    
    vector<double> d_coeffs = JMT(d_start, d_end, 4);

    //cout << d_coeffs[0] << ", " 
    //     << d_coeffs[1] << ", "
    //     << d_coeffs[2] << ", "
    //     << d_coeffs[3] << ", "
    //     << d_coeffs[4] << ", "
    //     << d_coeffs[5] << ", "
    //     << endl;

    // add new points to next path
    double t = 0.02;

    for (int i = 0; i < 50 - prev_size; i++) {    

        double s = quintic_eval(t, s_coeffs);
        double d = quintic_eval(t, d_coeffs);

	vector<double> xy = hwmap.frenet2cartesian( {s,d} );

        int n = next_path[X].size();
        next_path[X].push_back(xy[X]);
        next_path[Y].push_back(xy[Y]);

        n++;
        if (n > 1) {
            double x1 = next_path[X][n-2];
            double x2 = next_path[X][n-1];
            double y1 = next_path[Y][n-2];
            double y2 = next_path[Y][n-1];
            last_v = distance(x1,y1,x2,y2)/0.02; // velocity in global coordinates
        } else {
            last_v = (s - last_s)/0.02;
        }
      
        //cout << "(" << xy[X] << "," << xy[Y] << ") ("
        //     << s << "," << d << ")  " 
        //     << ms2mph(last_v) 
        //     << endl;

        //cout << (s-last_s) << "  "
        //     << (d-last_d) << "  "
        //     << ms2mph(last_v)
        //     << endl;

        last_s = s;
        last_d = d;

        t += 0.02;
    }

    return next_path;
}

vector<vector<double>> plan(
        const Telemetry &car,                        // IN
        const vector<vector<double>> &prev_path,     // IN
        const vector<vector<double>> &sensor_fusion, // IN
        const HighwayMap &hwmap)                     // IN
{
	return planD(car, prev_path, sensor_fusion, hwmap);
}
