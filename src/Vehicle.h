#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "helpers.h"

#include <vector>

using std::vector;

class HighwayMap; // Note: This should be a generic route class
class TrajectoryGenerator;

class Vehicle {

friend class TrajectoryGenerator;

public:

    enum VehicleState {
        START = 0, 
        KEEP_LANE, 
        CHANGE_LANE_LEFT,
        CHANGE_LANE_RIGHT,
        STOP
    };

private:

    struct LaneState {
        vector<double> in_front;
        vector<double> at_behind;
    };

    struct Estimate {
        int lane;
        double s;
        double speed;
        VehicleState state;
    };

protected:

    VehicleState state; // current vehicle state

    vector<double> xy; // global Cartesian position
    vector<double> sd; // Frenet position

    double yaw;   // vehicle's orientation, radians
    double speed; // current speed, m/s

    int lane; // current lane

    // latest behavior planning decisions
    double target_speed;
    int target_lane;
 
private:

    // Nearest vehicles at behind and in the front of the ego vehicle
    vector<LaneState> env; 

    HighwayMap const *route; // current route to follow


    TrajectoryGenerator *tg; 

    // all possible successor states of a vehicle state
    vector<vector<enum VehicleState>> next_states; 

private:
 
    // ---
    // Update environment (prediction) vector
    // The environment vector contains the closest vehicles in the 
    // front and at behind of the ego vehicle for each lane
    void update_env(const vector<vector<double>> &sensor_fusion);

    // ---
    // Helpers for behavior planning
    double speed_in_front(int lane) const;
    double free_space_in_front(int lane) const;
    double free_space_at_behind(int lane) const;
    double safe_speed(int lane) const;

    // ---
    // State realizer
    // Function calculates target speed and lane for
    // trajectory generator input
    void realize_state(VehicleState state);

    // --
    // Calculate the cost a state change
    double calculate_cost(const Estimate &) const;
  
    // --
    // Make rough trajectory estimation for a state within 
    // the behavior planning horizon. Behavior planner 
    // uses the estimate for cost calculations.
    Estimate make_estimate(VehicleState, double horizon) const;

public:

    Vehicle();

    ~Vehicle() {}

    void set_route(HighwayMap const *route) { this->route = route; } 
    void set_trajectory_generator(TrajectoryGenerator *tg) { this->tg = tg; } 

    // ---
    // Update vehicle and environment status with data from 
    // localization and sensor fusion
    void update_status(const Telemetry &t, 
        const vector<vector<double>> &sensor_fusion);

    // ---
    // Behavior planning
    void update_state();

    // ---
    // Generate next trajectory
    vector<vector<double>> new_trajectory(vector<vector<double>> prev_path);
};

#endif
