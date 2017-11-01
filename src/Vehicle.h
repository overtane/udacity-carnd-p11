#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "helpers.h"

#include <vector>

using std::vector;

class HighwayMap; // This should be a generic route class
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

    VehicleState state; // vehicle state

    vector<double> xy; // global Cartesian position
    vector<double> sd; // Frenet position

    double yaw;   // vehicle's orientation, radians
    double speed; // current speed, m/s

    int lane; // current lane

    double target_speed; // latest behavior planning decisions
    int target_lane;
 
private:

    vector<LaneState> env; // nearest vehicles at behind and in the front

    HighwayMap const *route; // current route to follow

    TrajectoryGenerator *tg;

    vector<vector<enum VehicleState>> next_states; // possible successor states

private:

    // update environment vector
    // environment vector contains the closest vehicles in the 
    // front and at behind of the ego vehicle for each lane
    void update_env(const vector<vector<double>> &sensor_fusion);

    double speed_in_front(int lane) const;
    double free_space_in_front(int lane) const;
    double free_space_at_behind(int lane) const;
        
    void realize_state(VehicleState state);

    double calculate_cost(const Estimate &) const;
  
    Estimate make_estimate(VehicleState, double time) const;

public:

    Vehicle();

    ~Vehicle() {}

    VehicleState get_state() const { return KEEP_LANE; }

    void set_route(HighwayMap const *route) { this->route = route; } 
    void set_trajectory_generator(TrajectoryGenerator *tg) { this->tg = tg; } 

    // update vehicle and environment status 
    void update_status(const Telemetry &t, const vector<vector<double>> &sensor_fusion);

    // behavior planning
    void update_state();

    // generate next trajectory
    vector<vector<double>> new_trajectory(vector<vector<double>> prev_path);
};

#endif
