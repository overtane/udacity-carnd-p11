
#include "Vehicle.h"
#include "TrajectoryGenerator.h"
#include "HighwayMap.h"

#include <iostream>
#include <algorithm>
#include <assert.h>

using std::cout;
using std::endl;
using std::min;
using std::max;

Vehicle::Vehicle() :
    state(START),
    xy({0.0,0.0}),
    sd({0.0,0.0}),
    yaw(0.0),
    speed(0.0),
    lane(0),
    target_speed(0.0),
    target_lane(0),
    env(0),
    route(0),
    tg(0)
{}   

double Vehicle::speed_in_front(int lane) const
{
    double vx = this->env[lane].in_front[3];
    double vy = this->env[lane].in_front[4];
    return sqrt(vx*vx + vy*vy);   
}

double Vehicle::free_space_in_front(int lane) const
{
    double space = 0.0;
    int n_lanes = (this->route) ? this->route->get_n_lanes() : 1;

    if (lane >= 0 && lane < n_lanes) { 
        if (this->env[lane].in_front.size() == 0)
            space = 10E6;
        else 
            space = this->env[lane].in_front[5] - this->sd[0];
    }
    return space;
}

double Vehicle::free_space_at_behind(int lane) const
{
    double space = 0.0;
    int n_lanes = (this->route) ? this->route->get_n_lanes() : 1;

    if (lane >= 0 && lane < n_lanes) { 
        if (this->env[lane].at_behind.size() == 0)
            space = 10E6;
        else 
            space = this->sd[0] - this->env[lane].at_behind[5];
    }
    return space;
}


void Vehicle::update_env(const vector<vector<double>> &sensor_fusion)
{
    this->env.clear();

    int n_lanes = (this->route) ? this->route->get_n_lanes() : 1;

    for (int i=0; i<n_lanes; i++) {
        LaneState l;
        this->env.push_back(l);
    }

    for (int i=0; i<sensor_fusion.size(); i++) {
        int lane = (this->route) ? route->get_lane(sensor_fusion[i][6]) : 0;

        lane = min(n_lanes,max(0,lane));
        double distance =  sensor_fusion[i][5] - this->sd[0];

        assert(lane >= 0 && lane < n_lanes);
        
        if (distance < 0.0) { // at behind
            if (this->free_space_at_behind(lane) > fabs(distance)) {
                this->env[lane].at_behind = sensor_fusion[i];
            }
        } else { // in front
            if (this->free_space_in_front(lane) > distance) {
                this->env[lane].in_front = sensor_fusion[i];
            }
        }
    } 

    /*
    for (int i=0; i<this->env.size(); i++) {
        double behind = 0.0; 
        double front = 0.0;

        if (env[i].at_behind.size() > 0)
            behind = this->sd[0] - env[i].at_behind[5];
        if (env[i].in_front.size() > 0)
            front = env[i].in_front[5] - this->sd[0];

        cout << "Lane " << route->get_n_lanes()-i
             << ": at behind "  << behind
             << " m, in front " << front
             << " m " << endl; 
    }
    */
}

void Vehicle::update_status(const Telemetry &t, const vector<vector<double>> &sensor_fusion)
{
    this->xy = {t.x, t.y};
    this->sd = {t.s, t.d};
    this->yaw = t.yaw;
    this->speed = t.speed;
    this->lane = this->route->get_lane(t.d);

    update_env(sensor_fusion);
}


void Vehicle::update_state() {
     
    this->state = VehicleState::KEEP_LANE;
    /*
    double free_space = this->free_space_in_front(this->lane);

    this->target_speed = route->get_speed_limit() - 0.5;
    this->target_lane = this->lane;

    if (free_space < route->get_safety_distance() + speed) {
        // space on right
        if (this->lane >= 0 && this->lane < this->route->get_n_lanes()-1) { 
            free_space = this->free_space_in_front(this->lane+1);
            if (free_space > route->get_safety_distance()) {
                this->state = VehicleState::CHANGE_LANE_RIGHT;
                this->target_lane = this->lane+1;
            }
        } else if (this->lane > 0) {
            free_space = this->free_space_in_front(this->lane-1);
            if (free_space > route->get_safety_distance()) {
                this->state = VehicleState::CHANGE_LANE_LEFT;
                this->target_lane = this->lane-1;
            }
        }
    }
    */
    realize_keep_lane();
}

void Vehicle::realize_keep_lane()
{
    double free_space = this->free_space_in_front(this->lane);

    if (free_space > route->get_safety_distance() + speed) {
        // accelerate and keep max speed
        this->target_speed = route->get_speed_limit() - 0.25;
    } else {
        // decelerate and follow the car in the front        
        this->target_speed = this->speed_in_front(this->lane);
    }

    this->target_lane = this->lane; 
};
    
void Vehicle::realize_change_lane_left() {}

void Vehicle::realize_change_lane_right() {}


vector<vector<double>> Vehicle::new_trajectory(vector<vector<double>> prev_path)
{
    return this->tg->new_trajectory(*this, prev_path, this->route);
}
