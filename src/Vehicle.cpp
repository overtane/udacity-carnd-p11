
#include "Vehicle.h"
#include "TrajectoryGenerator.h"
#include "HighwayMap.h"

#include <limits>
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
    tg(0),
    next_states(0)
{
    next_states.push_back({START, KEEP_LANE});                                     // START
    next_states.push_back({KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT, STOP}); // KEEP LANE
    next_states.push_back({KEEP_LANE, CHANGE_LANE_LEFT});                          // CHANGE LEFT
    next_states.push_back({KEEP_LANE, CHANGE_LANE_RIGHT});                         // CHANGE RIGHT
    next_states.push_back({STOP});                                                 // STOP 
}   

double Vehicle::speed_in_front(int lane) const
{
    double speed;
    int n_lanes = (this->route) ? this->route->get_n_lanes() : 1;
    
    if (lane < 0 || lane >= n_lanes)
        speed = 0.0;
    else if (this->env[lane].in_front.size() > 0) {
        double vx = this->env[lane].in_front[3];
        double vy = this->env[lane].in_front[4];
        speed = sqrt(vx*vx + vy*vy);
    } else
        speed = this->route->get_speed_limit();

    return speed;   
}

double Vehicle::free_space_in_front(int lane) const
{
    double space = 0.0;
    int n_lanes = (this->route) ? this->route->get_n_lanes() : 1;

    if (lane >= 0 && lane < n_lanes) { 
        if (this->env[lane].in_front.size() == 0)
            space = 1000;
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

        if (lane < 0 || lane > n_lanes) {
            // lane errors happen because of collisions etc.
            cout << "lane error: " << lane << "( d==)" << sensor_fusion[i][6] << endl;
            continue; // ignore this vehicle
        }
        
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

    /* Debug output
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


Vehicle::Estimate Vehicle::make_estimate(VehicleState state, double time) const
{
    Estimate est;

    // TODO 
    // - estimate acceleration!
    // - predict positions/trajectories

    switch (state){
        case VehicleState::START:
            est.lane = this->lane;
            est.speed = 9.9;
            break;
        case VehicleState::KEEP_LANE:
            est.lane = this->lane;
            est.speed = this->route->get_speed_limit();
            break;
        case VehicleState::CHANGE_LANE_LEFT:
            est.speed = this->route->get_speed_limit();
            est.lane = this->lane - 1;
            break;
        case VehicleState::CHANGE_LANE_RIGHT:
            est.speed = this->route->get_speed_limit();
            est.lane = this->lane + 1;
            break;
        case VehicleState::STOP:
            est.lane = this->lane;
            est.speed = max(this->speed-5.00, 0.0);
            break;
    }

    //est.s = this->sd[0] * est.speed * time;
    est.s = this->sd[0];
    est.state = state;

    return est;
}

const double COLLISION  = 10e6;
const double DANGER     = 10e5;
const double COMFORT    = 10e4;
const double EFFICIENCY = 10e2;

double Vehicle::calculate_cost(const Vehicle::Estimate &est) const 
{
    double cost = 0.0;
    double tot_cost = 0.0;
    double free_space_in_front =  this->free_space_in_front(est.lane);
    double free_space_at_behind = this->free_space_at_behind(est.lane);
    int n_lanes = route->get_n_lanes();
    double safety_margin = route->get_safety_distance();

    // Start cost 
    // Reward low speed for start state
    cost = 0.0;
    if (est.speed <= 10.0)
        if (this->speed <= 10.0)
            cost = -COMFORT;
        else
            cost = 2 * COMFORT;
    tot_cost += cost;

    // Keep lane cost
    // Reward keeping the lane, if conditions otherwise equal
    // (all lanes have traffic ahead)
    cost = 0.0;
    if (est.lane == this->lane)
        cost = -EFFICIENCY * 2;
    tot_cost += cost;


    // Road boundary cost
    // Penalize off the boundaries transitions heavily 
    cost = 0.0;
    if (est.lane >= route->get_n_lanes() || est.lane < 0.0)
        cost = COLLISION;
    tot_cost += cost;

    // Lane speed cost and free lane cost
    cost = 0.0;
    if (free_space_in_front < 75) {
       // If distance to vehicle in front is < 75m, 
       // Penalize low speed on lane
        double speed = this->speed_in_front(est.lane);
        double max_speed = this->route->get_speed_limit();
        cost = 4 * (max_speed - speed) * EFFICIENCY;
        //cout << "Lane speed cost: " << cost << endl;
    }
    tot_cost += cost;
 
    // Adjacent lane speed cost
    // Reward lane change if lane next to this one is fast
    // This should make the vehicle change lanes even if the
    // first lane change is not beneficial
    /* 
       TODO: needs more testing
    cost = 0.0;
    if (est.state == VehicleState::CHANGE_LANE_LEFT) {
       int lane = est.lane-1;
       if (lane>0 && lane<n_lanes)
           cost = -1 * min(75.0, this->free_space_in_front(lane)) * EFFICIENCY; 
    }
    if (est.state == VehicleState::CHANGE_LANE_RIGHT) {
       int lane = est.lane+1;
       if (lane>0 && lane<n_lanes)
           cost = -1 * min(75.0, this->free_space_in_front(lane)) * EFFICIENCY; 
    }
    tot_cost += cost;
    */

    // Collision costs
    // Penalize if vehicles are too close
    cost = 0.0;
    if (free_space_in_front < 6.5)
        cost += COLLISION;
    else if (free_space_in_front < safety_margin * .25)
        cost += DANGER;

    if (free_space_at_behind < 6.5)
        cost += COLLISION;

    //cout << "Collision cost: " << cost << endl;
    tot_cost += cost;


    // Free lane cost
    // Reward free space in front. Max reward when > 75
    cost = -1 * min(75.0, free_space_in_front) * EFFICIENCY; 
    //cout << "Free lane cost: " << cost << endl;
    tot_cost += cost;

    // Congestion cost
    // Penalize lanes with many vehicles
    // TODO

    return tot_cost;
}

void Vehicle::update_state() {
     
    VehicleState best_state; 
    int n_next_states = next_states[this->state].size();

    if (n_next_states == 0) {

        // This should never happen 
        best_state = this->state;

    } else if (n_next_states == 1) {
        
        // Only possible choice
        best_state = this->next_states[this->state][0];

    } else {
        // Find the state with lowest cost
        double min_cost = std::numeric_limits<double>::max();

        for (int i=0; i<next_states[this->state].size(); i++) {
            VehicleState state = this->next_states[this->state][i];

            Estimate est = this->make_estimate(state, 0.5);

            double cost = this->calculate_cost(est);

            cout << "Total cost: " << cost << " for state " << state << endl;

            if (cost < min_cost) {
                min_cost = cost;
                best_state = state;
            }
        }
    }


    if (best_state != this->state)
        cout << best_state << endl;

    this->state = best_state;
    this->realize_state(best_state);
}


void Vehicle::realize_state(VehicleState new_state)
{
    double free_space = 0.0;
    double safety_margin = min(this->route->get_safety_distance(),
                               this->speed * 1.5);
    double max_speed = route->get_speed_limit() - 0.15;

    switch(new_state) {

        case VehicleState::START:
            this->target_speed += 0.2;
            this->target_lane = this->lane;   
            break;

        case VehicleState::KEEP_LANE: {
            free_space = this->free_space_in_front(this->lane);

            if (free_space > safety_margin) {
                // accelerate and keep max speed
                this->target_speed = max_speed;
            } else if (free_space > safety_margin - safety_margin/3) {
                // decelerate and follow the car in the front        
                this->target_speed = this->speed_in_front(this->lane);  
            } else {
                // let the free space grow
                this->target_speed = this->speed_in_front(this->lane) - 3.0;
            }
            
            this->target_lane = this->lane; 
            break;
        }

        case VehicleState::CHANGE_LANE_LEFT:
            this->target_lane = this->lane - 1;
            free_space = this->free_space_in_front(this->target_lane);
            if (free_space < safety_margin/3)
                this->target_speed = this->speed_in_front(this->target_lane) - 4.0;
            else
                this->target_speed = max_speed;
            break;

        case VehicleState::CHANGE_LANE_RIGHT:
            this->target_lane = this->lane + 1;
            free_space = this->free_space_in_front(this->target_lane);
            if (free_space < safety_margin/3)
                this->target_speed = this->speed_in_front(this->target_lane) - 4.0;
            else
                this->target_speed = max_speed;
            break;

        case VehicleState::STOP:
            this->target_speed = 0;
            this->target_lane = this->lane;
            break;

        default:
            break;
    }

}

vector<vector<double>> Vehicle::new_trajectory(vector<vector<double>> prev_path)
{
    return this->tg->new_trajectory(*this, prev_path, this->route);
}
