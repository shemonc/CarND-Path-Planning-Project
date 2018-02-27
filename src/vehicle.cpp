#include<iostream>
#include "vehicle.h"
#include "cost.h"
#include<cmath>
#include<string>
#include<map>
#include<iterator>
#include"utils.h"

/*
 * Vehicle default constructor
 */

Vehicle::Vehicle ()
{
    this->state = CS;
}

/*
 * Vehicle
 */
Vehicle::Vehicle (int lane, float s, float v, float a, car_state_e state)
{
    this->lane = lane;
    this->tj.s = s;
    this->speed = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;
}

/*
 * ~Vehicle() 
 *
 *  Destructor
 */
Vehicle::~Vehicle() {}

/*
 * detect_closest_vehicle
 *
 * Returns true if a vehicle is found on left of the current vehicle,
 * false otherwise. The passed reference rVehicle is updated if a vehicle
 * is found.
 */
void
Vehicle::detect_closest_vehicle (const vector<Vehicle> &non_egos)
{

    float max_s = -1.0;
    float min_s = this->goal_s;
    
    this->nego.left = false;
    this->nego.right = false;
    this->nego.ahead = false;
    this->nego.behind = false;

    for (int i = 0; i < non_egos.size(); i++) {

        /*
         * find if this neighbour is the closet vehicle ahead of us.
         */
        float s_diff = non_egos[i].tj.s - this->tj.s;
        if (fabs(s_diff) <= SAFTY_DISTANCE) {
            float d_diff = non_egos[i].tj.d - this->tj.d;
            if (d_diff > 2 && d_diff < (2 + 4)) {
                this->nego.right = true;
                cout<<"A Vehicle on RIGHT"<<endl;
            }
            if (d_diff < -2 && d_diff > - (2 + 4)) {
                this->nego.left = true;
                cout<<"A Vehicle on LEFT"<<endl;
            }
            if (d_diff < 2 && d_diff > -2) {
                
                if (s_diff > 0 && non_egos[i].tj.s < min_s) {
                    min_s = non_egos[i].tj.s;
                    this->nego.ahead = true;
                    cout<<"A Vehicle Ahead"<<endl;

                }

                if (s_diff < 0 && non_egos[i].tj.s > max_s) {
                    max_s = non_egos[i].tj.s;
                    this->nego.behind = true;
                    cout<<"A Vehicle on Behind!"<<endl;
                }
            }
        }
        //cout<<" "<<endl;
    }
}

/*
 * get_vehicle_ahead
 *
 * Returns a true if a vehicle is found ahead of the current vehicle,
 * false otherwise. The passed reference rVehicle is updated if a vehicle
 * is found.
 */
bool
Vehicle::get_vehicle_ahead (vector<vector<float>> predictions, int lane,
                            Vehicle &vehicle)
{

    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle neighbour;

    for (int i = 0; i < predictions.size(); i++) {
        neighbour.vehicle_id = predictions[i][0];
        neighbour.lane = get_lane(predictions[i][6]);
        if (neighbour.lane < 0) {
            cout<<"Vehicle "<<neighbour.vehicle_id<<" is in a Invalid lane ";
            cout<<neighbour.lane<<endl;
            
            /*
             * take the next vehicle
             */
            continue;
        }
        neighbour.tj.s = predictions[i][5];        
        /*
         * find if this neighbour is the closet vehicle ahead of us.
         */
        if ((neighbour.lane == this->lane) && (neighbour.tj.s > this->tj.s) &&
                                            neighbour.tj.s < min_s) {
        
            min_s = neighbour.tj.s;
            neighbour.tj.d = predictions[i][6];
            double vx = predictions[i][3];
            double vy = predictions[i][4];
            neighbour.speed = sqrt(vx*vx + vy*vy);
            vehicle = neighbour;
            found_vehicle = true;
        }
    }
    
    return found_vehicle;
}

/*
 * get_vehicle_behind
 * 
 * Returns a true if a vehicle is found behind the current vehicle, false
 * otherwise. The passed reference rVehicle is updated if a vehicle is found.
 */
bool
Vehicle::get_vehicle_behind (vector<vector<float>> predictions, int lane,
                             Vehicle &vehicle)
{
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle neighbour;

    for (int i = 0; i < predictions.size(); i++) {
        neighbour.vehicle_id = predictions[i][0];
        neighbour.lane = get_lane(predictions[i][6]);
        if (neighbour.lane < 0) {
            cout<<"Vehicle "<<neighbour.vehicle_id<<" is in a Invalid lane ";
            cout<<neighbour.lane<<endl;
            
            /*
             * take the next vehicle
             */
            continue;
        }
        neighbour.tj.s = predictions[i][5];
        if (neighbour.lane == this->lane && neighbour.tj.s < this->tj.s && 
                                            neighbour.tj.s > max_s) {
            max_s = neighbour.tj.s;
            neighbour.tj.d = predictions[i][6];
            double vx = predictions[i][3];
            double vy = predictions[i][4];
            neighbour.speed = sqrt(vx*vx + vy*vy);
            vehicle = neighbour;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

#if 1
/*
 * get_kinematics
 *
 * Gets next timestep kinematics (position, velocity, acceleration) 
 * for a given lane. Tries to choose the maximum velocity and acceleration,
 * given other vehicle positions and accel/velocity constraints.
 */
vector<float>
Vehicle::get_kinematics (vector<vector<float>> predictions, int lane)
{
    
    float   new_position;
    float   new_velocity;
    float   new_accel;
    float   max_velocity_in_front;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    float   max_velocity_accel_limit = this->max_acceleration + this->speed;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            
            /*
             * must travel at the speed of traffic, regardless of preferred
             * buffer
             */
            new_velocity = vehicle_ahead.speed;         
        } else {
            max_velocity_in_front = (vehicle_ahead.tj.s - this->tj.s
                                     - this->preferred_buffer) +
                                       vehicle_ahead.speed - 0.5 * (this->a);
            new_velocity = 
                      min(min(max_velocity_in_front, max_velocity_accel_limit),
                      this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }
    cout<<__func__<<endl;
    cout<< "max_velocity_accel_limit "<< max_velocity_accel_limit<<endl;
    cout<< "new_velocity " << new_velocity <<endl;
    new_accel = new_velocity - this->speed; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->tj.s + new_velocity + new_accel / 2.0;
    return {new_position, new_velocity, new_accel};
    
}
#endif

/*
 * successor_states
 *
 * Provides the possible next states given the current state for the FSM
 * discussed in the course, with the exception that lane changes happen
 * instantaneously, so LCL and LCR can only transition back to KL.
 */
vector<car_state_e> 
Vehicle::successor_states() 
{
    vector<car_state_e> states;
    states.push_back(KL);
    car_state_e state = this->state;
    
    if (this->tj.d > 4 && nego.left == false ) {
            states.push_back(LCL);
    }
    if (this->tj.d < 8 && nego.right == false) {
            states.push_back(LCR);
    }
    
    /*
     * If state is "LCL" or "LCR", then just return "KL"
     */
    return states;
}

#if 0
/*
 * successor_states
 *
 * Provides the possible next states given the current state for the FSM
 * discussed in the course, with the exception that lane changes happen
 * instantaneously, so LCL and LCR can only transition back to KL.
 */
vector<car_state_e> 
Vehicle::successor_states() 
{
    vector<car_state_e> states;
    states.push_back(KL);
    car_state_e state = this->state;
    if(state == KL) {
        states.push_back(PLCL);
        states.push_back(PLCR);
    } else if (state == PLCL) {
        if (lane != lanes_available - 1 && nego.left == false ) {
            //states.push_back(PLCL);
            states.push_back(LCL);
        }
    } else if (state == PLCR) {
        if (lane != 0 && nego.right == false) {
           // states.push_back(PLCR);
            states.push_back(LCR);
        }
    }
    
    /*
     * If state is "LCL" or "LCR", then just return "KL"
     */

    return states;
}
#endif

/*
 * get_target_lane
 *
 */
int 
Vehicle::get_target_lane (car_state_e state)
{
    int target_lane;
    int current_lane = this->tj.d/4;

    switch (state) {
        case KL:
            target_lane = current_lane;
            break;
        case LCL:
            target_lane = current_lane + lane_direction[LCL];
            break;
        case LCR:
             target_lane = current_lane + lane_direction[LCR];
             break;
        default:
             cout<<"Error: Unknown ego state "<<state<<endl;
    }

    return target_lane;
}

vector<float>
Vehicle::get_inlane_vehicle_end_states (const int lane,
                                 map<int, vector<vector<double>>> &predictions)
{
    float closest_end_s = -1.0;
    float closest_end_velocity = -1.0;

    closest_end_s = this->goal_s;
    for (map<int, vector<vector<double>>>::const_iterator it = 
                        predictions.begin(); it != predictions.end(); it++) {
        
        vector<vector<double>> predict_states = it->second;
        float predicted_d = predict_states[0][2];
        int   predicted_lane = predicted_d/4;
        
        if (predicted_lane == lane) {
            
            /*
             * ego and non-ego in same lane
             */
            float predicted_start_s = predict_states[0][0];
            float predicted_end_s = predict_states[predict_states.size()-1][0];
            float predicted_end_s_d = predict_states[predict_states.size()-1][1];
            if (predicted_end_s < closest_end_s && predicted_start_s > this->tj.s) {
                closest_end_s = predicted_end_s;
                closest_end_velocity = predicted_end_s_d;
            }
        }
    }

    return {closest_end_s, closest_end_velocity};
}

vector<vector<double>>
Vehicle::get_predicted_end_states (car_state_e state, int target_lane,
                                   map<int, vector<vector<double>>> &predictions)
{
    float end_s, end_sd, end_sdd;
    float end_d, end_dd, end_ddd;
    vector<float> end_s_d;

    long duration = N_SAMPLES*TIME_BETWEEN_POINTS;
    float target_d = (float)target_lane*4 + 2;
 
    end_sdd = 0;
    end_sd = min(this->tj.s_dot + 0.224, SPEED_LIMIT); // need to be in meter instead 
                                                   // mile per hour ??
    end_s = this->tj.s + ((this->tj.s_dot + end_sd)/2)* duration;
    
    end_d = (float)target_lane*4 + 2;
    end_dd = 0;
    end_ddd = 0;
    
    end_s_d = get_inlane_vehicle_end_states(target_lane, predictions);
    float car_in_front_s = end_s_d[0];
    float car_in_front_velocity = end_s_d[1];
    if (car_in_front_s != -1.0 && car_in_front_velocity != -1.0) {
    float s_diff = car_in_front_s - end_s;
    if (s_diff < SAFTY_DISTANCE) {
        
        end_sd -= 0.224*2 ; //car_in_front_velocity;
        //end_s = car_in_front_s - SAFTY_DISTANCE;
        end_s = this->tj.s + ((this->tj.s_dot + end_sd)/2)* duration;

        cout<<__func__<<" Vehicle ahead with in safty distance: ";
        cout<<SAFTY_DISTANCE<<" meter"<<endl;
        cout <<__func__<<" Front vehicle velocity: "<<end_sd<<endl;
    }
    if (s_diff < 0.5* SAFTY_DISTANCE) {
    
        /*
         * too close, slow down
         */
        cout<< __func__ <<"Vehicle ahead too close !!"<<endl;
        //end_sd -= 0.224;
        end_sd = car_in_front_velocity;
        end_s = this->tj.s + ((this->tj.s_dot + end_sd)/2)* duration; 
    }
    }
    return {{end_s, end_sd, end_sdd}, {end_d, end_dd, end_ddd}};
}

vector<vector<double>>
Vehicle::generate_trajectory_based_on_prediction (
                                const vector<vector<double>> &end_states,
                                const double T)
{
    vector<double> longitudinal_trajectory;
    vector<double> lateral_trajectory;

    vector<double> start_s = {this->tj.s, this->tj.s_dot, this->tj.s_dot_dot};
    vector<double> end_s = end_states[0];
    
    vector<double> start_d = {this->tj.d, this->tj.d_dot, this->tj.d_dot_dot};
    vector<double> end_d = end_states[1];

    /*
     * calculate optimal jerk minimization trajectory
     */
    this->s_traj_coeffs = get_jerk_min_coeffs(start_s, end_s, T); 
    this->d_traj_coeffs = get_jerk_min_coeffs(start_d, end_d, T);
    
    /*
     * Generate longitudinal and lateral trajectory for the
     * duration of each time steps
     */
    for (double i = 0.0; i < N_SAMPLES; i++) {
        double t = i* T/N_SAMPLES;
        
        /*
         * s(t) = a_0 + a_1 * t + a_2 * t**2 + 
         *                                a_3 * t**3 + a_4 * t**4 + a_5 * t**5
         */
        double s_at_t = 0;
        double d_at_t = 0;
        for (int j = 0; j < this->s_traj_coeffs.size(); j++) {
            s_at_t += this->s_traj_coeffs[j]*pow(t, j);
            d_at_t += this->d_traj_coeffs[j]*pow(t, j);
        }
        longitudinal_trajectory.push_back(s_at_t);
        lateral_trajectory.push_back(d_at_t);
    }

    return {longitudinal_trajectory, lateral_trajectory};
}

#if 0
/*
 * choose_next_state
 *
 * INPUT: A predictions map. This is a map using vehicle id as keys
 * with predicted vehicle trajectories as values. A trajectory is a vector
 * of Vehicle objects. The first item in the trajectory represents the
 * vehicle at the current timestep. The second item in the trajectory 
 * represents the vehicle one timestep in the future.
 *
 * OUTPUT: The the best (lowest cost) trajectory for the ego vehicle
 * corresponding to the next ego vehicle state.
 *
 * Functions that will be useful:
 *
 * 1. successor_states() - Uses the current state to return a vector of
 *    possible successor states for the finite state machine.
 *    
 * 2. generate_trajectory(string state, vector<vector<float>> predictions)
 *    Returns a vector of Vehicle objects representing a vehicle trajectory
 *    , given a state and predictions.
 *    Note that trajectory vectors might have size 0 if no possible
 *    trajectory exists for the state. 
 *
 * 3. calculate_cost(Vehicle vehicle, vector<vector<float>> predictions,
 *                    vector<Vehicle> trajectory) - 
 *                    Included from cost.cpp, computes the cost for a
 *                    trajectory.
 */
vector<Vehicle>
Vehicle::choose_next_state (vector<vector<float>> predictions)
{    
    /*
     * Uses the current state to return a vector of possible successor 
     * states for the finite  state machine.
     */
    vector<vector<float>> ego_end_state;

    vector<car_state_e> states = successor_states();
    int total_states = states.size();
    float cost[total_states]; 
    for (int i = 0; i < total_states; i++) {
       
        /*
         * Get taregt lane
         */
         int target_lane = get_target_lane(states[i]);
         float target_d = (float)target_lane*4 + 2;
         
        /*
         * 
         */
        ego_end_state = get_predicted_end_states(target_lane, );




        /*
         *  Generate trajectory
         *  
         *  given a state and predictions. Note that trajectory vectors  might
         *  have size 0 if no possible trajectory exists for the state
         */
        vector<Vehicle> trajectory = generate_trajectory(states[i], 
                                                          predictions);
        
        /*
         * Calculate cost
         */
        if (trajectory.size() > 0 ) {
             cost[i] = calculate_cost(*this, predictions, trajectory);
        } else {
             cost[i] = 1000000000.0;
        } 
    }
    int min_cost_index = 0;
    float min_cost = 100000.0;
    for (int i = 0; i < total_states; i++) {
        cout<<"cost["<<i<<"]"<<cost[i]<<endl;
        if (min_cost > cost[i]) {
            min_cost = cost[i];
            min_cost_index = i;
        }
    }

    cout<<"min cost "<<min_cost<<"at index "<<min_cost_index<<endl;

    return generate_trajectory(states[min_cost_index], predictions);
}
#endif

/*
 * generate_trajectory
 *
 * Given a possible next state, generate the appropriate trajectory to
 * realize the next state.
 */
vector<Vehicle>
Vehicle::generate_trajectory (car_state_e state,
                              vector<vector<float>> predictions)
{
    vector<Vehicle> trajectory;
    if (state == CS) {
        cout<<"CS trajectory"<<endl;
        trajectory = constant_speed_trajectory();
    } else if (state == KL) {
        trajectory = keep_lane_trajectory(predictions);
        cout<<"KL trajectory"<<endl;
    } else if (state == LCL || state == LCR) {
        trajectory = lane_change_trajectory(state, predictions);
        cout<<"LCL or LCR trajectory"<<endl;
    } else if (state == PLCL || state == PLCR) {
        trajectory = prep_lane_change_trajectory(state, predictions);
        cout<<"PLCL or PLCR trajectory"<<endl;
    }
    return trajectory;
}



/*
 * get_lane
 *
 */
int
Vehicle::get_lane (float d)
{
    for (int i = 0; i < TOTAL_LANE_IN_SAME_SIDE; i++) {
        
        if (d >= i*EACH_LANE_WIDTH && d <= (i+1)*EACH_LANE_WIDTH ) {
            return i;
        }
    }

    cout<<"d: "<<d<<" Lane Not FOUND"<<endl;
    return -1;
}

/*
 * increment
 *
 */
void
Vehicle::increment (int dt = 1)
{
	this->tj.s = position_at(dt);
}

/*
 * position_at
 *
 */
float
Vehicle::position_at (int t)
{
    return this->tj.s + this->speed*t + this->a*t*t/2.0;
}

/*
 * constant_speed_trajectory
 *
 * Generate a constant speed trajectory.
 */
vector<Vehicle> 
Vehicle::constant_speed_trajectory ()
{
    
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = 
            { Vehicle(this->lane, this->tj.s, this->speed, this->a, this->state),
              Vehicle(this->lane, next_pos, this->speed, 0, this->state)};
    return trajectory;
}

/*
 * keep_lane_trajectory
 *
 * Generate a keep lane trajectory.
 */
vector<Vehicle> 
Vehicle::keep_lane_trajectory (vector<vector<float>> predictions)
{
    
    vector<Vehicle> trajectory = {Vehicle(lane, this->tj.s, this->speed, this->a,
                                  state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    cout<<__func__<<endl;
    cout<<"new_v"<<new_v<<endl;

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, KL));
    return trajectory;
}

/*
 * prep_lane_change_trajectory
 *
 * Generate a trajectory preparing for a lane change.
 */
vector<Vehicle> 
Vehicle::prep_lane_change_trajectory (car_state_e state, 
                                      vector<vector<float>> predictions)
{
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = 
            {Vehicle(this->lane, this->tj.s, this->speed, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = 
                                      get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        
        /*
         * Keep speed of current lane so as not to collide with car behind.
         */
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        
        /*
         * Choose kinematics with lowest velocity.
         */
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

/*
 * lane_change_trajectory
 *
 * Generate a lane change trajectory.
 */
vector<Vehicle> 
Vehicle::lane_change_trajectory (car_state_e state,
                                 vector<vector<float>> predictions)
{
    
    /*
     * Generate a lane change trajectory.
     */
    vector<Vehicle> trajectory;
    Vehicle         next_lane_vehicle;
    int             new_lane = this->lane + lane_direction[state];
    
    /*
     * Check if a lane change is possible (check if another vehicle occupies
     * that spot).
     */
    for (int i = 0; i < predictions.size(); i++ ) { 
        next_lane_vehicle.vehicle_id = predictions[i][0];
        next_lane_vehicle.lane = get_lane(predictions[i][6]);
        if (next_lane_vehicle.lane < 0) {
            cout<<"Vehicle "<<next_lane_vehicle.vehicle_id;
            cout<<" is in a Invalid lane ";
            cout<<next_lane_vehicle.lane<<endl;
            
            /*
             * take the next vehicle
             */
            continue;
        }
        next_lane_vehicle.tj.s = predictions[i][5];

        if (next_lane_vehicle.tj.s == this->tj.s && 
                                    next_lane_vehicle.lane == new_lane) {
            
            /*
             * If lane change is not possible, return empty trajectory.
             */
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->tj.s, this->speed, this->a,
                        this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                         kinematics[2], state));
    return trajectory;
}



/*
 * configure
 *
 * Set the Initial configuration of a vehicle
 */
void 
Vehicle::configure(float target_speed, int lane, int goal_lane,
                   float max_accel, car_state_e state, float distance_to_goal,
                   int available_lane)
{
    this->target_speed = target_speed;
    this->lane = lane;
    this->goal_lane = goal_lane;
    max_acceleration = max_accel;
    this->state = state;
    this->lanes_available = available_lane;
    this->goal_s = distance_to_goal;
}

/*
 * update_localization
 * 
 */
void
Vehicle::update_localization (float x, float y, float s, float d, float yaw,
                              float speed)
{
    this->x = x;
    this->y = y;
    this->tj.s = s;
    this->tj.d = d;
    this->yaw = yaw;
    this->speed = speed;
}


void 
Vehicle::set_trajectory_param(float s, float s_d, float s_dd, 
                              float d, float d_d, float d_dd)
{
    this->tj.s = s;
    this->tj.s_dot = s_d;
    this->tj.s_dot_dot = s_dd;
    this->tj.d = d;
    this->tj.d_dot = d_d;
    this->tj.d_dot_dot = d_dd;
}

vector<vector<double>>
Vehicle::get_predictions (int prev_points, int num_of_future_points)
{
    vector<vector<double>> predict;

    float start_time = prev_points*TIME_BETWEEN_POINTS;
    float duration = num_of_future_points*TIME_BETWEEN_POINTS;
    
    for (int i = 0; i < num_of_future_points; i++) {
        float t = start_time + (duration * i/num_of_future_points);
        float s_future = this->tj.s + this->tj.s_dot * t;
        float d_future = this->tj.d + this->tj.d_dot * t;
        predict.push_back({s_future,this->tj.s_dot, d_future, this->tj.d_dot});
    }
    return predict;
}
