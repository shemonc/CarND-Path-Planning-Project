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
 * ~Vehicle() 
 *
 *  Destructor
 */
Vehicle::~Vehicle() {}

/*
 * detect_closest_vehicle
 *
 * detect the vehicles close by ego vehicle and put them in
 * @in[non_egos] vector of vehicles
 */
void
Vehicle::detect_closest_vehicle (const vector<Vehicle> &non_egos)
{

    float max_s = -1.0;
    float min_s = this->goal_s;

    /*
     * reset any previous statets 
     */
    this->nego.left = false;
    this->nego.right = false;
    this->nego.ahead = false;
    this->nego.behind = false;

    for (int i = 0; i < non_egos.size(); i++) {

        /*
         * find if this neighbour is the closet vehicle ahead of us.
         */
        float s_diff = non_egos[i].tj.s - this->tj.s;
        if (fabs(s_diff) <= TRAJECTORY_DISTANCE) {
            float d_diff = non_egos[i].tj.d - this->tj.d;

            /*
             * A vehicle is on right
             */
            if (d_diff > DESIRED_LANE_POSITION && 
                    d_diff < (DESIRED_LANE_POSITION + LANE_WIDTH)) {
                this->nego.right = true;
                //cout<<"A Vehicle on RIGHT"<<endl;
            }

            /*
             * A vehicle on left
             */
            if (d_diff < -DESIRED_LANE_POSITION && 
                     d_diff > - (DESIRED_LANE_POSITION + LANE_WIDTH)) {
                this->nego.left = true;
                //cout<<"A Vehicle on LEFT"<<endl;
            }

            /*
             * In lane vehicle either in front or in back
             */
            if (d_diff < DESIRED_LANE_POSITION && 
                                    d_diff > -DESIRED_LANE_POSITION) {
                
                /*
                 * a vehicle in front
                 */
                if (s_diff > 0 && non_egos[i].tj.s < min_s) {
                    min_s = non_egos[i].tj.s;
                    this->nego.ahead = true;
                    //cout<<"A Vehicle Ahead"<<endl;
                }
                
                /*
                 * a vehicle in back
                 */
                if (s_diff < 0 && non_egos[i].tj.s > max_s) {
                    max_s = non_egos[i].tj.s;
                    this->nego.behind = true;
                    //cout<<"A Vehicle on Behind!"<<endl;
                }
            }
        }
    }
}

/*
 * successor_states
 *
 * Provides the possible next states given the current state for the FSM
 * with the exception that lane changes happen instantaneously, so LCL 
 * and LCR can only transition back to KL.
 */
vector<car_state_e> 
Vehicle::successor_states() 
{
    vector<car_state_e> states;
    states.push_back(KL);
    car_state_e state = this->state;
    
    if (this->tj.d > LANE_WIDTH && nego.left == false ) {
            states.push_back(LCL);
    }
    if (this->tj.d < 2*LANE_WIDTH && nego.right == false) {
            states.push_back(LCR);
    }
    
    /*
     * If state is "LCL" or "LCR", then just return "KL"
     */
    return states;
}

/*
 * get_target_lane
 * 
 * Translate state into lane number.
 * i.e. if currently on lan 0 and state is LCR,
 * target lane will be 1 ect.
 */
int 
Vehicle::get_target_lane (car_state_e state)
{
    int target_lane;
    int current_lane = this->tj.d/LANE_WIDTH;

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
        int   predicted_lane = predicted_d/LANE_WIDTH;
        
        if (predicted_lane == lane) {
            
            /*
             * ego and non-ego in same lane
             */
            float predicted_start_s = predict_states[0][0];
            float predicted_end_s = predict_states[predict_states.size()-1][0];
            float predicted_end_s_d = 
                                    predict_states[predict_states.size()-1][1];
            if (predicted_end_s < closest_end_s && predicted_start_s 
                                                                > this->tj.s) {
                closest_end_s = predicted_end_s;
                closest_end_velocity = predicted_end_s_d;
            }
        }
    }

    return {closest_end_s, closest_end_velocity};
}

/*
 * get_predicted_end_states
 *
 * Given multiple states and intended_lane find the predicted end states
 * of the vehicle object(ego) after the given duration 
 *
 * @[in] state: KL, LCR, LCL, etc.
 * @[in] target_lane: intended lane for the given state
 * @[in] predictions: predictions of end states of other vehicle
 * @[in] duration: end states after this time
 */
vector<vector<double>>
Vehicle::get_predicted_end_states (car_state_e state, int target_lane,
                                   map<int, vector<vector<double>>> &predictions)
{
    float end_s, end_sd, end_sdd;
    float end_d, end_dd, end_ddd;
    vector<float> end_s_d;
    float         s_diff;

    long duration = N_SAMPLES*TIME_BETWEEN_POINTS;
    float target_d = (float)target_lane*LANE_WIDTH + DESIRED_LANE_POSITION;
 
    end_sdd = 0;
    end_sd = min(this->tj.s_dot + 0.224, SPEED_LIMIT); // mph
    
    end_s = position_at(duration, end_sd);

    end_d = (float)target_lane*LANE_WIDTH + DESIRED_LANE_POSITION;
    end_dd = 0;
    end_ddd = 0;
    
    end_s_d = get_inlane_vehicle_end_states(target_lane, predictions);
    float car_in_front_s = end_s_d[0];
    float car_in_front_velocity = end_s_d[1];
    if (car_in_front_s != -1.0 && car_in_front_velocity != -1.0) {
        
        s_diff = car_in_front_s - end_s;
        if (s_diff < TRAJECTORY_DISTANCE) {
            
            end_sd -= 0.224*2 ; //car_in_front_velocity;
            end_s = position_at(duration, end_sd);

        }
        
        /*
         * re-evaluate the distance again to make sure
         * front vehicle is not with in safety distance
         */
        s_diff = car_in_front_s - end_s;
        if (s_diff < SAFETY_DISTANCE) {
        
            /*
             * too close, slow down
             */
            cout<<" Vehicle ahead too close !"<<endl;
            //end_sd -= 0.224;
            emergency_stop += 1;
            if (emergency_stop > 1) {
                end_sd = 0;
                cout << " Emergency Stop is requested"<<endl;
            } else {
                end_sd = car_in_front_velocity;
            }

            end_s = position_at(duration, end_sd);
        } else {
            emergency_stop = 0;
        }
    }

#if 0
    s_diff = car_in_front_s - end_s;
    if (s_diff > 0 &&  s_diff < 3 ) {
        cout << "Less than a meter, Stop! "<<endl;
        //RESET coordinate
        end_sd = 0.01;
    }
#endif

    return {{end_s, end_sd, end_sdd}, {end_d, end_dd, end_ddd}};
}

/*
 * Generate_trajectory_based_on_prediction
 *
 *  A polynomial solver.
 *  Given - Start state[s,sd,sdd] 
 *        - End states[sf, sdf, sddf] and 
 *        - time, T 
 *   Generate A jerk minized trajectory
 *
 *   @[in] end_states: vectors of end states[sf, sdf, sddf]
 *   @[in] Time T:
 *   @[out] {{a0_s, a1_s, a2_s, a3_s, a4_s, a5_s},
 *                                  {a0_d, a1_d, a2_d, a3_d, a4_d, a5_d}}
 *
 *    Return vectors of longitudinal and lateral jerk minimized coefficients
 *    as in stated above in [out] parameter
 */
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

/*
 * position_at
 * 
 * Calculate the avg position of a given vehicle after the @duration
 * and @end_velocity of this vehicle
 */
float
Vehicle::position_at (long duration, float end_velocity)
{
    return (this->tj.s + ((this->tj.s_dot + end_velocity)/2)* duration);
}

/*
 * configure
 *
 * Set the Initial configuration of a vehicle
 */
void 
Vehicle::configure(float target_speed, int lane, int first_pass,
                   float max_accel, car_state_e state, float distance_to_goal,
                   int available_lane)
{
    this->target_speed = target_speed;
    this->lane = lane;
    this->first_pass = first_pass;
    max_acceleration = max_accel;
    this->state = state;
    this->lanes_available = available_lane;
    this->goal_s = distance_to_goal;
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

/* get_predictions
 *
 * Generate Trajectories for a duration of @num_of_points starting from
 * time elapsed of the @prev_points for a given vehicle.
 *
 * @[out] The prediction of the final state {s, s_dot, d, d_dot} of a given
 *        vehicle at each points
 */

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
