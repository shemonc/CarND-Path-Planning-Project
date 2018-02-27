#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "utils.h"


const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);

/*
 * The weighted cost over all cost functions is computed in calculate_cost.
 * The data from get_helper_data will be very useful in your implementation
 * of the cost functions below. Please see get_helper_data for details on how
 * the helper data is computed. 
 */

float 
goal_distance_cost (const Vehicle & vehicle, const vector<Vehicle> &trajectory,
                    const vector<vector<float>> &predictions,
                    map<string, float> &data)
{
    /*
     * Cost increases based on distance of intended lane (for planning a lane
     * change) and final lane of trajectory. 
     * Cost of being out of goal lane also becomes larger as vehicle approaches
     * goal distance. This function is very similar to what you have already
     * implemented in the "Implement a Cost Function in C++" quiz.
     */
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] \
                        - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;
}

/*
 * inefficiency_cost
 *
 */
float
inefficiency_cost (const Vehicle &vehicle, const vector<Vehicle> &trajectory,
                   const vector<vector<float>> &predictions,
                   map<string, float> & data)
{
    /*
     * Cost becomes higher for trajectories with intended lane and final lane
     * that have traffic slower than vehicle's target speed. You can use the 
     * lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) 
     * function to determine the speed for a lane. 
     *
     * This function is very similar to what you have already implemented in
     * the "Implement a Second Cost Function in C++" quiz.
     */

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    
    /*
     * If no vehicle is in the proposed lane, we can travel at target speed.
     */
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }
    
    float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
                    - proposed_speed_final)/vehicle.target_speed;

    return cost;
}

/*
 * lane_speed
 *
 */
float 
lane_speed (const vector<vector<float>>& predictions, int lane)
{
    /*
     * All non ego vehicles in a lane have the same speed, so to get the speed
     * limit for a lane,we can just find one vehicle in that lane.
     */
    Vehicle vehicle;
    for (int i = 0; i < predictions.size(); i++ ) {
        vehicle.vehicle_id = predictions[i][0];
        vehicle.lane = vehicle.get_lane(predictions[i][6]);
        if (vehicle.lane == lane ) {
            double vx = predictions[i][3];
            double vy = predictions[i][4];
            vehicle.speed = sqrt(vx*vx + vy*vy);
            return vehicle.speed;
        }
    }

    //Found no vehicle in the lane
    return -1.0;
}

/*
 * calculate_cost
 *
 */
float
calculate_cost (const Vehicle &vehicle,
                const vector<vector<float>> &predictions,
                const vector<Vehicle> &trajectory)
{ 
    
    /*
     * Sum weighted cost functions to get total cost for trajectory.
     */
    map<string, float> trajectory_data = 
                            get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    /*
     * Add additional cost functions here.
     */
    vector <function<float(const Vehicle &, const vector<Vehicle> &,
           const vector<vector<float>>&, map<string, float> &)>> cf_list =
                                        {goal_distance_cost, inefficiency_cost};

    vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = 
            weight_list[i]*cf_list[i](vehicle, trajectory, predictions,
                                      trajectory_data);
        cost += new_cost;
    }
    return cost;
}

/*
 * get_helper_data
 *
 */
map<string, float>
get_helper_data (const Vehicle &vehicle, const vector<Vehicle> &trajectory,
                 const vector<vector<float>> & predictions)
{
    
    /*
     * Generate helper data to use in cost functions:
     * indended_lane: +/- 1 from the current lane if the vehicle is
     *                 planning or executing a lane change.
     * final_lane: The lane of the vehicle at the end of the trajectory.
     *             The lane is unchanged for KL and PLCL/PLCR trajectories.
     * distance_to_goal: The s distance of the vehicle to the goal.
     * 
     * Note that indended_lane and final_lane are both included to help
     * differentiate between planning and executing a lane change in 
     * the cost functions.
     */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;

    if (trajectory_last.state == PLCL) {
        intended_lane = trajectory_last.lane + 1;
    } else if (trajectory_last.state == PLCR) {
        intended_lane = trajectory_last.lane - 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.tj.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;

    return trajectory_data;
}


/*
 * Utlity cost functions
 */
double
logistic (double x)
{
    /*
     * A function that returns a value between 0 and 1 for x in the 
     * range[0, infinity] and 
     * - 1 to 1 for x in the range[-infinity, infinity]. 
     *   Useful for cost functions.
     */
  return 2.0/(1 + exp(-x)) - 1.0;
}

double 
nearest_approach (vector<double> const &s_traj, vector<double> const &d_traj,
                  vector<vector<double>> const &prediction)
{
  double closest_distance = 999999;     // some big number
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) 
                                    + pow(d_traj[i] - prediction[i][2], 2));
    if (current_dist < closest_distance) {
        closest_distance = current_dist;
    }
  }

  return closest_distance;
}

double
nearest_approach_to_any_vehicle (vector<double> const &s_traj,
                                 vector<double> const &d_traj,
                         map<int, vector<vector<double>>> const &predictions)
{
  
    /*
     * Calculates the closest distance to any vehicle for the given trajectory.
     */
    double closest = 999999;
    for (auto prediction : predictions) {
        double current_dist = nearest_approach(s_traj, d_traj,
                                               prediction.second);
        if (current_dist < closest) {
            closest = current_dist;
        }
    }

  return closest;
}

double
collision_cost (vector<double> const &s_traj, vector<double> const &d_traj,
                map<int,vector<vector<double>>> const &predictions)
{

    /*
     * Binary cost function which penalizes collisions.
     */
    double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj,
                                                     predictions);
    if (nearest < 2 * VEHICLE_RADIUS) {
        return 1.0;
    } else {
        return 0;
    }
}

double
buffer_cost (vector<double> const &s_traj, vector<double> const &d_traj,
             map<int,vector<vector<double>>> const &predictions)
{
  
    /*
     * Penalizes getting close to other vehicles.
     */
    double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj,
                                                     predictions);
    return logistic(2 * VEHICLE_RADIUS / nearest);
}

double 
nearest_approach_to_any_vehicle_in_lane (vector<double> s_traj, vector<double> d_traj, 
                                          map<int, vector<vector<double>>> predictions)
{
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double my_final_d = d_traj[d_traj.size() - 1];
    int my_lane = my_final_d / 4;
    vector<vector<double>> pred_traj = prediction.second;
    double pred_final_d = pred_traj[pred_traj.size() - 1][2];
    int pred_lane = pred_final_d / 4;
    if (my_lane == pred_lane) {
      double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if (current_dist < closest && current_dist < 120) {
        closest = current_dist;
      }
    }
  }
  return closest;
}

double 
in_lane_buffer_cost (vector<double> s_traj, vector<double> d_traj, map<int,
                     vector<vector<double>>> predictions) 
{
  /*
   * Penalizes getting close to other vehicles.
   */
  double nearest = nearest_approach_to_any_vehicle_in_lane(s_traj, d_traj,
                                                             predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double
not_middle_lane_cost (vector<double> d_traj) 
{
  // penalize not shooting for middle lane (d = 6)
  double end_d = d_traj[d_traj.size()-1];
  return logistic(pow(end_d-6, 2));
}

vector<double> 
differentiate (vector<double> traj)
{
    /*
     * Given a trajectory (a vector of positions), return the average velocity
     * between each pair as a vector.
     *
     * Also can be used to find accelerations from velocities, 
     * jerks from accelerations, etc. (i.e. discrete derivatives)
     */
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / TIME_BETWEEN_POINTS);
  }
  return velocities;
}

double
efficiency_cost (vector<double> s_traj)
{
  // Rewards high average speeds.
  vector<double> s_dot_traj = differentiate(s_traj);
  double final_s_dot, total = 0;

  // cout << "DEBUG - s_dot: ";
  // for (double s_dot: s_dot_traj) {
  //   cout << s_dot << ", ";
  //   total += s_dot;
  // }
  // cout << "/DEBUG" << endl;
  // double avg_vel = total / s_dot_traj.size();

    final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  // cout << "DEBUG - final s_dot: " << final_s_dot << endl;
  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
}

double 
max_jerk_cost (vector<double> s_traj)
{
  
    /*
     * Penalize exceeding MAX_INSTANTANEOUS_JERK
     */
    vector<double> s_velocity = differentiate(s_traj);
    vector<double> s_acceleration = differentiate(s_velocity);
    vector<double> s_jerks = differentiate(s_acceleration);
    for (double s_jerk : s_jerks) {
        if (fabs(s_jerk) > MAX_JERKS) {
            //cout<<"MAX JERK EXCEED !!!!"<<endl;
            return 1;
        }
    }

    return 0;
}

double
calculate_all_cost(vector<double> const &s_traj, vector<double> const &d_traj,
                   map<int, vector<vector<double>>> const &predictions)
{
    double sum_of_costs = 0.0;

    sum_of_costs += collision_cost(s_traj, d_traj, predictions) 
                                                      * COLLISION_COST_WEIGHT;
    sum_of_costs += buffer_cost(s_traj, d_traj, predictions) 
                                                         * BUFFER_COST_WEIGHT;
    sum_of_costs += 
            in_lane_buffer_cost(s_traj, d_traj, predictions) 
                                                  * IN_LANE_BUFFER_COST_WEIGHT;

    sum_of_costs += efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
    sum_of_costs += not_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;

    sum_of_costs += max_jerk_cost(s_traj) * MAX_JERK_COST_WEIGHT;


    return sum_of_costs;
}
