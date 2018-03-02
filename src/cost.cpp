#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "utils.h"

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
nearest_approach_to_any_vehicle_in_lane (vector<double> s_traj,
                                         vector<double> d_traj, 
                              map<int, vector<vector<double>>> predictions)
{
    
    /*
     * Determines the nearest the vehicle comes to any other vehicle
     * throughout a trajectory
     */
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
  /*
   * penalize not shooting for middle lane (d = 6)
   */
  double end_d = d_traj[d_traj.size()-1];
  return logistic(pow(end_d-6, 2));
}

vector<double> 
differentiate (vector<double> traj)
{
    /*
     * Given a trajectory (a vector of positions), return the
     * average velocity between each pair as a vector.
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
    /*
     * Rewards high average speeds.
     */
    vector<double> s_dot_traj = differentiate(s_traj);
    double final_s_dot, total = 0;
    final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
    
    return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
}

/*
 * max_jerk_cost
 *
 */
double 
max_jerk_cost (vector<double> trajectory)
{
  
    /*
     * Penalize exceeding MAX_INSTANTANEOUS_JERK
     */
    vector<double> s_velocity = differentiate(trajectory);
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

    sum_of_costs += max_jerk_cost(s_traj) * MAX_S_JERK_COST_WEIGHT;
    
    sum_of_costs += max_jerk_cost(d_traj) * MAX_D_JERK_COST_WEIGHT;


    return sum_of_costs;
}
