#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

typedef enum car_state {
    CS=0,               // Constant speed
    KL,                 // Keep Lane
    PLCL,               // Prepare for lane change on left
    PLCR,               // Prepare for lane change on right
    LCL,                // Lane change on left
    LCR                 // Lane change on right
} car_state_e;

typedef struct trajectory_st {
    float s;
    float s_dot;
    float s_dot_dot;
    float d;
    float d_dot;
    float d_dot_dot;
} trajectory_t;

typedef struct car_detected_st {
    bool left;
    bool right;
    bool ahead;
    bool behind;
} car_detected_t;

class Vehicle {
public:

    map<car_state_e, int> lane_direction = 
                            {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};

    int             vehicle_id;
    int             lane;
    trajectory_t    tj;
    float           a = 0.224;
    float           target_speed;
    int             lanes_available;
    float           max_acceleration;
    int             first_pass;
    float           goal_s;
    car_state_e     state;
    car_detected_t  nego;
    float           x;
    float           y;
    float           yaw;
    vector<double>  s_traj_coeffs;
    vector<double>  d_traj_coeffs;
    long            emergency_stop;

    /*
     * Constructor
     */
    Vehicle();
    
    /*
     * Destructor
     */
    virtual ~Vehicle();

    vector<car_state_e> successor_states();
    vector<Vehicle> generate_trajectory(car_state_e state, 
                                        vector<vector<float>> predictions);

    void increment(int dt);
    float position_at (long, float);
    void detect_closest_vehicle (const vector<Vehicle> &);
    vector<Vehicle> generate_predictions(int horizon=2);
    void realize_next_state(vector<Vehicle> trajectory);
    void configure(float, int, int, float, car_state_e, float, int);
    void set_trajectory_param(float, float, float, float, float, float);
    vector<vector<double>> get_predictions (int, int);
    vector<vector<double>> get_predicted_end_states (car_state_e, int,
                                    map<int, vector<vector<double>>> &);
    vector<float> get_inlane_vehicle_end_states (const int,
                                        map<int, vector<vector<double>>> &);
    int get_target_lane(car_state_e);
    vector<vector<double>> generate_trajectory_based_on_prediction(
                                const vector<vector<double>> &, const double);


};

#endif
