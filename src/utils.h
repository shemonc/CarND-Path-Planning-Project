#define N_SAMPLES              25      // 0.5 second
#define MAX_JERKS              10      // m/s/s/s
#define MAX_ACCEL              10      // m/s/s
#define SPEED_LIMIT            49.9    // MPH
#define TIME_BETWEEN_POINTS    0.02    // time between 2 points


/*
 * 50 mph = 80000/3600 = 22.2 m/s
 * Simulator runs every 20ms (50 frames for second)
 * so 22m or 30m is an optimal choice for trajectory generation
 */
#define TRAJECTORY_DISTANCE      30      // meter spline will generate the trajectory 
#define SAFETY_DISTANCE          0.3*TRAJECTORY_DISTANCE // vehicle too close
#define VEHICLE_RADIUS           1.25    // meter 
#define LANE_WIDTH               4
#define DESIRED_LANE_POSITION    2
#define TOTAL_AVAILABLE_LANE     3
#define LEFT_MOST_LANE           0
#define STARTING_RAMPUP_DISTANCE 200    //meter, to gain the desired speed

vector<double> get_jerk_min_coeffs(vector<double>, vector <double>, double);


