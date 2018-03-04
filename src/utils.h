#define N_SAMPLES              25      // 0.5 second
#define MAX_JERKS              10      // m/s/s/s
#define MAX_ACCEL              10      // m/s/s
#define SPEED_LIMIT            49.9    // MPH
#define TIME_BETWEEN_POINTS    0.02    // time between 2 points
#define TRAJECTORY_DISTANCE    30      // meter spline will generate the trajectory 
#define SAFETY_DISTANCE        0.3*TRAJECTORY_DISTANCE // vehicle too close
#define VEHICLE_RADIUS         1.25    // meter 
#define LANE_WIDTH             4
#define DESIRED_LANE_POSITION  2


vector<double> get_jerk_min_coeffs(vector<double>, vector <double>, double);


