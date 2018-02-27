#define N_SAMPLES              25      // 0.5 second
#define MAX_JERKS              10      // m/s/s/s
#define MAX_ACCEL              10      // m/s/s
#define SPEED_LIMIT            49.9    // MPH
#define TIME_BETWEEN_POINTS    0.02    // time between 2 points
#define SAFTY_DISTANCE         30      // 2+4+2 max lateral distance from a car
#define VEHICLE_RADIUS         1.25    // meter 
/*
 * Cost related weights
 */
#define COLLISION_COST_WEIGHT       9999 //99999
#define BUFFER_COST_WEIGHT          1000
#define IN_LANE_BUFFER_COST_WEIGHT  3000 //90000
#define EFFICIENCY_COST_WEIGHT      1000 //100
#define NOT_MIDDLE_LANE_COST_WEIGHT 10 //10
#define MAX_JERK_COST_WEIGHT        5555 //9999

vector<double> get_jerk_min_coeffs(vector<double>, vector <double>, double);


