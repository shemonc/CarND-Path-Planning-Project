#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

/*
 * Cost related weights
 */
#define COLLISION_COST_WEIGHT       50000 //99999 //9999
#define BUFFER_COST_WEIGHT          1000
#define IN_LANE_BUFFER_COST_WEIGHT  5000 //3000
#define EFFICIENCY_COST_WEIGHT      1000 //1000
#define NOT_MIDDLE_LANE_COST_WEIGHT 10 //10
#define MAX_S_JERK_COST_WEIGHT      110 //150 //100 //6500 //6000
#define MAX_D_JERK_COST_WEIGHT      200 //250 //150 //80

double calculate_all_cost(vector<double> const &, vector<double> const &,
                          map<int, vector<vector<double>>> const &);

#endif
