#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle &, const vector<vector<float>> &,
                     const vector<Vehicle> &);

float goal_distance_cost(const Vehicle &,  const vector<Vehicle> &,
                         const vector<vector<float>> &,
                         map<string, float> & data);

float inefficiency_cost(const Vehicle &, const vector<Vehicle> &,
                        const vector<vector<float>> &,
                        map<string, float> & data);

float lane_speed(const vector<vector<float>> &, int);

map<string, float> get_helper_data(const Vehicle &,
                                   const vector<Vehicle> &,
                                   const vector<vector<float>> &);

double calculate_all_cost(vector<double> const &, vector<double> const &,
                          map<int, vector<vector<double>>> const &);

#endif
