#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "utils.h"
#include "cost.h"
using namespace std;

// for convenience
using json = nlohmann::json;

Vehicle ego_car = Vehicle();

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
#if 1
    /*
     * mph
     */
    volatile double ref_vel = 0.0;
    /*
     * start in lane 1
     */
    int lane = 1;
#endif

/*
 * generate_predictions
 *
 *
 */
void 
get_nonego_cars_predictions (vector<vector<double>> sf, vector<Vehicle> &car,
                      map<int, vector<vector<double>>> &pred, int prev_points)
{

    Vehicle veh;
    vector<vector<double>> nonego_predict;

    for (int i = 0; i < sf.size(); i++) {
        double vx = sf[i][3];
        double vy = sf[i][4];
        double speed = sqrt(vx*vx + vy*vy);
        veh = Vehicle();
        veh.set_trajectory_param(sf[i][5], speed, 0, sf[i][6], 0, 0);
        veh.vehicle_id = sf[i][0];
        car.push_back(veh);
        nonego_predict = veh.get_predictions(prev_points, N_SAMPLES);
        pred[veh.vehicle_id] = nonego_predict;
    }
}


int main() {
  uWS::Hub h;

    /*
     * init the ego vehicle
     */
    float       target_speed = 49.5;
    int         init_lane = 1;
    int         goal_lane = 1;
    float       max_acceleration = 2 /* MAX_ACCELERATION */ ;
    car_state_e initial_state = CS; 
    float       distance_to_goal = 6945.554;
    int         available_lane = 3;

    /*
     * configure the ego vehicle with initial data
     */
     ego_car.configure(target_speed, init_lane, goal_lane, max_acceleration,
                       initial_state, distance_to_goal, available_lane);

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = distance_to_goal;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }


  // Have a reference velocity to target
  //double ref_vel = 49.5; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
            double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
                          
          	json msgJson;
            

           
            int prev_size = previous_path_x.size();
           
            if (prev_size > 0) {
                car_s = end_path_s;
                car_d = end_path_d;
            }

            /*
             * build s, s_d and s_dd 
             *       d, d_d and d_dd
             * for the ego car      
             */
             float ego_s, ego_s_dot, ego_s_dot_dot;
             float ego_d, ego_d_dot, ego_d_dot_dot;

            /*
             * update ego car's localization parameters 
             */
            //ego_car.update_localization(car_x, car_y, car_s, car_d, car_yaw,
            //                            car_speed);

            


#if 0
            bool too_close = false;
            /*
             * find ref_v to use
             */
            for (int i = 0; i < sensor_fusion.size(); i++) {
                
                /*
                 * car is in my lane
                 */
                float d = sensor_fusion[i][6];
                if (d < (2 + 4*lane + 2) && d > (2 + 4*lane -2)) {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    /*
                     * if using previous ponts can projects s value out
                     */
                    check_car_s += ((double)prev_size * 0.02 * check_speed);

                    /*
                     * check s values greater than mine and s gap
                     */
                    if ((check_car_s > car_s) && (check_car_s - car_s) < 30 ) 
                    {
                        
                        /*
                         * Do some logic here, lower reference velocity so we
                         * do not crash into the car
                         * infront of us, could also flag to try to chnage lanes
                         */
                        //ref_vel = 29.5; //mph
                        too_close = true;
                        /*
                        if (lane > 0) {
                            lane = 0;
                        }
                        */
                    }
                }
            }

            if (too_close) {
                ref_vel -= 0.224; //  5 m/s^2           
            } else if (ref_vel < 49.5) {
                ref_vel += 0.224;
            }
#endif
            /*
             * state measuring state machine
             */
            //vector<Vehicle> car  = ego_car.choose_next_state(sensor_fusion);

            /*
             * end of state machine
             */

            /*
             * Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
             * Later we will interpolate these waypoints with a spline and fill it in with
             * more points that control the speed.
             */
            vector<double> ptsx;
            vector<double> ptsy;

            /*
             * reference x,y, yaw states
             * Either we will reference the starting point as where the car is
             * or at the previous paths end point.
             */
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            /*
             * if previous size is almost empty, use the car as starting reference
             */
            //cout << "prev_size " << prev_size <<endl;
            if (prev_size < 3 ) {

                /*
                 * Use two points that make the path tangent to the car
                 */
                double prev_car_x = car_x - cos(ref_yaw); //car_yaw  should be ref_yaw
                double prev_car_y = car_y - sin(ref_yaw);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);

                /*
                 * set  s, s_dot and s_dot_dot
                 * same for d
                 */
                ego_s = car_s;
                ego_s_dot = car_speed;
                ego_s_dot_dot = 0.0;
                ego_d = car_d;
                ego_d_dot = 0.0;
                ego_d_dot_dot = 0.0;

            } else {
                
                /*
                 * use the previous path's end point as starting reference
                 */
                
                /*
                 * Redefine reference state as previous path end point
                 */
                ref_x = previous_path_x[prev_size -1];
                ref_y = previous_path_y[prev_size -1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
               
                double ref_x_prev_prev = previous_path_x[prev_size - 3];
                double ref_y_prev_prev = previous_path_y[prev_size - 3];
                double ref_yaw_prev = atan2(ref_y_prev - ref_y_prev_prev, 
                                            ref_x_prev - ref_x_prev_prev);
                /*
                 * Use two points that make the path tangent to the previous
                 * path's end point
                 */
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);

                /*
                 * set s, s_dot, s_dot_dot and  d, d_dot, d_dot_dot
                 */
                vector<double> frenet_prev = getFrenet(ref_x_prev,
                        ref_y_prev, ref_yaw, map_waypoints_x, map_waypoints_y);
                vector<double> frenet_prev_prev = getFrenet(ref_x_prev_prev,
                                ref_y_prev_prev, ref_yaw_prev, map_waypoints_x,
                                map_waypoints_y);

                /*
                 * Predict non-ego vehicle for next 1 second, compare to
                 * ego vehicle
                 * 
                 */
                ego_s = car_s;
                ego_s_dot = (car_s - frenet_prev[0])/TIME_BETWEEN_POINTS;
                float ego_s_dot_prev = (frenet_prev[0] 
                                     - frenet_prev_prev[0])/TIME_BETWEEN_POINTS;
                ego_s_dot_dot = (ego_s_dot - ego_s_dot_prev)/TIME_BETWEEN_POINTS;

                ego_d = car_d;
                ego_d_dot = (car_d - frenet_prev[1])/TIME_BETWEEN_POINTS;
                float ego_d_dot_prev = (frenet_prev[1] 
                                     - frenet_prev_prev[1])/TIME_BETWEEN_POINTS;
                ego_d_dot_dot = (ego_d_dot - ego_d_dot_prev)/TIME_BETWEEN_POINTS; 
            }
          
            //cout<<"ego s:"<<ego_s<<" s_dot:"<<ego_s_dot<<" s_dot_dot:";
            //cout<<ego_s_dot_dot<<endl;

            //cout<<"ego d:"<<ego_d<<" d_dot:"<<ego_d_dot<<" d_dot_dot:";
            //cout<<ego_d_dot_dot<<endl;
#if 0
            ego_car.set_trajectory_param(ego_s, ego_s_dot, ego_s_dot_dot,
                                         ego_d, ego_d_dot, ego_d_dot_dot);
#else
            ego_car.tj.s = car_s;
            //ego_car.tj.s_dot_dot = ego_s_dot_dot;
            //ego_car.tj.d_dot_dot = ego_d_dot_dot;
#endif
             
            /*
             * set the longitudinal velocity
             */
            ego_car.tj.s_dot = ref_vel;

            /*
             * set desired lane
             */
            ego_car.tj.d = lane*4 + 2;

            /*
             * Generate predictions of other cars using sensor fusion data
             */
            vector<Vehicle> non_egos;
            map<int, vector<vector<double>>> non_egos_prediction;

            /*
             * Create a prediction of non-ego vehicles for
             * 25*0.02 = 0.5 seconds
             */
            get_nonego_cars_predictions(sensor_fusion, non_egos,
                                            non_egos_prediction, prev_size);
            
            /*
             * Deleted the closes left, right, front & back vehicle
             * and set the flags
             */
            ego_car.detect_closest_vehicle(non_egos);
            
            int                     intended_lane;
            int                     best_lane = lane;
            double                  best_ref_velocity;
            double                  min_cost = 999999;
            car_state_e             best_state;
            vector<vector<double>>  best_traj, best_end_state;
            
            /*
             * Get the possible future states based on the
             * surrounding vehicle detections
             */
            vector<car_state_e> states = ego_car.successor_states();
            
            /*
             * Get end states (s_f, s_f_d, s_f_dd, d_f, d_f_d, d_f_dd)
             */
            for (int i = 0; i < states.size(); i++) {
                intended_lane = ego_car.get_target_lane(states[i]);
 
                vector<vector<double>> ego_predicted_final_states = 
                                    ego_car.get_predicted_end_states(states[i],
                                                          intended_lane,
                                                          non_egos_prediction);
                
                vector<double> ego_longitudinal_traj;
                vector<double> ego_lateral_traj;
                vector<vector<double>> trajectories = 
                              ego_car.generate_trajectory_based_on_prediction(
                              ego_predicted_final_states,
                              N_SAMPLES*TIME_BETWEEN_POINTS);
            
                /*
                 * predicted trajectory of eg for the duration of 
                 * N_SAMPLE*0.02 = 0.5 second
                 */
                ego_longitudinal_traj = trajectories[0];
                ego_lateral_traj = trajectories[1];
             
                double this_trajectory_cost = 
                    calculate_all_cost(ego_longitudinal_traj, ego_lateral_traj,
                                       non_egos_prediction);
                if (this_trajectory_cost < min_cost) {
                    min_cost = this_trajectory_cost;
                    best_traj = trajectories;
                    best_end_state = ego_predicted_final_states;
                    best_lane = best_end_state[1][0]/4; //intended_lane;
                    best_ref_velocity = best_end_state[0][1];

                    ref_vel = best_ref_velocity;
                    //ego_car.tj.s_dot = ref_vel;
                    cout<<"best velocity "<< best_ref_velocity <<" ref velocity "<<ref_vel<<endl;
                    
                    /*
                    if (ref_vel < 49) {
                        ref_vel += 0.224; //best_ref_velocity;
                    } else {

                        //cout<< "==>> velocity change from: "<< ref_vel <<" to: ";
                        //cout<< best_ref_velocity <<endl;
                        ref_vel -= 0.224;
                        cout<<"ref velocity "<<ref_vel<<endl;
                    }
                    */
                    //car_s = best_end_state[0][0];
                    ego_car.state = states[i];
                }
            } // for each state i.e. KL. LCL etc.
            if (best_lane >= 0 && best_lane < 3 && best_lane != lane ) {
                cout<< "==>> A Lane change from: "<< lane << " to: "<<best_lane;
                cout<<" is suggested"<<endl;
                lane = best_lane;
                //ego_car.tj.d = lane; 
                //car_s = best_end_state[0][0];
            }


            /*
             * In Frenet add evenly 30m spaced points ahead of the starting
             * reference
             */
            vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), 
                            map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane),
                            map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), 
                            map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);
            
          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

            for (int i = 0; i < ptsx.size(); i++) {
                
                /*
                 * Shift car reference angle to 0 degree
                 */
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                /*
                 * rotate
                 */
                ptsx[i] = shift_x * cos(0 - ref_yaw) 
                                                - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) 
                                                + shift_y * cos(0 - ref_yaw);
            }

            /*
             * create a spline
             */
            tk::spline s;

            /*
             * set (x,y ) points to the spline
             */
            s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
          	vector<double> next_y_vals;
 
            /*
             * Start with all of the previous path points from the last time
             */
            for (int i = 0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            /*
             * Calculate how to break up spline points so that we can travel
             * at our desired reference velocity
             */
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            double x_add_on = 0;

            /*
             * N x 0.02 x v = d
             *
             * Fill up the rest of our path planner after filling it with
             * previous points, here we will always output 50 points
             *
             * 50 mph = 80000/3600 = 22.2 m/s
             *
             * =>1 m/s = 50/22.2 = 2.25 MPH
             */
            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
                
                double N = (target_dist/(0.02*ref_vel/2.24)); // m/s*m/s * h/m ??
                double x_point = x_add_on + target_x/N;
                double y_point = s(x_point);
                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                /*
                 * rotate back to normal after rotating it earlier
                 */
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
                
                /*
                 * shift
                 */
                x_point += ref_x;
                y_point += ref_y;
                
                //cout << "x_point "<< x_point << " y_point " << y_point <<endl;
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);

            }


            /*
             * define a path made up of (x,y) points that the car will
             * visit sequentially every .02 seconds
             */
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
