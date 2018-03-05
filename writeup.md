#**CarND-Path-Planning-Project** 


---

## The Goals

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car's localization and sensor fusion data will be provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Steps
#### Note, following steps describe the workflow here to drive the car(ego vehicle) along the highway. These same steps are also embedded as comments into the code's relevant position , mostly in main.cpp file

### 1.
    Create N_SAMPLES=(25 for this demonstration) predictions of end stats (final s, final velocity ) for the non-ego vehicles for a duration of 25*0.02 = 0.5 seconds This is done in main.cpp,
    function get_nonego_cars_predictions  
      
#### Later this predicted data will be used to generate possible jerk minimized trajectories for this ego vehicle and a number of cost functions will be engaged to find the best optimal path for a given future duration(0.5 second in this excercise). From this optimal path the calculated reference velocity and intended lane will feed into spline to derive a smooth 50 points of the desired trajectory polynomial for the ego vehicle to travel

### 2.
Detect the closest left, right, front and the vehicle on the back of ego vehicle, as done on main.cpp around
line 429

### 3

 Get the possible future states such as KEEP_LANE, LANE_CHANGE_LEFT(LCL), LANE_CHANGE_RIGHT(LCR) etc based on the surrounding vehicle detections and ego's current lane, around line 441 in main.cpp

### 4, 5 & 6
Based on this possible states (KL, LCL, LCR) and intended lanes and based on predicted end states of non-ego vehicle (done in step 1 above) calculated the predicted (for 0.5 second) end stats of
ego vehicle (s_f, s_f_d, s_f_dd, d_f, d_f_d, d_f_dd)

This is done in line 446 to 470

### 7
Now that we know the  
*i) start states  
*ii) end states  and  
*iii) time  

Calculate a jerk minimize trajectory.All the time derivatives of S are further 6 or more have to be zero inorder for S to be jerk minimal.  

*Longitudinal polynomial  

s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5  

*Lateral polynomial  
d(t) = a_0 + a_1 * t + a_2 * t**2 +  a_3 * t**3 + a_4 * t**4 + a_5 * t**5  
    - 5th order polynomial  
    - 6 Coefficients i.e. 6 tunable parameters  

### 8

Among all the possible states (KL, LCL, LCR, derived in step 3) and all the predicted  possible trajectories of ego vehicle find the minimal cost trajectory, done around line 510, in main.cpp
Various cost functions such as collision_cost, max_jerk_cost in s and d direction, efficiency_cost etc are engaged to derive a minimal cost trajectory. These cost functions are defined in cost.cpp
file.

### 9, 10

From the minimal cost trajectory (predicted for 0.5 second ), get the velocity, corresponding lane
and end s of ego.  
if predicted velocity is zero within 0.5 second , we really need to do an emergency stop.  
Else feed this velocity and lane into spline to plot the trajectory smoothly. 

