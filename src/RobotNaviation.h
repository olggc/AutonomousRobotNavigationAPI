#ifndef DUMMY_H
#define DUMMY_H
#include <armadillo>
#include <iostream>
#include <string>	

using namespace arma;
using namespace std;

/** This is a Robot Navigation class for simulation and tests purpose.
* The main purpose of this class is to generate all dynamics and sensoring for simulating in-game enviroment 
* and test the efienciecy of kalman and particle filter for robot localization.
*/ 

class RobotNavigation
{
    public:
        /** dynamic matrix - the transition matrix containg system main dynamics
         */
        mat dynamic_matrix;
        /** control matrix - the control matrix containg influence of control dynamics on the system
         */ 
        mat control_matrix;
        /** output matrix - the output matrix containg the dynamics of sensors measure
         */ 
        mat output_matrix;
        /**  landmarks map - a map containing all the land marks of the footbal field (X,L and T) intersections (x,y,type)
         */ 
        mat landmarks_map;
        /** state - the robot actual state (x,y,th)
         */ 
        mat state;
        /** aimed state - the robot desired state (x,y,th)
         */ 
        mat aimed_state;
        /** control vector - vector containing the controllable variables of the system (v,w)
         */ 
        mat control_vector;
        /** path - matrix containg N points of a path [(x0,y0),(x1,y1),...(xn,yn)]
         */ 
        mat path;
        /** ball position - a vector which contain the actual ball position (x,y)
         */ 
        mat ball_position;
        /** landmarks seen - a matrix containing the landmarks seen at the actual moment (x,y,type)
         */ 
        mat landmarks_seen;
        /** robots position - all robots position in the field
         */ 
        mat robots_position;
        /** ellapsed time - ellapsed time since simulation start
         */ 
        float ellapsed_time;
        /** initial time - time which simulation started
         */ 
        float initial_time;
        /** time step - discrete time step for simulation
         */ 
        float time_step;
        /** control gain p - vector containing the gains for proportional control for pho, alpha and beta (kp,ka,kb)
         */ 
        mat control_gains_p;
        /** control gain pi - vector containing the gains for proportional-integral control for pho and alpha (kp,ka,ka_i)
         */ 
        mat control_gains_pi;
        /** control gain pid - vector containing the gains for proportional-integral-derivative control for pho and alpha (kp,ka,ka_i,ka_d)
         */ 
        mat control_gains_pid;
        /** Constructor for RobotNavigation
         */ 
        RobotNavigation();
        /** Destructor for RobotNavigation
         */ 
        ~RobotNavigation();


        void set_dynamic_matrix(mat dynamic_matrix);
        void set_control_matrix(mat controle_matrix);
        void set_output_matrix(mat output_matrix);

        void set_path(mat path);
        void set_landmarks_map(mat landmarks_map);

        void set_state(mat state);
        void set_aimed_state(mat aimed_state);
        void set_control_vector(mat control_vector);

        void set_ball_position(mat ball_position);
        void set_robots_position(mat robots_position);

        void set_initial_time(float initial_time);
        void set_time_step(float time_step);

        void load_imu_data(string imu_data_file);
        void load_landmarks_map(string landmarks_map_file);

        mat get_state();
        mat get_aimed_state();
        mat get_control_vector();
        mat get_ball_position();
        mat get_landmarks_seen();
        mat get_robots_position();
        mat get_ellapsed_time();

        mat control_p(mat aimed_state, mat control_gains);
        mat control_pi(mat aimed_state, mat control_gains);
        mat control_pid(mat aimed_state, mat control_gains);

        mat path_planner(mat state, mat aimed_state, mat ball_position, mat robots_position);
        mat velocity_odometry(mat state, mat control_vector, float dt);
        mat acceleration_odometry(mat state,mat imu_data, mat control_vector, float dt);
        mat landmark_detection(mat state, mat landmarks_map);

};


#endif