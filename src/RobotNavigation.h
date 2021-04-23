#ifndef ROBOTNAVIGATION_H
#define ROBOTNAVIGATION_H
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
        /** control gain p - vector containing the gains for proportional control for pho, alpha and beta (krho_p,kalpha_p,kbeta_p)
         */ 
        mat control_gains_p;
        /** control gain pi - vector containing the gains for proportional-integral control for pho and alpha (krho_p,kalpha_p,kalpha_i)
         */ 
        mat control_gains_pi;
        /** control gain pid - vector containing the gains for proportional-integral-derivative control for pho and alpha (krho_p,kalpha_p,kalpha_i,kalpha_d)
         */ 
        mat control_gains_pid;
        /** Constructor for RobotNavigation
         */ 
        RobotNavigation();
        /** Constructor for RobotNavigation that set up the main matrices of the class
         */ 
        RobotNavigation(mat dynamic_matrix, mat control_matrix, mat output_matrix, mat initial_state, float initial_time, float time_step);
        /** Destructor for RobotNavigation
         */ 
        ~RobotNavigation();
        /** Set Dynamic Matrix
         */ 
        void set_dynamic_matrix(mat dynamic_matrix);
        /** Set Control Matrix
         */ 
        void set_control_matrix(mat controle_matrix);
        /** Set Output Matrix
         */ 
        void set_output_matrix(mat output_matrix);
        /** Set Path
         */ 
        void set_path(mat path);
        /** Set Landmarks Map
         */ 
        void set_landmarks_map(mat landmarks_map);
        /** Set Robot State
         */ 
        void set_state(mat state);
        /** Set Robot Aimed State
         */ 
        void set_aimed_state(mat aimed_state);
        /** Set Control Vector
         */ 
        void set_control_vector(mat control_vector);
        /** Set Ball Position
         */ 
        void set_ball_position(mat ball_position);
        /** Set Robots Position
         */ 
        void set_robots_position(mat robots_position);
        /** Set Initial Time
         */ 
        void set_initial_time(float initial_time);
        /** Set Time Step
         */ 
        void set_time_step(float time_step);
        /** Load IMU Data from text file
         */ 
        void load_imu_data(string imu_data_file);
        /** Load Landmarks Map from text file
         * @param imu_data_file a text file with data collected from robots IMU
         */ 
        void load_landmarks_map(string landmarks_map_file);
        /** Get Robot State
         * @param landmarks_map_file a text file with all field landmarks
         */ 
        mat get_state();
        /** Get Robot Aimed State
         */ 
        mat get_aimed_state();
        /** Get Control Vector
         */ 
        mat get_control_vector();
        /** Get Ball Position
         */ 
        mat get_ball_position();
        /** Get Landmarks Seen
         */ 
        mat get_landmarks_seen();
        /** Get Robots Position
         */ 
        mat get_robots_position();
        /** Get Ellapsed Time
         */ 
        float get_ellapsed_time();
        /** Performs a Proporcional Control
         * @param aimed_state derised state of the robot
         * @param control_gains_p proportional gains (krho_p, kalpha_p, kbeta_p)
         * @return control vector (v,w)
         */ 
        mat control_p(mat aimed_state, mat control_gains_p);
        /** Performs a Proporcional-Integral Control
         * @param aimed_state derised state of the robot
         * @param control_gains_pi proportional-integral gains (krho_p, kalpha_p, kalpha_i)
         * @return control vector (v,w)
         */ 
        mat control_pi(mat aimed_state, mat control_gains_pi);
        /** Performs a Proporcional-Integral-Derivative Control
         * @param aimed_state desired state of the robot
         * @param control_gains_pid proportional-integral-derivative gains (krho_p, kalpha_p, kalpha_i, kalpha_d)
         * @return control vector (v,w)
         */ 
        mat control_pid(mat aimed_state, mat control_gains);
        /** Plan a path based on ball and Robots position in the field
         * @param state robot actual state
         * @param aimed_state desired state of the robot
         * @param ball_position position of the ball on the field
         * @param robots_position position of all robots to be evaded
         * @return a path (set of points (x,y) )
         */ 
        mat path_planner(mat state, mat aimed_state, mat ball_position, mat robots_position);
        /** Performs a odometry from velocity equations
         * @param state robot actual state
         * @param control_vector control vector (v,w)
         * @param dt time step
         * @return new state of the robot
         */ 
        mat velocity_odometry(mat state, mat control_vector, float dt);
        /** Performs a odometry from acceleration equations
         * @param state robot actual state
         * @param imu_data a set of collected data from Robots IMU
         * @param control_vector control vector (v,w)
         * @param dt time step
         * @return new state of the robot
         */ 
        mat acceleration_odometry(mat state,mat imu_data, mat control_vector, float dt);
        /** Performs a landmarks detection given robot position
         * @param state robot actual state
         * @param landmarks_map a map of landmarks of the field
         * @return landmarks seen from that stae
         */ 
        mat landmark_detection(mat state, mat landmarks_map);

};


#endif