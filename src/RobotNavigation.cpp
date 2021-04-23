#include "RobotNavigation.h"
#include <iostream>

using namespace std;
using namespace arma;


RobotNavigation::RobotNavigation()
{
    cout << "Creating a Robot Navigation Object" << endl;
}

RobotNavigation::RobotNavigation(mat A, mat B, mat C, mat X0, float t0, float dt)
{
    dynamic_matrix = A;
    control_matrix = B;
    output_matrix = C;
    state = X0;
    initial_time = t0;
    time_step = dt;

    cout << "dynamic matrix = " << A << endl;
    cout << "control matrix = " << B << endl;
    cout << "outup matrix = " << C << endl;
    cout << "state = " << X0 << endl;
    cout << "initial time = " << t0 << endl;
    cout << "time step = " << dt << endl;

}

RobotNavigation::~RobotNavigation()
{
    cout << "Destruction your Robot Navigation Object" << endl;
}

/** Set Dynamic Matrix
 */ 
void RobotNavigation::set_dynamic_matrix(mat A)
{
    dynamic_matrix = A;
}
/** Set Control Matrix
 */ 
void RobotNavigation::set_control_matrix(mat B)
{
    control_matrix = B;
}
/** Set Output Matrix
 */ 
void RobotNavigation::set_output_matrix(mat C)
{
    output_matrix = C;
}
/** Set Path
 */ 
void RobotNavigation::set_path(mat p)
{
    path = p;
}
/** Set Landmarks Map
 */ 
void RobotNavigation::set_landmarks_map(mat lm_map)
{
    landmarks_map = lm_map;
}
/** Set Robot State
 */ 
void RobotNavigation::set_state(mat xr)
{
    state = xr;
}
/** Set Robot Aimed State
 */ 
void RobotNavigation::set_aimed_state(mat xg)
{
    aimed_state = xg;
}
/** Set Control Vector
 */ 
void RobotNavigation::set_control_vector(mat u)
{
    control_vector = u;
}
/** Set Ball Position
 */ 
void RobotNavigation::set_ball_position(mat b_pos)
{
    ball_position = b_pos;
}
/** Set Robots Position
 */ 
void RobotNavigation::set_robots_position(mat r_pos)
{
    robots_position = r_pos;
}
/** Set Initial Time
 */ 
void RobotNavigation::set_initial_time(float t0)
{
    initial_time = t0;
}
/** Set Time Step
 */ 
void RobotNavigation::set_time_step(float dt)
{
    time_step = dt;
}
/** Load IMU Data from text file
 */ 
void RobotNavigation::load_imu_data(string imu_data_file)
{

}
/** Load Landmarks Map from text file
 * @param imu_data_file a text file with data collected from robots IMU
 */ 
void RobotNavigation::load_landmarks_map(string landmarks_map_file)
{

}
/** Get Robot State
 * @param landmarks_map_file a text file with all field landmarks
 */ 
mat RobotNavigation::get_state()
{
    return state;
}
/** Get Robot Aimed State
 */ 
mat RobotNavigation::get_aimed_state()
{
    return aimed_state;
}
/** Get Control Vector
 */ 
mat RobotNavigation::get_control_vector()
{
    return control_vector;
}
/** Get Ball Position
 */ 
mat RobotNavigation::get_ball_position()
{
    return ball_position;
}
/** Get Landmarks Seen
 */ 
mat RobotNavigation::get_landmarks_seen()
{
    return landmarks_seen;
}
/** Get Robots Position
 */ 
mat RobotNavigation::get_robots_position()
{
    return robots_position;
}
/** Get Ellapsed Time
 */ 
float RobotNavigation::get_ellapsed_time()
{
    return ellapsed_time;
}
/** Performs a Proporcional Control
 * @param aimed_state derised state of the robot
 * @param control_gains_p proportional gains (krho_p, kalpha_p, kbeta_p)
 * @return control vector (v,w)
 */ 
mat RobotNavigation::control_p(mat aimed_state, mat control_gains_p)
{

}
/** Performs a Proporcional-Integral Control
 * @param aimed_state derised state of the robot
 * @param control_gains_pi proportional-integral gains (krho_p, kalpha_p, kalpha_i)
 * @return control vector (v,w)
 */ 
mat RobotNavigation::control_pi(mat aimed_state, mat control_gains_pi)
{

}
/** Performs a Proporcional-Integral-Derivative Control
 * @param aimed_state desired state of the robot
 * @param control_gains_pid proportional-integral-derivative gains (krho_p, kalpha_p, kalpha_i, kalpha_d)
 * @return control vector (v,w)
 */ 
mat RobotNavigation::control_pid(mat aimed_state, mat control_gains)
{

}
/** Plan a path based on ball and Robots position in the field
 * @param state robot actual state
 * @param aimed_state desired state of the robot
 * @param ball_position position of the ball on the field
 * @param robots_position position of all robots to be evaded
 * @return a path (set of points (x,y) )
 */ 
mat RobotNavigation::path_planner(mat state, mat aimed_state, mat ball_position, mat robots_position)
{

}
/** Performs a odometry from velocity equations
 * @param state robot actual state
 * @param control_vector control vector (v,w)
 * @param dt time step
 * @return new state of the robot
 */ 
mat RobotNavigation::velocity_odometry(mat state, mat control_vector, float dt)
{

}
/** Performs a odometry from acceleration equations
 * @param state robot actual state
 * @param imu_data a set of collected data from Robots IMU
 * @param control_vector control vector (v,w)
 * @param dt time step
 * @return new state of the robot
 */ 
mat RobotNavigation::acceleration_odometry(mat state,mat imu_data, mat control_vector, float dt)
{

}
/** Performs a landmarks detection given robot position
 * @param state robot actual state
 * @param landmarks_map a map of landmarks of the field
 * @return landmarks seen from that stae
 */ 
mat RobotNavigation::landmark_detection(mat state, mat landmarks_map)
{
    
}
