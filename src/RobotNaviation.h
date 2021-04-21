#ifndef DUMMY_H
#define DUMMY_H
#include <armadillo>

using namespace arma;
/** This is a Robot Navigation class that generate all dynamics for simulation and tests purpose.
* Dummy class with do_process function
*/ 
class RobotNavigation
{
    private:
        mat dynamic_matrix, controle_matrix, output_matrix;
        mat path, landmarks_map;
    protected:

    public:
        mat state, aimed_state, control_vector, imu_data;
        mat ball_position, landmarks_seen, robots_position;
        float ellapsed_time, initial_time, time_step;
        mat control_gains;

        RobotNavigation();
        ~RobotNavigation();

        void set_dynamic_matrix(mat dynamic_matrix);
        void set_control_matrix(mat controle_matrix);
        void set_output_matrix(mat output_matrix);

        void set_path(mat path);
        void set_landmarks_map(mat landmarks_map);

        void set_state(mat state);
        void set_aimed_state(mat aimed_state);
        void set_control_vector(mat control_vector);
        void set_imu_data(mat imu_data);

        void set_ball_position(mat ball_position);
        void set_robots_position(mat robots_position);

        void set_ellapsed_time(float ellapsed_time);
        void set_initial_time(float initial_time);
        void set_time_step(float time_step);

        mat get_state();
        mat get_aimed_state();
        mat get_control_vector();
        mat get_ball_position();
        mat get_landmarks_seen();
        mat get_robots_position();

        mat control_p(mat aimed_state, mat control_gains);
        mat control_pi(mat aimed_state, mat control_gains);
        mat control_pid(mat aimed_state, mat control_gains);

        mat path_planner(mat state, mat aimed_state, mat ball_position, mat robots_position);
        mat velocity_odometry(mat state, mat control_vector, float dt);
        mat accelartion_odometry(mat state,mat imu_data, mat contrl_vector, float dt);

};


#endif