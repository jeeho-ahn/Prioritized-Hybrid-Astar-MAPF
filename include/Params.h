#ifndef PARAMS_H
#define PARAMS_H

#include <cmath>

struct Params {
    double xy_resolution = 0.1;
    double yaw_resolution = M_PI / 6.0; // 30 degrees
    double time_step = 2.0;
    double time_resolution = time_step;
    double min_x = -3.0;
    double min_y = -3.0;
    double max_x = 8.0;
    double max_y = 8.0;
    double max_steer = M_PI / 6.0; // 30 degrees
    double turn_penalty = 1.25;
    double reverse_penalty = 50.0;
    double switch_penalty = 1.2;
    double wait_penalty = 0.05;
    double max_time = 100.0;
    int collision_steps = 3;
    double analytic_threshold = 5.0;
    double rs_step_size = 0.2;
    double inflation = 1.1;
    double safety_margin = 0.05;
    double movement_length = 0.0;  // Added back for dynamic calculation
};

#endif // PARAMS_H
