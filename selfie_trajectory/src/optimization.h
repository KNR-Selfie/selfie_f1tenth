#include "spline.hpp"

//lidar version
void two_wall_planner(spline_t left_spline,spline_t right_spline, spline_t &path_line);
void one_wall_planner(spline_t spl,int offset, spline_t &path_line);
