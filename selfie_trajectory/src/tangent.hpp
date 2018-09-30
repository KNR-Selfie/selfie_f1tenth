#pragma once 
#include <cmath>
#include "spline.hpp"

class tangent{
  public:
    float a;
    float b;
    float angle_deg;
    float angle_rad;
    int tangent_point;
    spline_t tangent_spline;

    tangent();
    void angle();
    void calculate(spline_t spl, int x);
};
