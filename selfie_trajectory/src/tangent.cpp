#include "tangent.hpp"


tangent::tangent()
{


}

void tangent::calculate(spline_t spl, int x)
{   tangent_spline=spl;
    tangent_point=x;
    a = spl.spline.deriv(1,x);
    b =-a*x+spl.spline(x);

}

void tangent::angle()
{

    //angle_deg = atan2( (a*LIDAR_MAT_HEIGHT+b) - a*tangent_point+b,LIDAR_MAT_HEIGHT - tangent_point);
    angle_rad = atan(a);
    angle_deg = angle_rad * 180 / M_PI;

}

