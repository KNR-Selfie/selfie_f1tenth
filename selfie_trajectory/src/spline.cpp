#include "spline.hpp"

Point::Point(int param_x, int param_y)
{
    x = param_x;
    y = param_y;
}

Point::Point()
{
    
}

spline_t::spline_t()
{


}

void spline_t::set_spline(std::vector<Point> vec, bool qubic)
{
    X.clear();
    Y.clear();

    for(int i=0;i<vec.size();i++)
    {   
	X.push_back(vec[i].y);
        Y.push_back(vec[i].x);
    }
    

    spline.set_points(X,Y,qubic);
}


