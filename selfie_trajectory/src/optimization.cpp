
#include "optimization.h"

using namespace std;

void two_wall_planner(spline_t left_spline,spline_t right_spline, spline_t &path_line)
{

    vector<Point>pom;
    pom.push_back(Point(0,0));

    pom.push_back(Point((left_spline.Y[left_spline.Y.size()/2] + right_spline.Y[right_spline.Y.size()/2])/2, (left_spline.X[left_spline.X.size()/2]+right_spline.X[right_spline.X.size()/2])/2));

    pom.push_back(Point(left_spline.Y.back() + right_spline.Y.back()/2,(left_spline.X.back()+right_spline.X.back())/2)); //midle of space between top detected points

    path_line.set_spline(pom,true);

}
void one_wall_planner(spline_t spl,int offset, spline_t &path_line)
{
    vector<Point>pom;
    double max = spl.X.back();

    pom.push_back(Point(0,0));
    pom.push_back(Point(spl.spline(max + (max)/2)+offset,max + (max)/2));
    pom.push_back(Point(spl.Y[0]+offset/2,spl.X[0]+abs(offset/2)));
    path_line.set_spline(pom,true);

}

