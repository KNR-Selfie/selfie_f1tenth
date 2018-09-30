#pragma once

#include <stdio.h>
#include <cstdio>
#include <vector>
#include <iostream>
#include "spline.h"


class Point
{
    public:
    int x;
    int y;
    Point(int param_x, int param_y);
    Point();
};



using namespace tk;

class spline_t
{
public:
    tk::spline spline;
    std :: vector<double> X;
    std :: vector<double> Y;

    spline_t();
    void set_spline(std::vector<Point> vec, bool qubic); //wektor musi być posortowany po x
};
