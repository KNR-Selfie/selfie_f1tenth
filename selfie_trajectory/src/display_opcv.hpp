#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define WIDTH 1000
#define HEIGHT 1000

cv::Mat frame(HEIGHT, WIDTH, CV_8UC3);

bool draw_data(std::vector<cv::Point> &input, cv::Scalar color)
{
    cv::circle(frame, cv::Point(WIDTH/2, HEIGHT/2), 20, cv::Scalar(0, 255, 255), 5, 8, 0);

    for(uint32_t i = 0; i < input.size(); i++)
    {
        cv::circle(frame, cv::Point(input[i].x, input[i].y), 1, color, 2, 8, 0);
        //std::cout << i << ": [" << input[i].x << " ; " << input[i].y << "] " << std::endl;
    }
    cv::imshow("Data", frame);
    char keypressed = cv::waitKey(10);
    if(keypressed == 27)
        return 0;
    else
        return 1;
}
