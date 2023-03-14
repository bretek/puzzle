#include "opencv_utils.hpp"

cv::Point findNearestPointOnContour(std::vector<cv::Point> contour, cv::Point point)
{
    double lowest_point_distance = getPointDistance(point, contour[0]);
    cv::Point nearest_point;
    for (auto contour_point = contour.begin(); contour_point != contour.end(); ++contour_point)
    {
        if (getPointDistance(point, *contour_point) < lowest_point_distance)
        {
            nearest_point = *contour_point;
            lowest_point_distance = getPointDistance(point, *contour_point);
        }
    }
    
    return nearest_point;
}

std::vector<cv::Mat> splitContourAtPoints(std::vector<cv::Point> contour, std::vector<cv::Point> points, cv::Size mat_size)
{
    int edge_index = 0;
    std::vector<cv::Mat> splitContours(0);

    splitContours.push_back(cv::Mat(mat_size, CV_8U, cv::Scalar(0)));
    for (auto point = contour.begin(); point != contour.end(); ++point)
    {
        splitContours[edge_index%(points.size())].at<uchar>(*point) = 255;
        if (std::find(points.begin(), points.end(), cv::Point(point->x, point->y)) != points.end())
        {
            edge_index++;
            if (edge_index < points.size())
            {
                splitContours.push_back(cv::Mat(1000, 1000, CV_8U, cv::Scalar(0)));
            }
        }
    }

    return splitContours;
}

double getPointDistance(cv::Point point1, cv::Point point2)
{
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}