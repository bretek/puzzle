#include <vector>

#include <opencv2/highgui.hpp>

cv::Point findNearestPointOnContour(std::vector<cv::Point> contour, cv::Point point);
std::vector<cv::Mat> splitContourAtPoints(std::vector<cv::Point> contour, std::vector<cv::Point> points, cv::Size mat_size);
double getPointDistance(cv::Point point1, cv::Point point2);