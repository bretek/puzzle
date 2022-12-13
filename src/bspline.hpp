#include <iostream>
#include <math.h>
#include <vector>

class BSpline
{
    public:
    BSpline() {};
    BSpline(std::vector<std::vector<double>> control_points);
    std::vector<double> evalBSpline(double t);
    int getNumPeriods();
    void printBezierPoints();

    private:
    void genBezierPoints();
    std::vector<std::vector<double>> getLineThirds(std::vector<double> point1, std::vector<double> point2);

    int num_control_points;
    std::vector<std::vector<double>> control_points;
    int num_bezier_points;
    std::vector<std::vector<double>> bezier_points;
};

std::vector<double> evalCubicBezier(double t, std::vector<std::vector<double>> points);