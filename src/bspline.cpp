#include "bspline.hpp"

BSpline::BSpline(std::vector<std::vector<double>> control_points)
{
    // fix first point
    control_points.insert(control_points.begin(), control_points[0]);
    control_points.insert(control_points.begin(), control_points[0]);
    control_points.insert(control_points.begin(), control_points[0]);

    // fix last point
    control_points.insert(control_points.end(), control_points[control_points.size()-1]);
    control_points.insert(control_points.end(), control_points[control_points.size()-1]);
    control_points.insert(control_points.end(), control_points[control_points.size()-1]);

    this->control_points = control_points;
    num_control_points = control_points.size();
    genBezierPoints();
}

std::vector<double> BSpline::evalBSpline(double t)
{
    int bezier_group = (int)t;

    return evalCubicBezier(t - bezier_group, std::vector<std::vector<double>> { bezier_points[bezier_group * 4], 
                                                                                bezier_points[bezier_group * 4 + 1], 
                                                                                bezier_points[bezier_group * 4 + 2], 
                                                                                bezier_points[bezier_group * 4 + 3] });
}

int BSpline::getNumPeriods()
{
    return num_control_points - 4;
}

void BSpline::printBezierPoints()
{
    for (int bezier_point = 0; bezier_point < num_bezier_points; ++bezier_point)
    {
        std::cout << bezier_points[bezier_point][0] << ',' << bezier_points[bezier_point][1] << '\n';
    }
}

void BSpline::genBezierPoints()
{
    // generate middle points
    for (int control_point = 0; control_point + 3 < control_points.size(); ++control_point)
    {
        std::vector<double> control_point0 = control_points[control_point];
        std::vector<double> control_point1 = control_points[control_point + 1];
        std::vector<double> control_point2 = control_points[control_point + 2];
        std::vector<double> control_point3 = control_points[control_point + 3];

        std::vector<double> third0 = getLineThirds(control_point0, control_point1)[1];
        std::vector<double> third1 = getLineThirds(control_point1, control_point2)[0];
        std::vector<double> third2 = getLineThirds(control_point1, control_point2)[1];
        std::vector<double> third3 = getLineThirds(control_point2, control_point3)[0];

        std::vector<double> bezier_point0 = { third0[0] + ((third1[0] - third0[0])/2), third0[1] + ((third1[1] - third0[1])/2) };
        std::vector<double> bezier_point1 = third1;
        std::vector<double> bezier_point2 = third2;
        std::vector<double> bezier_point3 = { third2[0] + ((third3[0] - third2[0])/2), third2[1] + ((third3[1] - third2[1])/2) };

        bezier_points.push_back(bezier_point0);
        bezier_points.push_back(bezier_point1);
        bezier_points.push_back(bezier_point2);
        bezier_points.push_back(bezier_point3);
    }

    num_bezier_points = bezier_points.size();
}

std::vector<std::vector<double>> BSpline::getLineThirds(std::vector<double> point1, std::vector<double> point2)
{
    double third_of_line_x = (point2[0] - point1[0]) / 3;
    double third_of_line_y = (point2[1] - point1[1]) / 3;

    std::vector<double> third_point1 = { point1[0] + third_of_line_x, point1[1] + third_of_line_y };
    std::vector<double> third_point2 = { point2[0] - third_of_line_x, point2[1] - third_of_line_y };

    return std::vector<std::vector<double>> { third_point1, third_point2 };
}

std::vector<double> evalCubicBezier(double t, std::vector<std::vector<double>> points)
{
    double x = pow(1-t, 3) * points[0][0] +
                3 * pow(1-t, 2) * t * points[1][0] +
                3 * (1-t) * pow(t, 2) * points[2][0] +
                pow(t, 3) * points[3][0];

    double y = pow(1-t, 3) * points[0][1] +
                3 * pow(1-t, 2) * t * points[1][1] +
                3 * (1-t) * pow(t, 2) * points[2][1] +
                pow(t, 3) * points[3][1];

    return std::vector<double> {x, y};
}