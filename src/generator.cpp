#include "generator.hpp"

int main ()
{
    std::srand(50);

    Puzzle puzzle;
    puzzle.addPuzzleImg("../tests/big_test.png");
    puzzle.findPuzzleDims(100);
    puzzle.drawPuzzleLines("classic");
    puzzle.cutPuzzle();
    //puzzle.showImg();
    puzzle.saveLines();

    return 0;
}

void Puzzle::addPuzzleImg(std::string puzzle_image)
{
    img = cv::imread(puzzle_image, cv::IMREAD_GRAYSCALE);

    img_res_x = img.cols;
    img_res_y = img.rows;
}

void Puzzle::findPuzzleDims(int num_pieces)
{
    // only calculate one way, then flip if needed
    bool flipped = false;
    int height = img_res_y;
    int width = img_res_x;

    if (width < height)
    {
        int temp = height;
        height = width;
        width = temp;
        flipped = true;
    }

    // find all pairs of divisors
    int lastVal = std::sqrt(num_pieces);
    int divisor = 2;

    std::vector<int> bestPairRatio;
    float desiredRatio = (float)width/(float)height;
    float bestRatio = 20000;

    while (divisor <= lastVal)
    {
        if (num_pieces%divisor == 0)
        {
            // save if ratio is closest to desired
            float ratio = (float)(num_pieces/divisor)/(float)divisor;
            if (abs(ratio - desiredRatio) < bestRatio)
            {
                bestRatio = abs(ratio - desiredRatio);
                bestPairRatio = std::vector<int> {(num_pieces/divisor), divisor};
            }
        }
        divisor++;
    }

    pieceSideLength = (float)img_res_x/(float)(bestPairRatio[0]);

    // unflip
    if (!flipped)
    {
        puzzle_width = bestPairRatio[0];
        puzzle_height = bestPairRatio[1];
    }
    else
    {
        puzzle_width = bestPairRatio[1];
        puzzle_height = bestPairRatio[0];
    }
}

void Puzzle::drawPuzzleLines(std::string mode)
{
    lines = cv::Mat(img_res_x, img_res_y, CV_8UC1, cv::Scalar(255));

    // draw edges
    drawPuzzleEdges(lines);

    if (mode == "straight")
    {
        drawPuzzleLinesStraight(lines);
    }
    else if (mode == "wonky")
    {
        drawPuzzleLinesWonkyStraight(lines);
    }
    else if (mode == "classic")
    {
        drawPuzzleLinesClassic(lines);
    }
}

void Puzzle::drawPuzzleLinesClassic(cv::Mat &lines)
{
    // vertical lines
    for (int x = 1; x < puzzle_width; ++x)
    {
        std::vector<std::vector<double>> bspline_control_points;

        for (int y = 0; y < puzzle_height; ++y)
        {
            std::vector<std::vector<double>> points = genPuzzleShapePoints((double)(x * pieceSideLength), (double)(y * pieceSideLength), true);
            bspline_control_points.insert(bspline_control_points.end(), std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()));
        }

        BSpline my_spline(bspline_control_points);
        drawBSpline(lines, my_spline);
    }

    // horizontal lines
    for (int y = 1; y < puzzle_height; ++y)
    {
        std::vector<std::vector<double>> bspline_control_points;

        for (int x = 0; x < puzzle_width; ++x)
        {
            std::vector<std::vector<double>> points = genPuzzleShapePoints((double)(x * pieceSideLength), (double)(y * pieceSideLength), false);
            bspline_control_points.insert(bspline_control_points.end(), std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()));
        }

        BSpline my_spline(bspline_control_points);
        drawBSpline(lines, my_spline);
    }
}

void Puzzle::drawPuzzleLinesStraight(cv::Mat &lines)
{
    // vertical lines
    for (int x = 1; x < puzzle_width; ++x)
    {
        cv::Point2d point1(x*pieceSideLength, 0);
        cv::Point2d point2(x*pieceSideLength, puzzle_height*pieceSideLength);
        cv::line(lines, point1, point2, (0,0,0));
    }

    // horizontal lines
    for (int y = 1; y < puzzle_height; ++y)
    {
        cv::Point2d point1(0, y*pieceSideLength);
        cv::Point2d point2(puzzle_width*pieceSideLength, y*pieceSideLength);
        cv::line(lines, point1, point2, (0,0,0));
    }
}

void Puzzle::drawPuzzleLinesWonkyStraight(cv::Mat &lines)
{
    int max_deviation = 30;

    // vertical lines
    for (int x = 1; x < puzzle_width; ++x)
    {
        int centre_line = x*pieceSideLength;
        int last_point = centre_line + randomOffset(max_deviation);
        int next_point = centre_line + randomOffset(max_deviation);
        for (int y = 0; y < puzzle_height; ++y)
        {
            cv::Point2d point1(last_point, y*pieceSideLength);
            cv::Point2d point2(next_point, (y+1)*pieceSideLength);
            cv::line(lines, point1, point2, (0,0,0));

            last_point = next_point;
            next_point = centre_line + randomOffset(max_deviation);
        }
    }

    // horizontal lines
    for (int y = 1; y < puzzle_height; ++y)
    {
        int centre_line = y*pieceSideLength;
        int last_point = centre_line + randomOffset(max_deviation);
        int next_point = centre_line + randomOffset(max_deviation);
        for (int x = 0; x < puzzle_width; ++x)
        {
            cv::Point2d point1(x*pieceSideLength, last_point);
            cv::Point2d point2((x+1)*pieceSideLength, next_point);
            cv::line(lines, point1, point2, (0,0,0));

            last_point = next_point;
            next_point = centre_line + randomOffset(max_deviation);
        }
    }
}

void Puzzle::drawPuzzleEdges(cv::Mat &lines)
{
    // top
    cv::Point2d point1(0, 0);
    cv::Point2d point2(puzzle_width*pieceSideLength, 0);
    cv::line(lines, point1, point2, (0,0,0));

    // bottom
    point1 = cv::Point2d(0, puzzle_height*pieceSideLength);
    point2 = cv::Point2d(puzzle_width*pieceSideLength, puzzle_height*pieceSideLength);
    cv::line(lines, point1, point2, (0,0,0));

    // left
    point1 = cv::Point2d(0, 0);
    point2 = cv::Point2d(0, puzzle_height*pieceSideLength);
    cv::line(lines, point1, point2, (0,0,0));

    // right
    point1 = cv::Point2d(puzzle_width*pieceSideLength, 0);
    point2 = cv::Point2d(puzzle_width*pieceSideLength, puzzle_height*pieceSideLength);
    cv::line(lines, point1, point2, (0,0,0));
}

int Puzzle::randomOffset(int max_deviation)
{
    return (rand() % max_deviation) - max_deviation/2;
}

double Puzzle::randomOffset(double max_deviation)
{
    return (((double)rand() / RAND_MAX) * 2 * max_deviation) - max_deviation;
}

void Puzzle::cutPuzzle()
{
    int piece_num = 0;
    // iterate a pixel in every piece
    for (int x = 0; x < puzzle_width; ++x)
    {
        for (int y = 0; y < puzzle_height; ++y)
        {
            int current_x = x*pieceSideLength + pieceSideLength/2;
            int current_y = y*pieceSideLength + pieceSideLength/2;
            int min_padding = 50;
            double scale_factor = 0;

            // flood fill, threshold, non zero, bound and crop to get single piece
            cv::Mat current_piece, threshold, points;
            cv::floodFill(lines, current_piece, cv::Point(current_x, current_y), cv::Scalar(), 0, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_MASK_ONLY | (255 << 8));
            cv::threshold(current_piece, threshold, 254, 255, cv::THRESH_BINARY);
            cv::findNonZero(threshold, points);
            cv::Rect bounds = cv::boundingRect(threshold);
            current_piece = cv::Mat(current_piece, bounds);

            // write out
            std::string piece_name = "../output_pieces/piece" + std::to_string(piece_num) + ".png";
            cv::imwrite(piece_name, current_piece);
            piece_num++;
        }
    }
}

void Puzzle::showImg()
{
    cv::imshow("Puzzle", img);
    cv::imshow("Puzzle lines", lines);
    cv::waitKey(0);
}

void Puzzle::saveLines()
{
    cv::imwrite("puzzleOutput.png", lines);
}

void drawBSpline(cv::Mat &lines, BSpline bspline)
{
    // get curve points
    std::vector<std::vector<double>> result_points;

    for (double t = 0; t < bspline.getNumPeriods(); t+=0.05)
    {
        std::vector<double> new_point = bspline.evalBSpline(t);
        result_points.push_back(new_point);
    }

    // convert vector double points to pixel points
    cv::Mat pixel_points;

    for (auto &point : result_points)
    {
        pixel_points.push_back(cv::Point (point[0], point[1]));
    }

    // draw
    cv::polylines(lines, pixel_points, false, cv::Scalar(0));
}

std::vector<std::vector<double>> Puzzle::genPuzzleShapePoints(double x, double y, bool vertical)
{
    if (vertical)
    {
        double temp = x;
        x = y;
        y = temp;
    }

    bool flipped = false;
    if (std::rand() % 2 == 1)
    {
        flipped = true;
    }

    double tab_centre = x + pieceSideLength/2 + randomOffset(pieceSideLength/20);
    double tab_width1 = pieceSideLength/20; // affects width where tab attaches to piece
    double tab_width2 = pieceSideLength/5; // affects width of tab
    double tab_height = 2*pieceSideLength/5 + randomOffset(pieceSideLength/20); // affects tab height
    double tab_height2 = pieceSideLength/4; // affects shape of tab

    if (flipped)
    {
        tab_height = -tab_height;
        tab_height2 = - tab_height2;
    }

    std::vector<double> point1 = { x, y };
    std::vector<double> point2 = { tab_centre + tab_width1 + randomOffset(pieceSideLength/25), y + randomOffset(pieceSideLength/25) };
    std::vector<double> point3 = { tab_centre - tab_width2 + randomOffset(pieceSideLength/25), y + tab_height2 + randomOffset(pieceSideLength/25) };
    std::vector<double> point4 = { tab_centre + randomOffset(pieceSideLength/25), y + tab_height + randomOffset(pieceSideLength/25) };
    std::vector<double> point5 = { tab_centre + tab_width2 + randomOffset(pieceSideLength/25), y + tab_height2 + randomOffset(pieceSideLength/25) };
    std::vector<double> point6 = { tab_centre - tab_width1 + randomOffset(pieceSideLength/25), y + randomOffset(pieceSideLength/25) };
    std::vector<double> point7 = { x + pieceSideLength, y };

    std::vector<std::vector<double>> points = { point1, point2, point3, point4, point5, point6, point7 };

    if (vertical)
    {
        for (auto &point : points)
        {
            double temp = point[0];
            point[0] = point[1];
            point[1] = temp;
        }
    }

    return points;
}