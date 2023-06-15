#include "generator.hpp"

// the output x and y resolution of each jigsaw piece image, -2
#define TARGET_RES 1078

void generatePuzzle(std::string ratio_goal_image, int num_pieces_goal)
{
    std::srand(std::time(0));

    Puzzle puzzle;
    std::cout << "Loading image...\n";
    puzzle.addPuzzleImg(ratio_goal_image);
    std::cout << "Done!\n";

    std::cout << "Finding puzzle dimensions...\n";
    puzzle.findPuzzleDims(num_pieces_goal);
    std::cout << "Done!\n";

    std::cout << "Drawing puzzle lines...\n";
    puzzle.drawPuzzleLines("classic");
    std::cout << "Done!\n";

    std::cout << "Cutting puzzle...\n";
    puzzle.cutPuzzleHiRes();
    std::cout << "Done!\n";

    puzzle.saveLines();
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

        bsplines_vertical.push_back(BSpline(bspline_control_points));
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

        bsplines_horizontal.push_back(BSpline(bspline_control_points));
    }

    for (auto &bspline : bsplines_vertical)
    {
        drawBSpline(lines, bspline);
    }

    for (auto &bspline : bsplines_horizontal)
    {
        drawBSpline(lines, bspline);
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
    std::vector<double> top_left = { 0, 0 };
    std::vector<double> top_right = { puzzle_width*pieceSideLength, 0 };
    std::vector<double> bottom_left = { 0, puzzle_height*pieceSideLength };
    std::vector<double> bottom_right = { puzzle_width*pieceSideLength, puzzle_height*pieceSideLength };

    BSpline bspline_top(std::vector<std::vector<double>>{ top_left, top_right });
    BSpline bspline_bottom(std::vector<std::vector<double>>{ bottom_left, bottom_right });
    BSpline bspline_left(std::vector<std::vector<double>>{ top_left, bottom_left });
    BSpline bspline_right(std::vector<std::vector<double>>{ top_right, bottom_right });

    bsplines_edges = { bspline_top, bspline_bottom, bspline_left, bspline_right };

    // draw
    drawBSpline(lines, bspline_top);
    drawBSpline(lines, bspline_bottom);
    drawBSpline(lines, bspline_left);
    drawBSpline(lines, bspline_right);
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

void Puzzle::cutPuzzleHiRes()
{
    int piece_num = 0;
    int maximum_piece_side_length = pieceSideLength*2;

    double scale = TARGET_RES/maximum_piece_side_length;

    double time_per_side = 7; // how many bspline time sections are in each puzzle piece side

    // iterate each central puzzle piece
    for (int x = -1; x < puzzle_width - 1; ++x)
    {
        BSpline left_spline;
        BSpline right_spline;

        // left edge special case
        if (x == -1)
        {
            right_spline = bsplines_vertical[0];
        }
        // right edge special case
        else if (x == puzzle_width - 2)
        {
            left_spline = bsplines_vertical.back();
        }
        else
        {
            left_spline = bsplines_vertical[x];
            right_spline = bsplines_vertical[x+1];
        }

        double vertical_time_period_start = 1;

        for (int y = -1; y < puzzle_height - 1; ++y)
        {
            BSpline top_spline;
            BSpline bottom_spline;
            // top edge special case
            if (y == -1)
            {
                bottom_spline = bsplines_horizontal[0];
            }
            // bottom edge special case
            else if (y == puzzle_height-2) {
                top_spline = bsplines_horizontal.back();
            }
            else
            {
                top_spline = bsplines_horizontal[y];
                bottom_spline = bsplines_horizontal[y+1];
            }

            double horizontal_time_period_start = ((x+1)*time_per_side)+1;

            cv::Mat new_piece(maximum_piece_side_length*scale, maximum_piece_side_length*scale, CV_8U, cv::Scalar(0));

            double top_left_x = (x+1)*pieceSideLength - pieceSideLength/2;
            double top_left_y = (y+1)*pieceSideLength - pieceSideLength/2;

            if (x == -1)
            {
                cv::Point point1(pieceSideLength/2 * scale, 10);
                cv::Point point2(pieceSideLength/2 * scale, maximum_piece_side_length*scale - 10);
                cv::line(new_piece, point1, point2, cv::Scalar(255));
            }
            else
            {
                drawHiResBSpline(new_piece, 
                            top_left_x, 
                            top_left_y, 
                            left_spline, 
                            vertical_time_period_start, 
                            vertical_time_period_start+time_per_side+1, 
                            scale, 0.05);
            }

            if (x == puzzle_width-2)
            {
                cv::Point point1(3*pieceSideLength/2 * scale, 10);
                cv::Point point2(3*pieceSideLength/2 * scale, maximum_piece_side_length*scale - 10);
                cv::line(new_piece, point1, point2, cv::Scalar(255));
            }
            else
            {
                drawHiResBSpline(new_piece, 
                                top_left_x, 
                                top_left_y, 
                                right_spline, 
                                vertical_time_period_start, 
                                vertical_time_period_start+time_per_side+1,
                                scale, 0.05);
            }

            if (y == -1)
            {
                cv::Point point1(10, pieceSideLength/2 * scale);
                cv::Point point2(maximum_piece_side_length*scale - 10, pieceSideLength/2 * scale);
                cv::line(new_piece, point1, point2, cv::Scalar(255));
            }
            else
            {
                drawHiResBSpline(new_piece, 
                                top_left_x, 
                                top_left_y, 
                                top_spline, 
                                horizontal_time_period_start, 
                                horizontal_time_period_start+time_per_side+1, 
                                scale, 0.05);
            }

            if (y == puzzle_height-2)
            {
                cv::Point point1(10, 3*pieceSideLength/2 * scale);
                cv::Point point2(maximum_piece_side_length*scale - 10, 3*pieceSideLength/2 * scale);
                cv::line(new_piece, point1, point2, cv::Scalar(255));
            }
            else
            {
                drawHiResBSpline(new_piece, 
                                top_left_x, 
                                top_left_y, 
                                bottom_spline, 
                                horizontal_time_period_start, 
                                horizontal_time_period_start+time_per_side+1, 
                                scale, 0.05);
            }

            horizontal_time_period_start += time_per_side;
            vertical_time_period_start += time_per_side;

            fillImageLines(new_piece);

            // write out
            std::string piece_name = "../output_pieces/piece" + std::to_string(piece_num) + ".png";
            cv::imwrite(piece_name, new_piece);
            piece_num++;
        }
        vertical_time_period_start += time_per_side;
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

void drawHiResBSpline(cv::Mat &img, int img_offset_x, int img_offset_y, BSpline bspline, double t0, double t1, double scale, double resolution)
{
    // get curve points
    std::vector<std::vector<double>> result_points;

    for (double t = t0; t < t1; t+=resolution)
    {
        std::vector<double> new_point = bspline.evalBSpline(t);

        // increase resolution
        new_point[0] *= scale;
        new_point[1] *= scale;

        result_points.push_back(new_point);
    }

    // convert vector double points to pixel points
    cv::Mat pixel_points;

    for (auto &point : result_points)
    {
        pixel_points.push_back(cv::Point (point[0] - (img_offset_x*scale), point[1] - (img_offset_y*scale)));
        //pixel_points.push_back(cv::Point (point[0], point[1]));
    }

    // draw
    cv::polylines(img, pixel_points, false, cv::Scalar(255));
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

void fillImageLines(cv::Mat &img)
{
    // flood fill, threshold, non zero, bound and crop to get single piece
    cv::Mat current_piece, threshold, points;
    cv::floodFill(img, current_piece, cv::Point(img.size().width/2, img.size().height/2), cv::Scalar(), 0, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_MASK_ONLY | (255 << 8));
    cv::threshold(current_piece, threshold, 254, 255, cv::THRESH_BINARY);
    cv::findNonZero(threshold, points);

    // crop image
    //cv::Rect bounds = cv::boundingRect(threshold);
    //current_piece = cv::Mat(current_piece, bounds);

    img = threshold;
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