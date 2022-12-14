#include <iostream>
#include <math.h>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "bspline.hpp"

class Puzzle
{
    public:
    void addPuzzleImg(std::string puzzle_image);
    void findPuzzleDims(int num_pieces);
    void drawPuzzleLines(std::string mode);
    void cutPuzzle();
    void cutPuzzleHiRes();
    void showImg();
    void saveLines();

    private:
    int randomOffset(int max_deviation);
    double randomOffset(double max_deviation);

    void drawPuzzleEdges(cv::Mat &lines);
    void drawPuzzleLinesStraight(cv::Mat &lines);
    void drawPuzzleLinesWonkyStraight(cv::Mat &lines);
    void drawPuzzleLinesClassic(cv::Mat &lines);

    std::vector<std::vector<double>> genPuzzleShapePoints(double x, double y, bool vertical);

    cv::Mat img;
    cv::Mat lines;
    int img_res_x;
    int img_res_y; 
    float pieceSideLength;

    int puzzle_width;
    int puzzle_height;

    std::vector<BSpline> bsplines_vertical;
    std::vector<BSpline> bsplines_horizontal;

    std::vector<BSpline> bsplines_edges;
};

int generatePuzzle();
void drawHiResBSpline(cv::Mat &img, int img_offset_x, int img_offset_y, BSpline bspline, double t0, double t1, double scale, double resolution);
void drawBSpline(cv::Mat &lines, BSpline bspline);
void fillImageLines(cv::Mat &img);