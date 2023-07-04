#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>
#include <set>
#include <tuple>
#include <map>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "opencv_utils.hpp"

#define OUT_EDGE 0
#define IN_EDGE 1
#define FLAT_EDGE 2

#define TOP_EDGE 0
#define LEFT_EDGE 1
#define BOTTOM_EDGE 2
#define RIGHT_EDGE 3

#define CORNER_PIECE 2
#define EDGE_PIECE 1
#define NORMAL_PIECE 0

struct PuzzlePiece;
struct PuzzleEdge;

struct PuzzlePiece
{
    std::string id;
    cv::Mat piece_mat;
    std::vector<std::shared_ptr<PuzzleEdge>> edges;
    int piece_type;
};

struct PuzzleEdge
{
    int edge_type; // 0 - out, 1 - in, 2 - edge
    int edge_side;
    cv::Mat edge_mat;
    std::shared_ptr<PuzzlePiece> piece;
};

typedef std::tuple<int, int> Coordinate;
typedef std::tuple<std::shared_ptr<PuzzlePiece>, Coordinate> PlacedPiece;

template <typename T>
class TreeNode
{
    public:
    TreeNode(std::shared_ptr<TreeNode<T>> parent, T value);
    std::shared_ptr<TreeNode<T>> parent;
    T value;
    std::vector<std::shared_ptr<TreeNode<T>>> children;
};

template <typename T>
class Tree
{
    public:
    std::shared_ptr<TreeNode<T>> root_node;

    Tree(std::shared_ptr<TreeNode<T>> root_node);
    void insertNode(std::shared_ptr<TreeNode<T>> parent, std::shared_ptr<TreeNode<T>> new_node);
    void removeNode(std::shared_ptr<TreeNode<T>> node);
    private:
    std::vector<std::shared_ptr<TreeNode<T>>> nodes;
};

int createPossiblePieceTreeChildren(TreeNode<PlacedPiece>* node, 
                                    std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matching_edges, 
                                    int direction);

int solvePuzzle();

std::vector<std::shared_ptr<PuzzlePiece>> findEdgesFromImages(const std::string directory_path);
std::shared_ptr<PuzzlePiece> findPieceEdges(std::filesystem::path image_path);
int detectEdgeType(cv::Mat edge, cv::Rect puzzle_piece_corners_rect);
int detect_piece_type(std::shared_ptr<PuzzlePiece> piece);
bool isSpaceOccupied(Tree<PlacedPiece>* tree, int x, int y);

std::vector<PlacedPiece> placePuzzlePieces(std::vector<std::shared_ptr<PuzzlePiece>> pieces);
std::shared_ptr<Tree<PlacedPiece>> solvePuzzleEdge(std::vector<std::shared_ptr<PuzzlePiece>> pieces);

void rotatePiece(std::shared_ptr<PuzzlePiece> piece);

bool sortbyth(const std::tuple<std::shared_ptr<PuzzleEdge>, float>& a, 
              const std::tuple<std::shared_ptr<PuzzleEdge>, float>& b);
std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> findMatchingEdges(std::shared_ptr<PuzzleEdge> current_edge, std::vector<std::shared_ptr<PuzzlePiece>> all_pieces);
float getLineImageDifference(cv::Mat mat1, cv::Mat mat2);
float compareEdges(PuzzleEdge edge1, PuzzleEdge edge2);

void showSolvedPuzzle(std::vector<PlacedPiece> placed_pieces);