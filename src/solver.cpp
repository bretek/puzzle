#include "solver.hpp"

#define PIECE_SIMILARITY_THRESHOLD 0.85

int solvePuzzle()
{
    std::vector<std::shared_ptr<PuzzlePiece>> pieces = findEdgesFromImages("../output_pieces");
    std::vector<PlacedPiece> placed_pieces = placePuzzlePieces(pieces);
    showSolvedPuzzle(placed_pieces);

    return 0;
}

std::vector<std::shared_ptr<PuzzlePiece>> findEdgesFromImages(const std::string directory_path)
{
    std::filesystem::path dir = directory_path;
    std::vector<std::shared_ptr<PuzzlePiece>> pieces;

    std::set<std::filesystem::path> sorted_paths;
    for (auto &file : std::filesystem::directory_iterator{dir})
    {
        pieces.push_back(findPieceEdges(file.path()));
    }

    return pieces;
}

int detect_piece_type(std::shared_ptr<PuzzlePiece> piece)
{
    int flat_sides = 0;
    for (auto edge : piece->edges)
    {
        if (edge->edge_type == FLAT_EDGE)
        {
            flat_sides++;
        }
    }
    
    return flat_sides;
}

std::shared_ptr<PuzzlePiece> findPieceEdges(std::filesystem::path image_path)
{
    // load image
    cv::Mat piece_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    cv::resize(piece_image, piece_image, cv::Size(600, 600));
    cv::threshold(piece_image, piece_image, 127, 255, 0);

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(piece_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // find corners
    cv::Mat corners(piece_image.size().width, piece_image.size().height, CV_8U, cv::Scalar(0));
    cv::cornerHarris(piece_image, corners, 7, 5, 0.003);
    cv::threshold(corners, corners, 2, 255, cv::THRESH_BINARY);

    // find corner coordinates
    cv::Mat labels, stats, centroids;
    corners.convertTo(corners, CV_8U);
    cv::connectedComponentsWithStats(corners, labels, stats, centroids);

    // find nearest point from each corner to contour
    std::vector<cv::Point> corner_points(0);
    for (int corner_num = 1; corner_num < centroids.size().height; ++corner_num)
    {
        cv::Point corner_coord(centroids.row(corner_num));
        corner_points.push_back(findNearestPointOnContour(contours[0], corner_coord));
    }

    // split contour into puzzle edges
    std::vector<cv::Mat> puzzle_edges = splitContourAtPoints(contours[0], corner_points, piece_image.size());

    // create PuzzlePiece instance
    std::shared_ptr<PuzzlePiece> piece(new PuzzlePiece);
    piece->id = image_path;
    piece->piece_mat = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    cv::Rect puzzle_piece_corners_rect = cv::boundingRect(corners);

    if (puzzle_edges.size() != 4)
    {
        std::cout << "ABORT: Puzzle piece not read correctly: " << image_path << std::endl;
        std::exit(-1);
    }

    for (int edge_num = 0; edge_num < puzzle_edges.size(); ++edge_num)
    {
        std::shared_ptr<PuzzleEdge> edge(new PuzzleEdge);
        edge->edge_type = detectEdgeType(puzzle_edges[edge_num], puzzle_piece_corners_rect);
        edge->edge_side = edge_num;
        cv::Rect bound_rect = cv::boundingRect(puzzle_edges[edge_num]);

        // crop
        edge->edge_mat = cv::Mat(puzzle_edges[edge_num], bound_rect);
        edge->piece = piece;
        piece->edges.push_back(edge);
    }

    // set piece type
    piece->piece_type = detect_piece_type(piece);

    return piece;
}

int detectEdgeType(cv::Mat edge, cv::Rect puzzle_piece_corners_rect)
{
    cv::Rect bound_rect = cv::boundingRect(edge);
    int padding = 10; // dead zone for in out determination

    // work out edge type
    if (bound_rect.width < 40 || bound_rect.height < 40)
    {
        return FLAT_EDGE;
    }

    if (bound_rect.x < puzzle_piece_corners_rect.x - padding || 
        bound_rect.x+bound_rect.width > puzzle_piece_corners_rect.x+puzzle_piece_corners_rect.width + padding ||
        bound_rect.y < puzzle_piece_corners_rect.y - padding ||
        bound_rect.y+bound_rect.height > puzzle_piece_corners_rect.y+puzzle_piece_corners_rect.height + padding)
    {
        return OUT_EDGE;
    }
    else
    {
        return IN_EDGE;
    }
}

bool isSpaceOccupied(Tree<PlacedPiece>* tree, int x, int y)
{
    TreeNode<PlacedPiece>* current_node = tree->root_node;
    while (current_node->children.size() != 0)
    {
        Coordinate piece_coordinate = std::get<1>(current_node->value);
        if (std::get<0>(piece_coordinate) == x && std::get<1>(piece_coordinate) == y)
        {
            return true;
        }
        current_node = current_node->children[0];
    }
    
    return false;
}

std::vector<PlacedPiece> placePuzzlePieces(std::vector<std::shared_ptr<PuzzlePiece>> pieces)
{
    std::vector<PlacedPiece> placed_pieces = solvePuzzleEdge(pieces);

    for (auto piece : placed_pieces)
    {
        std::shared_ptr<PuzzlePiece> placed_piece = std::get<0>(piece);
        pieces.erase(std::remove(pieces.begin(), pieces.end(), placed_piece), pieces.end());
    }

    int max_x = 0;
    int max_y = 0;

    for (auto placed_piece : placed_pieces)
    {
        if (std::get<0>(std::get<1>(placed_piece)) > max_x)
        {
            max_x = std::get<0>(std::get<1>(placed_piece));
        }
        if (std::get<1>(std::get<1>(placed_piece)) > max_y)
        {
            max_y = std::get<1>(std::get<1>(placed_piece));
        }
    }

    // start from top left
    TreeNode<PlacedPiece> root_node(nullptr, placed_pieces[1]);
    TreeNode<PlacedPiece>* current_node = &root_node;
    Tree<PlacedPiece> placed_pieces_tree(current_node);

    int direction = BOTTOM_EDGE;
    
    //while not all pieces in
    while (pieces.size() > 0)
    {
        std::shared_ptr<PuzzlePiece> current_piece = std::get<0>(current_node->value);
        // if no children
        if (current_node->children.size() == 0)
        {
            // create children
            // find possible children
            std::vector<std::shared_ptr<PuzzlePiece>> children(0);
            std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matching_edges = findMatchingEdges(current_piece->edges[direction], pieces);
            for (auto match_data : matching_edges)
            {
                std::shared_ptr<PuzzleEdge> edge = std::get<0>(match_data);
                
                // run checks on piece

                children.push_back(edge->piece);
            }
            // if children exist add them
            if (children.size() != 0)
            {
                int x = std::get<0>(std::get<1>(current_node->value));
                int y = std::get<1>(std::get<1>(current_node->value));
                switch (direction)
                {
                    case (TOP_EDGE):
                        y--;
                        break;
                    case (BOTTOM_EDGE):
                        y++;
                        break;
                    case (LEFT_EDGE):
                        x--;
                        break;
                    case (RIGHT_EDGE):
                        x++;
                        break;
                }
                Coordinate new_coord(x, y);

                for (auto child : children)
                {
                    TreeNode<PlacedPiece>* new_node = new TreeNode<PlacedPiece>(current_node, PlacedPiece(child, new_coord));
                    placed_pieces_tree.insertNode(current_node, new_node);
                }
            }
            // otherwise pop node
            else
            {
                while(current_node->children.size() == 0 && current_node->parent != nullptr)
                {
                    std::cout << "Popping node: " << std::get<0>(current_node->value)->id << std::endl;

                    pieces.push_back(current_piece);
                    TreeNode<PlacedPiece>* temp_node = current_node->parent;
                    placed_pieces_tree.removeNode(current_node);
                    current_node = temp_node;
                }

                if (current_node->parent == nullptr)
                {
                    std::cout << "ABORT: No solution found" << std::endl;
                    current_node->children.clear();
                    pieces.clear();
                    std::exit(-1);
                }
            }
        }

        // if there are children move to left most
        else
        {
            // move to leftmost child
            current_node = current_node->children[0];
            current_piece = std::get<0>(current_node->value);
            std::cout << "Pushing node: " << current_piece->id << std::endl;

            // rotate again
            std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matching_edge = findMatchingEdges(std::get<0>(current_node->parent->value)->edges[direction], std::vector<std::shared_ptr<PuzzlePiece>>{{ current_piece }});
            std::shared_ptr<PuzzleEdge> matched_edge = std::get<0>(matching_edge[0]);
            std::shared_ptr<PuzzlePiece> matched_piece = matched_edge->piece;
            while (matched_edge->edge_side != (direction + 2) % 4)
            {
                rotatePiece(matched_piece);
            }

            // place
            pieces.erase(std::remove(pieces.begin(), pieces.end(), current_piece), pieces.end());

            int x = std::get<0>(std::get<1>(current_node->value));
            int y = std::get<1>(std::get<1>(current_node->value));

            if (!isSpaceOccupied(&placed_pieces_tree, x, y+1) && y+1 < max_y)
            {
                direction = BOTTOM_EDGE;
            }
            else if (!isSpaceOccupied(&placed_pieces_tree, x, y-1)  && y-1 > 0)
            {
                direction = TOP_EDGE;
            }
            else if (!isSpaceOccupied(&placed_pieces_tree, x+1, y)  && x+1 < max_x)
            {
                direction = RIGHT_EDGE;
            }
            else if (!isSpaceOccupied(&placed_pieces_tree, x-1, y)  && x-1 > 0)
            {
                direction = LEFT_EDGE;
            }
            else
            {
                std::cout << "No where to go" << std::endl;
            }
        }
    }

    // place pieces depth first
    placed_pieces.push_back(placed_pieces_tree.root_node->children[0]->value);
    current_node = placed_pieces_tree.root_node->children[0];

    while (current_node->children.size() != 0)
    {
        current_node = current_node->children[0];
        placed_pieces.push_back(current_node->value);
    }

    return placed_pieces;
}

int createPossiblePieceTreeChildren(TreeNode<std::shared_ptr<PuzzlePiece>>* node, 
                                    Tree<std::shared_ptr<PuzzlePiece>>* tree, 
                                    std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matching_edges, 
                                    int direction)
{
    if (matching_edges.size() == 0)
    {
        return -1;
    }

    int num_matches = 0;

    for (int possible_match_index = 0; possible_match_index < matching_edges.size(); ++possible_match_index)
    {
        std::shared_ptr<PuzzleEdge> matched_edge = std::get<0>(matching_edges[possible_match_index]);
        std::shared_ptr<PuzzlePiece> matched_piece = matched_edge->piece;

        if (matched_piece->edges[(matched_edge->edge_side + 3) % 4]->edge_type == FLAT_EDGE)
        {
            num_matches++;
            while (matched_edge->edge_side != (direction + 2) % 4 || matched_piece->edges[(direction + 1) % 4]->edge_type != FLAT_EDGE)
            {
                rotatePiece(matched_piece);
            }

            TreeNode<std::shared_ptr<PuzzlePiece>>* new_node = new TreeNode<std::shared_ptr<PuzzlePiece>>(node, matched_piece);
            tree->insertNode(node, new_node);
        }
    }

    if (num_matches == 0)
    {
        return -1;
    }

    return 0;
}

std::vector<PlacedPiece> solvePuzzleEdge(std::vector<std::shared_ptr<PuzzlePiece>> pieces)
{
    // identify corners and pieces
    std::vector<std::shared_ptr<PuzzlePiece>> edges;
    std::shared_ptr<PuzzlePiece> first_corner(nullptr);
    for (auto piece : pieces)
    {
        if (piece->piece_type == EDGE_PIECE)
        {
            edges.push_back(piece);
        }
        else if (piece->piece_type == CORNER_PIECE)
        {
            if (first_corner == nullptr)
            {
                first_corner = piece;
            }
            else
            {
                edges.push_back(piece);
            }
        }
    }

    // rotate first corner to have flat edges left and top
    while (first_corner->edges[0]->edge_type != FLAT_EDGE || first_corner->edges[1]->edge_type != FLAT_EDGE)
    {
        rotatePiece(first_corner);
    }

    TreeNode<std::shared_ptr<PuzzlePiece>> root_node(nullptr, first_corner);
    TreeNode<std::shared_ptr<PuzzlePiece>>* current_node = &root_node;
    Tree<std::shared_ptr<PuzzlePiece>> placed_pieces_tree(current_node);

    std::vector<std::shared_ptr<PuzzlePiece>> remaining_edges = edges;
    int x = 1;
    int y = 0;
    int direction = RIGHT_EDGE;

    while (remaining_edges.size() > 0)
    {
        if (current_node->children.size() == 0)
        {
            std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matching_edges = findMatchingEdges(current_node->value->edges[direction], remaining_edges);
            if (createPossiblePieceTreeChildren(current_node, &placed_pieces_tree, matching_edges, direction) == -1 || x < 0 || y < 0)
            {
                if (x < 0 || y < 0)
                {
                    current_node->children.clear();
                }

                // pop node
                while(current_node->children.size() == 0 && current_node->parent != nullptr)
                {
                    std::cout << "Popping node: " << current_node->value->id << std::endl;
                    switch (direction)
                    {
                        case RIGHT_EDGE:
                        x--;
                        break;
                        case LEFT_EDGE:
                        x++;
                        break;
                        case TOP_EDGE:
                        y++;
                        break;
                        case BOTTOM_EDGE:
                        y--;
                        break;
                    }

                    if (current_node->value->piece_type == CORNER_PIECE)
                    {
                        direction += 1;
                    }

                    remaining_edges.push_back(current_node->value);
                    TreeNode<std::shared_ptr<PuzzlePiece>>* temp_node = current_node->parent;
                    placed_pieces_tree.removeNode(current_node);
                    current_node = temp_node;
                }

                if (current_node->parent == nullptr)
                {
                    std::cout << "ABORT: No solution found" << std::endl;
                    current_node->children.clear();
                    remaining_edges.clear();
                    std::exit(-1);
                }
            }
        }
        else
        {
            // move to leftmost child
            current_node = current_node->children[0];
            std::cout << "Pushing node: " << current_node->value->id << std::endl;

            // rotate again
            std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matching_edge = findMatchingEdges(current_node->parent->value->edges[direction], std::vector<std::shared_ptr<PuzzlePiece>>{{ current_node->value }});
            std::shared_ptr<PuzzleEdge> matched_edge = std::get<0>(matching_edge[0]);
            std::shared_ptr<PuzzlePiece> matched_piece = matched_edge->piece;
            while (matched_edge->edge_side != (direction + 2) % 4 || matched_piece->edges[(direction + 1) % 4]->edge_type != FLAT_EDGE)
            {
                rotatePiece(matched_piece);
            }

            // place
            remaining_edges.erase(std::remove(remaining_edges.begin(), remaining_edges.end(), current_node->value), remaining_edges.end());
            if (current_node->value->piece_type == CORNER_PIECE)
            {
                direction -= 1;
            }

            switch (direction)
            {
                case RIGHT_EDGE:
                x++;
                break;
                case LEFT_EDGE:
                x--;
                break;
                case TOP_EDGE:
                y--;
                break;
                case BOTTOM_EDGE:
                y++;
                break;
            }
        }
    }

    std::vector<PlacedPiece> placed_pieces(0);
    // depth first search tree, place each node
    x = 1;
    y = 0;
    direction = RIGHT_EDGE;

    placed_pieces.push_back(PlacedPiece(placed_pieces_tree.root_node->value, Coordinate(0, 0)));
    current_node = placed_pieces_tree.root_node;

    while (current_node->children.size() != 0)
    {
        current_node = current_node->children[0];
        placed_pieces.push_back(PlacedPiece(current_node->value, Coordinate(x, y)));

        if (current_node->value->piece_type == CORNER_PIECE)
        {
            direction -= 1;
        }

        switch (direction)
        {
            case RIGHT_EDGE:
            x++;
            break;
            case LEFT_EDGE:
            x--;
            break;
            case TOP_EDGE:
            y--;
            break;
            case BOTTOM_EDGE:
            y++;
            break;
        }
    }

    return placed_pieces;
}

void rotatePiece(std::shared_ptr<PuzzlePiece> piece)
{
    std::shared_ptr<PuzzleEdge> saved_edge = piece->edges[3];
    for (int edge_num = 2; edge_num >= 0; --edge_num)
    {
        piece->edges[edge_num + 1] = piece->edges[edge_num];
        piece->edges[edge_num + 1]->edge_side += 1;
    }
    piece->edges[0] = saved_edge;
    piece->edges[0]->edge_side = TOP_EDGE;

    // rotate
    cv::rotate(piece->piece_mat, piece->piece_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
    for (auto edge : piece->edges)
    {
        cv::rotate(edge->edge_mat, edge->edge_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
}

bool sortbyth(const std::tuple<std::shared_ptr<PuzzleEdge>, float>& a, const std::tuple<std::shared_ptr<PuzzleEdge>, float>& b)
{
    return (std::get<1>(a) > std::get<1>(b));
}

std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> findMatchingEdges(std::shared_ptr<PuzzleEdge> current_edge, std::vector<std::shared_ptr<PuzzlePiece>> all_pieces)
{
    std::vector<std::tuple<std::shared_ptr<PuzzleEdge>, float>> matchingEdges(0);

    for (auto piece : all_pieces)
    {
        //std::cout << "\nComparing piece " << current_edge->piece->id << " with piece " << piece->id << "\n";
        for (auto edge : piece->edges)
        {
            float edge_similarity = compareEdges(*edge, *current_edge);
            //std::cout << edge_similarity << ", ";

            if (edge_similarity > PIECE_SIMILARITY_THRESHOLD)// && &edge != &current_edge)
            {
                matchingEdges.push_back(std::tuple(edge, edge_similarity));
            }
        }
        //std::cout << "\n\n";
    }

    // sort by similarity
    sort(matchingEdges.begin(), matchingEdges.end(), sortbyth);

    return matchingEdges;
}

float getLineImageDifference(cv::Mat mat1, cv::Mat mat2) // mats need to be same size
{
    cv::Mat mat1_filled, mat2_filled, diff_mat;
    mat1_filled = mat1.clone();
    mat2_filled = mat2.clone();
    cv::floodFill(mat1_filled, cv::Point(mat1.size().width/2, mat1.size().height/2), cv::Scalar(200));
    cv::floodFill(mat2_filled, cv::Point(mat2.size().width/2, mat2.size().height/2), cv::Scalar(200));
    cv::absdiff(mat1_filled, mat2_filled, diff_mat);

    int total_pixels = mat1.size().width * mat1.size().height;
    int different_pixels = cv::countNonZero(diff_mat);
    float normalised_difference = (float)different_pixels / (float)total_pixels;

    return normalised_difference;
}

float compareEdges(PuzzleEdge edge1, PuzzleEdge edge2)
{
    if (edge1.edge_type == FLAT_EDGE && edge2.edge_type == FLAT_EDGE)
    {
        return 1.0;
    }

    if (edge1.edge_type + edge2.edge_type != IN_EDGE + OUT_EDGE)
    {
        return 0.0;
    }

    // rotate edge2 to match edge1
    cv::Mat rotated_edge2_mat = edge2.edge_mat.clone();
    int rotation_difference = edge1.edge_side - edge2.edge_side + 2;
    cv::RotateFlags rotational_flag = rotation_difference < 0 ? cv::ROTATE_90_CLOCKWISE : cv::ROTATE_90_COUNTERCLOCKWISE;
    for (int i = 1; i <= std::abs(rotation_difference); ++i)
    {
        cv::rotate(rotated_edge2_mat, rotated_edge2_mat, rotational_flag);
    }

    cv::Size edge1Size = edge1.edge_mat.size();
    cv::Size edge2Size = rotated_edge2_mat.size();
    int min_height = std::min(edge1Size.height, edge2Size.height);
    int min_width = std::min(edge1Size.width, edge2Size.width);
    cv::Rect crop(0,0,min_width,min_height);
    float smallest_difference = getLineImageDifference(edge1.edge_mat(crop), rotated_edge2_mat(crop));

    return 1.0 - smallest_difference;
}

void showSolvedPuzzle(std::vector<PlacedPiece> placed_pieces)
{
    int width = 0;
    int height = 0;

    for (auto placed_piece : placed_pieces)
    {
        if (std::get<0>(std::get<1>(placed_piece)) > width)
        {
            width = std::get<0>(std::get<1>(placed_piece));
        }
        if (std::get<1>(std::get<1>(placed_piece)) > height)
        {
            height = std::get<1>(std::get<1>(placed_piece));
        }
    }

    int image_resolution_x = 1080; // TODO work these out
    int image_resolution_y = 1080;

    cv::Mat solved_puzzle((height+1)*image_resolution_y, (width+1)*image_resolution_x, CV_8U, cv::Scalar(0));

    for (auto placed_piece : placed_pieces)
    {
        // copy puzzle piece onto coordinate
        int x = std::get<0>(std::get<1>(placed_piece));
        int y = std::get<1>(std::get<1>(placed_piece));

        if (x >= 0 && y >= 0)
        {
            cv::Rect paste_dst(x*image_resolution_x, y*image_resolution_y, image_resolution_x, image_resolution_y);
            std::get<0>(placed_piece)->piece_mat.copyTo(solved_puzzle(paste_dst));
        }
    }

    cv::imwrite("../tests_output/solved_puzzle.png", solved_puzzle);
    //cv::imshow(std::to_string(std::rand()), solved_puzzle);
    //cv::waitKey(0);
}

template <typename T>
TreeNode<T>::TreeNode(TreeNode<T>* parent, T value)
{
    this->parent = parent;
    this->value = value;
    children = std::vector<TreeNode<T>*>(0);
}

template <typename T>
Tree<T>::Tree(TreeNode<T>* root_node)
{
    this->root_node = root_node;
    this->nodes = std::vector<TreeNode<T>*>{{ root_node }};
}

template <typename T>
void Tree<T>::insertNode(TreeNode<T>* parent, TreeNode<T>* new_node)
{
    parent->children.push_back(new_node);
    nodes.push_back(new_node);
}

template <typename T>
void Tree<T>::removeNode(TreeNode<T>* node)
{
    nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());
    node->parent->children.erase(std::remove(node->parent->children.begin(), node->parent->children.end(), node), node->parent->children.end()); // remove reference to node
    delete node;
}