#include "generator.hpp"
#include "solver.hpp"

int main()
{
    generatePuzzle("../examples/generate_examples/target_ratio.png", 50);
    solvePuzzle();
    return 0;
}