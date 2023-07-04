# Jigsaw Puzzle Generator and Solver

Generates high resolution images of jigsaw puzzle pieces, for a jigsaw of a given number of pieces. Uses an image, the width and height proportions of which the jigsaw tries to match.

Solves the jigsaw puzzle given a folder of these generated single piece image. Works well for 20-40 pieces, but as the jigsaw gets bigger there are more puzzles that take a very long time to solve or aren't solved.

Each piece is blank. The aim of this project is to solve a jigsaw just from the piece shapes, not any image printed on the jigsaw.

Below is an example of a puzzle generated with 20 pieces:

![Alt text](examples/generate_examples/example1/jigsaw.png?raw=true "Generated jigsaw puzzle")

Each piece is also rendered individually at higher quality:

![Alt text](examples/generate_examples/example1/pieces/5.png?raw=true "Generated jigsaw puzzle piece")
![Alt text](examples/generate_examples/example1/pieces/10.png?raw=true "Generated jigsaw puzzle piece")

The solver uses the pieces folder, and produces the following solution:

![Alt text](examples/solve_examples/example1/solved_puzzle.png?raw=true "Solved jigsaw puzzle")

This has a different rotation to the original generated jigsaw, but each piece is in the correct position and orientation.

A bigger example using 50 pieces can be found in the examples folder.
