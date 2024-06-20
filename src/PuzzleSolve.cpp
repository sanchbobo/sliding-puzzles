/*
Documentation for PuzzleSolve.cpp

  Author Information
  Name: Oleksandr Bondarenko
  Student ID: w1936916

File Description
This C++ program appears to involve computational logic for solving a puzzle, possibly related to grid navigation or arrangement, 
as indicated by the usage of Position structures and hashing.

Includes
- <iostream>: For input and output stream operations.
- <vector>: To use the vector container.
- <queue>: For queue data structures.
- <unordered_map>: To use associative containers that store elements formed by the combination of a key value and a mapped value.
- <utility>: This includes utility components, potentially for pair and tuple operations.
- <algorithm>: For functions that perform algorithm operations (e.g., sort, search).
- <fstream>: For file stream operations.

Data Structures
  Position
	- x (int): The x-coordinate of the position.
	- y (int): The y-coordinate of the position.
	- operator==: Compares two Position objects for equality.
	- operator!=: Compares two Position objects for inequality.
	- friend std::ostream& operator<<: Outputs the position to a standard output stream in the format (x,y).
	  std::hash<Position>
	- operator(): Computes a hash value for a given Position object.
	- hashInt: Computes a hash value for an integer.
	- combineHashes: Combines two hash values into a single hash value.

Functions
  getNodes
	- Parameters: const Position& pos, const std::vector<std::string> grid
	- Return Type: std::vector<Position>
	- Description: Computes the nodes of the graph from a given position on a grid based on sliding movement rules.

  bfs
	- Parameters: const std::vector<std::string>& grid, const Position& start, const Position& finish, std::unordered_map<Position, Position>& pathTrack
	- Return Type: bool
	- Description: Performs a breadth-first search (BFS) to find a path from the start to the finish position on a grid.

  getPathTrack
	- Parameters: Position start, Position finish, std::unordered_map<Position, Position>& pathTrack
	- Return Type: std::vector<Position>
	- Description: Reconstructs the path from the start position to the finish position using a tracking map.

  readGridFromFile
	- Parameters: const std::string& filename
	- Return Type: std::vector<std::string>
	- Description: Reads a grid from a specified file and returns it as a vector of strings.

  findStartPosition
	- Parameters: const std::vector<std::string>& grid
	- Return Type: Position
	- Description: Searches for the starting position ('S') in a grid and returns its coordinates.

  findFinishPosition
	- Parameters: const std::vector<std::string>& grid
	- Return Type: Position
	- Description: Searches for the finish position ('F') in a grid and returns its coordinates.

  printPath
	- Parameters: const std::vector<Position>& path
	- Return Type: void
	- Description: Prints the steps of a path as a sequence of directions and positions to the console.

  solvePuzzle
	- Parameters: const std::vector<std::string>& grid
	- Return Type: void
	- Description: Solves a puzzle by finding a path from the start position to the finish position on a given grid.

Main Function
- Reads a grid from a file named "puzzle_20.txt" using readGridFromFile.
- Calls solvePuzzle with the read grid to solve the puzzle.
- Returns 0 to indicate successful program execution.

Additional Comments
- The program appears to be designed to solve a puzzle by finding a path from the start to the finish position on a grid.
- It uses a breadth-first search (BFS) algorithm to explore the grid and find a viable path.
- The program reads the grid from a file, identifies the start and finish positions, and outputs the path if found.
- The Position struct represents a coordinate point in a 2D space, and the std::hash specialization enables Position objects to be used in hash-based containers.
- The program includes utility functions for grid navigation, pathfinding, and path reconstruction.

*/


#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <fstream>

/**
 * @struct Position
 * @brief Defines a position on a two-dimensional grid using x and y coordinates.
 *
 * This struct is designed to represent a coordinate point in a 2D space, such as a position
 * on a game board or a grid layout. It provides utility functions to compare positions and
 * output them to a stream.
 */
struct Position {
    int x; ///< The x-coordinate of the position.
    int y; ///< The y-coordinate of the position.

    /**
     * Compares this position with another position for equality.
     *
     * @param other The other position to compare against.
     * @return true if both x and y coordinates of this position and the other are equal, false otherwise.
     */
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    /**
     * Compares this position with another position for inequality.
     *
     * @param other The other position to compare against.
     * @return true if either the x or y coordinates of this and the other position do not match, false otherwise.
     */
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }

    /**
     * Outputs the position to a standard output stream in the format (x,y).
     *
     * This function overloads the stream insertion operator to provide an easy way
     * to output the coordinates of the position in a human-readable format.
     *
     * @param os The output stream to which the coordinates are to be sent.
     * @param pos The position whose coordinates are to be outputted.
     * @return A reference to the output stream to allow for chaining of output operations.
     */
    friend std::ostream& operator<<(std::ostream& os, const Position& pos) {
        os << '(' << pos.x << "," << pos.y << ')';
        return os;
    }
};


/**
 * @namespace std
 * Specialization of the std::hash template for the Position struct.
 * This enables Position objects to be used as keys in hash-based containers like std::unordered_map.
 */

namespace std {
    /**
     * @struct hash<Position>
     * @brief Specializes the std::hash template for the Position struct.
     *
     * This struct provides a custom hash function for Position objects, which is necessary
     * when Positions are used as keys in the standard unordered associative containers that
     * require a hash function, such as std::unordered_map and std::unordered_set.
     */
    template<>
    struct hash<Position> {
        /**
         * Computes a hash value for a given Position object.
         *
         * The hash is computed by combining the hashes of the individual x and y coordinates
         * of the position, ensuring a uniform distribution of hash values for positions across
         * a 2D space which improves performance in hash-based containers.
         *
         * @param pos The position for which the hash is to be computed.
         * @return A size_t representing the hash value of the position.
         */
        size_t operator()(const Position& pos) const {
            return combineHashes(pos.x, pos.y);
        }

    private:
        /**
         * Computes a hash value for an integer.
         *
         * Utilizes the standard std::hash<int> functor to generate a hash value for an integer.
         *
         * @param i The integer to hash.
         * @return A size_t representing the hash value of the integer.
         */
        static size_t hashInt(int i) {
            return std::hash<int>()(i);
        }

        /**
         * Combines two hash values into a single hash value.
         *
         * Uses a combination of shifting and exclusive OR to blend the hash values of
         * the x and y coordinates of a position. This method reduces the chances of
         * collisions in hash values for different positions.
         *
         * @param x The hash value of the x-coordinate.
         * @param y The hash value of the y-coordinate.
         * @return A size_t representing the combined hash value.
         */
        static size_t combineHashes(int x, int y) {
            const size_t shift = 16;  // A chosen shift value to mix the bits.
            size_t hashX = hashInt(x);
            size_t hashY = hashInt(y);

            // Combine hashes by shifting and using bitwise operations to reduce collision.
            return hashX ^ (hashY << shift | hashY >> (32 - shift)); 
        }
    };
}



/**
 * @brief Computes the nodes of the graph from a given position on a grid based on sliding movement rules.
 *
 * This function identifies all possible positions that can be reached from the current position 
 * by sliding in one of the four cardinal directions (left, right, up, and down) until an obstacle 
 * ('0') or the boundaries of the grid are encountered. If the finish ('F') is reached during the slide,
 * it stops and includes that position as a neighbor.
 *
 * @param pos The current position on the grid.
 * @param grid The 2D grid represented as a vector of strings, where each string represents a row of the grid.
 *             Each character in the string can be '.', '0', 'S', or 'F' representing an open space, obstacle,
 *             start, or finish respectively.
 * @return std::vector<Position> A vector containing all valid positions that can be reached from the current
 *         position according to the sliding rules of the puzzle.
 */
std::vector<Position> getNodes(const Position& pos, const std::vector<std::string> grid) {
    std::vector<Position> nodes;
    // Directions are represented as pairs of x (column offset) and y (row offset)
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, 1}, {0, -1}};
    
    for (const std::pair<int, int>& direction : directions) {
        Position next = pos;
        bool foundFinish = false;
        // Slide in the direction until an obstacle or boundary is encountered
        while (true) {
            next.x += direction.first;
            next.y += direction.second;
            // Check for boundaries and obstacles
            if (next.x < 1 || next.x > grid[0].size() || next.y < 1 || next.y > grid.size() || grid[next.y - 1][next.x - 1] == '0') {
                // Step back to the last valid position if an obstacle or boundary is hit
                next.x -= direction.first;
                next.y -= direction.second;
                break;
            } else if (grid[next.y - 1][next.x - 1] == 'F') {
                // Stop if the finish is found
                foundFinish = true;
                break;
            }
        }
        // Include the position if it is different from the starting position or if the finish was found
        if (next != pos || foundFinish) {
            nodes.push_back(next);
        }
    }
    return nodes;
}



/**
 * @brief Performs a breadth-first search (BFS) to find a path from the start to the finish position on a grid.
 *
 * This function explores the grid by expanding outwards from the start position in all allowable directions
 * using the rules defined by getNodes. It tracks the path traversed using a hash map that associates
 * each visited position with the position from which it was first reached. If the finish position is reached,
 * the function returns true, indicating that a path exists.
 *
 * @param grid The grid represented as a vector of strings, where each string is a row of the grid.
 *             Each character in the string represents the state of the grid cell:
 *             '.', '0', 'S', or 'F' (open, obstacle, start, or finish).
 * @param start The starting position for the BFS.
 * @param finish The target position to reach.
 * @param pathTrack A hash map that records the predecessor of each visited position, allowing the path to be
 *                  reconstructed if necessary.
 * @return true if a path to the finish is found, false otherwise.
 */
bool bfs(const std::vector<std::string>& grid, const Position& start, const Position& finish, std::unordered_map<Position, Position>& pathTrack) {
    std::queue<Position> vertexQueue; // Queue to manage the BFS frontier.
    vertexQueue.push(start); // Start from the initial position.
    pathTrack[start] = start; // Initialize the path tracking with the start position.
    
    while (!vertexQueue.empty()) {
        Position current = vertexQueue.front(); // Get the current position to explore.
        vertexQueue.pop(); // Remove the current position from the queue.
        
        if (current == finish) {
            return true; // Return true immediately if the finish is reached.
        }
        
        // Explore all neighboring positions from the current position.
        for (const Position& next : getNodes(current, grid)) {
            if (!pathTrack.count(next)) { // Check if the position has not been visited.
                vertexQueue.push(next); // Add the position to the queue for further exploration.
                pathTrack[next] = current; // Track the path by associating the new position with its predecessor.
            }
        }
    }
    
    return false; // Return false if no path to the finish was found.
}


/**
 * @brief Reconstructs the path from the start position to the finish position using a tracking map.
 *
 * This function traces back the path from the finish to the start by following the predecessor
 * relationships recorded in the pathTrack map. The function assumes that a complete path exists
 * in the pathTrack map, which must have been filled by a successful search algorithm like BFS.
 *
 * @param start The starting position of the path.
 * @param finish The finish position of the path.
 * @param pathTrack A hash map where each key is a Position that was reached during the search
 *                  and each value is the Position from which the key Position was first reached.
 * @return std::vector<Position> A vector of positions forming a path from the start to the finish,
 *         including both the start and finish positions and ordered from start to finish.
 */
std::vector<Position> getPathTrack(Position start, Position finish, std::unordered_map<Position, Position>& pathTrack) {
    std::vector<Position> path; // Vector to store the path from start to finish.

    // Start from the finish position and trace back to the start position using the pathTrack map.
    for (Position coordinates = finish; coordinates != start; coordinates = pathTrack[coordinates]) {
        path.push_back(coordinates); // Add the current position to the path.
    }
    path.push_back(start); // Add the start position to the path as the final step of backtracking.

    reverse(path.begin(), path.end()); // Reverse the path to order it from start to finish.
    return path; // Return the reconstructed path.
}


/**
 * @brief Reads a grid from a specified file and returns it as a vector of strings.
 *
 * This function opens a text file containing a grid layout where each line of the file
 * represents a row of the grid. It reads the file line by line and stores each line in a
 * vector. If the file cannot be opened, it displays an error message and returns an empty
 * vector. This function is typically used to load puzzle grids or game boards from files,
 * where each character in a line represents a specific cell in the grid.
 *
 * @param filename The path and name of the file to read from.
 * @return std::vector<std::string> A vector of strings, where each string represents a row of the grid.
 *         Returns an empty vector if the file cannot be opened or read.
 */
std::vector<std::string> readGridFromFile(const std::string& filename) {
    std::ifstream inputFile(filename); // Stream for reading the file.

    // Check if the file is successfully opened.
    if (!inputFile.is_open()) {
        std::cout << "Error opening the file" << '\n'; // Display an error message if the file cannot be opened.
        return {}; // Return an empty vector as an error signal.
    }

    std::vector<std::string> grid; // Vector to store the grid lines.
    std::string line; // Temporary string to hold each read line.

    // Read the file line by line.
    while (getline(inputFile, line)) {
        grid.push_back(line); // Add each line to the grid vector.
    }

    return grid; // Return the vector containing the grid.
}


/**
 * @brief Searches for the starting position ('S') in a grid and returns its coordinates.
 *
 * This function iterates over each cell in a 2D grid represented as a vector of strings,
 * searching for the character 'S' that indicates the start position. If found, it returns
 * the position as a Position object with x and y coordinates corresponding to the column
 * and row indices, respectively. If the start position is not found, it returns an invalid
 * position indicated by coordinates {-1, -1}.
 *
 * @param grid The grid represented as a vector of strings, where each string is a row,
 *             and each character in the string represents a cell in the grid.
 * @return Position The coordinates of the start position. If the start position is not found,
 *         returns {-1, -1} indicating an invalid position.
 */
Position findStartPosition(const std::vector<std::string>& grid) {
    // Iterate over each row of the grid.
    for (int i = 1; i <= grid.size(); i++) {
        // Iterate over each column in the current row.
        for (int j = 1; j <= grid[i].size(); j++) {
            // Check if the current cell contains the start position 'S'.
            if (grid[i - 1][j - 1] == 'S') {
                return {j, i}; // Return the position as {column index, row index}.
            }
        }
    }
    return {-1, -1}; // Return an invalid position if 'S' is not found.
}


/**
 * @brief Searches for the finish position ('F') in a grid and returns its coordinates.
 *
 * This function iterates over each cell in a 2D grid represented as a vector of strings,
 * searching for the character 'F' that indicates the finish position. If found, it returns
 * the position as a Position object with x and y coordinates corresponding to the column
 * and row indices, respectively. If the finish position is not found, it returns an invalid
 * position indicated by coordinates {-1, -1}.
 *
 * @param grid The grid represented as a vector of strings, where each string is a row,
 *             and each character in the string represents a cell in the grid.
 * @return Position The coordinates of the finish position. If the finish position is not found,
 *         returns {-1, -1} indicating an invalid position.
 */
Position findFinishPosition(const std::vector<std::string>& grid) {
    // Iterate over each row of the grid.
    for (int i = 1; i <= grid.size(); i++) {
        // Iterate over each column in the current row.
        for (int j = 1; j <= grid[i].size(); j++) {
            // Check if the current cell contains the finish position 'F'.
            if (grid[i - 1][j - 1] == 'F') {
                return {j, i}; // Return the position as {column index, row index}.
            }
        }
    }
    return {-1, -1}; // Return an invalid position if 'F' is not found.
}


/**
 * @brief Prints the steps of a path as a sequence of directions and positions to the console.
 *
 * This function takes a vector of Position objects, which represents a path from a start to a finish
 * position, and prints each move as a step starting from the start position. Each step specifies the
 * direction moved (left, right, up, down) and the new position after making the move. It starts by
 * printing the start position, followed by each movement direction and the subsequent position,
 * and concludes with a "Done!" message indicating the end of the path.
 *
 * @param path A vector of Position objects representing the path to be printed. Each position
 *             in the vector should be one move away from the previous position based on the rules
 *             of the grid or the pathfinding algorithm used.
 */
void printPath(const std::vector<Position>& path) {
    if (path.empty()) {
        std::cout << "The path is empty. No moves to display.\n";
        return;
    }

    // Print the start position.
    std::cout << "1.  Start at " << path[0] << '\n';

    // Iterate through the path and determine the direction of each step.
    for (int i = 1; i < path.size(); i++) {
        std::string direction; // Holds the direction of movement.
        
        if (path[i].x < path[i - 1].x) {
            direction = "left";
        } else if (path[i].y < path[i - 1].y) {
            direction = "down";
        } else if (path[i].x > path[i - 1].x) {
            direction = "right";
        } else {
            direction = "up";
        }

        // Format and print the step number and direction.
        std::cout << i + 1 << ". ";
        if (i < 9) { // Adjust spacing for single-digit steps.
            std::cout << " ";
        }
        std::cout << "Move " << direction << " to " << path[i] << '\n';
    }

    // Print the conclusion of the path.
    std::cout << path.size() + 1 << ". ";
    if (path.size() < 10) {
        std::cout << " ";
    }
    std::cout << "Done!" << '\n';
}


/**
 * @brief Solves a puzzle by finding a path from the start position to the finish position on a given grid.
 *
 * This function handles the process of solving a puzzle on a grid by identifying the start and finish positions,
 * conducting a BFS to find a viable path between them, and printing the path if one exists. If no path is found,
 * it outputs a message indicating the absence of a path. The function leverages other functions like findStartPosition,
 * findFinishPosition, bfs, getPathTrack, and printPath to manage these tasks.
 *
 * @param grid The grid represented as a vector of strings, where each string is a row of the grid and each character
 *             in the string represents a cell in the grid. The grid cells may contain '.', '0', 'S', or 'F' indicating
 *             open space, obstacles, start, and finish, respectively.
 */
void solvePuzzle(const std::vector<std::string>& grid) {
    // Find the start and finish positions in the grid.
    Position start = findStartPosition(grid);
    Position finish = findFinishPosition(grid);

    // Ensure both start and finish were found.
    if (start.x == -1 || finish.x == -1) {
        std::cout << "Error: Start or finish position not found in the grid." << '\n';
        return;
    }

    // Perform a BFS to determine if there is a path from start to finish.
    std::unordered_map<Position, Position> pathTrack; // Map to track the path from one position to another.
    bool found = bfs(grid, start, finish, pathTrack); // BFS to find the path.

    if (found) {
        // If a path is found, reconstruct and print the path.
        std::vector<Position> path = getPathTrack(start, finish, pathTrack);
        printPath(path);
    } else {
        // Output a message if no path is found.
        std::cout << "No path found!" << '\n';
    }
}


/**
 * @brief Entry point of the puzzle solver application.
 *
 * This main function initializes the application by reading a grid from a specified file and then
 * using it to solve a puzzle. The grid is expected to be in a file where each line corresponds to
 * a row of the puzzle grid, with specific characters representing different elements of the puzzle
 * (e.g., '.', '0', 'S', 'F' for open space, obstacles, start, and finish, respectively). After reading
 * the grid, the function calls solvePuzzle to attempt to find and display a path from the start to
 * the finish position. If the file cannot be read or the puzzle cannot be solved, appropriate messages
 * are displayed.
 *
 * @return int Returns 0 upon successful completion of the application, indicating that the program
 *         executed without errors.
 */
int main() {
    // Read the grid from a file named "puzzle_10.txt".
    std::vector<std::string> grid = readGridFromFile("puzzle_2560.txt");

    // Attempt to solve the puzzle using the read grid.
    solvePuzzle(grid);

    // Return 0 to indicate successful execution.
    return 0;
}
