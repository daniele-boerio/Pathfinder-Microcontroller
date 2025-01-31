/*
 * mapOperations.c
 *
 *  Created on: Oct 18, 2024
 *      Author: daniele-boerio
 */

#include "mapOperations.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>

Point boundaries[MAX_POINTS];  // Vertices of the polygon
uint8_t boundary_count = 0;  // number of vertices

Node* neighbors[4] = {NULL};
Node** nodes;
uint8_t rows;
uint8_t cols;
double cellSize;

long minX, maxX;
long minY, maxY;

char buff[50];
char* mex = "";

/**
 * @brief Calculates the number of steps for a wheel to rotate by a given angle in radians.
 *
 * This function computes the number of motor steps needed to rotate the wheel by a certain
 * angle (in radians), based on the wheel's diameter, the wheel distance, and the number of
 * steps per full revolution of the wheel.
 *
 * @param radiants The angle in radians for the desired wheel rotation.
 *
 * @return The number of steps required for the wheel to rotate by the given angle.
 */
long calcolaNumeroPassiRotazione(double radiants) {
	// Formula to calculate the number of steps based on the angle in radians
	return (long)((radiants * DISTANZA_RUOTE/2  * PASSI_PER_GIRO_RUOTA)/(DIAMETRO_RUOTA * M_PI));
}

/**
 * @brief Calculates the rotation angle in radians for a given number of motor steps.
 *
 * This function computes the rotation angle in radians based on the number of steps taken
 * by the motor. It uses the wheel's diameter, the number of steps per revolution, and the
 * wheel distance to perform the calculation.
 *
 * @param n_passi The number of motor steps.
 *
 * @return The rotation angle in radians corresponding to the given number of steps.
 */
double calcolaAngoloRotazione(unsigned long n_passi) {
	// Formula to calculate the rotation angle in radians
	return (double) ((n_passi * (DIAMETRO_RUOTA * M_PI)) / (PASSI_PER_GIRO_RUOTA * DISTANZA_RUOTE/2));
}

/**
 * @brief Calculates the required steps and rotation angle to reach a target point.
 *
 * This function calculates the number of motor steps and the rotation angle needed to
 * move the robot from its current position to the target point. It uses the Euclidean
 * distance formula to calculate the number of steps, and the `atan2` function to
 * calculate the angle. The rotation angle is adjusted to ensure it stays within the
 * range of -π to π radians.
 *
 * @param target_point The target point to reach.
 * @param steps The pointer to store the number of steps required to reach the target point.
 * @param rotation_angle The pointer to store the required rotation angle to face the target point.
 */
void calculate_steps_and_angle(Point target_point, unsigned long *steps, double *rotation_angle) {
    // Calculate the difference in x and y
    long delta_x = (long) target_point.x - current_position.x;
    long delta_y = (long) target_point.y - current_position.y;

    // Calculate the Euclidean distance (number of steps to take)
    *steps = (unsigned long) sqrt((double)delta_x * delta_x + (double)delta_y * delta_y);

    // Calculate the angle between the current position and the target point
    double target_angle = (double) atan2(delta_y, delta_x);  // Angle in radians

    // Calculate the required rotation (difference between target angle and current angle)
    *rotation_angle = target_angle - current_angle;

    // Adjust the rotation angle to be within the range of -π to π radians
    if(*rotation_angle > M_PI){
        *rotation_angle -= 2 * M_PI;
    } else if (*rotation_angle < -M_PI){
        *rotation_angle += 2 * M_PI;
    }
}

/**
 * @brief Heuristic function for Euclidean distance.
 *
 * This heuristic function computes the Euclidean distance between two nodes. It is typically
 * used in pathfinding algorithms such as A* to estimate the cost of reaching the destination
 * from a given node. In this case, it calculates the Manhattan distance (sum of absolute differences).
 *
 * @param a The first node.
 * @param b The second node.
 *
 * @return The Euclidean distance between the two nodes.
 */
double heuristic(Node a, Node b) {
	return fabs(a.row - b.row) + fabs(a.column - b.column);
}

/**
 * @brief Calculates the Manhattan distance from a node to the closest boundary.
 *
 * This function calculates the minimum distance from the given node to the nearest boundary of the grid.
 * The distance is calculated as the Manhattan distance, considering the closest boundary on both the row
 * and column axes.
 *
 * @param node The node for which the distance to the nearest boundary is calculated.
 *
 * @return The Manhattan distance to the closest boundary.
 */
double heuristicToBoundary(Node node) {
    // Calculate the minimum distance to the top or bottom boundary
    int minRowDist = fmin(node.row, rows - 1 - node.row);
    // Calculate the minimum distance to the left or right boundary
    int minColDist = fmin(node.column, cols - 1 - node.column);
    // Return the Manhattan distance to the closest boundary
    return (double) (minRowDist + minColDist);
}

/**
 * @brief Checks if a point is inside a polygon using the ray-casting algorithm.
 *
 * This function determines whether a given point is inside a polygon by counting the number of
 * intersections between a horizontal ray originating from the point and the edges of the polygon.
 * If the number of intersections is odd, the point is inside the polygon; otherwise, it's outside.
 *
 * @param p The point to check.
 *
 * @return True if the point is inside the polygon, false otherwise.
 */
bool isPointInPolygon(Point p) {
    int count = 0;
    // Loop through all edges of the polygon
    for (int i = 0; i < boundary_count; i++) {
        Point v1 = boundaries[i];
        Point v2 = boundaries[(i + 1) % boundary_count];

        // Check if the ray intersects the edge
        if ((v1.y > p.y) != (v2.y > p.y)) {
            // Calculate the x-coordinate of the intersection
            double x_intersection = v1.x + (double)(p.y - v1.y) * (v2.x - v1.x) / (v2.y - v1.y);
            // If the intersection is to the right of the point, increment the count
            if (x_intersection > p.x) {
                count++;
            }
        }
    }

    // Return true if the number of intersections is odd (inside the polygon)
    return count % 2 == 1;
}

/**
 * @brief Checks if a point is on a polygon segment.
 *
 * This function checks whether a given point lies on one of the edges of the polygon,
 * considering both vertical and non-vertical segments. It first checks for vertical segments,
 * and then calculates the line equation (slope and intercept) for non-vertical segments.
 *
 * @param p The point to check.
 *
 * @return True if the point lies on a polygon segment, false otherwise.
 */
bool isPointOnSegment(Point p) {
    // Loop through all edges of the polygon
    for (int i = 0; i < boundary_count; i++) {
        Point p1 = boundaries[i];
        Point p2 = boundaries[(i + 1) % boundary_count];

        if (p1.x == p2.x) {
            // Vertical line case: Check if the point lies on the vertical segment
            if (p.x == p1.x) {
                if((p.y >= p1.y && p.y <= p2.y) || (p.y <= p1.y && p.y >= p2.y)) {
                    return true;
                } else {
                    continue;
                }
            } else {
                continue; // x is not on the vertical segment
            }
        }

        // Calculate the slope (m) for non-vertical segments
        double m = (p2.y - p1.y) / (p2.x - p1.x);

        // Calculate the y-intercept (q) for the line
        double q = p1.y - m * p1.x;

        // Calculate the expected y for the given x-coordinate
        double y = m * p.x + q;

        // Check if x is within the segment range and if y matches the point's y
        if (p.x >= (p1.x < p2.x ? p1.x : p2.x) && p.x <= (p1.x > p2.x ? p1.x : p2.x)) {
            if (round(y) == p.y) {
                return true;
            } else {
                continue;
            }
        } else {
            continue; // x is not valid for the segment
        }
    }
    return false;
}

/**
 * @brief Checks if a point is between two other points.
 *
 * This function checks if a given point lies between two other points along both the x and y axes.
 * It compares the x and y coordinates of the point with the coordinates of the two points to
 * determine if the point is within the bounds of the two points.
 *
 * @param p The point to check.
 * @param p1 The first point.
 * @param p2 The second point.
 *
 * @return True if the point lies between the two other points, false otherwise.
 */
bool isPointBetweenTwoPoints(Point p, Point p1, Point p2) {
    // Check if the point lies within the bounds of the two points on the x-axis
    if (p.x >= (p1.x < p2.x ? p1.x : p2.x) && p.x <= (p1.x > p2.x ? p1.x : p2.x)) {
        // Check if the point lies within the bounds of the two points on the y-axis
        if (p.y >= (p1.y < p2.y ? p1.y : p2.y) && p.y <= (p1.y > p2.y ? p1.y : p2.y)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks if a grid cell is located on a boundary segment of the polygon.
 *
 * This function checks if the bottom-left corner of a grid cell lies on any boundary segment
 * of the polygon. It converts the cell's row and column into coordinates and then checks
 * if the point is on a polygon segment.
 *
 * @param row The row index of the grid cell.
 * @param column The column index of the grid cell.
 *
 * @return True if the cell's bottom-left corner lies on a boundary segment, false otherwise.
 */
bool isCellOnSegment(int8_t row, int8_t column) {
    // Calculate the coordinates of the bottom-left corner of the grid cell
    double xLeft = minX + column * cellSize;
    double yBottom = minY + row * cellSize;

    // Loop through all the boundary segments of the polygon
    for (int i = 0; i < boundary_count; i++) {
        Point p1 = boundaries[i];
        Point p2 = boundaries[(i + 1) % boundary_count];
        Point target = {xLeft, yBottom};

        // Check if the segment intersects the grid cell using the helper function
        if(isSegmentInsideSquare(p1, p2, target)){
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks if a segment intersects with a square.
 *
 * This function checks if the segment between two points intersects the grid cell's bottom-left corner.
 * It performs a geometric check using the cross product and checks if the point lies on the segment
 * and within the bounds of the segment.
 *
 * @param p1 The first point of the segment.
 * @param p2 The second point of the segment.
 * @param target The point to check.
 *
 * @return True if the segment intersects the square, false otherwise.
 */
bool isSegmentInsideSquare(Point p1, Point p2, Point target) {
    // Tolerance for floating-point comparisons
    const double epsilon = 1e-6;

    // Condition 1: Check if the point is on the line defined by the segment
    double crossProduct = (target.y - p1.y) * (p2.x - p1.x) - (target.x - p1.x) * (p2.y - p1.y);
    if (fabs(crossProduct) > epsilon) {
        return false;  // The point is not on the line
    }

    // Condition 2: Check if the point is within the bounds of the segment
    double minX = fmin(p1.x, p2.x), maxX = fmax(p1.x, p2.x);
    double minY = fmin(p1.y, p2.y), maxY = fmax(p1.y, p2.y);

    if (target.x < minX || target.x > maxX || target.y < minY || target.y > maxY) {
        return false;  // The point is outside the segment
    }

    return true;  // The point is on the segment
}

/**
 * @brief Computes the grid dimensions and cell size based on the polygon boundaries.
 *
 * This function calculates the dimensions of the grid and the size of the grid cells, ensuring
 * that the grid's total number of cells does not exceed the maximum allowed number of boundaries.
 * It adjusts the cell size accordingly if the grid exceeds the maximum boundary count.
 *
 * @param cellSize The calculated cell size (output).
 * @param rows The number of rows in the grid (output).
 * @param cols The number of columns in the grid (output).
 */
void computeGridDimensions(double *cellSize, uint8_t *rows, uint8_t *cols) {
    long width = maxX - minX;
    long height = maxY - minY;

    // Start with an initial cell size >= 3500
    *cellSize = 3500;

    // Calculate the initial number of columns and rows based on the cell size
    *cols = (int)ceil((double)width / *cellSize);
    *rows = (int)ceil((double)height / *cellSize);

    // Ensure the total number of cells does not exceed MAX_BOUNDARIES
    while ((*rows) * (*cols) > MAX_BOUNDARIES) {
        // Increase the cell size gradually if too many cells are created
        *cellSize += 100;
        *cols = (int)ceil((double)width / *cellSize);
        *rows = (int)ceil((double)height / *cellSize);
    }

    // Ensure the cell size is >= 3500 and the number of cells is within the boundary limits
}

/**
 * @brief Creates and initializes the grid nodes based on the polygon boundaries.
 *
 * This function calculates the grid's minimum and maximum x and y coordinates based on the polygon
 * boundary points, then initializes the grid dimensions and allocates memory for the nodes. Each node
 * represents a grid cell, and its properties (boundary, obstacle, validity) are set accordingly.
 *
 * @return True if the grid nodes were created successfully, false otherwise.
 */
bool createNodes(void) {
    // Initialize the grid's min and max coordinates based on the polygon boundaries
    minX = boundaries[0].x;
    maxX = boundaries[0].x;
    minY = boundaries[0].y;
    maxY = boundaries[0].y;

    for (int i = 1; i < boundary_count; i++) {
        if (boundaries[i].x < minX) minX = boundaries[i].x;
        if (boundaries[i].x > maxX) maxX = boundaries[i].x;
        if (boundaries[i].y < minY) minY = boundaries[i].y;
        if (boundaries[i].y > maxY) maxY = boundaries[i].y;
    }

    // Compute grid dimensions and the cell size
    computeGridDimensions(&cellSize, &rows, &cols);

    // Allocate memory for the nodes (rows x columns)
    nodes = malloc(rows * sizeof(Node *));
    if (nodes == NULL) {
        mex = "Errore prima dimensione\n";
        HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
        return false;
    }

    // Allocate memory for each row of nodes
    for (int i = 0; i < rows; i++) {
        nodes[i] = malloc(cols * sizeof(Node));
        if (nodes[i] == NULL) {
            mex = "Errore seconda dimensione\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
            return false;
        }
    }

    // Initialize each node's properties
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            nodes[i][j].row = i;
            nodes[i][j].column = j;

            double leftBottomX = minX + j * cellSize;
            double leftBottomY = minY + i * cellSize;

            Point leftBottom = {leftBottomX, leftBottomY};

            // Check if the cell's bottom-left corner is inside the polygon
            bool obstacleLeftBottom = isPointInPolygon(leftBottom);

            // Check if the cell's boundary intersects the polygon
            nodes[i][j].isBoundary = isCellOnSegment(i, j);

            // Mark the node as an obstacle if it's not inside the polygon
            nodes[i][j].isObstacle = !(obstacleLeftBottom);
            nodes[i][j].isValid = false;
        }
    }
    return true;
}

/**
 * @brief Finds the grid cell that contains a given point.
 *
 * This function checks whether the provided point is inside the grid's boundaries or on the segment. If valid, it calculates the row and column corresponding to the point's location in the grid.
 *
 * @param p The point to be located on the grid.
 * @param row Pointer to store the row index of the grid cell.
 * @param column Pointer to store the column index of the grid cell.
 *
 * @return true if the point is inside the boundaries and within a valid grid cell, false otherwise.
 */
bool findNode(Point p, int8_t* row, int8_t* column) {
    if (!isPointInPolygon(p) && !isPointOnSegment(p)) {
        return false; // Point is outside the grid boundaries
    }

    *column = (p.x - minX) / cellSize;
    *row = (p.y - minY) / cellSize;

    // If we are asking for a value on the border, set row to row-1
    if(*row == rows){
        *row = rows - 1;
    }
    // If we are asking for a value on the border, set column to column-1
    if(*column == cols){
        *column = cols - 1;
    }

    return true;
}

/**
 * @brief Finds the position of a point on a segment.
 *
 * This function calculates the intersection of a vertical or non-vertical segment with the left-bottom corner of the given grid cell. It determines whether the point lies on the segment and returns the corresponding coordinates.
 *
 * @param row The row index of the grid cell.
 * @param column The column index of the grid cell.
 * @param resultX Pointer to store the x-coordinate of the point on the segment.
 * @param resultY Pointer to store the y-coordinate of the point on the segment.
 *
 * @return true if the point is on a segment, false otherwise.
 */
bool findPositionOnSegment(int8_t row, int8_t column, double* resultX, double* resultY){
    double xLeft = minX + column * cellSize;
    double xRight = xLeft + cellSize;
    double yBottom = minY + row * cellSize;
    double yTop = yBottom + cellSize;

    for (int i = 0; i < boundary_count; i++) {
        Point p1 = boundaries[i];
        Point p2 = boundaries[(i + 1) % boundary_count];

        if (p1.x == p2.x) {
            // Case of vertical line
            if (xLeft <= p1.x && xRight >= p1.x && ((yBottom >= p1.y && yBottom <= p2.y) || (yBottom <= p1.y && yBottom >= p2.y))) {
                *resultY = yBottom;  // Use the lower height of the square
                *resultX = p1.x;     // x is fixed for vertical segments
                return true;
            }
            continue; // x is not part of the vertical segment
        }

        // Calculate the slope m
        double m = (p2.y - p1.y) / (p2.x - p1.x);

        // Calculate the intercept q
        double q = p1.y - m * p1.x;

        // Calculate y
        double y = m * xLeft + q;

        double x = xLeft;
        if (m != 0) {
            x = (yBottom - q) / m;
        }

        // Check if x is within the segment
        if (xLeft >= (p1.x < p2.x ? p1.x : p2.x) && xLeft <= (p1.x > p2.x ? p1.x : p2.x)) {
            if (yBottom >= (p1.y < p2.y ? p1.y : p2.y) && yBottom <= (p1.y > p2.y ? p1.y : p2.y)) {
                if (round(y) >= yBottom && round(y) <= yTop) {
                    *resultY = round(y);
                    *resultX = xLeft;
                    return true;
                } else if (round(x) >= xLeft && round(x) <= xRight) {
                    *resultY = yBottom;
                    *resultX = round(x);
                    return true;
                } else {
                    continue;
                }
            }
        } else {
            continue; // x is not valid for the segment
        }
    }
    return false;
}

/**
 * @brief Frees the memory allocated for the grid nodes.
 *
 * This function iterates through the grid and deallocates the memory used to store the nodes, ensuring no memory leaks occur.
 */
void freeNodes(void) {
	// free the memory
    for (int i = 0; i < rows; i++) {
        free(nodes[i]);
    }
    free(nodes);
}

/**
 * @brief Retrieves the neighboring nodes of a given node.
 *
 * This function finds the four neighboring nodes (up, down, left, and right) of a given node in the grid, if they exist. It stores the neighboring nodes in the provided array.
 *
 * @param currentNode Pointer to the current node.
 * @param neighbors Array of pointers to store the neighboring nodes.
 */
void getNeighbors(Node* currentNode, Node** neighbors) {
    int row = currentNode->row;
    int column = currentNode->column;
    if(row > 0){
        neighbors[0] = &nodes[row-1][column];  // bottom
    }else{
        neighbors[0] = NULL;
    }

    if(row < (rows - 1)){
        neighbors[1] = &nodes[row+1][column];  // up
    }else{
        neighbors[1] = NULL;
    }

    if(column > 0){
        neighbors[2] = &nodes[row][column-1];  // left
    }else{
        neighbors[2] = NULL;
    }

    if(column < (cols - 1)){
        neighbors[3] = &nodes[row][column+1];  // right
    }else{
        neighbors[3] = NULL;
    }
}

/**
 * @brief Initializes the priority queue.
 *
 * This function sets the size of the priority queue to zero, effectively initializing it.
 *
 * @param pq Pointer to the priority queue to be initialized.
 */
void initQueue(PriorityQueue* pq) {
    pq->size = 0;
}

/**
 * @brief Initializes the visited node list.
 *
 * This function sets the size of the visited node list to zero, effectively initializing it.
 *
 * @param vnl Pointer to the visited node list to be initialized.
 */
void initVisistedList(VisitedNodeList* vnl) {
    vnl->size = 0;
}

/**
 * @brief Clears the priority queue.
 *
 * This function resets all nodes in the priority queue to NULL and sets the size to zero.
 *
 * @param pq Pointer to the priority queue to be cleared.
 */
void clearQueue(PriorityQueue* pq) {
    for (int i = 0; i < pq->size; i++) {
        if (pq->nodes[i] != NULL) { // Check if the pointer is not already NULL
            pq->nodes[i] = NULL;   // Set the pointer to NULL
        }
    }
    pq->size = 0;  // Reset the size of the queue
}

/**
 * @brief Clears the visited node list.
 *
 * This function resets all nodes in the visited node list to NULL and sets the size to zero.
 *
 * @param vnl Pointer to the visited node list to be cleared.
 */
void clearVisitedList(VisitedNodeList* vnl) {
    for (int i = 0; i < vnl->size; i++) {
        if (vnl->nodes[i] != NULL) { // Check if the pointer is not already NULL
            vnl->nodes[i] = NULL;   // Set the pointer to NULL
        }
    }
    vnl->size = 0;  // Reset the size of the list
}

/**
 * @brief Initializes a node.
 *
 * This function initializes a node with the specified values and returns a pointer to the node.
 *
 * @param row The row index of the node.
 * @param column The column index of the node.
 * @param direction The direction of the node.
 * @param f_score The f_score of the node.
 * @param h_score The h_score of the node.
 * @param g_score The g_score of the node.
 * @param parent Pointer to the parent node.
 *
 * @return Pointer to the initialized node.
 */
Node* initNode(int row, int column, int8_t direction, double f_score, double h_score, double g_score, Node* parent) {
    nodes[row][column].row = row;
    nodes[row][column].column = column;
    nodes[row][column].direction = direction;
    nodes[row][column].f_score = f_score;
    nodes[row][column].h_score = h_score;
    nodes[row][column].g_score = g_score;
    nodes[row][column].parent = parent;

    return &nodes[row][column];
}

/**
 * @brief Adds a node to the priority queue.
 *
 * This function adds the specified node to the priority queue and increases the size of the queue.
 *
 * @param pq Pointer to the priority queue.
 * @param node Pointer to the node to be added.
 */
void enqueue(PriorityQueue* pq, Node* node) {
	// add the node to the queue
    pq->nodes[pq->size++] = node;
}

/**
 * @brief Marks a node as visited.
 *
 * This function adds the specified node to the visited node list and increments the list size.
 *
 * @param vnl Pointer to the visited node list.
 * @param node Pointer to the node to be marked as visited.
 */
void visitedNode(VisitedNodeList* vnl, Node* node) {
    vnl->nodes[vnl->size++] = node;
}

/**
 * @brief Checks if a node is in the priority queue.
 *
 * This function checks if the specified node is present in the priority queue.
 *
 * @param pq Pointer to the priority queue.
 * @param node Pointer to the node to be checked.
 *
 * @return true if the node is in the queue, false otherwise.
 */
bool findInQueue(PriorityQueue* pq, Node* node) {
    for (int i = 0; i < pq->size; i++) {
        if (pq->nodes[i] == node) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks if a node has been visited.
 *
 * This function checks if the specified node is present in the visited node list.
 *
 * @param vnl Pointer to the visited node list.
 * @param node Pointer to the node to be checked.
 *
 * @return true if the node has been visited, false otherwise.
 */
bool isVisited(VisitedNodeList* vnl, Node* node) {
    for (int i = 0; i < vnl->size; i++) {
        if (vnl->nodes[i] == node) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Removes and returns the node with the lowest f_score from the priority queue.
 *
 * This function finds the node with the lowest f_score in the queue. If there is a tie in f_score, it compares the h_score and direction.
 * The node is then removed from the queue and returned.
 *
 * @param pq Pointer to the priority queue.
 *
 * @return Pointer to the node with the lowest f_score.
 */
Node* dequeue(PriorityQueue* pq) {
    int minIndex = 0;
    for (int i = 1; i < pq->size; i++) {
        // Compare f_score
        if (pq->nodes[i]->f_score < pq->nodes[minIndex]->f_score) {
            minIndex = i;
        }
        // If f_scores are equal
        else if (pq->nodes[i]->f_score == pq->nodes[minIndex]->f_score) {
            // Compare h_score
            if (pq->nodes[i]->h_score < pq->nodes[minIndex]->h_score) {
                minIndex = i;
            }
            // If h_scores are also equal, compare direction with priority 3, 2, 1, 0
            else if (pq->nodes[i]->h_score == pq->nodes[minIndex]->h_score) {
                if (pq->nodes[i]->direction < pq->nodes[minIndex]->direction) {
                    minIndex = i;
                }
            }
        }
    }
    Node* minNode = pq->nodes[minIndex];
    // Replace the removed node with the last node and reduce the queue size
    pq->nodes[minIndex] = pq->nodes[--pq->size];
    return minNode;
}

/**
 * @brief Finds the shortest path from the start node to the goal node using the A* algorithm.
 *
 * This function implements the A* search algorithm to find the shortest path from the start node to the goal node. It maintains a priority queue to explore the nodes with the lowest f_score, where f_score is the sum of g_score and h_score. The algorithm terminates when it reaches the goal or when the open set is empty, indicating no path exists.
 *
 * @param startRow The row index of the starting node.
 * @param startColumn The column index of the starting node.
 * @param goalRow The row index of the goal node.
 * @param goalColumn The column index of the goal node.
 *
 * @return Pointer to the goal node if a path is found, NULL if no path exists.
 */
Node* findShortestPathAStar(int8_t startRow, int8_t startColumn, int8_t goalRow, int8_t goalColumn) {
    // Initialize priority queue and visited node list
    PriorityQueue openSet;
    VisitedNodeList visited;
    Node* currentNode = NULL;
    Node* neighbor = NULL;

    initQueue(&openSet);
    initVisistedList(&visited);

    // Initialize start and goal nodes
    Node startNode = nodes[startRow][startColumn];
    Node goalNode = nodes[goalRow][goalColumn];

    // Calculate heuristic value for the start node
    double value = heuristic(startNode, goalNode);
    Node* startNodePointer = initNode(startRow, startColumn, -1, value, value, 0, NULL);

    // Enqueue start node if it's not an obstacle
    if (!startNode.isObstacle) {
        enqueue(&openSet, startNodePointer);
    }

    // Main A* loop
    while (openSet.size > 0) {
        // Dequeue the node with the lowest f_score
        currentNode = dequeue(&openSet);
        visitedNode(&visited, currentNode);

        // Check if the goal has been reached
        if (currentNode->h_score == 0) {
            // Clear open set and visited list before returning the result
            clearQueue(&openSet);
            clearVisitedList(&visited);

            // Transmit success message over UART
            mex = "Percorso trovato\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
            return currentNode;
        }

        // Get neighbors of the current node
        getNeighbors(currentNode, neighbors);

        // Explore each neighbor
        for (int i = 0; i < 4; i++) {
            neighbor = neighbors[i];
            if (!neighbor) continue;  // Skip invalid neighbors
            if (neighbor->isObstacle) continue;  // Skip obstacles
            if (neighbor->isBoundary) continue;  // Skip boundaries

            // Calculate g_score, h_score, and f_score for the neighbor
            double tentative_g_score = heuristic(startNode, *neighbor);
            double tentative_h_score = heuristic(*neighbor, goalNode);
            double tentative_f_score = tentative_g_score + tentative_h_score;

            // Check if the neighbor is visited
            if (!isVisited(&visited, neighbor)) {
                if (!findInQueue(&openSet, neighbor)) {
                    // Initialize the neighbor and enqueue it
                    neighbor = initNode(neighbor->row, neighbor->column, i, tentative_f_score, tentative_h_score, tentative_h_score, currentNode);
                    enqueue(&openSet, neighbor);
                } else if (tentative_g_score < neighbor->g_score) {
                    // Update the neighbor with a better score
                    neighbor->direction = i;
                    neighbor->g_score = tentative_g_score;
                    neighbor->h_score = tentative_h_score;
                    neighbor->f_score = tentative_f_score;
                    neighbor->parent = currentNode;
                }
            }
        }
    }

    // No path found, transmit failure message over UART
    mex = "Percorso non trovato\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);

    // Clear open set and visited list before returning NULL
    clearQueue(&openSet);
    clearVisitedList(&visited);

    return NULL;
}

/**
 * @brief Finds the nearest wall (boundary) from the start node using the A* algorithm.
 *
 * This function uses a modified A* search to find the nearest wall (boundary) from the start node. It operates similarly to the previous A* implementation but aims to stop when the boundary is reached. The heuristic function used is adjusted to focus on reaching a boundary.
 *
 * @param startRow The row index of the starting node.
 * @param startColumn The column index of the starting node.
 *
 * @return Pointer to the node that is a boundary if found, NULL if no boundary is found.
 */
Node* findNearestWall(int8_t startRow, int8_t startColumn) {
    // Initialize priority queue and visited node list
    PriorityQueue openSet;
    VisitedNodeList visited;
    Node* currentNode = NULL;
    Node* neighbor = NULL;

    initQueue(&openSet);
    initVisistedList(&visited);

    // Initialize the start node
    Node startNode = nodes[startRow][startColumn];

    // Calculate heuristic value for the start node to the nearest boundary
    double value = heuristicToBoundary(startNode);
    Node* startNodePointer = initNode(startRow, startColumn, -1, value, value, 0, NULL);

    // Enqueue the start node
    enqueue(&openSet, startNodePointer);

    // Main loop to find the nearest wall
    while (openSet.size > 0) {
        // Dequeue the node with the lowest f_score
        currentNode = dequeue(&openSet);
        visitedNode(&visited, currentNode);

        // Check if a boundary has been reached
        if (currentNode->isBoundary) {
            // Clear open set and visited list before returning the result
            clearQueue(&openSet);
            clearVisitedList(&visited);

            // Transmit success message over UART
            mex = "Percorso verso il bordo trovato\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
            return currentNode;
        }

        // Get neighbors of the current node
        getNeighbors(currentNode, neighbors);

        // Explore each neighbor
        for (int i = 0; i < 4; i++) {
            neighbor = neighbors[i];
            if (!neighbor) continue;  // Skip invalid neighbors
            if (neighbor->isObstacle && !neighbor->isBoundary) continue;  // Skip obstacles unless they are boundaries

            // Calculate g_score, h_score, and f_score for the neighbor
            double tentative_g_score = heuristic(startNode, *neighbor);
            double tentative_h_score = heuristicToBoundary(*neighbor);
            double tentative_f_score = tentative_g_score + tentative_h_score;

            // Check if the neighbor is visited
            if (!isVisited(&visited, neighbor)) {
                if (!findInQueue(&openSet, neighbor)) {
                    // Initialize the neighbor and enqueue it
                    neighbor = initNode(neighbor->row, neighbor->column, i, tentative_f_score, tentative_h_score, tentative_h_score, currentNode);
                    enqueue(&openSet, neighbor);
                } else if (tentative_g_score < neighbor->g_score) {
                    // Update the neighbor with a better score
                    neighbor->direction = i;
                    neighbor->g_score = tentative_g_score;
                    neighbor->h_score = tentative_h_score;
                    neighbor->f_score = tentative_f_score;
                    neighbor->parent = currentNode;
                }
            }
        }
    }

    // No boundary found, transmit failure message over UART
    mex = "Percorso non trovato:\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);

    // Clear open set and visited list before returning NULL
    clearQueue(&openSet);
    clearVisitedList(&visited);

    return NULL;
}

/**
 * @brief Scans the grid to collect valid points based on obstacles and boundaries.
 *
 * This function scans the grid in a vertical manner to identify points that are not obstacles or boundaries. It checks neighboring cells and stores valid points in the result arrays, incrementing the count as it finds valid positions.
 *
 * @param resultRows Array that will hold the row indices of valid points.
 * @param resultColumns Array that will hold the column indices of valid points.
 * @param count Pointer to an integer that tracks the number of valid points found.
 */
void scanMatrix(int8_t* resultRows, int8_t* resultColumns, uint8_t* count) {
    *count = 0;

    // Simple vertical scan to collect valid points
    for (int j = 0; j < cols; j++) {
        for (int i = 0; i < rows; i++) {
            if (nodes[i][j].isObstacle || nodes[i][j].isBoundary) {
                if (i == 0) {
                    // If it's the first row, check the cell below
                    if (!(nodes[i+1][j].isObstacle || nodes[i+1][j].isBoundary)) {
                        resultRows[*count] = (int8_t) i+1;
                        resultColumns[*count] = (int8_t) j;
                        (*count)++;
                    }
                } else if (i == rows - 1) {
                    // If it's the last row, check the cell above
                    if (!(nodes[i-1][j].isObstacle || nodes[i-1][j].isBoundary)) {
                        resultRows[*count] = (int8_t) i-1;
                        resultColumns[*count] = (int8_t) j;
                        (*count)++;
                    }
                } else {
                    // For middle rows, check both above and below
                    if (!(nodes[i-1][j].isObstacle || nodes[i-1][j].isBoundary)) {
                        resultRows[*count] = (int8_t) i-1;
                        resultColumns[*count] = (int8_t) j;
                        (*count)++;
                    }
                    if (!(nodes[i+1][j].isObstacle || nodes[i+1][j].isBoundary)) {
                        resultRows[*count] = (int8_t) i+1;
                        resultColumns[*count] = (int8_t) j;
                        (*count)++;
                    }
                }
            }
            // If it's the last row, check for valid points
            if (i == rows - 1) {
                if (!(nodes[i][j].isObstacle || nodes[i][j].isBoundary)) {
                    resultRows[*count] = (int8_t) i;
                    resultColumns[*count] = (int8_t) j;
                    (*count)++;
                }
            }
        }
    }

    // Sort the results in snake-like pattern
    snakeSortResultsFromFirstValidColumn(resultRows, resultColumns, *count);
}

/**
 * @brief Sorts the scan results in a "snake" pattern, based on the first valid column.
 *
 * This function reorders the collected points in a snake-like fashion, first finding the column with the first valid point and then sorting each column either upward or downward based on its parity.
 *
 * @param resultRows Array holding the row indices of valid points.
 * @param resultColumns Array holding the column indices of valid points.
 * @param count The number of valid points to sort.
 */
void snakeSortResultsFromFirstValidColumn(int8_t* resultRows, int8_t* resultColumns, uint8_t count) {
    // Temporary arrays for sorted results
    int8_t tempRows[count];
    int8_t tempColumns[count];

    // Counter for building the new order
    int index = 0;

    // Find the first column with at least one valid point
    int firstValidColumn = -1;
    for (int j = 0; j < cols; j++) {
        for (int i = 0; i < count; i++) {
            if (resultColumns[i] == j) {
                firstValidColumn = j;
                break;
            }
        }
        if (firstValidColumn != -1) {
            break;
        }
    }

    // Process column by column starting from the first valid column
    for (int col = firstValidColumn; col < cols; col++) {
        // Find all cells belonging to the current column
        int colStart = index;
        for (int i = 0; i < count; i++) {
            if (resultColumns[i] == col) {
                tempRows[index] = (int8_t) resultRows[i];
                tempColumns[index] = (int8_t) resultColumns[i];
                index++;
            }
        }
        int colEnd = index;

        // If the column is odd (compared to the first valid column), reverse the order of rows
        if ((col - firstValidColumn) % 2 != 0) {
            for (int i = 0; i < (colEnd - colStart) / 2; i++) {
                int8_t tempRow = (int8_t) tempRows[colStart + i];
                tempRows[colStart + i] = (int8_t) tempRows[colEnd - 1 - i];
                tempRows[colEnd - 1 - i] = (int8_t) tempRow;

                int8_t tempCol = (int8_t) tempColumns[colStart + i];
                tempColumns[colStart + i] = (int8_t) tempColumns[colEnd - 1 - i];
                tempColumns[colEnd - 1 - i] = (int8_t) tempCol;
            }
        }
    }

    // Copy the sorted results back into the original arrays
    for (int i = 0; i < count; i++) {
        resultRows[i] = (int8_t) tempRows[i];
        resultColumns[i] = (int8_t) tempColumns[i];
    }
}

/**
 * @brief Prints the coordinates of the points.
 *
 * This function iterates over the arrays `resultRows` and `resultColumns` to format and print each point's coordinates. It transmits the formatted string via UART for display or further processing.
 *
 * @param resultRows Array containing the row indices of the points to print.
 * @param resultColumns Array containing the column indices of the points to print.
 * @param count The number of points to print.
 */
void printMatrix(int8_t* resultRows, int8_t* resultColumns, uint8_t count) {
    for (int i = 0; i < count; i++) {
        int len = snprintf(buff, sizeof(buff), "(%d, %d)\n", resultColumns[i], resultRows[i]);
        // Transmit the string by UART
        HAL_UART_Transmit(&huart6, (uint8_t*)buff, len, HAL_MAX_DELAY);
    }
}

/**
 * @brief Calculates the Euclidean distance between two points.
 *
 * This function computes the straight-line distance between two points `a` and `b` using the Pythagorean theorem.
 *
 * @param a The first point.
 * @param b The second point.
 *
 * @return The Euclidean distance between the two points.
 */
double distance(Point a, Point b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief Determines the shortest direction between two points.
 *
 * This function identifies the two consecutive vertices (`p1`, `p2`) from the polygon boundary that enclose the target point. It then calculates the clockwise and counterclockwise distances between the vertices and selects the shorter path.
 *
 * @param target The target point for which the direction is being determined.
 * @param clockwise Pointer to a boolean that will be set to `true` if the shortest path is clockwise, `false` if counterclockwise.
 */
void shortestDirection(Point target, bool *clockwise) {
    int p1_index = -1, p2_index = -1;

    // Find the two consecutive vertices (p1, p2) between which the target lies
    for (int i = 0; i < boundary_count; i++) {
        Point p1 = boundaries[i];
        Point p2 = boundaries[(i + 1) % boundary_count];

        if(isPointBetweenTwoPoints(target, p1, p2)){
            p1_index = i;
            p2_index = (i + 1) % boundary_count;
            break;
        }
    }

    // Calculate the total distances in both directions
    double clockwise_distance = 0;
    double counterclockwise_distance = 0;

    // Calculate clockwise distance
    for (int i = 0; i < boundary_count; i++){
        if(i == p1_index || i == p2_index){
            break;
        }
        clockwise_distance++;
    }

    // Calculate counterclockwise distance
    for (int i = boundary_count - 2; i > 0; i--){
        if(i == p1_index || i == p2_index){
            break;
        }
        counterclockwise_distance++;
    }

    // Determine the shortest path
    *clockwise = clockwise_distance <= counterclockwise_distance;
}
