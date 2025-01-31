/*
 * mapOperations.h
 *
 *  Created on: Oct 18, 2024
 *      Author: daniele-boerio
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#ifndef SRC_MAPOPERATIONS_H_
#define SRC_MAPOPERATIONS_H_

#define MAX_POINTS 100 						// max number of vertices in the polygon
#define MAX_BOUNDARIES 961 					// max number of cell in the polygon


#define DIAMETRO_RUOTA 68					//68 mm
#define DISTANZA_RUOTE 320.635				//320 mm
#define PASSI_PER_GIRO_RUOTA 3200			// number of steps to do a full rotation of each weel


typedef struct {
    double x;
    double y;
} Point;


// a node represent a cell in the polygon
typedef struct Node {
	int8_t row;
	int8_t column;
	bool isBoundary;
	bool isObstacle;     	// 1 if the cell is an obstacle, 0 otherwise
	bool isValid;     		// 1 if the cell is valid, 0 otherwise
	int8_t direction;
    double f_score;
    double h_score;
    double g_score;
    struct Node* parent;
} Node;

// priority queue
typedef struct {
    Node* nodes[MAX_BOUNDARIES];
    int size;
} PriorityQueue;

// visited list
typedef struct {
    Node* nodes[MAX_BOUNDARIES];
    int size;
} VisitedNodeList;

extern Point boundaries[MAX_POINTS];
extern uint8_t boundary_count;
extern UART_HandleTypeDef huart6;
extern Point current_position;
extern double current_angle;
extern double cellSize;
extern long minY;
extern long minX;
extern Node** nodes;
extern uint8_t rows;
extern uint8_t cols;

long calcolaNumeroPassiRotazione(double);
double calcolaAngoloRotazione(unsigned long);
void calculate_steps_and_angle(Point, unsigned long*, double*);

double heuristic(Node, Node);
double heuristicToBoundary(Node);

bool isPointInPolygon(Point);
bool isPointOnSegment(Point);
bool isPointBetweenTwoPoints(Point, Point, Point);
bool isCellOnSegment(int8_t, int8_t);
bool isSegmentInsideSquare(Point, Point, Point);
void computeGridDimensions(double*, uint8_t*, uint8_t*);
bool createNodes(void);

bool findNode(Point, int8_t*, int8_t*);
bool findPositionOnSegment(int8_t, int8_t, double*, double*);
void freeNodes(void);
void getNeighbors(Node*, Node**);
void initQueue(PriorityQueue*);
void initVisistedList(VisitedNodeList*);
void clearQueue();
void clearVisitedList();

Node* initNode(int, int, int8_t, double, double, double, Node*);
void enqueue(PriorityQueue*, Node*);
void visitedNode(VisitedNodeList*, Node*);
bool findInQueue(PriorityQueue*, Node*);
bool isVisited(VisitedNodeList*, Node*);
Node* dequeue(PriorityQueue*);
Node* findShortestPathAStar(int8_t, int8_t, int8_t, int8_t);
Node* findNearestWall(int8_t, int8_t);
void scanMatrix(int8_t*, int8_t*, uint8_t*);
void snakeSortResultsFromFirstValidColumn(int8_t*, int8_t*, uint8_t);
void printMatrix(int8_t*, int8_t*, uint8_t);
double distance(Point, Point);
void shortestDirection(Point, bool*);

#endif /* SRC_MAPOPERATIONS_H_ */
