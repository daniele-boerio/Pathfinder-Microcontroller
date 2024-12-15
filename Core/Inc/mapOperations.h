/*
 * mapOperations.h
 *
 *  Created on: Oct 18, 2024
 *      Author: dboer
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#ifndef SRC_MAPOPERATIONS_H_
#define SRC_MAPOPERATIONS_H_

#define MAX_POINTS 100 						//massimo numero di vertici del poligono
#define MAX_BOUNDARIES 961 					//massimo numero di celle per dividere il poligono che il sistema può sostenere

//Constants for save and load from flash memory
#define FLASH_USER_START_ADDR   0x08040000  // Indirizzo del settore 6 della Flash
#define START_CONTROL_VALUE -1000			// Valore iniziale per lo storage dei punti in memoria
#define END_CONTROL_VALUE 1000				// Valore finale per lo storage dei punti in memoria


#define DIAMETRO_RUOTA 68					//68 mm
#define DISTANZA_RUOTE 320.635				//320 mm circa
#define PASSI_PER_GIRO_RUOTA 3200			//numero di passi per fare un giro intero della ruota


#define EPSILON 1e-9 // Tolleranza per confronti


typedef struct {
    double x;
    double y;
} Point;


// Nodo per rappresentare ogni punto esplorato
typedef struct Node {
	int8_t row;
	int8_t column;
	bool isBoundary;
	bool isObstacle;     // 1 se la cella è un ostacolo, 0 altrimenti
	bool isValid;     // 1 se la cella è valida, 0 altrimenti
	int8_t direction;
    double f_score;
    double h_score;
    double g_score;
    struct Node* parent;
} Node;

// Implementazione della coda di priorità semplice (per esempio, una lista non ordinata da migliorare con un heap)
typedef struct {
    Node* nodes[MAX_BOUNDARIES];
    int size;
} PriorityQueue;

typedef struct {
    Node* nodes[MAX_BOUNDARIES];
    int size;
} VisitedNodeList;

extern Point boundaries[MAX_POINTS];
extern uint8_t boundary_count;
extern UART_HandleTypeDef huart6;  // Dichiara huart6 come esterna
extern Point current_position;  // Dichiarazione della posizione corrente
extern double current_angle;    // Angolo attuale della macchina (in radianti)
extern double cellSize;
extern long minY;
extern long minX;
extern Node** nodes;
extern uint8_t rows;
extern uint8_t cols;


void save_polygon_to_flash(void);
void erase_polygon(void);
bool load_polygon_from_flash(void);
void add_boundary_point(long, long);
void transmit_coordinates(long, long);

void add_forward(unsigned long);
void add_backward(unsigned long);
void add_right(double);
void add_left(double);

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
