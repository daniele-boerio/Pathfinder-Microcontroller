/*
 * mapOperations.c
 *
 *  Created on: Oct 18, 2024
 *      Author: dboer
 */

#include "mapOperations.h"
#include "stm32f4xx_hal.h"
#include <math.h>   // Per sqrt e atan2
#include <stdbool.h>

Point boundaries[MAX_POINTS];  // Array di punti del poligono
uint8_t boundary_count = 0;  // Numero di punti del poligono

Node* neighbors[4] = {NULL};
Node** nodes;
uint8_t rows;
uint8_t cols;
double cellSize;

long minX, maxX;
long minY, maxY;

char buff[50];
char* mex = "";

// Funzione per sbloccare la memoria Flash, cancellare un settore e scrivere i dati
void save_polygon_to_flash(void) {
    uint32_t flash_address = FLASH_USER_START_ADDR;

    // Sblocca l'accesso alla Flash
    HAL_FLASH_Unlock();

    // Cancella il settore della Flash (settore 6 in questo esempio)
    FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);

    for (int i = 0; i < 4; i++){
    	// Scrivi i valori di controllo iniziale
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long)START_CONTROL_VALUE);
    	flash_address += sizeof(long);
    }

    // Scrivi i punti del poligono nella Flash
    for (int i = 0; i < boundary_count; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long) boundaries[i].x);
        flash_address += sizeof(long);  // Ogni long occupa 4 byte
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long) boundaries[i].y);
        flash_address += sizeof(long); // Ogni long occupa 4 byte
    }

    for (int i = 0; i < 4; i++){
    	// Scrivi i valori di controllo finale
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long)END_CONTROL_VALUE);
    	flash_address += sizeof(long);
    }

    // Blocca di nuovo la Flash per prevenire scritture accidentali
    HAL_FLASH_Lock();
}

void erase_polygon(void) {

    // Sblocca l'accesso alla Flash
    HAL_FLASH_Unlock();

    // Cancella il settore della Flash (settore 6 in questo esempio)
    FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);

    // Blocca di nuovo la Flash per prevenire scritture accidentali
    HAL_FLASH_Lock();

    // Cancella i punti del poligono dalla RAM con memset
    boundary_count = 0;
    memset(boundaries, 0, sizeof(boundaries));  // Imposta tutti i byte di boundaries a 0
}

// Funzione per caricare il poligono dalla memoria Flash
bool load_polygon_from_flash(void) {
    uint32_t flash_address = FLASH_USER_START_ADDR;

    // Leggi il valore di controllo iniziale
    for (int i = 0; i < 4; i++){
        long start_value = *(uint32_t*)flash_address;
        if (start_value != START_CONTROL_VALUE) {
            return false;  // Il valore di controllo iniziale non corrisponde
        }
        flash_address += sizeof(long);  // Avanza all'indirizzo successivo
    }

    // Leggi i punti dalla Flash
    uint16_t count = 0;
    while (1) {
        // Verifica il valore di controllo finale prima di salvare i punti
    	long end_value_1, end_value_2, end_value_3, end_value_4;
    	end_value_1 = *(long*)(flash_address);
    	end_value_2 = *(long*)(flash_address + sizeof(long));
    	end_value_3 = *(long*)(flash_address + 2 * sizeof(long));
    	end_value_4 = *(long*)(flash_address + 3 * sizeof(long));

        if ((end_value_1 == END_CONTROL_VALUE) && (end_value_2 == END_CONTROL_VALUE) && (end_value_3 == END_CONTROL_VALUE) && (end_value_4 == END_CONTROL_VALUE)) {
            break;  // Se incontriamo i valori di controllo finale, fermiamo la lettura
        }

        long x = *(long*)flash_address;
        flash_address += sizeof(long);
        long y = *(long*)flash_address;
        flash_address += sizeof(long);


        // Salva il punto letto nell'array
        boundaries[count].x = (double) x;
        boundaries[count].y = (double) y;
        count++;

        // Limita il numero massimo di punti che puoi leggere
        if (count >= MAX_POINTS) {
            break;
        }
    }
    for (int i = 0; i < boundary_count; i++) {
    	// Formatto i valori x e y del punto corrente come stringa
    	int len = snprintf(buff, sizeof(buff), "Punto %d: x = %ld, y = %ld\n", i, (long) boundaries[i].x, (long) boundaries[i].y);

    	// Trasmetto la stringa via UART
    	HAL_UART_Transmit(&huart6, (uint8_t*)buff, len, HAL_MAX_DELAY);
    }

    boundary_count = count;  // Restituisci il numero di punti letti
    return true;  // I dati sono stati letti correttamente
}

// Funzione per aggiungere un punto al poligono
void add_boundary_point(long x, long y) {
    if (boundary_count < MAX_POINTS) {
        boundaries[boundary_count].x = (double) x;
        boundaries[boundary_count].y = (double) y;
        boundary_count++;

        // Trasmetti le coordinate via UART
        transmit_coordinates(x, y);
    }
}

// Funzione per inviare le coordinate via UART
void transmit_coordinates(long x, long y) {

    // Usa angle e power come necessario
    snprintf(buff, sizeof(buff), "(%ld, %ld)\n", x, y);
    HAL_UART_Transmit(&huart6, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}

// Funzione per aggiungere un punto dopo essere andato in avanti di "steps" passi
void add_forward(unsigned long steps) {
    // Calcola il movimento in base all'angolo corrente
	double delta_x = steps * cos(current_angle);
	double delta_y = steps * sin(current_angle);

    // Aggiorna la posizione
    current_position.x += delta_x;
    current_position.y += delta_y;

    // Aggiungi la nuova posizione ai confini del poligono
    add_boundary_point(current_position.x, current_position.y);
}

// Funzione per aggiungere un punto dopo essere andato indietro di "steps" passi
void add_backward(unsigned long steps) {
    // Calcola il movimento in base all'angolo corrente
	double delta_x = steps * cos(current_angle);
	double delta_y = steps * sin(current_angle);

    // Aggiorna la posizione (muovendo indietro)
    current_position.x -= delta_x;
    current_position.y -= delta_y;

    // Aggiungi la nuova posizione ai confini del poligono
    add_boundary_point(current_position.x, current_position.y);
}

// Funzione per modificare l'angolazione corrente dopo essere andato a destra di "radiants" radianti
void add_right(double radiants) {
	current_angle -= radiants;
}

// Funzione per modificare l'angolazione corrente dopo essere andato a sinistra di "radiants" radianti
void add_left(double radiants) {
	current_angle += radiants;
}

long calcolaNumeroPassiRotazione(double radiants) {
    // Formula per calcolare il numero di passi dall'angolo in gradi
	//return (unsigned long)((radiants * (float)(DISTANZA_RUOTE)/2.0 * M_PI * (float)(PASSI_PER_GIRO_RUOTA))/((float)(DIAMETRO_RUOTA) * M_PI * 180.0));

	// Formula per calcolare il numero di passi dall'angolo in radianti
	return (long)((radiants * DISTANZA_RUOTE/2  * PASSI_PER_GIRO_RUOTA)/(DIAMETRO_RUOTA * M_PI));
}

double calcolaAngoloRotazione(unsigned long n_passi) {
	// Formula per calcolare l'angolo in gradi
	//return (float) ((n_passi * ((float)(DIAMETRO_RUOTA) * M_PI) * 180.0) / ((float)(PASSI_PER_GIRO_RUOTA) * (float)(DISTANZA_RUOTE)/2.0 * M_PI));

    // Formula per calcolare l'angolo in radianti
	return (double) ((n_passi * (DIAMETRO_RUOTA * M_PI)) / (PASSI_PER_GIRO_RUOTA * DISTANZA_RUOTE/2));
}

// Funzione per calcolare la distanza e l'angolazione rispetto a un punto target
void calculate_steps_and_angle(Point target_point, unsigned long *steps, double *rotation_angle) {
    // Calcola la differenza in x e y
	long delta_x = (long) target_point.x - current_position.x;
	long delta_y = (long) target_point.y - current_position.y;

    // Calcola la distanza euclidea (numero di passi da fare)
    *steps = (unsigned long) sqrt((double)delta_x * delta_x + (double)delta_y * delta_y);

    // Calcola l'angolo tra la posizione corrente e il punto di destinazione
    double target_angle = (double) atan2(delta_y, delta_x);  // Angolo in radianti

    // Calcola la rotazione necessaria (differenza tra l'angolo target e l'angolo attuale)
    *rotation_angle = target_angle - current_angle;


    if(*rotation_angle > M_PI){
    	*rotation_angle -= 2 * M_PI;
    }else if (*rotation_angle < -M_PI){
    	*rotation_angle += 2 * M_PI;
    }
}

// Funzione euristica: distanza Euclidea
double heuristic(Node a, Node b) {
	return fabs(a.row - b.row) + fabs(a.column - b.column);
}

double heuristicToBoundary(Node node) {
    int minRowDist = fmin(node.row, rows - 1 - node.row); // Distanza al bordo superiore o inferiore
    int minColDist = fmin(node.column, cols - 1 - node.column); // Distanza al bordo sinistro o destro
    return (double) (minRowDist + minColDist); // Distanza di Manhattan al bordo più vicino
}

bool isPointInPolygon(Point p) {
	int count = 0;
	for (int i = 0; i < boundary_count; i++) {
		Point v1 = boundaries[i];
		Point v2 = boundaries[(i + 1) % boundary_count];

		// Controlla se il raggio interseca il lato
		if ((v1.y > p.y) != (v2.y > p.y)) {
			double x_intersection = v1.x + (double)(p.y - v1.y) * (v2.x - v1.x) / (v2.y - v1.y);
			if (x_intersection > p.x) {
				count++;
			}
		}
	}

	// Restituisce true se il numero di intersezioni è dispari
	return count % 2 == 1;
}

// Funzione per verificare la collinearità e l'appartenenza al segmento
bool isPointOnSegment(Point p) {
	for (int i = 0; i < boundary_count; i++) {
		Point p1 = boundaries[i];
		Point p2 = boundaries[(i + 1) % boundary_count];

		if (p1.x == p2.x) {
		        // Caso di retta verticale
		        if (p.x == p1.x) {
		            if((p.y >= p1.y && p.y <= p2.y)||(p.y <= p1.y && p.y >= p2.y)){
		            	return true;
		            }else{
		            	continue;
		            }
		        } else {
		            continue; // x non appartiene al segmento verticale
		        }
		    }

		    // Calcola il coefficiente angolare m
		    double m = (p2.y - p1.y) / (p2.x - p1.x);

		    // Calcola l'intercetta q
		    double q = p1.y - m * p1.x;

		    // Calcola y
		    double y = m * p.x + q;

		    // Verifica se x appartiene al segmento
		    if (p.x >= (p1.x < p2.x ? p1.x : p2.x) && p.x <= (p1.x > p2.x ? p1.x : p2.x)) {
		    	if(round(y) == p.y ){
		    		return true;
		    	}else{
		    		continue;
		    	}
		    } else {
		        continue; // x non è valido per il segmento
		    }
	}
	return false;
}

bool isPointBetweenTwoPoints(Point p, Point p1, Point p2){
	if (p.x >= (p1.x < p2.x ? p1.x : p2.x) && p.x <= (p1.x > p2.x ? p1.x : p2.x)) {
		if(p.y >= (p1.y < p2.y ? p1.y : p2.y) && p.y <= (p1.y > p2.y ? p1.y : p2.y)){
			return true;
		}}
    return false;
}

bool isCellOnSegment(int8_t row, int8_t column) {

	double xLeft = minX + column * cellSize;
	double yBottom = minY + row * cellSize;
	for (int i = 0; i < boundary_count; i++) {
		Point p1 = boundaries[i];
		Point p2 = boundaries[(i + 1) % boundary_count];
		Point target = {xLeft, yBottom};

		if(isSegmentInsideSquare(p1, p2, target)){
			return true;
		}
	}
	return false;
}

bool isSegmentInsideSquare(Point p1, Point p2, Point target) {
    // Calcolo della tolleranza per confronti di valori floating-point
    const double epsilon = 1e-6;

    // Condizione 1: Verifica che il punto sia sulla retta
    double crossProduct = (target.y - p1.y) * (p2.x - p1.x) - (target.x - p1.x) * (p2.y - p1.y);
    if (fabs(crossProduct) > epsilon) {
        return false;  // Non è sulla retta
    }

    // Condizione 2: Verifica che il punto sia tra i due estremi del segmento
    double minX = fmin(p1.x, p2.x), maxX = fmax(p1.x, p2.x);
    double minY = fmin(p1.y, p2.y), maxY = fmax(p1.y, p2.y);

    if (target.x < minX || target.x > maxX || target.y < minY || target.y > maxY) {
        return false;  // È fuori dal segmento
    }

    return true;  // Il punto appartiene al segmento
}

void computeGridDimensions(double *cellSize, uint8_t *rows, uint8_t *cols) {
    long width = maxX - minX;
    long height = maxY - minY;

    // Imposta una dimensione iniziale per le celle, garantendo che sia >= 3500
    *cellSize = 3500;

    // Calcola il numero iniziale di colonne e righe basato su cellSize
    *cols = (int)ceil((double)width / *cellSize);
    *rows = (int)ceil((double)height / *cellSize);

    // Controlla se il numero totale di celle eccede MAX_BOUNDARIES
    while ((*rows) * (*cols) > MAX_BOUNDARIES) {
        // Incrementa la dimensione delle celle per ridurre il numero totale di celle
        *cellSize += 100; // Aumenta gradualmente la dimensione delle celle
        *cols = (int)ceil((double)width / *cellSize);
        *rows = (int)ceil((double)height / *cellSize);
    }

    // A questo punto, cellSize >= 3500 e il numero di celle è <= MAX_BOUNDARIES
}

bool createNodes(void){
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

	computeGridDimensions(&cellSize, &rows, &cols);

	nodes = malloc(rows * sizeof(Node *));
	if (nodes == NULL) {
		mex = "Errore prima dimensione\n";
		HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
		return false;
	}

	for (int i = 0; i < rows; i++) {
		nodes[i] = malloc(cols * sizeof(Node));
		if (nodes[i] == NULL) {
			mex = "Errore seconda dimensione\n";
			HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
			return false;
		}
	}

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			nodes[i][j].row = i;
			nodes[i][j].column = j;

			double leftBottomX = minX + j * cellSize;
			double leftBottomY = minY + i * cellSize;

			Point leftBottom = {leftBottomX, leftBottomY};

			bool obstacleLeftBottom = isPointInPolygon(leftBottom);

			nodes[i][j].isBoundary = isCellOnSegment(i, j);


			nodes[i][j].isObstacle = !(obstacleLeftBottom);
			nodes[i][j].isValid = false;
		}
	}
	return true;
}

// Funzione per trovare la cella in cui si trova il punto
bool findNode(Point p, int8_t* row, int8_t* column) {
    if (!isPointInPolygon(p) && !isPointOnSegment(p)) {
        return false; // Punto fuori dai limiti della griglia
    }

    *column = (p.x - minX) / cellSize;
    *row = (p.y - minY) / cellSize;

    // stiamo chiedendo il valore sul bordo quindi setto a row-1
    if(*row == rows){
    	*row = rows-1;
    }
    //stiamo chiedendo il valore sul bordo quindi setto a column-1
    if(*column == cols){
    	*column = cols-1;
    }

    return true;
}

bool findPositionOnSegment(int8_t row, int8_t column, double* resultX, double* resultY){
	double xLeft = minX + column * cellSize;
	double xRight = xLeft + cellSize;
	double yBottom = minY + row * cellSize;
	double yTop = yBottom + cellSize;
	for (int i = 0; i < boundary_count; i++) {
		Point p1 = boundaries[i];
		Point p2 = boundaries[(i + 1) % boundary_count];


		if (p1.x == p2.x) {
			// Caso di retta verticale
			if (xLeft <= p1.x && xRight >= p1.x && ((yBottom >= p1.y && yBottom <= p2.y)||(yBottom <= p1.y && yBottom >= p2.y))) {
			    *resultY = yBottom;  // Usa l'altezza inferiore del quadrato
			    *resultX = p1.x;     // La x è fissa per segmenti verticali
			    return true;
			}
			continue; // x non appartiene al segmento verticale
		}

		// Calcola il coefficiente angolare m
		double m = (p2.y - p1.y) / (p2.x - p1.x);

		// Calcola l'intercetta q
		double q = p1.y - m * p1.x;

		// Calcola y
		double y = m * xLeft + q;

		double x = xLeft;
		if( m != 0){
			x = (yBottom - q) / m;
		}


		// Verifica se x appartiene al segmento
		if (xLeft >= (p1.x < p2.x ? p1.x : p2.x) && xLeft <= (p1.x > p2.x ? p1.x : p2.x)) {
			if(yBottom >= (p1.y < p2.y ? p1.y : p2.y) && yBottom <= (p1.y > p2.y ? p1.y : p2.y)){
				if (round(y) >= yBottom && round(y) <= yTop){
					*resultY = round(y);
					*resultX = xLeft;
					return true;
				}else if(round(x) >= xLeft && round(x) <= xRight){
					*resultY = yBottom;
					*resultX = round(x);
					return true;
				}else{
					continue;
				}
			}
		} else {
			continue; // x non è valido per il segmento
		}
	}
	return false;
}

// Libera la memoria di tutti i nodi
void freeNodes(void) {
	// Libera la memoria
	    for (int i = 0; i < rows; i++) {
	        free(nodes[i]);
	    }
	    free(nodes);
}

// Funzione per ottenere i vicini di un nodo
void getNeighbors(Node* currentNode, Node** neighbors) {
	int row = currentNode->row;
	int column = currentNode->column;
	if(row > 0){
		neighbors[0] = &nodes[row-1][column];  // sotto
	}else{
		neighbors[0] = NULL;
	}

	if(row < (rows - 1)){
		neighbors[1] = &nodes[row+1][column];  // sopra
	}else{
		neighbors[1] = NULL;
	}

	if(column > 0){
		neighbors[2] = &nodes[row][column-1];  // sinistra
	}else{
		neighbors[2] = NULL;
	}

	if(column < (cols - 1)){
		neighbors[3] = &nodes[row][column+1];  // destra
	}else{
		neighbors[3] = NULL;
	}
}

void initQueue(PriorityQueue* pq) {
    pq->size = 0;
}

void initVisistedList(VisitedNodeList* vnl) {
	vnl->size = 0;
}

void clearQueue(PriorityQueue* pq){
	for (int i = 0; i < pq->size; i++) {
	    if (pq->nodes[i] != NULL) { // Controlla che il puntatore non sia già NULL
	    	pq->nodes[i] = NULL;   // Imposta il puntatore a NULL
	    }
	}
	pq->size = 0;
}

void clearVisitedList(VisitedNodeList* vnl) {
	for (int i = 0; i < vnl->size; i++) {
	    if (vnl->nodes[i] != NULL) { // Controlla che il puntatore non sia già NULL
	    	vnl->nodes[i] = NULL;   // Imposta il puntatore a NULL
	    }
	}
	vnl->size = 0;
}

Node* initNode(int row, int column, int8_t direction, double f_score, double h_score, double g_score, Node* parent){
	nodes[row][column].row = row;
	nodes[row][column].column = column;
	nodes[row][column].direction = direction;
	nodes[row][column].f_score = f_score;
	nodes[row][column].h_score = h_score;
	nodes[row][column].g_score = g_score;
	nodes[row][column].parent = parent;

	return &nodes[row][column];
}

void enqueue(PriorityQueue* pq, Node* node) {
    // Aggiungi il nodo alla coda
    pq->nodes[pq->size++] = node;
}

void visitedNode(VisitedNodeList* vnl, Node* node) {
	vnl->nodes[vnl->size++] = node;
}

bool findInQueue(PriorityQueue* pq, Node* node) {
    for (int i = 0; i < pq->size; i++) {
        if (pq->nodes[i] == node) {
            return true;
        }
    }
    return false;
}

bool isVisited(VisitedNodeList* vnl, Node* node) {
    for (int i = 0; i < vnl->size; i++) {
        if (vnl->nodes[i] == node) {
            return true;
        }
    }
    return false;
}

Node* dequeue(PriorityQueue* pq) {
    int minIndex = 0;
    for (int i = 1; i < pq->size; i++) {
        // Confronto del f_score
        if (pq->nodes[i]->f_score < pq->nodes[minIndex]->f_score) {
            minIndex = i;
        }
        // Se i f_score sono uguali
        else if (pq->nodes[i]->f_score == pq->nodes[minIndex]->f_score) {
            // Confronto h_score
            if (pq->nodes[i]->h_score < pq->nodes[minIndex]->h_score) {
                minIndex = i;
            }
            // Se anche gli h_score sono uguali, confronta la direzione con priorità 3, 2, 1, 0
            else if (pq->nodes[i]->h_score == pq->nodes[minIndex]->h_score) {
                if (pq->nodes[i]->direction < pq->nodes[minIndex]->direction) {
                    minIndex = i;
                }
            }
        }
    }
    Node* minNode = pq->nodes[minIndex];
    // Sostituisci il nodo rimosso con l'ultimo nodo e riduci la dimensione della coda
    pq->nodes[minIndex] = pq->nodes[--pq->size];
    return minNode;
}

// Funzione principale A*
Node* findShortestPathAStar(int8_t startRow, int8_t startColumn, int8_t goalRow, int8_t goalColumn) {
	PriorityQueue openSet;
	VisitedNodeList visited;

    Node* currentNode = NULL;
    Node* neighbor = NULL;

    initQueue(&openSet);
    initVisistedList(&visited);

    Node startNode = nodes[startRow][startColumn];
    Node goalNode = nodes[goalRow][goalColumn];

    double value = heuristic(startNode, goalNode);
    Node* startNodePointer = initNode(startRow, startColumn, -1, value, value, 0, NULL);

    if(!startNode.isObstacle){
    	enqueue(&openSet, startNodePointer);
    }

    while (openSet.size > 0) {
        currentNode = dequeue(&openSet);
        visitedNode(&visited, currentNode);

        // Verifica se abbiamo raggiunto il goal
        if (currentNode->h_score == 0) {
            clearQueue(&openSet);
            clearVisitedList(&visited);

            mex = "Percorso trovato\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
            return currentNode;
        }

        getNeighbors(currentNode, neighbors);

        for (int i = 0; i < 4; i++) {
            neighbor = neighbors[i];
            if(!neighbor) continue;
            if(neighbor->isObstacle) continue;
            if(neighbor->isBoundary) continue;

            double tentative_g_score = heuristic(startNode, *neighbor);
            double tentative_h_score = heuristic(*neighbor, goalNode);
            double tentative_f_score = tentative_g_score + tentative_h_score;
            if(!isVisited(&visited, neighbor)){
                if (!findInQueue(&openSet, neighbor)) {
                	neighbor = initNode(neighbor->row, neighbor->column, i, tentative_f_score, tentative_h_score, tentative_h_score, currentNode);
                	enqueue(&openSet, neighbor);
                } else if (tentative_g_score < neighbor->g_score) {
                	neighbor->direction = i;
                	neighbor->g_score = tentative_g_score;
                	neighbor->h_score = tentative_h_score;
                	neighbor->f_score = tentative_f_score;
                	neighbor->parent = currentNode;
                }
            }
        }
    }

    mex = "Percorso non trovato\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);

    clearQueue(&openSet);
    clearVisitedList(&visited);

    return NULL;
}

// Funzione principale A*
Node* findNearestWall(int8_t startRow, int8_t startColumn) {
	PriorityQueue openSet;
	VisitedNodeList visited;

    Node* currentNode = NULL;
    Node* neighbor = NULL;

    initQueue(&openSet);
    initVisistedList(&visited);

    Node startNode = nodes[startRow][startColumn];

    double value = heuristicToBoundary(startNode);
    Node* startNodePointer = initNode(startRow, startColumn, -1, value, value, 0, NULL);

    enqueue(&openSet, startNodePointer);

    while (openSet.size > 0) {
        currentNode = dequeue(&openSet);
        visitedNode(&visited, currentNode);

        // Verifica se abbiamo raggiunto il goal
        if (currentNode->isBoundary) {
            clearQueue(&openSet);
            clearVisitedList(&visited);

            mex = "Percorso verso il bordo trovato\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
            return currentNode;
        }

        getNeighbors(currentNode, neighbors);

        for (int i = 0; i < 4; i++) {
            neighbor = neighbors[i];
            if(!neighbor) continue;
            if(neighbor->isObstacle && !neighbor->isBoundary) continue;

            double tentative_g_score = heuristic(startNode, *neighbor);
            double tentative_h_score = heuristicToBoundary(*neighbor);
            double tentative_f_score = tentative_g_score + tentative_h_score;
            if(!isVisited(&visited, neighbor)){
                if (!findInQueue(&openSet, neighbor)) {
                	neighbor = initNode(neighbor->row, neighbor->column, i, tentative_f_score, tentative_h_score, tentative_h_score, currentNode);
                	enqueue(&openSet, neighbor);
                } else if (tentative_g_score < neighbor->g_score) {
                	neighbor->direction = i;
                	neighbor->g_score = tentative_g_score;
                	neighbor->h_score = tentative_h_score;
                	neighbor->f_score = tentative_f_score;
                	neighbor->parent = currentNode;
                }
            }
        }
    }

    mex = "Percorso non trovato:\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)mex, strlen(mex), HAL_MAX_DELAY);
    clearQueue(&openSet);
    clearVisitedList(&visited);
    return NULL;
}

void scanMatrix(int8_t* resultRows, int8_t* resultColumns, uint8_t* count) {
    *count = 0;

    // Scansione verticale semplice per raccogliere i punti validi
    for (int j = 0; j < cols; j++) {
        for (int i = 0; i < rows; i++) {
            if (nodes[i][j].isObstacle || nodes[i][j].isBoundary) {
                if (i == 0) {
                    if (!(nodes[i+1][j].isObstacle || nodes[i+1][j].isBoundary)) {
                        resultRows[*count] = (int8_t) i+1;
                        resultColumns[*count] = (int8_t) j;
                        (*count)++;
                    }
                } else if (i == rows - 1) {
                	if (!(nodes[i-1][j].isObstacle || nodes[i-1][j].isBoundary)) {
                		resultRows[*count] = (int8_t) i-1;
                		resultColumns[*count] = (int8_t) j;
                		(*count)++;
                	}
                } else {
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
            if (i == rows - 1) {
            	if (!(nodes[i][j].isObstacle || nodes[i][j].isBoundary)) {
            		resultRows[*count] = (int8_t) i;
            		resultColumns[*count] = (int8_t) j;
            		(*count)++;
            	}
            }
        }
    }

    // Ordina i risultati in modalità serpente
    snakeSortResultsFromFirstValidColumn(resultRows, resultColumns, *count);
}

void snakeSortResultsFromFirstValidColumn(int8_t* resultRows, int8_t* resultColumns, uint8_t count) {
    // Array temporanei per i risultati ordinati
	int8_t tempRows[count];
	int8_t tempColumns[count];

    // Contatori per costruire il nuovo ordine
    int index = 0;

    // Trova la prima colonna con almeno un punto valido
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

    // Procedi colonna per colonna a partire dalla prima colonna valida
    for (int col = firstValidColumn; col < cols; col++) {
        // Trova tutte le celle appartenenti alla colonna attuale
        int colStart = index;
        for (int i = 0; i < count; i++) {
            if (resultColumns[i] == col) {
                tempRows[index] = (int8_t) resultRows[i];
                tempColumns[index] = (int8_t) resultColumns[i];
                index++;
            }
        }
        int colEnd = index;

        // Se la colonna è dispari rispetto alla prima valida, inverti l'ordine delle righe
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

    // Copia i risultati ordinati indietro nei vettori originali
    for (int i = 0; i < count; i++) {
        resultRows[i] = (int8_t) tempRows[i];
        resultColumns[i] = (int8_t) tempColumns[i];
    }
}

// Funzione per stampare i punti
void printMatrix(int8_t* resultRows, int8_t* resultColumns, uint8_t count) {
    for (int i = 0; i < count; i++) {
        int len = snprintf(buff, sizeof(buff), "(%d, %d)\n", resultColumns[i], resultRows[i]);
        // Trasmetto la stringa via UART
        HAL_UART_Transmit(&huart6, (uint8_t*)buff, len, HAL_MAX_DELAY);
    }
}

// Funzione per calcolare la distanza tra due punti
double distance(Point a, Point b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Funzione per determinare il verso di percorrenza
void shortestDirection(Point target, bool *clockwise) {
    int p1_index = -1, p2_index = -1;

    // Trova i due vertici consecutivi (p1, p2) tra cui si trova il target
    for (int i = 0; i < boundary_count; i++) {
        Point p1 = boundaries[i];
        Point p2 = boundaries[(i + 1) % boundary_count];

        if(isPointBetweenTwoPoints(target, p1, p2)){
            p1_index = i;
            p2_index = (i + 1) % boundary_count;
            break;
        }
    }

    // Calcola le distanze totali
    double clockwise_distance = 0;
    double counterclockwise_distance = 0;

    for (int i = 0; i < boundary_count; i++){
    	if(i == p1_index || i == p2_index){
    		break;
    	}
    	clockwise_distance++;
    }

    for (int i = boundary_count - 2; i > 0; i--){
    	if(i == p1_index || i == p2_index){
    		break;
    	}
    	counterclockwise_distance++;
    }

    // Determina il percorso più breve
    *clockwise = clockwise_distance <= counterclockwise_distance;
}
