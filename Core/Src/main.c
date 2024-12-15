/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "mapOperations.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// variabili per ricevere e mandare messaggi dalla seriale
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t rxByte;
uint8_t rxIndex = 0;
char buffer[50];
uint8_t len;

//variabile per mandare stringhe via UART
char* message = "";

//numero di passi che ha fatto la macchina
unsigned long counter = 0;

//numero di passi che deve fare la macchina
unsigned long targetSteps = 0;

//variabile per far partire le ruote più lente per non dare strattoni ai motori
uint8_t counterDelay = 0;

// Logica per ridurre del 5% i passi del motore destro
uint8_t stepRightCounter = 0;
bool stepOK = false;

/*stato del programma
 * 0 -> TODO
 * 1 -> prende un punto dal tracciato e calcola l'angolo tra il punto corrente e quello finale e il numero di passi da fare per arrivare al punto finale
 * 2 -> calcola i passi per girarti dell'angolo indicato
 * 3 -> aspetta che finisca di girare
 * 4 -> vai dritto
 * 5 -> aspetta che abbia finito di andare dritto e torna allo stato 1
 */
uint8_t state = 0;

//Stati per quando la macchina sta eseguendo il perimetro
typedef enum {
    STATE_INIT,
    STATE_DETERMINE_STOP_INDEX,
    STATE_MOVE_CLOCKWISE,
    STATE_MOVE_COUNTERCLOCKWISE,
    STATE_STOP,
	STATE_PERIMETER_COMPLETE,
	STATE_OBSTACLE_PERIMETER
} StatePerimeter;

//Stati per quando la macchina si muove verso un obiettivo
typedef enum {
    STATE_CALCULATE_MOVEMENT,
    STATE_ROTATE,
    STATE_WAIT_FOR_ROTATION,
    STATE_MOVE_FORWARD,
    STATE_WAIT_FOR_MOVEMENT,
    STATE_UPDATE_POSITION,
    STATE_MOVE_TO_GOAL_COMPLETE,
	STATE_OBSTACLE_MOVE
} MoveToGoalState;

//Stati per quando la macchina deve decidere in che direzione andare durante la scansione
typedef enum {
    STATE_CHECK_DIRECTION_CHANGE,
    STATE_LOG_DIRECTION_CHANGE,
    STATE_CALCULATE_TARGET,
    STATE_MOVE_TO_TARGET,
    STATE_FINAL_POINT_LOG,
    STATE_SELECT_DIRECTION_COMPLETE,
	STATE_OBSTACLE_SELECT_DIRECTION
} SelectDirectionState;

//Stati per quando la macchina sta tornando a casa
typedef enum {
    STATE_FIND_SEGMENT,
    STATE_VERIFY_SEGMENT_FOUND,
    STATE_CALCULATE_PATH,
    STATE_FOLLOW_PATH_CLOCKWISE,
    STATE_FOLLOW_PATH_COUNTERCLOCKWISE,
    STATE_BACK_HOME_COMPLETE,
	STATE_OBSTACLE_HOME
} BackHomeState;

//Stati per quando la macchina sta scansionando l'area
typedef enum {
    STATE_NORMAL,
    STATE_OBSTACLE,
	STATE_FINISHED
} ScanState;

StatePerimeter currentStatePerimeter = STATE_INIT; // Stato iniziale
MoveToGoalState currentStateMoveToGoal = STATE_CALCULATE_MOVEMENT; // Stato iniziale
SelectDirectionState currentStateDirection = STATE_CHECK_DIRECTION_CHANGE; // Stato iniziale
BackHomeState currentStateBackHome = STATE_FIND_SEGMENT; // Stato iniziale
ScanState stateScan = STATE_NORMAL; // Stato iniziale


//flag per fermare in emergenza la macchina o per farla ripartire
bool stop = true;
//flag per indicare se la macchina ha finito di fare gli step indicati quindi è nella posizione richiesta
bool moved = true;
//flag per indicare che hai un ostacolo davanti a te
bool obstacle = false;

//flag per indicare se la macchina è andata avanti per salvare il punto nel tracciato
bool avanti = false;
//flag per indicare se la macchina è andata indietro per salvare il punto nel tracciato
bool indietro = false;
//flag per indicare se la macchina è andata a destra per indicare quanti passi e quindi di quanti gradi mi sono spostato
bool destra = false;
//flag per indicare se la macchina è andata a destra per indicare quanti passi e quindi di quanti gradi mi sono spostato
bool sinistra = false;

//posizione corrente della macchina nello spazio
Point current_position = {0.0, 0.0};

//angolo corrente della macchina
double current_angle = M_PI/2;

//posizione della casa base
Point zero = {0.0, 0.0};

//numero di steps e angolo di rotazione che deve fare la macchina
unsigned long steps = 0;
double rotation_angle = 0.0;
long rotation_steps = 0;

//direzione precedente e attuale [0, 1, 2, 3, -1] -> [Sopra, Sotto, Destra, Sinistra]
int8_t previousDir;
int8_t dir;

//colonna e riga della mappa dove si inizia il processo
int8_t startX, startY = 0;

//righe e colonne scansionate verticalmente tramite la funzione scanMatrix
int8_t scanRows[60], scanColumns[60]; // Buffer per i risultati
//numero di scansioni effettuate
uint8_t scanNumber = 0;



// Callback di ricezione dei messaggi seriali
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        if (rxByte == '\n') {        // Controlla il terminatore '\n'
            rxBuffer[rxIndex] = '\0'; // Termina la stringa
            rxIndex = 0;              // Resetta l'indice per la prossima ricezione

            // Chiama la funzione per elaborare la stringa ricevuta
            Process_Received_String((char*)rxBuffer);

        } else if (rxByte != '\r') { // Ignora '\r'
            rxBuffer[rxIndex++] = rxByte; // Aggiungi il carattere al buffer
            if (rxIndex >= RX_BUFFER_SIZE) {
                rxIndex = 0; // Evita overflow del buffer
            }
        }

    	// Riavvia la ricezione del prossimo byte
    	HAL_UART_Receive_IT(&huart6, &rxByte, 1);
    }
}

// Funzione per elaborare la stringa ricevuta
void Process_Received_String(char* str) {

    if(strcmp(str, "save") == 0){
    	save_polygon_to_flash();
    	message = "Tracciato Salvato\n";
    	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    }else if(strcmp(str, "erase") == 0){
    	erase_polygon();
    	message = "Tracciato Cancellato!\n";
    	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    }else if(strcmp(str, "check") == 0){
    	if(load_polygon_from_flash()){
    		message = "Tracciato in memoria\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}else{
    		message = "Tracciato non in memoria\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}
    }else if(strcmp(str, "start") == 0){
    	stop = false;
    	if(load_polygon_from_flash()){
    		message = "Tagliaerba in Movimento\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    		state = 11;
    	}else{
    		message = "Tracciato non in memoria\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}
    }else if(strcmp(str, "stop") == 0){
    	stop = !stop;
    	if(stop){
    		message = "Tagliaerba Fermato\n";
        	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}else{
    		message = "Tagliaerba in Movimento\n";
        	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}

    }else if(strcmp(str, "home") == 0){
    	message = "Tagliaerba torna a casa\n";
    	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	state = 1;
    }else if(strcmp(str, "obstacle") == 0){
    	obstacle = !obstacle;
    	if(obstacle){
    		checkStates();
    	}else{
        	stop = false;
        	message = "Oggetto liberato!\n";
    		HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    	}

    }else if(strcmp(str, "1") == 0){
    	//caso 1 verificato
    	add_boundary_point(0, 28000);
    	add_boundary_point(28000, 28000);
    	add_boundary_point(28000, 0);
    	add_boundary_point(0, 0);
    }else if(strcmp(str, "2") == 0){
    	//caso 2 verificato
    	add_boundary_point(-3500, 28000);
    	add_boundary_point(21000, 28000);
    	add_boundary_point(24500, 0);
    	add_boundary_point(0, 0);
    }else if(strcmp(str, "3") == 0){
    	//caso 3 verificato
    	add_boundary_point(3500, 28000);
    	add_boundary_point(28000, 28000);
    	add_boundary_point(24500, 0);
    	add_boundary_point(0, 0);
    }else if(strcmp(str, "4") == 0){
    	//caso 4 verificato
    	add_boundary_point(3500, 28000);
    	add_boundary_point(28000, 24500);
    	add_boundary_point(24500, 3500);
    	add_boundary_point(0, 0);
    }else if(strcmp(str, "5") == 0){
    	//caso 5 verificato:
    	add_boundary_point(0, 28000);
    	add_boundary_point(-28000, 28000);
    	add_boundary_point(-28000, 0);
    	add_boundary_point(0, 0);
    }else if(strcmp(str, "6") == 0){
    	//caso 6 verificato:
    	add_boundary_point(0, -28000);
    	add_boundary_point(-28000, -28000);
    	add_boundary_point(-28000, 0);
    	add_boundary_point(0, 0);
    }else if(strcmp(str, "7") == 0){
    	//caso 7 verificato:
    	add_boundary_point(0, 10500);
    	add_boundary_point(10500, 10500);
    	add_boundary_point(10500, 21000);
    	add_boundary_point(0, 21000);
    	add_boundary_point(0, 28000);
    	add_boundary_point(17500, 28000);
    	add_boundary_point(17500, 21000);
    	add_boundary_point(24500, 21000);
    	add_boundary_point(24500, 28000);
    	add_boundary_point(28000, 28000);
    	add_boundary_point(28000, 0);
    	add_boundary_point(0, 0);
    }else{
    	// Trova i caratteri ':' e ';' nella stringa
    	char *colon_ptr = strchr(str, ':');
    	char *semicolon_ptr = strchr(str, ';');

    	// Verifica che entrambi i caratteri siano presenti e in posizione corretta
    	if (colon_ptr != NULL && semicolon_ptr != NULL && semicolon_ptr > colon_ptr) {
    		if(state == 0){
				*semicolon_ptr = '\0';  // Termina la stringa dopo la seconda parte
				char *part1 = str;
				char *part2 = colon_ptr + 1;

				*colon_ptr = '\0';  // Termina la prima parte

				// Converti le parti in numeri interi
				int angle = atoi(part1);
				int force = atoi(part2);

				// Usa angle e power come necessario
				len = snprintf(buffer, sizeof(buffer), "%d:%d;\n", angle, force);
				HAL_UART_Transmit(&huart6, (uint8_t*)message, len, HAL_MAX_DELAY);

            	Control_Motors(angle, force);
            }else{
            	message = "Tagliaerba ancora in movimento, aspettare la fine del processo\n";
            	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
            }
    	} else {

    		len = snprintf(buffer, sizeof(buffer), "Not valid: %s", str);
            HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    	}
    }
}

//funzione per controllare i motori via seriale
void Control_Motors(int angle, int force)
{
	//quando rilascio i pulsanti viene mandato il comando 000:000; e qui si ferma la macchina
	if(angle == 0 && force == 0){
		stop = true;
		//se sono andato avanti aggiungo il punto in avanti di n passi
		if(avanti){
			avanti = false;
			add_forward(counter);
		//se sono andato indietro aggiungo il punto indietro di n passi
		}else if(indietro){
			indietro = false;
			add_backward(counter);
		//se sono andato a sinistra aggiorno l'angolo di rotazione della mia macchina
		}else if(sinistra){
			sinistra = false;
		    float angleLeft = calcolaAngoloRotazione(counter);
			add_left(angleLeft);
		//se sono andato a destra aggiorno l'angolo di rotazione della mia macchina
		}else if(destra){
			destra = false;
		    float angleRight = calcolaAngoloRotazione(counter);
			add_right(angleRight);
		}

	//quando clicco un pulsante della macchina si muoverà in base a quale clicco
	}else{
		if(angle == 90){
			//macchina va dritta fino a quando non rilascio il pulsante
			move_forward(0, ULONG_MAX);
			avanti = true;
		}else if(angle == 270){
			//macchina va indietro fino a quando non rilascio il pulsante
			move_backward(0, ULONG_MAX);
			indietro = true;
		}else if(angle == 180){
			//macchina gira a sinistra fino a quando non rilascio il pulsante
			move_left(0, ULONG_MAX);
			sinistra = true;
		}else if (angle == 0){
			//macchina gira a destra fino a quando non rilascio il pulsante
			move_right(0, ULONG_MAX);
			destra = true;
		}
	}
}

//Interrupt del timer, questo timer fornisce gli steps alla mia macchina
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//il mio motore è settato con 1/8 step quindi 200*8 per fare un giro completo * 2 perchè questo timer fa il toggle
    if (htim->Instance == TIM2) {
    	if(!stop){
            // Esegui qui l'operazione che desideri a 2 kHz
        	if(targetSteps > counter){
        		if(counter < 50){
        			counterDelay++;
        			if(counterDelay >= 4){
        				stepOK = true;
        			}
        		}else if(counter < 100){
        			counterDelay++;
        			if(counterDelay >= 3){
        				stepOK = true;
        			}
        		}else if(counter < 150){
        			counterDelay++;
        			if(counterDelay >= 2){
        				stepOK = true;
        			}
        		}else{
        			stepOK = true;
        		}

        		if(stepOK == true){
                    // Step motore sinistro
                    HAL_GPIO_TogglePin(STEP_LEFT_GPIO_Port, STEP_LEFT_Pin);

                    stepRightCounter++;
                    if (stepRightCounter >= 100) { // Ogni 70 passi del sinistro, il destro ne salta 1 quindi riduzione del tot%
                        stepRightCounter = 0;  // Reset del contatore per il motore destro
                    } else {
                        HAL_GPIO_TogglePin(GPIOD, STEP_RIGHT_Pin);  // Step motore destro
                    }
            		counter++;
            		stepOK = false;
            		counterDelay = 0;
        		}
        	}else{
        		moved = true;
        	}
    	}
    }
}

void move_forward(unsigned long cnt, unsigned long trgStep){
	//abilito i motori
	HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_SET);

	stop = false;
	moved = false;
	counter = cnt;
	targetSteps = trgStep;
}

void move_backward(unsigned long cnt, unsigned long trgStep){
	//abilito i motori
	HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_RESET);

	stop = false;
	moved = false;
	counter = cnt;
	targetSteps = trgStep;
}

void move_right(unsigned long cnt, unsigned long trgStep){
	//abilito i motori
	HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_RESET);

	stop = false;
	moved = false;
	counter = cnt;
	targetSteps = trgStep;
}

void move_left(unsigned long cnt, unsigned long trgStep){
	//abilito i motori
	HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_SET);

	stop = false;
	moved = false;
	counter = cnt;
	targetSteps = trgStep;
}

void followPerimeter(Point* target, bool clockwise) {
	currentStatePerimeter = STATE_INIT; // Stato iniziale
    int stop_index = -1;
     // Indice per i boundary
    bool completed = false;

    while (!completed) {
        switch (currentStatePerimeter) {
            case STATE_INIT:
                // Inizializzazione dello stato
                stop_index = -1; // Reset del stop_index
                currentStatePerimeter = STATE_DETERMINE_STOP_INDEX; // Passa allo stato successivo
                break;

            case STATE_DETERMINE_STOP_INDEX:
                if (target != NULL) {
                    for (int i = 0; i < boundary_count; i++) {
                        Point p1 = boundaries[i];
                        Point p2 = boundaries[(i + 1) % boundary_count];
                        if (isPointBetweenTwoPoints(*target, p1, p2)) {
                            stop_index = clockwise ? (i + 1) % boundary_count : i;
                            break;
                        }
                    }
                }
                // Determina lo stato successivo in base alla direzione
                currentStatePerimeter = clockwise ? STATE_MOVE_CLOCKWISE : STATE_MOVE_COUNTERCLOCKWISE;
                break;

            case STATE_MOVE_CLOCKWISE:

                for (int i = 0; i < boundary_count; i++) {
                    // Fermati se siamo arrivati al stop_index
                    if (stop_index != -1 && i == stop_index) {
                    	currentStatePerimeter = STATE_STOP; // Passa allo stato STOP
                        return;  // Esce dalla funzione se arriva a stop_index
                    }

                    Point goal = boundaries[i];
                    moveToGoal(goal);  // Funzione che muove il robot verso il punto goal
                }
                currentStatePerimeter = STATE_PERIMETER_COMPLETE; // Passa allo stato STOP
                break;

            case STATE_MOVE_COUNTERCLOCKWISE:
                // Movimento antiorario: partire da boundary_count - 2 e arrivare fino a 0, poi l'ultimo punto
                for (int i = boundary_count - 2; i >= 0; i--) {
                    // Fermati se siamo arrivati al stop_index
                    if (stop_index != -1 && i == stop_index) {
                    	currentStatePerimeter = STATE_STOP; // Passa allo stato STOP
                        return;  // Esce dalla funzione se arriva a stop_index
                    }

                    Point goal = boundaries[i];
                    moveToGoal(goal);  // Funzione che muove il robot verso il punto goal
                }
                // Dopo aver raggiunto l'ultimo punto (boundary_count - 1)
                Point lastGoal = boundaries[boundary_count - 1];
                moveToGoal(lastGoal);  // Muove il robot all'ultimo punto

                currentStatePerimeter = STATE_PERIMETER_COMPLETE; // Passa allo stato STOP
                break;
            case STATE_STOP:
                // Stato finale
                completed = true; // Esce dal ciclo
                break;

            case STATE_OBSTACLE_PERIMETER:
            	break;

            default:
                // Gestione errore di stato sconosciuto
                completed = true; // Termina con un errore
                break;
        }
    }
}

void moveToGoal(Point goal) {
	currentStateMoveToGoal = STATE_CALCULATE_MOVEMENT; // Stato iniziale
    unsigned long steps = 0;
    double rotation_angle = 0;
    long rotation_steps = 0;

    while (1) {
        switch (currentStateMoveToGoal) {
            case STATE_CALCULATE_MOVEMENT:
                // Calcola i passi e l'angolo necessari per raggiungere il goal
                calculate_steps_and_angle(goal, &steps, &rotation_angle);
                rotation_steps = calcolaNumeroPassiRotazione(rotation_angle);
                currentStateMoveToGoal = STATE_ROTATE; // Passa allo stato di rotazione
                break;

            case STATE_ROTATE:
                // Esegui la rotazione
                if (rotation_steps < 0) {
                    move_right(0, (unsigned long)labs(rotation_steps));  // Ruota a destra
                } else if (rotation_steps > 0) {
                    move_left(0, (unsigned long)rotation_steps);  // Ruota a sinistra
                }
                currentStateMoveToGoal = STATE_WAIT_FOR_ROTATION; // Passa allo stato di attesa
                break;

            case STATE_WAIT_FOR_ROTATION:
                // Aspetta che la rotazione finisca
                if (moved) {
                	currentStateMoveToGoal = STATE_MOVE_FORWARD; // Passa allo stato di avanzamento
                }
                break;

            case STATE_MOVE_FORWARD:
                // Esegui l'avanzamento
                move_forward(0, steps);
                currentStateMoveToGoal = STATE_WAIT_FOR_MOVEMENT; // Passa allo stato di attesa per il movimento
                break;

            case STATE_WAIT_FOR_MOVEMENT:
                // Aspetta che l'avanzamento finisca
                if (moved) {
                	currentStateMoveToGoal = STATE_UPDATE_POSITION; // Passa allo stato di aggiornamento della posizione
                }
                break;

            case STATE_UPDATE_POSITION:
                // Aggiorna la posizione e l'angolo attuale
                current_position = goal;
                current_angle += rotation_angle;
                currentStateMoveToGoal = STATE_MOVE_TO_GOAL_COMPLETE; // Passa allo stato di completamento
                break;

            case STATE_MOVE_TO_GOAL_COMPLETE:
                // Stato finale: esci dalla funzione
                return;
            case STATE_OBSTACLE_MOVE:
            	current_angle += rotation_angle;
            	current_position.x = current_position.x + counter * cos(current_angle);
            	current_position.y = current_position.y + counter * sin(current_angle);
            	return;

            default:
                // Stato sconosciuto: termina con errore
                return;
        }
    }
}

void selectDirection(int previousDir, int dir, Node* node) {
	currentStateDirection = STATE_CHECK_DIRECTION_CHANGE;
    Point target = {0, 0};
    int8_t current_row = 0;
    int8_t current_column = 0;
    int8_t diffRows = 0;
    int8_t diffColumns = 0;
    bool directionLogged = false;

    while (1) {
        switch (currentStateDirection) {
            case STATE_CHECK_DIRECTION_CHANGE:
                // Verifica se è necessario registrare un cambio di direzione
                if (dir != previousDir && dir != -1 && previousDir != -2) {
                	currentStateDirection = STATE_LOG_DIRECTION_CHANGE;
                } else if (dir == -1) {
                	currentStateDirection = STATE_FINAL_POINT_LOG;
                } else {
                	currentStateDirection = STATE_SELECT_DIRECTION_COMPLETE;
                }
                break;

            case STATE_LOG_DIRECTION_CHANGE:
                if (!directionLogged) {
                    len = snprintf(buffer, sizeof(buffer), "Cambio direzione al punto (%d, %d)\n", node->column, node->row);
                    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
                    directionLogged = true;
                }
                findNode(current_position, &current_row, &current_column);
                diffRows = abs(node->row - current_row);
                diffColumns = abs(node->column - current_column);
                currentStateDirection = STATE_CALCULATE_TARGET;
                break;

            case STATE_CALCULATE_TARGET:
                // Calcola il target in base alla direzione
                if (previousDir == 0 || previousDir == 1) { // Sopra o Sotto
                    target.x = current_position.x;
                    target.y = node->row * cellSize - abs(minY);
                } else if (previousDir == 2 || previousDir == 3) { // Destra o Sinistra
                    target.x = node->column * cellSize - abs(minX);
                    target.y = current_position.y;
                } else {
                    target.x = current_position.x;
                    target.y = current_position.y;
                }
                currentStateDirection = STATE_MOVE_TO_TARGET;
                break;

            case STATE_MOVE_TO_TARGET:
                moveToGoal(target);
                if (currentStateDirection != STATE_OBSTACLE_SELECT_DIRECTION) {
                    currentStateDirection = STATE_SELECT_DIRECTION_COMPLETE;
                }
                break;

            case STATE_FINAL_POINT_LOG:
                len = snprintf(buffer, sizeof(buffer), "Punto finale: (%d, %d)\n", node->column, node->row);
                HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

                findNode(current_position, &current_row, &current_column);
                diffRows = abs(node->row - current_row);
                diffColumns = abs(node->column - current_column);

                if (diffRows > 0 || diffColumns > 0) {
                    if (diffRows > 0) {
                        target.x = current_position.x;
                        target.y = node->row * cellSize - abs(minY);
                    } else {
                        target.x = node->column * cellSize - abs(minX);
                        target.y = current_position.y;
                    }
                }
                currentStateDirection = STATE_MOVE_TO_TARGET;
                break;

            case STATE_SELECT_DIRECTION_COMPLETE:
                return;

            case STATE_OBSTACLE_SELECT_DIRECTION:
                stateScan = STATE_OBSTACLE;
                return;

            default:
                return;
        }
    }
}

void backHome() {
	currentStateBackHome = STATE_FIND_SEGMENT;
    int start_index = -1;
    int target_index = boundary_count - 1;  // Indice del punto (0,0)
    int clockwise_steps = 0;
    int counterclockwise_steps = 0;
    bool clockwise = true;
    int index = 0;
    Point next_goal;

    while (1) {
        switch (currentStateBackHome) {
            case STATE_FIND_SEGMENT:
                // Trova il segmento in cui si trova la current_position
                for (int i = 0; i < boundary_count; i++) {
                    Point p1 = boundaries[i];
                    Point p2 = boundaries[(i + 1) % boundary_count];
                    if (isPointBetweenTwoPoints(current_position, p1, p2)) {
                        start_index = i;
                        break;
                    }
                }
                currentStateBackHome = STATE_VERIFY_SEGMENT_FOUND;
                break;

            case STATE_VERIFY_SEGMENT_FOUND:
                // Verifica se il segmento è stato trovato
                if (start_index == -1) {
                    currentStateBackHome = STATE_BACK_HOME_COMPLETE;  // Esci dalla funzione
                } else {
                	currentStateBackHome = STATE_CALCULATE_PATH;
                }
                break;

            case STATE_CALCULATE_PATH:
                // Calcola il percorso in senso orario e antiorario
                clockwise_steps = (target_index - start_index + boundary_count) % boundary_count;
                counterclockwise_steps = (start_index - target_index + boundary_count) % boundary_count;

                // Decidi la direzione più breve
                clockwise = (clockwise_steps <= counterclockwise_steps);

                // Imposta l'indice iniziale per il percorso
                if (clockwise) {
                    index = (start_index + 1) % boundary_count;
                    currentStateBackHome = STATE_FOLLOW_PATH_CLOCKWISE;
                } else {
                    index = start_index;
                    currentStateBackHome = STATE_FOLLOW_PATH_COUNTERCLOCKWISE;
                }
                break;

            case STATE_FOLLOW_PATH_CLOCKWISE:
                // Movimento in senso orario verso il punto (0,0)
                next_goal = boundaries[index];
                moveToGoal(next_goal);  // Muovi verso il prossimo punto
                if (index == target_index) {
                	currentStateBackHome = STATE_BACK_HOME_COMPLETE;
                } else {
                    index = (index + 1) % boundary_count;
                }
                break;

            case STATE_FOLLOW_PATH_COUNTERCLOCKWISE:
                // Movimento in senso antiorario verso il punto (0,0)
                next_goal = boundaries[index];
                moveToGoal(next_goal);  // Muovi verso il prossimo punto
                if (index == target_index) {
                	currentStateBackHome = STATE_BACK_HOME_COMPLETE;
                } else {
                    index = (index - 1 + boundary_count) % boundary_count;
                }
                break;

            case STATE_BACK_HOME_COMPLETE:
                // Stato finale, uscita dalla funzione
                return;

            case STATE_OBSTACLE_HOME:
            	break;

            default:
                // Stato sconosciuto, uscita di emergenza
                return;
        }
    }
}

void checkStates(){
	switch (state) {
	case 14:
		//scansione interna del poligono
		switch (currentStateDirection){
		case STATE_MOVE_TO_TARGET:
			stop = true;
			obstacle = false;
			currentStateMoveToGoal = STATE_OBSTACLE_MOVE;
			currentStateDirection = STATE_OBSTACLE_SELECT_DIRECTION;
			// Stato ostacoli: ricalcola il percorso
			len = snprintf(buffer, sizeof(buffer),"Ostacolo rilevato! Ricalcolo del percorso...\n");
			HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
			break;
		default:
			stop = true;
			len = snprintf(buffer, sizeof(buffer),"Ostacolo rilevato! Eliminare l'ostacolo\n");
			HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
			break;
		}

		break;
	default:
		//in tutti gli altri casi stoppi la macchina e aspetti che l'ostacolo non ci sia più per poter andare avanti
		stop = true;
		// Stato ostacoli: ricalcola il percorso
		len = snprintf(buffer, sizeof(buffer),"Ostacolo rilevato! Eliminare l'ostacolo\n");
		HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
		break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // Avvia il timer e la seriale in modalità interrupt
  HAL_TIM_Base_Start_IT(&htim2);  // Se usi TIM2
  HAL_UART_Receive_IT(&huart6, &rxByte, 1); // Ricevi un byte alla volta

  //disabilito i motori
  HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (state) {
	  	  case 0:
	  		  break;
	  	  case 1:
	  		  moveToGoal(zero);
	  		  state = 2;
	  		  currentStateMoveToGoal = STATE_CALCULATE_MOVEMENT;
	  		  break;
	      case 2:
	    	  //rigirati in avanti

	    	  // Calcola i passi per raggiungere l'angolo PI/2
	    	  float angle = M_PI/2- current_angle;
	    	  // Normalizza tra -pi e pi
	    	  if (angle > M_PI){
	    		  angle -= 2 * M_PI;
	    	  }
			  else if (angle < -M_PI){
				  angle += 2 * M_PI;
			  }

	    	  rotation_steps = calcolaNumeroPassiRotazione(angle);

	    	  if (rotation_steps < 0) {
	    		  move_right(0, labs(rotation_steps));  // Ruota a destra
	    	  } else if (rotation_steps > 0) {
	    		  move_left(0, labs(rotation_steps));  // Ruota a sinistra
	    	  }
	    	  state = 3;
	          break;
	      case 3:
	    	  //aspetta che finisca di girarsi e poi vai avanti
	    	  if(moved){
	    		  current_angle = M_PI/2;
	    		  // Aggiungi la nuova posizione ai confini del poligono
	    		  add_boundary_point(current_position.x, current_position.y);
		    	  HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_SET);
		    	  HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_SET);
	    		  state = 0;
	    	  }
	    	  break;
	      case 10:
			  followPerimeter(NULL, true);
			  message = "perimetro concluso!\n";
	    	  HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
	    	  state = 11;
	    	  break;
	      case 11:
	    	  //creo la mappa per scansionare il poligono
	    	  if(createNodes()){
	    		  state = 12;
	    	  }else{
	    		  state = 0;
	    	  }
	    	  break;
	      case 12:
	    	  //scansiono l'area interna del poligono
	    	  message = "scansione:\n";
	    	  HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

	    	  current_position = zero;
	    	  int8_t startRow = 0;
	    	  int8_t startColumn = 0;
	    	  uint8_t index = 0;

	    	  scanMatrix(scanRows, scanColumns, &scanNumber);
	    	  printMatrix(scanRows, scanColumns, scanNumber);
	    	  state = 13;
	    	  break;
	      case 13:
	    	  //in questo stato si cerca di andare al punto iniziale della scansione

    		  findNode(current_position, &startRow, &startColumn);
    		  //cerco il primo punto dove andare per far partire la scansione
    		  len = snprintf(buffer, sizeof(buffer), "Tracciato: (%d, %d) -> (%d, %d)\n", startColumn, startRow, scanColumns[0], scanRows[0]);
    		  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    		  Node* currentNode = NULL;
    		  if(scanRows[0] != startRow || scanColumns[0] != startColumn){
        		  currentNode = findShortestPathAStar(scanRows[0], scanColumns[0], startRow, startColumn);
    		  }

    		  if(currentNode == NULL){
    			  //se non trovo un percorso per arrivare al punto iniziale mi muovo sul bordo per arrivare a tale punto
    			  len = snprintf(buffer, sizeof(buffer), "Nodo inarrivabile, percorso per: (%d, %d)\n", scanColumns[0], scanRows[0]);
    			  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    			  Node* wallNode = findNearestWall(scanRows[0], scanColumns[0]);

    			  if(wallNode != NULL){
    				  len = snprintf(buffer, sizeof(buffer), "vado a: (%d, %d)\n", wallNode->column, wallNode->row);
    				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    				  //trovo le coordinate per il punto di riferimento
    				  double resultX, resultY = 0.0;
    				  if(findPositionOnSegment(wallNode->row, wallNode->column, &resultX, &resultY)){
    					  len = snprintf(buffer, sizeof(buffer), "posizione: (%d, %d)\n", (int) round(resultX), (int) round(resultY));
    					  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    					  //fai partire la funzione per arrivare nel punto preciso seguendo il perimetro
    					  Point target = {round(resultX), round(resultY)};

    					  bool clockwise = false;
    					  shortestDirection(target, &clockwise);

    					  followPerimeter(&target, clockwise);

    			    	  moveToGoal(target);
    				  }else{
    					  //fuori dal poligono, vai allo stato 0
    					  message = "punto trovato fuori dal poligono, errore!\n";
    					  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    					  state = 0;
    					  break;
    				  }

    				  //a questo punto sono nel punto di entrata del poligono, vado verso il punto iniziale della scansione
    				  wallNode = wallNode->parent;

    				  previousDir = -2; // Direzione iniziale inesistente

    				  while (wallNode) {
    					  dir = wallNode->direction;

    					  selectDirection(previousDir, dir, wallNode);

    					  wallNode = wallNode->parent;

    					  previousDir = dir;
    				  }
    				  index++;

    			  }else{
    				  //non ho trovato un percorso per arrivare al punto iniziale, vai a stato 0
    				  message = "non ho modo di entrare nel poligono, cambia poligono!\n";
    				  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    				  state = 0;
    				  break;
    			  }
    		  }
    		  //ho trovato un percorso per arrivare al punto iniziale, allora ci vado
    		  state = 14;
	    	  break;
	      case 14:
	    	  switch (stateScan) {
	    	  case STATE_NORMAL:
	    		  if (currentNode == NULL) {
	    			  // Trova il nodo di partenza
	    			  findNode(current_position, &startRow, &startColumn);
	    			  // Cerca il primo punto per iniziare la scansione
	    			  len = snprintf(buffer, sizeof(buffer),
	    					  "Tracciato: (%d, %d) -> (%d, %d)\n",
							  startColumn, startRow, scanColumns[index], scanRows[index]);
	    			  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	    			  // Trova il percorso più breve verso il punto di scansione
	        		  if(scanRows[index] != startRow || scanColumns[index] != startColumn){
	        			  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index],startRow, startColumn);
	        		  }
	    		  }
	    		  previousDir = -2;
	    		  // Segui il percorso per il punto corrente
	    		  while (currentNode) {
	    			  dir = currentNode->direction;

	    			  // Usa selectDirection() per muoversi
	    			  selectDirection(previousDir, dir, currentNode);

	    			  // Controlla se è stato rilevato un ostacolo
	    			  if (stateScan == STATE_OBSTACLE) {
	    				  break;
	    			  }

	    			  currentNode = currentNode->parent;
	    			  previousDir = dir;
	    		  }

	    		  if (stateScan != STATE_OBSTACLE) {
	    			  // Passa al punto successivo se non c'è ostacolo
	    			  index++;
	    			  if (index == scanNumber) {
	    				  // Scansione terminata, torna a casa
	    				  stateScan = STATE_FINISHED;
	    			  }
	    		  }
	    		  break;
	    	  case STATE_OBSTACLE:
	    		  // Ricalcola il percorso con A*
	    		  findNode(current_position, &startRow, &startColumn);
	    		  int obstacleRow = 0;
	    		  int obstacleColumn = 0;

	    		  if(previousDir == -2){
	    			  if(dir == 0){
	    				  //ho un ostacolo sopra di me
	    				  nodes[startRow+1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow+1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(dir == 1){
	    				  //ho un ostacolo sotto di me
	    				  nodes[startRow-1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow-1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(dir == 2){
	    				  //ho un ostacolo a destra
	    				  nodes[startRow][startColumn + 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn+1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(dir == 3){
	    				  //ho un ostacolo a sinistra
	    				  nodes[startRow][startColumn - 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn - 1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }
	    		  }else{
	    			  if(previousDir == 0){
	    				  //ho un ostacolo sopra di me
	    				  nodes[startRow+1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow+1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(previousDir == 1){
	    				  //ho un ostacolo sotto di me
	    				  nodes[startRow-1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow-1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(previousDir == 2){
	    				  //ho un ostacolo a destra
	    				  nodes[startRow][startColumn + 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn+1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(previousDir == 3){
	    				  //ho un ostacolo a sinistra
	    				  nodes[startRow][startColumn - 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn-1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }
	    		  }

				  len = snprintf(buffer, sizeof(buffer), "ora: (%d, %d), ostacolo: (%d, %d)\n", startColumn, startRow, obstacleColumn, obstacleRow);
				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	    		  // Ritorna allo stato normale
	    		  stateScan = STATE_NORMAL;
	    		  break;

	    	  case STATE_FINISHED:
	    		  // Scansione completata
	    		  state = 15;
	    		  break;

	    	  default:
	    		  // Stato non riconosciuto (errore)
	    		  len = snprintf(buffer, sizeof(buffer), "Errore: Stato non riconosciuto!\n");
	    		  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	    		  break;
	    	  }
	    	  break;
	      case 15:
	    	  createNodes();
	    	  current_position.x = 0;
	    	  current_position.y = 35000;
	    	  findNode(current_position, &startRow, &startColumn);
	    	  Node* toWall = NULL;
	    	  if(!(startRow == rows-1 || startColumn == cols -1)){

				  toWall = findNearestWall(startRow, startColumn);
				  Node* results[2];
				  int index_wall = 0;

				  while (toWall){
					  results[index_wall] = toWall;
					  toWall = toWall->parent;
					  index_wall++;
				  }

				  previousDir = -2; // Direzione iniziale inesistente
				  if(index_wall > 1){
					  for(int i = index_wall-1; i>1; i--){
						  toWall = results[i];
						  len = snprintf(buffer, sizeof(buffer), "vado a: (%d, %d)\n", toWall->column, toWall->row);
						  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

						  dir = toWall->direction;

						  selectDirection(previousDir, dir, toWall);

						  toWall = toWall->parent;

						  previousDir = dir;
					  }
				  }


				  toWall = results[0];

				  //vado sul bordo del poligono ora
				  len = snprintf(buffer, sizeof(buffer), "vado a: (%d, %d)\n", toWall->column, toWall->row);
				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	    	  }else{
	    		  toWall = &nodes[startRow][startColumn];
	    	  }
			  //trovo le coordinate per il punto di riferimento
			  double resultX, resultY = 0.0;
			  if(findPositionOnSegment(toWall->row, toWall->column, &resultX, &resultY)){
				  len = snprintf(buffer, sizeof(buffer), "posizione: (%d, %d)\n", (int) round(resultX), (int) round(resultY));
				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

				  //fai partire la funzione per arrivare nel punto preciso seguendo il perimetro
				  Point target = {round(resultX), round(resultY)};
				  moveToGoal(target);

				  backHome();
				  message = "arrivato a casa!\n";
				  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
			  }else{
				  //fuori dal poligono, vai allo stato 0
				  message = "punto trovato fuori dal poligono, errore!\n";
				  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
			  }
			  freeNodes();
			  state = 0;
	    	  break;
	      default:
	          // codice da eseguire se nessun caso corrisponde
	          break;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin|ENABLE_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin|DIR_LEFT_Pin|STEP_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STEP_LEFT_GPIO_Port, STEP_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_RIGHT_Pin ENABLE_LEFT_Pin */
  GPIO_InitStruct.Pin = ENABLE_RIGHT_Pin|ENABLE_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_RIGHT_Pin DIR_LEFT_Pin STEP_RIGHT_Pin */
  GPIO_InitStruct.Pin = DIR_RIGHT_Pin|DIR_LEFT_Pin|STEP_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_LEFT_Pin */
  GPIO_InitStruct.Pin = STEP_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_LEFT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
