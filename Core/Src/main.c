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
#include "memoryManager.h"
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

// serial variables
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t rxByte;
uint8_t rxIndex = 0;
char buffer[50];
uint8_t len;
char* message = "";

// number of steps of the machine
unsigned long counter = 0;

// number of steps that the machine has to do
unsigned long targetSteps = 0;

// number of steps to let the machine move slowly at the start
uint8_t counterDelay = 0;

// number of steps that the right motor has to do (because this motor is moving faster)
uint8_t stepRightCounter = 0;
bool stepOK = false;

/*
 * states of the program
 * 0 		: nothing
 * 1 -> 3 	: move the machine to the (0;0) coordinate
 * 10 -> 15	: move to a target coordinate
 */
uint8_t state = 0;

// States for when the machine is following the perimeter
typedef enum {
    STATE_INIT,
    STATE_DETERMINE_STOP_INDEX,
    STATE_MOVE_CLOCKWISE,
    STATE_MOVE_COUNTERCLOCKWISE,
    STATE_STOP,
	STATE_PERIMETER_COMPLETE,
	STATE_OBSTACLE_PERIMETER
} StatePerimeter;

// States for when the machine is going to a specific coordinate
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

// States for when the machine is checking in which direction has to go
typedef enum {
    STATE_CHECK_DIRECTION_CHANGE,
    STATE_LOG_DIRECTION_CHANGE,
    STATE_CALCULATE_TARGET,
    STATE_MOVE_TO_TARGET,
    STATE_FINAL_POINT_LOG,
    STATE_SELECT_DIRECTION_COMPLETE,
	STATE_OBSTACLE_SELECT_DIRECTION
} SelectDirectionState;

// States for when the machine is going back to home (0,0)
typedef enum {
    STATE_FIND_SEGMENT,
    STATE_VERIFY_SEGMENT_FOUND,
    STATE_CALCULATE_PATH,
    STATE_FOLLOW_PATH_CLOCKWISE,
    STATE_FOLLOW_PATH_COUNTERCLOCKWISE,
    STATE_BACK_HOME_COMPLETE,
	STATE_OBSTACLE_HOME
} BackHomeState;

// States for when the machine is scanning the area
typedef enum {
    STATE_NORMAL,
    STATE_OBSTACLE,
	STATE_FINISHED
} ScanState;

StatePerimeter currentStatePerimeter = STATE_INIT; // Init state
MoveToGoalState currentStateMoveToGoal = STATE_CALCULATE_MOVEMENT; // Init state
SelectDirectionState currentStateDirection = STATE_CHECK_DIRECTION_CHANGE; // Init state
BackHomeState currentStateBackHome = STATE_FIND_SEGMENT; // Init state
ScanState stateScan = STATE_NORMAL; // Init state


// flag to stop the machine for emergency
bool stop = true;
// flag to indicate when the car has moved in the target location
bool moved = true;
//flag to indicate if there is an obstacle
bool obstacle = false;

// Flag to indicate whether the machine has moved forward to save the point in the track
bool forward = false;

// Flag to indicate whether the machine has moved backward to save the point in the track
bool backward = false;

// Flag to indicate whether the machine has moved to the right to track the number of steps and the corresponding angle change
bool right = false;

// Flag to indicate whether the machine has moved to the left to track the number of steps and the corresponding angle change
bool left = false;


// Current position of the machine in space
Point current_position = {0.0, 0.0};

// Current angle of the machine (initialized to π/2 radians, meaning it starts facing upwards)
double current_angle = M_PI / 2;

// Position of the home base
Point zero = {0.0, 0.0};

// Number of steps and rotation angle required for the machine
unsigned long steps = 0;
double rotation_angle = 0.0;
long rotation_steps = 0;

// Previous and current direction [0, 1, 2, 3, -1] -> [Up, Down, Right, Left]
int8_t previousDir;
int8_t dir;

// Column and row in the map where the process starts
int8_t startX, startY = 0;

// Rows and columns scanned vertically using the scanMatrix function
int8_t scanRows[60], scanColumns[60]; // Buffer for scan results

// Number of scans performed
uint8_t scanNumber = 0;

/**
 * @brief Handler for sensor interrupts.
 *
 * This function is called when an interrupt occurs on the GPIO pin 0. It calls the
 * `HAL_GPIO_EXTI_IRQHandler()` function to handle the interrupt for the pin.
 */
void EXTI0_0_IRQHandler(void){
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief Callback for sensor interrupt.
 *
 * This function is called when a GPIO interrupt is triggered. If the interrupt
 * is from GPIO pin 0, it checks the state of the obstacle sensor. If an obstacle
 * is detected, it calls `checkStates()`. Otherwise, it sends a message via UART
 * indicating that the area is free.
 *
 * @param GPIO_Pin The pin that triggered the interrupt.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		obstacle = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		if(obstacle){
			checkStates();
		}else{
			stop = false;
			message = "Object Free!\n";
			HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
		}
	}
}

/**
 * @brief Callback function for receiving UART data.
 *
 * This function is called when UART data is received. It processes the received
 * byte, and when a newline character (`\n`) is detected, it processes the
 * complete string by calling `Process_Received_String()`. It then continues the
 * reception of data via UART.
 *
 * @param huart Pointer to the UART handle.
 */
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

/**
 * @brief Processes the received command string.
 *
 * This function processes the received command and performs the corresponding action
 * such as saving, erasing, checking the track, starting or stopping the robot, or controlling
 * the motors based on the parsed command. It sends status messages back via UART.
 *
 * @param str The command string received via UART.
 */
void Process_Received_String(char* str) {

    if(strcmp(str, "save") == 0){
    	save_polygon_to_flash();
    	message = "Track Saved\n";
    	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    }else if(strcmp(str, "erase") == 0){
    	erase_polygon();
    	message = "Track Deleted!\n";
    	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    }else if(strcmp(str, "check") == 0){
    	if(load_polygon_from_flash()){
    		message = "Track in memory\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}else{
    		message = "Track not in memory\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}
    }else if(strcmp(str, "start") == 0){
    	stop = false;
    	if(load_polygon_from_flash()){
    		message = "Track in memory\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    		state = 10;
    	}else{
    		message = "Track not in memory\n";
    		HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}
    }else if(strcmp(str, "stop") == 0){
    	stop = !stop;
    	if(stop){
    		message = "Machine stopped\n";
        	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}else{
    		message = "Machine in acrion\n";
        	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	}

    }else if(strcmp(str, "home") == 0){
    	message = "Machine is going back Home...\n";
    	HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    	state = 1;
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
    }else {
        // Find ':' and ';' characters in the string
        char *colon_ptr = strchr(str, ':');
        char *semicolon_ptr = strchr(str, ';');

        // Verify that both characters are present and in the correct position
        if (colon_ptr != NULL && semicolon_ptr != NULL && semicolon_ptr > colon_ptr) {
            if(state == 0){
                *semicolon_ptr = '\0';  // Terminate the string after the second part
                char *part1 = str;
                char *part2 = colon_ptr + 1;

                *colon_ptr = '\0';  // Terminate the first part

                // Convert parts to integers
                int angle = atoi(part1);
                int force = atoi(part2);

                // Use angle and power as needed
                len = snprintf(buffer, sizeof(buffer), "%d:%d;\n", angle, force);
                HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

                Control_Motors(angle, force);
            } else {
                message = "Mower still in motion, please wait for the process to complete\n";
                HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
            }
        } else {
            len = snprintf(buffer, sizeof(buffer), "Not valid: %s", str);
            HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
    }
}

/**
 * @brief Controls the robot motors based on the received angle and force.
 *
 * This function processes the received angle and force parameters and moves the
 * robot accordingly. If both values are zero, the robot stops. Otherwise, it
 * moves the robot in the direction specified by the angle (forward, backward,
 * left, or right).
 *
 * @param angle The desired angle of movement (0 for right, 90 for forward,
 *              180 for left, and 270 for backward).
 * @param force The force to apply for the movement (affects motor speed or power).
 */
void Control_Motors(int angle, int force) {
    // If both angle and force are zero, stop the machine
    if(angle == 0 && force == 0){
        stop = true;
        if(forward){
            forward = false;
            add_forward(counter);
        } else if(backward){
            backward = false;
            add_backward(counter);
        } else if(left){
            left = false;
            float angleLeft = calcolaAngoloRotazione(counter);
            add_left(angleLeft);
        } else if(right){
            right = false;
            float angleRight = calcolaAngoloRotazione(counter);
            add_right(angleRight);
        }
    } else {
        // Move the machine based on the command received
        if(angle == 90){
            move_forward(0, ULONG_MAX);
            forward = true;
        } else if(angle == 270){
            move_backward(0, ULONG_MAX);
            backward = true;
        } else if(angle == 180){
            move_left(0, ULONG_MAX);
            left = true;
        } else if (angle == 0){
            move_right(0, ULONG_MAX);
            right = true;
        }
    }
}

/**
 * @brief Timer interrupt callback for motor step generation.
 *
 * This callback is triggered by a timer interrupt and is responsible for generating
 * steps for the motors. The function checks whether the robot should continue
 * moving or if the movement should stop based on the target step count. It manages
 * the step generation for both the left and right motors, ensuring proper synchronization.
 *
 * @param htim The timer handle associated with the interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // My motor is set to 1/8 step, so 200 * 8 to complete a full revolution * 2 because this timer toggles
    if (htim->Instance == TIM2) {
        if (!stop) {
            // Perform the desired operation at 2 kHz
            if (targetSteps > counter) {
                if (counter < 50) {
                    counterDelay++;
                    if (counterDelay >= 4) {
                        stepOK = true;
                    }
                } else if (counter < 100) {
                    counterDelay++;
                    if (counterDelay >= 3) {
                        stepOK = true;
                    }
                } else if (counter < 150) {
                    counterDelay++;
                    if (counterDelay >= 2) {
                        stepOK = true;
                    }
                } else {
                    stepOK = true;
                }

                if (stepOK == true) {
                    // Left motor step
                    HAL_GPIO_TogglePin(STEP_LEFT_GPIO_Port, STEP_LEFT_Pin);

                    stepRightCounter++;
                    if (stepRightCounter >= 100) { // Every 70 steps of the left motor, the right motor skips 1, reducing by a total percentage
                        stepRightCounter = 0;  // Reset the counter for the right motor
                    } else {
                        HAL_GPIO_TogglePin(GPIOD, STEP_RIGHT_Pin);  // Right motor step
                    }
                    counter++;
                    stepOK = false;
                    counterDelay = 0;
                }
            } else {
                moved = true;
            }
        }
    }
}

/**
 * @brief Moves the robot forward.
 *
 * This function enables the motors and sets the direction pins to move the robot forward.
 * It sets the `stop` flag to `false` (indicating that the robot should move), and initializes the movement counter and target steps.
 *
 * @param cnt The current step count (starting point for the movement).
 * @param trgStep The target step count (number of steps to be taken).
 */
void move_forward(unsigned long cnt, unsigned long trgStep){
    // Enable the motors by setting the enable pins to active low
    HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

    // Set the direction for forward movement
    // LEFT motor: Reset direction (forward)
    // RIGHT motor: Set direction (forward)
    HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_SET);

    // Set flags and counters for movement
    stop = false;     // Robot is not stopped, so it will move
    moved = false;    // Movement has not been completed yet
    counter = cnt;    // Set the current count of steps
    targetSteps = trgStep;  // Set the target step count
}

/**
 * @brief Moves the robot backward.
 *
 * This function enables the motors and sets the direction pins to move the robot backward.
 * It sets the `stop` flag to `false` (indicating that the robot should move), and initializes the movement counter and target steps.
 *
 * @param cnt The current step count (starting point for the movement).
 * @param trgStep The target step count (number of steps to be taken).
 */
void move_backward(unsigned long cnt, unsigned long trgStep){
    // Enable the motors by setting the enable pins to active low
    HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

    // Set the direction for backward movement
    // LEFT motor: Set direction (backward)
    // RIGHT motor: Reset direction (backward)
    HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_RESET);

    // Set flags and counters for movement
    stop = false;     // Robot is not stopped, so it will move
    moved = false;    // Movement has not been completed yet
    counter = cnt;    // Set the current count of steps
    targetSteps = trgStep;  // Set the target step count
}

/**
 * @brief Moves the robot to the right.
 *
 * This function enables the motors and sets the direction pins to move the robot to the right.
 * It sets the `stop` flag to `false` (indicating that the robot should move), and initializes the movement counter and target steps.
 *
 * @param cnt The current step count (starting point for the movement).
 * @param trgStep The target step count (number of steps to be taken).
 */
void move_right(unsigned long cnt, unsigned long trgStep){
    // Enable the motors by setting the enable pins to active low
    HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

    // Set the direction for right movement
    // LEFT motor: Reset direction (forward)
    // RIGHT motor: Reset direction (forward)
    HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_RESET);

    // Set flags and counters for movement
    stop = false;     // Robot is not stopped, so it will move
    moved = false;    // Movement has not been completed yet
    counter = cnt;    // Set the current count of steps
    targetSteps = trgStep;  // Set the target step count
}

/**
 * @brief Moves the robot to the left.
 *
 * This function enables the motors and sets the direction pins to move the robot to the left.
 * It sets the `stop` flag to `false` (indicating that the robot should move), and initializes the movement counter and target steps.
 *
 * @param cnt The current step count (starting point for the movement).
 * @param trgStep The target step count (number of steps to be taken).
 */
void move_left(unsigned long cnt, unsigned long trgStep){
    // Enable the motors by setting the enable pins to active low
    HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_RESET);

    // Set the direction for left movement
    // LEFT motor: Set direction (backward)
    // RIGHT motor: Set direction (backward)
    HAL_GPIO_WritePin(GPIOD, DIR_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, DIR_RIGHT_Pin, GPIO_PIN_SET);

    // Set flags and counters for movement
    stop = false;     // Robot is not stopped, so it will move
    moved = false;    // Movement has not been completed yet
    counter = cnt;    // Set the current count of steps
    targetSteps = trgStep;  // Set the target step count
}

/**
 * @brief Moves the robot along the perimeter of the defined boundaries.
 *
 * The robot follows the perimeter in either a clockwise or counterclockwise direction
 * until it reaches the stop point, which is determined based on the target position.
 * If no target is provided, the robot completes a full perimeter loop.
 *
 * @param target Pointer to the target Point where the robot should stop. If NULL,
 *               the robot completes the entire perimeter.
 * @param clockwise Boolean indicating the direction of movement:
 *                  - true: move clockwise
 *                  - false: move counterclockwise
 */
void followPerimeter(Point* target, bool clockwise) {
    currentStatePerimeter = STATE_INIT; // Initial state
    int stop_index = -1;
    bool completed = false; // Completion flag

    while (!completed) {
        switch (currentStatePerimeter) {
            case STATE_INIT:
                // Initialize state
                stop_index = -1; // Reset stop_index
                currentStatePerimeter = STATE_DETERMINE_STOP_INDEX; // Move to the next state
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
                // Determine the next state based on the direction
                currentStatePerimeter = clockwise ? STATE_MOVE_CLOCKWISE : STATE_MOVE_COUNTERCLOCKWISE;
                break;

            case STATE_MOVE_CLOCKWISE:
                for (int i = 0; i < boundary_count; i++) {
                    // Stop if we reach stop_index
                    if (stop_index != -1 && i == stop_index) {
                        currentStatePerimeter = STATE_STOP; // Move to STOP state
                        return;  // Exit function if stop_index is reached
                    }

                    Point goal = boundaries[i];
                    moveToGoal(goal);  // Move the robot toward the goal point
                }
                currentStatePerimeter = STATE_PERIMETER_COMPLETE; // Move to STOP state
                break;

            case STATE_MOVE_COUNTERCLOCKWISE:
                // Counterclockwise movement: start from boundary_count - 2 and go down to 0, then the last point
                for (int i = boundary_count - 2; i >= 0; i--) {
                    // Stop if we reach stop_index
                    if (stop_index != -1 && i == stop_index) {
                        currentStatePerimeter = STATE_STOP; // Move to STOP state
                        return;  // Exit function if stop_index is reached
                    }

                    Point goal = boundaries[i];
                    moveToGoal(goal);  // Move the robot toward the goal point
                }
                // After reaching the last point (boundary_count - 1)
                Point lastGoal = boundaries[boundary_count - 1];
                moveToGoal(lastGoal);  // Move the robot to the last point

                currentStatePerimeter = STATE_PERIMETER_COMPLETE; // Move to STOP state
                break;

            case STATE_STOP:
                // Final state
                completed = true; // Exit loop
                break;

            case STATE_OBSTACLE_PERIMETER:
                break;

            default:
                // Handle unknown state error
                completed = true; // Terminate with an error
                break;
        }
    }
}

/**
 * @brief Moves the robot towards a specified goal point.
 *
 * This function controls the movement of the robot in a state-based manner.
 * It calculates the necessary steps and rotation angle, executes the rotation,
 * moves forward to the goal, and updates the robot's position upon completion.
 *
 * @param goal The target point the robot should move towards.
 */
void moveToGoal(Point goal) {
    currentStateMoveToGoal = STATE_CALCULATE_MOVEMENT; // Initial state
    unsigned long steps = 0;   // Number of movement steps required
    double rotation_angle = 0; // Angle required to reach the goal
    long rotation_steps = 0;   // Steps required to rotate

    while (1) {
        switch (currentStateMoveToGoal) {
            case STATE_CALCULATE_MOVEMENT:
                // Calculate the number of steps and rotation angle needed to reach the goal
                calculate_steps_and_angle(goal, &steps, &rotation_angle);
                rotation_steps = calcolaNumeroPassiRotazione(rotation_angle);
                currentStateMoveToGoal = STATE_ROTATE; // Transition to rotation state
                break;

            case STATE_ROTATE:
                // Perform rotation to align with the goal
                if (rotation_steps < 0) {
                    move_right(0, (unsigned long)labs(rotation_steps));  // Rotate right
                } else if (rotation_steps > 0) {
                    move_left(0, (unsigned long)rotation_steps);  // Rotate left
                }
                currentStateMoveToGoal = STATE_WAIT_FOR_ROTATION; // Transition to wait state
                break;

            case STATE_WAIT_FOR_ROTATION:
                // Wait until rotation is complete
                if (moved) {
                    currentStateMoveToGoal = STATE_MOVE_FORWARD; // Proceed to forward movement
                }
                break;

            case STATE_MOVE_FORWARD:
                // Move forward towards the goal
                move_forward(0, steps);
                currentStateMoveToGoal = STATE_WAIT_FOR_MOVEMENT; // Transition to movement wait state
                break;

            case STATE_WAIT_FOR_MOVEMENT:
                // Wait until forward movement is complete
                if (moved) {
                    currentStateMoveToGoal = STATE_UPDATE_POSITION; // Proceed to update position
                }
                break;

            case STATE_UPDATE_POSITION:
                // Update the robot's position and current angle
                current_position = goal;
                current_angle += rotation_angle;
                currentStateMoveToGoal = STATE_MOVE_TO_GOAL_COMPLETE; // Mark movement as complete
                break;

            case STATE_MOVE_TO_GOAL_COMPLETE:
                // Final state: exit the function
                return;

            case STATE_OBSTACLE_MOVE:
                // Handle movement in the presence of obstacles
                current_angle += rotation_angle;
                current_position.x = current_position.x + counter * cos(current_angle);
                current_position.y = current_position.y + counter * sin(current_angle);
                return;

            default:
                // Unknown state: exit with an error
                return;
        }
    }
}

/**
 * @brief Selects the direction and moves the robot accordingly.
 *
 * This function manages the process of checking and logging direction changes,
 * calculating the target position, and moving towards it. It ensures that the
 * robot correctly follows its path and logs significant direction changes.
 *
 * @param previousDir The previous movement direction of the robot.
 * @param dir The new movement direction.
 * @param node Pointer to the target node (destination).
 */
void selectDirection(int previousDir, int dir, Node* node) {
    currentStateDirection = STATE_CHECK_DIRECTION_CHANGE; // Initial state
    Point target = {0, 0};  // Target position for movement
    int8_t current_row = 0;
    int8_t current_column = 0;
    int8_t diffRows = 0;
    int8_t diffColumns = 0;
    bool directionLogged = false; // Tracks if direction change has been logged

    while (1) {
        switch (currentStateDirection) {
            case STATE_CHECK_DIRECTION_CHANGE:
                // Check if a direction change should be logged
                if (dir != previousDir && dir != -1 && previousDir != -2) {
                    currentStateDirection = STATE_LOG_DIRECTION_CHANGE;
                } else if (dir == -1) {
                    currentStateDirection = STATE_FINAL_POINT_LOG;
                } else {
                    currentStateDirection = STATE_SELECT_DIRECTION_COMPLETE;
                }
                break;

            case STATE_LOG_DIRECTION_CHANGE:
                // Log the direction change
                if (!directionLogged) {
                    len = snprintf(buffer, sizeof(buffer),
                                   "Cambio direzione al punto (%d, %d)\n",
                                   node->column, node->row);
                    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
                    directionLogged = true;
                }
                // Find the robot's current node position
                findNode(current_position, &current_row, &current_column);
                diffRows = abs(node->row - current_row);
                diffColumns = abs(node->column - current_column);
                currentStateDirection = STATE_CALCULATE_TARGET;
                break;

            case STATE_CALCULATE_TARGET:
                // Determine the target point based on the previous direction
                if (previousDir == 0 || previousDir == 1) { // Moving Up or Down
                    target.x = current_position.x;
                    target.y = node->row * cellSize - abs(minY);
                } else if (previousDir == 2 || previousDir == 3) { // Moving Right or Left
                    target.x = node->column * cellSize - abs(minX);
                    target.y = current_position.y;
                } else { // No movement change
                    target.x = current_position.x;
                    target.y = current_position.y;
                }
                currentStateDirection = STATE_MOVE_TO_TARGET;
                break;

            case STATE_MOVE_TO_TARGET:
                // Move the robot to the calculated target
                moveToGoal(target);
                if (currentStateDirection != STATE_OBSTACLE_SELECT_DIRECTION) {
                    currentStateDirection = STATE_SELECT_DIRECTION_COMPLETE;
                }
                break;

            case STATE_FINAL_POINT_LOG:
                // Log the final point
                len = snprintf(buffer, sizeof(buffer),
                               "Punto finale: (%d, %d)\n",
                               node->column, node->row);
                HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

                // Find the robot's current node position
                findNode(current_position, &current_row, &current_column);
                diffRows = abs(node->row - current_row);
                diffColumns = abs(node->column - current_column);

                // Determine the correct target point
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
                // Movement process completed successfully
                return;

            case STATE_OBSTACLE_SELECT_DIRECTION:
                // Handle obstacle detection and update state
                stateScan = STATE_OBSTACLE;
                return;

            default:
                // Unknown state, terminate
                return;
        }
    }
}

/**
 * @brief Navigate the robot back to its home position (point (0,0)) along the polygon boundary.
 *
 * This function handles the process of moving the robot back to its starting point
 * by navigating along the polygon boundary in either a clockwise or counterclockwise direction.
 * It checks which direction would take the robot to its destination in the shortest number of steps
 * and moves the robot along the selected path.
 */
void backHome() {
    currentStateBackHome = STATE_FIND_SEGMENT;  // Initial state: search for the segment containing the current position
    int start_index = -1;  // Index of the segment where the robot currently is
    int target_index = boundary_count - 1;  // The index of the home point (0,0) in the boundary list
    int clockwise_steps = 0;  // Number of steps to reach the home point clockwise
    int counterclockwise_steps = 0;  // Number of steps to reach the home point counterclockwise
    bool clockwise = true;  // Flag indicating the selected direction (clockwise or counterclockwise)
    int index = 0;  // Index for traversing the boundary
    Point next_goal;  // The next point on the path to move to

    while (1) {
        switch (currentStateBackHome) {
            case STATE_FIND_SEGMENT:
                // Find the segment where the robot is currently located
                for (int i = 0; i < boundary_count; i++) {
                    Point p1 = boundaries[i];  // First point of the segment
                    Point p2 = boundaries[(i + 1) % boundary_count];  // Second point of the segment
                    if (isPointBetweenTwoPoints(current_position, p1, p2)) {
                        start_index = i;  // Set the index of the segment where the robot is
                        break;
                    }
                }
                currentStateBackHome = STATE_VERIFY_SEGMENT_FOUND;  // Move to the next state to verify the segment
                break;

            case STATE_VERIFY_SEGMENT_FOUND:
                // Check if the robot is on a valid segment
                if (start_index == -1) {
                    currentStateBackHome = STATE_BACK_HOME_COMPLETE;  // No valid segment found, exit the function
                } else {
                    currentStateBackHome = STATE_CALCULATE_PATH;  // Proceed to calculate the path
                }
                break;

            case STATE_CALCULATE_PATH:
                // Calculate the shortest path to the home point, both clockwise and counterclockwise
                clockwise_steps = (target_index - start_index + boundary_count) % boundary_count;
                counterclockwise_steps = (start_index - target_index + boundary_count) % boundary_count;

                // Decide the shortest path (clockwise or counterclockwise)
                clockwise = (clockwise_steps <= counterclockwise_steps);

                // Set the initial index for following the path
                if (clockwise) {
                    index = (start_index + 1) % boundary_count;  // Start moving clockwise from the next segment
                    currentStateBackHome = STATE_FOLLOW_PATH_CLOCKWISE;
                } else {
                    index = start_index;  // Start moving counterclockwise from the current segment
                    currentStateBackHome = STATE_FOLLOW_PATH_COUNTERCLOCKWISE;
                }
                break;

            case STATE_FOLLOW_PATH_CLOCKWISE:
                // Move clockwise along the boundary towards the home point (0,0)
                next_goal = boundaries[index];  // The next point on the boundary
                moveToGoal(next_goal);  // Move to the goal
                if (index == target_index) {
                    currentStateBackHome = STATE_BACK_HOME_COMPLETE;  // If the home point is reached, exit
                } else {
                    index = (index + 1) % boundary_count;  // Move to the next segment clockwise
                }
                break;

            case STATE_FOLLOW_PATH_COUNTERCLOCKWISE:
                // Move counterclockwise along the boundary towards the home point (0,0)
                next_goal = boundaries[index];  // The next point on the boundary
                moveToGoal(next_goal);  // Move to the goal
                if (index == target_index) {
                    currentStateBackHome = STATE_BACK_HOME_COMPLETE;  // If the home point is reached, exit
                } else {
                    index = (index - 1 + boundary_count) % boundary_count;  // Move to the previous segment counterclockwise
                }
                break;

            case STATE_BACK_HOME_COMPLETE:
                // Final state: exit the function once the home point is reached
                return;

            case STATE_OBSTACLE_HOME:
                // Handle obstacle detection while returning home (not implemented in this code)
                break;

            default:
                // Emergency exit for unknown states
                return;
        }
    }
}

/**
 * @brief Checks the current state and handles obstacle detection and response during navigation.
 *
 * This function monitors the robot's state and reacts accordingly when an obstacle is detected.
 * It handles two cases: one for internal polygon scanning (when the robot is moving toward a target)
 * and another for all other cases where the robot should stop and wait for the obstacle to be cleared.
 */
void checkStates() {
    switch (state) {
        case 14:
            // Internal scanning of the polygon
            switch (currentStateDirection) {
                case STATE_MOVE_TO_TARGET:
                    // If the robot is moving towards a target and detects an obstacle
                    stop = true;  // Stop the robot
                    obstacle = false;  // Reset obstacle flag
                    currentStateMoveToGoal = STATE_OBSTACLE_MOVE;  // Change the state to handle obstacle
                    currentStateDirection = STATE_OBSTACLE_SELECT_DIRECTION;  // Change direction state to obstacle handling

                    // Log the obstacle detection and recalculation of the path
                    len = snprintf(buffer, sizeof(buffer), "Obstacle detected! Recalculating path...\n");
                    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
                    break;

                default:
                    // For other cases when an obstacle is detected
                    stop = true;  // Stop the robot
                    // Log the obstacle detection and the need to clear it
                    len = snprintf(buffer, sizeof(buffer), "Obstacle detected! Clear the obstacle\n");
                    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
                    break;
            }
            break;

        default:
            // For all other cases, stop the robot and wait for the obstacle to be cleared
            stop = true;  // Stop the robot
            // Log the obstacle detection and the need to clear it
            len = snprintf(buffer, sizeof(buffer), "Obstacle detected! Clear the obstacle\n");
            HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
            break;
    }
}


/* USER CODE END 0 */

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
	  		  // move the machine to coordinate (0,0)
	  		  moveToGoal(zero);
	  		  state = 2;
	  		  currentStateMoveToGoal = STATE_CALCULATE_MOVEMENT;
	  		  break;
	      case 2:
	    	  //turn forward

	    	  // calculate steps between current angle and PI/2 (forward rotation)
	    	  float angle = M_PI/2 - current_angle;
	    	  // Normalize between -pi e pi
	    	  if (angle > M_PI){
	    		  angle -= 2 * M_PI;
	    	  }
			  else if (angle < -M_PI){
				  angle += 2 * M_PI;
			  }

	    	  rotation_steps = calcolaNumeroPassiRotazione(angle);

	    	  if (rotation_steps < 0) {
	    		  move_right(0, labs(rotation_steps));  // turn right
	    	  } else if (rotation_steps > 0) {
	    		  move_left(0, labs(rotation_steps));  // turn left
	    	  }
	    	  state = 3;
	          break;
	      case 3:
	    	  // wait until the machine is complete rotated and be ready for the next comand
	    	  if(moved){
	    		  current_angle = M_PI/2;
	    		  add_boundary_point(current_position.x, current_position.y);
		    	  HAL_GPIO_WritePin(GPIOE, ENABLE_LEFT_Pin, GPIO_PIN_SET);
		    	  HAL_GPIO_WritePin(GPIOE, ENABLE_RIGHT_Pin, GPIO_PIN_SET);
	    		  state = 0;
	    	  }
	    	  break;
	      case 10:
	    	  // follow the perimeter
			  followPerimeter(NULL, true);
			  message = "Perimeter concluded!\n";
	    	  HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
	    	  state = 11;
	    	  break;
	      case 11:
	    	  // creation of the polygon map
	    	  if(createNodes()){
	    		  state = 12;
	    	  }else{
	    		  state = 0;
	    	  }
	    	  break;
	      case 12:
	    	  // scan the internal area
	    	  message = "Scan:\n";
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
	    	  // find and go to the initial point to scan the entire polygon
    		  findNode(current_position, &startRow, &startColumn);
    		  len = snprintf(buffer, sizeof(buffer), "Track: (%d, %d) -> (%d, %d)\n", startColumn, startRow, scanColumns[0], scanRows[0]);
    		  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    		  Node* currentNode = NULL;
    		  if(scanRows[0] != startRow || scanColumns[0] != startColumn){
        		  currentNode = findShortestPathAStar(scanRows[0], scanColumns[0], startRow, startColumn);
    		  }

    		  if(currentNode == NULL){
    			  // if don't find a path to the initial point, move using the perimeter to reach the point
    			  len = snprintf(buffer, sizeof(buffer), "Unreachable node, path to: (%d, %d)\n", scanColumns[0], scanRows[0]);
    			  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    			  Node* wallNode = findNearestWall(scanRows[0], scanColumns[0]);

    			  if(wallNode != NULL){
    				  len = snprintf(buffer, sizeof(buffer), "going to (%d, %d)\n", wallNode->column, wallNode->row);
    				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    				  double resultX, resultY = 0.0;
    				  if(findPositionOnSegment(wallNode->row, wallNode->column, &resultX, &resultY)){
    					  len = snprintf(buffer, sizeof(buffer), "position: (%d, %d)\n", (int) round(resultX), (int) round(resultY));
    					  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    					  Point target = {round(resultX), round(resultY)};

    					  bool clockwise = false;
    					  shortestDirection(target, &clockwise);

    					  followPerimeter(&target, clockwise);

    			    	  moveToGoal(target);
    				  }else{
    					  message = "point found outside the polygon, error!\n";
    					  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    					  state = 0;
    					  break;
    				  }

    				  //at this point I am at the entry point of the polygon, I go towards the starting point of the scan
    				  wallNode = wallNode->parent;

    				  previousDir = -2; // Initial direction non-existent

    				  while (wallNode) {
    					  dir = wallNode->direction;

    					  selectDirection(previousDir, dir, wallNode);

    					  wallNode = wallNode->parent;

    					  previousDir = dir;
    				  }
    				  index++;

    			  }else{
    				  //didn't find a path to get to the starting point, go to state 0
    				  message = "I have no way to enter the range, change map!\n";
    				  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    				  state = 0;
    				  break;
    			  }
    		  }
    		  //I found a path to get to the starting point, so I'll go there
    		  state = 14;
	    	  break;
	      case 14:
	    	  switch (stateScan) {
	    	  case STATE_NORMAL:
	    		  if (currentNode == NULL) {
	    			  // Find the starting node
	    			  findNode(current_position, &startRow, &startColumn);
	    			  // Search for the first point to start the scan
	    			  len = snprintf(buffer, sizeof(buffer),
	    					  "Track: (%d, %d) -> (%d, %d)\n",
							  startColumn, startRow, scanColumns[index], scanRows[index]);
	    			  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	    			  // Find the shortest path to the scan point
	        		  if(scanRows[index] != startRow || scanColumns[index] != startColumn){
	        			  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index],startRow, startColumn);
	        		  }
	    		  }
	    		  previousDir = -2;
	    		  // Follow the path to the current point
	    		  while (currentNode) {
	    			  dir = currentNode->direction;
	    			  selectDirection(previousDir, dir, currentNode);

	    			  // Check if an obstacle has been detected
	    			  if (stateScan == STATE_OBSTACLE) {
	    				  break;
	    			  }

	    			  currentNode = currentNode->parent;
	    			  previousDir = dir;
	    		  }

	    		  if (stateScan != STATE_OBSTACLE) {
	    			  // Move to the next point if there is no obstacle
	    			  index++;
	    			  if (index == scanNumber) {
	    				  // Scan finished, go home
	    				  stateScan = STATE_FINISHED;
	    			  }
	    		  }
	    		  break;
	    	  case STATE_OBSTACLE:
	    		  // Recalculate the route with A*
	    		  findNode(current_position, &startRow, &startColumn);
	    		  int obstacleRow = 0;
	    		  int obstacleColumn = 0;

	    		  if(previousDir == -2){
	    			  if(dir == 0){
	    				  //I have an obstacle above me
	    				  nodes[startRow+1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow+1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(dir == 1){
	    				  //I have an obstacle under me
	    				  nodes[startRow-1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow-1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(dir == 2){
	    				  //I have an obstacle on the right
	    				  nodes[startRow][startColumn + 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn+1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(dir == 3){
	    				  //I have an obstacle on the left
	    				  nodes[startRow][startColumn - 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn - 1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }
	    		  }else{
	    			  if(previousDir == 0){
	    				  //I have an obstacle above me
	    				  nodes[startRow+1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow+1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(previousDir == 1){
	    				  //I have an obstacle under me
	    				  nodes[startRow-1][startColumn].isObstacle = true;
	    				  obstacleRow = startRow-1;
	    				  obstacleColumn = startColumn;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(previousDir == 2){
	    				  //I have an obstacle on the right
	    				  nodes[startRow][startColumn + 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn+1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }else if(previousDir == 3){
	    				  //I have an obstacle on the left
	    				  nodes[startRow][startColumn - 1].isObstacle = true;
	    				  obstacleRow = startRow;
	    				  obstacleColumn = startColumn-1;
	    				  currentNode = findShortestPathAStar(scanRows[index], scanColumns[index], startRow, startColumn);
	    			  }
	    		  }

				  len = snprintf(buffer, sizeof(buffer), "position: (%d, %d), obstacle: (%d, %d)\n", startColumn, startRow, obstacleColumn, obstacleRow);
				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	    		  stateScan = STATE_NORMAL;
	    		  break;

	    	  case STATE_FINISHED:
	    		  // Scan completed
	    		  state = 15;
	    		  break;

	    	  default:
	    		  len = snprintf(buffer, sizeof(buffer), "Error: State not recognized!\n");
	    		  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	    		  break;
	    	  }
	    	  break;
	      case 15:
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

				  previousDir = -2; // inesistent initial direction
				  if(index_wall > 1){
					  for(int i = index_wall-1; i>1; i--){
						  toWall = results[i];
						  len = snprintf(buffer, sizeof(buffer), "going to: (%d, %d)\n", toWall->column, toWall->row);
						  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

						  dir = toWall->direction;

						  selectDirection(previousDir, dir, toWall);

						  toWall = toWall->parent;

						  previousDir = dir;
					  }
				  }


				  toWall = results[0];

				  // going in the perimeter
				  len = snprintf(buffer, sizeof(buffer), "going to: (%d, %d)\n", toWall->column, toWall->row);
				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	    	  }else{
	    		  toWall = &nodes[startRow][startColumn];
	    	  }
			  double resultX, resultY = 0.0;
			  if(findPositionOnSegment(toWall->row, toWall->column, &resultX, &resultY)){
				  len = snprintf(buffer, sizeof(buffer), "poisition: (%d, %d)\n", (int) round(resultX), (int) round(resultY));
				  HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, HAL_MAX_DELAY);

				  //fai partire la funzione per arrivare nel punto preciso seguendo il perimetro
				  Point target = {round(resultX), round(resultY)};
				  moveToGoal(target);

				  backHome();
				  message = "back Home!\n";
				  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
			  }else{
				  //outside the polygon, go to state 0
				  message = "point found outside the polygon, error!\n";
				  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
			  }
			  freeNodes();
			  state = 0;
	    	  break;
	      default:
	    	  // code to execute if no cases match
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
