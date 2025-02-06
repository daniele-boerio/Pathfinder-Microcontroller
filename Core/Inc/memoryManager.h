/*
 * memoryManager.h
 *
 *  Created on: Jan 31, 2025
 *      Author: daniele-boerio
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#ifndef SRC_MEMORYMANAGER_H_
#define SRC_MEMORYMANAGER_H_

//Constants for save and load from flash memory
#define FLASH_USER_START_ADDR   0x08040000  // Address of sector 6 in the Flash Memory
#define START_CONTROL_VALUE -1000			// Initial value to check if is there are valid points
#define END_CONTROL_VALUE 1000				// Final value to check if is there are valid points

extern char buff[50];
extern char* mex;

void save_polygon_to_flash(void);
void erase_polygon(void);
bool load_polygon_from_flash(void);
void add_boundary_point(long, long);
void transmit_coordinates(long, long);

void add_forward(unsigned long);
void add_backward(unsigned long);
void add_right(double);
void add_left(double);


#endif /* SRC_MEMORYMANAGER_H_ */
