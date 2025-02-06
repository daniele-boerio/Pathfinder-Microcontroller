/*
 * memoryManager.c
 *
 *  Created on: Jan 31, 2025
 *      Author: daniele-boerio
 */

#include "memoryManager.h"
#include "mapOperations.h"
#include "stm32f4xx_hal.h"
#include <math.h>   // Per sqrt e atan2
#include <stdbool.h>

/**
 * @brief Saves the polygon to flash memory.
 *
 * This function unlocks the Flash memory, erases the sector containing the polygon data, and writes the polygon points and control values to the Flash memory.
 * The control values ensure the integrity of the saved data and can be used for verification when loading the data.
 *
 * @note This function writes to Flash sector 6 and uses 4-byte `long` values for the polygon points and control data.
 */
void save_polygon_to_flash(void) {
    uint32_t flash_address = FLASH_USER_START_ADDR;

    // Unlock the flash
    HAL_FLASH_Unlock();

    // Erase the entire sector 6
    FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);

    for (int i = 0; i < 4; i++){
    	// Write the initial check value
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long)START_CONTROL_VALUE);
    	flash_address += sizeof(long);
    }

    // Write the vertices in the flash
    for (int i = 0; i < boundary_count; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long) boundaries[i].x);
        flash_address += sizeof(long);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long) boundaries[i].y);
        flash_address += sizeof(long);
    }

    for (int i = 0; i < 4; i++){
    	// Write the final check value
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (long)END_CONTROL_VALUE);
    	flash_address += sizeof(long);
    }

    // Block the Flash sector
    HAL_FLASH_Lock();
}

/**
 * @brief Erases the polygon from both Flash memory and RAM.
 *
 * This function unlocks the Flash memory, erases the sector containing the polygon data, and clears the `boundaries` array in RAM. It sets the boundary count to 0.
 *
 * @note This function writes to Flash sector 6.
 */
void erase_polygon(void) {

	// Unlock the flash
    HAL_FLASH_Unlock();

    // Erase the entire sector 6
    FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);

    // Block the sector 6
    HAL_FLASH_Lock();

    // Delete all the point stored in RAM
    boundary_count = 0;
    memset(boundaries, 0, sizeof(boundaries));
}

/**
 * @brief Loads the polygon from Flash memory.
 *
 * This function reads the polygon points from Flash memory, starting with the control values to ensure data integrity. It checks the control values
 * to ensure the data is valid before reading the polygon points. The function then stores the points into the `boundaries` array.
 *
 * @return `true` if the polygon was loaded successfully, `false` if the control values do not match.
 */
bool load_polygon_from_flash(void) {
    uint32_t flash_address = FLASH_USER_START_ADDR;

    // Read the initial control value
    for (int i = 0; i < 4; i++) {
        long start_value = *(uint32_t*)flash_address;
        if (start_value != START_CONTROL_VALUE) {
            return false;  // The initial control value does not match
        }
        flash_address += sizeof(long);  // Move to the next address
    }

    // Read the points from flash
    uint16_t count = 0;
    while (1) {
        // Check the final control value before saving the points
        long end_value_1, end_value_2, end_value_3, end_value_4;
        end_value_1 = *(long*)(flash_address);
        end_value_2 = *(long*)(flash_address + sizeof(long));
        end_value_3 = *(long*)(flash_address + 2 * sizeof(long));
        end_value_4 = *(long*)(flash_address + 3 * sizeof(long));

        if ((end_value_1 == END_CONTROL_VALUE) && (end_value_2 == END_CONTROL_VALUE) &&
            (end_value_3 == END_CONTROL_VALUE) && (end_value_4 == END_CONTROL_VALUE)) {
            break;  // If we encounter the final control values, stop reading
        }

        long x = *(long*)flash_address;
        flash_address += sizeof(long);
        long y = *(long*)flash_address;
        flash_address += sizeof(long);

        // Save the read point in the array
        boundaries[count].x = (double) x;
        boundaries[count].y = (double) y;
        count++;

        // Limit the maximum number of points that can be read
        if (count >= MAX_POINTS) {
            break;
        }
    }

    // Transmit the loaded points via UART
    for (int i = 0; i < boundary_count; i++) {
        // Format the current point's x and y values as a string
        int len = snprintf(buff, sizeof(buff), "Point %d: x = %ld, y = %ld\n", i, (long) boundaries[i].x, (long) boundaries[i].y);

        // Transmit the string via UART
        HAL_UART_Transmit(&huart6, (uint8_t*)buff, len, HAL_MAX_DELAY);
    }

    boundary_count = count;  // Return the number of points read
    return true;  // Data has been successfully read
}

/**
 * @brief Adds a new point to the polygon.
 *
 * This function adds a new point to the `boundaries` array, incrementing the boundary count. The point is also transmitted via UART for further processing or display.
 *
 * @param x The x-coordinate of the new point.
 * @param y The y-coordinate of the new point.
 */
void add_boundary_point(long x, long y) {
    if (boundary_count < MAX_POINTS) {
        boundaries[boundary_count].x = (double) x;
        boundaries[boundary_count].y = (double) y;
        boundary_count++;

        // Transmit the coordinates via UART
        transmit_coordinates(x, y);
    }
}

/**
 * @brief Transmits coordinates via UART.
 *
 * This function formats the given coordinates as a string and sends the string via UART to a connected device for display or processing.
 *
 * @param x The x-coordinate to transmit.
 * @param y The y-coordinate to transmit.
 */
void transmit_coordinates(long x, long y) {
    // Use angle and power as necessary
    snprintf(buff, sizeof(buff), "(%ld, %ld)\n", x, y);
    HAL_UART_Transmit(&huart6, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}

/**
 * @brief Adds a point to the polygon after moving forward by a given number of steps.
 *
 * This function calculates the new position based on the current angle and moves forward by the specified number of steps.
 * It then updates the current position and adds the new position to the polygon boundary.
 *
 * @param steps The number of steps to move forward.
 */
void add_forward(unsigned long steps) {
    // Calculate the movement based on the current angle
    double delta_x = steps * cos(current_angle);
    double delta_y = steps * sin(current_angle);

    // Update the position
    current_position.x += delta_x;
    current_position.y += delta_y;

    // Add the new position to the polygon boundary
    add_boundary_point(current_position.x, current_position.y);
}

/**
 * @brief Adds a point to the polygon after moving backward by a given number of steps.
 *
 * This function calculates the new position based on the current angle and moves backward by the specified number of steps.
 * It then updates the current position and adds the new position to the polygon boundary.
 *
 * @param steps The number of steps to move backward.
 */
void add_backward(unsigned long steps) {
    // Calculate the movement based on the current angle
    double delta_x = steps * cos(current_angle);
    double delta_y = steps * sin(current_angle);

    // Update the position (moving backward)
    current_position.x -= delta_x;
    current_position.y -= delta_y;

    // Add the new position to the polygon boundary
    add_boundary_point(current_position.x, current_position.y);
}

/**
 * @brief Modifies the current angle after turning right by a specified number of radians.
 *
 * This function adjusts the current angle by subtracting the specified number of radians, simulating a turn to the right.
 *
 * @param radiants The number of radians to turn right.
 */
void add_right(double radiants) {
    current_angle -= radiants;
}

/**
 * @brief Modifies the current angle after turning left by a specified number of radians.
 *
 * This function adjusts the current angle by adding the specified number of radians, simulating a turn to the left.
 *
 * @param radiants The number of radians to turn left.
 */
void add_left(double radiants) {
	current_angle += radiants;
}
