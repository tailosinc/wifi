/**
 * @file     sensor_data.h
 * @version  1.0
 * @date     2014-April-09
 *
 * Copyright(C) Redpine Signals 2014
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief Example usage for Application data to displayed over webpage
 * 
 * @section Description
 * This file contains the definitions of data structures & APIs used to model some
 * sample application data that can be stored in the module & displayed embedded in
 * a webpage. This file also utilizes the JSON parser to convert to & from JSON
 * representation of data.
 *
 *
 */


/**
 * Includes
 */
#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "rsi_global.h"
#include "rsi_api.h"

/* User Data Structure */
typedef struct app_data_s {
    int  temperature;
    int  accelerometer_x;
    int  accelerometer_y;
    int  accelerometer_z;
    int  checkbox_1;
    int  checkbox_2;
    int  radio_selection;
} app_data_t;

/**
 * Global Variables
 */
extern app_data_t sensor_data;

/**
 * Function declarations
 */

/* This function is used to convert the structure to JSON string form */
uint8* sensor_data_stringify(uint8* json, app_data_t* sensor_data);


/* This function is used to initialize the structure with default values
   These values can be retrieved from sensors or something similar
 */
void sensor_data_init(app_data_t* sensor_data);


/* This function is used to update the sensor data structure with the updates
   received from the browser.
 */
uint8 sensor_data_update(app_data_t* sensor_data, uint8* json);


/* Helper function declarations */

/* This function is used to extract filename out of the received json update data. */
uint8* json_extract_filename(uint8* json, uint8* buffer);

/* Helper function which actually performs the update */
void sensor_data_update_helper(app_data_t* sensor_data, uint8* json);

int is_int(uint8 c);
int is_float(uint8 c);

void json_extract_float(uint8* json, uint8* key, float* val);
void json_extract_int(uint8* json, uint8* key, int* val);
void json_extract_boolean(uint8* json, uint8* key, int* val);


#endif