/*
 * ESP32_DSCNN.h
 *
 *  Created on: 2020. 11. 02.
 *      Author: CW_KIM 
 */
#ifndef COMPONENTS_ESP32_DSCNN
#define COMPONENTS_ESP32_DSCNN

#define DL_VER 2

#include <math.h>

#include "dl_lib_matrix3dq.h"
#include "dl_lib_matrix3d.h"

#include "8x30_ds_cnn_inputs.h"
#include "8x30_ds_cnn_weights.h"

// sensor x window size
// #include "8x50_ds_cnn_inputs.h"
// #include "8x50_ds_cnn_weights.h"

#include "esp_log.h"
#include "esp_timer.h"

#define STATIC_EXPONENT -10
#define ReLuSTATE 1

#define DSCNN_MAX_SENSOR 8
#define DSCNN_MAX_WINDOW 50
#define DSCNN_CLASS 8

enum{
    __TEST_OFF__ = 0,
    __TEST_ON__,
    __TEST_ON_BLE__,
    __TEST_ON_BLE_RAW__,
    __TEST_ON_CANLOG__,
    __TEST_ACC__
};

float sensor_max[DSCNN_MAX_SENSOR];

typedef struct Driver_Vehicle_State_Behavior{
    int window_size;
    int sensors;
    int num_class;
} driver_behaivor;

driver_behaivor deep_driver;

/**
 * @brief detect dangerouse drving using 8 sensors
 *          using snesors
 *           - steering angle
 *           - brake pressure
 *           - vehicle speed
 *           - throttle
 *           - accelerator position
 *           - yaw rate
 *           - longitudinal rate
 *           - lateral rate
 * 
 * @param nor_data_inputs input data for inference
 * @param driver_data setting DL model data
*/
int dangerous_DLmodel8(int16_t *nor_data_inputs, driver_behaivor driver_data);

void setinput(int inputs);
void debug_logi(char *layer,dl_matrix3dq_t *check_matrix3dq);
int stride_reshaper(int size, int wh, int striding);

#endif /* COMPONENTS_ESP32_DSCNN */


