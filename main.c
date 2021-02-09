#include "main.h"

#include<stdio.h> 
#include<stdlib.h>
#include<time.h>

// BLE variable - ESP32_BLE.h
extern uint16_t spp_conn_id;
extern uint16_t spp_handle_table[SPP_IDX_NB];
extern esp_gatt_if_t spp_gatts_if;
extern bool is_connected;
extern uint8_t ble_receive_buf[BLE_BUF_SIZE];

extern int wrrite_len;

// ------- DSCNN setup -----------
int window_size = 30; // 5s
int sensors = 8;
int num_class = 17;
int time_flow = 0;
int test_mode = __TEST_OFF__;

bool accmetric = false;
float accuracy = 0;
int true_positive = 0;
int ground_truth = 0;
int row_ = 0;

float sensor_max_input[12]  = {
    460,
    100,
    120,
    120,
    120,
    120,
    120,
    100,
    100,
    1,
	1,
    22.5 
};

// -------- Sensor input -----------
int16_t NOR_SWA = 0;
int16_t NOR_BR = 0;
int16_t NOR_SP = 0;
int16_t NOR_SPFL = 0;
int16_t NOR_SPFR = 0;
int16_t NOR_SPRL = 0;
int16_t NOR_SPRR = 0;
int16_t NOR_TH = 0;
int16_t NOR_ACC = 0;
int16_t NOR_LT = 0;
int16_t NOR_LO = 0;
int16_t NOR_YAW = 0;

int16_t *nor_data_inputs;
uint8_t behavior = 0;
uint8_t danger = 0;
uint8_t result = 0;

extern driver_behaivor deep_driver;
// ---------------------------------
int DSCNN_info_transmit_period = 0;

/* ---------- CAN ---------- */
//frame buffer
obd_data_t data;
bool CAN_receive_flag = false;
CAN_frame_t __RX_frame;
CAN_device_t CAN_cfg = {
	.speed = CAN_SPEED_500KBPS,		// CAN Node baudrade
	.tx_pin_id = GPIO_NUM_4,		// CAN TX pin
	.rx_pin_id = GPIO_NUM_5,		// CAN RX pin
	.rx_queue = NULL,				// FreeRTOS queue for RX frames
};
/* --------------------------- */
TaskHandle_t xHandle_BLE_spp_init = NULL;
TaskHandle_t xHandle_BLE_receive = NULL;
TaskHandle_t xHandle_CAN_transmit = NULL;
TaskHandle_t xHandle_CAN_receive = NULL;
TaskHandle_t xHandle_DSCNN_info_periodic_transmit = NULL;

void task_DSCNN_info_periodic_transmit(void *pvParameters){
	(void)pvParameters;

	int input_rand = 1;

	ESP_LOGI("DSCNN","Start dscnn");

	deep_driver.window_size = window_size;
	deep_driver.sensors = sensors;
	deep_driver.num_class = num_class;

	nor_data_inputs = malloc(sizeof(int16_t) * window_size * sensors);  
	memset(nor_data_inputs,0,sizeof(int16_t) * window_size * sensors);

	waited_voating = malloc(sizeof(uint8_t) * window_size);
	memset(waited_voating, 0, sizeof(uint8_t) * window_size);

	int *wvoating;
	wvoating = malloc(sizeof(float) * num_class);

	int task_counter = 0;
	int time_flow = 0;

	while(1){
		vTaskDelay(DSCNN_info_transmit_period / portTICK_PERIOD_MS);
	
    	long long timer = esp_timer_get_time();

		if(task_counter >= 10){
			memmove(speed_accel + 1, speed_accel, sizeof(int) * 5);
			memmove(yaw_rate + 1, yaw_rate, sizeof(float) * 5);

			speed_accel[0] = speed;
			yaw_rate[0] = yaw;

			float s3_yaw = 0;
			float s6_yaw = 0;

			for(int i=0;i<3;i++)
				s3_yaw += yaw_rate[i];

			s6_yaw += s3_yaw;
			for(int i=3;i<6;i++)
				s6_yaw += yaw_rate[i];

			task_counter = 0;
		}
		else{
			task_counter++;
		}

		if(test_mode == __TEST_OFF__ || test_mode == __TEST_ACC__ || test_mode == __TEST_ON_CANLOG__){
			NOR_SWA = norm_calc(steering_wheel_angle, 0, _Signed_);
			NOR_BR = norm_calc(brake, 1, _Unsigned_);
			NOR_SP = norm_calc(speed, 2, _Unsigned_);
			NOR_SPFL = norm_calc(FL_speed, 3, _Unsigned_);
			NOR_SPFR = norm_calc(FR_speed, 4, _Unsigned_);
			NOR_SPRL = norm_calc(RL_speed, 5, _Unsigned_);
			NOR_SPRR = norm_calc(RR_speed, 6, _Unsigned_);
			NOR_TH = norm_calc(throtle, 7, _Unsigned_);
			NOR_ACC = norm_calc(accel, 8, _Unsigned_);
			NOR_LT = norm_calc(lateral, 9, _Signed_);
			NOR_LO = norm_calc(longitudinal, 10, _Signed_);
			NOR_YAW = norm_calc(yaw, 11, _Signed_);

			memmove(nor_data_inputs, nor_data_inputs + sensors, sizeof(int16_t) * ((window_size - 1) * sensors));

			nor_data_inputs[(window_size * sensors) - 8] = NOR_SWA;
			nor_data_inputs[(window_size * sensors) - 7] = NOR_BR;
			nor_data_inputs[(window_size * sensors) - 6] = NOR_SP;
			nor_data_inputs[(window_size * sensors) - 5] = NOR_TH;
			nor_data_inputs[(window_size * sensors) - 4] = NOR_ACC;
			nor_data_inputs[(window_size * sensors) - 3] = NOR_YAW;
			nor_data_inputs[(window_size * sensors) - 2] = NOR_LT;
			nor_data_inputs[(window_size * sensors) - 1] = NOR_LO;
		}
		else if(test_mode == __TEST_ON__){
			// input_rand = 0;
			input_rand = rand() % 15;

			// ESP_LOGI("INPUT","INPUT: %d",input_rand);
			setinput(input_rand);
			for(int w = 0; w < window_size; w++){
				nor_data_inputs[w * sensors] = input_item_array[input_rand][w * sensors];
				nor_data_inputs[w * sensors + 1] = input_item_array[input_rand][w * sensors + 1];
				nor_data_inputs[w * sensors + 2] = input_item_array[input_rand][w * sensors + 2];
				nor_data_inputs[w * sensors + 3] = input_item_array[input_rand][w * sensors + 3];
				nor_data_inputs[w * sensors + 4] = input_item_array[input_rand][w * sensors + 4];
				nor_data_inputs[w * sensors + 5] = input_item_array[input_rand][w * sensors + 5];
				nor_data_inputs[w * sensors + 6] = input_item_array[input_rand][w * sensors + 6];
				nor_data_inputs[w * sensors + 7] = input_item_array[input_rand][w * sensors + 7];				
			}
		}		

		behavior = dangerous_DLmodel8(nor_data_inputs, deep_driver);
		
		if(test_mode == __TEST_ACC__){
			setinput(ground_truth);
			row_++;
			if(behavior == ground_truth)
				true_positive++;
		}
				
		// result init
		result = 0;

		// post processing
		if(behavior == 0 || behavior == 1 || behavior == 2 || behavior == 7){
			if(speed_accel[0] - speed_accel[1] >= 10 && (speed_accel[1] <= 5 || speed_accel[2] <= 5)){
				// result = num_class + 2;	// 급출발
				result = 18;
			}
			else if(speed_accel[0] - speed_accel[1] >= 8){
				// result = num_class + 1; 	// 급가속
				result = 17;
			}
			else if(speed_accel[1] - speed_accel[0] >= 14){
				if(speed_accel[0] >= 6){
					// result = num_class + 3; // 급감속
					result = 19;
				}
				else{
					// result = num_class + 4; // 급정지
					result = 20;
				}
			}
			else{
				result = behavior;
			}
		}
		else if(behavior == 11 || behavior == 5){
			if(speed_accel[0] >= 30){
				if(abs(yaw_rate[0] - yaw_rate[1]) >= 10){
					if(speed_accel[0] - speed_accel[1] >= 3){
						// result = num_class + 5; // 좌급앞지르기
						result = 21;
					}
				}
			}
			else{
				result = behavior;
			}
		}
		else if(behavior == 12 || behavior == 6){
			if(speed_accel[0] >= 30){
				if(abs(yaw_rate[0] - yaw_rate[1]) >= 10){
					if(speed_accel[0] - speed_accel[1] >= 3){
						// result = num_class + 6; // 우급앞지르기
						result = 22;
					}
				}
			}
			else{
				result = behavior;
			}
		}
		else{
			result = behavior;
		}

		if(test_mode != __TEST_OFF__){
			if(test_mode == __TEST_ACC__)
				ESP_LOGI("HOW LONG", "%lld us ROW: %d TP: %d ACC: %.6f %%", esp_timer_get_time() - timer, row_, true_positive, ((float)true_positive/(float)row_) * 100);
			if(test_mode == __TEST_ON__)
				ESP_LOGI("HOW LONG", "%lld us", esp_timer_get_time() - timer); 
		}
	}
}

int16_t norm_calc(float data, int sensor_num, int sign){
	int16_t NORM_data = 0;
	
	// ESP_LOGI("NORM_INPUT","%.6f %d snesor %.6f",data,sensor_num,sensor_max_input[sensor_num]);

	//unsigned
	if(sign == _Unsigned_){
		if(data >= sensor_max_input[sensor_num]){
			NORM_data = 1024 * 31.75;
		}
		else if(data >= (sensor_max_input[sensor_num]/2) && data < sensor_max_input[sensor_num]){
			NORM_data = 1024 * ((data - sensor_max_input[sensor_num]/2)/(sensor_max_input[sensor_num]/2)) * 31.75;
		}
		else if(data < (sensor_max_input[sensor_num]/2) && data > 0){
			NORM_data = 1024 * ((data - sensor_max_input[sensor_num]/2)/(sensor_max_input[sensor_num]/2)) * 32;
		}
		else{
			NORM_data = 1024 * -32;
		}
	}
	else if(sign == _Signed_){
		if(data >= sensor_max_input[sensor_num]){
			NORM_data = 1024 * 31.75;
		}
		else if(data > 0 && data < sensor_max_input[sensor_num]){
			NORM_data = 1024 * (data / sensor_max_input[sensor_num]) * 31.75;
		}
		else if(data == 0){
			NORM_data = 0;
		}
		else if(data < 0 && data > (-1 * sensor_max_input[sensor_num])){
			NORM_data = 1024 * (data / sensor_max_input[sensor_num]) * 32;
		}
		else{
			NORM_data = 1024 * -32;
		}
	}
	else{
		ESP_LOGI("NORM","ERROR please select the sign");
	}

	return NORM_data;
}

void Task_init(){	
	DSCNN_info_transmit_period = 100;
	task_create();
}

void GPIO_init()
{
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO25/26
	io_conf.pin_bit_mask = ((1 << GPIO_NUM_25) | (1 << GPIO_NUM_26));
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
}

void task_delete() {
	if( xHandle_BLE_spp_init != NULL ) {
		vTaskDelete(xHandle_BLE_spp_init);
		xHandle_BLE_spp_init = NULL;
	}
	if( xHandle_BLE_receive != NULL ) {
		vTaskDelete(xHandle_BLE_receive);
		xHandle_BLE_receive = NULL;
	}
	if( xHandle_CAN_transmit != NULL ) {
		vTaskDelete(xHandle_CAN_transmit);
		xHandle_CAN_transmit = NULL;
	}
	if( xHandle_CAN_receive != NULL ) {
		vTaskDelete(xHandle_CAN_receive);
		xHandle_CAN_receive = NULL;
	}
	if( xHandle_DSCNN_info_periodic_transmit != NULL ) {
		vTaskDelete(xHandle_DSCNN_info_periodic_transmit);
		xHandle_DSCNN_info_periodic_transmit = NULL;
	}
}

void task_create() {
	xTaskCreate(&task_CAN_transmit, "CAN_transmit", 4096, NULL, 1, &xHandle_CAN_transmit);
	xTaskCreate(&task_CAN_receive, "CAN_receive", 4096, NULL, 2, &xHandle_CAN_receive);
	xTaskCreate(&spp_task, "spp_cmd_task", 4096, NULL, 3, &xHandle_BLE_spp_init);
	xTaskCreate(&task_BLE_receive, "BLE_receive", 4096, NULL, 4, &xHandle_BLE_receive);
}



void app_main(void)
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}

	ESP_ERROR_CHECK( err );

	CAN_init();
	GPIO_init();
	CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
	spp_init();
}
