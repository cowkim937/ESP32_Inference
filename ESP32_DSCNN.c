#include "ESP32_DSCNN.h"

int debug_mode = __TEST_OFF__;

int max_value;
int ans_index;

int con[DSCNN_CLASS];

char class_label[17][64]={
	"Straight",
	"Left Curve",
	"Right Curve",
	"Left Turn",
	"Right Turn",
	"Left Change Lane",
	"Right Change Lane",
	"Stop",
	"U-Turn",
	"Sudden Left Turn",
	"Sudden Right Turn",
	"Sudden Left Change Lane",
	"Sudden Right Change Lane",
	"Weaving",
	"Left Jerking",
	"Right Jerking",
	"Sudden U-Turn"
};

int input_test = 0;

dl_matrix3dq_t *input_matrix3dq = NULL;
dl_matrix3dq_t *filter_matrix3dq = NULL;
dl_matrix3dq_t *buffer_matrix3dq = NULL;
dl_matrix3dq_t *bias_matrix3dq = NULL;
dl_matrix3dq_t *output_matrix3dq = NULL;

void DSCNN_init(){
    memset(con, 0, sizeof(int) * DSCNN_CLASS);
}

void setinput(int inputs){
    input_test = inputs;
}

int dangerous_DLmodel8(int16_t *nor_data_inputs, driver_behaivor driver_data){
    char *debug_mode = NULL;
    
    int windowsize = driver_data.window_size;
    int sensors = driver_data.sensors;
    int padding = 2;
    int strider = 0;

    // long long timer = esp_timer_get_time();

    // input
	input_matrix3dq = dl_matrix3dq_alloc(1, (sensors + padding), (windowsize + padding), 1, STATIC_EXPONENT);
	memset(input_matrix3dq->item, 0, sizeof(qtp_t) * (sensors + padding) * (windowsize + padding));

    for(int inp=0; inp < windowsize; inp++){
        memcpy(input_matrix3dq->item + ((inp + 1) * (sensors + padding) + 1),
              nor_data_inputs + (inp * sensors),
              sizeof(qtp_t) * sensors);
        // ESP_LOGI("INPUTDATA","%3d %3d", inp, nor_data_inputs[inp]);
    }
	
    debug_logi("dl_input", input_matrix3dq);


    // Input check log independent code
    int plog = 0;
    char logiout[1024] = {0, };

    for (int hlog = 0; hlog < input_matrix3dq->h; hlog++) {
        memset(logiout, 0, sizeof(char) * 1024);
        sprintf(logiout, "%3d %15s ", hlog, "NorInput");
        for (int wlog = 0; wlog < input_matrix3dq->w; wlog++) {
            sprintf(logiout, "%s %6d ", logiout, *(input_matrix3dq->item + plog + (wlog * input_matrix3dq->c)));
        }
        plog += (input_matrix3dq->w * input_matrix3dq->c);
        ESP_LOGI("INPUT", "%s", logiout);
    }


    // conv1
	filter_matrix3dq = dl_matrix3dq_alloc(conv_1_weights_0.n,
                                          conv_1_weights_0.w,
                                          conv_1_weights_0.h,
                                          conv_1_weights_0.c,
                                          conv_1_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_1_weights_0_item_array,
            sizeof(conv_1_weights_0_item_array));

	buffer_matrix3dq = dl_matrix3dq_alloc(conv_1_biases_0.n,
                                          conv_1_biases_0.w,
                                          conv_1_biases_0.h,
                                          conv_1_biases_0.c,
                                          conv_1_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_1_biases_0.n,
                                        conv_1_biases_0.w,
                                        conv_1_biases_0.h,
                                        conv_1_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_1_biases_0_item_array,
            sizeof(conv_1_biases_0_item_array));

    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);
    
	output_matrix3dq = dl_matrix3dqq_conv_3x3_with_bias_relu(input_matrix3dq,
                                                            filter_matrix3dq,
                                                            bias_matrix3dq,
                                                            dscnn_model_info[strider][0],
                                                            dscnn_model_info[strider][1],
                                                            PADDING_VALID,
                                                            STATIC_EXPONENT,
                                                            debug_mode);

    windowsize = stride_reshaper(windowsize, 0, strider);
    sensors = stride_reshaper(sensors, 0, strider);
    strider++;

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv1_out", output_matrix3dq);

	// conv2_ds
	input_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_dw_conv_depthwise_weights_0.n,
                                        (sensors + padding),
                                        (windowsize + padding),
                                        conv_ds_1_dw_conv_depthwise_weights_0.c,
                                        STATIC_EXPONENT);

	memset(input_matrix3dq->item,
            0,
            sizeof(qtp_t) *
            (sensors + padding) *
            (windowsize + padding) *
            conv_ds_1_dw_conv_depthwise_weights_0.c);

    for(int inp=0; inp < windowsize; inp++)
       memcpy(input_matrix3dq->item + (inp * (sensors + padding) * conv_ds_1_dw_conv_depthwise_weights_0.c),
              output_matrix3dq->item + (inp * sensors * conv_ds_1_dw_conv_depthwise_weights_0.c),
              sizeof(qtp_t) * sensors * conv_ds_1_dw_conv_depthwise_weights_0.c);
    
    debug_logi("conv2_dw_in", output_matrix3dq);

	dl_matrix3dq_free(output_matrix3dq);

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_dw_conv_depthwise_weights_0.n,
                                          conv_ds_1_dw_conv_depthwise_weights_0.w,
                                          conv_ds_1_dw_conv_depthwise_weights_0.h,
                                          conv_ds_1_dw_conv_depthwise_weights_0.c,
                                          conv_ds_1_dw_conv_depthwise_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_1_dw_conv_depthwise_weights_0_item_array,
            sizeof(conv_ds_1_dw_conv_depthwise_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_dw_conv_biases_0.n,
                                          conv_ds_1_dw_conv_biases_0.w,
                                          conv_ds_1_dw_conv_biases_0.h,
                                          conv_ds_1_dw_conv_biases_0.c,
                                          conv_ds_1_dw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_dw_conv_biases_0.n,
                                        conv_ds_1_dw_conv_biases_0.w,
                                        conv_ds_1_dw_conv_biases_0.h,
                                        conv_ds_1_dw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_1_dw_conv_biases_0_item_array,
            sizeof(conv_ds_1_dw_conv_biases_0_item_array));
    
    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);
    
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq,
                                                                filter_matrix3dq,
                                                                bias_matrix3dq,
                                                                dscnn_model_info[strider][0],
                                                                dscnn_model_info[strider][1],
                                                                PADDING_VALID,
                                                                STATIC_EXPONENT,
                                                                ReLuSTATE,
                                                                debug_mode);

    windowsize = stride_reshaper(windowsize, 0, strider);
    sensors = stride_reshaper(sensors, 0, strider);
    strider++;

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv2_dw_out", output_matrix3dq);

	// conv2_pw
	input_matrix3dq = output_matrix3dq;

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_pw_conv_weights_0.n,
                                          conv_ds_1_pw_conv_weights_0.w,
                                          conv_ds_1_pw_conv_weights_0.h,
                                          conv_ds_1_pw_conv_weights_0.c,
                                          conv_ds_1_pw_conv_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_1_pw_conv_weights_0_item_array,
            sizeof(conv_ds_1_pw_conv_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_pw_conv_biases_0.n,
                                          conv_ds_1_pw_conv_biases_0.w,
                                          conv_ds_1_pw_conv_biases_0.h,
                                          conv_ds_1_pw_conv_biases_0.c,
                                          conv_ds_1_pw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_pw_conv_biases_0.n,
                                        conv_ds_1_pw_conv_biases_0.w,
                                        conv_ds_1_pw_conv_biases_0.h,
                                        conv_ds_1_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_1_pw_conv_biases_0_item_array,
            sizeof(conv_ds_1_pw_conv_biases_0_item_array));

    // for(int w=0;w<conv_ds_1_pw_conv_biases_0.w;w++){
    //     for(int h=0;h<conv_ds_1_pw_conv_biases_0.h;h++){
    //         *(buffer_matrix3dq->item + (w + h) * conv_ds_1_pw_conv_biases_0.c) = 
    //             *(buffer_matrix3dq->item + (w + h) * conv_ds_1_pw_conv_biases_0.c)/8;
    //     }
    // }

    // *(buffer_matrix3dq->item + sizeof(qtp_t) * conv_ds_1_pw_conv_biases_0.c) = 
    //     *(buffer_matrix3dq->item + sizeof(qtp_t) * conv_ds_1_pw_conv_biases_0.c)/8;

    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);

	output_matrix3dq = dl_matrix3dq_alloc(conv_ds_1_pw_conv_biases_0.n,
                                        sensors,
                                        windowsize,
                                        conv_ds_1_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);

	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq,
                                            input_matrix3dq,
                                            filter_matrix3dq,
                                            bias_matrix3dq,
                                            DL_XTENSA_IMPL,
                                            debug_mode);

    debug_logi("conv2_pw_out", output_matrix3dq);

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
    dl_matrix3dq_free(buffer_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

	// conv3_ds
	input_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_dw_conv_depthwise_weights_0.n,
                                        (sensors + padding - 1),
                                        (windowsize + padding),
                                        conv_ds_2_dw_conv_depthwise_weights_0.c,
                                        STATIC_EXPONENT);

	memset(input_matrix3dq->item,
            0,
            sizeof(qtp_t) *
            (sensors + padding - 1) *
            (windowsize + padding) *
            conv_ds_2_dw_conv_depthwise_weights_0.c);

    for(int inp=0; inp < windowsize; inp++)
       memcpy(input_matrix3dq->item + ((inp + 1) * (sensors + padding - 1) * conv_ds_2_dw_conv_depthwise_weights_0.c),
              output_matrix3dq->item + (inp * sensors * conv_ds_2_dw_conv_depthwise_weights_0.c),
              sizeof(qtp_t) * sensors * conv_ds_2_dw_conv_depthwise_weights_0.c);

    debug_logi("conv3_dw_in", input_matrix3dq);

	dl_matrix3dq_free(output_matrix3dq);

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_dw_conv_depthwise_weights_0.n,
                                          conv_ds_2_dw_conv_depthwise_weights_0.w,
                                          conv_ds_2_dw_conv_depthwise_weights_0.h,
                                          conv_ds_2_dw_conv_depthwise_weights_0.c,
                                          conv_ds_2_dw_conv_depthwise_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_2_dw_conv_depthwise_weights_0_item_array,
            sizeof(conv_ds_2_dw_conv_depthwise_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_dw_conv_biases_0.n,
                                          conv_ds_2_dw_conv_biases_0.w,
                                          conv_ds_2_dw_conv_biases_0.h,
                                          conv_ds_2_dw_conv_biases_0.c,
                                          conv_ds_2_dw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_dw_conv_biases_0.n,
                                        conv_ds_2_dw_conv_biases_0.w,
                                        conv_ds_2_dw_conv_biases_0.h,
                                        conv_ds_2_dw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_2_dw_conv_biases_0_item_array,
            sizeof(conv_ds_2_dw_conv_biases_0_item_array));
    
    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);
    
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq,
                                                                filter_matrix3dq,
                                                                bias_matrix3dq,
                                                                dscnn_model_info[strider][0],
                                                                dscnn_model_info[strider][1],
                                                                PADDING_VALID,
                                                                STATIC_EXPONENT,
                                                                ReLuSTATE,
                                                                debug_mode);
    
    windowsize = stride_reshaper(windowsize, 0, strider);
    sensors = stride_reshaper(sensors, 0, strider);
    strider++;

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv3_dw_out", output_matrix3dq);

	// conv3_pw
	input_matrix3dq = output_matrix3dq;

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_pw_conv_weights_0.n,
                                          conv_ds_2_pw_conv_weights_0.w,
                                          conv_ds_2_pw_conv_weights_0.h,
                                          conv_ds_2_pw_conv_weights_0.c,
                                          conv_ds_2_pw_conv_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_2_pw_conv_weights_0_item_array,
            sizeof(conv_ds_2_pw_conv_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_pw_conv_biases_0.n,
                                          conv_ds_2_pw_conv_biases_0.w,
                                          conv_ds_2_pw_conv_biases_0.h,
                                          conv_ds_2_pw_conv_biases_0.c,
                                          conv_ds_2_pw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_pw_conv_biases_0.n,
                                        conv_ds_2_pw_conv_biases_0.w,
                                        conv_ds_2_pw_conv_biases_0.h,
                                        conv_ds_2_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_2_pw_conv_biases_0_item_array,
            sizeof(conv_ds_2_pw_conv_biases_0_item_array));

    // for(int w=0;w<sizeof(conv_ds_2_pw_conv_biases_0_item_array)/sizeof(qtp_t);w++){
    //         *(buffer_matrix3dq->item + w * conv_ds_2_pw_conv_biases_0.c) = 
    //             *(buffer_matrix3dq->item + w * conv_ds_2_pw_conv_biases_0.c)/8;
    // }

    debug_logi("conv3_pw_bias", buffer_matrix3dq);

    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);

	output_matrix3dq = dl_matrix3dq_alloc(conv_ds_2_pw_conv_biases_0.n,
                                        sensors,
                                        windowsize,
                                        conv_ds_2_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);

	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq,
                                            input_matrix3dq,
                                            filter_matrix3dq,
                                            bias_matrix3dq,
                                            DL_XTENSA_IMPL,
                                            debug_mode);

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
    dl_matrix3dq_free(buffer_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv3_pw_out", output_matrix3dq);

	// conv4_ds
	input_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_dw_conv_depthwise_weights_0.n,
                                        (sensors + padding),
                                        (windowsize + padding),
                                        conv_ds_3_dw_conv_depthwise_weights_0.c,
                                        STATIC_EXPONENT);
	memset(input_matrix3dq->item,
            0,
            sizeof(qtp_t) *
            conv_ds_3_dw_conv_depthwise_weights_0.n *
            (sensors + padding) *
            (windowsize + padding) *
            conv_ds_3_dw_conv_depthwise_weights_0.c);

    for(int inp=0; inp < windowsize; inp++)
        memcpy(input_matrix3dq->item + ((inp + 1) * (sensors + padding) * conv_ds_3_dw_conv_depthwise_weights_0.c + conv_ds_3_dw_conv_depthwise_weights_0.c),
              output_matrix3dq->item + (inp * sensors * conv_ds_3_dw_conv_depthwise_weights_0.c),
              sizeof(qtp_t) * sensors * conv_ds_3_dw_conv_depthwise_weights_0.c);

    debug_logi("conv4_dw_in", input_matrix3dq);

	dl_matrix3dq_free(output_matrix3dq);

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_dw_conv_depthwise_weights_0.n,
                                          conv_ds_3_dw_conv_depthwise_weights_0.w,
                                          conv_ds_3_dw_conv_depthwise_weights_0.h,
                                          conv_ds_3_dw_conv_depthwise_weights_0.c,
                                          conv_ds_3_dw_conv_depthwise_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_3_dw_conv_depthwise_weights_0_item_array,
            sizeof(conv_ds_3_dw_conv_depthwise_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_dw_conv_biases_0.n,
                                          conv_ds_3_dw_conv_biases_0.w,
                                          conv_ds_3_dw_conv_biases_0.h,
                                          conv_ds_3_dw_conv_biases_0.c,
                                          conv_ds_3_dw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_dw_conv_biases_0.n,
                                        conv_ds_3_dw_conv_biases_0.w,
                                        conv_ds_3_dw_conv_biases_0.h,
                                        conv_ds_3_dw_conv_biases_0.c,
                                        STATIC_EXPONENT);

	memcpy(buffer_matrix3dq->item,
            &conv_ds_3_dw_conv_biases_0_item_array,
            sizeof(conv_ds_3_dw_conv_biases_0_item_array));
    
    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);
    
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq,
                                                                filter_matrix3dq,
                                                                bias_matrix3dq,
                                                                dscnn_model_info[strider][0],
                                                                dscnn_model_info[strider][1],
                                                                PADDING_VALID,
                                                                STATIC_EXPONENT,
                                                                ReLuSTATE,
                                                                debug_mode);

    windowsize = stride_reshaper(windowsize, 0, strider);
    sensors = stride_reshaper(sensors, 0, strider);
    strider++;

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv4_dw_out", output_matrix3dq);

	// conv4_pw
	input_matrix3dq = output_matrix3dq;

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_pw_conv_weights_0.n,
                                          conv_ds_3_pw_conv_weights_0.w,
                                          conv_ds_3_pw_conv_weights_0.h,
                                          conv_ds_3_pw_conv_weights_0.c,
                                          conv_ds_3_pw_conv_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_3_pw_conv_weights_0_item_array,
            sizeof(conv_ds_3_pw_conv_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_pw_conv_biases_0.n,
                                          conv_ds_3_pw_conv_biases_0.w,
                                          conv_ds_3_pw_conv_biases_0.h,
                                          conv_ds_3_pw_conv_biases_0.c,
                                          conv_ds_3_pw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_pw_conv_biases_0.n,
                                        conv_ds_3_pw_conv_biases_0.w,
                                        conv_ds_3_pw_conv_biases_0.h,
                                        conv_ds_3_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_3_pw_conv_biases_0_item_array,
            sizeof(conv_ds_3_pw_conv_biases_0_item_array));

    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);

	output_matrix3dq = dl_matrix3dq_alloc(conv_ds_3_pw_conv_biases_0.n,
                                        sensors,
                                        windowsize,
                                        conv_ds_3_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);

	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq,
                                            input_matrix3dq,
                                            filter_matrix3dq,
                                            bias_matrix3dq,
                                            DL_XTENSA_IMPL,
                                            debug_mode);

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv4_pw_out", output_matrix3dq);

	// conv5_ds
	input_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_dw_conv_depthwise_weights_0.n,
                                        (sensors + padding),
                                        (windowsize + padding),
                                        conv_ds_4_dw_conv_depthwise_weights_0.c,
                                        STATIC_EXPONENT);

	memset(input_matrix3dq->item,
            0,
            sizeof(qtp_t) *
            (sensors + padding) *
            (windowsize + padding) *
            conv_ds_4_dw_conv_depthwise_weights_0.c);

    for(int inp=0; inp < windowsize; inp++){
        memcpy(input_matrix3dq->item + ((inp + 1) * (sensors + padding) * conv_ds_4_dw_conv_depthwise_weights_0.c + conv_ds_4_dw_conv_depthwise_weights_0.c),
              output_matrix3dq->item + (inp * sensors * conv_ds_4_dw_conv_depthwise_weights_0.c),
              sizeof(qtp_t) * sensors * conv_ds_4_dw_conv_depthwise_weights_0.c);
    }

    debug_logi("conv5_dw_in", input_matrix3dq);

	dl_matrix3dq_free(output_matrix3dq);

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_dw_conv_depthwise_weights_0.n,
                                          conv_ds_4_dw_conv_depthwise_weights_0.w,
                                          conv_ds_4_dw_conv_depthwise_weights_0.h,
                                          conv_ds_4_dw_conv_depthwise_weights_0.c,
                                          conv_ds_4_dw_conv_depthwise_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_4_dw_conv_depthwise_weights_0_item_array,
            sizeof(conv_ds_4_dw_conv_depthwise_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_dw_conv_biases_0.n,
                                          conv_ds_4_dw_conv_biases_0.w,
                                          conv_ds_4_dw_conv_biases_0.h,
                                          conv_ds_4_dw_conv_biases_0.c,
                                          conv_ds_4_dw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_dw_conv_biases_0.n,
                                        conv_ds_4_dw_conv_biases_0.w,
                                        conv_ds_4_dw_conv_biases_0.h,
                                        conv_ds_4_dw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_4_dw_conv_biases_0_item_array,
            sizeof(conv_ds_4_dw_conv_biases_0_item_array));
    
    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);

	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq,
                                                                filter_matrix3dq,
                                                                bias_matrix3dq,
                                                                dscnn_model_info[strider][0],
                                                                dscnn_model_info[strider][1],
                                                                PADDING_VALID,
                                                                STATIC_EXPONENT,
                                                                ReLuSTATE,
                                                                debug_mode);

    windowsize = stride_reshaper(windowsize, 0, strider);
    sensors = stride_reshaper(sensors, 0, strider);
    strider++;

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv5_dw_out", output_matrix3dq);

	// conv5_pw
	input_matrix3dq = output_matrix3dq;

	filter_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_pw_conv_weights_0.n,
                                          conv_ds_4_pw_conv_weights_0.w,
                                          conv_ds_4_pw_conv_weights_0.h,
                                          conv_ds_4_pw_conv_weights_0.c,
                                          conv_ds_4_pw_conv_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &conv_ds_4_pw_conv_weights_0_item_array,
            sizeof(conv_ds_4_pw_conv_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_pw_conv_biases_0.n,
                                          conv_ds_4_pw_conv_biases_0.w,
                                          conv_ds_4_pw_conv_biases_0.h,
                                          conv_ds_4_pw_conv_biases_0.c,
                                          conv_ds_4_pw_conv_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_pw_conv_biases_0.n,
                                        conv_ds_4_pw_conv_biases_0.w,
                                        conv_ds_4_pw_conv_biases_0.h,
                                        conv_ds_4_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &conv_ds_4_pw_conv_biases_0_item_array,
            sizeof(conv_ds_4_pw_conv_biases_0_item_array));
    
    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);

	output_matrix3dq = dl_matrix3dq_alloc(conv_ds_4_pw_conv_biases_0.n,
                                        sensors,
                                        windowsize,
                                        conv_ds_4_pw_conv_biases_0.c,
                                        STATIC_EXPONENT);

	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq,
                                            input_matrix3dq,
                                            filter_matrix3dq,
                                            bias_matrix3dq,
                                            DL_XTENSA_IMPL,
                                            debug_mode);

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);

    debug_logi("conv5_pw_out", output_matrix3dq);

	// global pool
	input_matrix3dq = dl_matrix3dq_global_pool(output_matrix3dq);
	dl_matrix3dq_free(output_matrix3dq);

    debug_logi("GPooling", input_matrix3dq);

	// fc
	filter_matrix3dq = dl_matrix3dq_alloc(fc1_weights_0.w,
                                          fc1_weights_0.n,
                                          fc1_weights_0.c,
                                          fc1_weights_0.h,
                                          fc1_weights_0.exponent);

	memcpy(filter_matrix3dq->item,
            &fc1_weights_0_item_array,
            sizeof(fc1_weights_0_item_array));

    buffer_matrix3dq = dl_matrix3dq_alloc(fc1_biases_0.n,
                                          fc1_biases_0.w,
                                          fc1_biases_0.h,
                                          fc1_biases_0.c,
                                          fc1_biases_0.exponent);

	bias_matrix3dq = dl_matrix3dq_alloc(fc1_biases_0.n,
                                        fc1_biases_0.w,
                                        fc1_biases_0.h,
                                        fc1_biases_0.c,
                                        STATIC_EXPONENT);
                                        
	memcpy(buffer_matrix3dq->item,
            &fc1_biases_0_item_array,
            sizeof(fc1_biases_0_item_array));

    dl_matrix3dq_shift_exponent(bias_matrix3dq, buffer_matrix3dq, STATIC_EXPONENT);
    dl_matrix3dq_free(buffer_matrix3dq);

	output_matrix3dq = dl_matrix3dq_alloc(fc1_biases_0.n,
                                          fc1_biases_0.w,
                                          fc1_biases_0.h,
                                          fc1_biases_0.c,
                                          STATIC_EXPONENT);

	dl_matrix3dqq_fc_with_bias(output_matrix3dq,
                                input_matrix3dq,
                                filter_matrix3dq,
                                bias_matrix3dq,
                                DL_XTENSA_IMPL,
                                debug_mode);

	dl_matrix3dq_free(input_matrix3dq);
	dl_matrix3dq_free(filter_matrix3dq);
	dl_matrix3dq_free(bias_matrix3dq);
    
    debug_logi("FUlly", output_matrix3dq);

    // result
    max_value = output_matrix3dq->item[0];
    ans_index = 0;
	for(int find_result=1; find_result < fc1_biases_0.c; find_result++){
		int vlaue = *(output_matrix3dq->item + find_result);
        if(vlaue > max_value){
			max_value = vlaue;
			ans_index = find_result;
		}
	}

    ESP_LOGI("DSCNN RESULT","TEST INPUT: %s RESULT: %s", class_label[input_test], class_label[ans_index]);
    // if(debug_mode == __TEST_ON__){
    //     ESP_LOGI("DSCNN RESULT","TEST INPUT: %2d RESULT: %2d", input_test, ans_index);
    // }

	dl_matrix3dq_free(output_matrix3dq);

    // if(ans_index >= 8)
    //     ans_index++;

    // ESP_LOGI("HOW LONG", "%lld us", esp_timer_get_time() - timer);

	return ans_index;
}

void debug_logi(char *layer, dl_matrix3dq_t *check_matrix3dq){
    if(debug_mode == __TEST_ON__){
        ESP_LOGI("DSCNN_SET", "%15s %3d %3d %3d %3d %3d",
            layer,
            check_matrix3dq->w,
            check_matrix3dq->h,
            check_matrix3dq->c,
            check_matrix3dq->n,
            check_matrix3dq->exponent);

        int p = 0;
        char logiout[1024];
        
        memset(logiout, 0, sizeof(char) * 1024);

        for (int h = 0; h < check_matrix3dq->h; h++) {
            memset(logiout, 0, sizeof(char) * 1024);
            sprintf(logiout, "%3d %15s ", h, layer);
            for (int w = 0; w < check_matrix3dq->w; w++) {
                sprintf(logiout, "%s %6d ", logiout, *(check_matrix3dq->item + p + (w * check_matrix3dq->c)));
            }
            p += (check_matrix3dq->w * check_matrix3dq->c);
            ESP_LOGI("DSCNN", "%s", logiout);
        }
    }
}

int stride_reshaper(int size, int wh, int striding){
    return ceil((double)size/(double)dscnn_model_info[striding][wh]);
}