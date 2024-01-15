#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

const char *TAG = "ROS2 node ESP32CAM";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define STR_BUF_SIZE 512
#define CONF_OPTION_SEPARATOR ','
#define CONF_KEYVAL_SEPARATOR '='

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String config_msg;
sensor_msgs__msg__Image img_msg;


esp_err_t init_camera(void) {
    #define CAM_PIN_PWDN 32
    #define CAM_PIN_RESET -1 //software reset will be performed
    #define CAM_PIN_XCLK 0
    #define CAM_PIN_SIOD 26
    #define CAM_PIN_SIOC 27

    #define CAM_PIN_D7 35
    #define CAM_PIN_D6 34
    #define CAM_PIN_D5 39
    #define CAM_PIN_D4 36
    #define CAM_PIN_D3 21
    #define CAM_PIN_D2 19
    #define CAM_PIN_D1 18
    #define CAM_PIN_D0 5
    #define CAM_PIN_VSYNC 25
    #define CAM_PIN_HREF 23
    #define CAM_PIN_PCLK 22
    camera_config_t camera_config = {
        .pin_pwdn  = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_UXGA,

        .jpeg_quality = 10,
        .fb_count = 2,
        // .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY //CAMERA_GRAB_LATEST. Sets when buffers should be filled
    };
    esp_err_t ret = esp_camera_init(&camera_config);
    if(ret != ESP_OK) return ret;
    sensor_t * s = esp_camera_sensor_get();
    ret = s->set_framesize(s, FRAMESIZE_QQVGA);
    if(ret != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

void publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (!timer) return;

    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();
    if(!fb) {
        ESP_LOGW(TAG, "Camera capture failed.");
        return;
    }

    img_msg.step = fb->width;
    img_msg.height = fb->height;
    img_msg.width = fb->width;
    img_msg.data.data = fb->buf;
    img_msg.data.size = fb->len;
    img_msg.data.capacity = fb->len;

    ESP_LOGD(TAG, "Publishing Image: %dx%d - %d B", fb->width, fb->height, fb->len);
    RCSOFTCHECK(rcl_publish(&publisher, &img_msg, NULL));
    
    // Cleanup
    esp_camera_fb_return(fb);
    img_msg.data.data = NULL;
    img_msg.data.size = 0;
    img_msg.data.capacity = 0;
}

void camera_config_set(const char *key, const int32_t val) {
    if(!key) return;
    int ret = 0;
    const char *local_tag = "Camera config";
    sensor_t *s = esp_camera_sensor_get();
    if(!s) {
        ESP_LOGW(local_tag, "Camera sensor nor found.");
        return;
    }
    ESP_LOGI(local_tag, "Updating config with %s = %ld", key, val);
    if     (!strcmp(key, "framesize"))      ret = s->set_framesize(s, (framesize_t)val);
    else if(!strcmp(key, "contrast"))       ret = s->set_contrast(s, val);
    else if(!strcmp(key, "brightness"))     ret = s->set_brightness(s, val);
    else if(!strcmp(key, "saturation"))     ret = s->set_saturation(s, val);
    else if(!strcmp(key, "sharpness"))      ret = s->set_sharpness(s, val);
    else if(!strcmp(key, "denoise"))        ret = s->set_denoise(s, val);
    else if(!strcmp(key, "gainceiling"))    ret = s->set_gainceiling(s, (gainceiling_t)val);
    else if(!strcmp(key, "quality"))        ret = s->set_quality(s, val);
    else if(!strcmp(key, "colorbar"))       ret = s->set_colorbar(s, val);
    else if(!strcmp(key, "whitebal"))       ret = s->set_whitebal(s, val);
    else if(!strcmp(key, "gain_ctrl"))      ret = s->set_gain_ctrl(s, val);
    else if(!strcmp(key, "exposure_ctrl"))  ret = s->set_exposure_ctrl(s, val);
    else if(!strcmp(key, "hmirror"))        ret = s->set_hmirror(s, val);
    else if(!strcmp(key, "vflip"))          ret = s->set_vflip(s, val);
    else if(!strcmp(key, "aec2"))           ret = s->set_aec2(s, val);
    else if(!strcmp(key, "awb_gain"))       ret = s->set_awb_gain(s, val);
    else if(!strcmp(key, "agc_gain"))       ret = s->set_agc_gain(s, val);
    else if(!strcmp(key, "aec_value"))      ret = s->set_aec_value(s, val);
    else if(!strcmp(key, "special_effect")) ret = s->set_special_effect(s, val);
    else if(!strcmp(key, "wb_mode"))        ret = s->set_wb_mode(s, val);
    else if(!strcmp(key, "ae_level"))       ret = s->set_ae_level(s, val);
    else if(!strcmp(key, "dcw"))            ret = s->set_dcw(s, val);
    else if(!strcmp(key, "bpc"))            ret = s->set_bpc(s, val);
    else if(!strcmp(key, "wpc"))            ret = s->set_wpc(s, val);
    else if(!strcmp(key, "raw_gma"))        ret = s->set_raw_gma(s, val);
    else if(!strcmp(key, "lenc"))           ret = s->set_lenc(s, val);
    if(ret) ESP_LOGW(local_tag, "Updating camera settings failed (%d).", ret);
}

void subscribe_config_callback(const void *msg) {
    const std_msgs__msg__String *parsed_msg = (const std_msgs__msg__String *)msg;
    ESP_LOGI("DEBUG subscribtion", "%s", parsed_msg->data.data);

    char message[STR_BUF_SIZE] = {};

    strncpy(message, parsed_msg->data.data, STR_BUF_SIZE);

    char *key = NULL;
    char *val = NULL;
    for(size_t i = 0; i < parsed_msg->data.capacity; i++){
        if(!key) key = &message[i];
        if(message[i] == CONF_OPTION_SEPARATOR || message[i] == '\0'){
            message[i] = '\0';
            if(val){
                camera_config_set(key, atoi(val));
                key = NULL;
                val = NULL;
            }
        }
        else if(message[i] == CONF_KEYVAL_SEPARATOR) {
            message[i] = '\0';
            val = &message[i+1];
        }
    }
}

void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ESP_LOGI("DEBUG", "options init");
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    ESP_LOGI("DEBUG", "network init");
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    //RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
    ESP_LOGI("DEBUG", "support init");
    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    ESP_LOGI("DEBUG", "node create");
    RCCHECK(rclc_node_init_default(&node, "esp32_camera", "", &support));

    // create publisher
    ESP_LOGI("DEBUG", "publisher init");
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
        "/esp32/camera/image_jpeg"));

    // Create subscriber.
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32/camera/config"));

    // create timer,
    rcl_timer_t timer;
    ESP_LOGI("DEBUG", "timer init");
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        0,
        publish_timer_callback));

    config_msg.data = micro_ros_string_utilities_init_with_size(STR_BUF_SIZE);
    img_msg.header.frame_id = micro_ros_string_utilities_init("ESP32 camera");
    img_msg.header.stamp.sec = 0;
    img_msg.header.stamp.nanosec = 0;
    img_msg.encoding = micro_ros_string_utilities_init("jpeg");
    img_msg.is_bigendian = 0;


    // create executor
    rclc_executor_t executor;
    ESP_LOGI("DEBUG", "executor init");
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor,
                                           &subscriber,
                                           &config_msg,
                                           &subscribe_config_callback,
                                           ON_NEW_DATA));

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}


void app_main(void) {
    esp_err_t err = ESP_OK;


#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_LOGI(TAG, "Successfully connected to network.");
#endif
    
    err = init_camera();
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Camera failed to initialize. [%s](%d)", esp_err_to_name(err), err);
        return;
    }

    xTaskCreate(micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);

}
