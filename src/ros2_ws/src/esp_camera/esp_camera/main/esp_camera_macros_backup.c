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
#include <std_msgs/msg/int32.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

const char *TAG = "ROS2 node ESP32CAM";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(KEY) \
    FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK_TYPED(int, KEY) 
#define FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK_TYPED(DATATYPE, KEY)       \
    rcl_subscription_t subscriber_##KEY;                                    \
    void config_subscription_cb_##KEY(const void *msg) {                    \
        std_msgs__msg__Int32 *parsed_msg = (std_msgs__msg__Int32 *)msg;     \
        sensor_t *s = esp_camera_sensor_get();                              \
        int ret = 0;                                                        \
        if((ret = s->set_##KEY(s, (DATATYPE)parsed_msg->data))){            \
            ESP_LOGW("ESP32CAM config " #KEY,                               \
                     "Failed to set " #KEY " with %d as return code.", ret);\
        }                                                                   \
        else {                                                              \
            ESP_LOGI("ESP32CAM config " #KEY,                               \
                     "Successfully set " #KEY " = %d",                      \
                     (DATATYPE)parsed_msg->data);                           \
        }                                                                   \
    }

FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK_TYPED(pixformat_t, pixformat)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK_TYPED(framesize_t, framesize)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK_TYPED(gainceiling_t, gainceiling)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(contrast)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(brightness)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(saturation)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(sharpness)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(denoise)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(quality)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(colorbar)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(whitebal)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(gain_ctrl)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(exposure_ctrl)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(hmirror)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(vflip)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(aec2)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(awb_gain)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(agc_gain)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(aec_value)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(special_effect)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(wb_mode)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(ae_level)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(dcw)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(bpc)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(wpc)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(raw_gma)
FUNC_CAMERA_CONFIG_SUBSCRIPTION_CALLBACK(lenc)


rcl_publisher_t publisher;
std_msgs__msg__Int32 config_msg;


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
        .frame_size = FRAMESIZE_QQVGA,

        .jpeg_quality = 10,
        .fb_count = 2,
        // .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY //CAMERA_GRAB_LATEST. Sets when buffers should be filled
    };
    return esp_camera_init(&camera_config);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (!timer) return;


    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();
    if(!fb) {
        ESP_LOGE(TAG, "Camera capture failed.");
        return;
    }

    sensor_msgs__msg__Image msg = {
        .header = {
            .frame_id = {
                .data = "<frame_id>",
                .size = strlen(msg.header.frame_id.data),
                .capacity = strlen(msg.header.frame_id.data) + 1
            },
            .stamp = {
                .sec = 10,
                .nanosec = 69
            }
        },
        .encoding = {
            .data = "jpeg", // As you are sending JPEG encoded data
            .size = 4,
            .capacity = 5
        },
        .is_bigendian = 0, // Most architectures are little endian
        .step = fb->width, //  * <bytes_per_pixel>; // For JPEG, this can be just the width
        .height = fb->height,
        .width = fb->width,
        .data = {
            .data = fb->buf,
            .size = fb->len,
            .capacity = fb->len
        }
    };


    ESP_LOGI(TAG, "Publishing Image: %dx%d - %d B", fb->width, fb->height, fb->len);
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    
    // Cleanup
    esp_camera_fb_return(fb);
    msg.data.data = NULL;
    msg.data.size = 0;
    msg.data.capacity = 0;
    msg.header.frame_id.data = NULL;
    msg.header.frame_id.size = 0;
    msg.header.frame_id.capacity = 0;
    msg.encoding.data = NULL;
    msg.encoding.size = 0;
    msg.encoding.capacity = 0;
    sensor_msgs__msg__Image__fini(&msg);
}


void micro_ros_task(void * arg)
{
    #define REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, topic_prefix, KEY) do { \
        sprintf(topic_buffer, "%s/%s", topic_prefix, #KEY);                             \
        ESP_LOGI("DEBUG", #KEY " subscription init for '%s'", topic_buffer);            \
        RCCHECK(rclc_subscription_init_default(                                         \
            &subscriber_##KEY,                                                          \
            &node,                                                                      \
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),                          \
            topic_buffer));                                                             \
        ESP_LOGI("DEBUG", #KEY " add to executor");                                     \
        RCCHECK(rclc_executor_add_subscription(                                         \
            &executor,                                                                  \
            &subscriber_##KEY,                                                          \
            &config_msg,                                                                \
            &config_subscription_cb_##KEY,                                              \
            ON_NEW_DATA));                                                              \
    } while(0)

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    char topic_buffer[128] = {};

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
        "/camera/esp32/image_jpeg"));

    // create timer,
    rcl_timer_t timer;
    ESP_LOGI("DEBUG", "timer init");
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    rclc_executor_t executor;
    ESP_LOGI("DEBUG", "executor init");
    RCCHECK(rclc_executor_init(&executor, &support.context, 28, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", pixformat);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", framesize);
    // REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", gainceiling);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", contrast);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", brightness);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", saturation);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", sharpness);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", denoise);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", quality);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", colorbar);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", whitebal);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", gain_ctrl);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", exposure_ctrl);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", hmirror);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", vflip);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", aec2);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", awb_gain);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", agc_gain);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", aec_value);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", special_effect);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", wb_mode);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", ae_level);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", dcw);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", bpc);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", wpc);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", raw_gma);
    REGISTER_CONFIG_SUBSCRIPTION_CALLBACK(topic_buffer, "/camera/esp32/config", lenc);

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
