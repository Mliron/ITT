#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <micro_ros_utilities/string_utilities.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif


#include "driver/rmt.h"
#include "led_strip.h"

const char *TAG = "ROS2 node LEDSTRIP";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define STR_BUF_SIZE 512
#define RMT_TX_CHANNEL     RMT_CHANNEL_0
#define RMT_LED_STRIP_GPIO 18
#define LED_COUNT          30

rcl_subscription_t subscriber;
std_msgs__msg__String color_msg;

led_strip_t* strip = NULL;

void subscribe_ledcolor_callback(const void *msg) {
    std_msgs__msg__String *parsed_msg = (std_msgs__msg__String *)msg;
    if(parsed_msg->data.data[0] != '#'){
        ESP_LOGW(TAG, "Received invalid color '%s'", parsed_msg->data.data);
        return;
    }
    if(!strip) {
        ESP_LOGW(TAG, "LED strip not initialized!");
        return;
    }

    uint8_t r, g, b;
    char tmp[3] = {0};
    switch(parsed_msg->data.size) {
        case 4: {
            tmp[0] = parsed_msg->data.data[1];
            r = (strtol(tmp, NULL, 16)&0xf) << 4;
            tmp[0] = parsed_msg->data.data[2];
            g = (strtol(tmp, NULL, 16)&0xf) << 4;
            tmp[0] = parsed_msg->data.data[3];
            b = (strtol(tmp, NULL, 16)&0xf) << 4;
        } break;
        case 7: {
            tmp[0] = parsed_msg->data.data[1];
            tmp[1] = parsed_msg->data.data[2];
            r = strtol(tmp, NULL, 16)&0xff;
            tmp[0] = parsed_msg->data.data[3];
            tmp[1] = parsed_msg->data.data[4];
            g = strtol(tmp, NULL, 16)&0xff;
            tmp[0] = parsed_msg->data.data[5];
            tmp[1] = parsed_msg->data.data[6];
            b = strtol(tmp, NULL, 16)&0xff;
        } break;
        default: {
            ESP_LOGW(TAG, "Received invalid color '%s'", parsed_msg->data.data);
            return;
        } break;
    }
    ESP_LOGI(TAG, "Changing colors to {%d, %d, %d}", r, g, b);
    for (int i = 0; i < LED_COUNT; i++) {
        ESP_ERROR_CHECK(strip->set_pixel(strip, i, r, g, b));
    }
    ESP_ERROR_CHECK(strip->refresh(strip, 0));
}

void led_strip_init(){
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_LED_STRIP_GPIO, RMT_TX_CHANNEL);
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_COUNT, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
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
    RCCHECK(rclc_node_init_default(&node, "esp32_led_strip", "", &support));

    // Create subscriber.
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32/led_strip/color"));

    led_strip_init();
    color_msg.data = micro_ros_string_utilities_init_with_size(STR_BUF_SIZE);

    // create executor
    rclc_executor_t executor;
    ESP_LOGI("DEBUG", "executor init");
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor,
                                           &subscriber,
                                           &color_msg,
                                           &subscribe_ledcolor_callback,
                                           ON_NEW_DATA));

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}


void app_main(void) {
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_LOGI(TAG, "Successfully connected to network.");
#endif


    xTaskCreate(micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);
}
