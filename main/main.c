#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "ardupilotmega/mavlink.h"
#include "driver/gpio.h"
#include <inttypes.h>


#define UART_PORT UART_NUM_2
#define UART_TXD  (GPIO_NUM_17)
#define UART_RXD  (GPIO_NUM_16)
#define UART_RTS  (UART_PIN_NO_CHANGE)
#define UART_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

 uint32_t custom_mode = 5; /*<  A bitfield for use for autopilot-specific flags*/
 uint8_t type = 14; /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
 uint8_t autopilot = 3; /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
 uint8_t base_mode = 80; /*<  System mode bitmap.*/
 uint8_t system_status = 3; /*<  System status flag.*/
 uint8_t mavlink_version = 3;


void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TXD, UART_RXD, UART_RTS, UART_CTS);
}
void app_main(void)
{
    uart_init();
    gpio_set_pull_mode(UART_RXD, GPIO_PULLUP_ONLY);
    uint8_t data[BUF_SIZE];
    mavlink_message_t msg;
    mavlink_status_t status;
    mavlink_message_t msg_heartbeat;
    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        //printf("system_id = [%d], component_id = [%d]\n", data[3],data[4]);
        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        ESP_LOGI("MAVLINK", "Heartbeat received: system status = %d,base_mode = [%d], custom = [%ld], type = [%d], auto_pilot = [%d], version = [%d]", hb.system_status, hb.base_mode, hb.custom_mode, hb.type, hb.autopilot, hb.mavlink_version);
                        break;
                    }
                    case MAVLINK_MSG_ID_SYS_STATUS: {
                        mavlink_sys_status_t sys;
                        mavlink_msg_sys_status_decode(&msg, &sys);
                        ESP_LOGI("MAVLINK", "drop rate: %d V", sys.drop_rate_comm);
                        break;
                    }
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg, &att);
                        ESP_LOGI("MAVLINK", "roll = %f, pitch = %f, yaw = %f", att.roll* 180 / 3.1415926, att.pitch* 180 / 3.1415926, att.yaw* 180 / 3.1415926);
                        break;
                    }
                    case MAVLINK_MSG_ID_GPS_RAW_INT: {
                        mavlink_gps_raw_int_t gps;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps);
                        ESP_LOGI("MAVLINK", "GPS: lat = %ld, lon = %ld, alt = %ld", gps.lat, gps.lon, gps.alt);
                        break;
                    }
                    case MAVLINK_MSG_ID_BATTERY_STATUS: {
                        mavlink_battery_status_t bat;
                        mavlink_msg_battery_status_decode(&msg, &bat);
                        ESP_LOGI("MAVLINK", "Battery status: voltage = %d V, current = %d A", bat.voltages[0], bat.current_battery);
                        break;
                    }
                    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
                        mavlink_servo_output_raw_t servo;
                        mavlink_msg_servo_output_raw_decode(&msg, &servo);
                        ESP_LOGI("MAVLINK", "Servo output: servo1 = %d, servo2 = %d, servo3 = %d, servo4 = %d", servo.servo1_raw, servo.servo2_raw,servo.servo3_raw,servo.servo4_raw);
                        break;
                    }
                    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
                        mavlink_rc_channels_raw_t rc;
                        mavlink_msg_rc_channels_decode(&msg, &rc);
                        ESP_LOGI("MAVLINK", "RC channels: channel1 = %d, channel2 = %d, channel3 = %d, channel4 = %d, channel5 = %d, RSSI = %d", rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,rc.chan5_raw,rc.rssi);
                        break;
                    }
                    case MAVLINK_MSG_ID_HIGHRES_IMU: {
                        mavlink_highres_imu_t imu;
                        mavlink_msg_highres_imu_decode(&msg, &imu);
                        ESP_LOGI("MAVLINK", "IMU: ax = %f, ay = %f, az = %f, temperature = %.2f", imu.xacc, imu.yacc, imu.zacc, imu.temperature);
                        break;
                    }
                }
            }           
        }
        /*uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        
        mavlink_msg_heartbeat_pack( 1, 200, &msg_heartbeat, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_STABILIZE_DISARMED, 0, MAV_STATE_ACTIVE);
        uint16_t len_1 = mavlink_msg_to_send_buffer(buffer, &msg_heartbeat);
        uart_write_bytes(UART_PORT, (const char *)buffer, len_1);*/


        mavlink_message_t msg_1;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_command_long_pack(
            1, 200,     // 自己的 sysid, compid
            &msg_1,
            1, 0,       // 目標 sysid=1, compid=0 (APM)
            MAV_CMD_COMPONENT_ARM_DISARM,
            0,          // confirmation
            1, 0, 0, 0, 0, 0, 0  // param1=1 表示 ARM
        );

        uint16_t len_2 = mavlink_msg_to_send_buffer(buf, &msg_1);
        uart_write_bytes(UART_PORT, (const char*)buf, len_2);


        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
