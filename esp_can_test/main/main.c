#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

#define TWAI_TX_GPIO  5   // 改成你的 TX 引脚
#define TWAI_RX_GPIO  4   // 改成你的 RX 引脚

static void print_status(const char* tag)
{
    twai_status_info_t s;
    twai_get_status_info(&s);

    const char* state =
        (s.state == TWAI_STATE_STOPPED) ? "STOPPED" :
        (s.state == TWAI_STATE_RUNNING) ? "RUNNING" :
        (s.state == TWAI_STATE_BUS_OFF) ? "BUS_OFF" :
        (s.state == TWAI_STATE_RECOVERING) ? "RECOVERING" : "UNKNOWN";

    printf("[%s] state=%s  TEC=%d  REC=%d  bus_err=%d  tx_failed=%d  rx_missed=%d  rx_overrun=%d\n",
           tag, state, s.tx_error_counter, s.rx_error_counter, s.bus_error_count,
           s.tx_failed_count, s.rx_missed_count, s.rx_overrun_count);
}

void app_main(void)
{
    // 1) 配置：先用 500kbps（你也可以换成 250/1M）
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    print_status("after_start");

    // 2) 准备一帧：标准帧 ID=0x123，8字节
    twai_message_t tx_msg = {
        .identifier = 0x123,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {0xDE,0xAD,0xBE,0xEF,0x11,0x22,0x33,0x44}
    };

    while (1) {
        // 发一帧
        esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            printf("TX OK\n");
        } else {
            printf("TX FAIL err=0x%x\n", err);
        }

        // 看状态（最关键）
        print_status("after_tx");

        // 也顺便收一下（如果总线上有人发）
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            printf("RX id=0x%03lx dlc=%d data0=0x%02x\n",
                   rx_msg.identifier, rx_msg.data_length_code, rx_msg.data[0]);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
