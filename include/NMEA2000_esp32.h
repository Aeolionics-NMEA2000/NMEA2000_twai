// SPDX-FileCopyrightText: 2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT
//

#ifndef TEST_ESP32_NMEA2000_ESP32_H
#define TEST_ESP32_NMEA2000_ESP32_H

#include <NMEA2000.h>
#include "driver/twai.h"

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif

class tNMEA2000_esp32 : public tNMEA2000 {
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;

public:
    tNMEA2000_esp32(const gpio_num_t tx_pin = ESP32_CAN_TX_PIN, const gpio_num_t rx_pin = ESP32_CAN_RX_PIN) : tNMEA2000(), tx_pin(tx_pin), rx_pin(rx_pin) {
    }

protected:
    bool CANOpen() override {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
            return false;
        }

        if (twai_start() != ESP_OK) {
            return false;
        }

        uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
        if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
            return false;
        }

        return true;
    }

    bool CANSendFrame(const unsigned long id, const unsigned char len, const unsigned char *buf, const bool wait_sent) override {
        auto message = twai_message_t{};
        message.extd = true;
        message.identifier = id;
        message.data_length_code = len;
        memcpy(message.data, buf, len);

        return twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK;
    }

    bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override {
        twai_message_t message;
        switch (twai_receive(&message, pdMS_TO_TICKS(0))) {
            case ESP_ERR_TIMEOUT:
                return false;
            case ESP_OK:
                id = message.identifier;
                len = message.data_length_code;
                memcpy(buf, message.data, len);
                return true;
            default:
                return false;
        }
    }
};
#endif //TEST_ESP32_NMEA2000_ESP32_H
