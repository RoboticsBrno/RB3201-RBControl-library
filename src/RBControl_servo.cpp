#include <algorithm>
#include <chrono>
#include <FreeRTOS.h>
#include <freertos/task.h>
#include "RBControl_servo.hpp"

#include "uart.h"

#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace rb {

SmartServoBus::SmartServoBus() : m_bus(NULL) {
}

void SmartServoBus::install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin)
{
    m_uart = uart;
    m_uart_pin = pin;

    m_mutex.lock();
    if(m_bus == NULL) {
        m_bus = new lw::Bus();
        for(uint8_t i = 0; i < servo_count; ++i) {
            m_servos.push_back(servo_info(m_bus->getServo(i+1)));
        }
    }
    m_mutex.unlock();

    m_uart_queue = xQueueCreate(64, sizeof(struct tx_request));
    xTaskCreatePinnedToCore(&SmartServoBus::uartRoutineTrampoline,
        "rbsmartservobus_uart", 4096, this, 1, NULL, 0);
    xTaskCreate(&SmartServoBus::regulatorRoutineTrampoline, "rbsmartservobus_reg", 4096, this, 2, NULL);
}

void SmartServoBus::set(uint8_t id, float angle, float speed, float speed_raise) {
    speed = std::max(1.f, std::min(240.f, speed)) / 10.f;
    angle = std::max(0.f, std::min(360.f, angle)) * 100;

    m_mutex.lock();
    auto& si = m_servos[id];
    if(si.current != angle) {
        if(si.target == 0xFFFF) {
            si.current = angle + 1;
        } else if((si.current > si.target) != (si.current > angle)) {
            si.speed_coef = 0.f;
        }
        si.target = angle;
        si.speed_target = speed;
        si.speed_raise = speed_raise;
    }
    m_mutex.unlock();
}

void SmartServoBus::limit(uint8_t id,  Angle b, Angle t) {
    m_mutex.lock();
    auto pkt = m_servos[id].servo.limit(b, t);
    m_mutex.unlock();
    send(pkt);
}

void SmartServoBus::regulatorRoutineTrampoline(void *cookie) {
    ((SmartServoBus*)cookie)->regulatorRoutine();
}

void SmartServoBus::regulatorRoutine() {
    struct rx_response resp = { 0 };
    auto queue = xQueueCreate(1, sizeof(struct rx_response));
    while(true) {
        const uint32_t diff_ms = 50;
        const size_t servos = m_servos.size();
        for(size_t i = 0; i < servos; ++i) {
            m_mutex.lock();
            auto& s = m_servos[i];
            if(s.current == s.target) {
                vTaskDelay(MS_TO_TICKS(15));
                m_mutex.unlock();
                continue;
            }

            float speed = s.speed_target;
            if(s.speed_coef < 1.f) {
                s.speed_coef = std::min(1.f, s.speed_coef + s.speed_raise);
                speed *= (s.speed_coef * s.speed_coef);
            }

            int32_t dist = abs(int32_t(s.target) - int32_t(s.current));
            dist = std::max(1, std::min(dist, int32_t(speed * diff_ms)));
            if(dist > 0) {
                if (s.target < s.current) {
                    s.current -= dist;
                } else {
                    s.current += dist;
                }
            }

            if(dist <= 0 || s.current == s.target) {
                s.current = s.target;
                s.speed_coef = 0.f;
            }

            auto pkt = s.servo.move(Angle::deg(float(s.current)/100.f), std::chrono::milliseconds(diff_ms));
            m_mutex.unlock();

            send(pkt, queue);
            xQueueReceive(queue, &resp, portMAX_DELAY);
        }
    }
}

void SmartServoBus::update(uint32_t diff_ms) {

}

float SmartServoBus::pos(uint8_t id) {
    lw::Packet pkt(id+1, lw::Command::SERVO_POS_READ);

    struct rx_response resp = { 0 };
    auto queue = xQueueCreate(1, sizeof(struct rx_response));
    send(pkt, queue, true);

    xQueueReceive(queue, &resp, portMAX_DELAY);
    vQueueDelete(queue);

    if(resp.size != 0x08) {
        return nanf("");
    }

    float val = (float)((resp.data[6] << 8) | resp.data[5]);
    val = (val / 1000.f) * 240.f;

    m_mutex.lock();
    m_servos[id].current = val* 100;
    m_servos[id].target = val* 100;
    m_servos[id].servo.move(Angle::deg(val));
    m_mutex.unlock();

    return val;
}

void SmartServoBus::uartSwitchToTx() {
    rbu::uart_set_pin(m_uart, m_uart_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void SmartServoBus::uartRoutineTrampoline(void *cookie) {
    ((SmartServoBus*)cookie)->uartRoutine();
}

void SmartServoBus::uartRoutine() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(rbu::uart_param_config( m_uart, &uart_config ));

    gpio_output_disable( m_uart_pin );
    uartSwitchToTx();
    rbu::uart_set_pin(m_uart, UART_PIN_NO_CHANGE, m_uart_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_ERROR_CHECK(rbu::uart_driver_install(m_uart, 256, 0, 0, NULL, 0));

    rbu::uart_set_half_duplex_pin(m_uart, m_uart_pin);

    gpio_set_pull_mode(GPIO_NUM_14, (gpio_pull_mode_t)GPIO_PULLUP_ONLY);
    gpio_matrix_in(GPIO_NUM_14, U1RXD_IN_IDX, 0);
    gpio_set_direction(GPIO_NUM_14, (gpio_mode_t)GPIO_MODE_INPUT);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << GPIO_NUM_4;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NUM_4, 0);

    struct tx_request req;
    struct rx_response resp;
    auto tm_last = xTaskGetTickCount();
    constexpr auto min_delay = MS_TO_TICKS(15);
    while(true) {
        if(xQueueReceive(m_uart_queue, &req, portMAX_DELAY) != pdTRUE)
            continue;

        const auto diff = xTaskGetTickCount() - tm_last;
        if(diff < min_delay) {
            vTaskDelay(min_delay - diff);
        }

        uartSwitchToTx();
        gpio_set_level(GPIO_NUM_4, 1);
        rbu::uart_tx_chars(m_uart, req.data, req.size);
        tm_last = xTaskGetTickCount();
        req.size = uartReceive((uint8_t*)req.data, sizeof(req.data));

        //lw::Packet pkt((uint8_t*)req.data, req.size);
        //pkt.dump();
        if(req.size != 0 && req.expect_response) {
            resp.size = uartReceive(resp.data, sizeof(resp.data));
            //lw::Packet pkt(resp.data, resp.size);
            //pkt.dump();
        } else {
            resp.size = 0;
        }

        if(req.responseQueue) {
            xQueueSend(req.responseQueue, &resp, 300 / portTICK_PERIOD_MS);
        }
    }
}

size_t SmartServoBus::uartReceive(uint8_t *buff, size_t bufcap) {
    size_t bufsize = 0;
    const TickType_t wait_period = MS_TO_TICKS(4);
    constexpr TickType_t timeout = MS_TO_TICKS(20);

    while(true) {
        size_t avail = 0;
        size_t need = 0;
        const size_t oldsize = bufsize;
        switch(oldsize) {
            case 0:
            case 1:
                need = 1;
                break;
            case 2:
                need = 3;
                break;
            case 5:
                need = buff[3]-2;
                break;
            default:
                return bufsize;
        }

        if(need + oldsize > bufcap) {
            printf("Invalid servo packet size received: %d, aborting.\n", (int)buff[3]);
            abort();
            return 0;
        }

        TickType_t waiting = 0;
        while(rbu::uart_get_buffered_data_len(m_uart, &avail) != ESP_OK || avail < need) {
            if(waiting >= timeout)
                return 0;
            vTaskDelay(wait_period);
            waiting += wait_period;
            //printf("%u %u %u\n", oldsize, need, avail);
        }

        int res = rbu::uart_read_bytes(m_uart, buff + oldsize, need, 0);
        if(res < 0 || ((size_t)res) != need) {
            printf("Invalid servo packet read: %d, aborting.\n", res);
            abort();
            return 0;
        }
        bufsize += need;

       // printf("%u %u %u %X\n", oldsize, need, avail, (int)buff[oldsize]);

        if(oldsize < 2 && buff[oldsize] != 0x55)
            bufsize = 0;
    }
    return 0;
}

void SmartServoBus::send(const lw::Packet& pkt, QueueHandle_t responseQueue, bool expect_response) {
    send((const char *)pkt._data.data(), pkt._data.size(), responseQueue, expect_response);
}

void SmartServoBus::send(const char *data, size_t size, QueueHandle_t responseQueue, bool expect_response) {
    struct tx_request req = { 0 };
    req.size = (uint8_t)size;
    req.expect_response = expect_response;
    req.responseQueue = responseQueue;

    if(sizeof(req.data) < size) {
        printf("SmartServoBus packet is too big, %u > %u", size, sizeof(req.data));
        abort();
    }
    memcpy(req.data, data, size);
    xQueueSend(m_uart_queue, &req, portMAX_DELAY);
}

}; // namespace rb