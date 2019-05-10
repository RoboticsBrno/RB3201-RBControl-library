#include <algorithm>
#include <chrono>
#include <math.h>
#include <FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "RBControl_servo.hpp"

#include "half_duplex_uart.h"

#define TAG "RBControlSmartServoBus"
#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace rb {

SmartServoBus::SmartServoBus() {
}

void SmartServoBus::install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin)
{
    if(!m_servos.empty() || servo_count == 0)
        return;

    for(uint8_t i = 0; i < servo_count; ++i) {
        m_servos.push_back(servo_info());
    }

    m_uart = uart;
    m_uart_pin = pin;
    
    m_uart_queue = xQueueCreate(8, sizeof(struct tx_request));

    xTaskCreatePinnedToCore(&SmartServoBus::uartRoutineTrampoline, "rbsmartservobus_uart", 2048, this, 1, NULL, 0);
    xTaskCreate(&SmartServoBus::regulatorRoutineTrampoline, "rbsmartservobus_reg", 1024, this, 2, NULL);

    float val;
    for(uint8_t i = 0; i < servo_count; ++i) {
        for(int x = 0; x < 5; ++x) {
            val = pos(i);
            if(!std::isnan(val)) {
                break;
            } else {
                ESP_LOGW(TAG, "failed to read servo %d pos, attempt %d", i, x+1);
            }
        }

        if(std::isnan(val)) {
            ESP_LOGE(TAG, "failed to read position from servo %d, aborting.", i);
            abort();
        }

        val *= 100;

        m_mutex.lock();
        m_servos[i].current = val;
        m_servos[i].target = val;
        m_mutex.unlock();
    }
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
    auto pkt = lw::Servo::limit(id+1, b, t);
    send(pkt);
}

void SmartServoBus::regulatorRoutineTrampoline(void *cookie) {
    ((SmartServoBus*)cookie)->regulatorRoutine();
}

void SmartServoBus::regulatorRoutine() {
    const size_t servos_cnt = m_servos.size();

    constexpr uint32_t msPerServo = 30;
    constexpr auto ticksPerServo = MS_TO_TICKS(msPerServo);
    const uint32_t msPerIter = servos_cnt * msPerServo + 10;
    const auto ticksPerIter = MS_TO_TICKS(msPerIter);

    auto queue = xQueueCreate(1, sizeof(struct rx_response));
    while(true) {
        const auto tm_iter_start = xTaskGetTickCount();
        for(size_t i = 0; i < servos_cnt; ++i) {
            const auto tm_servo_start = xTaskGetTickCount();
            regulateServo(queue, i, msPerIter);
            const auto diff = xTaskGetTickCount() - tm_servo_start;
            if(diff < ticksPerServo) {
                vTaskDelay(ticksPerServo - diff);
            }
        }

        const auto diff = xTaskGetTickCount() - tm_iter_start;
        if(diff < ticksPerIter) {
            vTaskDelay(ticksPerIter - diff);
        }
    }
}

bool SmartServoBus::regulateServo(QueueHandle_t responseQueue, size_t id, uint32_t timeSliceMs) {
    float move_pos_deg;
    auto& s = m_servos[id];

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if(s.current == s.target) {
            return false;
        }

        float speed = s.speed_target;
        if(s.speed_coef < 1.f) {
            s.speed_coef = std::min(1.f, s.speed_coef + s.speed_raise);
            speed *= (s.speed_coef * s.speed_coef);
        }

        int32_t dist = abs(int32_t(s.target) - int32_t(s.current));
        dist = std::max(1, std::min(dist, int32_t(speed * timeSliceMs)));
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
        move_pos_deg = float(s.current)/100.f;
    }

    const auto pkt = lw::Servo::move(id+1, Angle::deg(move_pos_deg), std::chrono::milliseconds(timeSliceMs));
    send(pkt, responseQueue, false, true);

    struct rx_response resp;
    xQueueReceive(responseQueue, &resp, portMAX_DELAY);
    return true;
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

    return val;
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
    ESP_ERROR_CHECK(half_duplex::uart_param_config( m_uart, &uart_config ));

    // is this necessary?
    //gpio_output_disable( m_uart_pin );

    ESP_ERROR_CHECK(half_duplex::uart_driver_install(m_uart, 256, 0, 0, NULL, 0));

    half_duplex::uart_set_half_duplex_pin(m_uart, m_uart_pin);

    /*
    // is this necessary?
    gpio_set_pull_mode(GPIO_NUM_14, (gpio_pull_mode_t)GPIO_PULLUP_ONLY);
    gpio_matrix_in(GPIO_NUM_14, U1RXD_IN_IDX, 0);
    gpio_set_direction(GPIO_NUM_14, (gpio_mode_t)GPIO_MODE_INPUT); */

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

        half_duplex::uart_tx_chars(m_uart, req.data, req.size);
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
            ESP_LOGE(TAG, "invalid packet size received: %d, aborting.\n", (int)buff[3]);
            abort();
            return 0;
        }

        TickType_t waiting = 0;
        while(half_duplex::uart_get_buffered_data_len(m_uart, &avail) != ESP_OK || avail < need) {
            if(waiting >= timeout)
                return 0;
            vTaskDelay(wait_period);
            waiting += wait_period;
            //printf("%u %u %u\n", oldsize, need, avail);
        }

        int res = half_duplex::uart_read_bytes(m_uart, buff + oldsize, need, 0);
        if(res < 0 || ((size_t)res) != need) {
            ESP_LOGE(TAG, "invalid packet read: %d, aborting.\n", res);
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

void SmartServoBus::send(const lw::Packet& pkt, QueueHandle_t responseQueue, bool expect_response, bool to_front) {
    struct tx_request req = { 0 };
    req.size = (uint8_t)pkt._data.size();
    req.expect_response = expect_response;
    req.responseQueue = responseQueue;

    if(sizeof(req.data) < pkt._data.size()) {
        ESP_LOGE(TAG, "packet is too big, %u > %u", pkt._data.size(), sizeof(req.data));
        abort();
    }
    memcpy(req.data, pkt._data.data(), pkt._data.size());
    if(to_front) {
        xQueueSendToFront(m_uart_queue, &req, portMAX_DELAY);
    } else {
        xQueueSendToBack(m_uart_queue, &req, portMAX_DELAY);
    }
}

}; // namespace rb