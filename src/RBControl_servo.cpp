#include "RBControl_servo.hpp"
#include "RBControl_manager.hpp"
#include <algorithm>
#include <chrono>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "half_duplex_uart.h"

#define TAG "RBControlSmartServoBus"
#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace rb {

SmartServoBus::SmartServoBus() {
}

void SmartServoBus::install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin) {
    if (!m_servos.empty() || servo_count == 0)
        return;

    m_servos.resize(servo_count);

    m_uart = uart;
    m_uart_pin = pin;

    m_uart_queue = xQueueCreate(8, sizeof(struct tx_request));

    TaskHandle_t task;
    xTaskCreatePinnedToCore(&SmartServoBus::uartRoutineTrampoline, "rbservo_uart", 2048, this, 1, &task, 1);
    Manager::get().monitorTask(task);

    xTaskCreate(&SmartServoBus::regulatorRoutineTrampoline, "rbservo_reg", 1536, this, 2, &task);
    Manager::get().monitorTask(task);

    Angle val;
    for (uint8_t i = 0; i < servo_count; ++i) {
        for (int x = 0; x < 3; ++x) {
            val = pos(i);
            if (!val.isNaN()) {
                break;
            } else {
                ESP_LOGW(TAG, "failed to read servo %d pos, attempt %d", i, x + 1);
            }
        }

        if (val.isNaN()) {
            ESP_LOGE(TAG, "failed to read position from servo %d, it will not work!", i);
            continue;
        }

        const uint16_t deg_val = 100 * val.deg();

        m_mutex.lock();
        m_servos[i].current = deg_val;
        m_servos[i].target = deg_val;
        m_mutex.unlock();
    }
}

void SmartServoBus::setId(uint8_t newId, uint8_t destId) {
    auto pkt = lw::Packet::setId(destId, newId);
    send(pkt);
}

uint8_t SmartServoBus::getId(uint8_t destId) {
    struct rx_response resp;
    sendAndReceive(lw::Packet::getId(destId), resp);
    if (resp.size != 7)
        return 0xFF;
    return resp.data[5];
}

void SmartServoBus::set(uint8_t id, Angle ang, float speed, float speed_raise) {
    speed = std::max(1.f, std::min(240.f, speed)) / 10.f;
    const uint16_t angle = std::max(0.f, std::min(360.f, (float)ang.deg())) * 100;

    std::lock_guard<std::mutex> lock(m_mutex);

    auto& si = m_servos[id];
    if (!si.hasValidCurrent()) {
        const auto cur = pos(id);
        if (cur.isNaN()) {
            ESP_LOGE(TAG, "failed to get servo %d position, can't move it!", int(id));
            return;
        }
        const uint16_t deg_val = 100 * cur.deg();
        si.current = deg_val;
        si.target = deg_val;
    }

    if (si.current == angle)
        return;

    if ((si.current > si.target) != (si.current > angle)) {
        si.speed_coef = 0.f;
    }

    si.target = angle;
    si.speed_target = speed;
    si.speed_raise = speed_raise;
}

Angle SmartServoBus::pos(uint8_t id) {
    lw::Packet pkt(id, lw::Command::SERVO_POS_READ);

    struct rx_response resp;
    sendAndReceive(pkt, resp, true);
    if (resp.size != 0x08) {
        return Angle::nan();
    }

    float val = (float)((resp.data[6] << 8) | resp.data[5]);
    val = (val / 1000.f) * 240.f;

    return Angle::deg(val);
}

Angle SmartServoBus::posOffline(uint8_t id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto& s = m_servos[id];
    if (s.current == 0xFFFF)
        return Angle::nan();
    return Angle::deg(Angle::_T(s.current) / 100.f);
}

void SmartServoBus::limit(uint8_t id, Angle bottom, Angle top) {
    auto pkt = lw::Servo::limit(id, bottom, top);
    send(pkt);
}

void SmartServoBus::setAutoStop(uint8_t id, bool enable) {
    m_mutex.lock();
    m_servos[id].auto_stop = enable;
    m_mutex.unlock();
}

void SmartServoBus::regulatorRoutineTrampoline(void* cookie) {
    ((SmartServoBus*)cookie)->regulatorRoutine();
}

void SmartServoBus::regulatorRoutine() {
    const size_t servos_cnt = m_servos.size();

    constexpr uint32_t msPerServo = 30;
    constexpr auto ticksPerServo = MS_TO_TICKS(msPerServo);
    const uint32_t msPerIter = servos_cnt * msPerServo;
    const auto ticksPerIter = MS_TO_TICKS(msPerIter);

    auto queue = xQueueCreate(1, sizeof(struct rx_response));
    while (true) {
        const auto tm_iter_start = xTaskGetTickCount();
        for (size_t i = 0; i < servos_cnt; ++i) {
            const auto tm_servo_start = xTaskGetTickCount();
            regulateServo(queue, i, msPerIter);
            const auto diff = xTaskGetTickCount() - tm_servo_start;
            if (diff < ticksPerServo) {
                vTaskDelay(ticksPerServo - diff);
            }
        }

        const auto diff = xTaskGetTickCount() - tm_iter_start;
        if (diff < ticksPerIter) {
            vTaskDelay(ticksPerIter - diff);
        }
    }
}

bool SmartServoBus::regulateServo(QueueHandle_t responseQueue, size_t id, uint32_t timeSliceMs) {
    float move_pos_deg;
    auto& s = m_servos[id];
    struct rx_response resp;

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (s.auto_stop) {
            lw::Packet pos_req(id, lw::Command::SERVO_POS_READ);
            send(pos_req, responseQueue, true);
            xQueueReceive(responseQueue, &resp, portMAX_DELAY);
            if (resp.size == 0x08) {
                const float val = (float)((resp.data[6] << 8) | resp.data[5]);
                const int val_int = (val / 1000.f) * 24000.f;
                const int diff = val_int - int(s.current);
                if (abs(diff) > 300) {
                    if (++s.auto_stop_counter > 5) {
                        s.target = val_int + (diff > 0 ? -200 : 200);
                        s.auto_stop_counter = 0;
                    }
                } else if (s.auto_stop_counter != 0) {
                    s.auto_stop_counter = 0;
                }
            }
        }

        if (s.current == s.target) {
            return false;
        }

        float speed = s.speed_target;
        if (s.speed_coef < 1.f) {
            s.speed_coef = std::min(1.f, s.speed_coef + (s.speed_raise * timeSliceMs));
            speed *= (s.speed_coef * s.speed_coef);
        }

        int32_t dist = abs(int32_t(s.target) - int32_t(s.current));
        dist = std::max(1, std::min(dist, int32_t(speed * timeSliceMs)));
        if (dist > 0) {
            if (s.target < s.current) {
                s.current -= dist;
            } else {
                s.current += dist;
            }
        }

        if (dist <= 0 || s.current == s.target) {
            s.current = s.target;
            s.speed_coef = 0.f;
        }
        move_pos_deg = float(s.current) / 100.f;
    }

    const auto pkt = lw::Servo::move(id, Angle::deg(move_pos_deg), std::chrono::milliseconds(timeSliceMs - 5));
    send(pkt, responseQueue, false, true);

    if (xQueueReceive(responseQueue, &resp, 500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Response to move packet not received!");
    }
    return true;
}

void SmartServoBus::uartRoutineTrampoline(void* cookie) {
    ((SmartServoBus*)cookie)->uartRoutine();
}

void SmartServoBus::uartRoutine() {
    {
        const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_ERROR_CHECK(half_duplex::uart_param_config(m_uart, &uart_config));
        ESP_ERROR_CHECK(half_duplex::uart_driver_install(m_uart, 256, 0, 0, NULL, 0));
        half_duplex::uart_set_half_duplex_pin(m_uart, m_uart_pin);
    }

    struct tx_request req;
    struct rx_response resp;
    auto tm_last = xTaskGetTickCount();
    constexpr auto min_delay = MS_TO_TICKS(15);
    while (true) {
        if (xQueueReceive(m_uart_queue, &req, portMAX_DELAY) != pdTRUE)
            continue;

        const auto diff = xTaskGetTickCount() - tm_last;
        if (diff < min_delay) {
            vTaskDelay(min_delay - diff);
        }

        half_duplex::uart_tx_chars(m_uart, req.data, req.size);
        tm_last = xTaskGetTickCount();
        req.size = uartReceive((uint8_t*)req.data, sizeof(req.data));

        if (req.size != 0 && req.expect_response) {
            resp.size = uartReceive(resp.data, sizeof(resp.data));
        } else {
            resp.size = 0;
        }

        if (req.responseQueue) {
            xQueueSend(req.responseQueue, &resp, 300 / portTICK_PERIOD_MS);
        }
    }
}

size_t SmartServoBus::uartReceive(uint8_t* buff, size_t bufcap) {
    constexpr TickType_t wait_period = MS_TO_TICKS(4);
    constexpr TickType_t timeout = MS_TO_TICKS(20);

    size_t bufsize = 0;

    while (true) {
        size_t avail = 0;
        size_t need = 0;
        const size_t oldsize = bufsize;
        switch (oldsize) {
        case 0:
        case 1:
            need = 1;
            break;
        case 2:
            need = 3;
            break;
        case 5:
            need = buff[3] - 2;
            break;
        default:
            return bufsize;
        }

        if (need + oldsize > bufcap) {
            ESP_LOGE(TAG, "invalid packet size received: %d.\n", (int)buff[3]);
            return 0;
        }

        TickType_t waiting = 0;
        while (half_duplex::uart_get_buffered_data_len(m_uart, &avail) != ESP_OK || avail < need) {
            if (waiting >= timeout) {
                ESP_LOGE(TAG, "timeout when waiting for data!");
                return 0;
            }
            vTaskDelay(wait_period);
            waiting += wait_period;
        }

        int res = half_duplex::uart_read_bytes(m_uart, buff + oldsize, need, 0);
        if (res < 0 || ((size_t)res) != need) {
            ESP_LOGE(TAG, "invalid packet read: %d, aborting.\n", res);
            abort();
            return 0;
        }
        bufsize += need;

        if (oldsize < 2 && buff[oldsize] != 0x55)
            bufsize = 0;
    }
    return 0;
}

void SmartServoBus::send(const lw::Packet& pkt, QueueHandle_t responseQueue, bool expect_response, bool to_front) {
    struct tx_request req = { 0 };
    req.size = (uint8_t)pkt._data.size();
    req.expect_response = expect_response;
    req.responseQueue = responseQueue;

    if (sizeof(req.data) < pkt._data.size()) {
        ESP_LOGE(TAG, "packet is too big, %u > %u", pkt._data.size(), sizeof(req.data));
        abort();
    }
    memcpy(req.data, pkt._data.data(), pkt._data.size());
    if (to_front) {
        xQueueSendToFront(m_uart_queue, &req, portMAX_DELAY);
    } else {
        xQueueSendToBack(m_uart_queue, &req, portMAX_DELAY);
    }
}

void SmartServoBus::sendAndReceive(const lw::Packet& pkt, struct SmartServoBus::rx_response& res, bool to_front) {

    memset(&res, 0, sizeof(struct rx_response));

    auto queue = xQueueCreate(1, sizeof(struct rx_response));
    send(pkt, queue, true);
    xQueueReceive(queue, &res, portMAX_DELAY);
    vQueueDelete(queue);
}

}; // namespace rb
