#include <algorithm>
#include <chrono>
#include <FreeRTOS.h>
#include <freertos/task.h>
#include "RBControl_servo.hpp"

#include "uart.h"

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
    xTaskCreate(&SmartServoBus::uartRoutineTrampoline, "rbsmartservobus_uart", 4096, this, 1, NULL);
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

void SmartServoBus::update(uint32_t diff_ms) {
    int i = 0;
    m_mutex.lock();

#if 0
    if(uart_get_buffered_data_len(m_bus->_uart, &avail) == ESP_OK && avail >= 5) {
        uart_read_bytes( m_bus->_uart, buff, 5, 0);
        lw::Packet pkt(buff, 5);
        uart_read_bytes( m_bus->_uart, buff, pkt.size() - 2, 100);
        pkt._data.insert(pkt._data.end(), buff, buff + (pkt.size() - 2));
        pkt.dump();
    }
#elif 0
    while(uart_get_buffered_data_len(m_bus->_uart, &avail) == ESP_OK && avail > 0) {
        uart_read_bytes( m_bus->_uart, buff, 1, 0);
        printf("%02x ", (int)buff[0]);
    }
#endif

    for(auto &s : m_servos) {
        i++;
        if(s.current == s.target)
            continue;

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

        //printf("%d %d %f %f %f %u\n", dist, s.current, speed, s.speed_target, s.speed_coef, diff_ms);
        auto pkt = s.servo.move(Angle::deg(float(s.current)/100.f), std::chrono::milliseconds(diff_ms));
        send(pkt);
    }
    m_mutex.unlock();
}

float SmartServoBus::pos(uint8_t id) {
    lw::Packet pkt(id+1, lw::Command::SERVO_POS_READ);

    struct rx_response resp = { 0 };
    auto queue = xQueueCreate(1, sizeof(struct rx_response));
    send(pkt, queue, true);

    xQueueReceive(queue, &resp, portMAX_DELAY);
    vQueueDelete(queue);

    if(resp.size != 0x08) {
        printf("Servo %u\n", resp.size);
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

    struct tx_request req;
    struct rx_response resp;
    auto ticks_after_write = xTaskGetTickCount();
    while(true) {
        if(xQueueReceive(m_uart_queue, &req, portMAX_DELAY) != pdTRUE)
            continue;

        if(req.expect_response) {
            const auto diff = xTaskGetTickCount() - ticks_after_write;
            const auto delay = 50 / portTICK_PERIOD_MS;
            if(diff < delay)
                vTaskDelay(delay - diff);
        }

        uartSwitchToTx();
        //rbu::uart_write_bytes(m_uart, req.data, req.size);
        rbu::uart_tx_chars(m_uart, req.data, req.size);
        req.size = uartReceive((uint8_t*)req.data, sizeof(req.data));

        ticks_after_write = xTaskGetTickCount();

        //lw::Packet pkt((uint8_t*)req.data, req.size);
       // pkt.dump();
        if(req.size != 0 && req.expect_response) {
            resp.size = uartReceive(resp.data, sizeof(resp.data));
          // lw::Packet pkt(resp.data, resp.size);
           // pkt.dump();
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
    const TickType_t wait_period = 10 / portTICK_PERIOD_MS;
    const TickType_t timeout = 300 / portTICK_PERIOD_MS;

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
        }

        TickType_t waiting = 0;
        while(rbu::uart_get_buffered_data_len(m_uart, &avail) != ESP_OK || avail < need) {
            vTaskDelay(wait_period);
            waiting += wait_period;
            //printf("%u %u %u\n", oldsize, need, avail);
            //if(waiting >= timeout)
            //    return 0;
        }

        int res = rbu::uart_read_bytes(m_uart, buff + oldsize, need, 0);
        if(res < 0 || ((size_t)res) != need) {
            printf("Invalid servo packet read: %d, aborting.\n", res);
            abort();
        }
        bufsize += need;

        //printf("%u %u %u %X\n", oldsize, need, avail, (int)buff[oldsize]);

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