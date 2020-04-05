#pragma once

#include <chrono>
#include <driver/uart.h>
#include <esp_log.h>
#include <soc/io_mux_reg.h>
#include <stdexcept>
#include <vector>

#include "RBControl_angle.hpp"

namespace lw {

enum class Command {
    SERVO_MOVE_TIME_WRITE = 1,
    SERVO_MOVE_TIME_READ,
    SERVO_MOVE_TIME_WAIT_WRITE = 7,
    SERVO_MOVE_TIME_WAIT_READ,
    SERVO_MOVE_START = 11,
    SERVO_MOVE_STOP,
    SERVO_ID_WRITE,
    SERVO_ID_READ,
    SERVO_ANGLE_OFFSET_ADJUST = 17,
    SERVO_ANGLE_OFFSET_WRITE,
    SERVO_ANGLE_OFFSET_READ,
    SERVO_ANGLE_LIMIT_WRITE,
    SERVO_ANGLE_LIMIT_READ,
    SERVO_VIN_LIMIT_WRITE,
    SERVO_VIN_LIMIT_READ,
    SERVO_TEMP_MAX_LIMIT_WRITE,
    SERVO_TEMP_MAX_LIMIT_READ,
    SERVO_TEMP_READ,
    SERVO_VIN_READ,
    SERVO_POS_READ,
    SERVO_OR_MOTOR_MODE_WRITE,
    SERVO_OR_MOTOR_MODE_READ,
    SERVO_LOAD_OR_UNLOAD_WRITE,
    SERVO_LOAD_OR_UNLOAD_READ,
    SERVO_LED_CTRL_WRITE,
    SERVO_LED_CTRL_READ,
    SERVO_LED_ERROR_WRITE,
    SERVO_LED_ERROR_READ
};

using Id = uint8_t;

struct Packet {
    Packet() = default;

    Packet(const uint8_t* data, int len) {
        for (int i = 0; i < len; i++) {
            _data.push_back(data[i]);
        }
    }

    template <typename... Args>
    Packet(Id id, Command c, Args... data) {
        _buildHeader();
        _data.push_back(id);
        _data.push_back(3);
        _data.push_back(static_cast<uint8_t>(c));
        _pushData(data...);
        _data.push_back(_checksum(_data));
    }

    static Packet move(Id id, uint16_t position, uint16_t time) {
        return Packet(id, Command::SERVO_MOVE_TIME_WRITE,
            position & 0xFF, position >> 8,
            time & 0xFF, time >> 8);
    }

    static Packet limitAngle(Id id, uint16_t low, uint16_t high) {
        return Packet(id, Command::SERVO_ANGLE_LIMIT_WRITE,
            low & 0xFF, low >> 8, high & 0xFF, high >> 8);
    }

    static Packet setId(Id id, Id newId) {
        return Packet(id, Command::SERVO_ID_WRITE, newId);
    }

    static Packet getId(Id id) {
        return Packet(id, Command::SERVO_ID_READ);
    }

    void _buildHeader() {
        _data.push_back(0x55);
        _data.push_back(0x55);
    }

    void _pushData() {}

    template <typename... Args>
    void _pushData(uint8_t d, Args... data) {
        _data.push_back(d);
        _data[3]++;
        _pushData(data...);
    }

    static uint8_t _checksum(const std::vector<uint8_t>& data,
        int offset = 2, int end_offset = 0) {
        uint8_t sum = 0;
        for (int i = offset; i < data.size() - end_offset; i++)
            sum += data[i];
        return ~sum;
    }

    int size() const {
        if (_data.size() < 4)
            return -1;
        return _data[3];
    }

    bool valid() const {
        if (_data.size() < 6)
            return false;
        uint8_t c = _checksum(_data, 2, 1);
        if (c != _data.back())
            return false;
        if (size() + 3 != _data.size())
            return false;
        return true;
    }

    void dump() {
        printf("[");
        bool first = true;
        for (auto x : _data) {
            if (!first)
                printf(", ");
            first = false;
            printf("%02X", (int)x);
        }
        printf("]\n");
    }

    std::vector<uint8_t> _data;
};

class Servo {
public:
    static int posFromDeg(float angle) {
        return angle * 1000 / 240;
    }

    // Move servo to given position (in degree) in given time (in milliseconds)
    static Packet move(Id id, rb::Angle pos, std::chrono::milliseconds t) {
        float position = pos.deg();
        int time = t.count();
        if (position < 0 || position > 240)
            ESP_LOGE("LX16A", "Position out of range");
        if (time < 0)
            ESP_LOGE("LX16A", "Time is negative");
        if (time > 30000)
            ESP_LOGE("LX16A", "Time is out of range");
        auto p = Packet::move(id, Servo::posFromDeg(position), time);
        return p;
    }

    static Packet move(Id id, rb::Angle pos) {
        float position = pos.deg();
        if (position < 0 || position > 240)
            ESP_LOGE("LX16A", "Position out of range");
        auto p = Packet::move(id, Servo::posFromDeg(position), 0);
        return p;
    }

    // Set limits for the movement
    static Packet limit(Id id, rb::Angle b, rb::Angle t) {
        int bottom = b.deg();
        int top = t.deg();
        if (bottom < 0 || bottom > 240)
            ESP_LOGE("LX16A", "Bottom limit out of range");
        if (top < 0 || top > 240)
            ESP_LOGE("LX16A", "Top limit out of range");
        auto p = Packet::limitAngle(id, Servo::posFromDeg(bottom), Servo::posFromDeg(top));
        return p;
    }

    static Packet setId(Id oldId, Id newId) {
        if (newId >= 254) {
            ESP_LOGE("LX16A", "Invalid ID specified");
        }
        auto p = Packet::setId(oldId, newId);
        return p;
    }
};

} // namespace lw