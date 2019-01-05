#pragma once

#include <driver/adc.h>
#include <driver/gpio.h>

namespace rb {

enum class MotorId : uint8_t {
    M1,
    M2,
    M3,
    M4,
    M5,
    M6,
    M7,
    M8,

    MAX,
};

inline MotorId operator++( MotorId& x ) { return x = MotorId(static_cast<uint8_t>(x) + 1); }

static const gpio_num_t IO0 = GPIO_NUM_0;
static const gpio_num_t IO2 = GPIO_NUM_2;
static const gpio_num_t IO12 = GPIO_NUM_12;
static const gpio_num_t IO4 = GPIO_NUM_4;
static const gpio_num_t IO5 = GPIO_NUM_5;
static const gpio_num_t IO13 = GPIO_NUM_13;
static const gpio_num_t IO14 = GPIO_NUM_14;
static const gpio_num_t IO15 = GPIO_NUM_15;
static const gpio_num_t IO16 = GPIO_NUM_16;
static const gpio_num_t IO17 = GPIO_NUM_17;
static const gpio_num_t IO18 = GPIO_NUM_18;
static const gpio_num_t IO19 = GPIO_NUM_19;
static const gpio_num_t IO21 = GPIO_NUM_21;
static const gpio_num_t IO25 = GPIO_NUM_25;
static const gpio_num_t IO26 = GPIO_NUM_26;
static const gpio_num_t IO27 = GPIO_NUM_27;
static const gpio_num_t IO32 = GPIO_NUM_32;
static const gpio_num_t IO33 = GPIO_NUM_33;
static const gpio_num_t IO34 = GPIO_NUM_34;
static const gpio_num_t IO35 = GPIO_NUM_35;

static const gpio_num_t RCKMOT = IO0;
static const gpio_num_t SERMOT = IO2;
static const gpio_num_t SCKMOT = IO12;

static const gpio_num_t ENC1A = IO21;
static const gpio_num_t ENC1B = IO19;
static const gpio_num_t ENC2A = IO18;
static const gpio_num_t ENC2B = IO5;
static const gpio_num_t ENC3A = IO17;
static const gpio_num_t ENC3B = IO16;
static const gpio_num_t ENC4A = IO15;
static const gpio_num_t ENC4B = IO13;
static const gpio_num_t ENC5A = IO4;
static const gpio_num_t ENC5B = IO15;
static const gpio_num_t ENC6A = IO27;
static const gpio_num_t ENC6B = IO26;
static const gpio_num_t ENC7A = IO25;
static const gpio_num_t ENC7B = IO33;
static const gpio_num_t ENC8A = IO32;
static const gpio_num_t ENC8B = IO35;

static const gpio_num_t POWER_OFF = IO32;
static const gpio_num_t BATT_REF = IO34;
static const adc1_channel_t BATT_ADC_CHANNEL = ADC1_CHANNEL_6;

static const gpio_num_t PIEZO_A = ENC7A;
static const gpio_num_t PIEZO_B = ENC7B;

static const gpio_num_t I2C_MASTER_SDA = GPIO_NUM_23;
static const gpio_num_t I2C_MASTER_SCL = GPIO_NUM_22;
static const int I2C_ADDR_EXPANDER = 0x20 | 0x01;

static const int EA0 = 0;
static const int EA1 = 1;
static const int EA2 = 2;
static const int EA3 = 3;
static const int EA4 = 4;
static const int EA5 = 5;
static const int EA6 = 6;
static const int EA7 = 7;

static const int EB0 = 8;
static const int EB1 = 9;
static const int EB2 = 10;
static const int EB3 = 11;
static const int EB4 = 12;
static const int EB5 = 13;
static const int EB6 = 14;
static const int EB7 = 15;

static const int SW1 = EB0;
static const int SW2 = EB1;
static const int SW3 = EB2;
static const int LED_RED = EB3;
static const int LED_YELLOW = EB4;
static const int LED_GREEN = EB5;
static const int LED_BLUE = EB6;
static const int POWER_OFF_EXPANDER = EB7;



} // namespace rb