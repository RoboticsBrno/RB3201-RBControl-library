#pragma once

#include <driver/adc.h>

namespace rb {

static const gpio_num_t IO0 = GPIO_NUM_0;
static const gpio_num_t IO2 = GPIO_NUM_2;
static const gpio_num_t IO12 = GPIO_NUM_12;

static const gpio_num_t RCKMOT = IO0;
static const gpio_num_t SERMOT = IO2;
static const gpio_num_t SCKMOT = IO12;

static const gpio_num_t IO25 = GPIO_NUM_25;
static const gpio_num_t IO32 = GPIO_NUM_32;
static const gpio_num_t IO33 = GPIO_NUM_33;
static const gpio_num_t IO34 = GPIO_NUM_34;
static const gpio_num_t IO35 = GPIO_NUM_35;

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



} // namespace rb