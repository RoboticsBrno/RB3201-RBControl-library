#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <driver/periph_ctrl.h>

#include "RBControl_encoder.hpp"
#include "RBControl_pinout.hpp"
#include "RBControl_manager.hpp"

#define TAG "RbEncoder"

#define ENC_COUNT           static_cast<int>(MotorId::MAX)
#define PCNT_H_LIM_VAL      32767
#define PCNT_L_LIM_VAL     (-32768)
#define INC_PER_REVOLUTION  2       //PCNT increments per 1 engine revolution
#define ESP_INTR_FLAG_DEFAULT 0
#define ENC_DEBOUNCE_US 20          //[microseconds]
#define MAX_ENGINE_PERIOD_US 100000    //engine period limit separating zero result [us]
#define MIN_ENGINE_PERIOD_US 1000      //engine period limit separating zero results [us]

namespace rb {

static const pcnt_unit_t PCNT_UNITS[ENC_COUNT] = {
    PCNT_UNIT_0,
    PCNT_UNIT_1,
    PCNT_UNIT_2,
    PCNT_UNIT_3,
    PCNT_UNIT_4,
    PCNT_UNIT_5,
    PCNT_UNIT_6,
    PCNT_UNIT_7
};

const gpio_num_t ENCODER_PINS[ENC_COUNT*2] = {
    ENC1A, ENC1B,
    ENC2A, ENC2B,
    ENC3A, ENC3B,
    ENC4A, ENC4B,
    ENC5A, ENC5B,
    ENC6A, ENC6B,
    ENC7A, ENC7B,
    ENC8A, ENC8B,
};

PcntInterruptHandler& PcntInterruptHandler::get(Manager *manager) {
    static PcntInterruptHandler instance(manager);
    return instance;
}

PcntInterruptHandler::PcntInterruptHandler(Manager *manager) {
    pcnt_isr_register(isrHandler, manager, ESP_INTR_FLAG_DEFAULT, NULL);
}

PcntInterruptHandler::~PcntInterruptHandler() {
}

void PcntInterruptHandler::enable(int index) {
    pcnt_intr_enable((pcnt_unit_t)index);
}

void IRAM_ATTR PcntInterruptHandler::isrHandler(void *cookie) {
    auto *man = (Manager*)cookie;
    uint32_t intr_status = PCNT.int_st.val;
    for (int i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {

            const Manager::Event ev = {
                .type = Manager::EVENT_ENCODER_PCNT,
                .data = {
                    .encoderPcnt = {
                        .status = PCNT.status_unit[i].val,
                        .id = MotorId(i),
                    },
                },
            };

            PCNT.int_clr.val = BIT(i);
            if (man->queueFromIsr(&ev)) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

Encoder::Encoder(rb::Manager& man, rb::MotorId id) : m_manager(man), m_id(id) {
    if(m_id >= MotorId::MAX) {
        ESP_LOGE(TAG, "Invalid encoder index %d, using 0 instead.", (int)m_id);
        m_id = MotorId::M1;
    }

    m_counter = 0;
    m_counter_time_us_last = esp_timer_get_time();
    m_counter_time_us_diff = 0;

    m_target_direction = 0;
    m_target_callback = NULL;
    m_target = 0;
}

Encoder::~Encoder() {

}

void Encoder::install() {
    const auto encA = ENCODER_PINS[static_cast<int>(m_id)*2];
    const auto encB = ENCODER_PINS[static_cast<int>(m_id)*2 + 1];

    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_POSEDGE;  //ANYEDGE gives oscillating time differences in engine rotor half turns
        io_conf.pin_bit_mask = (1ULL << encA);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
    }
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;  //ANYEDGE gives oscillating time differences in engine rotor half turns
        io_conf.pin_bit_mask = (1ULL << encB);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
    }

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(encA, isrGpio, this);

    pcnt_init(PCNT_UNITS[static_cast<int>(m_id)], encA, encB);
}

void Encoder::pcnt_init(pcnt_unit_t pcntUnit, gpio_num_t GPIO_A, gpio_num_t GPIO_B)
{
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        GPIO_A,   //pulse_gpio_num
        GPIO_B,    //ctrl_gpio_num
        // What to do when control input is low or high?
        PCNT_MODE_KEEP,   //lctrl_mode  // Keep the primary counter mode if high
        PCNT_MODE_REVERSE,  //hctrl_mode   // Reverse counting direction if low
        // What to do on the positive / negative edge of pulse input?
        PCNT_COUNT_INC, //pos_mode   // Count up on the positive edge
        PCNT_COUNT_DEC, //neg_mode   // Keep the counter value on the negative edge
        // Set the maximum and minimum limit values to watch
        PCNT_H_LIM_VAL, //counter_h_lim
        PCNT_L_LIM_VAL, //counter_l_lim
        pcntUnit,   //unit
        PCNT_CHANNEL_0, //channel
    };
    pcnt_unit_config(&pcnt_config); //Initialize PCNT units

    /* Configure and enable the input filter */
    pcnt_set_filter_value(pcntUnit, 255);
    pcnt_filter_enable(pcntUnit);

    /* interrupts */
    PcntInterruptHandler::get(&m_manager).enable(pcntUnit);

    /* Set threshold 0 and 1 values and enable events to watch */
    /*pcnt_set_event_value(pcntUnit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(pcntUnit, PCNT_EVT_THRES_1);
    pcnt_set_event_value(pcntUnit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(pcntUnit, PCNT_EVT_THRES_0);*/
    /* Enable events on zero, maximum and minimum limit values */
    //pcnt_event_enable(pcntUnit, PCNT_EVT_ZERO);
    pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);
    pcnt_event_enable(pcntUnit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(pcntUnit);
    pcnt_counter_clear(pcntUnit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(pcntUnit);
}

void IRAM_ATTR Encoder::isrGpio(void* cookie)
{
    auto& enc = *((Encoder*)cookie);
    const Manager::Event ev = {
        .type = Manager::EVENT_ENCODER_EDGE,
        .data = {
            .encoderEdge = {
                .timestamp = esp_timer_get_time(),
                .id = enc.m_id,
                .pinLevel = (uint8_t)gpio_get_level(ENCODER_PINS[static_cast<int>(enc.m_id)*2 + 1]),
            },
        },
    };

    if(enc.m_manager.queueFromIsr(&ev)) {
        portYIELD_FROM_ISR();
    }
}

void Encoder::onEdgeIsr(int64_t timestamp, uint8_t pinLevel) {
    std::function<void(Encoder&)> callback;

    m_time_mutex.lock();
    if(timestamp > m_counter_time_us_last + ENC_DEBOUNCE_US) {
        m_counter_time_us_diff = timestamp - m_counter_time_us_last;
        if(pinLevel) {
            m_counter_time_us_diff = -m_counter_time_us_diff;
        }
        m_counter_time_us_last = timestamp;

        ESP_LOGD(TAG, "Edge %d %d %d", (int)m_id, value(), (int)pinLevel);

        if(m_target_direction != 0) {
            const auto val = value();
            if ((m_target_direction > 0 && val >= m_target) ||
                (m_target_direction < 0 && val <= m_target)) {
                m_manager.setMotors().power(m_id, 0).set(true);
                m_target_direction = 0;
                callback = m_target_callback;
            }
        }
    }
    m_time_mutex.unlock();

    if(callback)
        callback(*this);
}

void Encoder::onPcntIsr(uint32_t status) {
    m_time_mutex.lock();
    if (status & PCNT_STATUS_L_LIM_M) {
        m_counter.fetch_add(PCNT_L_LIM_VAL);
    } else if (status & PCNT_STATUS_H_LIM_M) {
        m_counter.fetch_add(PCNT_H_LIM_VAL);
    } else {
        ESP_LOGE(TAG, "invalid pcnt state 0x%08x", status);
    }
    m_time_mutex.unlock();
}

int32_t Encoder::value() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNITS[static_cast<int>(m_id)], &count);
    return m_counter.load() + count;
}

float Encoder::speed() {
    m_time_mutex.lock();
    const auto last = m_counter_time_us_last;
    const auto diff = m_counter_time_us_diff;
    m_time_mutex.unlock();

    if(esp_timer_get_time() > (last + MAX_ENGINE_PERIOD_US)) {
        return 0.f;
    } else if(abs(diff) < MIN_ENGINE_PERIOD_US) {
        return 0.f;
    } else {
        return 1000000.f / diff;
    }
}

void Encoder::driveToValue(int32_t positionAbsolute, uint8_t power, std::function<void(Encoder&)> callback) {
    if(power == 0)
        return;

    ESP_LOGD(TAG, "driveToValue %d %d %d %p", positionAbsolute, this->value(), power, callback);

    const auto current = this->value();
    if(current == positionAbsolute)
        return;

    m_time_mutex.lock();
    if(m_target_direction != 0 && m_target_callback) {
        m_target_callback(*this);
    }
    m_target_callback = callback;
    m_target = positionAbsolute;
    m_target_direction = (positionAbsolute > current ? 1 : -1);
    m_manager.motor(m_id)->power(static_cast<int8_t>(power) * m_target_direction);
    m_time_mutex.unlock();
}

void Encoder::drive(int32_t positionRelative, uint8_t power, std::function<void(Encoder&)> callback) {
    driveToValue(value() + positionRelative, power, callback);
}

};