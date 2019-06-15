#include <esp_log.h>

#include "RBControl_linesensor.hpp"

#define TAG "RBControlLineSensor"

namespace rb {


LineSensor::LineSensor() {

}

LineSensor::~LineSensor() {

}

esp_err_t LineSensor::install(const LineSensor::Config& cfg) {
    bool expected = false;
    if(!m_installed.compare_exchange_strong(expected, true))
        return ESP_OK;

    esp_err_t ret;
    spi_bus_config_t buscfg = { 0 };
    buscfg.miso_io_num = cfg.pin_miso;
    buscfg.mosi_io_num = cfg.pin_mosi;
    buscfg.sclk_io_num = cfg.pin_sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    
    spi_device_interface_config_t devcfg = { 0 };
    devcfg.clock_speed_hz = cfg.freq;
    devcfg.mode = 0;
    devcfg.spics_io_num = cfg.pin_cs;
    devcfg.queue_size = LEDS;

    ret = spi_bus_initialize(cfg.spi_dev, &buscfg, 1);
    if(ret != ESP_OK) {
        m_installed = false;
        return ret;
    }

    ret = spi_bus_add_device(cfg.spi_dev, &devcfg, &m_spi);
    if(ret != ESP_OK) {
        spi_bus_free(cfg.spi_dev);
        m_installed = false;
        return ret;
    }
    return ESP_OK;
}

esp_err_t LineSensor::read(std::vector<uint16_t>& results, uint8_t leds_mask) {
    if(!m_installed) {
        return ESP_FAIL;
    }

    const size_t orig_size = results.size();
    int requested = 0;

    spi_transaction_t transactions[LEDS] = { 0 };
    for(int i = 0; i < LEDS; ++i) {
        if(((1 << i) & leds_mask) == 0)
            continue;

        auto &t = transactions[i];
        t.user = (void*)requested;
        t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
        t.length = 3 * 8;
        t.tx_data[0] = 1;
        t.tx_data[1] = (0 << 7) | ((i & 0x07) << 4);

        esp_err_t res = spi_device_queue_trans(m_spi, &transactions[i], 100);
        if(res != ESP_OK)
            return res;
        ++requested;
    }

    if(requested == 0)
        return ESP_OK;
    
    results.resize(orig_size + requested);

    spi_transaction_t *trans = NULL;
    for(int i = 0; i < requested; ++i) {
        esp_err_t res = spi_device_get_trans_result(m_spi, &trans, portMAX_DELAY);
        if(res != ESP_OK) {
            results.resize(orig_size);
            return res;
        }

        const int idx = (int)trans->user;
        results[orig_size + idx] = ((trans->rx_data[1] & 0x03) << 8) | trans->rx_data[2];
    }
    return ESP_OK;
}

int16_t LineSensor::readLine(bool white_line) {
    std::vector<uint16_t> vals;

    auto res = this->read(vals);
    if(res != ESP_OK) {
        ESP_LOGE(TAG, "read() failed: %d", res);
        return 0;
    }

    uint32_t weighted = 0;
    uint16_t sum = 0;

    for(size_t i = 0; i < vals.size(); ++i) {
        auto val = vals[i];
        if(white_line)
            val = MAX_VAL - val;

        if(val > 50) {
            weighted += uint32_t(val) * i * MAX_VAL;
            sum += val;
        }
    }

    if(sum == 0)
        return 0;

    constexpr int16_t middle = float(LEDS-1)/2 * MAX_VAL;
    return (weighted / sum) - middle;
}

}; // namespace rb
