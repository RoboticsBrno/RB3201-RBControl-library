#include "RBControl_nvs.hpp"

namespace rb {

Nvs::Nvs(const char *name, const char *partiton)
    : m_dirty( false )
{
    ESP_ERROR_CHECK(initFlash());
    ESP_ERROR_CHECK(nvs_open_from_partition(partiton, name, NVS_READWRITE, &m_handle));
}

Nvs::~Nvs() {
    if ( m_dirty )
        commit();
}

esp_err_t Nvs::initFlash() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

bool Nvs::existsInt(const char *key) {
    int value;
    return nvs_get_i32(m_handle, key, &value) == ESP_OK;
}

int Nvs::getInt(const char *key) {
    int value;
    ESP_ERROR_CHECK(nvs_get_i32(m_handle, key, &value));
    return value;
}

void Nvs::writeInt(const char *key, int value) {
    ESP_ERROR_CHECK(nvs_set_i32(m_handle, key, value));
    m_dirty = true;
}

void Nvs::commit() {
    nvs_commit(m_handle);
    m_dirty = false;
}

} // namespace rb