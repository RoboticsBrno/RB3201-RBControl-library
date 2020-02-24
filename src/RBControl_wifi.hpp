#include <atomic>

#include <esp_event_loop.h>
#include <lwip/ip4_addr.h>

namespace rb {

class WiFiInitializer;
/**
 * \brief Helper class for connecting to the wifi
 */
class WiFi {
    friend class WiFiInitializer;
public:
    //!< Connect to a wifi network with given ssid (name) and password
    static void connect(const char *ssid, const char *password);

    //!< Create a wifi network with given ssid (name) and password
    static void startAp(const char *ssid, const char *password, uint8_t channel = 6);

    //!< Return current IP address of the ESP32
    static ip4_addr_t getIp() { return m_ip.load(); }

private:
    static void init();

    static esp_err_t eventHandler(void *ctx, system_event_t *event);

    static std::atomic<ip4_addr_t> m_ip;
};
};
