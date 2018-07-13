namespace rb {

/**
 * \brief Helper class for connecting to the wifi
 */
class WiFi {
public:
    //!< Connect to a wifi network with given ssid (name) and password
    static void connect(const char *ssid, const char *password);

    //!< Create a wifi network with given ssid (name) and password
    static void startAp(const char *ssid, const char *password, uint8_t channel = 6);

private:
    static void init();
};
};