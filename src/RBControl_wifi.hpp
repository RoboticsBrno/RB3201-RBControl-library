namespace rb {

/**
 * \brief Helper class for connecting to the wifi
 */
class WiFi {
public:
    //!< Connect to a wifi network with given ssid (name) and password
    static void connect(const char *ssid, const char *password);
};
};