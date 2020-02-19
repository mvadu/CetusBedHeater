#include <Preferences.h>

class WiFiConfig
{
private:
    static char constexpr const *pref_namespace = "WiFiCred";

public:
    String ssid;
    String password;
    String clientName;
    bool valid;
    static WiFiConfig ReadConfig();
    static void SaveConfig(WiFiConfig cfg);
};

enum WifiStatus
{
    Undefined = 0,
    NotConnected = 1,
    Connecting = 2,
    Connected = 3,
    ConnectFailed = 4
};