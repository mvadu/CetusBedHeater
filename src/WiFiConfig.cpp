#include "WiFiConfig.hpp"

WiFiConfig WiFiConfig::ReadConfig()
{
    Preferences preferences;
    WiFiConfig cfg;
    preferences.begin(pref_namespace, true);
    cfg.ssid = preferences.getString("ssid", "esp");
    cfg.password = preferences.getString("pwd", "");
    cfg.clientName = preferences.getString("name", "esp");
    cfg.valid = preferences.getBool("valid");
    preferences.end();
    return cfg;
}

void WiFiConfig::SaveConfig(WiFiConfig cfg)
{
    Preferences preferences;
    preferences.begin(pref_namespace, false);
    preferences.putString("ssid", cfg.ssid);
    preferences.putString("pwd", cfg.password);
    preferences.putString("name", cfg.clientName);
    preferences.putBool("valid", cfg.valid);
    preferences.end();
}
