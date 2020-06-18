#include <PidPwm.hpp>

class Serializable
{
public:
    virtual size_t size() const = 0;
    virtual bool serialize(char *dataOut) const = 0;
    virtual bool deserialize(const char *dataIn) = 0;
};

class ConfigManager
{
public:
    static bool SaveConfig(const Serializable &config, String name);
    static bool LoadConfig(String name, Serializable &config);
};

class WiFiConfig : public Serializable
{
public:
    String ssid;
    String password;
    String clientName;
    bool valid;
    size_t size() const override;
    bool serialize(char *dataOut) const override;
    bool deserialize(const char *dataIn) override;
};

class PIDConfig : public Serializable
{

public:
    PidPwm::PidParam param;
    size_t size() const override;
    bool serialize(char *dataOut) const override;
    bool deserialize(const char *dataIn) override;
};

class PWMConfig : public Serializable
{
public:
    uint8_t minD, maxD, Temp;
    size_t size() const override;
    bool serialize(char *dataOut) const override;
    bool deserialize(const char *dataIn) override;
};

enum WifiStatus
{
    Undefined = 0,
    NotConnected = 1,
    Connecting = 2,
    Connected = 3,
    ConnectFailed = 4
};

class LoadCellConfig : public Serializable
{
public:
    unsigned long calibValuePerGram;
    size_t size() const override;
    bool serialize(char *dataOut) const override;
    bool deserialize(const char *dataIn) override;
};

class SensorConfig : public Serializable
{
public:
    size_t size() const override;
    bool serialize(char *dataOut) const override;
    bool deserialize(const char *dataIn) override;

    uint8_t ds_Vcc, ds_data;
    uint8_t mosfet_Vcc, mosfet_Gnd, pwm_Pin;
    uint8_t htu_Vcc, htu_Gnd, htu_Scl, htu_Sda;
    uint8_t hx711_Vcc, hx711_Sda, hx711_Sck;
};