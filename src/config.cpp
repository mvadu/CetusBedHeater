#include "config.hpp"
#include <Preferences.h>

bool ConfigManager::SaveConfig(const Serializable &config, String name)
{
    bool retVal = true;
    size_t data_len = config.size();
    char *buffer = (char *)malloc(data_len * sizeof(char));
    if (buffer != NULL)
    {
        if (config.serialize(buffer))
        {
            Serial.printf("Saving %s, len: %d\n", name.c_str(), data_len);
            Preferences preferences;
            if (preferences.begin(name.c_str(), false))
            {
                //preferences.putULong("size", data_len);
                preferences.putBytes("blob", buffer, data_len);
                preferences.end();
            }
            else
                retVal = false;
        }
        else
            retVal = false;
        delete[] buffer;
    }
    else
        retVal = false;
    return retVal;
}

bool ConfigManager::LoadConfig(String name, Serializable &config)
{
    bool retVal = true;
    size_t data_len;
    Preferences preferences;
    if (preferences.begin(name.c_str(), true))
    {
        data_len = preferences.getBytesLength("blob");
        if (data_len > 0)
        {
            Serial.printf("Reading %s, len: %d\n", name.c_str(), data_len);

            char *buffer = (char *)malloc(data_len * sizeof(char));
            if (buffer != NULL)
            {
                preferences.getBytes("blob", buffer, data_len);
                if (!config.deserialize(buffer))
                    retVal = false;
                delete[] buffer;
            }
            else
                retVal = false;
            preferences.end();
        }
        else
        {
            preferences.end();
            retVal = false;
        }
    }
    else
        retVal = false;
    return retVal;
}

size_t WiFiConfig::size() const
{
    size_t len = 0;
    //we will save the actual length in preference. length() does not include NULL at the end
    len = sizeof(size_t) + ssid.length() + sizeof('\0');
    len += sizeof(size_t) + password.length() + sizeof('\0');
    len += sizeof(size_t) + clientName.length() + sizeof('\0');
    len += sizeof(valid);
    return len;
}

bool WiFiConfig::serialize(char *dataOut) const
{
    char *p = dataOut;
    size_t len = 0;

    len = ssid.length() + sizeof('\0');
    memcpy(p, &len, sizeof(size_t));
    p += sizeof(size_t);
    memcpy(p, ssid.c_str(), len);
    p += len;

    len = password.length() + sizeof('\0');
    memcpy(p, &len, sizeof(size_t));
    p += sizeof(size_t);
    memcpy(p, password.c_str(), len);
    p += len;

    len = clientName.length() + sizeof('\0');
    memcpy(p, &len, sizeof(size_t));
    p += sizeof(size_t);
    memcpy(p, clientName.c_str(), len);
    p += len;

    memcpy(p, &valid, sizeof(valid));
    return true;
}

bool WiFiConfig::deserialize(const char *dataIn)
{
    const char *p = dataIn;
    size_t len = 0;

    memcpy(&len, p, sizeof(size_t));

    if (len == 0)
        return false;
    p += sizeof(size_t);
    this->ssid = String(p);
    p += len;

    memcpy(&len, p, sizeof(size_t));
    if (len > 0)
    {
        p += sizeof(size_t);
        this->password = String(p);
        p += len;
    }

    memcpy(&len, p, sizeof(size_t));
    if (len > 0)
    {
        p += sizeof(size_t);
        this->clientName = String(p);
        p += len;
    }
    memcpy(&valid, p, sizeof(valid));
    return true;
}

size_t PIDConfig::size() const
{
    return sizeof(this->param);
}

bool PIDConfig::serialize(char *dataOut) const
{
    memcpy(dataOut, &param, sizeof(param));
    return true;
}

bool PIDConfig::deserialize(const char *dataIn)
{
    memcpy(&param, dataIn, sizeof(param));
    return true;
}

size_t PWMConfig::size() const
{
    return sizeof(this->Temp) + sizeof(this->maxD) + sizeof(this->minD);
}

bool PWMConfig::serialize(char *dataOut) const
{
    char *p = dataOut;
    memcpy(p, &Temp, sizeof(Temp));
    p += sizeof(Temp);

    memcpy(p, &maxD, sizeof(maxD));
    p += sizeof(maxD);

    memcpy(p, &minD, sizeof(minD));
    p += sizeof(minD);
    return true;
}

bool PWMConfig::deserialize(const char *dataIn)
{
    const char *p = dataIn;

    memcpy(&Temp, p, sizeof(Temp));
    p += sizeof(Temp);

    memcpy(&maxD, p, sizeof(maxD));
    p += sizeof(maxD);

    memcpy(&minD, p, sizeof(minD));
    p += sizeof(minD);
    return true;
}
