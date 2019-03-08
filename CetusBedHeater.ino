#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
//#include <WebServer.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <rom/rtc.h>
#include <Update.h>
#include <StreamString.h>
#include "soc/efuse_reg.h"
#include <driver/gpio.h>

#include "web_pages.h"
#include "src/DS18B20/DS18B20.h"
#include "src/PIDPwm/PidPwm.h"
#include "versionInfo.h"

class ChipInfo
{
  public:
    uint8_t reason;
    const char *sdkVersion;
    uint8_t chipVersion;
    uint8_t coreCount;
    uint8_t featureBT;
    uint8_t featureBLE;
    uint8_t featureWiFi;
    bool internalFlash;
    uint8_t flashSize;

    ChipInfo()
    {
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        reason = rtc_get_reset_reason(0);
        sdkVersion = ESP.getSdkVersion();
        chipVersion = chip_info.revision;
        //chipVersion = REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15;
        coreCount = chip_info.cores;
        featureBT = (chip_info.features & CHIP_FEATURE_BT) ? 1 : 0;
        featureBLE = (chip_info.features & CHIP_FEATURE_BLE) ? 1 : 0;
        featureWiFi = 1;
        internalFlash = (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? 1 : 0;
        flashSize = spi_flash_get_chip_size() / (1024 * 1024);
    }
};
ChipInfo chipInfo;

class WiFiConfig
{
  public:
    String ssid;
    String password;
    bool valid;
};
volatile RTC_NOINIT_ATTR bool resetConfig;

WiFiConfig config;
const char *pref_namespace = "WiFiCred";
const char *rootUrl = "cetus3d.local";
const char *hostName = "cetus3d";
const char *http_username = "cetus";
const char *http_password = "CetusAdmin";
float targetTemp = 60;

TimerHandle_t fadeLedTimer;
void IRAM_ATTR resetModule();

//heater sensor and pwm out
PidPwm *pid;
DS18B20 *ds;

unsigned long heaterStartTs;
/////// Handle long press of the button to reset WiFi credentials
const uint8_t resetBtn = 0; //GPIO0, do.it board has a button attached
volatile long btnPressed;
const unsigned long interval = 30;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
///////////////////////

enum WifiStatus
{
    Undefined = 0,
    NotConnected = 1,
    Connecting = 2,
    Connected = 3,
    ConnectFailed = 4
};

WifiStatus wifiStatus = WifiStatus::Undefined;

AsyncWebServer server(80);

WiFiConfig ReadConfig()
{
    Preferences preferences;
    WiFiConfig cnfg;
    preferences.begin(pref_namespace, true);
    cnfg.ssid = preferences.getString("ssid", "");
    cnfg.password = preferences.getString("pwd", "");
    cnfg.valid = preferences.getBool("valid");
    preferences.end();
    return cnfg;
}

void SaveConfig(WiFiConfig cnfg)
{
    Preferences preferences;
    preferences.begin(pref_namespace, false);
    preferences.putString("ssid", cnfg.ssid);
    preferences.putString("pwd", cnfg.password);
    preferences.putBool("valid", cnfg.valid);
    preferences.end();
}

int ScanWiFi(AsyncResponseStream *response)
{
    //scan state in Async mode, default value is -2/scan failed
    int numberOfNetworks = WiFi.scanComplete();
    //start a new scan if previous one failed
    if (numberOfNetworks == WIFI_SCAN_FAILED)
    {
        WiFi.scanNetworks(true);
        Serial.println("Start scanning for networks");
        response->printf("{\"accessPoints\":[]}");
    }
    else if (numberOfNetworks)
    {
        Serial.printf("Number of networks found: %d \n", numberOfNetworks);
        response->printf("{\"accessPoints\":[");
        for (int i = 0; i < numberOfNetworks; i++)
        {
            response->printf("{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 0 : 1);
            if (i + 1 < numberOfNetworks)
                response->printf(",");
        }
        response->printf("]}");
        //delete last scan result
        WiFi.scanDelete();
        if (numberOfNetworks == WIFI_SCAN_FAILED)
        {
            WiFi.scanNetworks(true);
            Serial.println("Refresh networks");
        }
        //assume previous attempt failed
        wifiStatus = WifiStatus::ConnectFailed;
    }
    return numberOfNetworks;
}

bool SetupAccessPoint()
{
    IPAddress apIP(192, 168, 1, 1);
    DNSServer dnsServer;

    uint64_t id = ESP.getEfuseMac() & 0xFFFFFFFFFFFF;
    char apName[22] = {0};
    sprintf(apName, "Espressif-%8X", id);
    const char *password = "espressif";

    bool exitConfigWizard = false;

    server.onNotFound([](AsyncWebServerRequest *request) {
        onNotFound(request);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->host() != rootUrl)
        {
            request->redirect(rootUrl);
            return;
        }
        serveStatic(request, WiFisetup_html, sizeof(WiFisetup_html));
    });

    server.on("/getChipInfo", HTTP_GET, [](AsyncWebServerRequest *request) {
        getChipInfo(request);
    });

    server.on("/getApList", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->addHeader("Cache-Control", "max-age=-1");
        ScanWiFi(response);
        request->send(response);
    });

    server.on("/connect", HTTP_POST, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("text/plain");
        response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        response->addHeader("Expires", "-1");
        if ((wifiStatus == WifiStatus::Undefined || wifiStatus == WifiStatus::ConnectFailed) &&
            request->hasParam("ssid", true) && request->hasParam("pwd", true))
        {
            config.ssid = request->getParam("ssid", true)->value();
            config.password = request->getParam("pwd", true)->value();
            wifiStatus = WifiStatus::NotConnected;
        }
        response->printf("%d", wifiStatus);
        request->send(response);
    });

    server.on("/getWifiStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
        getWifiStatus(request);
    });

    server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request) {
        SaveConfig(config);
        request->send(204); //success, No data
    });

    server.on("/exit", HTTP_GET, [&](AsyncWebServerRequest *request) {
        exitConfigWizard = true;
        request->send(204); //success, No data
    });

    WiFi.softAP(apName, password);
    Serial.printf("AP Name: %s\n", apName);
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.begin();

    while (!exitConfigWizard)
    {
        dnsServer.processNextRequest();
        delay(10);
        yield();
        if (wifiStatus == WifiStatus::NotConnected)
        {
            initWifiConnection();
        }
    }
    if (wifiStatus == WifiStatus::Connected)
    {
        delay(2000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        Serial.println("WiFi setup complete, rebooting..");
        delay(1000);
        ESP.restart();
    }
}

void onNotFound(AsyncWebServerRequest *request)
{
    StreamString s;
    Serial.printf("Host: %s", request->host().c_str());
    if (request->host() != rootUrl)
    {
        s.printf("http://%s/", rootUrl);
        request->redirect(s);
    }
    else
        request->send(404); //Sends 404 File Not Found
}

void serveStatic(AsyncWebServerRequest *request, const uint8_t *page, size_t size)
{
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", page, size);
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
}

void getWifiStatus(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    response->printf("{\"status\":%d", wifiStatus);
    if (wifiStatus == WifiStatus::Connected)
    {
        response->printf(",\"IP\":\"%s\",\"AP\":\"%s\"", WiFi.localIP().toString().c_str(), config.ssid.c_str());
    }
    response->printf("}");
    request->send(response);
}

void getChipInfo(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "max-age=86400");
    response->printf("{");
    response->printf("\"FirmwareVersion\":\"%d.%d\",", firmwareInfo.Major,firmwareInfo.Minor);
    response->printf("\"FirmwareTime\":%llu,", firmwareInfo.BuildTime);
    response->printf("\"reason\":%d,", chipInfo.reason);
    response->printf("\"sdkVersion\":\"%s\",", chipInfo.sdkVersion);
    response->printf("\"chipVersion\":%d,", chipInfo.chipVersion);
    response->printf("\"coreCount\":%d,",chipInfo.coreCount);
    response->printf("\"featureBT\":%d,",chipInfo.featureBT);
    response->printf("\"featureBLE\":%d,",chipInfo.featureBLE);
    response->printf("\"featureWiFi\":%d,",chipInfo.featureWiFi);
    response->printf("\"internalFlash\":%d,",chipInfo.internalFlash);
    response->printf("\"flashSize\":%d}", chipInfo.flashSize);
                     
    request->send(response);
}

void getHeaterStatus(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (pid->isRunning())
    {
        response->printf("{\"Heater\":true,\"Target\":%.3f,\"Current\":%.3f,\"DutyCycle\":%.3f,\"RunTime\":%lu}",
                         pid->getTarget(), pid->getCurrent(), pid->getOutputDS(), (millis() - heaterStartTs) / 1000);
    }
    else
    {
        response->printf("{\"Heater\":false,\"Current\":%.3f}", ds->getTemperature());
    }
    request->send(response);
}

void setHeaterTemperature(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("text/plain");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (request->hasParam("temp", true))
    {
        targetTemp = atof(request->getParam("temp", true)->value().c_str());
        if (pid->setTarget(targetTemp))
            response->print("OK");
        else
            response->print("Invalid");
    }
    else
    {
        response->print("Invalid");
    }
    request->send(response);
}
void setPidParam(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("text/plain");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (request->hasParam("p", true) && request->hasParam("i", true) && request->hasParam("d", true))
    {
        PidPwm::PidParam param;
        param.proportionalGain = atof(request->getParam("p", true)->value().c_str());
        param.integralGain = atof(request->getParam("i", true)->value().c_str());
        param.derivativeGain = atof(request->getParam("d", true)->value().c_str());
        pid->setTuningParams(param);
        response->print("OK");
    }
    else
    {
        response->print("Invalid");
    }
    request->send(response);
}
void getPidParam(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    PidPwm::PidParam p = pid->getTuningParams();
    response->printf("{\"Proportional\":%f,\"Integral\":%f,\"Derivitive\":%f}",
                     p.proportionalGain, p.integralGain, p.derivativeGain);
    request->send(response);
}

void heaterPower(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("text/plain");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");

    if (request->hasParam("turnOn", true))
    {
        Serial.print(request->getParam("turnOn", true)->value());
        if (request->getParam("turnOn", true)->value() == "1")
        {
            if (pid->begin())
            {
                heaterStartTs = millis();
                response->print("OK");
            }
            else
                response->print("Invalid");
        }
        else
        {
            Serial.println("Shutdown heater");
            pid->shutdown();
            heaterStartTs = 0;
        }
        response->print("OK");
    }
    else
    {
        response->print("Invalid");
    }
    request->send(response);
}

void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        Serial.printf("UploadStart: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        { //start with max available size
            Update.printError(Serial);
        }
    }
    /* flashing firmware to ESP*/
    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
    }
    if (final)
    {
        if (Update.end(true))
        { //true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", index + len);
        }
        else
        {
            Update.printError(Serial);
        }
    }
}

bool startMainserver()
{
    if (!WiFi.isConnected())
        return false;
    Serial.println("Starting main http server");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        serveStatic(request, esp32portal_html, sizeof(esp32portal_html));
    });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->authenticate(http_username, http_password))
            return request->requestAuthentication();
        serveStatic(request, config_html, sizeof(config_html));
    });

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        StreamString str;
        Update.printError(str);
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->addHeader("Connection", "close");
        response->printf("{\"UpdateStatus\":%d,\"Message\":\"%s\"}",
                         !Update.hasError(), Update.hasError() ? str.c_str() : "Success");
        request->send(response);
        delay(1000);
        ESP.restart();
    },
              onUpload);

    server.on("/getWifiStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
        getWifiStatus(request);
    });

    server.on("/getHeaterStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
        getHeaterStatus(request);
    });

    server.on("/setHeaterTemperature", HTTP_POST, [](AsyncWebServerRequest *request) {
        setHeaterTemperature(request);
    });

    server.on("/setPidParam", HTTP_POST, [](AsyncWebServerRequest *request) {
        setPidParam(request);
    });

    server.on("/getPidParam", HTTP_GET, [](AsyncWebServerRequest *request) {
        getPidParam(request);
    });

    server.on("/heaterPower", HTTP_POST, [](AsyncWebServerRequest *request) {
        heaterPower(request);
    });

    server.on("/getChipInfo", HTTP_GET, [](AsyncWebServerRequest *request) {
        getChipInfo(request);
    });

    server.on("/restart", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200);
        ESP.restart();
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        onNotFound(request);
    });

    MDNS.begin(hostName);
    DefaultHeaders::Instance().addHeader("server", "ESP32");
    server.begin();
    // Add service to MDNS-SD
    //MDNS.addService("_http", "_tcp", 80);
    MDNS.addService("http", "tcp", 80);
}

bool initWifiConnection()
{
    if (WiFi.status() == WL_CONNECTED)
        return true;

    Serial.printf("Trying to connect - %s, %s\n", config.ssid.c_str(), config.password.c_str());
    xTimerStart(fadeLedTimer, 0);
    wifiStatus = WifiStatus::Connecting;
    WiFi.disconnect(true);  // delete old config
    WiFi.persistent(false); //Avoid to store Wifi configuration in Flash
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(config.ssid.c_str(), config.password.c_str());
}

void connectionFailed()
{
    wifiStatus = WifiStatus::ConnectFailed;
    xTimerStop(fadeLedTimer, 0);
    ledcWrite(1, 0);
}

void wifiConnected()
{
    wifiStatus = WifiStatus::Connected;
    if (!config.valid)
        config.valid = true;
    else
        startMainserver();
    xTimerStop(fadeLedTimer, 0);
    ledcWrite(1, 16000);
}

void IRAM_ATTR resetModule()
{
    portENTER_CRITICAL_ISR(&timerMux);
    if (digitalRead(resetBtn) == HIGH)
    {                   // assumes btn is LOW when pressed
        btnPressed = 0; // btn not pressed so reset clock
    }
    else if ((++btnPressed) >= interval)
    {
        resetConfig = true;
        ESP.restart();
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.printf("[WiFi-event] event: %d timestamp:%d\n", event, millis());
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.printf("WiFi connected IP address: %s duration %d\n", WiFi.localIP().toString().c_str(), millis());
        wifiConnected();
        break;
    case SYSTEM_EVENT_STA_START:
        //set sta hostname here
        WiFi.setHostname(hostName);
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.printf("WiFi lost IP");
        wifiStatus = WifiStatus::NotConnected;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        wifi_err_reason_t reason = (wifi_err_reason_t)info.disconnected.reason;
        switch (reason)
        {
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_802_1X_AUTH_FAILED:
            Serial.printf("WiFi Auth failed");
            connectionFailed();
            break;
        }
        break;
    }
}

void fadeLedCallback(TimerHandle_t xTimer)
{
    static uint16_t dc = 0;
    noInterrupts();
    ledcWrite(1, dc);
    dc += 500;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);

    if (chipInfo.reason == POWERON_RESET || chipInfo.reason == RTCWDT_RTC_RESET)
        resetConfig = false;

    if (!resetConfig)
        config = ReadConfig();
    Serial.printf("Found config %s, %s, %d \n", config.ssid.c_str(), config.password.c_str(), config.valid);

    pinMode(LED_BUILTIN, OUTPUT); // set the LED pin mode
    ledcAttachPin(LED_BUILTIN, 1);
    ledcSetup(1, 12000, 16);

    fadeLedTimer = xTimerCreate(
        "ledFadeTimer",
        pdMS_TO_TICKS(500),
        pdTRUE,
        (void *)0,
        fadeLedCallback);

    hw_timer_t *timer = timerBegin(0, 80, true);     //timer 0, div 80 with 8MHz, its 1us
    timerAttachInterrupt(timer, &resetModule, true); //attach callback
    timerAlarmWrite(timer, 100000, true);            //  1sec
    timerAlarmEnable(timer);                         //enable interrupt
    if (!config.valid)
    {
        Serial.println("No valid WiFi credentials found, starting WiFi config wizard");
        SetupAccessPoint();
    }
    else
    {
        WiFi.mode(WIFI_STA);
        initWifiConnection();
    }
    // powerup sensor breakout
    //    pinMode(14, PULLUP);
    //    digitalWrite(14, HIGH);

    // gpio_pad_select_gpio(GPIO_NUM_14);
    // gpio_set_direction(GPIO_NUM_14, gpio_mode_t::GPIO_MODE_OUTPUT);
    // gpio_pullup_en(GPIO_NUM_14);

    // powerup mosfet breakout
    pinMode(25, PULLUP);
    digitalWrite(25, HIGH);
    pinMode(27, OUTPUT);
    digitalWrite(27, LOW);
    delay(500);

    ds = new DS18B20(12, 14, DS18B20::Resolution::bit_10);
    PidPwm::PidParam param = {2, 5, 0.5};
    pid = new PidPwm(26, 2, 24000, 10, 100, param, [&]() -> double {
        //limit the bed to 100°C, if its more than that report NAN, PidPwm should cut off output bringing down the temp
        double t = ds->getTemperature();
        // for (int i = 0; i < 5; i++)
        // {
        //     double t1 = ds->getTemperature();
        //     if (!isnan(t1))
        //     {
        //         t += t1;
        //         t /= 2.0;
        //     }
        //     delay(50);
        // }

        if (t > 100)
            return NAN;
        //hard off if we are already at the set Target
        if (isnan(targetTemp) || t > targetTemp)
            return NAN;
        return t;
    });
    pid->setTarget(targetTemp);
    pid->setComputeInterval(1000);
}

void loop()
{
    yield();
    delay(1000);
    //Serial.printf("%.3f\t%.3f\n", pid->getCurrent(),pid->getOutputDS()*100);
}

//////////////////////////////////////////////////////////////////////////////////////////////
