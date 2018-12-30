#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
//#include <WebServer.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <rom/rtc.h>

#include "web_pages.h"
#include "DS18B20.h"
#include "PidPwm.h"

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
const char *rootUrl = "http://esp32.local/";
const char *hostName = "esp32";
TimerHandle_t fadeLedTimer;
void IRAM_ATTR resetModule();

//heater sensor and pwm out
PidPwm *pid;
DS18B20 *ds;

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
        if (request->host() != "esp32.local")
        {
            request->redirect(rootUrl);
            return;
        }
        serveStatic(request, WiFisetup_html, sizeof(WiFisetup_html));
    });

    server.on("/bootMsg", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        //RESET_REASON reason = rtc_get_reset_reason(0);
        // Serial.begin(115200);
        // Serial.printf("Starting Esp32 with IDF %s reset reason %d\n", ESP.getSdkVersion(), reason);
        response->printf("{\"boot_reason\":%d}", rtc_get_reset_reason(0));
        request->send(response);
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
}

void onNotFound(AsyncWebServerRequest *request)
{
    if (request->host() != "esp32.local")
        request->redirect(rootUrl);
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

void getHeaterStatus(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (pid->isRunning())
    {
        response->printf("{\"Heater\":true,\"Target\":%.3f,\"Current\":%.3f,\"DutyCycle\":%.3f}",
                         pid->getTarget(), pid->getCurrent(), pid->getOutputDS());
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
        float f = atof(request->getParam("temp", true)->value().c_str());
        pid->setTarget(f);
        response->print("OK");
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
            Serial.println("Starting heater");
            pid->begin();
            Serial.println("Started heater");
        }
        else
        {
            Serial.println("Shutdown heater");
            pid->shutdown();
        }
        response->print("OK");
    }
    else
    {
        response->print("Invalid");
    }
    request->send(response);
}

bool startMainserver()
{
    if (!WiFi.isConnected())
        return false;
    Serial.println("Starting main http server");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        serveStatic(request, esp32portal_html, sizeof(esp32portal_html));
    });

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

    server.onNotFound([](AsyncWebServerRequest *request) {
        onNotFound(request);
    });

    MDNS.begin(hostName);
    server.begin();
    // Add service to MDNS-SD
    MDNS.addService("_http", "_tcp", 80);
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
    WiFi.setHostname(hostName);
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
    {
        config.valid = true;
        delay(5000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        Serial.println("WiFi setup complete, rebooting..");
        delay(1000);
        ESP.restart();
    }
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
    // Serial.printf("[WiFi-event] event: %d timestamp:%d\n", event, millis());
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        // Serial.printf("WiFi connected IP address: %s duration %d\n", WiFi.localIP().toString().c_str(), millis());
        wifiConnected();
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        wifiStatus = WifiStatus::NotConnected;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        wifi_err_reason_t reason = (wifi_err_reason_t)info.disconnected.reason;
        switch (reason)
        {
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_802_1X_AUTH_FAILED:
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
    RESET_REASON reason = rtc_get_reset_reason(0);
    Serial.begin(115200);
    // Serial.printf("Starting Esp32 with IDF %s reset reason %d\n", ESP.getSdkVersion(), reason);
    if (reason == POWERON_RESET || reason == RTCWDT_RTC_RESET)
        resetConfig = false;

    if (!resetConfig)
        config = ReadConfig();
    Serial.printf("Found config %s, %s, %d \n", config.ssid.c_str(), config.password.c_str(), config.valid);

    pinMode(LED_BUILTIN, OUTPUT); // set the LED pin mode
    ledcAttachPin(LED_BUILTIN, 1);
    ledcSetup(1, 12000, 16);

    pinMode(13, OUTPUT); // powerup mosfet breakout
    digitalWrite(13, HIGH);

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
    ds = new DS18B20(15, DS18B20::Resolution::bit_11);
    PidPwm::PidParam param = {10, 1, 0};
    pid = new PidPwm(12, 2, 24000, 10, 35, param, [&]() {
        return ds->getTemperature();
    });
}

void loop()
{
    yield();
    delay(1000);
    Serial.printf("%.3f\t%.3f\n", pid->getCurrent(),pid->getOutputDS()*100);
}

//////////////////////////////////////////////////////////////////////////////////////////////
