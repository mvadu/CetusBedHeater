#include "ChipInfo.hpp"
#include <DS18B20.h>
#include "WiFiConfig.hpp"
#include "web_pages.h"
#include "versionInfo.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
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
#include <PidPwm.hpp>

/// region variables
PidPwm *pid;
DS18B20 *ds;
WiFiConfig config;
volatile RTC_NOINIT_ATTR bool resetConfig;
unsigned long heaterStartTs;
unsigned long timeoutTs;
bool heaterTimedOut;
float timeout;
/////// Handle long press of the button to reset WiFi credentials
const uint8_t resetBtn = 0; //GPIO0, do.it board has a button attached
volatile long btnPressed;
const unsigned long interval = 30;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TimerHandle_t timerFadeLed;

unsigned long id;
String hostName;

char constexpr const *http_username = "cetus";
char constexpr const *http_password = "CetusAdmin";
float targetTemp = 60;
ChipInfo *chipInfo;
WifiStatus wifiStatus = WifiStatus::Undefined;
AsyncWebServer server(80);
///

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
wl_status_t initWifiConnection();
void fadeLedCallback(TimerHandle_t xTimer);
void connectionFailed();
void wifiConnected();
void IRAM_ATTR resetModule();
void turnOffHeater();
void SetupAccessPoint();
bool startMainserver();
void getChipInfo(AsyncWebServerRequest *request);
void onNotFound(AsyncWebServerRequest *request);
void serveStatic(AsyncWebServerRequest *request, const uint8_t *page, size_t size);
void getWifiStatus(AsyncWebServerRequest *request);
int ScanWiFi(AsyncResponseStream *response);
void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void setTimeout(AsyncWebServerRequest *request);
void setHeaterTemperature(AsyncWebServerRequest *request);
void getHeaterStatus(AsyncWebServerRequest *request);
void setPidParam(AsyncWebServerRequest *request);
void getPidParam(AsyncWebServerRequest *request);
void heaterPower(AsyncWebServerRequest *request);

void setup()
{
    // put your setup code here, to run once:
    id = (ESP.getEfuseMac() >> 16) & 0xFFFFFFFF;
    hostName = "esp_";
    hostName.concat(id);
    Serial.begin(115200);
    chipInfo = new ChipInfo();
    if (chipInfo->reason == POWERON_RESET || chipInfo->reason == RTCWDT_RTC_RESET)
        resetConfig = false;

    if (!resetConfig)
    {
        config = WiFiConfig::ReadConfig();
        Serial.printf("Found config %s, %s, %s, %d \n", config.ssid.c_str(), config.password.c_str(), config.clientName.c_str(), config.valid);
    }

    pinMode(LED_BUILTIN, OUTPUT); // set the LED pin mode
    ledcAttachPin(LED_BUILTIN, 1);
    ledcSetup(1, 12000, 16);

    timerFadeLed = xTimerCreate(
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

        hostName = config.clientName;
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        WiFi.setHostname(hostName.c_str());
        initWifiConnection();
    }
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
    if (pid->isRunning() && !heaterTimedOut && millis() > timeoutTs)
    {
        turnOffHeater();
        heaterTimedOut = true;
    }
    delay(1000);
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

void fadeLedCallback(TimerHandle_t xTimer)
{
    static uint16_t dc = 0;
    noInterrupts();
    ledcWrite(1, dc);
    dc += 500;
}

void connectionFailed()
{
    wifiStatus = WifiStatus::ConnectFailed;
    xTimerStop(timerFadeLed, 0);
    ledcWrite(1, 0);
}

void wifiConnected()
{
    wifiStatus = WifiStatus::Connected;
    if (!config.valid)
        config.valid = true;
    else
        startMainserver();
    xTimerStop(timerFadeLed, 0);
    ledcWrite(1, 16000);
}

void getChipInfo(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "max-age=86400");
    response->printf("{");
    response->printf("\"FirmwareVersion\":\"%d.%d\",", firmwareInfo.Major, firmwareInfo.Minor);
    response->printf("\"FirmwareTime\":%llu,", firmwareInfo.BuildTime);
    response->printf("\"reason\":%d,", chipInfo->reason);
    response->printf("\"sdkVersion\":\"%s\",", chipInfo->sdkVersion);
    response->printf("\"chipVersion\":%d,", chipInfo->chipVersion);
    response->printf("\"coreCount\":%d,", chipInfo->coreCount);
    response->printf("\"featureBT\":%d,", chipInfo->featureBT);
    response->printf("\"featureBLE\":%d,", chipInfo->featureBLE);
    response->printf("\"featureWiFi\":%d,", chipInfo->featureWiFi);
    response->printf("\"internalFlash\":%d,", chipInfo->internalFlash);
    response->printf("\"flashSize\":%d}", chipInfo->flashSize);

    request->send(response);
}

String downcaseAndRemoveLocalPrefix(const String hostName)
{
    String host = hostName;
    host.toLowerCase();
    host.replace(".local", "");
    return host;
}

void onNotFound(AsyncWebServerRequest *request)
{
    StreamString s;
    Serial.printf("Host: %s", request->host().c_str());
    if (downcaseAndRemoveLocalPrefix(request->host()) != hostName)
    {
        s.printf("http://%s.local/", hostName.c_str());
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

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.printf("[WiFi-event] event: %d timestamp:%lu\n", event, millis());
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.printf("WiFi connected IP address: %s duration %lu\n", WiFi.localIP().toString().c_str(), millis());
        wifiConnected();
        break;
    case SYSTEM_EVENT_STA_START:
        //set sta hostname here
        WiFi.setHostname(hostName.c_str());
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.printf("WiFi lost IP");
        wifiStatus = WifiStatus::NotConnected;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        wifi_err_reason_t reason = (wifi_err_reason_t)info.disconnected.reason;
        switch (reason)
        {
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_802_1X_AUTH_FAILED:
            Serial.printf("WiFi Auth failed");
            connectionFailed();
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

wl_status_t initWifiConnection()
{
    if (WiFi.status() == WL_CONNECTED)
        return WL_CONNECTED;

    Serial.printf("Trying to connect - %s, %s\n", config.ssid.c_str(), config.password.c_str());
    xTimerStart(timerFadeLed, 0);
    wifiStatus = WifiStatus::Connecting;
    WiFi.disconnect(true);  // delete old config
    WiFi.persistent(false); //Avoid to store Wifi configuration in Flash
    WiFi.onEvent(WiFiEvent);
    //WiFi.setHostname(config.clientName.c_str());
    return WiFi.begin(config.ssid.c_str(), config.password.c_str());
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

void SetupAccessPoint()
{
    DNSServer dnsServer;
    const char *password = "espressif";
    bool exitConfigWizard = false;

    server.onNotFound([](AsyncWebServerRequest *request) {
        onNotFound(request);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
              Serial.printf("client host: %s\n", request->host().c_str());
              if (downcaseAndRemoveLocalPrefix(request->host()) != downcaseAndRemoveLocalPrefix(hostName))
              {
                  onNotFound(request);
              }
              serveStatic(request, WiFisetup_html, sizeof(WiFisetup_html));
          })
        .setFilter(ON_AP_FILTER);

    server.on("/getChipInfo", HTTP_GET, [](AsyncWebServerRequest *request) {
              getChipInfo(request);
          })
        .setFilter(ON_AP_FILTER);

    server.on("/getApList", HTTP_GET, [](AsyncWebServerRequest *request) {
              AsyncResponseStream *response = request->beginResponseStream("application/json");
              response->addHeader("Cache-Control", "max-age=-1");
              ScanWiFi(response);
              request->send(response);
          })
        .setFilter(ON_AP_FILTER);

    server.on("/connect", HTTP_POST, [](AsyncWebServerRequest *request) {
              AsyncResponseStream *response = request->beginResponseStream("text/plain");
              response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
              response->addHeader("Expires", "-1");
              if (request->hasParam("client", true))
              {
                  config.clientName = request->getParam("client", true)->value();
                  if (config.clientName == NULL or config.clientName == "")
                  {
                      config.clientName = hostName;
                  }
              }
              if ((wifiStatus == WifiStatus::Undefined || wifiStatus == WifiStatus::ConnectFailed) &&
                  request->hasParam("ssid", true) && request->hasParam("password", true))
              {
                  config.ssid = request->getParam("ssid", true)->value();
                  config.password = request->getParam("password", true)->value();
                  wifiStatus = WifiStatus::NotConnected;
              }
              response->printf("%d", wifiStatus);
              request->send(response);
          })
        .setFilter(ON_AP_FILTER);

    server.on("/getWifiStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
              getWifiStatus(request);
          })
        .setFilter(ON_AP_FILTER);

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
              if (request->hasParam("client", true))
              {
                  Serial.printf("Got client name %s", request->getParam("client", true)->value().c_str());
                  config.clientName = request->getParam("client", true)->value();
                  if (config.clientName == NULL or config.clientName == "")
                  {
                      config.clientName = hostName;
                  }
              }
              Serial.printf("Saving config %s, %s, %s, %d \n", config.ssid.c_str(), config.password.c_str(), config.clientName.c_str(), config.valid);
              WiFiConfig::SaveConfig(config);
              request->send(204); //success, No data
          })
        .setFilter(ON_AP_FILTER);

    server.on("/exit", HTTP_GET, [&](AsyncWebServerRequest *request) {
              exitConfigWizard = true;
              request->send(204); //success, No data
          })
        .setFilter(ON_AP_FILTER);

    WiFi.softAP(hostName.c_str(), password);
    Serial.printf("AP Name: %s - Gateway IP: %s\n", hostName.c_str(), WiFi.softAPIP().toString().c_str());
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.begin();

    while (!exitConfigWizard)
    {
        yield();
        dnsServer.processNextRequest();
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

bool startMainserver()
{
    if (!WiFi.isConnected())
        return false;
    Serial.println("Starting main http server");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        serveStatic(request, main_portal_html, sizeof(main_portal_html));
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

    server.on("/setTimeout", HTTP_POST, [](AsyncWebServerRequest *request) {
        setTimeout(request);
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

    DefaultHeaders::Instance().addHeader("server", "ESP32");
    server.begin();
    Serial.println("http server started");
    return true;
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

void getHeaterStatus(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (pid->isRunning())
    {
        response->printf("{\"Heater\":true,\"Target\":%.3f,\"Current\":%.3f,\"DutyCycle\":%.3f,\"RunTime\":%lu,\"Timeout\":%lu}",
                         pid->getTarget(), ds->getTemperature(), pid->getOutputDS(), (millis() - heaterStartTs), (timeoutTs - millis()));
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

void setTimeout(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("text/plain");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (request->hasParam("timeout", true))
    {
        timeout = atof(request->getParam("timeout", true)->value().c_str());
        if (timeout > 0)
        {
            timeoutTs = millis() + (timeout * 3600.0 * 1000);
            response->print("OK");
        }
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
                //automatically turf off after 2 hours
                timeout = 2;
                timeoutTs = millis() + (timeout * 3600.0 * 1000);
                response->print("OK");
                heaterTimedOut = false;
            }
            else
                response->print("Invalid");
        }
        else
        {
            Serial.println("Shutdown heater");
            turnOffHeater();
            response->print("OK");
        }
    }
    else
    {
        response->print("Invalid");
    }
    request->send(response);
}

void turnOffHeater()
{
    pid->shutdown();
    heaterStartTs = 0;
    timeoutTs = 0;
}