#include "ChipInfo.hpp"
#include <DS18B20.h>
#include <htu21d.h>
#include <HX711.h>
#include "config.hpp"
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
#include <time.h>
#include <lwip/apps/sntp.h>

/// region variables
enum HeaterStatus
{
    TurnedOff,
    Heating,
    TimedOut
};

PidPwm *pid;
DS18B20 *ds;
HTU21D *htu;
HX711 *hx;

unsigned long heaterStartTs;
unsigned long timeoutTs;
long timeoutSec;
time_t uptime;

const uint8_t ds_Vcc = 23;
const uint8_t ds_Gnd = 19;
const uint8_t ds_data = 18;

//const uint8_t mosfet_Vcc = 15;
//const uint8_t mosfet_Gnd = 15;
const uint8_t pwm_Pin = 2;

//const uint8_t htu_Vcc = 15;
//const uint8_t htu_Gnd = 18;
const uint8_t htu_Scl = 22;
const uint8_t htu_Sda = 21;

//const uint8_t hx711_Vcc = 15;
//const uint8_t hx711_Gnd = 18;
const uint8_t hx711_Sda = 23;
const uint8_t hx711_Sck = 19;

WiFiConfig wifiConfig;
PIDConfig pidConfig;
PWMConfig pwmConfig;
LoadCellConfig hxConfig;

volatile RTC_NOINIT_ATTR bool resetConfig;

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
float maxTemp = 100;
const float criticalMax = 150;

ChipInfo *chipInfo;
WifiStatus wifiStatus = WifiStatus::Undefined;
IPAddress gateway;
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
String downcaseAndRemoveLocalPrefix(const String hostName);
void getChipInfo(AsyncWebServerRequest *request);
void onNotFound(AsyncWebServerRequest *request);
void serveStatic(AsyncWebServerRequest *request, const uint8_t *page, size_t size);
void getWifiStatus(AsyncWebServerRequest *request);
int ScanWiFi(AsyncResponseStream *response);
void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void setTimeout(AsyncWebServerRequest *request);
void setHeaterTemperature(AsyncWebServerRequest *request);
void getHeaterStatus(AsyncWebServerRequest *request);
void heaterPower(AsyncWebServerRequest *request);
void setConfig(AsyncWebServerRequest *request);
void getConfig(AsyncWebServerRequest *request);
void getAmbient(AsyncWebServerRequest *request);
void getWeight(AsyncWebServerRequest *request);
void getSensors(AsyncWebServerRequest *request);
bool syncTime();
time_t currentTime();

void setup()
{
    Serial.begin(115200);

    // put your setup code here, to run once:
    // id = (ESP.getEfuseMac() >> 16) & 0xFFFFFFFF;
    // hostName = "esp_";
    // hostName.concat(id);

    // delay(2000);
    // chipInfo = new ChipInfo();
    // if (chipInfo->reason == POWERON_RESET || chipInfo->reason == RTCWDT_RTC_RESET)
    //     resetConfig = false;

    // if (!resetConfig)
    // {
    //     ConfigManager::LoadConfig("WiFiConfig", wifiConfig);
    //     Serial.printf("Found wifiConfig %s, %s, %s, %d \n", wifiConfig.ssid.c_str(), wifiConfig.password.c_str(), wifiConfig.clientName.c_str(), wifiConfig.valid);
    // }

    // pinMode(LED_BUILTIN, OUTPUT); // set the LED pin mode
    // ledcAttachPin(LED_BUILTIN, 1);
    // ledcSetup(1, 12000, 16);

    // timerFadeLed = xTimerCreate(
    //     "ledFadeTimer",
    //     pdMS_TO_TICKS(500),
    //     pdTRUE,
    //     (void *)0,
    //     fadeLedCallback);

    // hw_timer_t *timer = timerBegin(0, 80, true);     //timer 0, div 80 with 8MHz, its 1us
    // timerAttachInterrupt(timer, &resetModule, true); //attach callback
    // timerAlarmWrite(timer, 100000, true);            //  1sec
    // timerAlarmEnable(timer);                         //enable interrupt
    // if (!wifiConfig.valid)
    // {
    //     Serial.println("No valid WiFi credentials found, starting WiFi wifiConfig wizard");
    //     SetupAccessPoint();
    // }
    // else
    // {

    //     hostName = wifiConfig.clientName;
    //     WiFi.mode(WIFI_STA);
    //     WiFi.disconnect();
    //     WiFi.setHostname(hostName.c_str());
    //     initWifiConnection();
    // }
    // // powerup mosfet breakout
    // // pinMode(mosfet_Vcc, OUTPUT);
    // // digitalWrite(mosfet_Vcc, HIGH);
    // // pinMode(mosfet_Gnd, OUTPUT);
    // // digitalWrite(mosfet_Gnd, LOW);

    // if (!ConfigManager::LoadConfig("PIDConfig", pidConfig))
    //     pidConfig.param = {2, 5, 0.5};

    // if (!ConfigManager::LoadConfig("PWMConfig", pwmConfig))
    // {
    //     pwmConfig.Temp = criticalMax;
    //     pwmConfig.minD = 0;
    //     pwmConfig.maxD = 100;
    // }

    //DS18B20PAR does not need a Power pin connected, DS18B20 goes to parasite power mode if VDD is grounded
    pinMode(ds_Gnd, OUTPUT);
    digitalWrite(ds_Gnd, LOW);
    pinMode(ds_Vcc, OUTPUT);
    digitalWrite(ds_Vcc, LOW);

    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);

    delay(1000);
    ds = new DS18B20(ds_data, DS18B20::Resolution::bit_10);

    if (ds != NULL && !ds->begin())
    {
        delete ds;
        ds = NULL;
        Serial.println("DS18B20 NOT found!\n");
    }
    else
    {
        pinMode(ds_data, OUTPUT);
        digitalWrite(ds_Vcc, LOW);

        // gpio_reset_pin((gpio_num_t)ds_data);
        // gpio_pad_select_gpio((gpio_num_t)ds_data);
        // gpio_set_direction((gpio_num_t)ds_data, gpio_mode_t::GPIO_MODE_INPUT_OUTPUT);
    }
    delay(100);
    // if (ds != NULL)
    // {
    //     Serial.printf("DS18B20 found! Temp: %f\n", ds->getTemperature());
    //     pid = new PidPwm(pwm_Pin, 2, 24000, 10, criticalMax, pidConfig.param, [&]() -> double {
    //         //limit the bed to 100°C, if its more than that report NAN, PidPwm should cut off output bringing down the temp
    //         double t = ds->getTemperature();

    //         return t;
    //     });
    //     pid->setComputeInterval(1000);
    //     pid->setLimitsPercentage(pwmConfig.minD, pwmConfig.maxD);
    //     pid->setTarget(targetTemp);
    // }

    // powerup HTU21D breakout
    // pinMode(htu_Vcc, OUTPUT);
    // digitalWrite(htu_Vcc, HIGH);
    // pinMode(htu_Gnd, OUTPUT);
    // digitalWrite(htu_Gnd, LOW);
    // delay(1000);

    // htu = new HTU21D(false);
    // if (htu != NULL && htu->begin(htu_Sda, htu_Scl))
    // {
    //     Serial.println("HTU21D found!\n");
    //     Serial.printf("Temp:%f, Humid:%f \n", htu->getTemperature(), htu->getHumidity());
    // }
    // else
    // {
    //     delete htu;
    //     htu = NULL;
    //     Serial.println("HTU21D NOT found!\n");
    // }

    // if (!ConfigManager::LoadConfig("LoadCellConfig", hxConfig))
    // {
    //     //TODO: calibrated value
    //     hxConfig.calibValuePerGram = 1;
    // }

    // pinMode(hx711_Vcc, OUTPUT);
    // digitalWrite(hx711_Vcc, HIGH);
    // if (htu_Gnd != hx711_Gnd)
    // {
    //     pinMode(hx711_Gnd, OUTPUT);
    //     digitalWrite(hx711_Gnd, LOW);
    // }
    // delay(500);
    // hx = new HX711();
    // if (hx != NULL && hx->begin(hx711_Sck, hx711_Sda))
    // {
    //     Serial.println("HX711 found!\n");
    //     Serial.printf("Load Cell Output:%lu \n", hx->getRawReading());
    //     hx->tare();
    //     Serial.printf("Load Cell Output after tare:%lu \n", hx->getRawReading());
    // }
    // else
    // {
    //     delete hx;
    //     hx = NULL;
    //     Serial.println("HX711 NOT found!\n");
    // }
}

void loop()
{
    if (ds != NULL)
    {
        Serial.printf("%f\n", ds->getTemperature());
    }
    else
    {
        bool b = gpio_get_level((gpio_num_t)ds_data);
        Serial.printf("%x\n", b);
        gpio_set_level((gpio_num_t)ds_data, 1);
    }

    if (pid != NULL && currentTime() > timeoutTs)
    {
        //Serial.println("Timed out");
        turnOffHeater();
    }
    yield();
    delay(1000);
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
              //Serial.printf("client host: %s\n", request->host().c_str());
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
                  wifiConfig.clientName = request->getParam("client", true)->value();
                  if (wifiConfig.clientName == NULL or wifiConfig.clientName == "")
                  {
                      wifiConfig.clientName = hostName;
                  }
              }
              if ((wifiStatus == WifiStatus::Undefined || wifiStatus == WifiStatus::ConnectFailed) &&
                  request->hasParam("ssid", true) && request->hasParam("password", true))
              {
                  wifiConfig.ssid = request->getParam("ssid", true)->value();
                  wifiConfig.password = request->getParam("password", true)->value();
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
                  wifiConfig.clientName = request->getParam("client", true)->value();
                  if (wifiConfig.clientName == NULL or wifiConfig.clientName == "")
                  {
                      wifiConfig.clientName = hostName;
                  }
              }
              Serial.printf("Saving wifiConfig %s, %s, %s, %d \n", wifiConfig.ssid.c_str(), wifiConfig.password.c_str(), wifiConfig.clientName.c_str(), wifiConfig.valid);
              ConfigManager::SaveConfig(wifiConfig, "WiFiConfig");
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

    server.on(
        "/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
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

    server.on("/setConfig", HTTP_POST, [](AsyncWebServerRequest *request) {
        setConfig(request);
    });

    server.on("/getConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
        getConfig(request);
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

    server.on("/getAmbient", HTTP_GET, [](AsyncWebServerRequest *request) {
        getAmbient(request);
    });

    server.on("/getWeight", HTTP_GET, [](AsyncWebServerRequest *request) {
        getWeight(request);
    });

    server.on("/getSensors", HTTP_GET, [](AsyncWebServerRequest *request) {
        getSensors(request);
    });

    DefaultHeaders::Instance().addHeader("server", "ESP32");
    server.begin();
    Serial.println("http server started");
    return true;
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
    if (!wifiConfig.valid)
        wifiConfig.valid = true;
    else
    {
        syncTime();
        startMainserver();
    }
    xTimerStop(timerFadeLed, 0);
    ledcWrite(1, 16000);
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.printf("[WiFi-event] event: %d timestamp:%lu\n", event, millis());
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        gateway = IPAddress(info.got_ip.ip_info.gw.addr);
        Serial.printf("WiFi connected IP address: %s ; From: %s, duration %lu\n",
                      WiFi.localIP().toString().c_str(), gateway.toString().c_str(), millis());
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

    Serial.printf("Trying to connect - %s, %s\n", wifiConfig.ssid.c_str(), wifiConfig.password.c_str());
    xTimerStart(timerFadeLed, 0);
    wifiStatus = WifiStatus::Connecting;
    WiFi.disconnect(true);  // delete old wifiConfig
    WiFi.persistent(false); //Avoid to store Wifi configuration in Flash
    WiFi.onEvent(WiFiEvent);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    return WiFi.begin(wifiConfig.ssid.c_str(), wifiConfig.password.c_str());
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
    //Serial.printf("Host: %s", request->host().c_str());
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

void getWifiStatus(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    response->printf("{\"status\":%d", wifiStatus);
    if (wifiStatus == WifiStatus::Connected)
    {
        response->printf(",\"IP\":\"%s\",\"AP\":\"%s\",\"Name\":\"%s\"",
                         WiFi.localIP().toString().c_str(), wifiConfig.ssid.c_str(), wifiConfig.clientName.c_str());
    }
    response->printf("}");
    request->send(response);
}

void getChipInfo(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "max-age=86400");
    response->printf("{");
    response->printf("\"FirmwareVersion\":\"%d.%d\",", firmwareInfo.Major, firmwareInfo.Minor);
    response->printf("\"FirmwareTime\":\"%s\",", firmwareInfo.BuildTime);
    response->printf("\"reason\":%d,", chipInfo->reason);
    response->printf("\"sdkVersion\":\"%s\",", chipInfo->sdkVersion);
    response->printf("\"chipVersion\":%d,", chipInfo->chipVersion);
    response->printf("\"coreCount\":%d,", chipInfo->coreCount);
    response->printf("\"featureBT\":%d,", chipInfo->featureBT);
    response->printf("\"featureBLE\":%d,", chipInfo->featureBLE);
    response->printf("\"featureWiFi\":%d,", chipInfo->featureWiFi);
    response->printf("\"internalFlash\":%d,", chipInfo->internalFlash);
    response->printf("\"flashSize\":%d,", chipInfo->flashSize);
    response->printf("\"UpTime\":%lu}", uptime);
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

void getHeaterStatus(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "max-age=1");
    if (ds != NULL && pid != NULL)
    {
        if (pid->isRunning())
        {
            response->printf("{\"Heater\":true,\"Target\":%.3f,\"Current\":%.3f,\"DutyCycle\":%.3f,\"StartTime\":%lu,\"Timeout\":%lu}",
                             pid->getTarget(), ds->getTemperature(), pid->getOutputDS(), heaterStartTs, timeoutSec);
        }
        else
        {
            response->printf("{\"Heater\":false,\"Current\":%.3f}", ds->getTemperature());
        }
    }
    else
    {
        response->printf("{\"Heater\":false,\"Current\":null}");
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
        if (pid != NULL && pid->setTarget(targetTemp))
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
    if (request->hasParam("timeoutSec", true))
    {
        long t = atol(request->getParam("timeoutSec", true)->value().c_str());
        if (t > 0)
        {
            timeoutSec += t;
            timeoutTs += currentTime() + timeoutSec;
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
            if (pid != NULL && pid->begin())
            {
                heaterStartTs = currentTime();
                //automatically turf off after 120 minutes
                timeoutSec = 120 * 60;
                timeoutTs = currentTime() + timeoutSec;
                response->print("OK");
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
    if (pid != NULL)
    {
        pid->shutdown();
        heaterStartTs = 0;
        timeoutTs = 0;
    }
}

void setConfig(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("text/plain");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (request->hasParam("config", true))
    {
        if (request->getParam("config", true)->value() == "PidParams")
        {
            if (request->hasParam("p", true) && request->hasParam("i", true) && request->hasParam("d", true))
            {

                pidConfig.param.proportionalGain = atof(request->getParam("p", true)->value().c_str());
                pidConfig.param.integralGain = atof(request->getParam("i", true)->value().c_str());
                pidConfig.param.derivativeGain = atof(request->getParam("d", true)->value().c_str());
                if (pid != NULL)
                    pid->setTuningParams(pidConfig.param);
                if (ConfigManager::SaveConfig(pidConfig, "PIDConfig"))
                    response->print("OK");
                else
                    response->print("Unable to Save");
            }
            else
            {
                response->print("Invalid");
            }
        }
        else if (request->getParam("config", true)->value() == "PwmParams")
        {
            if (request->hasParam("Temp", true) && request->hasParam("maxD", true) && request->hasParam("minD", true))
            {

                pwmConfig.Temp = atoi(request->getParam("Temp", true)->value().c_str());
                pwmConfig.maxD = atoi(request->getParam("maxD", true)->value().c_str());
                pwmConfig.minD = atoi(request->getParam("minD", true)->value().c_str());
                if (pwmConfig.maxD > pwmConfig.minD && pwmConfig.maxD <= 100 && pwmConfig.minD >= 0 && pwmConfig.Temp < criticalMax)
                {
                    if (pid != NULL)
                    {
                        pid->setLimitsPercentage(pwmConfig.minD, pwmConfig.maxD);
                        if (pwmConfig.Temp < pid->getTarget())
                            pid->setTarget(pwmConfig.Temp);
                    }
                    if (ConfigManager::SaveConfig(pwmConfig, "PWMConfig"))
                        response->print("OK");
                    else
                        response->print("Unable to Save");
                }
                else
                {
                    response->print("Invalid Param");
                }
            }
            else
            {
                response->print("Invalid Param");
            }
        }
        else if (request->getParam("config", true)->value() == "NetParams")
        {
            if (request->hasParam("ssid", true) && request->hasParam("password", true) && request->hasParam("client", true))
            {
                wifiConfig.clientName = request->getParam("client", true)->value();
                wifiConfig.ssid = request->getParam("ssid", true)->value();
                wifiConfig.password = request->getParam("password", true)->value();
                if (ConfigManager::SaveConfig(wifiConfig, "WiFiConfig"))
                    response->print("OK");
                else
                    response->print("Unable to Save");
                response->print("OK");
            }
            else
            {
                response->print("Invalid Param");
            }
        }
        else if (request->getParam("config", true)->value() == "LoadCellParams")
        {
            if (request->hasParam("calibValuePerGram", true))
            {

                hxConfig.calibValuePerGram = atol(request->getParam("calibValuePerGram", true)->value().c_str());
                if (ConfigManager::SaveConfig(hxConfig, "LoadCellConfig"))
                    response->print("OK");
                else
                    response->print("Unable to Save");
            }
            else
            {
                response->print("Invalid Param");
            }
        }
        else
        {
            response->printf("Invalid config:%s", request->getParam("config")->value().c_str());
        }
    }
    else
    {
        response->print("Missing Param:config");
    }
    request->send(response);
}

void getConfig(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    if (request->hasParam("config"))
    {
        if (request->getParam("config")->value() == "PidParams")
        {
            response->printf("{\"p\":%f,\"i\":%f,\"d\":%f}",
                             pidConfig.param.proportionalGain, pidConfig.param.integralGain, pidConfig.param.derivativeGain);
        }
        else if (request->getParam("config")->value() == "PwmParams")
        {
            response->printf("{\"Temp\":%u,\"maxD\":%u,\"minD\":%u}",
                             pwmConfig.Temp, pwmConfig.maxD, pwmConfig.minD);
        }
        else if (request->getParam("config")->value() == "NetParams")
        {
        }
        else if (request->getParam("config")->value() == "LoadCellParams")
        {
            response->printf("{\"calibValuePerGram\":%lu}",
                             hxConfig.calibValuePerGram);
        }
        else
        {
            response->printf("Invalid config:%s", request->getParam("config")->value().c_str());
        }
    }
    else
    {
        response->print("Missing Param:config");
    }
    request->send(response);
}

void getAmbient(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "max-age=10");
    if (htu != NULL)
    {
        response->printf("{\"Temperature\":%.2f,\"Humidity\":%.2f}", htu->getTemperature(), htu->getHumidity());
    }
    else
    {
        response->printf("{\"Temperature\":null,\"Humidity\":null}");
    }
    request->send(response);
}

void getWeight(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "max-age=10");
    if (hx != NULL)
    {
        unsigned long reading = hx->getRawReading();
        double weight = reading / hxConfig.calibValuePerGram;
        response->printf("{\"Weight\":%.2f,\"LoadCellReading\":%ld}", weight, reading);
    }
    else
    {
        response->printf("{\"Weight\":null,\"LoadCellReading\":null}");
    }
    request->send(response);
}

void getSensors(AsyncWebServerRequest *request)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Expires", "-1");
    response->printf("{\"bedHeater\":%s,\"ambientTemp\":%s,\"weighingScale\":%s}",
                     ds != NULL ? "true" : "false", htu != NULL ? "true" : "false", hx != NULL ? "true" : "false");
    request->send(response);
}

//Syncs with ntp server
bool syncTime()
{
    const char *sntpServer = gateway.toString().c_str(); //"pool.ntp.org";
    const long minEpoch = 1577836800;                    //epoch for 1/1/2020

    Serial.printf("Default Time %ld - millis: %lu\n", uptime, millis());

    time(&uptime);

    if (uptime < minEpoch)
    {
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, (char *)sntpServer);
        sntp_init();
        // wait for time to be set
        unsigned int t = 5 * 1000;
        unsigned int s = millis();
        while (uptime < minEpoch && (t > millis() - s))
        {
            delay(5);
            time(&uptime);
        }
        Serial.printf("Time received %ld - millis: %lu\n", uptime, millis());
    }

    if (uptime > minEpoch)
    {
        uptime -= millis() / 1000;
        return true;
    }
    else
    {
        uptime = minEpoch;
        return false;
    }
}

time_t currentTime()
{
    return uptime + millis() / 1000;
}