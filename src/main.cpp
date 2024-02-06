#include <Arduino.h>
#include <DNSServer.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const byte DNS_PORT = 53;
DNSServer dnsServer;

char sta_ssid[32] = {0};
char sta_password[64] = {0};

IPAddress local_IP(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const char *ssid = "";
const char *password = "";
const char *myName = "esp-temperature-sensor";

uint16_t wifiNameLableID;
uint16_t wifiPassLableID;
uint16_t buttonWifiUpdateID;

const PROGMEM char *MQTT_CLIENT_ID = "ESP_Temperature_Sensor";
const PROGMEM char *MQTT_SERVER_IP = "192.168.31.178";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char *MQTT_USER = "mqtt_sender";
const PROGMEM char *MQTT_PASSWORD = "1234";

// MQTT: topics
const PROGMEM char *MQTT_LIGHT_STATE_TOPIC = "outdoor/humidity/sensor1";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const uint16_t EEPROM_SIZE = 256;

unsigned long reconnectTime = 0;
void publishSensorState();
void publishHumidity();

const uint8_t MSG_BUFFER_SIZE = 60;
char msg_buffer[MSG_BUFFER_SIZE];

bool connectToWifi();
void initAp();
void EEPROMUpdate(int const address, uint8_t const value);
void saveWifiInfo(uint8_t wifiWork, char sta_ssid[32], char sta_password[64]);
void readWifiInfo();
void initDNS();
void connectToMQTT();

void mqtt_callback(char *p_topic, byte *p_payload, unsigned int p_length);

void wifiButtonUpdateCallback(Control *sender, int type)
{
    if (type == B_UP)
    {
        char *ssid;
        int stringLength;
        stringLength = ESPUI.getControl(wifiNameLableID)->value.length() + 1;
        ssid = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(wifiNameLableID)->value.toCharArray(ssid, stringLength);

        char *pass;
        stringLength = ESPUI.getControl(wifiPassLableID)->value.length() + 1;
        pass = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(wifiPassLableID)->value.toCharArray(pass, stringLength);
        saveWifiInfo(1, ssid, pass);
        readWifiInfo();
        connectToWifi();
        free(ssid);
        free(pass);
    }
}

void wifiNameTextCallback(Control *sender, int type)
{
}

void wifiPassTextCallback(Control *sender, int type)
{
}

void setup(void)
{
    ESPUI.setVerbosity(Verbosity::VerboseJSON);
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    Serial.println(EEPROM.read(0));
    if (EEPROM.read(0) != 1)
    {
        initAp();
    }
    else
    {
        Serial.print("\nTry to connect to existing network");
        readWifiInfo();
        if (!connectToWifi())
        {
            initAp();
        }
    }

    initDNS();

    wifiNameLableID = ESPUI.addControl(
        ControlType::Text, "WIFI Name", sta_ssid, ControlColor::Alizarin, Control::noParent, &wifiNameTextCallback);
    ESPUI.addControl(Max, "", "31", None, wifiNameLableID);

    wifiPassLableID = ESPUI.addControl(
        ControlType::Text, "WIFI Password", sta_password, ControlColor::Alizarin, Control::noParent, &wifiPassTextCallback);
    ESPUI.addControl(Max, "", "63", None, wifiPassLableID);

    buttonWifiUpdateID = ESPUI.addControl(
        ControlType::Button, "Update Wifi", "Press", ControlColor::Peterriver, Control::noParent, &wifiButtonUpdateCallback);

    ESPUI.begin("Dashboard");

    client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    client.setCallback(mqtt_callback);
    connectToMQTT();
}

void loop(void)
{
    dnsServer.processNextRequest();
}

bool connectToWifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setHostname(myName);
    WiFi.begin(sta_ssid, sta_password);

    int waittingTime = 0;
    do
    {
        delay(100);
        waittingTime++;
        Serial.print(".");
        if (waittingTime >= 1000)
        {
            waittingTime = 0;
            EEPROMUpdate(0, 0); // time out -> clear saved wifi information and return false
            EEPROM.commit();
            return false;
        }
    } while (WiFi.status() != WL_CONNECTED);
    return true;
}

void initAp()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, local_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(myName);
}

void initDNS()
{
    if (dnsServer.start(DNS_PORT, "*", local_IP))
    {
        Serial.println("dnsServer initialated");
    }
}

void EEPROMUpdate(int const address, uint8_t const value)
{
    if (EEPROM.read(address) != value)
    {
        EEPROM.write(address, value);
    }
}

void saveWifiInfo(uint8_t wifiWork, char sta_ssid[32], char sta_password[64])
{
    EEPROMUpdate(0, wifiWork);

    EEPROMUpdate(1, (byte)strlen(sta_ssid)); // write the length of ssid

    byte index = 2;
    for (int i = 0; i < (int)strlen(sta_ssid); i++)
    {
        EEPROMUpdate(index, sta_ssid[i]);
        index++;
    } // write ssid

    EEPROMUpdate(index, (byte)strlen(sta_password)); // write the length of password
    index++;

    for (int i = 0; i < (int)strlen(sta_password); i++)
    {
        EEPROMUpdate(index, sta_password[i]);
        index++;
    } // write password

    EEPROM.commit();
}

void readWifiInfo()
{
    byte length = EEPROM.read(1);

    byte index1 = 2;
    while (index1 < length + 2)
    {
        sta_ssid[index1 - 2] = EEPROM.read(index1);
        index1++;
    }
    length = EEPROM.read(index1);

    index1++;
    byte index2 = 0;
    while (index2 < length)
    {
        sta_password[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
}

void mqtt_callback(char *p_topic, byte *p_payload, unsigned int p_length)
{
}

void connectToMQTT()
{
    // Loop until connection lost
    while (!client.connected())
    {
        Serial.println("connect to MQTT server");
        // try to connect
        if (millis() - reconnectTime < 0)
        { // if internal timer(millis) leaked and reset after 50 days
            reconnectTime = 0;
        }
        if (millis() - reconnectTime > 3000)
        {
            if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
            {
                Serial.println("connected");

                // Once connected, publish an announcement...
                // publish the initial values
                publishSensorState();
                publishHumidity();

                // ... and resubscribe
            }
            else
            {
                Serial.print("Error:");
                Serial.println(client.state());
                Serial.println("reconnect in 3 seconds");
                reconnectTime = millis();
            }
        }
    }
}

void publishSensorState(){}
void publishHumidity()
{
    DynamicJsonDocument jsonDocument(20);
    jsonDocument["humidity"] = 60;
    String jsonString;
    serializeJson(jsonDocument, jsonString);
    jsonString.toCharArray(msg_buffer, 20);
    client.publish(MQTT_LIGHT_STATE_TOPIC, msg_buffer, true);
}