#include <Arduino.h>
#include <DNSServer.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const byte DNS_PORT = 53;
DNSServer dnsServer;

char *sta_ssid;
char *sta_password;

IPAddress local_IP(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const char *myName = "esp-temperature-sensor";

uint16_t wifiNameTextID;
uint16_t wifiPassTextID;
uint16_t buttonWifiUpdateID;

uint16_t mqttServerIPTextID;
uint16_t mqttServerPortTextID;
uint16_t mqttUserTextID;
uint16_t mqttPassTextID;
uint16_t mqttTopicTextID;
uint16_t buttonMqttUpdateID;

// char *MQTT_SERVER_IP = "192.168.31.178";
// uint16_t MQTT_SERVER_PORT = 1883;
// char *MQTT_USER = "mqtt_sender";
// char *MQTT_PASSWORD = "1234";
char *mqttServerIp;
char *mqttServerPort;
char *mqttUser;
char *mqttPassword;
char *mqttTopic;

bool isMqttConnected = false;

// MQTT: topics
// const PROGMEM char *MQTT_SENSOR_STATE_TOPIC = "outdoor/humidity/sensor1";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const uint16_t EEPROM_SIZE = 512;

unsigned long reconnectTime = 0;
unsigned long updateTime = 0;
unsigned long updateInterval = 600000;

void publishSensorState();
void publishHumidity();

const uint8_t MSG_BUFFER_SIZE = 60;
char msg_buffer[MSG_BUFFER_SIZE];

bool connectToWifi();
void initAp();
void EEPROMUpdate(int const address, uint8_t const value);
void saveWifiInfo(byte wifiWork, char *sta_ssid, char *sta_password);
void readWifiInfo();
void initDNS();
void connectToMqtt();
void readMqttInfo();
void saveMqttInfo(uint8_t isAvailable, char *ip, char *port, char *user, char *pass, char *topic);

void addWifiControl();
void addMqttControl();

void mqtt_callback(char *p_topic, byte *p_payload, unsigned int p_length);

void wifiButtonUpdateCallback(Control *sender, int type)
{
    if (type == B_UP)
    {
        Serial.println("WIFI Name");
        Serial.println(ESPUI.getControl(wifiNameTextID)->value);
        char *ssid;
        int stringLength;
        stringLength = ESPUI.getControl(wifiNameTextID)->value.length() + 1;
        ssid = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(wifiNameTextID)->value.toCharArray(ssid, stringLength);

        char *pass;
        stringLength = ESPUI.getControl(wifiPassTextID)->value.length() + 1;
        pass = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(wifiPassTextID)->value.toCharArray(pass, stringLength);

        saveWifiInfo(1, ssid, pass);
        readWifiInfo();
        connectToWifi();
        free(ssid);
        free(pass);
    }
}

void textCallback(Control *sender, int type)
{
    // This callback is needed to handle the changed values, even though it doesn't do anything itself.
}

void mqttButtonUpdateCallback(Control *sender, int type)
{
    if (type == B_UP)
    {
        Serial.println("mqtt button pressed");

        char *ip;
        int stringLength;
        stringLength = ESPUI.getControl(mqttServerIPTextID)->value.length() + 1;
        ip = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(mqttServerIPTextID)->value.toCharArray(ip, stringLength);

        Serial.println("ip");
        Serial.println(ESPUI.getControl(mqttServerIPTextID)->value);

        char *port;
        stringLength = ESPUI.getControl(mqttServerPortTextID)->value.length() + 1;
        port = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(mqttServerPortTextID)->value.toCharArray(port, stringLength);

        char *user;
        stringLength = ESPUI.getControl(mqttUserTextID)->value.length() + 1;
        user = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(mqttUserTextID)->value.toCharArray(user, stringLength);

        char *pass;
        stringLength = ESPUI.getControl(mqttPassTextID)->value.length() + 1;
        pass = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(mqttPassTextID)->value.toCharArray(pass, stringLength);

        char *topic;
        stringLength = ESPUI.getControl(mqttTopicTextID)->value.length() + 1;
        topic = (char *)calloc(stringLength, sizeof(char));
        ESPUI.getControl(mqttTopicTextID)->value.toCharArray(topic, stringLength);

        saveMqttInfo(1, ip, port, user, pass, topic);
        readMqttInfo();
        connectToMqtt();
        free(ip);
        free(port);
        free(user);
        free(pass);
        free(topic);
    }
}

void setup(void)
{
    ESPUI.setVerbosity(Verbosity::VerboseJSON);
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);

    setCpuFrequencyMhz(80);

    Serial.println(EEPROM.read(0));
    if (EEPROM.read(0) != 1)
    {
        initAp();
    }
    else
    {

        readWifiInfo();
        Serial.print("\nwifi read");
        if (!connectToWifi())
        {
            initAp();
            Serial.print("\ninitialed");
        }
    }
    Serial.print("\nconnected");
    initDNS();
    readMqttInfo();
    addWifiControl();
    addMqttControl();

    ESPUI.begin("Dashboard");
    Serial.print("\nbegin!");
    client.setServer(mqttServerIp, atoi(mqttServerPort));
    client.setCallback(mqtt_callback);
    if (EEPROM.read(100) == 1)
    {
        connectToMqtt();
    }
}

void loop(void)
{
    
    if(((millis() - updateTime) > updateInterval)||(updateTime < 0)){
        publishHumidity();
        updateTime = millis();
    }
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

void saveWifiInfo(byte isAvailable, char *sta_ssid, char *sta_password)
{
    byte index = 0;
    EEPROMUpdate(0, isAvailable);
    index++;
    EEPROMUpdate(1, (byte)strlen(sta_ssid)); // write the length of ssid
    index++;

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

void generalCallback(Control *sender, int type) {}

void readWifiInfo()
{
    if (sta_ssid != NULL)
    {
        free(sta_ssid);
        sta_ssid = NULL;
    }
    if (sta_password != NULL)
    {
        free(sta_password);
        sta_password = NULL;
    }
    byte index1 = 1;
    byte length = EEPROM.read(index1); // start from 0 since index 1 is availability
    sta_ssid = (char *)malloc((length + 1) * sizeof(char));
    index1++;
    byte index2 = 0;
    while (index2 < length)
    {
        sta_ssid[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    sta_ssid[index2] = '\0';

    length = EEPROM.read(index1);
    sta_password = (char *)malloc((length + 1) * sizeof(char));
    index1++;
    index2 = 0;
    while (index2 < length)
    {
        sta_password[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    sta_password[index2] = '\0';
}

void mqtt_callback(char *p_topic, byte *p_payload, unsigned int p_length)
{
}

void connectToMqtt()
{
    byte retryTimes = 0;
    Serial.println("connect to MQTT server");
    // Loop until connection lost
    while (!client.connected() && (retryTimes < 10))
    {
        Serial.print(".");
        // try to connect

        if (client.connect(myName, mqttUser, mqttPassword))
        {
            Serial.println("connected");

            // Once connected, publish an announcement...
            // publish the humidity values
            publishHumidity();
            return;
        }
        else
        {
            Serial.print("Error:");
            Serial.println(client.state());
            Serial.println("reconnect in 1 second");
            reconnectTime = millis();
        }
        delay(3000);
        retryTimes++;
    }
    EEPROMUpdate(100, 0);
    EEPROM.commit();
}

void publishHumidity()
{
    StaticJsonDocument<50> jsonDocument;
    int minValue = 2138; //min
    int maxValue = 8000; //max
    int range = maxValue - minValue; //range
    int sensorValue = analogRead(32);
    float humidity = 100 - (((float)(sensorValue - minValue) / range) * 100);


    char humidityStr[6]; //
    dtostrf(humidity, 5, 1, humidityStr);

    jsonDocument["humidity"] = humidityStr;
    String jsonString;
    serializeJson(jsonDocument, jsonString);


    jsonString.toCharArray(msg_buffer, 50);
    client.publish(mqttTopic, msg_buffer, true);


    Serial.print("humidity: ");
    Serial.println(humidityStr);
}

void addWifiControl()
{
    auto wifiTab = ESPUI.addControl(Tab, "", "WiFi information");

    wifiNameTextID = ESPUI.addControl(
        ControlType::Text, "WIFI Name", "", ControlColor::Alizarin, wifiTab, textCallback);
    ESPUI.addControl(Max, "", "31", None, wifiNameTextID);

    wifiPassTextID = ESPUI.addControl(
        ControlType::Text, "WIFI Password", "", ControlColor::Alizarin, wifiTab, textCallback);
    ESPUI.addControl(Max, "", "63", None, wifiPassTextID);

    buttonWifiUpdateID = ESPUI.addControl(
        ControlType::Button, "Update Wifi", "Confirm", ControlColor::Peterriver, wifiTab, &wifiButtonUpdateCallback);
}

void addMqttControl()
{
    auto mqttTab = ESPUI.addControl(Tab, "", "MQTT information");

    mqttServerIPTextID = ESPUI.addControl(
        ControlType::Text, "MQTT server IP", "", ControlColor::Alizarin, mqttTab, textCallback);
    ESPUI.addControl(Max, "", "31", None, mqttServerIPTextID);

    mqttServerPortTextID = ESPUI.addControl(
        ControlType::Text, "MQTT server port", "", ControlColor::Alizarin, mqttTab, textCallback);
    ESPUI.addControl(Max, "", "31", None, mqttServerPortTextID);

    mqttUserTextID = ESPUI.addControl(
        ControlType::Text, "MQTT user ID", "", ControlColor::Alizarin, mqttTab, textCallback);
    ESPUI.addControl(Max, "", "31", None, mqttUserTextID);

    mqttPassTextID = ESPUI.addControl(
        ControlType::Text, "MQTT user password", "", ControlColor::Alizarin, mqttTab, textCallback);
    ESPUI.addControl(Max, "", "31", None, mqttPassTextID);

    mqttTopicTextID = ESPUI.addControl(
        ControlType::Text, "MQTT topic", "", ControlColor::Alizarin, mqttTab, textCallback);
    ESPUI.addControl(Max, "", "63", None, mqttTopicTextID);

    buttonMqttUpdateID = ESPUI.addControl(
        ControlType::Button, "Update MQTT", "Confirm", ControlColor::Peterriver, mqttTab, &mqttButtonUpdateCallback);
}

void saveMqttInfo(uint8_t isAvailable, char *ip, char *port, char *user, char *pass, char *topic)
{
    Serial.println("save mqtt");
    byte index = 100;
    EEPROMUpdate(index, isAvailable);
    index++;

    EEPROMUpdate(index, (byte)strlen(ip)); // write the length of ip
    index++;

    for (int i = 0; i < (int)strlen(ip); i++)
    {
        EEPROMUpdate(index, ip[i]);
        index++;
    } // write ip

    EEPROMUpdate(index, (byte)strlen(port)); // write the length of port
    index++;
    for (int i = 0; i < (int)strlen(port); i++)
    {
        EEPROMUpdate(index, port[i]);
        index++;
    } // write port

    EEPROMUpdate(index, (byte)strlen(user)); // write the length of user
    Serial.print(EEPROM.read(index));
    index++;
    for (int i = 0; i < (int)strlen(user); i++)
    {
        EEPROMUpdate(index, user[i]);
        index++;
    } // write user

    EEPROMUpdate(index, (byte)strlen(pass)); // write the length of password
    Serial.print(EEPROM.read(index));
    index++;
    for (int i = 0; i < (int)strlen(pass); i++)
    {
        EEPROMUpdate(index, pass[i]);
        index++;
    } // write password

    EEPROMUpdate(index, (byte)strlen(topic)); // write the length of topic
    Serial.print(EEPROM.read(index));
    index++;
    for (int i = 0; i < (int)strlen(topic); i++)
    {
        EEPROMUpdate(index, topic[i]);
        index++;
    } // write topic

    EEPROM.commit();
}

void readMqttInfo()
{
    if (mqttServerIp != NULL)
    {
        free(mqttServerIp);
        mqttServerIp = NULL;
    }
    if (mqttServerPort != NULL)
    {
        free(mqttServerPort);
        mqttServerPort = NULL;
    }
    if (mqttUser != NULL)
    {
        free(mqttUser);
        mqttUser = NULL;
    }
    if (mqttPassword != NULL)
    {
        free(mqttPassword);
        mqttPassword = NULL;
    }
    if (mqttTopic != NULL)
    {
        free(mqttTopic);
        mqttTopic = NULL;
    }
    Serial.println("read mqtt info");
    byte index1 = 101;
    byte length = EEPROM.read(index1);
    index1++;
    byte index2 = 0;
    
    mqttServerIp = (char *)malloc((length + 1) * sizeof(char));
    while (index2 < length)
    {
        mqttServerIp[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    mqttServerIp[index2] = '\0';

    length = EEPROM.read(index1);
    mqttServerPort = (char *)malloc((length + 1) * sizeof(char));
    index1++;
    index2 = 0;
    while (index2 < length)
    {
        mqttServerPort[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    mqttServerPort[index2] = '\0';

    length = EEPROM.read(index1);
    mqttUser = (char *)malloc((length + 1) * sizeof(char));
    index1++;
    index2 = 0;
    while (index2 < length)
    {
        mqttUser[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    mqttUser[index2] = '\0';

    length = EEPROM.read(index1);
    mqttPassword = (char *)malloc((length + 1) * sizeof(char));
    index1++;
    index2 = 0;
    while (index2 < length)
    {
        mqttPassword[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    mqttPassword[index2] = '\0';

    length = EEPROM.read(index1);
    mqttTopic = (char *)malloc((length + 1) * sizeof(char));
    index1++;
    index2 = 0;
    while (index2 < length)
    {
        mqttTopic[index2] = EEPROM.read(index1);
        index2++;
        index1++;
    }
    mqttTopic[index2] = '\0';
}