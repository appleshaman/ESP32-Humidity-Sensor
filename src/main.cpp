#include <Arduino.h>
#include <DNSServer.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <EEPROM.h>

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

const uint16_t EEPROM_SIZE = 256;

bool connectToWifi();
void initAp();
void EEPROMUpdate(int const address, uint8_t const value);
void saveWifiInfo(uint8_t wifiWork, char sta_ssid[32], char sta_password[64]);
void readWifiInfo();
void initDNS();

void wifiButtonUpdateCallback(Control *sender, int type)
{
    if (type == B_UP)
    {
        Serial.println("Button UP");
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
        Serial.println(EEPROM.read(0));


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