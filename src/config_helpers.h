

#ifndef CONFIG_HELPERS_H
#define CONFIG_HELPERS_H

#include <SPIFFS.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <WiFiManager.h>
#include <Preferences.h>

class ConfigHelpers
{
public:
    ConfigHelpers() {
    };

    void readConfigFile(Preferences *preferences, char *server_address, char *participant, char *position)
    {
        String servAdrr = preferences->getString("server_address", "");
        String partic = preferences->getString("participant", "");
        String pos = preferences->getString("position", "");
        strlcpy(server_address, servAdrr.c_str(), 20);
        strlcpy(participant, partic.c_str(), 50);
        strlcpy(position, pos.c_str(), 50);
    }

    void writeConfigFile(Preferences *preferences, char *server_address, char *participant, char *position, WiFiManagerParameter *custom_server_address, WiFiManagerParameter *custom_participant, WiFiManagerParameter *custom_position)
    {
        // read updated parameters
        strlcpy(server_address, custom_server_address->getValue(), 20);
        strlcpy(participant, custom_participant->getValue(), 50);
        strlcpy(position, custom_position->getValue(), 50);

        Serial.println("saving config");
        preferences->putString("server_address", server_address);
        preferences->putString("participant", participant);
        preferences->putString("position", position);

        Serial.println("\tserver_address : " + String(server_address));
        Serial.println("\tparticipant : " + String(participant));
        Serial.println("\tposition : " + String(position));
    }
};

#endif