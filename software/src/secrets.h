// Secrets for your local home network

// This is a "hard way" to configure your local WiFi network name and passphrase
// into the source code and the uploaded sketch.
//
// Using the WiFi Manager is preferred and avoids reprogramming when your network changes.
// See https://homeding.github.io/#page=/wifimanager.md

// ssid and passPhrase can be used when compiling for a specific environment as a 2. option.

// add you wifi network name and PassPhrase or use WiFi Manager
#ifndef INCLUDE_SECRETS_H
#define INCLUDE_SECRETS_H

#ifndef STASSID
#define STASSID "ESSID"
#define STAPSK "yourpassword"
#endif

extern const char *ssid;
extern const char *passPhrase;

#define MQTT_SERVER         "192.168.5.4"
#define MQTT_SERVERPORT     1883
#define MQTT_USERNAME       ""
#define MQTT_PASSWORD       ""

#endif
