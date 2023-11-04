/**
 * @file        main.h
 * @brief       Definitions of main.cpp
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2013, 2014, 2021, 2023
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2021-02-16 17:51:49 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#ifndef INCLUDE_MAIN_H
#define INCLUDE_MAIN_H

#include <stdint.h>

#include <Arduino.h>
#include <ESP8266WebServer.h>
#include "PubSubClient.h"       /* MQTT */

#include "common.h"
#include "config.h"
#include "secrets.h"  // add WLAN Credentials in here.

#include <FS.h>        // File System for Web Server Files
#include <LittleFS.h>  // This file system is used.

// need a WebServer for http access on port 80.
extern ESP8266WebServer httpServer;

extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

extern uint32_t mqtt_connect_start_time;
extern uint32_t mqtt_publish_start_time;
extern uint32_t mqtt_publish_interval_sec;
extern String hostname;
extern String mqttTopicPrefix;
#if TEMPERATURE_SENSOR != SENSOR_NONE
extern String mqttTopicTemperature;
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
extern String mqttTopicHumidity;
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
extern String mqttTopicPressure;
#endif

extern void setLed(bool_t on=TRUE);
extern void setLed2(bool_t on=TRUE);
extern String readStringFromFile(const String& filename, uint16_t lineIdx = 0, uint8_t commentChar = COMMENT_CHAR, bool_t removeTrailingSpaces = TRUE);

#endif /* INCLUDE_MAIN_H */

