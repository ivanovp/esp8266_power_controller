/**
 * @file        power_meter.h
 * @brief       Definitions of power_meter.cpp
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2013, 2014, 2021, 2023
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2021-02-16 17:51:49 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#ifndef INCLUDE_POWER_METER_H
#define INCLUDE_POWER_METER_H

#include <stdint.h>

#include <Arduino.h>
#include <ESP8266WebServer.h>

#include "common.h"
#include "config.h"
#include "secrets.h"  // add WLAN Credentials in here.

#include <FS.h>        // File System for Web Server Files
#include <LittleFS.h>  // This file system is used.

#if ENABLE_POWER_METER
#define OBIS2MQTT_FLAG_REMOVE_UNIT          (1u << 1)   /* Remove unit. Example: "50.01*Hz" => "50.01" */
#define OBIS2MQTT_FLAG_DO_NOT_PUBLISH       (1u << 2)   /* Do not publish via MQTT. */
#define OBIS2MQTT_FLAG_DO_NOT_SHOW_INDEX    (1u << 3)   /* Do not show on index page (index.htm). */

#define OBIS2MQTT_GLOBAL_FLAG_REPLACE_STAR              (1u << 0)   /* Replace '*' to space. Example: "50.01*Hz" => "50.01 Hz" */
#define OBIS2MQTT_GLOBAL_FLAG_REMOVE_UNIT               (1u << 1)   /* Remove unit. Example: "50.01*Hz" => "50.01" */
/* 1: Publish all codes, except messages marked with OBIS2MQTT_FLAG_DO_NOT_PUBLISH. */
/* 0: Publish only codes found in "obis2mqtt.txt" */
#define OBIS2MQTT_GLOBAL_FLAG_PUBLISH_ALL               (1u << 2)
#define OBIS2MQTT_GLOBAL_FLAG_SHOW_ALL                  (1u << 3)
/* 1: Publish OBIS codes and MQTT message as well */
/* 0: Publish MQTT message from "obis2mqtt.txt" */
#define OBIS2MQTT_GLOBAL_FLAG_ALWAYS_PUBLISH_OBIS_CODES (1u << 4)
#define OBIS2MQTT_GLOBAL_FLAG_ALWAYS_SHOW_OBIS_CODES    (1u << 5)

/* Structure to translate OBIS code to MQTT topic and human readble string */
typedef struct
{
    String obis_code;
    String mqtt_topic;              /* Example: "imported_energy" */
    String human_readable_string;   /* Example: "Imported energy" */
    String unit;                    /* Example: "Hz", "kvar", "kW" */
    uint8_t flags;                  /* See OBIS2MQTT_FLAG_* */
} obis_code_to_mqtt_topic_t;

typedef enum
{
    PMSTATE_UNDEFINED = 0,
    /* Last chunk of OBIS codes were processed, waiting for next chunk, which begins with the identifier */
    /* The obis_data structure contains valid data in this state. */
    PMSTATE_DONE_WAIT_FOR_IDENTIFIER = 1,
    /* Next line after identifier is an empty line */
    PMSTATE_EMPTY_LINE = 2,
    /* OBIS codes and data */
    PMSTATE_DATA = 3,
    /* Last piece is a CRC16 number */
    PMSTATE_CRC = 4,
} power_meter_state_machine_t;

/* Received data from smart meter */
typedef struct
{
    String obis_code;
    String data_without_unit;   /* Data without unit */
    String data_with_unit;      /* Data with unit */
} obis_data_t;

/* Contents of this array is read from file "obis2mqtt.txt" at startup */
extern obis_code_to_mqtt_topic_t obis2mqtt[MAX_OBIS_CODE_NUM];
extern uint8_t obis2mqtt_length;
/* "obis2mqtt.txt" has been processed and obis2mqtt array is valid */
extern bool obis2mqtt_valid;
extern uint8_t obis2mqtt_global_flags;

extern power_meter_state_machine_t power_meter_state;
extern String pm_identifier;

/* Last received data */
extern obis_data_t obis_data[MAX_OBIS_CODE_NUM];
/* Number of valid data in obis_data */
extern uint8_t obis_data_length;
/* Actual index of obis_data to publish via MQTT */
extern uint8_t obis_data_publish_idx;
extern uint32_t obis_data_timestamp_ms;
/* Currently received data (temporary, it might be unfinished)*/
extern obis_data_t obis_data_processing[MAX_OBIS_CODE_NUM];
/* Actual index in obis_data_processing */
extern uint8_t obis_data_processing_idx;

#if ENABLE_POWER_METER_RELAY
extern String import_power_obis_code;
extern String current_obis_code;
extern uint8_t minimum_current_A;
extern float import_power_threshold_kW;
extern uint32_t relay_cooldown_time_ms;

extern bool_t relay_is_on;
extern uint32_t relay_cooldown_timestamp_ms;

#if ENABLE_POWER_METER_DIGI_POT
extern uint8_t power_meter_digi_pot_value;
#endif
#endif

extern void power_meter_set_digi_pot(uint8_t value);
extern obis_data_t * find_obis_data(String& obis_code);
extern void publishObisData();
extern void power_meter_task();
extern void power_meter_init();
#endif

#endif /* INCLUDE_POWER_METER_H */

