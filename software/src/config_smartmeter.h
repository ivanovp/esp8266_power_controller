/**
 * @file        config_smartmeter.h
 * @brief       Configuration definitions for smart meter
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2013, 2014, 2021, 2023
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2021-02-16 17:51:49 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

#define HW_TYPE_ESP01           0   /* ESP8266-01 a.k.a. ESP-01 */
#define HW_TYPE_ESP201          1   /* ESP201 */
#define HW_TYPE_WEMOS_D1_MINI   2   /* Wemos D1 Mini */

#define HW_TYPE                 HW_TYPE_WEMOS_D1_MINI

#define ENABLE_MQTT_CLIENT      0   /* 1: enable MQTT client, 0: disable MQTT client */
#define ENABLE_FIRMWARE_UPDATE  1   /* 1: enable firmware update through HTTP, 0: disable firmware update */
#define ENABLE_RESET            1   /* 1: enable reset through HTTP, 0: disable reset */

/* 1: Enable power metering through serial input */
/*    Standard: DSMR (Dutch Smart Meter Requirements) */
/*    Smart meter's P1 port should be connected to RX pin of ESP8266! */
/* 0: No power metering */
#define ENABLE_POWER_METER                  1
#if ENABLE_POWER_METER
#define ENABLE_POWER_METER_CRC_CHECK        0
#define ENABLE_POWER_METER_DEBUG            1
#define ENABLE_POWER_METER_RELAY            1
#define ENABLE_POWER_METER_DIGI_POT         1
#define ENABLE_POWER_METER_REQUEST          0

#if ENABLE_POWER_METER_RELAY
#define ENABLE_POWER_METER_RELAY_DEBUG      1
#define ENABLE_POWER_METER_RELAY_TEST       1
#define ENABLE_POWER_METER_LCD              1
#define POWER_METER_RELAY_PIN               15
#endif
#if ENABLE_POWER_METER_DIGI_POT
#define ENABLE_POWER_METER_DIGI_POT_TEST    0       /* Warning: this might be an endless loop! */
/* 1: Shift register is used to set digital potentiometer */
/* 0: Individual GPIO pins are used to set set digital potentiometer */
#define ENABLE_POWER_METER_DIGI_POT_SR      1

#define POWER_METER_DIGI_POT_BITS           6       /* Number of optotriacs */
#define POWER_METER_DIGI_POT_MIN            0
#define POWER_METER_DIGI_POT_MAX            ((1u << POWER_METER_DIGI_POT_BITS) - 1u)

#if ENABLE_POWER_METER_DIGI_POT_TEST
#define POWER_METER_DIGI_POT_TEST_INPUT_PIN 16
#define ENABLE_POWER_METER_DIGI_POT_TEST_BTN 1  /* Two buttons connected to analog input (A0) */
#endif

#if ENABLE_POWER_METER_DIGI_POT_SR
/* RCLK pin of 74HC595 shift register */
#define POWER_METER_DIGI_POT_SR_RCLK_PIN    2   /* GPIO2 */
#else
#define POWER_METER_DIGI_POT_PIN0           16
#define POWER_METER_DIGI_POT_PIN1           14
#define POWER_METER_DIGI_POT_PIN2           12
#define POWER_METER_DIGI_POT_PIN3           13
#endif

#if ENABLE_POWER_METER_REQUEST
/* Periodic LOW/HIGH setting is currently disabled. Reason: */
/* My smart meter (Sanxing SX631) is sending OBIS data in every 10 seconds, */
/* and it cannot be forced to send it in every 2 or 5 seconds by pulling */
/* request pin low then high. */
#define POWER_METER_REQUEST_PERIOD_MS       0   /* 0: periodic LOW/HIGH disabled */
#define POWER_METER_REQUEST_WIDTH_MS        0   /* 0: periodic LOW/HIGH disabled */
#define POWER_METER_REQUEST_PIN             0   /* GPIO0 */
#endif

/* Maximum current consumption in Ampers */
#define POWER_METER_DIGI_POT_MAX_CURRENT_A  25
#endif
#endif

#define ENABLE_TM1637_DISP      0

#define ENABLE_SHT3X            0   /* Temperature and humidity sensor (I2C) */
#define ENABLE_BMP280           0   /* Pressure and temperature sensor (I2C) */
#define ENABLE_DS18B20          0   /* Temperature sensor (Dallas 1-wire) */
#define ENABLE_LM75             1   /* Temperature sensor (I2C) */
#define ENABLE_DHT              0   /* DHT11/DHT22 temperature and humidity sensor (special 1-wire) */
#define ENABLE_I2C              0

/* Do not change the following values! */
#define SENSOR_NONE             0
#define SENSOR_SHT3X            1
#define SENSOR_BMP280           2
#define SENSOR_DS18B20          3
#define SENSOR_LM75             4
#define SENSOR_DHT              5

/* Select which sensor provides the pressure, temperature, humidity values */
#define PRESSURE_SENSOR         SENSOR_NONE
#define TEMPERATURE_SENSOR      SENSOR_LM75
#define HUMIDITY_SENSOR         SENSOR_NONE

// name of the server. You reach it using http://sensor<MAC>
#define DEFAULT_HOSTNAME "sensor"

// TODO define time zone in file!
// local time zone definition (Berlin, Belgrade, Budapest, Oslo, Paris, etc.)
#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"

#define MQTT_CONNECT_RETRY_SEC                  5
#define MQTT_PUBLISH_INTERVAL_SEC               10
#define HTTP_SERVER_PORT                        80
#define DEFAULT_HOMEPAGE_REFRESH_INTERVAL_SEC   60

#define SHT31_ADDRESS                   0x44
#define DS18B20_PIN                     14 /* GPIO14 */
#define DHTPIN                          13 /* GPIO13 */
#define DHTTYPE                         DHT22     // DHT22=DHT 22 (AM2302), DHT21=DHT 21 (AM2301)

/* Blue LED on Wemos D1 mini */
/* Note: RCLK of 74HC595 and Blue LED use the same GPIO on Wireless Sensor Controller Board Rev A! */
#if POWER_METER_DIGI_POT_SR_RCLK_PIN != 2
#define LED_PIN                         2   /* GPIO2 */
#define LED_INVERTED                    1
#endif
/* Green LED (LED2) on red power controller board */
#define LED2_PIN                        12   /* GPIO12 */
#define LED2_INVERTED                   0

#if ENABLE_TM1637_DISP
#define TM1637_CLK                      13
#define TM1637_DIO                      12
#endif

#if ENABLE_POWER_METER
#define MAX_OBIS_CODE_NUM               50
#define POWER_METER_CONFIG_FILE         "obis2mqtt.txt"

#if ENABLE_POWER_METER_RELAY
#define POWER_METER_RELAY_CONFIG_FILE   "relay.txt"
#if ENABLE_POWER_METER_DIGI_POT
#define POWER_METER_DIGI_POT_CONFIG_FILE "digipot.txt"
#endif
#endif /* ENABLE_POWER_METER_RELAY */
#endif /* ENABLE_POWER_METER */

#define COMMENT_CHAR                    ';'

#define BOARD_RESET_TIME_MS             1000

#endif /* INCLUDE_CONFIG_H_ */
