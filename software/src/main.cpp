/**
 * @file        main.cpp
 * @brief       Main program
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2013, 2014, 2021, 2023
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2023-10-12 11:28:21 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
// Example firmware upload via curl:
// curl -F "image=@.pio/build/esp07/firmware.bin" http://smartmeter.home/update.htm
#include <ESP8266HTTPUpdateServer.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_BMP280.h"
#include "DallasTemperature.h"        /* DS18B20 */
#include "Temperature_LM75_Derived.h" /* LM75 and derivatives (TMP100, TMP102) */
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "DHT_U.h"
#include "PubSubClient.h" /* MQTT */
#include "TM1637Display.h"

#include "main.h"
#include "common.h"
#include "config.h"
#include "secrets.h" // add WLAN Credentials in here.
#include "power_meter.h"

#include <FS.h>       // File System for Web Server Files
#include <LittleFS.h> // This file system is used.

#define PLAUSIBLE_TEMPERATURE_THRESHOLD_C (-99.0f)
#define MEASUREMENT_RETRY_DELAY_MS (100)
#define MEASUREMENT_RETRY_COUNT (3)

const char *ssid = STASSID;
const char *passPhrase = STAPSK;

#if ENABLE_SHT3X
Adafruit_SHT31 sht;
float temperature_sht3x;
float humidity_sht3x;
#endif
#if ENABLE_BMP280
Adafruit_BMP280 bmp;
float pressure_bmp280;
float temperature_bmp280;
#endif
#if ENABLE_DS18B20
OneWire one_wire(DS18B20_PIN);
DallasTemperature dallas_temperature(&one_wire);
float temperature_ds18b20;
#endif
#if ENABLE_LM75
Generic_LM75 lm75;
float temperature_lm75;
#endif
#if ENABLE_DHT
DHT_Unified dht(DHTPIN, DHTTYPE);
float temperature_dht;
float humidity_dht;
#endif

#if TEMPERATURE_SENSOR != SENSOR_NONE
float temperature;
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
float humidity;
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
float pressure;
/* Calibration values from pressurecalib.txt */
float pressureOffset = 0.0f; /* First line in pressurecalib.txt */
float pressureFactor = 1.0f; /* Second line in pressurecalib.txt */
#endif
#if ENABLE_POWER_METER
#define TITLE_STR "Power measurement"
#else
#if TEMPERATURE_SENSOR != SENSOR_NONE && HUMIDITY_SENSOR != SENSOR_NONE && PRESSURE_SENSOR != SENSOR_NONE
#define TITLE_STR "Pressure, temperature and humidity measurement"
#endif
#if TEMPERATURE_SENSOR != SENSOR_NONE && HUMIDITY_SENSOR != SENSOR_NONE && PRESSURE_SENSOR == SENSOR_NONE
#define TITLE_STR "Temperature and humidity measurement"
#endif
#if TEMPERATURE_SENSOR == SENSOR_NONE && HUMIDITY_SENSOR != SENSOR_NONE && PRESSURE_SENSOR != SENSOR_NONE
#define TITLE_STR "Pressure and humidity measurement"
#endif
#if TEMPERATURE_SENSOR != SENSOR_NONE && HUMIDITY_SENSOR == SENSOR_NONE && PRESSURE_SENSOR != SENSOR_NONE
#define TITLE_STR "Pressure and temperature measurement"
#endif
#if TEMPERATURE_SENSOR != SENSOR_NONE && HUMIDITY_SENSOR == SENSOR_NONE && PRESSURE_SENSOR == SENSOR_NONE
#define TITLE_STR "Temperature measurement"
#endif
#if TEMPERATURE_SENSOR == SENSOR_NONE && HUMIDITY_SENSOR != SENSOR_NONE && PRESSURE_SENSOR == SENSOR_NONE
#define TITLE_STR "Humidity measurement"
#endif
#if TEMPERATURE_SENSOR == SENSOR_NONE && HUMIDITY_SENSOR == SENSOR_NONE && PRESSURE_SENSOR != SENSOR_NONE
#define TITLE_STR "Pressure measurement"
#endif
#if TEMPERATURE_SENSOR == SENSOR_NONE && HUMIDITY_SENSOR == SENSOR_NONE && PRESSURE_SENSOR == SENSOR_NONE
#error No sensors defined!
#endif
#endif

// need a WebServer for http access on port 80.
ESP8266WebServer httpServer(HTTP_SERVER_PORT);
#if ENABLE_FIRMWARE_UPDATE
ESP8266HTTPUpdateServer httpUpdater;
#endif

String hostname;

WiFiClient wifiClient;
#if ENABLE_MQTT_CLIENT
PubSubClient mqttClient(wifiClient);

uint32_t mqtt_connect_start_time = 0;
uint32_t mqtt_publish_start_time = 0;
uint32_t mqtt_publish_interval_sec = MQTT_PUBLISH_INTERVAL_SEC;
String mqttTopicPrefix;
#if TEMPERATURE_SENSOR != SENSOR_NONE
String mqttTopicTemperature;
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
String mqttTopicHumidity;
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
String mqttTopicPressure;
#endif
#endif /* ENABLE_MQTT_CLIENT */

uint32_t homepage_refresh_interval_sec = DEFAULT_HOMEPAGE_REFRESH_INTERVAL_SEC; /* First line homepage_refresh_interval.txt */
#if ENABLE_RESET
bool_t board_reset = FALSE;
uint32_t board_reset_timestamp_ms = 0;
#endif

#if ENABLE_TM1637_DISP
TM1637Display display(TM1637_CLK, TM1637_DIO);
#endif

// The text of builtin files are in this header file
#include "builtinfiles.h"

void setLed(bool_t on)
{
#if LED_PIN != 0
    if (
#if LED_INVERTED
        !on // Inverse logic on Wemos D1 Mini
#else
        on
#endif
    )
    {
        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(LED_PIN, LOW);
    }
#endif
}

void setLed2(bool_t on)
{
#if LED2_PIN != 0
    if (
#if LED2_INVERTED
        !on
#else
        on
#endif
    )
    {
        digitalWrite(LED2_PIN, HIGH);
    }
    else
    {
        digitalWrite(LED2_PIN, LOW);
    }
#endif
}

/*
 * Read given line of a file and return.

 * @param[in] filename      File to open and read.
 * @param[in] lineIdx       Line index (nth) to read.
 * @param[in] commentChar   Ignore this character and all data after this character
 *
 * @return nth line of file.
 */
String readStringFromFile(const String &filename, uint16_t lineIdx, uint8_t commentChar, bool_t removeTrailingSpaces)
{
    String content;
    uint16_t idx;
    int commentCharIdx;

    File file = LittleFS.open(filename, "r");
    if (file)
    {
        /* Read nth - 1 line from file and discard */
        idx = lineIdx;
        while (idx-- > 0)
        {
            file.readStringUntil('\n');
        }
        if (file.available())
        {
            content = file.readStringUntil('\n');
            if (commentChar != 0)
            {
                commentCharIdx = content.indexOf(commentChar);
                /* Check if comment character is in the string */
                if (commentCharIdx != -1)
                {
                    /* Remove comment character and evrything after that*/
                    content.remove(commentCharIdx);
                }
            }

            if (removeTrailingSpaces)
            {
                idx = content.length() - 1;
                /* Remove trailing spaces */
                while (content[idx] == ' ' || content[idx] == '\t')
                {
                    content.remove(idx);
                    idx--;
                }
            }
        }
        TRACE("Line #%i from file '%s': '%s'\n", lineIdx, filename.c_str(), content.c_str());
    }
    else
    {
        ERROR("Failed to open file %s for reading\n", filename.c_str());
    }

    return content;
}

void measure_internal()
{
    uint8_t retry_cnt;

#if ENABLE_SHT3X
    retry_cnt = MEASUREMENT_RETRY_COUNT;
    do
    {
        temperature_sht3x = sht.readTemperature();
        humidity_sht3x = sht.readHumidity();
        TRACE("Temperature_SHT3X: %.2f\n", temperature_sht3x + 0.005f);
        TRACE("Humidity_SHT3X: %.2f\n", humidity_sht3x + 0.005f);
#if TEMPERATURE_SENSOR == SENSOR_SHT3X
        temperature = temperature_sht3x;
#endif
#if HUMIDITY_SENSOR == SENSOR_SHT3X
        humidity = humidity_sht3x;
#endif
        if (temperature_sht3x < PLAUSIBLE_TEMPERATURE_THRESHOLD_C)
        {
            delay(MEASUREMENT_RETRY_DELAY_MS);
        }
    } while (temperature_sht3x < PLAUSIBLE_TEMPERATURE_THRESHOLD_C && retry_cnt--);
#endif

#if ENABLE_BMP280
    retry_cnt = MEASUREMENT_RETRY_COUNT;
    do
    {
        pressure_bmp280 = bmp.readPressure() / 100.0f;
        pressure_bmp280 += pressureOffset;
        pressure_bmp280 *= pressureFactor;
        temperature_bmp280 = bmp.readTemperature();
        TRACE("Temperature_BMP280: %.2f\n", temperature_bmp280 + 0.005f);
        TRACE("Pressure_BMP280: %.2f\n", pressure_bmp280 + 0.005f);
#if PRESSURE_SENSOR == SENSOR_BMP280
        pressure = pressure_bmp280;
#endif
        if (temperature_bmp280 < PLAUSIBLE_TEMPERATURE_THRESHOLD_C)
        {
            delay(MEASUREMENT_RETRY_DELAY_MS);
        }
    } while (temperature_bmp280 < PLAUSIBLE_TEMPERATURE_THRESHOLD_C && retry_cnt--);
#endif

#if ENABLE_DS18B20
    retry_cnt = MEASUREMENT_RETRY_COUNT;
    do
    {
        dallas_temperature.requestTemperatures();
        temperature_ds18b20 = dallas_temperature.getTempCByIndex(0);
        TRACE("Temperature_DS18B20: %.2f\n", temperature_ds18b20 + 0.005f);
#if TEMPERATURE_SENSOR == SENSOR_DS18B20
        temperature = temperature_ds18b20;
#endif
        if (temperature_ds18b20 < PLAUSIBLE_TEMPERATURE_THRESHOLD_C)
        {
            delay(MEASUREMENT_RETRY_DELAY_MS);
        }
    } while (temperature_ds18b20 < PLAUSIBLE_TEMPERATURE_THRESHOLD_C && retry_cnt--);
#endif

#if ENABLE_LM75
    retry_cnt = MEASUREMENT_RETRY_COUNT;
    do
    {
        temperature_lm75 = lm75.readTemperatureC();
        TRACE("Temperature_LM75: %.2f\n", temperature_lm75 + 0.005f);
#if TEMPERATURE_SENSOR == SENSOR_LM75
        temperature = temperature_lm75;
#endif
        if (temperature_lm75 < PLAUSIBLE_TEMPERATURE_THRESHOLD_C)
        {
            delay(MEASUREMENT_RETRY_DELAY_MS);
        }
    } while (temperature_lm75 < PLAUSIBLE_TEMPERATURE_THRESHOLD_C && retry_cnt--);
#endif

#if ENABLE_DHT
    retry_cnt = MEASUREMENT_RETRY_COUNT;
    do
    {
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        if (isnan(event.temperature))
        {
            temperature_dht = -100.0f;
        }
        else
        {
            temperature_dht = event.temperature;
        }
        TRACE("Temperature_DHT: %.2f\n", temperature_dht + 0.005f);
#if TEMPERATURE_SENSOR == SENSOR_DHT
        temperature = temperature_dht;
#endif
        dht.humidity().getEvent(&event);
        if (isnan(event.relative_humidity))
        {
            humidity_dht = -100.0f;
        }
        else
        {
            humidity_dht = event.relative_humidity;
        }
        TRACE("Humidity_DHT: %.2f\n", humidity_dht + 0.005f);
#if HUMIDITY_SENSOR == SENSOR_DHT
        humidity = humidity_dht;
#endif
        if (temperature_dht < PLAUSIBLE_TEMPERATURE_THRESHOLD_C)
        {
            delay(MEASUREMENT_RETRY_DELAY_MS);
        }
    } while (temperature_dht < PLAUSIBLE_TEMPERATURE_THRESHOLD_C && retry_cnt--);
#endif

#if TEMPERATURE_SENSOR != SENSOR_NONE
    TRACE("Temperature: %.2f C\n", temperature + 0.005f);
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
    TRACE("Humidity: %.2f %%\n", humidity + 0.005f);
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
    TRACE("Pressure: %.2f hPa\n", pressure + 0.005f);
#endif
}

void measure()
{
    setLed(TRUE);
    measure_internal();
    setLed(FALSE);
#if ENABLE_TM1637_DISP
#if 1
    uint8_t segments[4];
    if (temperature >= 0 && temperature < 100)
    {
        /* Between 0 and 100 Celsius */
        segments[0] = display.encodeDigit((int)(temperature / 10) % 10);
        segments[1] = display.encodeDigit(((int)temperature) % 10);
        segments[2] = SEG_E; /* no decimal point, so use segment E as decimap point*/
        segments[3] = display.encodeDigit((int)((temperature + 0.05f) * 10) % 10);
    }
    else if (temperature >= 100)
    {
        /* Greater than 100 Celsius */
        segments[0] = 0;
        segments[1] = display.encodeDigit((int)(temperature / 100) % 10);
        segments[2] = display.encodeDigit((int)(temperature / 10) % 10);
        segments[3] = display.encodeDigit(((int)temperature) % 10);
    }
    else
    {
        /* Negative temperature */
        segments[0] = SEG_G;
        segments[1] = display.encodeDigit((int)(-temperature / 100) % 10);
        segments[2] = display.encodeDigit((int)(-temperature / 10) % 10);
        segments[3] = display.encodeDigit(((int)-temperature) % 10);
    }
    display.setSegments(segments);
#else
    display.showNumberDec((int)temperature);
#endif
#endif
}

// ===== Simple functions used to answer simple GET requests =====

#if 0
// This function is called when the WebServer was requested without giving a filename.
// This will redirect to the file index.htm when it is existing otherwise to the built-in upload.htm page
void handleRedirect()
{
    TRACE("Redirect...\n");
    String url = "/index.htm";

    if (!LittleFS.exists(url))
    {
        url = "/upload.htm";
    }

    httpServer.sendHeader("Location", url, true);
    httpServer.send(302);
}
#endif

/*
 * It generates page /index.htm and /
 */
void handleIndex()
{
    String buf;

    measure();

    buf += "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Strict//EN\"";
    buf += "\"http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd\">";
    buf += "<html xmlns=\"http://www.w3.org/1999/xhtml\" lang=\"en\" xml:lang=\"en\">";
    buf += "<head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"/>";
    if (homepage_refresh_interval_sec > 0)
    {
        buf += "<meta http-equiv=\"refresh\" content=\"" + String(homepage_refresh_interval_sec) + "\"/>"; // automatically reload in 60 seconds
    }
    buf += "<meta name=\"mobile-web-app-capable\" content=\"yes\">";
#if ENABLE_POWER_METER
    buf += "<meta name=\"viewport\" content=\"user-scalable=no, width=device-width, initial-scale=1.0, maximum-scale=1.0\"/>";
#else
    buf += "<meta name=\"viewport\" content=\"user-scalable=no, width=device-width, initial-scale=1.2, maximum-scale=1.2\"/>";
#endif
    buf += "<title>" TITLE_STR "</title>";
    buf += "<link Content-Type=\"text/css\" href=\"/style.css\" rel=\"stylesheet\" />";
    buf += "</head><body>";
    buf += "<h1>" TITLE_STR "</h1>";
    buf += "<p>";
#if ENABLE_POWER_METER
    uint16_t idx;
    String data;

    if (obis_data_length)
    {
        /* OBIS data received from smart meter */
        buf += "<table><tr>";
        if (obis2mqtt_valid)
        {
            /* There is obis2mqtt conversion table (from configuration file) */
            if (obis2mqtt_global_flags & OBIS2MQTT_GLOBAL_FLAG_ALWAYS_SHOW_OBIS_CODES)
            {
                buf += "<th>OBIS code (register)</th>";
            }
            buf += "<th>Description</th><th>Value</th></tr>";
            /* There is obis2mqtt.txt configuration */
            obis_data_t *p_obis_data = NULL;

            for (idx = 0; idx < obis2mqtt_length && idx < MAX_OBIS_CODE_NUM; idx++)
            {
                p_obis_data = find_obis_data(obis2mqtt[idx].obis_code);
                if (p_obis_data && !(obis2mqtt[idx].flags & OBIS2MQTT_FLAG_DO_NOT_SHOW_INDEX))
                {
                    buf += "<tr>";
                    data = p_obis_data->data_with_unit;
                    /* OBIS code found, we can translate OBIS code to a human readable string */
                    if (obis2mqtt_global_flags & OBIS2MQTT_GLOBAL_FLAG_ALWAYS_SHOW_OBIS_CODES)
                    {
                        /* Show OBIS code */
                        buf += "<td>" + p_obis_data->obis_code + "</td>";
                    }
                    /* Show data with pre-defined human readable string */
                    buf += "<td>" + obis2mqtt[idx].human_readable_string + "</td><td>" + data + "</td>";
                    buf += "</tr>";
                }
            }
        }
        else
        {
            /* No obis2mqtt.txt configuration, show raw OBIS code and data */
            buf += "<th>OBIS code</th><th>Value</th></tr>";
            for (idx = 0; idx < obis_data_length && idx < MAX_OBIS_CODE_NUM; idx++)
            {
                buf += "<tr><td>" + obis_data[idx].obis_code + "</td><td>" + obis_data[idx].data_with_unit + "</td></tr>";
            }
        }
        buf += "</table>";
        buf += "<br>OBIS data age: " + String((millis() - obis_data_timestamp_ms) / 1000) + " second<br>";
    }
    else
    {
        buf += "No OBIS data received yet.<br>";
    }
#if ENABLE_POWER_METER_RELAY
    buf += "<br>Relay is ";
    if (relay_is_on)
    {
        buf += "ON";
    }
    else
    {
        buf += "OFF";
    }
    buf += "<br>";
#if ENABLE_POWER_METER_DIGI_POT
    buf += "Digital potmeter: " + String(power_meter_digi_pot_value);
    buf += " (max. " + String(POWER_METER_DIGI_POT_MAX) + ")<br>";
#endif
    buf += "<br>";
#endif
#endif
#if TEMPERATURE_SENSOR != SENSOR_NONE
    buf += "Temperature: " + String(temperature + 0.005f, 2) + " &deg;C<br>";
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
    buf += "Humidity: " + String(humidity + 0.005f, 2) + " %<br>";
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
    buf += "Pressure: " + String(pressure + 0.005f, 2) + " hPa<br>";
#endif
    buf += "<br>Uptime: " + String(millis() / 1000) + " seconds<br>";
    buf += "</p>";
    buf += "<p><a href=\"/admin.htm\">Admin</a></p>";
    buf += "<p>Copyright (C) Peter Ivanov &lt;<a href=\"mailto:ivanovp@gmail.com\">ivanovp@gmail.com</a>&gt;, 2023.<br>";
    buf += "</p></body></html>";

    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.send(200, "text/html; charset=utf-8", buf);
}

#if ENABLE_RESET
/*
 * It resets the board.
 */
void handleReset()
{
    String buf;

    buf += "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Strict//EN\"";
    buf += "\"http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd\">";
    buf += "<html xmlns=\"http://www.w3.org/1999/xhtml\" lang=\"en\" xml:lang=\"en\">";
    buf += "<head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"/>";
    buf += "<meta http-equiv=\"refresh\" content=\"" + String(BOARD_RESET_TIME_MS / 1000 + 5) + "; URL=index.htm\" />";
    buf += "<meta name=\"mobile-web-app-capable\" content=\"yes\">";
    buf += "<title>" TITLE_STR "</title>";
    buf += "<link Content-Type=\"text/css\" href=\"/style.css\" rel=\"stylesheet\" />";
    buf += "</head><body>";
    buf += "<p>Board will be resetted in " + String(BOARD_RESET_TIME_MS) + " milliseconds...";
    buf += "</p></body></html>";

    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.send(200, "text/html; charset=utf-8", buf);

    board_reset = TRUE;
    board_reset_timestamp_ms = millis();
}
#endif

// This function is called when the WebServer was requested to list all existing files in the filesystem.
// a JSON array with file information is returned.
void handleListFiles()
{
    Dir dir = LittleFS.openDir("/");
    String result;

    result += "[\n";
    while (dir.next())
    {
        if (result.length() > 4)
        {
            result += ",";
        }
        result += "  {";
        result += " \"name\": \"" + dir.fileName() + "\", ";
        result += " \"size\": " + String(dir.fileSize()) + ", ";
        result += " \"time\": " + String(dir.fileTime());
        result += " }\n";
        // jc.addProperty("size", dir.fileSize());
    } // while
    result += "]";
    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.send(200, "text/javascript; charset=utf-8", result);
}

// This function is called when the sysInfo service was requested.
void handleSysInfo()
{
    String result;

    FSInfo fs_info;
    LittleFS.info(fs_info);

    result += "{\n";
    result += "  \"firmwareVersionMajor\": " TOSTR(VERSION_MAJOR) "\n";
    result += "  , \"firmwareVersionMinor\": " TOSTR(VERSION_MINOR) "\n";
    result += "  , \"firmwareVersionRevision\": " TOSTR(VERSION_REVISION) "\n";
    result += "  , \"hostName\": \"" + String(WiFi.getHostname()) + "\"\n";
    result += "  , \"macAddress\": \"" + WiFi.macAddress() + "\"\n";
    result += "  , \"ipAddress\": \"" + WiFi.localIP().toString() + "\"\n";
    result += "  , \"ipMask\": \"" + WiFi.subnetMask().toString() + "\"\n";
    result += "  , \"dnsIp\": \"" + WiFi.dnsIP().toString() + "\"\n";
    result += "  , \"flashSize\": " + String(ESP.getFlashChipSize()) + "\n";
    result += "  , \"freeHeap\": " + String(ESP.getFreeHeap()) + "\n";
    result += "  , \"fsTotalBytes\": " + String(fs_info.totalBytes) + "\n";
    result += "  , \"fsUsedBytes\": " + String(fs_info.usedBytes) + "\n";
    result += "  , \"uptime_ms\": " + String(millis()) + "\n";
    result += "  , \"pressureSensorType\": " TOSTR(PRESSURE_SENSOR) "\n";
    result += "  , \"temperatureSensorType\": " TOSTR(TEMPERATURE_SENSOR) "\n";
    result += "  , \"humiditySensorType\": " TOSTR(HUMIDITY_SENSOR) "\n";
#if ENABLE_SHT3X
    result += "  , \"SHT3X\": 1\n";
#endif
#if ENABLE_BMP280
    result += "  , \"BMP280\": 1\n";
#endif
#if ENABLE_DS18B20
    result += "  , \"DS18B20\": 1\n";
#endif
#if ENABLE_LM75
    result += "  , \"LM75\": 1\n";
#endif
#if ENABLE_DHT
    result += "  , \"DHT\": 1\n";
#endif
#if ENABLE_POWER_METER
    result += "  , \"powerMeter\": 1\n";
#if ENABLE_POWER_METER_CRC_CHECK
    result += "  , \"powerMeterCrcCheck\": 1\n";
#endif
#if ENABLE_POWER_METER_RELAY
    result += "  , \"powerMeterRelay\": 1\n";
#endif
#if ENABLE_POWER_METER_RELAY_TEST
    result += "  , \"powerMeterRelayTest\": 1\n";
#endif
#if ENABLE_POWER_METER_DIGI_POT
    result += "  , \"powerMeterDigiPot\": 1\n";
    result += "  , \"powerMeterDigiPotBits\": " TOSTR(POWER_METER_DIGI_POT_BITS) "\n";
#if ENABLE_POWER_METER_DIGI_POT_TEST
    result += "  , \"powerMeterDigiPotTest\": 1\n";
#endif
#endif
#if ENABLE_POWER_METER_LCD
    result += "  , \"powerMeterRelayLcd\": 1\n";
#endif
#endif
    result += "  , \"compileDate\": \"" __DATE__ "\"\n";
    result += "  , \"compileTime\": \"" __TIME__ "\"\n";
    result += "}";
    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.send(200, "text/javascript; charset=utf-8", result);
} // handleSysInfo()

/*
 * It returns measured values in JSON array.
 */
void handleSensor()
{
    String result;

    measure();

    result += "[\n";
#if TEMPERATURE_SENSOR != SENSOR_NONE
    result += "  { \"name\": \"Temperature\", \"value\": " + String(temperature + 0.005f, 2) + ", \"unit\": \"°C\" },\n";
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
    result += "  { \"name\": \"Humidity\", \"value\": " + String(humidity + 0.005f, 2) + ", \"unit\": \"%\" },\n";
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
    result += "  { \"name\": \"Pressure\", \"value\": " + String(pressure + 0.005f, 2) + ", \"unit\": \"hPa\" }, \n";
#endif
#if ENABLE_SHT3X
    result += "  { \"name\": \"Temperature_SHT3X\", \"value\": " + String(temperature_sht3x + 0.005f, 2) + ", \"unit\": \"°C\" },\n";
    result += "  { \"name\": \"Humidity_SHT3X\", \"value\": " + String(humidity_sht3x + 0.005f, 2) + ", \"unit\": \"%\" }, \n";
#endif
#if ENABLE_BMP280
    result += "  { \"name\": \"Temperature_BMP280\", \"value\": " + String(temperature_bmp280 + 0.005f, 2) + ", \"unit\": \"°C\" },\n";
    result += "  { \"name\": \"Pressure_BMP280\", \"value\": " + String(pressure_bmp280 + 0.005f, 2) + ", \"unit\": \"hPa\" }, \n";
#endif
#if ENABLE_DS18B20
    result += "  { \"name\": \"Temperature_DS18B20\", \"value\": " + String(temperature_ds18b20 + 0.005f, 2) + ", \"unit\": \"°C\" },\n";
#endif
#if ENABLE_LM75
    result += "  { \"name\": \"Temperature_LM75\", \"value\": " + String(temperature_lm75 + 0.005f, 2) + ", \"unit\": \"°C\" },\n";
#endif
#if ENABLE_DHT
    result += "  { \"name\": \"Temperature_DHT\", \"value\": " + String(temperature_dht + 0.005f, 2) + ", \"unit\": \"°C\" },\n";
    result += "  { \"name\": \"Humidity_DHT\", \"value\": " + String(humidity_dht + 0.005f, 2) + ", \"unit\": \"%\" }, \n";
#endif
    result += "]";
    /* Remove very last comma from string (incompatible with JSON)
     * Compile time solution would be better...
     */
    uint16_t len = result.length();
    while (--len > 0)
    {
        if (result[len] == ',')
        {
            result.remove(len, 1);
            break;
        }
    }

    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.send(200, "text/javascript; charset=utf-8", result);
} // handleSysInfo()

// ===== Request Handler class used to answer more complex requests =====

// The FileServerHandler is registered to the web server to support DELETE and UPLOAD of files into the filesystem.
class FileServerHandler : public RequestHandler
{
public:
    // @brief Construct a new File Server Handler object
    // @param fs The file system to be used.
    // @param path Path to the root folder in the file system that is used for serving static data down and upload.
    // @param cache_header Cache Header to be used in replies.
    FileServerHandler()
    {
        TRACE("FileServerHandler is registered\n");
    }

    // @brief check incoming request. Can handle POST for uploads and DELETE.
    // @param requestMethod method of the http request line.
    // @param requestUri request ressource from the http request line.
    // @return true when method can be handled.
    bool canHandle(HTTPMethod requestMethod, const String &requestUri) override
    {
        bool can = false;

        if (requestMethod == HTTP_POST)
        {
            can = true;
        }
        else if (requestMethod == HTTP_DELETE)
        {
            can = true;
        }

        return can;
    } // canHandle()

    bool canUpload(const String &uri) override
    {
        // only allow upload on root fs level.
        return (uri == "/");
    } // canUpload()

    bool handle(ESP8266WebServer &httpServer, HTTPMethod requestMethod, const String &requestUri) override
    {
        // ensure that filename starts with '/'
        String fileName = requestUri;
        TRACE("handle, requestMethod: %i, requestUri: %s\n", requestMethod, requestUri.c_str());
        if (!fileName.startsWith("/"))
        {
            fileName = "/" + fileName;
        }

        if (requestMethod == HTTP_POST)
        {
            // all done in upload. no other forms.
        }
        else if (requestMethod == HTTP_DELETE)
        {
            TRACE("Deleting %s... ", fileName.c_str());
            if (LittleFS.exists(fileName))
            {
                if (LittleFS.remove(fileName))
                {
                    TRACE("Done.\n");
                }
                else
                {
                    ERROR("Cannot delete %s!\n", fileName.c_str());
                }
            }
            else
            {
                ERROR("Cannot delete %s as it does not exists!\n", fileName.c_str());
            }
        } // if

        httpServer.send(200); // all done.
        return (true);
    } // handle()

    // uploading process
    void upload(ESP8266WebServer UNUSED &server, const String UNUSED &_requestUri, HTTPUpload &upload) override
    {
        // ensure that filename starts with '/'
        String fileName = upload.filename;
        if (!fileName.startsWith("/"))
        {
            fileName = "/" + fileName;
        }

        if (upload.status == UPLOAD_FILE_START)
        {
            // Open the file
            if (LittleFS.exists(fileName))
            {
                LittleFS.remove(fileName);
            }
            m_fsUploadFile = LittleFS.open(fileName, "w");
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
            // Write received bytes
            if (m_fsUploadFile)
            {
                m_fsUploadFile.write(upload.buf, upload.currentSize);
            }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
            // Close the file
            if (m_fsUploadFile)
            {
                m_fsUploadFile.close();
            }
        }
    } // upload()

protected:
    File m_fsUploadFile;
};

#if ENABLE_MQTT_CLIENT
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    TRACE("MQTT callback, topic: %s, payload: %s\n", topic, payload);
}
#endif

void i2c_scan()
{
    byte error, address;

    TRACE("I2C scan...\n");
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            TRACE("I2C device found at address 0x%02X\n", address);
        }
    }
}

// Setup everything to make the webserver work.
void setup(void)
{
#if LED_PIN != 0
    pinMode(LED_PIN, OUTPUT);
#endif
#if LED2_PIN != 0
    pinMode(LED2_PIN, OUTPUT);
#endif
    setLed(TRUE);
    setLed2(TRUE);

    delay(250); // wait for serial monitor to start completely.

    // Use Serial port for some trace information from the example
    Serial.begin(115200);
    Serial.setDebugOutput(false);

    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println("Wireless sensor and power controller firmware started");
    Serial.printf("Compiled on " __DATE__ " " __TIME__ "\n");

    Serial.printf("Pressure sensor type: " TOSTR(PRESSURE_SENSOR) "\n");
    Serial.printf("Temperature sensor type: " TOSTR(TEMPERATURE_SENSOR) "\n");
    Serial.printf("Humidity sensor type: " TOSTR(HUMIDITY_SENSOR) "\n");
    setLed(FALSE);
    setLed2(FALSE);

#if ENABLE_TM1637_DISP
    const uint8_t segments[4] = {0xFFu, 0xFFu, 0xFFu, 0xFFu};
    display.setBrightness(3);
    display.setSegments(segments);
#endif

#if ENABLE_BMP280 || ENABLE_SHT3X || ENABLE_LM75 || ENABLE_I2C
#if HW_TYPE == HW_TYPE_ESP01
    Wire.begin(0, 2); // for ESP8266-01 (ESP-01)
#elif HW_TYPE == HW_TYPE_WEMOS_D1_MINI
    Wire.begin(); // for Wemos D1 Mini
#else
    Wire.begin(); // for ESP-07
#endif
    i2c_scan();
#endif

#if ENABLE_SHT3X
    TRACE("\n");
    if (sht.begin(SHT31_ADDRESS))
    {
        TRACE("SHT3x initialized\n");
        TRACE("SHT3x status: 0x%X\n", sht.readStatus());

        TRACE("Temperature: %.2f\n", sht.readTemperature() + 0.005f);
        TRACE("Humidity: %.2f\n", sht.readHumidity() + 0.005f);
    }
    else
    {
        ERROR("ERROR: Cannot initialize SHT3x!\n");
    }
#endif

#if ENABLE_BMP280
    TRACE("\n");
    if (bmp.begin())
    {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        TRACE("BMP280 initialized\n");

        TRACE("Temperature: %.2f\n", bmp.readTemperature() + 0.005f);
        TRACE("Pressure: %.2f\n", bmp.readPressure() / 100.0f + 0.005f);
    }
    else
    {
        ERROR("ERROR: Cannot initialize BMP280!\n");
    }
#endif

#if ENABLE_DS18B20
    TRACE("\n");
    dallas_temperature.begin();
    if (dallas_temperature.getDS18Count())
    {
        TRACE("DS18B20 initialized, DS18xxx count: %i\n", dallas_temperature.getDS18Count());
        dallas_temperature.requestTemperatures();
        TRACE("Temperature: %.2f\n", dallas_temperature.getTempCByIndex(0) + 0.005f);
    }
    else
    {
        ERROR("ERROR: No DS18B20 detected on 1-wire-bus!\n");
    }
#endif

#if ENABLE_LM75
    TRACE("\n");
    if (lm75.readTemperatureC() > -127.0) // TODO how to determine if chip exists?
    {
        TRACE("LM75 initialized\n");
        TRACE("Temperature: %.2f\n", lm75.readTemperatureC() + 0.005f);
    }
    else
    {
        ERROR("ERROR: Cannot initialize LM75!\n");
    }
#endif
#if ENABLE_DHT
    TRACE("\n");
    dht.begin();
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature))
    {
        TRACE("DHT initialized\n");
        TRACE("Temperature: %.2f\n", event.temperature + 0.005f);
        dht.humidity().getEvent(&event);
        TRACE("Humidity: %.2f\n", event.relative_humidity + 0.005f);
    }
    else
    {
        ERROR("ERROR: Cannot initialize DHT!\n");
    }
#endif

    TRACE("\n");
    TRACE("Mounting the filesystem...\n");
    if (!LittleFS.begin())
    {
        TRACE("could not mount the filesystem...\n");
        delay(2000);
        ESP.restart();
    }

    // start WiFI
    WiFi.mode(WIFI_STA);
    if (strlen(ssid) == 0)
    {
        WiFi.begin();
    }
    else
    {
        WiFi.begin(ssid, passPhrase);
    }


    hostname = readStringFromFile("/hostname.txt");
    if (hostname.isEmpty())
    {
        // allow to address the device by the given name e.g. http://sensor<MAC>
        hostname = DEFAULT_HOSTNAME;
        String mac = WiFi.macAddress();
        TRACE("MAC: %s\n", mac.c_str());
        hostname += mac[9];
        hostname += mac[10];
        hostname += mac[12];
        hostname += mac[13];
        hostname += mac[15];
        hostname += mac[16];
    }
#if ENABLE_MQTT_CLIENT
    mqttTopicPrefix = "/sensors/" + hostname + "/";
#if TEMPERATURE_SENSOR != SENSOR_NONE
    mqttTopicTemperature = mqttTopicPrefix + "temperature";
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
    mqttTopicHumidity = mqttTopicPrefix + "humidity";
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
    mqttTopicPressure = mqttTopicPrefix + "pressure";
#endif
#endif /* ENABLE_MQTT_CLIENT */

    TRACE("Setting WiFi hostname: %s\n", hostname.c_str());
    WiFi.setHostname(hostname.c_str());
    TRACE("WiFi hostname: %s\n", WiFi.getHostname());

    String str;
#if ENABLE_MQTT_CLIENT
    str = readStringFromFile("/mqtt_publish_interval.txt");
    if (!str.isEmpty())
    {
        mqtt_publish_interval_sec = str.toInt();
        TRACE("Setting measurement interval to %i seconds...\n", mqtt_publish_interval_sec);
    }
#endif

    str = readStringFromFile("/homepage_refresh_interval.txt");
    if (!str.isEmpty())
    {
        homepage_refresh_interval_sec = str.toInt();
        TRACE("Setting homepage refresh interval to %i seconds...\n", homepage_refresh_interval_sec);
    }

#if PRESSURE_SENSOR != SENSOR_NONE
    str = readStringFromFile("/pressurecalib.txt", 0);
    if (!str.isEmpty())
    {
        pressureOffset = str.toFloat();
        TRACE("Setting pressure offset to %f...\n", pressureOffset);
    }
    str = readStringFromFile("/pressurecalib.txt", 1);
    if (!str.isEmpty())
    {
        pressureFactor = str.toFloat();
        TRACE("Setting pressure factor to %f...\n", pressureFactor);
    }
#endif
#if ENABLE_POWER_METER
    power_meter_init();
#endif

    TRACE("Connecting to WiFi...\n");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        TRACE(".");
    }
    TRACE("connected.\n");
    randomSeed(micros());
    TRACE("IP address: %s\n", WiFi.localIP().toString().c_str());

    // Ask for the current time using NTP request builtin into ESP firmware.
    TRACE("Setup NTP...\n");
    configTime(TIMEZONE, "pool.ntp.org");

    TRACE("Register service handlers...\n");

#if ENABLE_FIRMWARE_UPDATE
    MDNS.begin(hostname);
    httpUpdater.setup(&httpServer, "/update.htm");
#endif

    // serve a built-in htm page
    httpServer.on("/upload.htm", []()
                  { httpServer.send(200, "text/html", FPSTR(uploadContent)); });

    // register a redirect handler when only domain name is given.
    //   server.on("/", HTTP_GET, handleRedirect);
    httpServer.on("/", HTTP_GET, handleIndex);
    httpServer.on("/index.htm", HTTP_GET, handleIndex);
#if ENABLE_RESET
    httpServer.on("/reset_confirm.htm", HTTP_GET, handleReset);
#endif

    // register some REST services
    httpServer.on("/file_list.json", HTTP_GET, handleListFiles);
    httpServer.on("/sysinfo.json", HTTP_GET, handleSysInfo);
    httpServer.on("/sensor.json", HTTP_GET, handleSensor);

    // UPLOAD and DELETE of files in the file system using a request handler.
    httpServer.addHandler(new FileServerHandler());

    // enable CORS header in webserver results
    httpServer.enableCORS(true);

    // enable ETAG header in webserver results from serveStatic handler
    httpServer.enableETag(true);

    // serve all static files
    httpServer.serveStatic("/", LittleFS, "/");

    // handle cases when file is not found
    httpServer.onNotFound([]()
                          {
    // standard not found in browser.
    httpServer.send(404, "text/html", FPSTR(notFoundContent)); });

    httpServer.begin();
    TRACE("HTTP server started at http://%s:%i/\n", hostname.c_str(), HTTP_SERVER_PORT);

#if ENABLE_FIRMWARE_UPDATE
    MDNS.addService("http", "tcp", HTTP_SERVER_PORT);
    TRACE("HTTPUpdateServer ready at http://%s:%i/update.htm\n", hostname.c_str(), HTTP_SERVER_PORT);
#endif

#if ENABLE_MQTT_CLIENT
    mqttClient.setServer(MQTT_SERVER, MQTT_SERVERPORT);
    mqttClient.setCallback(mqtt_callback);
#if ENABLE_POWER_METER
    mqttClient.setBufferSize(1088);
#endif
#endif /* ENABLE_MQTT_CLIENT */
} // setup

#if ENABLE_MQTT_CLIENT
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care of connecting.
void mqtt_task()
{
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED)
    {
        if ((mqtt_connect_start_time == 0) || (millis() >= mqtt_connect_start_time))
        {
            TRACE("Connecting to MQTT... ");

            if (mqttClient.connect(hostname.c_str(), MQTT_USERNAME, MQTT_PASSWORD))
            {
                TRACE("Connected to MQTT broker\n");
            }
            else
            {
                ERROR("ERROR: Cannot connect to MQTT broker! Error: %i\n", mqttClient.state());
                TRACE("Retrying MQTT connection in %i seconds...", MQTT_CONNECT_RETRY_SEC);
                mqttClient.disconnect();
                mqtt_connect_start_time = millis() + SEC_TO_MS(MQTT_CONNECT_RETRY_SEC);
                mqtt_publish_start_time = millis();
            }
        }
    }

    if (mqttClient.connected())
    {
        if (millis() >= mqtt_publish_start_time)
        {
            /* Next measurement in 10 seconds */
            mqtt_publish_start_time = millis() + SEC_TO_MS(mqtt_publish_interval_sec);
            measure();
            String str;
#if TEMPERATURE_SENSOR != SENSOR_NONE
            str = String(temperature + 0.005f, 2);
            mqttClient.publish(mqttTopicTemperature.c_str(), str.c_str());
            TRACE("Publish %s, %s\n", mqttTopicTemperature.c_str(), str.c_str());
#endif
#if HUMIDITY_SENSOR != SENSOR_NONE
            str = String(humidity + 0.005f, 2);
            mqttClient.publish(mqttTopicHumidity.c_str(), str.c_str());
            TRACE("Publish %s, %s\n", mqttTopicHumidity.c_str(), str.c_str());
#endif
#if PRESSURE_SENSOR != SENSOR_NONE
            str = String(pressure + 0.005f, 2);
            mqttClient.publish(mqttTopicPressure.c_str(), str.c_str());
            TRACE("Publish %s, %s\n", mqttTopicPressure.c_str(), str.c_str());
#endif
        }
#if ENABLE_POWER_METER
        publishObisData();
#endif
    }
    mqttClient.loop();
}
#endif /* ENABLE_MQTT_CLIENT */

// run the server...
void loop(void)
{
#if ENABLE_RESET
    uint32_t now;
#endif

    httpServer.handleClient();
#if ENABLE_FIRMWARE_UPDATE
    MDNS.update();
#endif
#if ENABLE_MQTT_CLIENT
    mqtt_task();
#endif
#if ENABLE_POWER_METER
    power_meter_task();
#endif
#if ENABLE_RESET
    now = millis();
    if (board_reset && now >= BOARD_RESET_TIME_MS && now - BOARD_RESET_TIME_MS >= board_reset_timestamp_ms)
    {
        TRACE("Restarting...");
        ESP.restart();
    }
#endif
}
