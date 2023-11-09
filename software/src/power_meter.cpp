/**
 * @file        power_meter.cpp
 * @brief       Power meter, smartmeter
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2013, 2014, 2021, 2023
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2023-09-10 21:35:30 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#include <Arduino.h>
#include <ESP8266WebServer.h>
#include "PubSubClient.h"       /* MQTT */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "power_meter.h"
#include "main.h"
#include "common.h"
#include "config.h"
#include "secrets.h"  // add WLAN Credentials in here.

#include <FS.h>        // File System for Web Server Files
#include <LittleFS.h>  // This file system is used.

#if ENABLE_POWER_METER

/* Contents of this array is read from file "obis2mqtt.txt" at startup */
obis_code_to_mqtt_topic_t obis2mqtt[MAX_OBIS_CODE_NUM];
uint8_t obis2mqtt_length = 0;
/* "obis2mqtt.txt" has been processed and obis2mqtt array is valid */
bool obis2mqtt_valid = false;
uint8_t obis2mqtt_global_flags = OBIS2MQTT_GLOBAL_FLAG_REMOVE_UNIT | OBIS2MQTT_GLOBAL_FLAG_PUBLISH_ALL;

power_meter_state_machine_t power_meter_state = PMSTATE_UNDEFINED;
String pm_identifier;

/* Last received data */
obis_data_t obis_data[MAX_OBIS_CODE_NUM];
/* Number of valid data in obis_data */
uint8_t obis_data_length = 0;
/* Actual index of obis_data to publish via MQTT */
uint8_t obis_data_publish_idx = UINT8_MAX;
uint32_t obis_data_timestamp_ms = 0;
/* Currently received data (temporary, it might be unfinished)*/
obis_data_t obis_data_processing[MAX_OBIS_CODE_NUM];
/* Actual index in obis_data_processing */
uint8_t obis_data_processing_idx = 0;

#if ENABLE_POWER_METER_RELAY
String import_power_obis_code = "";
String current_obis_code = "";
uint8_t minimum_current_A = 0;
float import_power_threshold_kW = 0.0f;
uint32_t relay_cooldown_time_ms = 0;

bool_t prev_relay_is_on = FALSE;
bool_t relay_is_on = FALSE;
uint32_t relay_cooldown_timestamp_ms = 0;

float    import_power_kW = 0.0f;
uint16_t current_A = 0;

#if ENABLE_POWER_METER_DIGI_POT
uint16_t max_current_A = POWER_METER_DIGI_POT_MAX_CURRENT_A;
uint8_t power_meter_digi_pot_value = POWER_METER_DIGI_POT_MAX;
uint8_t power_meter_digi_pot_table[POWER_METER_DIGI_POT_MAX_CURRENT_A + 1] = { 0xFF };
uint32_t power_meter_digi_pot_timestamp_ms = 0;
uint32_t power_meter_digi_pot_time_period_ms = 0;
bool_t power_meter_digi_pot_recalculate = FALSE;
#endif /* ENABLE_POWER_METER_DIGI_POT */
#endif /* ENABLE_POWER_METER_RELAY */

#if ENABLE_POWER_METER_REQUEST && POWER_METER_REQUEST_PERIOD_MS > 0 && POWER_METER_REQUEST_WIDTH_MS > 0
uint32_t power_meter_request_timestamp_ms = 0;
#endif

#if ENABLE_POWER_METER_LCD
#define SCREEN_WIDTH_INIT   128 // OLED display width, in pixels
#define SCREEN_HEIGHT_INIT  64  // OLED display height, in pixels
#define OLED_RESET          -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_I2C_ADDRESS  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH        64 // OLED display width, in pixels
#define SCREEN_HEIGHT       48  // OLED display height, in pixels
#define SCREEN_OFFSET_X     32
#define SCREEN_OFFSET_Y     16
#define SCREEN_TEXT_HEIGHT  17
#define SCREEN_TEXT_WIDTH   12
Adafruit_SSD1306 display(SCREEN_WIDTH_INIT, SCREEN_HEIGHT_INIT, &Wire, OLED_RESET);
#endif /* ENABLE_POWER_METER_LCD */
#endif /* ENABLE_POWER_METER */

// The text of builtin files are in this header file
#include "builtinfiles.h"

#if ENABLE_POWER_METER

/**
 * @brief Clear text row on LCD.
 * Note: No display.display() is called at the end!
 *
 * @param text_y Index of row to clear. Range: 0..3 (when font size 2 is used)
 */
void power_meter_lcd_clear_row(uint8_t text_y)
{
#if ENABLE_POWER_METER_LCD
    uint8_t y;
    /* Clear previous text */
    for (y = SCREEN_OFFSET_Y + (text_y * SCREEN_TEXT_HEIGHT);
         y < SCREEN_OFFSET_Y + ((text_y + 1) * SCREEN_TEXT_HEIGHT);
         y++)
    {
        display.drawFastHLine(SCREEN_OFFSET_X, y, SCREEN_WIDTH, BLACK);
    }
#endif /* ENABLE_POWER_METER_LCD */
}

/**
 * @brief Clear text row on LCD and display.
 * Note: display.display() is called at the end!
 *
 * @param text_y Index of row to clear. Range: 0..3 (when font size 2 is used)
 */
void power_meter_lcd_clear_row_and_display(uint8_t text_y)
{
#if ENABLE_POWER_METER_LCD
    uint8_t y;
    /* Clear previous text */
    for (y = SCREEN_OFFSET_Y + (text_y * SCREEN_TEXT_HEIGHT);
         y < SCREEN_OFFSET_Y + ((text_y + 1) * SCREEN_TEXT_HEIGHT);
         y++)
    {
        display.drawFastHLine(SCREEN_OFFSET_X, y, SCREEN_WIDTH, BLACK);
    }
    display.display();
#endif /* ENABLE_POWER_METER_LCD */
}

/**
 * Printf function which uses LCD interface to send.
 * Clears necessary area before printing.
 *
 * Example:
<pre>
LCD_printf (0, 0, "N:%02i\r\n", number);
</pre>
 *
 * @param text_x Coordinate X on LCD.
 * @param text_y Coordinate Y on LCD.
 * @param fmt Printf format string. Example: "Value: %02i\r\n"
 */
void power_meter_lcd_printf(uint8_t text_x, uint8_t text_y, const char *fmt, ...)
{
#if ENABLE_POWER_METER_LCD
    static va_list valist;
    char buf[16];
    uint8_t y_min = SCREEN_OFFSET_Y + (text_y * SCREEN_TEXT_HEIGHT);
    uint8_t y_max = SCREEN_OFFSET_Y + ((text_y + 1) * SCREEN_TEXT_HEIGHT);
    uint8_t y;
    uint8_t x = SCREEN_OFFSET_X + (text_x * SCREEN_TEXT_WIDTH);
    uint8_t width;

    va_start (valist, fmt);
    vsnprintf (buf, sizeof (buf), fmt, valist);
    va_end (valist);

    width = strlen(buf) * SCREEN_TEXT_WIDTH;
    /* Clear previous text */
    for (y = y_min; y < y_max; y++)
    {
        display.drawFastHLine(x, y, width, BLACK);
    }
    display.setCursor(x, y_min);
    display.print(buf);
    display.display();
#endif
}

#if ENABLE_POWER_METER_RELAY
/**
 * @brief Switches relay on or off.
 *
 * Publishes relay status over MQTT and displays status on LCD too.
 *
 * @param on TRUE: switch relay on. FALSE: switch relay off.
 */
void power_meter_switch_relay(bool_t on)
{
#if ENABLE_MQTT_CLIENT
    String   mqttTopic;
    boolean  ok;
    String   mqttMsg;

    mqttTopic = mqttTopicPrefix + "relay";
#endif
    if (on)
    {
#if ENABLE_POWER_METER_RELAY_DEBUG
        TRACE("Switching relay ON\n");
#endif
        digitalWrite(POWER_METER_RELAY_PIN, HIGH);
#if ENABLE_MQTT_CLIENT
        mqttMsg = "1";
#endif
    }
    else
    {
#if ENABLE_POWER_METER_RELAY_DEBUG
        TRACE("Switching relay OFF\n");
#endif
        digitalWrite(POWER_METER_RELAY_PIN, LOW);
#if ENABLE_MQTT_CLIENT
        mqttMsg = "0";
#endif
    }

#if ENABLE_MQTT_CLIENT
    if (mqttClient.connected())
    {
        ok = mqttClient.publish(mqttTopic.c_str(), mqttMsg.c_str());
        if (!ok)
        {
            TRACE("ERROR: ");
        }
        TRACE("Publish %s, %s\n", mqttTopic.c_str(), mqttMsg.c_str());
    }
#endif

    /* Draw new text */
    power_meter_lcd_printf(0, 0, on ? "ON " : "OFF");
}

/**
 * @brief It controls relay switching.
 *
 * If relay is off:
 * If 0 energy is imported, actual current represents the exported current.
 * If exported current is over threshold, it switches on the relay.
 *
 * If relay is on:
 * If imported power is over threshold, it switches off the relay.
 */
void power_meter_relay_task()
{
    uint8_t  i;
    String   obis_data_str;

    /* Check if OBIS codes and minimum current are configured via "relay.txt" */
    if (!import_power_obis_code.isEmpty() && !current_obis_code.isEmpty())
    {
        /* Retrieving import power (Watts) and actual current (Ampers) */
        for (i = 0; i < obis_data_length; i++)
        {
            if (obis_data[i].obis_code == import_power_obis_code)
            {
                obis_data_str = obis_data[i].data_without_unit;
                import_power_kW = obis_data_str.toFloat();
#if ENABLE_POWER_METER_RELAY_DEBUG
                TRACE("Import power OBIS code found, import power: %.3f\n", import_power_kW);
#endif
            }
            if (obis_data[i].obis_code == current_obis_code)
            {
                obis_data_str = obis_data[i].data_without_unit;
                current_A = obis_data_str.toInt();
#if ENABLE_POWER_METER_RELAY_DEBUG
                TRACE("Actual current OBIS code found, current: %i A\n", current_A);
#endif
            }
        }

        if (relay_is_on)
        {
            if (import_power_kW > import_power_threshold_kW)
            {
#if ENABLE_POWER_METER_RELAY_DEBUG
                FILE_TRACE("Importing power, switching relay off!\n");
#endif
                relay_is_on = FALSE;
                relay_cooldown_timestamp_ms = millis();
            }
        }
        else
        {
            /* Relay is OFF */
            if (import_power_kW == 0.0f && current_A >= minimum_current_A)
            {
                /* First line can underflow, second line can overflow so first check for underflow/overflow */
                if ((relay_cooldown_timestamp_ms > relay_cooldown_time_ms && relay_cooldown_timestamp_ms < millis() - relay_cooldown_time_ms)
                    || (relay_cooldown_timestamp_ms <= relay_cooldown_time_ms && relay_cooldown_timestamp_ms + relay_cooldown_time_ms <= millis()))
                {
#if ENABLE_POWER_METER_RELAY_DEBUG
                    FILE_TRACE("Exporting current %i A (minimum current %i A), switching relay on!\n", current_A, minimum_current_A);
#endif
                    relay_is_on = TRUE;
                }
                else
                {
#if ENABLE_POWER_METER_RELAY_DEBUG
                    FILE_TRACE("Relay cooldown prevents switching relay on for %li ms\n",
                        relay_cooldown_timestamp_ms + relay_cooldown_time_ms - millis());
#endif
                }
            }
        }
    }

    power_meter_switch_relay(relay_is_on);
    prev_relay_is_on = relay_is_on;
}

#if ENABLE_POWER_METER_DIGI_POT
void power_meter_set_sr(uint8_t value)
{
    /* Load 8-bit into 74HC595 shift register */
    SPI.transfer(value);
    /* Strobe RCLK pin to put data to output flip-flops of 74HC595 */
    digitalWrite(POWER_METER_DIGI_POT_SR_RCLK_PIN, HIGH);
    digitalWrite(POWER_METER_DIGI_POT_SR_RCLK_PIN, LOW);
}

/**
 * @brief Set value of digital potentiometer.
 *
 * Publishes digital potmeter status over MQTT and displays status on LCD as well.
 *
 * @param value Value to set. Range: POWER_METER_DIGI_POT_MIN .. POWER_METER_DIGI_POT_MAX
 */
void power_meter_set_digi_pot(uint8_t value)
{
#if ENABLE_MQTT_CLIENT
    String   mqttTopic;
    boolean  ok;
    String   mqttMsg;

    mqttTopic = mqttTopicPrefix + "digiPot";
#endif
    if (value <= POWER_METER_DIGI_POT_MAX)
    {
        TRACE("Setting power controller's digital potentiometer to %i (max. %i)\n", value, POWER_METER_DIGI_POT_MAX);
#if ENABLE_POWER_METER_DIGI_POT_SR
        uint8_t txByte = value;
        uint8_t txByteReversed = 0;
        uint8_t cntr = POWER_METER_DIGI_POT_BITS;
        /* Resistor values are printed in reversed order on PCB Rev. A, so we */
        /* reverse bits of 'value' */
        /* Note: this can be skipped if resistors are soldered in reverse order... */
        do
        {
            if (txByte & 0x01)
            {
                txByteReversed |= 0x01;
            }
            txByte >>= 1;
            if (cntr != 1)
            {
                txByteReversed <<= 1;
            }
        } while (--cntr > 0);
        power_meter_set_sr(txByteReversed);
#else
        if (value & 0x01)
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN0, HIGH);
        }
        else
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN0, LOW);
        }
#if POWER_METER_DIGI_POT_BITS >= 2
        if (value & 0x02)
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN1, HIGH);
        }
        else
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN1, LOW);
        }
#endif
#if POWER_METER_DIGI_POT_BITS >= 3
        if (value & 0x04)
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN2, HIGH);
        }
        else
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN2, LOW);
        }
#endif
#if POWER_METER_DIGI_POT_BITS >= 4
        if (value & 0x08)
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN3, HIGH);
        }
        else
        {
            digitalWrite(POWER_METER_DIGI_POT_PIN3, LOW);
        }
#endif
#endif
    }
    else
    {
        Serial.printf("ERROR: invalid digital potmeter value %i!\n", value);
    }

    /* Draw new text */
    power_meter_lcd_printf(0, 1, "dp:%-2i", value);

#if ENABLE_MQTT_CLIENT
    if (mqttClient.connected())
    {
        mqttMsg = String(value);
        ok = mqttClient.publish(mqttTopic.c_str(), mqttMsg.c_str());
        if (!ok)
        {
            TRACE("ERROR: ");
        }
        TRACE("Publish %s, %s\n", mqttTopic.c_str(), mqttMsg.c_str());
    }
#endif
}

/**
 * @brief Digital potentiometer handler.
 *
 * If relay is on, calculates the digital potentiometers value to be set.
 */
void power_meter_digi_pot_task()
{
    /* Check if digital potentiometer's table is configured via "digipot.txt" */
    /* and OBIS code is configured via "relay.txt" */
    if (power_meter_digi_pot_table[0] != 0xFF && !current_obis_code.isEmpty())
    {
        if (relay_is_on)
        {
            /* Relay is on */
            if (!prev_relay_is_on                                          // Note: only execute once after switching on relay!
                || power_meter_digi_pot_recalculate                        // or when re-calculation needed
                || power_meter_digi_pot_value == POWER_METER_DIGI_POT_MIN) // or when digital potmeter is set to zero (~0A is used!)
            {
                power_meter_digi_pot_recalculate = FALSE;
                if (current_A < max_current_A)
                {
                    /* Only need to re-calculate if not at max. current */
                    power_meter_digi_pot_timestamp_ms = millis();
                    power_meter_digi_pot_value = power_meter_digi_pot_table[current_A];
#if ENABLE_POWER_METER_RELAY_DEBUG
                    FILE_TRACE("Actual current (%i A) is below or equal to maximum current (%i A)\n", current_A, max_current_A);
#endif
                }
                else
                {
                    power_meter_digi_pot_timestamp_ms = 0;
                    power_meter_digi_pot_value = power_meter_digi_pot_table[max_current_A];
#if ENABLE_POWER_METER_RELAY_DEBUG
                    FILE_TRACE("Actual current (%i A) is over maximum current (%i A)!\n", current_A, max_current_A);
#endif
                }
            }
            if (power_meter_digi_pot_timestamp_ms != 0
                && power_meter_digi_pot_timestamp_ms + power_meter_digi_pot_time_period_ms <= millis()
                /* No need to recalculate if at maximum current */
                && power_meter_digi_pot_value < power_meter_digi_pot_table[max_current_A])
            {
                /* If re-calculation of current is needed, switch off load, */
                /* so next time we will have the maximum current value and */
                /* we can calculate the best potentiometer value. */
                power_meter_digi_pot_recalculate = TRUE;
                power_meter_digi_pot_value = POWER_METER_DIGI_POT_MIN;
#if ENABLE_POWER_METER_RELAY_DEBUG
                FILE_TRACE("Re-calculating digital potmeter value\n");
#endif
            }
        }
        else
        {
            /* Relay is off */
            power_meter_digi_pot_value = POWER_METER_DIGI_POT_MIN;
#if ENABLE_POWER_METER_RELAY_DEBUG
            FILE_TRACE("Relay is off, minimum digital potmeter value\n");
#endif
        }

    }

    power_meter_set_digi_pot(power_meter_digi_pot_value);
}
#endif // ENABLE_POWER_METER_DIGI_POT
#endif

#if ENABLE_POWER_METER_CRC_CHECK
/**
 * @brief Advance CRC16 number by one character.
 *
 * @param actual_crc16 Actual CRC16 value.
 * @param chr Character to be added to the actual CRC16 value.
 * @return uint16_t Freshly calculated CRC16 value.
 */
uint16_t update_crc16_chr(uint16_t actual_crc16, uint8_t chr)
{
    uint8_t i;

    actual_crc16 ^= chr;
    for (i = 0; i < 8; ++i)
    {
        if (actual_crc16 & 1)
        {
            actual_crc16 = (actual_crc16 >> 1) ^ 0xA001;
        }
        else
        {
            actual_crc16 = (actual_crc16 >> 1);
        }
    }
    return actual_crc16;
}

/**
 * @brief Advance CRC16 number by string.
 *
 * @param actual_crc16 Actual CRC16 value.
 * @param str String to be added to the actual CRC16 value.
 * @return uint16_t Freshly calculated CRC16 value.
 */
uint16_t update_crc16_str(uint16_t actual_crc16, String str)
{
    uint16_t i;
    uint16_t len = str.length();

    for (i = 0; i < len; ++i)
    {
        actual_crc16 = update_crc16_chr(actual_crc16, str[i]);
    }
    return actual_crc16;
}
#endif

/**
 * @brief Find OBIS code in the configuration
 *
 * @param obis_code OBIS code to find.
 * @return obis_code_to_mqtt_topic_t* Pointer to data for OBIS code: human readable string, MQTT topic
 */
obis_code_to_mqtt_topic_t * find_obis2mqtt(String& obis_code)
{
    uint16_t idx;
    obis_code_to_mqtt_topic_t * p_obis2mqtt = NULL;

    for (idx = 0; idx < obis2mqtt_length && idx < MAX_OBIS_CODE_NUM; idx++)
    {
        if (obis2mqtt[idx].obis_code == obis_code)
        {
            p_obis2mqtt = &( obis2mqtt[idx] );
            break;
        }
    }

    return p_obis2mqtt;
}

/**
 * @brief Find OBIS code in the last received OBIS data
 *
 * @param obis_code OBIS code to find.
 * @return obis_data_t* Pointer to data for OBIS code
 */
obis_data_t * find_obis_data(String& obis_code)
{
    uint16_t idx;
    obis_data_t * p_obis_data = NULL;

    for (idx = 0; idx < MAX_OBIS_CODE_NUM; idx++)
    {
        if (obis_data[idx].obis_code == obis_code)
        {
            p_obis_data = &( obis_data[idx] );
            break;
        }
    }

    return p_obis_data;
}

#if ENABLE_MQTT_CLIENT
/**
 * @brief Publish OBIS data via MQTT.
 */
void publishObisData()
{
    bool ok1 = true;
    bool ok2 = true;
    uint16_t idx;
    String   mqttTopic;
    String   data;
    obis_code_to_mqtt_topic_t * p_obis2mqtt;

    if (mqttClient.connected() &&obis_data_publish_idx < obis_data_length)
    {
        idx = obis_data_publish_idx;
#if ENABLE_POWER_METER_DEBUG >= 3
        Serial.printf("[%s]: [%s]\n", obis_data[idx].obis_code.c_str(),
                      obis_data[idx].data.c_str());
#endif
        if (obis2mqtt_global_flags & OBIS2MQTT_GLOBAL_FLAG_REMOVE_UNIT)
        {
            data = obis_data[idx].data_without_unit;
        }
        else
        {
            data = obis_data[idx].data_with_unit;
        }
        p_obis2mqtt = find_obis2mqtt(obis_data[idx].obis_code);
        if ((obis2mqtt_global_flags & OBIS2MQTT_GLOBAL_FLAG_ALWAYS_PUBLISH_OBIS_CODES) || (!p_obis2mqtt && (obis2mqtt_global_flags & OBIS2MQTT_GLOBAL_FLAG_PUBLISH_ALL)))
        {
            /* Publish data with OBIS code */
            mqttTopic = mqttTopicPrefix + obis_data[idx].obis_code;
            ok1 = mqttClient.publish(mqttTopic.c_str(), data.c_str());
            if (!ok1)
            {
                TRACE("ERROR: ");
            }
            TRACE("Publish %s, %s\n", mqttTopic.c_str(), data.c_str());
        }
        if (p_obis2mqtt && !(p_obis2mqtt->flags & OBIS2MQTT_FLAG_DO_NOT_PUBLISH))
        {
            if (p_obis2mqtt->flags & OBIS2MQTT_FLAG_REMOVE_UNIT)
            {
                data = obis_data[idx].data_without_unit;
            }
            else
            {
                data = obis_data[idx].data_with_unit;
            }
            /* Publish data with pre-defined MQTT topic */
            mqttTopic = mqttTopicPrefix + p_obis2mqtt->mqtt_topic;
            ok2 = mqttClient.publish(mqttTopic.c_str(), data.c_str());
            if (!ok2)
            {
                TRACE("ERROR: ");
            }
            TRACE("Publish %s, %s\n", mqttTopic.c_str(), data.c_str());
        }
        if (ok1 && ok2)
        {
            obis_data_publish_idx++;
        }
    }
}
#endif /* ENABLE_MQTT_CLIENT */

/**
 * @brief Processes OBIS codes coming from the smart meter through serial line (RS-232).
 *
 */
void power_meter_task()
{
    String line;
    String str;
    String obis_code;
    String data_with_unit;
    String data_without_unit;
    char c;
    uint16_t idx;
    uint32_t len;
#if ENABLE_POWER_METER_CRC_CHECK
    String crc;
    static uint16_t calculated_crc16;
    uint16_t received_crc16;
#endif

    /* Check if data can be received */
    while (Serial.available() > 0)
    {
        /* Read whole line, it ends with CR+LF */
        /* readStringUntil will cut LF, we need to cut CR at the end */
        line = Serial.readStringUntil(CHR_LF);
        len = line.length();
        c = line[len - 1];
        if (c != CHR_CR)
        {
            Serial.printf("%s ERROR: Expected CR character, received: 0x%02X\n", __FUNCTION__, (int)c);
        }
        line.remove(len - 1, 1);  /* remove last character (CR) */
        switch (power_meter_state)
        {
            default:
            case PMSTATE_UNDEFINED:
            case PMSTATE_DONE_WAIT_FOR_IDENTIFIER:
                c = line[0];
                if (c == '/')
                {
#if ENABLE_POWER_METER_CRC_CHECK
                    calculated_crc16 = update_crc16_str(0, line);
                    calculated_crc16 = update_crc16_chr(calculated_crc16, CHR_CR);
                    calculated_crc16 = update_crc16_chr(calculated_crc16, CHR_LF);
#endif
                    line.remove(0, 1); /* Remove first character '/' */
                    pm_identifier = line;
#if ENABLE_POWER_METER_DEBUG >= 1
                    setLed2(TRUE);
#endif
#if ENABLE_POWER_METER_DEBUG >= 4
                    Serial.printf("identifier: [%s]\n", pm_identifier.c_str());
#endif
                    obis_data_processing_idx = 0;
                    for (idx = 0; idx < MAX_OBIS_CODE_NUM; idx++)
                    {
                        obis_data_processing[idx].obis_code.clear();
                        obis_data_processing[idx].data_with_unit.clear();
                        obis_data_processing[idx].data_without_unit.clear();
                    }
                    power_meter_state = PMSTATE_EMPTY_LINE;
                }
                break;

            case PMSTATE_EMPTY_LINE:
#if ENABLE_POWER_METER_CRC_CHECK
                calculated_crc16 = update_crc16_chr(calculated_crc16, CHR_CR);
                calculated_crc16 = update_crc16_chr(calculated_crc16, CHR_LF);
#endif
                if (line.length())
                {
                    Serial.printf("%s ERROR: Expected empty line, received: [%s]\n", __FUNCTION__, line.c_str());
                }
                power_meter_state = PMSTATE_DATA;
                break;

            case PMSTATE_DATA:
                /* Check if not CRC */
                c = line[0];
                if (c != '!')
                {
                    /* Not CRC */
#if ENABLE_POWER_METER_CRC_CHECK
                    calculated_crc16 = update_crc16_str(calculated_crc16, line);
                    calculated_crc16 = update_crc16_chr(calculated_crc16, CHR_CR);
                    calculated_crc16 = update_crc16_chr(calculated_crc16, CHR_LF);
#endif
                    idx = line.indexOf('(');
                    obis_code = line.substring(0, idx);
                    line.remove(0, idx + 1);
#if ENABLE_POWER_METER_DEBUG >= 4
                    Serial.printf("obis_code: [%s] ", obis_code.c_str());
#endif
                    data_with_unit.clear();
                    int len = line.length();
#if ENABLE_POWER_METER_DEBUG >= 3
                    Serial.println(len);
#endif
                    len--;
                    data_with_unit.reserve(len);
                    for (idx = 0; idx < len; idx++)
                    {
                        char c = line[idx];
                        if (c != ')')
                        {
                            data_with_unit += c;
                        }
                        else
                        {
                            if (line[++idx] != '(')
                            {
                                break;
                            }
                            data_with_unit += ",";
                        }
                    }
                    if (obis_data_processing_idx < MAX_OBIS_CODE_NUM)
                    {
                        obis_data_processing[obis_data_processing_idx].obis_code = obis_code;
                        /* Remove leading zeros. Example: it converts 0000012.34 to 12.34, 0000.1234 to 0.1234 */
                        bool end = false;
                        while (!end)
                        {
                            if (data_with_unit.length() > 1 && data_with_unit[0] == '0' && data_with_unit[1] >= '0' && data_with_unit[1] <= '9')
                            {
                                data_with_unit.remove(0, 1);
                            }
                            else
                            {
                                end = true;
                            }
                        }
                        data_without_unit = data_with_unit;
                        /* Remove unit. Example: change "50.00*Hz" to "50.00" */
                        idx = data_without_unit.indexOf('*');
                        data_without_unit.remove(idx);
                        /* Replace '*' to ' '. Example: change "50.00*Hz" to "50.00 Hz" */
                        data_with_unit.replace('*', ' ');
                        obis_data_processing[obis_data_processing_idx].data_with_unit = data_with_unit;
                        obis_data_processing[obis_data_processing_idx].data_without_unit = data_without_unit;
                        obis_data_processing_idx++;
                    }
                    else
                    {
                        Serial.printf("%s ERROR: Too many OBIS data! Increase MAX_OBIS_CODE_NUM!\n", __FUNCTION__);
                    }
#if ENABLE_POWER_METER_DEBUG >= 4
                    Serial.printf("data: [%s]\n", data.c_str());
#endif
                    break;
                }
                else
                {
                    power_meter_state = PMSTATE_CRC;
                }
                /* If line starts with '!', CRC is coming, fall through next state */

            case PMSTATE_CRC:
                /* The '!' at the beginning of line shall be also taken into account when calculating CRC! */
#if ENABLE_POWER_METER_CRC_CHECK
                calculated_crc16 = update_crc16_chr(calculated_crc16, '!');
                crc = line.substring(1);
                received_crc16 = strtol(crc.c_str(), NULL, 16);
#if ENABLE_POWER_METER_DEBUG >= 2
                Serial.printf("Received CRC: [%s] 0x%X calculated CRC: 0x%X\n", crc.c_str(), received_crc16, calculated_crc16);
#endif
#endif /* ENABLE_POWER_METER_CRC_CHECK */
#if ENABLE_POWER_METER_CRC_CHECK
                if (received_crc16 == calculated_crc16)
#endif
                {
                    /* Copy temporary data to obis_data, which can be shown on index page */
                    /* AND can be published via MQTT */
                    for (idx = 0; idx < obis_data_processing_idx && idx < MAX_OBIS_CODE_NUM; idx++)
                    {
                        obis_data[idx] = obis_data_processing[idx];
                    }
                    obis_data_length = obis_data_processing_idx;
                    obis_data_publish_idx = 0; /* Start publish OBIS data by index 0 */
                    obis_data_timestamp_ms = millis();
#if ENABLE_POWER_METER_RELAY
                    power_meter_relay_task();
#if ENABLE_POWER_METER_DIGI_POT
                    power_meter_digi_pot_task();
#endif /* ENABLE_POWER_METER_DIGI_POT */
#endif /* ENABLE_POWER_METER_RELAY */
                }
#if ENABLE_POWER_METER_CRC_CHECK
                else
                {
                    Serial.printf("%s ERROR: CRC mismatch! Received CRC: [%s] 0x%X calculated CRC: 0x%X\n",
                                  __FUNCTION__, crc.c_str(), received_crc16, calculated_crc16);
                }
#endif /* ENABLE_POWER_METER_CRC_CHECK */
#if ENABLE_POWER_METER_DEBUG >= 1
                setLed2(FALSE);
#endif
                power_meter_state = PMSTATE_DONE_WAIT_FOR_IDENTIFIER;
                break;
        }
    }

#if ENABLE_POWER_METER_LCD
    static uint16_t prev_remaining_time_sec = UINT16_MAX;
    bool_t display_remaining_time = FALSE;
    static bool_t prev_display_remaining_time = FALSE;
    uint16_t remaining_time_sec;

    if (relay_is_on
        && power_meter_digi_pot_timestamp_ms != 0
        && power_meter_digi_pot_timestamp_ms + power_meter_digi_pot_time_period_ms > millis())
    {
        remaining_time_sec = (power_meter_digi_pot_timestamp_ms + power_meter_digi_pot_time_period_ms - millis()) / 1000;
        display_remaining_time = TRUE;
    }
    if ((display_remaining_time && prev_remaining_time_sec != remaining_time_sec)
        || (prev_display_remaining_time != display_remaining_time))
    {
        if (display_remaining_time)
        {
            /* Draw new text */
            power_meter_lcd_printf(0, 2, "%-5i", remaining_time_sec);
            prev_remaining_time_sec = remaining_time_sec;
        }
        else
        {
            power_meter_lcd_clear_row_and_display(2);
        }
    }
    prev_display_remaining_time = display_remaining_time;

    static bool_t prev_power_meter_digi_pot_recalculate = (bool_t)-1;
    if (prev_power_meter_digi_pot_recalculate != power_meter_digi_pot_recalculate)
    {
        if (power_meter_digi_pot_recalculate)
        {
            power_meter_lcd_printf(0, 2, "Recal");
        }
        else
        {
            power_meter_lcd_printf(0, 2, "-----");
        }
        prev_power_meter_digi_pot_recalculate = power_meter_digi_pot_recalculate;
    }
#endif
#if ENABLE_POWER_METER_REQUEST && POWER_METER_REQUEST_PERIOD_MS > 0 && POWER_METER_REQUEST_WIDTH_MS > 0
    uint32_t now = millis();
    const uint32_t t1 = POWER_METER_REQUEST_PERIOD_MS;
    const uint32_t t2 = POWER_METER_REQUEST_PERIOD_MS + POWER_METER_REQUEST_WIDTH_MS;
    if (now > t1 && power_meter_request_timestamp_ms < now - t1)
    {
        digitalWrite(POWER_METER_REQUEST_PIN, HIGH);
    }
    if (now > t2 && power_meter_request_timestamp_ms < now - t2)
    {
        digitalWrite(POWER_METER_REQUEST_PIN, LOW);
        power_meter_request_timestamp_ms = millis();
    }
#endif
}

#if ENABLE_POWER_METER_DIGI_POT_TEST_BTN
/**
 * @brief Read buttons connected to analog input (A0)
 *
 *      ___ 10k            ___ 4k7
 * A0 -|___|----+---BTN0--|___|---
 *              |                _|_
 *              |
 *              |          ___ 10k
 *              +---BTN1--|___|---
 *                               _|_
 *
 * Returns:
 * 0 - if no buttons pressed
 * 1 - if BTN0 is pressed
 * 2 - if BTN1 is pressed
 * 3 - if BTN0 and BTN1 are also pressed
 */
uint8_t power_meter_read_btn_int()
{
    uint8_t  btn = 0;
    uint16_t analog = analogRead(A0);

    if (analog < 280)
    {
        btn = 3;
    }
    else if (analog >= 320 && analog <= 380)
    {
        btn = 1;
    }
    else if (analog >= 500 && analog <= 900)
    {
        btn = 2;
    }

    return btn;
}
#endif

uint8_t power_meter_read_btn(bool_t wait_for_release = TRUE)
{
    uint8_t  btn = 0;

#if ENABLE_POWER_METER_DIGI_POT_TEST_BTN
    btn = power_meter_read_btn_int();
    /* Check if button pressed */
    if (btn != 0 && wait_for_release)
    {
        /* Read again, if both button pressed */
        delay(10);
        btn = power_meter_read_btn_int();
        /* Wait until all buttons released */
        while (analogRead(A0) < 900)
        {
            delay(10);
        }
    }
#endif

    return btn;
}

void power_meter_digi_pot_test()
{
#if 0
    /* Test 74HC595 shift register */
    uint16_t x = 1;
    while (x <= 0x80)
    {
        Serial.println(x);
        power_meter_set_sr((uint8_t)x);
        delay(500);
        x <<= 1;
    }
    x >>= 2;
    while (x != 0x00)
    {
        Serial.println(x);
        power_meter_set_sr((uint8_t)x);
        delay(500);
        x >>= 1;
    }
    power_meter_set_sr(0);
#endif
#if ENABLE_POWER_METER_DIGI_POT_TEST
#if POWER_METER_DIGI_POT_TEST_INPUT_PIN == 0
    uint8_t cntr = POWER_METER_DIGI_POT_MAX;
#else
    pinMode(POWER_METER_DIGI_POT_TEST_INPUT_PIN, INPUT_PULLUP);
    TRACE("Check if GPIO%i is LOW... ", POWER_METER_DIGI_POT_TEST_INPUT_PIN);
    bool_t test_input_is_low = digitalRead(POWER_METER_DIGI_POT_TEST_INPUT_PIN) == LOW;
    if (!test_input_is_low)
    {
        TRACE("HIGH, skipping digital potmeter test\n");
    }
    if (test_input_is_low)
#endif
    {
        TRACE("LOW, starting digital potmeter test\n");
        power_meter_switch_relay(TRUE);
        power_meter_lcd_printf(0, 2, "Auto");
        uint8_t value = POWER_METER_DIGI_POT_MIN;
#if POWER_METER_DIGI_POT_TEST_INPUT_PIN == 0
        while (cntr--)
#else
        while (digitalRead(POWER_METER_DIGI_POT_TEST_INPUT_PIN) == LOW && power_meter_read_btn() == 0)
#endif
        {
            if (value > POWER_METER_DIGI_POT_MAX)
            {
                value = POWER_METER_DIGI_POT_MIN;
            }
            power_meter_set_digi_pot(value);
            uint32_t now = millis();
            while (now + 5000 > millis())
            {
                if (digitalRead(POWER_METER_DIGI_POT_TEST_INPUT_PIN) == LOW && power_meter_read_btn(FALSE) == 0)
                {
                    delay(10);
                }
                else
                {
                    break;
                }
            }
            value++;
        }
#if ENABLE_POWER_METER_DIGI_POT_TEST_BTN
        value--;
        power_meter_lcd_printf(0, 2, "Manu");
        bool_t btn0Pressed = FALSE;
        bool_t btn1Pressed = FALSE;
        while (!btn0Pressed || !btn1Pressed)
        {
            uint8_t btn;
            /* Wait for pressing one of buttons */
            while ((btn = power_meter_read_btn()) == 0)
            {
                delay(20);
            }
            btn0Pressed = btn & 1;
            btn1Pressed = btn & 2;
            if (btn0Pressed)
            {
                if (value >= POWER_METER_DIGI_POT_MAX)
                {
                    value = POWER_METER_DIGI_POT_MIN;
                }
                else
                {
                    value++;
                }
            }
            if (btn1Pressed)
            {
                if (value == POWER_METER_DIGI_POT_MIN)
                {
                    value = POWER_METER_DIGI_POT_MAX;
                }
                else
                {
                    value--;
                }
            }
            power_meter_set_digi_pot(value);
        }
#endif
        power_meter_lcd_clear_row_and_display(2);
    }
#endif /* ENABLE_POWER_METER_DIGI_POT_TEST_BTN */
#endif /* ENABLE_POWER_METER_DIGI_POT_TEST */
}

/**
 * @brief Initialize smart meter reading.
 * It reads configuration from file.
 */
void power_meter_init()
{
    String str;
    String line;
    String obis_code;
    String mqtt_topic;
    String human_readable_string;
    uint8_t flags;                  /* See OBIS2MQTT_FLAG_* */
    uint16_t idx;
    int16_t comma_idx;
    int16_t comma_idx2;
    int16_t comma_idx3;
    bool ok;

#if ENABLE_POWER_METER_REQUEST
    pinMode(POWER_METER_REQUEST_PIN, OUTPUT);
    digitalWrite(POWER_METER_REQUEST_PIN, HIGH);
#if POWER_METER_REQUEST_PERIOD_MS > 0 && POWER_METER_REQUEST_WIDTH_MS > 0
    power_meter_request_timestamp_ms = millis();
#endif
#endif /* ENABLE_POWER_METER_REQUEST */
    TRACE("Reading OBIS2MQTT configuration\n");
    File file = LittleFS.open(POWER_METER_CONFIG_FILE, "r");
    if (file)
    {
        line = file.readStringUntil('\n');
        /* Very first line shall contain the global flags (one integer number) */
        /* See OBIS2MQTT_GLOBAL_FLAG_* */
        obis2mqtt_global_flags = line.toInt();
        TRACE("OBIS2MQTT global flags: %i\n", obis2mqtt_global_flags);

        obis2mqtt_length = 0;
        idx = 0;
        obis2mqtt_valid = true;
        while (file.available() && obis2mqtt_valid)
        {
            line = file.readStringUntil('\n');
            int commentCharIdx = line.indexOf(COMMENT_CHAR);
            /* Check if comment character is in the string */
            if (commentCharIdx != -1)
            {
                /* Remove comment character and evrything after that */
                line.remove(commentCharIdx);
            }
            //TRACE("Line #%i from file '%s': '%s'\n", idx, POWER_METER_CONFIG_FILE, line.c_str());
            comma_idx = line.indexOf(',');
            if (comma_idx < 0)
            {
                Serial.printf("ERROR: syntax error, no comma found at line %i: '%s'\n", idx + 1, line.c_str());
                obis2mqtt_valid = false;
            }
            comma_idx2 = line.indexOf(',', comma_idx + 1);
            if (comma_idx2 < 0)
            {
                Serial.printf("ERROR: syntax error, no 2nd comma found at line %i: '%s'\n", idx + 1, line.c_str());
                obis2mqtt_valid = false;
            }
            comma_idx3 = line.indexOf(',', comma_idx2 + 1);
            if (comma_idx3 < 0)
            {
                Serial.printf("ERROR: syntax error, no 3rd comma found at line %i: '%s'\n", idx + 1, line.c_str());
                obis2mqtt_valid = false;
            }
            if (obis2mqtt_valid)
            {
                obis_code = line.substring(0, comma_idx);
                mqtt_topic = line.substring(comma_idx + 1, comma_idx2);
                human_readable_string = line.substring(comma_idx2 + 1, comma_idx3);
                flags = line.substring(comma_idx3 + 1).toInt();
                Serial.printf("OBIS code: [%s], MQTT topic: [%s] Human str: [%s] flags: %i\n",
                              obis_code.c_str(), mqtt_topic.c_str(), human_readable_string.c_str(), flags);
                obis2mqtt[idx].obis_code = obis_code;
                obis2mqtt[idx].mqtt_topic = mqtt_topic;
                obis2mqtt[idx].human_readable_string = human_readable_string;
                obis2mqtt[idx].flags = flags;
            }
            idx++;
        }
        if (obis2mqtt_valid)
        {
            obis2mqtt_length = idx;
            TRACE("%i lines successfully processed\n", idx + 1);
        }
    }
    else
    {
        Serial.printf("ERROR: Failed to open file %s for reading\n", POWER_METER_CONFIG_FILE);
    }

#if ENABLE_POWER_METER_LCD
    ok = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDRESS);
    if (ok)
    {
        TRACE("LCD initialized successfully\n");
        // Clear the buffer
        display.clearDisplay();
        display.drawRect(SCREEN_OFFSET_X, SCREEN_OFFSET_Y, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
        display.display();
        delay(1000);
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
#if ENABLE_POWER_METER_RELAY
        display.setCursor(SCREEN_OFFSET_X, SCREEN_OFFSET_Y);
        display.println("---");
#if ENABLE_POWER_METER_DIGI_POT
        display.setCursor(SCREEN_OFFSET_X, SCREEN_OFFSET_Y + SCREEN_TEXT_HEIGHT);
        display.printf("dp:-");
#endif /* ENABLE_POWER_METER_DIGI_POT */
#endif /* ENABLE_POWER_METER_RELAY */
        display.display();
    }
    else
    {
        TRACE("ERROR: Cannot initialize LCD!\n");
    }
#endif /* ENABLE_POWER_METER_LCD */

#if ENABLE_POWER_METER_RELAY
    TRACE("Configuring relay output\n");
    pinMode(POWER_METER_RELAY_PIN, OUTPUT);
    power_meter_switch_relay(FALSE);
    import_power_obis_code = readStringFromFile(POWER_METER_RELAY_CONFIG_FILE, 0);
    current_obis_code = readStringFromFile(POWER_METER_RELAY_CONFIG_FILE, 1);
    str = readStringFromFile(POWER_METER_RELAY_CONFIG_FILE, 2);
    minimum_current_A = str.toInt();
    str = readStringFromFile(POWER_METER_RELAY_CONFIG_FILE, 3);
    import_power_threshold_kW  = str.toFloat();
    str = readStringFromFile(POWER_METER_RELAY_CONFIG_FILE, 4);
    relay_cooldown_time_ms = str.toInt();

#if ENABLE_POWER_METER_DIGI_POT
    TRACE("Configuring digital potmeter output\n");
#if ENABLE_POWER_METER_DIGI_POT_SR
    pinMode(POWER_METER_DIGI_POT_SR_RCLK_PIN, OUTPUT);
    SPI.begin();
    SPI.setFrequency(10000000); /* 10 MHz */
#if LED2_PIN == PIN_SPI_MISO || LED2_PIN == PIN_SPI_SS
    /* Re-initialize LED2 pin if interferes with SPI
       pins. MISO and SS pins are not used by shift register!
    */
    pinMode(LED2_PIN, OUTPUT);
    setLed2(FALSE);
#endif
#else
    pinMode(POWER_METER_DIGI_POT_PIN0, OUTPUT);
#if POWER_METER_DIGI_POT_BITS >= 2
    pinMode(POWER_METER_DIGI_POT_PIN1, OUTPUT);
#endif
#if POWER_METER_DIGI_POT_BITS >= 3
    pinMode(POWER_METER_DIGI_POT_PIN2, OUTPUT);
#endif
#if POWER_METER_DIGI_POT_BITS >= 4
    pinMode(POWER_METER_DIGI_POT_PIN3, OUTPUT);
#endif
#endif
    power_meter_digi_pot_value = POWER_METER_DIGI_POT_MIN;
    power_meter_set_digi_pot(power_meter_digi_pot_value);
    str = readStringFromFile(POWER_METER_DIGI_POT_CONFIG_FILE, 0);
    if (str.length())
    {
        power_meter_digi_pot_time_period_ms = str.toInt() * 1000;
        TRACE("Digital potentiometer time period %i ms\n", power_meter_digi_pot_time_period_ms);
    }
    str = readStringFromFile(POWER_METER_DIGI_POT_CONFIG_FILE, 1);
    if (str.length())
    {
        max_current_A = str.toInt();
        if (max_current_A >= POWER_METER_DIGI_POT_MAX_CURRENT_A)
        {
            ERROR("Invalid maximum current %i A found in %s!\n", max_current_A, POWER_METER_DIGI_POT_CONFIG_FILE);
            max_current_A = POWER_METER_DIGI_POT_MAX_CURRENT_A;
        }
    }
    TRACE("Maximum current %i A\n", max_current_A);
    TRACE("Maximum digital potentiometer value %i\n", POWER_METER_DIGI_POT_MAX);
    for (idx = 0; idx <= max_current_A; idx++)
    {
        str = readStringFromFile(POWER_METER_DIGI_POT_CONFIG_FILE, idx + 2);
        if (str.length())
        {
            power_meter_digi_pot_table[idx] = str.toInt();
            if (power_meter_digi_pot_table[idx] > POWER_METER_DIGI_POT_MAX)
            {
                ERROR("Invalid digital potentiometer value %i found in %s!\n",
                      power_meter_digi_pot_table[idx], POWER_METER_DIGI_POT_CONFIG_FILE);
                power_meter_digi_pot_table[idx] = POWER_METER_DIGI_POT_MAX;
            }
        }
    }
    for (idx = 0; idx <= max_current_A; idx++)
    {
        TRACE("Digital potentiometer value for %i A: %i\n", idx, power_meter_digi_pot_table[idx]);
    }
#if ENABLE_POWER_METER_DIGI_POT_TEST
    power_meter_digi_pot_test();
#endif /* ENABLE_POWER_METER_DIGI_POT_TEST */
#endif /* ENABLE_POWER_METER_DIGI_POT */
#if ENABLE_POWER_METER_RELAY_TEST
    TRACE("Switching relay on for 2 seconds...\n");
    power_meter_lcd_printf(0, 1, "Relay");
    power_meter_lcd_printf(0, 2, "test!");
    power_meter_switch_relay(TRUE);
    delay(2000);
    power_meter_switch_relay(FALSE);
    power_meter_lcd_clear_row_and_display(1);
    power_meter_lcd_clear_row_and_display(2);
#endif /* ENABLE_POWER_METER_RELAY_TEST */
#if ENABLE_POWER_METER_RELAY && ENABLE_POWER_METER_DIGI_POT
    power_meter_set_digi_pot(power_meter_digi_pot_value);
#endif /* ENABLE_POWER_METER_RELAY && ENABLE_POWER_METER_DIGI_POT */
}
#endif
