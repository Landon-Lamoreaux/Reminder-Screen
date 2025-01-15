#include <Arduino.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <lvgl.h>
#include <examples/lv_examples.h>
#include "../lv_conf.h"
#include <WiFiMulti.h>

/* ------------------------------ Defines ------------------------------ */

// Wifi login credentials.
#define WIFI_SSID "TDS0277"
#define WIFI_PASS "bgqp25thht"

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

/* ------------------------------ Functions ------------------------------ */

// Function to connect to a wifi network with the defined wifi credentials.
void connect_to_wifi()
{
    WiFiMulti wifiMulti;

    // Adding an access point to the wifi variable.
    wifiMulti.addAP(WIFI_SSID, WIFI_PASS);

    // Attemting to connect to the wifi network.
    while (wifiMulti.run() != WL_CONNECTED)
    {
        sleep(0.1);
        Serial.println("Connecting to Wifi.");
    }

    // Setting hostname and printing IP address on successful connection.
    WiFi.setHostname("Reminder Screen.");
    Serial.println("Connected to Wifi");
    Serial.println(WiFi.localIP());
}