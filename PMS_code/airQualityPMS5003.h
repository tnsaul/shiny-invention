/**
  Particulate matter sensor firmware for D1 Mini (ESP8266) and PMS5003

  Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1
  Mini (or other ESP8266-based board) and report the values to an MQTT
  broker and to the serial console. Also optionally show them on a 128x32
  I2C OLED display, with a mode button to change between display modes.

  External dependencies. Install using the Arduino library manager:
     "Adafruit GFX Library" by Adafruit
     "Adafruit SSD1306" by Adafruit
     "PubSubClient" by Nick O'Leary

  Bundled dependencies. No need to install separately:
     "PMS Library" by Mariusz Kacki, forked by SwapBap

  Written by Jonathan Oxer for www.superhouse.tv
    https://github.com/superhouse/AirQualitySensorD1Mini

  Inspired by https://github.com/SwapBap/WemosDustSensor/

  Copyright 2020 SuperHouse Automation Pty Ltd www.superhouse.tv
*/
//#define VERSION "2.6"  // Last version from JOxer
#define VERSION "3.0"   // TNS20210405 cloned above and cut to fit Home Assistant pattern
#include "esphome.h"
/*--------------------------- Configuration ------------------------------*/
// Configuration should be done in the included file:
//#include "config.h"

/*--------------------------- Libraries ----------------------------------*/

#include <SoftwareSerial.h>           // Allows PMS to avoid the USB serial port
#include "./PMS.h"                    // Particulate Matter Sensor driver (embedded)

/*--------------------------- Global Variables ---------------------------*/
// Particulate matter sensor
#define   PMS_STATE_ASLEEP        0   // Low power mode, laser and fan off
#define   PMS_STATE_WAKING_UP     1   // Laser and fan on, not ready yet
#define   PMS_STATE_READY         2   // Warmed up, ready to give data
uint8_t   g_pms_state           = PMS_STATE_WAKING_UP;
uint32_t  g_pms_state_start     = 0;  // Timestamp when PMS state last changed
uint8_t   g_pms_ae_readings_taken  = false;  // true/false: whether any readings have been taken
uint8_t   g_pms_ppd_readings_taken = false;  // true/false: whether PPD readings have been taken

uint16_t  g_pm1p0_sp_value      = 0;  // Standard Particle calibration pm1.0 reading
uint16_t  g_pm2p5_sp_value      = 0;  // Standard Particle calibration pm2.5 reading
uint16_t  g_pm10p0_sp_value     = 0;  // Standard Particle calibration pm10.0 reading

uint16_t  g_pm1p0_ae_value      = 0;  // Atmospheric Environment pm1.0 reading
uint16_t  g_pm2p5_ae_value      = 0;  // Atmospheric Environment pm2.5 reading
uint16_t  g_pm10p0_ae_value     = 0;  // Atmospheric Environment pm10.0 reading

uint32_t  g_pm0p3_ppd_value     = 0;  // Particles Per Deciliter pm0.3 reading
uint32_t  g_pm0p5_ppd_value     = 0;  // Particles Per Deciliter pm0.5 reading
uint32_t  g_pm1p0_ppd_value     = 0;  // Particles Per Deciliter pm1.0 reading
uint32_t  g_pm2p5_ppd_value     = 0;  // Particles Per Deciliter pm2.5 reading
uint32_t  g_pm5p0_ppd_value     = 0;  // Particles Per Deciliter pm5.0 reading
uint32_t  g_pm10p0_ppd_value    = 0;  // Particles Per Deciliter pm10.0 reading

uint8_t   g_uk_aqi_value        = 0;  // Air Quality Index value using UK reporting system
uint16_t  g_us_aqi_value        = 0;  // Air Quality Index value using US reporting system


// General
uint32_t g_device_id;                    // Unique ID from ESP chip ID

/*--------------------------- Function Signatures ------------------------*/
void updatePmsReadings();

/* -------------------------- Resources ----------------------------------*/
//#include "aqi.h"                         // Air Quality Index calculations

/*--------------------------- Instantiate Global Objects -----------------*/
// Software serial port
//SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN); // Rx pin = GPIO2 (D4 on Wemos D1 Mini)
//UARTComponent *pmsSer;

// Particulate matter sensor
PMS pms;                      // Use the software serial port for the PMS
PMS::DATA g_data;

/*--------------------------- Program ------------------------------------*/
class PMS5003CustomSensor : public Component, public Sensor, public UARTDevice
{
    public:
        Sensor *pms5003_PM1_sensor = new Sensor();
        Sensor *pms5003_PM2_5_sensor = new Sensor();
        Sensor *pms5003_PM10_sensor = new Sensor();
        Sensor *pms5003_PB0_3_sensor = new Sensor();
        Sensor *pms5003_PB0_5_sensor = new Sensor();
        Sensor *pms5003_PB1_0_sensor = new Sensor();
        Sensor *pms5003_PB2_5_sensor = new Sensor();
        Sensor *pms5003_PB5_0_sensor = new Sensor();
        Sensor *pms5003_PB10_sensor = new Sensor();
        Sensor *pms5003_UKAQI_sensor = new Sensor();

        // Constructor.
        PMS5003CustomSensor(UARTComponent *parent) : UARTDevice(parent) 
        {
            pms.setupUART(this);
            ESP_LOGD(TAG, "Instantiated.");
        }

        float get_setup_priority() const override { return esphome::setup_priority::LATE; }


        /**
         Setup
        */
        void setup() override 
        {

            ESP_LOGD(TAG, "Air Quality Sensor starting up, v%ld", VERSION);
            // Open a connection to the PMS and put it into passive mode
            //pmsSerial.begin(PMS_BAUD_RATE);   // Connection for PMS5003
            pms.passiveMode();                // Tell PMS to stop sending data automatically
            delay(100);
            pms.wakeUp();                     // Tell PMS to wake up (turn on fan and laser)

            // We need a unique device ID for our MQTT client connection
            g_device_id = ESP.getChipId();  // Get the unique ID of the ESP8266 chip
            ESP_LOGD(TAG, "Device ID: %lx", g_device_id);
        }

        /**
         Main loop
        */
        void loop() override
        {
            updatePmsReadings();
        }


    private:
        /**
         Update particulate matter sensor values
        */
        void updatePmsReadings()
        {
            uint32_t time_now = millis();
            // Check if we've been in the sleep state for long enough
            if (PMS_STATE_ASLEEP == g_pms_state)
            {
                if (time_now - g_pms_state_start
                    >= ((g_pms_report_period * 1000) - (g_pms_warmup_period * 1000)))
                {
                // It's time to wake up the sensor
                ESP_LOGD(TAG, "Waking up sensor");
                pms.wakeUp();
                g_pms_state_start = time_now;
                g_pms_state = PMS_STATE_WAKING_UP;
                }
            }

            // Check if we've been in the waking up state for long enough
            if (PMS_STATE_WAKING_UP == g_pms_state)
            {
                if (time_now - g_pms_state_start
                    >= (g_pms_warmup_period * 1000))
                {
                g_pms_state_start = time_now;
                g_pms_state = PMS_STATE_READY;
                }
            }

            // Put the *most recent* values into globals for reference elsewhere
            if (PMS_STATE_READY == g_pms_state)
            {
                //pms.requestRead();
                if (pms.readUntil(g_data))  // Use a blocking read to make sure we get values
                {
                g_pm1p0_sp_value   = g_data.PM_SP_UG_1_0;
                g_pm2p5_sp_value   = g_data.PM_SP_UG_2_5;
                g_pm10p0_sp_value  = g_data.PM_SP_UG_10_0;

                g_pm1p0_ae_value   = g_data.PM_AE_UG_1_0;
                g_pm2p5_ae_value   = g_data.PM_AE_UG_2_5;
                g_pm10p0_ae_value  = g_data.PM_AE_UG_10_0;

                g_pms_ae_readings_taken = true;

                // This condition below should NOT be required, but currently I get all
                // 0 values for the PPD results every second time. This check only updates
                // the global values if there is a non-zero result for any of the values:
                if (g_data.PM_TOTALPARTICLES_0_3 + g_data.PM_TOTALPARTICLES_0_5
                    + g_data.PM_TOTALPARTICLES_1_0 + g_data.PM_TOTALPARTICLES_2_5
                    + g_data.PM_TOTALPARTICLES_5_0 + g_data.PM_TOTALPARTICLES_10_0
                    != 0)
                {
                    g_pm0p3_ppd_value  = g_data.PM_TOTALPARTICLES_0_3;
                    g_pm0p5_ppd_value  = g_data.PM_TOTALPARTICLES_0_5;
                    g_pm1p0_ppd_value  = g_data.PM_TOTALPARTICLES_1_0;
                    g_pm2p5_ppd_value  = g_data.PM_TOTALPARTICLES_2_5;
                    g_pm5p0_ppd_value  = g_data.PM_TOTALPARTICLES_5_0;
                    g_pm10p0_ppd_value = g_data.PM_TOTALPARTICLES_10_0;
                    g_pms_ppd_readings_taken = true;
                }
                else{
                    ESP_LOGD(TAG, "False Zero value");
                }
                pms.sleep();

                // Calculate AQI values for the various reporting standards
                calculateUkAqi();

                // Report the new values
                //reportToMqtt();
                reportToSerial();
                reportToHome();

                g_pms_state_start = time_now;
                g_pms_state = PMS_STATE_ASLEEP;
                }
            }
        }

        /**
         Report the latest values to the serial console
        */
        void reportToSerial()
        {
            if (true == g_pms_ae_readings_taken)
            {
                /* Report PM1.0 AE value */
                ESP_LOGD(TAG, "PM1: %ld", g_pm1p0_ae_value);
                /* Report PM2.5 AE value */
                ESP_LOGD(TAG, "PM2.5: %ld", g_pm2p5_ae_value);
                /* Report PM10.0 AE value */
                ESP_LOGD(TAG, "PM10: %ld", g_pm10p0_ae_value);
            }

            if (true == g_pms_ppd_readings_taken)
            {
                /* Report PM0.3 PPD value */
                ESP_LOGD(TAG, "PB0.3: %lu", g_pm0p3_ppd_value);
                /* Report PM0.5 PPD value */
                ESP_LOGD(TAG, "PB0.5: %lu", g_pm0p5_ppd_value);
                /* Report PM1.0 PPD value */
                ESP_LOGD(TAG, "PB1: %lu", g_pm1p0_ppd_value);
                /* Report PM2.5 PPD value */
                ESP_LOGD(TAG, "PB2.5: %lu", g_pm2p5_ppd_value);
                /* Report PM5.0 PPD value */
                ESP_LOGD(TAG, "PB5: %lu", g_pm5p0_ppd_value);
                /* Report PM10.0 PPD value */
                ESP_LOGD(TAG, "PB10: %lu", g_pm10p0_ppd_value);
                /* Report UK AQI value */
                ESP_LOGD(TAG, "UKAQI: %ld", g_uk_aqi_value);
            }
        }

        /**
         Report the latest values to the Home Assistant
        */
        void reportToHome()
        {
            if (true == g_pms_ae_readings_taken)
            {
                /* Report PM1.0 AE value */
                pms5003_PM1_sensor->publish_state(g_pm1p0_ae_value);
                /* Report PM2.5 AE value */
                pms5003_PM2_5_sensor->publish_state(g_pm2p5_ae_value);
                /* Report PM10.0 AE value */
                pms5003_PM10_sensor->publish_state(g_pm10p0_ae_value);
            }

            if (true == g_pms_ppd_readings_taken)
            {
                /* Report PM0.3 PPD value */
                pms5003_PB0_3_sensor->publish_state(g_pm0p3_ppd_value);
                /* Report PM0.5 PPD value */
                pms5003_PB0_5_sensor->publish_state(g_pm0p5_ppd_value);
                /* Report PM1.0 PPD value */
                pms5003_PB1_0_sensor->publish_state(g_pm1p0_ppd_value);
                /* Report PM2.5 PPD value */
                pms5003_PB2_5_sensor->publish_state(g_pm2p5_ppd_value);
                /* Report PM5.0 PPD value */
                pms5003_PB5_0_sensor->publish_state(g_pm5p0_ppd_value);
                /* Report PM10.0 PPD value */
                pms5003_PB10_sensor->publish_state(g_pm10p0_ppd_value);
                /* Report UK AQI value */
                pms5003_UKAQI_sensor->publish_state(g_uk_aqi_value);
            }
        }

        /**
         Use the latest sensor readings to calculate the Air Quality
        Index value using the UK reporting method.
        */
        void calculateUkAqi()
        {
            uint8_t pm2p5_aqi = 0;
            if (g_pm2p5_ppd_value <= 11) {
                pm2p5_aqi = 1;
            } else if (g_pm2p5_ppd_value <= 23) {
                pm2p5_aqi = 2;
            } else if (g_pm2p5_ppd_value <= 35) {
                pm2p5_aqi = 3;
            } else if (g_pm2p5_ppd_value <= 41) {
                pm2p5_aqi = 4;
            } else if (g_pm2p5_ppd_value <= 47) {
                pm2p5_aqi = 5;
            } else if (g_pm2p5_ppd_value <= 53) {
                pm2p5_aqi = 6;
            } else if (g_pm2p5_ppd_value <= 58) {
                pm2p5_aqi = 7;
            } else if (g_pm2p5_ppd_value <= 64) {
                pm2p5_aqi = 8;
            } else if (g_pm2p5_ppd_value <= 70) {
                pm2p5_aqi = 9;
            } else {
                pm2p5_aqi = 10;
            }

            uint8_t pm10p0_aqi = 0;
            if (g_pm10p0_ppd_value <= 16) {
                pm10p0_aqi = 1;
            } else if (g_pm10p0_ppd_value <= 33) {
                pm10p0_aqi = 2;
            } else if (g_pm10p0_ppd_value <= 50) {
                pm10p0_aqi = 3;
            } else if (g_pm10p0_ppd_value <= 58) {
                pm10p0_aqi = 4;
            } else if (g_pm10p0_ppd_value <= 66) {
                pm10p0_aqi = 5;
            } else if (g_pm10p0_ppd_value <= 75) {
                pm10p0_aqi = 6;
            } else if (g_pm10p0_ppd_value <= 83) {
                pm10p0_aqi = 7;
            } else if (g_pm10p0_ppd_value <= 91) {
                pm10p0_aqi = 8;
            } else if (g_pm10p0_ppd_value <= 100) {
                pm10p0_aqi = 9;
            } else {
                pm10p0_aqi = 10;
            }

            if (pm10p0_aqi > pm2p5_aqi)
            {
                g_uk_aqi_value = pm10p0_aqi;
            } else {
                g_uk_aqi_value = pm2p5_aqi;
            }
        }
};