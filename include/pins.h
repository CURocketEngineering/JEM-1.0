#ifndef MARTHA_PINS_H
#define MARTHA_PINS_H

#include <Arduino.h>

#ifdef PCB_MARTHA

#define SENSOR_MISO_1 PA6 
#define SENSOR_MOSI_1 PA7 
#define SENSOR_SCK_1 PA5

// Sensors. Barometer, altimeter, and magnetometer
#define SENSOR_BARO_CS PB_14
#define SENSOR_LSM_CS PE3
#define SENSOR_LIS_CS PE4

#define FLASH_CS PB5 // Note: this is W250 now

#define LED_RED PE_12
#define LED_GREEN PE_13
#define LED_BLUE PE_14
#define BUZZER_PIN PA1

// For the Adafruit SPI Flash
#define EXTERNAL_FLASH_USE_CS PB5
#define EXTERNAL_FLASH_USE_SPI SPI

#endif // PCB_MARTHA

#ifdef BB_MARTHA

#define SENSOR_MISO PB4 
#define SENSOR_MOSI PB5
#define SENSOR_SCK PB3
#define SENSOR_BARO_CS PA1
#define SENSOR_LSM_CS PA0
#define SENSOR_LIS_CS PA3
#define FLASH_CS PB1
#define DEBUG_LED PA9
#define ADC_VOLTAGE PA0

#define EXTERNAL_FLASH_USE_CS PB1
#define EXTERNAL_FLASH_USE_SPI SPI

#endif // BB_MARTHA pins 
 
#ifdef PCB_JEM // JEM sensors also

#define SENSOR_MISO_1 PA6 
#define SENSOR_MOSI_1 PA7 
#define SENSOR_SCK_1 PA5

// Second SPI: used for sensors MS5607, BMI, and KX
#define SENSOR_MISO_2 PC_11
#define SENSOR_MOSI_2 PC_12
#define SENSOR_SCK_2 PC_10

// Sensors. MS5607, BMI, and KX are new!
#define SENSOR_BARO_CS PB_14
#define SENSOR_MS5607_CS PB_15
#define SENSOR_BMI_G_CS PE0
#define SENSOR_BMI_A_CS PE1 
#define SENSOR_KX_CS PE2
#define SENSOR_LSM_CS PE3
#define SENSOR_LIS_CS PE4

#define FLASH_CS PB5 // Note: this is W250 now

#define LED_RED PE_12
#define LED_GREEN PE_13
#define LED_BLUE PE_14
#define BUZZER_PIN PA1

// For the Adafruit SPI Flash
#define EXTERNAL_FLASH_USE_CS PB5
#define EXTERNAL_FLASH_USE_SPI SPI

#endif // end JEM