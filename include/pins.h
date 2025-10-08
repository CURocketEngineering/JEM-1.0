#ifndef MARTHA_PINS_H
#define MARTHA_PINS_H

#include <Arduino.h>

#ifdef PCB_MARTHA

#define SENSOR_MISO PA6 
#define SENSOR_MOSI PA7 
#define SENSOR_SCK PA5
#define SENSOR_BARO_CS PB2
#define SENSOR_LSM_CS PB0 
#define SENSOR_LIS_CS PA2
#define FLASH_CS PB1
#define DEBUG_LED PA9

// For the Adafruit SPI Flash
#define EXTERNAL_FLASH_USE_CS PB1
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

#define EXTERNAL_FLASH_USE_CS PB1
#define EXTERNAL_FLASH_USE_SPI SPI

#endif // BB_MARTHA pins 
 

#endif 