#include <unity.h>
#include <Adafruit_LSM6DSOX.h>
#include "pins.h"

#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

Adafruit_LSM6DSOX sox;

// Function to initialize the LSM6DSOX sensor for testing
void setupLSM6DSOX() {
    if (!sox.begin_SPI(SENSOR_LSM_CS)) {
        TEST_FAIL_MESSAGE("Failed to initialize LSM6DSOX sensor. Check wiring!");
    }

    sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

    // Verify configuration
    if (sox.getAccelRange() != LSM6DS_ACCEL_RANGE_16_G) {
        TEST_FAIL_MESSAGE("Failed to set accelerometer range!");
    }
    if (sox.getGyroRange() != LSM6DS_GYRO_RANGE_2000_DPS) {
        TEST_FAIL_MESSAGE("Failed to set gyroscope range!");
    }

    // Initialize the flash memory to check for interference
    if (!flash.begin()) {
        TEST_FAIL_MESSAGE("Failed to initialize flash memory!");
    }
}

// Test to validate accelerometer readings
void test_accelerometer_reading() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // Collect multiple readings to ensure stability
    for (int i = 0; i < 10; i++) {
        sox.getEvent(&accel, &gyro, &temp);
        Serial.print("Accel X: ");
        Serial.println(accel.acceleration.x);
        Serial.print("Accel Y: ");
        Serial.println(accel.acceleration.y);
        Serial.print("Accel Z: ");
        Serial.println(accel.acceleration.z);
        delay(10);
    }

    // Check if the accelerometer readings are within 9.8 m/s^2 for a stationary sensor
    TEST_ASSERT_FLOAT_WITHIN(9.8, 0.0, accel.acceleration.x);
    TEST_ASSERT_FLOAT_WITHIN(9.8, 0.0, accel.acceleration.y);
    TEST_ASSERT_FLOAT_WITHIN(9.8, 9.8, accel.acceleration.z); // Should be near 1g due to gravity
}

// Test to validate gyroscope readings
void test_gyroscope_reading() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // Collect multiple readings to ensure stability
    for (int i = 0; i < 10; i++) {
        sox.getEvent(&accel, &gyro, &temp);
        Serial.print("Gyro X: ");
        Serial.println(gyro.gyro.x);
        Serial.print("Gyro Y: ");
        Serial.println(gyro.gyro.y);
        Serial.print("Gyro Z: ");
        Serial.println(gyro.gyro.z);
        delay(100);
    }

    // Check if the gyroscope readings are near zero for a stationary sensor
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, gyro.gyro.x);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, gyro.gyro.y);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, gyro.gyro.z);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // Wait for native USB
    }
    UNITY_BEGIN();

    // Initialize the LSM6DSOX
    setupLSM6DSOX();

    // Run the tests
    RUN_TEST(test_accelerometer_reading);
    RUN_TEST(test_gyroscope_reading);

    UNITY_END();
}

void loop() {
    // Unity test framework doesn't use the loop function
}
