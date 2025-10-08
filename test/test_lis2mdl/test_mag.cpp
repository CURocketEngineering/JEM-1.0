#include <unity.h>
#include <Adafruit_LIS3MDL.h>
#include "pins.h"

Adafruit_LIS3MDL mag;

// Function to initialize the LIS3MDL sensor for testing
void setupLIS3MDL() {
    if (!mag.begin_SPI(SENSOR_LIS_CS)) {
        TEST_FAIL_MESSAGE("Failed to initialize LIS3MDL sensor. Check wiring!");
    }

    // Configure the magnetometer settings
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);

    // Verify configuration
    if (mag.getDataRate() != LIS3MDL_DATARATE_155_HZ) {
        TEST_FAIL_MESSAGE("Failed to set data rate!");
    }
    if (mag.getRange() != LIS3MDL_RANGE_4_GAUSS) {
        TEST_FAIL_MESSAGE("Failed to set range!");
    }
}

// Test to validate magnetometer readings
void test_magnetometer_reading() {
    sensors_event_t mag_event;

    // Collect multiple readings to ensure stability
    for (int i = 0; i < 10; i++) {
        mag.getEvent(&mag_event);
        Serial.print("Mag X: ");
        Serial.println(mag_event.magnetic.x);
        Serial.print("Mag Y: ");
        Serial.println(mag_event.magnetic.y);
        Serial.print("Mag Z: ");
        Serial.println(mag_event.magnetic.z);
        delay(100);
    }

    // Ensure none of them equal a perfect zero
    TEST_ASSERT_NOT_EQUAL(0, mag_event.magnetic.x);
    TEST_ASSERT_NOT_EQUAL(0, mag_event.magnetic.y);
    TEST_ASSERT_NOT_EQUAL(0, mag_event.magnetic.z);

    // Ensure the values are within 100 microteslas of zero
    TEST_ASSERT_FLOAT_WITHIN(100, 0, mag_event.magnetic.x);
    TEST_ASSERT_FLOAT_WITHIN(100, 0, mag_event.magnetic.y);
    TEST_ASSERT_FLOAT_WITHIN(100, 0, mag_event.magnetic.z);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // Wait for native USB
    }
    UNITY_BEGIN();

    // Initialize the LIS3MDL
    setupLIS3MDL();

    // Run the test
    RUN_TEST(test_magnetometer_reading);

    UNITY_END();
}

void loop() {
    // Unity test framework doesn't use the loop function
}
