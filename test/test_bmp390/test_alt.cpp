#include <unity.h>
#include <Async_BMP3XX.h>
#include "pins.h"

Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA (1013.25)

// Function to initialize the altimeter for testing
void setupAltimeter() {
    if (!bmp.begin_SPI(SENSOR_BARO_CS)) {
        TEST_FAIL_MESSAGE("Failed to initialize BMP3 sensor. Check wiring!");
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

// Test to check if the altimeter reads around 200 meters
void test_altimeter_reading() {
    float altitude = 0.0;

    // Must take a few readings to get a stable value (the first value is usually incorrect)
    for (int i = 0; i < 10; i++){
        // Perform a sensor reading
        if (!bmp.performReading()) {
            TEST_FAIL_MESSAGE("Failed to perform altimeter reading!");
        }

        altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        Serial.print("Altitude: ");
        Serial.println(altitude);
    }

    // Check if the altitude is within the expected range (±10 meters for margin)
    TEST_ASSERT_FLOAT_WITHIN(100.0, 200.0, altitude);
}

void test_async_and_sync_give_same_values(){
    float altitude_sync = 0.0;
    float altitude_async = 0.0;

    bmp.setConversionDelay(10); // 10 ms == 100 Hz
    bmp.startConversion(); // Start the first conversion

    // Collect the first conversion value
    delay(15);
    TEST_ASSERT_TRUE(bmp.updateConversion());
    altitude_async = 44330.0 * (1.0 - pow(bmp.getPressure() / 100.0f / SEALEVELPRESSURE_HPA, 0.1903));
    altitude_sync = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    TEST_ASSERT_FLOAT_WITHIN(0.00001, altitude_sync, altitude_async);

}

void test_async_integrity(){
    bmp.setConversionDelay(10); // 10 ms == 100 Hz
    bmp.startConversion(); // Start the first conversion

    // Collect the first conversion value
    delay(15);
    TEST_ASSERT_TRUE(bmp.updateConversion());
    float altitude_async = 44330.0 * (1.0 - pow(bmp.getPressure() / 100.0f / SEALEVELPRESSURE_HPA, 0.1903));

    // Collect the second conversion value
    delay(15);
    TEST_ASSERT_TRUE(bmp.updateConversion());
    float altitude_async2 = 44330.0 * (1.0 - pow(bmp.getPressure() / 100.0f / SEALEVELPRESSURE_HPA, 0.1903));

    // Check that the two values are not the same but are within a reasonable range
    TEST_ASSERT_NOT_EQUAL(altitude_async, altitude_async2);
    TEST_ASSERT_FLOAT_WITHIN(10.0, altitude_async, altitude_async2);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // wait for native usb
    }
    UNITY_BEGIN();

    // Initialize the altimeter
    setupAltimeter();

    // Run the test
    RUN_TEST(test_altimeter_reading);
    RUN_TEST(test_async_and_sync_give_same_values);
    RUN_TEST(test_async_integrity);

    UNITY_END();
}

void loop() {
    // Unity test framework doesn't use the loop function
}
