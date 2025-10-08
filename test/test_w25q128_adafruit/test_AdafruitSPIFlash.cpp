// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries
#include <unity.h>

#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

#define TEST_ADDRESS 0x00010000 // Example address for testing
#define TEST_DATA 0xA5         // Example test data pattern

Adafruit_SPIFlash flash(&flashTransport);

void test_flash_init() {
    TEST_ASSERT_EQUAL(true, flash.begin());
}

// Test JEDEC ID
void test_flash_correct_JEDEC_ID() {
    TEST_ASSERT_EQUAL(EXPECTED_JEDEC_ID, flash.getJEDECID());
}

// Test flash size
void test_flash_size_greater_than_zero() {
    TEST_ASSERT_GREATER_THAN(0, flash.size());
}

// Test write and read
void test_flash_write_read() {
    uint8_t write_data = TEST_DATA;
    uint8_t read_data = 0;

    // Erase a sector to ensure clean write
    TEST_ASSERT_EQUAL(true, flash.eraseSector(TEST_ADDRESS));

    // Write data to the flash
    TEST_ASSERT_EQUAL(true, flash.writeBuffer(TEST_ADDRESS, &write_data, 1));

    // Read back the data
    TEST_ASSERT_EQUAL(true, flash.readBuffer(TEST_ADDRESS, &read_data, 1));

    // Verify the written and read data are the same
    TEST_ASSERT_EQUAL(write_data, read_data);
}

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // wait for native usb
    }
    
    UNITY_BEGIN();
    RUN_TEST(test_flash_init);
    RUN_TEST(test_flash_correct_JEDEC_ID);
    RUN_TEST(test_flash_size_greater_than_zero);
    RUN_TEST(test_flash_write_read);

    Serial.println("Adafruit Serial Flash Info example");
    uint32_t jedec_id = flash.getJEDECID();
    Serial.print("JEDEC ID: 0x");
    Serial.println(jedec_id, HEX);
    Serial.print("Flash size (usable): ");
    Serial.print(flash.size() / 1024);
    Serial.println(" KB");

    UNITY_END();
}

void loop() {
  // nothing to do
}