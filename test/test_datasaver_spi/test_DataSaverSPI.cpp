#include <unity.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataNames.h"

#include "flash_config.h"

#define TEST_ADDRESS_1 0x002000 // Example address for testing
#define TEST_DATA_1 0xA5         // Example test data pattern

#define TEST_ADDRESS_2 0x003000 // Example address for testing
#define TEST_DATA_2 0xB2         // Example test data pattern

#define TEST_BYTE_SIZE 3296    // Size for time-based test case
#define TEST_TIME_LIMIT 1000   // Time limit in milliseconds

Adafruit_SPIFlash flash(&flashTransport);
DataSaverSPI dataSaver(10, &flash); // Save data every 10 ms

void test_flash_init() {
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(true, flash.begin());
    TEST_ASSERT_GREATER_THAN(0, flash.size());
    Serial.printf("test_flash_init execution time: %lu ms\n", millis() - start_time);
}

void test_flash_read_write(){
    uint8_t write_data = TEST_DATA_1;
    uint8_t read_data = 0;
    TEST_ASSERT_EQUAL(true, flash.eraseSector(TEST_ADDRESS_1 / SFLASH_SECTOR_SIZE));

    // Write data to the flash
    TEST_ASSERT_EQUAL(true, flash.writeBuffer(TEST_ADDRESS_1, &write_data, 1));

    // Read back the data
    TEST_ASSERT_EQUAL(true, flash.readBuffer(TEST_ADDRESS_1, &read_data, 1));

    // Verify the written and read data are the same
    TEST_ASSERT_EQUAL(write_data, read_data);

    // Erase sector and ensure it's erased
    TEST_ASSERT_EQUAL(true, flash.eraseSector(TEST_ADDRESS_1 / SFLASH_SECTOR_SIZE));
    TEST_ASSERT_EQUAL(true, flash.readBuffer(TEST_ADDRESS_1, &read_data, 1));
    TEST_ASSERT_EQUAL(0xff, read_data);

    // Use a different address and data pattern
    write_data = TEST_DATA_2;
    TEST_ASSERT_EQUAL(true, flash.eraseSector(TEST_ADDRESS_2 / SFLASH_SECTOR_SIZE));
    TEST_ASSERT_EQUAL(true, flash.writeBuffer(TEST_ADDRESS_2, &write_data, 1));
    TEST_ASSERT_EQUAL(true, flash.readBuffer(TEST_ADDRESS_2, &read_data, 1));
    TEST_ASSERT_EQUAL(write_data, read_data);
    TEST_ASSERT_EQUAL(true, flash.eraseSector(TEST_ADDRESS_2 / SFLASH_SECTOR_SIZE));
    
    // Make sure the sector is erased
    TEST_ASSERT_EQUAL(true, flash.readBuffer(TEST_ADDRESS_2, &read_data, 1));
    TEST_ASSERT_EQUAL(0xff, read_data);
}

void test_data_saver_begin() {
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(true, dataSaver.begin());
    Serial.printf("test_data_saver_begin execution time: %lu ms\n", millis() - start_time);
}

void test_save_data_point() {
    dataSaver.clearPostLaunchMode();
    DataPoint dp = {100, 50}; // Example DataPoint: timestamp = 100ms, value = 50
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1)); // 1 as name/ID
    TEST_ASSERT_EQUAL(100, dataSaver.getLastTimestamp());
    TEST_ASSERT_EQUAL(50, dataSaver.getLastDataPoint().data);
    Serial.printf("test_save_data_point execution time: %lu ms\n", millis() - start_time);
}

void test_time_based_write() {
    uint8_t testData[TEST_BYTE_SIZE];
    memset(testData, TEST_DATA_1, TEST_BYTE_SIZE); // Fill buffer with test data
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(TEST_BYTE_SIZE, flash.writeBuffer(TEST_ADDRESS_1, testData, TEST_BYTE_SIZE));
    unsigned long duration = millis() - start_time;
    Serial.printf("test_time_based_write execution time: %lu ms\n", duration);
    TEST_ASSERT_LESS_THAN(TEST_TIME_LIMIT, duration);
}

void test_flush_buffer_count(){
    // Writing enough data to trigger a certain number of flushes
    dataSaver.clearInternalState();

    // Write 1000 bytes to the buffer by sending 1000 / 5 = 200 DataPoints all with the same timestamp
    DataPoint dp = {100, 50};
    for (int i = 0; i < 200; i++) {
        TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1));
    }

    int expected_flush_count = 1000 / dataSaver.BUFFER_SIZE;
    TEST_ASSERT_EQUAL(expected_flush_count, dataSaver.getBufferFlushes());
}

void test_post_launch_mode() {
    unsigned long start_time = millis();
    dataSaver.clearPostLaunchMode();

    TEST_ASSERT_EQUAL(false, dataSaver.isPostLaunchMode());
    TEST_ASSERT_EQUAL(false, dataSaver.quickGetPostLaunchMode());
    
    dataSaver.launchDetected(500); // Launch detected at 500ms

    TEST_ASSERT_EQUAL(true, dataSaver.isPostLaunchMode());
    TEST_ASSERT_EQUAL(true, dataSaver.quickGetPostLaunchMode());
    DataPoint dp = {600, 80};
    TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1));
    // Check post-launch flag persistence
    dataSaver.clearPostLaunchMode();
    TEST_ASSERT_EQUAL(false, dataSaver.isPostLaunchMode());
    Serial.printf("test_post_launch_mode execution time: %lu ms\n", millis() - start_time);
}

void test_erase_all_data() {
    DataPoint dp = {100, 50};
    dataSaver.saveDataPoint(dp, 1);

    // Also write to flash to ensure it's erased
    uint8_t writeData = TEST_DATA_1;
    flash.writeBuffer(TEST_ADDRESS_1, &writeData, 1);

    unsigned long start_time = millis();
    dataSaver.eraseAllData();
    TEST_ASSERT_EQUAL(0, dataSaver.getLastTimestamp());

    // Read from flash to ensure it's erased
    uint8_t readData;
    flash.readBuffer(TEST_ADDRESS_1, &readData, 1);
    TEST_ASSERT_NOT_EQUAL(TEST_DATA_1, readData);

    // Ensure next write starts fresh
    dp.timestamp_ms = 200;
    TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1));
    TEST_ASSERT_EQUAL(200, dataSaver.getLastTimestamp());
    Serial.printf("test_erase_all_data execution time: %lu ms\n", millis() - start_time);
}

void test_post_launch_data_preservation() {
    // Ideally a launch detection should protect that previous 1 minute of data 
    // and prevent all newer data from being overwritten.

    // Reset data saver
    dataSaver.eraseAllData();

    int minute_5_ms = 5 * 60 * 1000;
    
    // Write 5 minutes of data
    Serial.println("Writing 5 minutes of data (pre-launch)");
    for (uint32_t i = 0; i <= minute_5_ms / 10; i++) {
        DataPoint dp = {i * 10, 12};  // pre-launch data has a value of 12
        dataSaver.saveDataPoint(dp, 1);
    }

    // Trigger a launch at the 5 minute mark
    dataSaver.launchDetected(minute_5_ms); // Launch detected at 5 minutes

    TEST_ASSERT_EQUAL(true, dataSaver.isPostLaunchMode());
    TEST_ASSERT_EQUAL(minute_5_ms, dataSaver.getLastTimestamp());

    // Print out the launch write address and next write address
    Serial.print("Launch write address: ");
    Serial.println(dataSaver.getLaunchWriteAddress());
    Serial.print("Next write address: ");
    Serial.println(dataSaver.getNextWriteAddress());
    
    // Ensure the launch write address is less than the getNextWriteAddress
    TEST_ASSERT_LESS_THAN(dataSaver.getNextWriteAddress(), dataSaver.getLaunchWriteAddress());

    

    // Write 5 more minutes of data
    Serial.println("Writing 5 minutes of data (post-launch)");
    for (uint32_t i = 0; i < minute_5_ms / 10; i++) {
        DataPoint dp = {i * 10 + minute_5_ms, 25}; // 5 minutes of post-launch data has a value of 25
        dataSaver.saveDataPoint(dp, 1);
    }

    // At this point if we write flash size number of bytes, we should loop back to the beginning
    // and overwrite the pre-launch data. But the post-launch data should be preserved.

    // saveDataPoint returns a 1 if the data was not saved due to post-launch data
    // if we keep saving we should get a 1 eventually

    // Write until we get a 1
    Serial.println("Writing until we loop back to the beginning");
    for (uint32_t i = 0; i < flash.size(); i++) {
        DataPoint dp = {i * 20 + 2 * minute_5_ms, 50};
        if (dataSaver.saveDataPoint(dp, 1) == 1) {
            break;
        }

        // Print progress
        if (i % 10000 == 0) {
            Serial.print("Progress (till wrap around): ");
            Serial.print(dataSaver.getNextWriteAddress() * 100 / flash.size());
            Serial.print("%");

            // print current write address
            Serial.print(" Current write address: ");
            Serial.println(dataSaver.getNextWriteAddress());
        }
    }

    // If we reached the end without getting a 1, something is wrong
    TEST_ASSERT_EQUAL(1, dataSaver.saveDataPoint({0, 0}, 1));

    Serial.print("Hit a post-launch data preservation limit at address: ");
    Serial.println(dataSaver.getNextWriteAddress());

    // Test that the nextwriteaddress is less than the launchwriteaddress
    TEST_ASSERT_LESS_THAN(dataSaver.getLaunchWriteAddress() + 100, dataSaver.getNextWriteAddress());
}

void test_data_point_byte_size_buffer_handling() {
    // Write a few data points and ensure that the address changed by the correct amount
    dataSaver.clearInternalState();
    DataPoint dp = {100, 50};

    int expected_in_buffer = 0;

    uint32_t startAddress = dataSaver.getNextWriteAddress();
    dataSaver.saveDataPoint(dp, 1); // Should write 5 bytes (timestamp) + 5 bytes (data record)
    expected_in_buffer += 10;

    // Because we haven't filled the buffer yet, the address should still be the same
    TEST_ASSERT_EQUAL(0, dataSaver.getNextWriteAddress() - startAddress);

    // However, the buffer index should now be 10
    TEST_ASSERT_EQUAL(10, dataSaver.getBufferIndex());

    int max_in_buffer = dataSaver.BUFFER_SIZE / 5 * 5; // Nearest multiple of 5 less than the buffer size

    // To cause the buffer to flush, let's write until we almost fill it up
    while (dataSaver.getBufferIndex() < max_in_buffer) {
        dataSaver.saveDataPoint(dp, 1); // Adds just 5 bytes b/c the timestamp is the same
        expected_in_buffer += 5;

        // Make sure the buffer index is correct
        TEST_ASSERT_EQUAL(expected_in_buffer, dataSaver.getBufferIndex());

        // Make sure the write address hasn't changed yet
        TEST_ASSERT_EQUAL(0, dataSaver.getNextWriteAddress() - startAddress);

        // The number of buffer flushes should still be 0
        TEST_ASSERT_EQUAL(0, dataSaver.getBufferFlushes());
    }

    // Check that the nextWriteAddress hasn't changed yet
    TEST_ASSERT_EQUAL(0, dataSaver.getNextWriteAddress() - startAddress);

    // Check the buffer index
    TEST_ASSERT_EQUAL(expected_in_buffer, dataSaver.getBufferIndex());

    // Write one more data point to flush the buffer
    dataSaver.saveDataPoint(dp, 1); // Adds 5 more bytes

    expected_in_buffer += 0;  // The name fits 

    // A flush should have just happend the entire buffer size is written to flash
    TEST_ASSERT_EQUAL(1, dataSaver.getBufferFlushes());

    // All of this data point is forced into the next buffer
    TEST_ASSERT_EQUAL(5, dataSaver.getBufferIndex());

    // Check that the nextWriteAddress has changed by the correct amount
    TEST_ASSERT_EQUAL(dataSaver.BUFFER_SIZE, dataSaver.getNextWriteAddress() - startAddress);
}

void test_expected_data_on_chip_2_buf_flushes(){
    // Write a few buffers worth of data and read the data to ensure it's all correct

    // Clear the target sector
    flash.eraseSector(DATA_START_ADDRESS / SFLASH_SECTOR_SIZE);

    // First byte should be all 0xff
    uint8_t readData;
    TEST_ASSERT_EQUAL(true, flash.readBuffer(DATA_START_ADDRESS, &readData, 1));
    TEST_ASSERT_EQUAL(0xff, readData);

    // Reset data saver
    dataSaver.clearInternalState();

    // 51 data points fit into a buffer (5 bytes per data point = 255 bytes)
    // Write 103 datapoints to fill 2 buffers (and flush the second one), then go through and read them all back
    // never change the timestamp to avoid extra complexity, only the first will trigger a timestamp
    int datapoints_per_buffer = dataSaver.BUFFER_SIZE / 5;
    int needed_datapoints = datapoints_per_buffer * 2; // No +1 b/c first timestamp
    for (int i = 0; i < needed_datapoints; i++) {
        DataPoint dp = {100, static_cast<float>(i) * 10};
        TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, i));
    }

    // Ensure that 2 flushes happended
    TEST_ASSERT_EQUAL(2, dataSaver.getBufferFlushes());

    TEST_ASSERT_EQUAL(256, SFLASH_PAGE_SIZE); // Ensure the page size is correct

    uint8_t name;

    // Read back the data
    for (int page = 0; page < 2; page++) {
        for (int i = 0; i < datapoints_per_buffer; i++) {
            int expVal = page * datapoints_per_buffer + (i - 1); // -1 because the first one is a timestamp
            uint32_t address = DATA_START_ADDRESS + page * SFLASH_PAGE_SIZE + i * 5;
            TEST_ASSERT_EQUAL(1, flash.readBuffer(address, &name, 1));
            
            // Check that the value and name are correct
            if (i == 0 && page == 0){
                // timestamp
                uint32_t timestamp;
                TEST_ASSERT_EQUAL(4, flash.readBuffer(address + 1, reinterpret_cast<uint8_t*>(&timestamp), 4));
                TEST_ASSERT_EQUAL(TIMESTAMP, name);
                TEST_ASSERT_EQUAL(100, timestamp);
            } else {
                float value; 
                TEST_ASSERT_EQUAL(4, flash.readBuffer(address + 1, reinterpret_cast<uint8_t*>(&value), 4));
                // Serial.print("Does ");
                // Serial.print(expVal);
                // Serial.print(" == ");
                // Serial.println(name);
                TEST_ASSERT_EQUAL(expVal, name); // Name
                TEST_ASSERT_EQUAL(expVal * 10, value); // Data
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // wait for native USB
    }

    UNITY_BEGIN();
    RUN_TEST(test_flash_init);
    RUN_TEST(test_flash_read_write);
    RUN_TEST(test_data_saver_begin);
    RUN_TEST(test_post_launch_mode);
    RUN_TEST(test_save_data_point);
    RUN_TEST(test_data_point_byte_size_buffer_handling);
    RUN_TEST(test_flush_buffer_count);
    RUN_TEST(test_time_based_write);
    RUN_TEST(test_expected_data_on_chip_2_buf_flushes);
    // RUN_TEST(test_erase_all_data);
    // RUN_TEST(test_post_launch_data_preservation);
    UNITY_END();
}

void loop() {
    // nothing to do here
}
