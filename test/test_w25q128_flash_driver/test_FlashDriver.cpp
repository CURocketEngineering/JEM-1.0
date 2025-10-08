// #include <Arduino.h>
// #include <unity.h>
// #include "pins.h"            // Your pin definitions
// #include "FlashDriver.h"     // Your Flash driver

// // Create a global instance of the flash driver.
// FlashDriver flashDriver(
//     SENSOR_MOSI,
//     SENSOR_MISO,
//     SENSOR_SCK,
//     FLASH_CS
// );

// //--------------------------------------------------
// // Test function declarations
// //--------------------------------------------------
// void test_flash_init();
// void test_flash_write_and_read();

// //--------------------------------------------------
// // Unity Setup
// //--------------------------------------------------
// void setup() {
//     Serial.begin(115200);
//     while (!Serial) {
//         Serial.println("Waiting for serial connection");
//         delay(100);
//     }
//     Serial.println("Starting test");

//     UNITY_BEGIN();
    
//     // New flash driver tests
//     RUN_TEST(test_flash_init);
//     RUN_TEST(test_flash_write_and_read);

//     UNITY_END();
// }

// void loop() {
//     // Not used in this test scenario
// }

// //--------------------------------------------------
// // New Flash Driver test functions
// //--------------------------------------------------

// /**
//  * @brief Ensure we can initialize the flash chip successfully
//  *        by checking if the read JEDEC ID matches the expected
//  *        manufacturer, device ID, etc.
//  */
// void test_flash_init() 
// {
//     // Initialize flash
//     FlashStatus status = flashDriver.initFlash();
    
//     // Check if initialization was successful
//     TEST_ASSERT_EQUAL_MESSAGE(
//         FLASH_SUCCESS,
//         status,
//         "Failed to initialize flash or invalid JEDEC ID."
//     );
// }

// /**
//  * @brief Erase a sector, write some data to the flash, and verify
//  *        that we can read the exact same data back.
//  */
// void test_flash_write_and_read() 
// {
//     // Address to test writing/reading. 
//     // For simplicity, let's use 0x000000 (start of flash).
//     const uint32_t testAddress = 0x000000;

//     // Example data to write
//     uint8_t writeData[] = { 0xDE, 0xAD, 0xBE, 0xEF };

//     // Step 1: Erase the sector where we intend to write.
//     //         Because we’re writing at address 0x000000,
//     //         the sector is also at 0x000000.
//     flashDriver.eraseSector(testAddress);

//     // Optional: Wait until erase completes
//     // (waitUntilNotBusy() is called internally, but you could 
//     //  also poll if needed.)

//     // Step 2: Write data to the flash
//     FlashStatus writeStatus = flashDriver.writeFlash(
//         testAddress, 
//         writeData, 
//         sizeof(writeData)
//     );
//     TEST_ASSERT_EQUAL_MESSAGE(
//         FLASH_SUCCESS,
//         writeStatus,
//         "Flash write operation failed."
//     );

//     // Step 3: Read data from the flash
//     uint8_t readData[sizeof(writeData)] = {0};
//     FlashStatus readStatus = flashDriver.readFlash(
//         testAddress, 
//         readData, 
//         sizeof(readData)
//     );
//     TEST_ASSERT_EQUAL_MESSAGE(
//         FLASH_SUCCESS,
//         readStatus,
//         "Flash read operation failed."
//     );

//     // Step 4: Verify that the data matches
//     TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(
//         writeData,
//         readData,
//         sizeof(writeData),
//         "Data read does not match data written."
//     );
// }
