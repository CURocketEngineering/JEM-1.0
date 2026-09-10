#include <Arduino.h>

#ifdef SIM
  #include "simulation/Serial_Sim_LSM6DSOX.h"
  #include "simulation/Serial_Sim_LIS3MDL.h"
  #include "simulation/Serial_Sim_BMP390.h"
  #include "simulation/Serial_Sim.h"
  // need to make sim tests for the new sensors
#else
  #include "Adafruit_LSM6DSOX.h"
  #include "Adafruit_LIS2MDL.h"
  #include <Async_BMP3XX.h>
#endif

// if we are a PCB JEM, otherwise just using martha stuff
#ifdef PCB_JEM
  #include "BMI088.h"
#endif

#include <Adafruit_Sensor.h>
#include "pins.h"
#include "UARTCommandHandler.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"
#include "data_handling/Telemetry.h"
#include "flash_config.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/FastLaunchDetector.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/GroundLevelEstimator.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/ApogeePredictor.h"
#include "state_estimation/States.h"
#include "state_estimation/StateMachine.h" 
#include "state_estimation/OrientationEstimator.h"
#include "PowerManagement.h"

#define SEALEVELPRESSURE_HPA (1013.25)


int last_led_toggle = 0;
int led_toggle_delay = 1000;
float loop_count = 0;
uint32_t start_time_s = 0;

Adafruit_LSM6DSOX sox;
Adafruit_LIS2MDL  mag;
Adafruit_BMP3XX   bmp;
Bmi088Accel       bmi_accel; // BMI088 is effectively two sensors in one.
Bmi088Gyro        bmi_gyro;

BatteryVoltage adcVolt(ADC_VOLTAGE, 134.33333f, 12, 7.0f); // Below 7 volts is considered low battery

Adafruit_SPIFlash flash(&flashTransport);
DataSaverSPI dataSaver(10, &flash); // Save data every 10 ms

SensorDataHandler xAclData(ACCELEROMETER_X, &dataSaver);
SensorDataHandler yAclData(ACCELEROMETER_Y, &dataSaver);
SensorDataHandler zAclData(ACCELEROMETER_Z, &dataSaver);

SensorDataHandler xGyroData(GYROSCOPE_X, &dataSaver);
SensorDataHandler yGyroData(GYROSCOPE_Y, &dataSaver);
SensorDataHandler zGyroData(GYROSCOPE_Z, &dataSaver);

SensorDataHandler voltageData(BATTERY_VOLTAGE, &dataSaver);

SensorDataHandler tempData(TEMPERATURE, &dataSaver);
SensorDataHandler pressureData(PRESSURE, &dataSaver);
SensorDataHandler altitudeData(ALTITUDE, &dataSaver);
DataPoint altDataPoint;

SensorDataHandler xMagData(MAGNETOMETER_X, &dataSaver);
SensorDataHandler yMagData(MAGNETOMETER_Y, &dataSaver);
SensorDataHandler zMagData(MAGNETOMETER_Z, &dataSaver);

// PCB JEM-exclusive sensors
#if defined(PCB_JEM)
SensorDataHandler xAclDataBMI(26, &dataSaver);



#endif

SensorDataHandler superLoopRate(AVERAGE_CYCLE_RATE, &dataSaver);

SensorDataHandler stateChange(STATE_CHANGE, &dataSaver);
SensorDataHandler currentState(CURRENT_STATE, &dataSaver);
SensorDataHandler flightIDSaver(FLIGHT_ID, &dataSaver);
float flightID;

//Orientation Estimation
SensorDataHandler rollData(ROLL, &dataSaver);
SensorDataHandler pitchData(PITCH, &dataSaver);
SensorDataHandler yawData(YAW, &dataSaver);
OrientationEstimator orientationEstimator;

NoiseVariances noiseVariances {0.25f, 1.0f}; // Example variances

VerticalVelocityEstimator verticalVelocityEstimator(noiseVariances);
SensorDataHandler estVerticalVelocity(EST_VERTICAL_VELOCITY, &dataSaver);

LaunchDetector launchDetector(40, 500, 25);
FastLaunchDetector fastLaunchDetector(30, 500);
ApogeeDetector apogeeDetector(1.0f);
GroundLevelEstimator groundLevelEstimator(0.1f);

ApogeePredictor apogeePredictor(verticalVelocityEstimator);
SensorDataHandler apogeeEstData(EST_APOGEE, &dataSaver);

StateMachine stateMachine(&dataSaver, &launchDetector, &apogeeDetector, &verticalVelocityEstimator, &fastLaunchDetector);

const std::array<SensorDataHandler*, 3> acclDataArray = {&xAclData, &yAclData, &zAclData};
const std::array<SensorDataHandler*, 3> gyroDataArray = {&xGyroData, &yGyroData, &zGyroData};
const std::array<SensorDataHandler*, 3> magDataArray = {&xMagData, &yMagData, &zMagData};
const std::array<SensorDataHandler*, 3> orientationDataArray = {&rollData, &pitchData, &yawData};

SendableSensorData aclDataSSD(acclDataArray, 102, 5);
SendableSensorData gyroDataSSD(gyroDataArray, 105, 5);
SendableSensorData altitudeDataSSD(&altitudeData, 5);
SendableSensorData apogeeEstDataSSD(&apogeeEstData, 2);
SendableSensorData tempDataSSD(&tempData, 1);
SendableSensorData pressureDataSSD(&pressureData, 1);
SendableSensorData magDataSSD(magDataArray, 111, 1);
SendableSensorData superLoopRateSSD(&superLoopRate, 1);
SendableSensorData currentStateSSD(&currentState, 1);
SendableSensorData flightIDSaverSSD(&flightIDSaver, 1);
SendableSensorData estVerticalVelocitySSD(&estVerticalVelocity, 1);
SendableSensorData voltageDataSSD(&voltageData, 1);
SendableSensorData orientationDataSSD(orientationDataArray, 120, 10);

const std::array <SendableSensorData*, 13> ssds = {
  &aclDataSSD,
  &gyroDataSSD,
  &altitudeDataSSD,
  &apogeeEstDataSSD,
  &tempDataSSD,
  &pressureDataSSD,
  &magDataSSD,
  &superLoopRateSSD,
  &currentStateSSD,
  &flightIDSaverSSD,
  &estVerticalVelocitySSD,
  &voltageDataSSD,
  &orientationDataSSD
};

CommandLine cmdLine(&Serial);

float longestLoopTime_ms = 0;

// Stream 
#ifdef USB_RADIO  // Redirects Radio output to USB Serial instead of hardware UART, for direct ground station testing without needing the radio
Telemetry telemetry(ssds, Serial, &cmdLine);
#else
HardwareSerial SUART1(PB7, PB6);
Telemetry telemetry(ssds, SUART1, &cmdLine);
#endif

// #include "commands.h"

void setup() {


  pinMode(DEBUG_LED, OUTPUT); // LED 

  #ifndef USB_RADIO
  SUART1.begin(57600);
  #endif
 

  Serial.begin(115200);
  // while (!Serial) delay(10); // Wait for Serial Monitor (Comment out if not using)



  Serial.println("Setting up accelerometer and gyroscope...");
  while (!sox.begin_SPI(SENSOR_LSM_CS)){
    Serial.println("Could not find LSM6DSOX. Check wiring.");
    delay(10);
  }


  Serial.println("Setting ACL and Gyro ranges and data rates...");
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS );

  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // If the range is not set correctly, then print a message
  if (sox.getAccelRange() != LSM6DS_ACCEL_RANGE_16_G) {
    Serial.println("Failed to set ACL range");
  }
  if (sox.getGyroRange() != LSM6DS_GYRO_RANGE_2000_DPS) {
    Serial.println("Failed to set Gyro range");
  }
  if (sox.getAccelDataRate() != LSM6DS_RATE_104_HZ) {
    Serial.println("Failed to set ACL data rate");
  }
  if (sox.getGyroDataRate() != LSM6DS_RATE_104_HZ) {
    Serial.println("Failed to set Gyro data rate");
  }

  // Setup for the magnetometer
  Serial.println("Setting up magnetometer...");
  while (!mag.begin_SPI(SENSOR_LIS_CS)) {
    Serial.println("Could not find sensor. Check wiring.");
    delay(10);
  }
  mag.setDataRate(LIS2MDL_RATE_100_HZ);

  if (mag.getDataRate() != LIS2MDL_RATE_100_HZ) {
    Serial.println("Failed to set Mag data rate");
  }

  while (! bmp.begin_SPI(SENSOR_BARO_CS)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    delay(10);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

  bmp.setConversionDelay(7); // Give the BMP 8ms to get the next set of data ready.
  bmp.startConversion(); // Start the first conversion (won't be able to collect for 8ms, so get it on the next loop)

  Serial.println("Setting up data saver...");

  // Initalize data saver
  if (!dataSaver.begin()) {
    Serial.println("Failed to initialize data saver");
  }

  Serial.println("Setup complete!");

  // cmdLine.addCommand("test", "t", testCommand);  
  // cmdLine.addCommand("ping", "p", ping);    
  // cmdLine.addCommand("clear_plm", "cplm", clearPostLaunchMode);
  // cmdLine.addCommand("restart", "r", restart);
  // cmdLine.addCommand("exit", "x", exitCommandMode);
  // cmdLine.addCommand("status", "s", printStatus);
  // cmdLine.addCommand("dump", "d", dumpFlash);

  #if !defined(USB_RADIO) && !defined(SIM) // Don't start cmd line if using USB radio or SIM serial input
  cmdLine.begin();
  #endif


  // Set save speeds
  tempData.restrictSaveSpeed(1000);
  pressureData.restrictSaveSpeed(1000);
  xMagData.restrictSaveSpeed(1000);
  yMagData.restrictSaveSpeed(1000);
  zMagData.restrictSaveSpeed(1000);
  superLoopRate.restrictSaveSpeed(1000);
  altitudeData.restrictSaveSpeed(10); // Save altitude every 10 ms (100hz)
  flightIDSaver.restrictSaveSpeed(10000);
  apogeeEstData.restrictSaveSpeed(10);
  currentState.restrictSaveSpeed(2000);
  voltageData.restrictSaveSpeed(2000);
  rollData.restrictSaveSpeed(100);
  pitchData.restrictSaveSpeed(100);
  yawData.restrictSaveSpeed(100);


  // Loop start time
  start_time_s = millis() / 1000;

  // Seed the random number generator
  randomSeed(analogRead(0));

  // Set the flight ID
  flightID = random(100000, 999999);

  // Simulation stuff

  #ifdef SIM
  while (!Serial) delay(10);
  SerialSim::getInstance().begin(&Serial, &stateMachine); 
  dataSaver.clearPostLaunchMode(); // Clear plm for sim
  telemetry.setCommandLine(nullptr); // SIM uses serial input for sensor simulation.
  #endif

}

void loop() {

  loop_count += 1;

  uint32_t current_time = millis();
  if (current_time - last_led_toggle > led_toggle_delay) {
    last_led_toggle = millis();
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }

  // Explicitly save a timestamp to ensure that all data from this loop is associated with the same timestamp and distinct from the previous loop
  dataSaver.saveTimestamp(current_time);

  flightIDSaver.addData(DataPoint(current_time, flightID));

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag_event; 

  // Cannot use cmdLine in SIM mode b/c they use the same serial port.
  // In USB_RADIO mode, only consume command-line bytes once telemetry has
  // switched into command mode on the radio stream.
  #if defined(SIM)
  SerialSim::getInstance().update();
  #elif defined(USB_RADIO)
  if (telemetry.isInCommandMode()) {
    cmdLine.readInput();
  }
  #else
  cmdLine.readInput();
  #endif

  sox.getEvent(&accel, &gyro, &temp);

  DataPoint xAclDataPoint(current_time, accel.acceleration.x);
  DataPoint yAclDataPoint(current_time, accel.acceleration.y);
  DataPoint zAclDataPoint(current_time, accel.acceleration.z);

  AccelerationTriplet aclTriplet = {xAclDataPoint, yAclDataPoint, zAclDataPoint};

  xAclData.addData(xAclDataPoint);
  yAclData.addData(yAclDataPoint);
  zAclData.addData(zAclDataPoint);

  mag.getEvent(&mag_event);

  DataPoint xMagDataPoint(current_time, mag_event.magnetic.x);
  DataPoint yMagDataPoint(current_time, mag_event.magnetic.y);
  DataPoint zMagDataPoint(current_time, mag_event.magnetic.z);
  
  xMagData.addData(xMagDataPoint);
  yMagData.addData(yMagDataPoint);
  zMagData.addData(zMagDataPoint);

  MagTriplet magTriplet = {xMagDataPoint, yMagDataPoint, zMagDataPoint};

  // Check periodically if a new reading is available
  if (bmp.updateConversion()) {
   
    float pres = bmp.getPressure();
    #ifdef SIM
      float alt = bmp.getAlt();
    #else
      // Simulation data might not store pressure in the same units, while meters is standard for alt
      float alt = 44330.0 * (1.0 - pow(pres / 100.0f / SEALEVELPRESSURE_HPA, 0.1903));
    #endif

    // Immediatly convert from ASL to AGL
    alt = groundLevelEstimator.update(alt);

    float temp = bmp.getTemperature();

    
    tempData.addData(DataPoint(current_time, temp));
    pressureData.addData(DataPoint(current_time, pres));
    altDataPoint.data = alt;
    altDataPoint.timestamp_ms = current_time;
    altitudeData.addData(altDataPoint);
    
    // Immediately start the next conversion
    bmp.startConversion();
  }

  // Will update the launch predictor and apogee detector
  // Will log updates to the data saver
  // Will put the data saver in post-launch mode if the launch predictor detects a launch
  // Serial.println("State machine update with alt of " + String(altDataPoint.data));
  stateMachine.update(
    aclTriplet,
    altDataPoint
  );

  estVerticalVelocity.addData(DataPoint(current_time, verticalVelocityEstimator.getEstimatedVelocity()));

  if (stateMachine.getState() >= STATE_ASCENT) {
    led_toggle_delay = 50;
  } else if (stateMachine.getState() == STATE_SOFT_ASCENT) {
    led_toggle_delay = 200;
  } else if (stateMachine.getState() <= STATE_ARMED){
    led_toggle_delay = 1000;
  }

  if (dataSaver.quickGetPostLaunchMode()){
    led_toggle_delay = 100; // Fast blink in post-launch mode and needs clear_plm before relaunch
  }

  // If post-launch, then start saving estimated apogee data
  if (stateMachine.getState() >= STATE_ASCENT) {
    apogeePredictor.analyticUpdate();
    groundLevelEstimator.launchDetected();
    apogeeEstData.addData(DataPoint(current_time, apogeePredictor.getPredictedApogeeAltitude_m()));
  }
  
  DataPoint xgyroDataPoint(current_time, gyro.gyro.x);
  DataPoint ygyroDataPoint(current_time, gyro.gyro.y);
  DataPoint zgyroDataPoint(current_time, gyro.gyro.z);
  
  xGyroData.addData(xgyroDataPoint);
  yGyroData.addData(ygyroDataPoint);
  zGyroData.addData(zgyroDataPoint);

  GyroTriplet gyroTriplet = {xgyroDataPoint, ygyroDataPoint, zgyroDataPoint};

  // Read and save battery voltage
  float voltage = adcVolt.readVoltage();
  voltageData.addData(DataPoint(current_time, voltage));

  // Update orientation estimator and save roll, pitch, yaw
  if (stateMachine.getState() >= STATE_ASCENT) {
    orientationEstimator.launchDetected();
  }
  orientationEstimator.update(aclTriplet, gyroTriplet, magTriplet, current_time);
  rollData.addData(DataPoint(current_time, orientationEstimator.getRoll()));
  pitchData.addData(DataPoint(current_time, orientationEstimator.getPitch()));
  yawData.addData(DataPoint(current_time, orientationEstimator.getYaw()));

  superLoopRate.addData(DataPoint(current_time, loop_count / (millis() / 1000 - start_time_s)));
  currentState.addData(DataPoint(current_time, stateMachine.getState()));


  // Won't consume bytes if in command mode
  telemetry.tick(current_time);

  // Throttle to 100 Hz
  int loop_time_ms = millis() - current_time;  // current_time was captured at the start of the loop
  
  // Wait 1000 loops before tracking longest loop time, to allow for any initial setup or variability to stabilize
  if (loop_time_ms > longestLoopTime_ms && loop_count > 1000) {
    longestLoopTime_ms = loop_time_ms;
  }
  if (loop_time_ms < 10) {
    delay(10 - loop_time_ms);
  }
}


