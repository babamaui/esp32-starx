#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Possibly helpful: https://randomnerdtutorials.com/esp32-ble-server-environmental-sensing-service/
// Latency: https://forum.arduino.cc/t/arduino-nan-ble-33-how-to-send-data/632690/6
// Latency fix??: https://forum.arduino.cc/t/ble-very-weak-signal/631751/29

// Onboard LED pin for testing
const int LED_PIN = 2;
bool led_state = false;

// BLE server name
#define bleServerName "ESP32_TEST"

// Define UUID
#define SERVICE_UUID (BLEUUID((uint16_t)0x0541)) // Motion Sensor

// Acceleration (ms^2) characteristic and descriptor UUID
BLECharacteristic AccelerationCharacteristic(BLEUUID((uint16_t)0x2713), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor AccelerationDescriptor(BLEUUID((uint16_t)0x2902));

// Server connection status
bool deviceConnected = false;

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Device Connected");
    // Turn on LED when connection is established
    digitalWrite(LED_PIN, HIGH);
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Device Disconnected");
    // Turn off LED when connection is lost
    digitalWrite(LED_PIN, LOW);
  }
};

// Create the MPU6050 sensor object
Adafruit_MPU6050 mpu;

// Initialize MPU6050
void setupMPU6050()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println("MPU6050 Configured.");
}

void setup()
{
  // Set LED pin as output
  pinMode(LED_PIN, OUTPUT);

  // Start serial communication
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *mpuService = pServer->createService(SERVICE_UUID);

  mpuService->addCharacteristic(&AccelerationCharacteristic);
  AccelerationCharacteristic.addDescriptor(&AccelerationDescriptor);

  // Initialize MPU6050
  mpuService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // // Print out the values
  // String output = "";
  // output += "Acceleration X: ";
  // output += static_cast<String>(a.acceleration.x);
  // output += " Y: ";
  // output += static_cast<String>(a.acceleration.y);
  // output += " Z: ";
  // output += static_cast<String>(a.acceleration.z);
  // output += " m/s^2";
  // output += " // ";
  // output += "Rotation X: ";
  // output += static_cast<String>(g.gyro.x);
  // output += " Y: ";
  // output += static_cast<String>(g.gyro.y);
  // output += " Z: ";
  // output += static_cast<String>(g.gyro.z);
  // output += " rad/s";
  // Serial.println(output);

  // Notify the client of the acceleration
  if (deviceConnected)
  {
    float x = a.acceleration.x;
    float y = a.acceleration.y;
    float z = a.acceleration.z;

    // Put the acceleration values into a byte array
    uint8_t accelerationData[12];
    memcpy(accelerationData, &x, sizeof(x));
    // memcpy(accelerationData, &x, sizeof(x));
    // memcpy(accelerationData + 4, &y, sizeof(y));
    // memcpy(accelerationData + 8, &z, sizeof(z));

    // Notify the client of the acceleration values
    AccelerationCharacteristic.setValue(accelerationData, 12);
    AccelerationCharacteristic.notify();
  }

  // Change this when actually using it
  delay(1000);
}