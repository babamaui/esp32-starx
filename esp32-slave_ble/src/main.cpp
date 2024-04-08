// Helpful links:
//  https://raw.githubusercontent.com/RuiSantosdotme/Random-Nerd-Tutorials/master/Projects/ESP32/BLE/ESP32_BLE_Client.ino

// Import needed header files
#include "Arduino.h"
#include "BLEDevice.h"

// Define characteristics of BLE Server
#define bleServerName "ESP32_TEST"
static BLEUUID SERVICE_UUID((uint16_t)0x0541);
static BLEUUID accelerationCharacteristicUUID((uint16_t)0x2713);

// Connectivity variables
static bool doConnect = false;
static bool connected = false;

// Server details
static BLEAddress *pServerAddress;
static BLERemoteCharacteristic *accelerationCharacteristic;

// Activate notifications
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

// Variables to store LED status
char *accelerationChar;

// Checkers for whether new readings are available
bool newData = false;

// Connect to the BLE Server that has matching name, service, and characteristics
bool connectToServer(BLEAddress pAddress)
{
  BLEClient *pClient = BLEDevice::createClient();

  // Connect to the BLE Server.
  pClient->connect(pAddress);
  Serial.println("Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(SERVICE_UUID.toString().c_str());
    return (false);
  }

  // Obtain a reference to the characteristics
  accelerationCharacteristic = pRemoteService->getCharacteristic(accelerationCharacteristicUUID);

  if (accelerationCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println("Found characteristics");

  // Assign callback functions for the Characteristics
  accelerationCharacteristic->registerForNotify(accelerationCallback);
  return true;
}

// Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // If advertised name matches
    if (advertisedDevice.getName() == bleServerName)
    {
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      Serial.println("Device found. Connecting!");
    }
  }
};

// When server sends notify of red characteristic
static void accelerationCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                 uint8_t *pData, size_t length, bool isNotify)
{
  // store temperature value
  accelerationChar = (char *)pData;
  newData = true;
}

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("Starting BLE client...");

  // Initialize BLE device
  BLEDevice::init("");

  // Start scanning
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop()
{
  if (doConnect == true)
  {
    if (connectToServer(*pServerAddress))
    {
      Serial.println("Connected to server");
      connected = true;
      doConnect = false;
    }
    else
    {
      Serial.println("Failed to connect to server");
    }
  }

  // If newReadings are available
  if (newData)
  {
    newData = false;
    Serial.println(accelerationChar);
  }

  // 500ms delay between loops
  delay(500);
}