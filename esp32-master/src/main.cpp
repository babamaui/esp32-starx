// Master/Sender Code

// Referenced:
// https://ok1tk.com/two-way-serial-bluetooth-communication-between-two-esp32-boards/

#include <Arduino.h>
#include "BluetoothSerial.h"

// Onboard LED pin for testing
const int LED_PIN = 2;
bool led_state = false;

// Connection status variables
bool connected = false;
int conn_attempts = 0;

// Bluetooth check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth not available or not enabled.
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled.
#endif

// MAC address of the receiver
String MACadd = "48:E7:29:A0:05:96";
uint8_t address[6] = {0x48, 0xE7, 0x29, 0xA0, 0x05, 0x96};

// Initialize SerialBT object
BluetoothSerial SerialBT;

// Bluetooth callback function for connection to the receiver
void BluetoothStatus(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    Serial.println("Connected");
    digitalWrite(LED_PIN, HIGH);
    connected = true;
  }
  else if (event == ESP_SPP_CLOSE_EVT)
  {
    Serial.println("Disconnected");
    digitalWrite(LED_PIN, LOW);
    connected = false;
  }
}

// Connects device to receiver
void BluetoothConnect()
{
  // Attempt connection to receiver specified above
  SerialBT.connect(address);
}

void setup()
{
  // Set LED pin as output
  pinMode(LED_PIN, OUTPUT);

  connected = false;

  Serial.begin(9600);

  // Define the callback function for the Bluetooth connection
  SerialBT.register_callback(BluetoothStatus);
  SerialBT.begin("ESP32test", true);
  Serial.println("Device has started in master mode, now searching for the receiver.");

  // Attempt connection to receiver
  Serial.println("Connecting...");
  BluetoothConnect();
}

uint8_t calculate_checksum(uint8_t *data)
{
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];
  return checksum;
}

void loop()
{
  // If not connected, attempt to connect
  if (!connected)
  {
    conn_attempts++;
    Serial.printf("Attempting connection ");
    Serial.printf("[Attempt %d]\n", conn_attempts);
    SerialBT.end();
    SerialBT.begin("ESP32test", true);
    BluetoothConnect();
    delay(10000);
  }

  // BT: Data send/receive via bluetooth
  // if (Serial.available())
  // {                                // BT: Checks if there are data from the serial monitor available
  //   SerialBT.write(Serial.read()); // BT: Sends the data via bluetooth
  // }
  // if (SerialBT.available())
  // {                                // BT: Checks if there are data from the bluetooth available
  //   Serial.write(SerialBT.read()); // BT: Write the data to the serial monitor
  // }

  // TEST: toggle LED via bluetooth
  // if (led_state = false)
  // {
  //   led_state = true;
  //   digitalWrite(LED_PIN, HIGH);
  // }
  // else
  // {
  //   led_state = false;
  //   digitalWrite(LED_PIN, LOW);
  // }
  // uint8_t send_data[1];
  // send_data[0] = 'T';
  // send_data[1] = led_state ? 0x01 : 0x00;
  // send_data[2] = 0x00;
  // send_data[3] = 0x00;
  // send_data[4] = 0x00;
  // send_data[5] = calculate_checksum(send_data);
  // Serial.write(send_data, 6);
  delay(20);
}