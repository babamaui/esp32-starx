// Slave/Receiver code

// Referenced:
// https://ok1tk.com/two-way-serial-bluetooth-communication-between-two-esp32-boards/

#include <Arduino.h>
#include "BluetoothSerial.h"

// Onboard LED pin for testing
const int LED_PIN = 2;
bool led_state = false;

// Connection status variables
bool connected = false;

// Bluetooth check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth not available or not enabled.
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled.
#endif

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

void setup(void)
{
  // Set LED pin as output
  pinMode(LED_PIN, OUTPUT);

  connected = false;

  Serial.begin(9600);

  // Define the callback function for the Bluetooth connection
  SerialBT.register_callback(BluetoothStatus);
  SerialBT.begin("ESP32test");
  Serial.println("Device started in slave mode, now waiting for a connection.");
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
  // BT: Blinking the bluetooth indication LED if the connection is not established
  if (!connected)
  {
    digitalWrite(LED_PIN, !led_state);
    led_state = !led_state;
    delay(500);
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

  // Serial.println("loop");
  // uint8_t recv_data[6];
  // if (SerialBT.available())
  // {
  //   Serial.println("Connected");
  //   SerialBT.readBytes(recv_data, 6);

  //   if (recv_data[0] != 'T')
  //   {
  //     Serial.print("Receive error!");
  //     return;
  //   }

  //   if (recv_data[5] != calculate_checksum(recv_data))
  //   {
  //     Serial.print("Decode error!");
  //     return;
  //   }
  //   if (recv_data[1] == 0x00)
  //   {
  //     digitalWrite(LED_PIN, LOW);
  //   }
  //   else
  //   {
  //     digitalWrite(LED_PIN, HIGH);
  //   }
}