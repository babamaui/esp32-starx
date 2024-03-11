// Master/Sender Code

#include <Arduino.h>
#include "BluetoothSerial.h"

// TEST: toggle LED via bluetooth
const int LED_PIN = 2;

BluetoothSerial SerialBT;

// MAC address of the receiver
String MACadr = "48:E7:29:A0:05:96";
uint8_t address[6] = {0x48, 0xE7, 0x29, 0xA0, 0x05, 0x96};
bool connected = false;
bool led_state = false;

void setup(void)
{
  // TEST: toggle LED via bluetooth
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  SerialBT.begin("ESP32test", true);
  Serial.println("The device started in master mode, make sure remote BT device is on!");

  connected = SerialBT.connect(address);

  if (connected)
  {
    Serial.println("Connected Succesfully!");
  }
  else
  {
    while (!SerialBT.connected(10000))
    {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart.");
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect())
  {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
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
  // TEST: toggle LED via bluetooth
  if (led_state = false)
  {
    led_state = true;
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    led_state = false;
    digitalWrite(LED_PIN, LOW);
  }

  uint8_t send_data[1];
  send_data[0] = 'T';
  send_data[1] = led_state ? 0x01 : 0x00;
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  send_data[4] = 0x00;
  send_data[5] = calculate_checksum(send_data);
  Serial.write(send_data, 6);
  delay(20);
}