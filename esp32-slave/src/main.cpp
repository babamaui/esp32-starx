#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// TEST: toggle LED via bluetooth
const int LED_PIN = 2;

BluetoothSerial SerialBT;

void setup(void)
{
  // TEST: toggle LED via bluetooth
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  SerialBT.begin("ESP32test");
  Serial.println("The device started, now you can pair it with bluetooth!");
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
  uint8_t recv_data[6];
  if (SerialBT.available())
  {
    SerialBT.readBytes(recv_data, 6);

    if (recv_data[0] != 'T')
    {
      Serial.print("Receive error!");
      return;
    }

    if (recv_data[5] != calculate_checksum(recv_data))
    {
      Serial.print("Decode error!");
      return;
    }
    if (recv_data[1] == 0x00)
    {
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  delay(20);
}