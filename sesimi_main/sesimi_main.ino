/*
  author: Allen Nikka
  Combination of arduino-cc1101 example.ino and NodeMCU blink code to test functionality.
*/
#include <Arduino.h>
#include "cc1101.h"
#include "ccpacket.h"

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define CC1101Interrupt 4 // Pin 19
#define CC1101_GDO0 19
#elif defined(__MK64FX512__)
// Teensy 3.5
#define CC1101Interrupt 9 // Pin 9
#define CC1101_GDO0 9
#else
#define CC1101Interrupt 5 // Pin D1
#define CC1101_GDO0 5
#define SS 15 //Select bit for HSPI bus on ESP8622 is pin 15
#endif

CC1101 radio;

byte syncWord[2] = {199, 10};
bool packetWaiting;

unsigned long lastSend = 0;
unsigned int sendDelay = 500;

void messageReceived()
{
  packetWaiting = true;
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  pinMode(SS, OUTPUT);
  radio.init();
  radio.setSyncWord(syncWord);
  // radio.setCarrierFreq(CFREQ_433);
  radio.disableAddressCheck();
  radio.setTxPowerAmp(PA_LongDistance);
  // radio.setModulation(ASK_OOK);
  radio.set433MHzAsk();
  //TODO: Determine why this is centered at 309Mhz not 300, play with smartRF settings?
  radio.setCarrierFreq(CFREQ_300);
  delay(1000);

  Serial.begin(9600);
  Serial.println("Beginning communication:");
  Serial.print(F("CC1101_PARTNUM "));
  Serial.println(radio.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_VERSION "));
  Serial.println(radio.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_MARCSTATE "));
  Serial.println(radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);
  Serial.print(F("CC1101_PATABLE "));
  byte buff[CC1101_PATABLE_SIZE];
  radio.readBurstReg(buff, CC1101_PATABLE, CC1101_PATABLE_SIZE); //FIXME, make private again
  for(size_t i = 0; i < CC1101_PATABLE_SIZE; ++i)
  {
    Serial.print(buff[i]);
    Serial.print(", ");
  }
  
  Serial.println();

  Serial.println(F("CC1101 radio initialized."));
  // attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
}

// Get signal strength indicator in dBm.
// See: http://www.ti.com/lit/an/swra114d/swra114d.pdf
int rssi(char raw)
{
  uint8_t rssi_dec;
  // TODO: This rssi_offset is dependent on baud and MHz; this is for 38.4kbps and 433 MHz.
  uint8_t rssi_offset = 74;
  rssi_dec = (uint8_t)raw;
  if (rssi_dec >= 128)
    return ((int)(rssi_dec - 256) / 2) - rssi_offset;
  else
    return (rssi_dec / 2) - rssi_offset;
}

// Get link quality indicator.
int lqi(char raw)
{
  return 0x3F - raw;
}

const char *sendBits[] = {"111", "000", "111", "000"};
uint8_t sendBitsIndex = 0;
uint8_t sendBitsSize = 4;

// the loop function runs over and over again forever
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Note that LOW is the voltage level
  if (packetWaiting)
  {
    // detachInterrupt(CC1101Interrupt);
    packetWaiting = false;
    CCPACKET packet;
    if (radio.receiveData(&packet) > 0)
    {
      Serial.println(F("Received packet..."));
      if (!packet.crc_ok)
      {
        Serial.println(F("crc not ok"));
      }
      Serial.print(F("lqi: "));
      Serial.println(lqi(packet.lqi));
      Serial.print(F("rssi: "));
      Serial.print(rssi(packet.rssi));
      Serial.println(F("dBm"));

      if (packet.crc_ok && packet.length > 0)
      {
        Serial.print(F("packet: len "));
        Serial.println(packet.length);
        Serial.println(F("data: "));
        Serial.println((const char *)packet.data);
      }
    }

    // attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
  }
  unsigned long now = millis();

  if (now > lastSend + sendDelay)
  {
    digitalWrite(LED_BUILTIN, LOW);
    // detachInterrupt(CC1101Interrupt); //TODO: Determine why these interrupts cause soft resets

    lastSend = now;
    // const char *message = "1010";
    const char *message = sendBits[sendBitsIndex % sendBitsSize];
    sendBitsIndex++;
    CCPACKET packet;
    // We also need to include the 0 byte at the end of the string
    packet.length = strlen(message) + 1;
    strncpy((char *)packet.data, message, packet.length);

    if (radio.sendData(packet))
    {
      Serial.println(F("Sent packet..."));
    }
    else
    {
      Serial.println(F("Failed to send packet..."));
    }

    // attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
  }
}
