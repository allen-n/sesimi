/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 * Copyright (c) 2016 Tyler Sommer <contact@tylersommer.pro>
 * 
 * This file is part of the CC1101 project.
 * 
 * CC1101 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * CC1101 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with CC1101; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 03/03/2011
 */

#include "cc1101.h"

/*
  Board definition Macros
*/
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define CC1101Interrupt 4 // Pin 19
#define CC1101_GDO0 19
#elif defined(__MK64FX512__)
// Teensy 3.5
#define CC1101Interrupt 9 // Pin 9
#define CC1101_GDO0 9
#else
// Interrupt Pin (also GDO0)
#define CC1101Interrupt 3 // Pin D1
#define CC1101_GDO0 5
// Serial clock output line of CC1101
#define CC1101_SCLK 4
// Data out line to CC1101 for continuous TX
#define CC1101_GDO2 4
// Select bit for HSPI bus on ESP8622 is pin 15
#define SS 15
#endif

/**
 * Macros
 */
// Select (SPI) CC1101
#define cc1101_Select() digitalWrite(SS, LOW)
// Deselect (SPI) CC1101
#define cc1101_Deselect() digitalWrite(SS, HIGH)
// Wait until SPI MISO line goes low
#define wait_Miso() while (digitalRead(MISO) > 0)
// Get GDO0 pin state
#define getGDO0state() digitalRead(CC1101_GDO0)
// Wait until GDO0 line goes high
#define wait_GDO0_high() while (!getGDO0state())
// Wait until GDO0 line goes low
#define wait_GDO0_low() while (getGDO0state())

/*
  Continuous TX Macros
*/
// Continuous TX 1
#define serial_Select() digitalWrite(CC1101_GDO0, HIGH)
// Continuous TX 0
#define serial_Deselect() digitalWrite(CC1101_GDO0, LOW)
// Get SCLK pin state
#define getSCLKstate() digitalRead(CC1101_SCLK)
// Wait until SCLK line goes high
#define wait_SCLK_high() while (!getSCLKstate())
// Wait until SCLK line goes low
#define wait_SCLK_low() while (getSCLKstate())

/**
  * PATABLE
  */
//const byte paTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};

/**
 * CC1101
 * 
 * Class constructor
 */
CC1101::CC1101(void)
{
  carrierFreq = CFREQ_868;
  channel = CC1101_DEFVAL_CHANNR;
  syncWord[0] = CC1101_DEFVAL_SYNC1;
  syncWord[1] = CC1101_DEFVAL_SYNC0;
  devAddress = CC1101_DEFVAL_ADDR;
}

/**
 * wakeUp
 * 
 * Wake up CC1101 from Power Down state
 */
void CC1101::wakeUp(void)
{
  cc1101_Select();   // Select CC1101
  wait_Miso();       // Wait until MISO goes low
  cc1101_Deselect(); // Deselect CC1101
}

/**
 * writeReg
 * 
 * Write single register into the CC1101 IC via SPI
 * 
 * 'regAddr'	Register address
 * 'value'	Value to be writen
 */
void CC1101::writeReg(byte regAddr, byte value)
{
  cc1101_Select();       // Select CC1101
  wait_Miso();           // Wait until MISO goes low
  SPI.transfer(regAddr); // Send register address
  SPI.transfer(value);   // Send value
  cc1101_Deselect();     // Deselect CC1101
}

/**
 * writeBurstReg
 * 
 * Write multiple registers into the CC1101 IC via SPI
 * 
 * 'regAddr'	Register address
 * 'buffer'	Data to be writen
 * 'len'	Data length
 */
void CC1101::writeBurstReg(byte regAddr, byte *buffer, byte len)
{
  byte addr, i;

  addr = regAddr | WRITE_BURST; // Enable burst transfer
  cc1101_Select();              // Select CC1101
  wait_Miso();                  // Wait until MISO goes low
  SPI.transfer(addr);           // Send register address

  for (i = 0; i < len; i++)
    SPI.transfer(buffer[i]); // Send value

  cc1101_Deselect(); // Deselect CC1101
}

/**
 * cmdStrobe
 * 
 * Send command strobe to the CC1101 IC via SPI
 * 
 * 'cmd'	Command strobe
 */
void CC1101::cmdStrobe(byte cmd)
{
  cc1101_Select();   // Select CC1101
  wait_Miso();       // Wait until MISO goes low
  SPI.transfer(cmd); // Send strobe command
  cc1101_Deselect(); // Deselect CC1101
}

/**
 * readReg
 * 
 * Read CC1101 register via SPI
 * 
 * 'regAddr'	Register address
 * 'regType'	Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 * 
 * Return:
 * 	Data byte returned by the CC1101 IC
 */
byte CC1101::readReg(byte regAddr, byte regType)
{
  byte addr, val;

  addr = regAddr | regType;
  cc1101_Select();          // Select CC1101
  wait_Miso();              // Wait until MISO goes low
  SPI.transfer(addr);       // Send register address
  val = SPI.transfer(0x00); // Read result
  cc1101_Deselect();        // Deselect CC1101

  return val;
}

/**
 * readBurstReg
 * 
 * Read burst data from CC1101 via SPI
 * 
 * 'buffer'	Buffer where to copy the result to
 * 'regAddr'	Register address
 * 'len'	Data length
 */
void CC1101::readBurstReg(byte *buffer, byte regAddr, byte len)
{
  byte addr, i;

  addr = regAddr | READ_BURST;
  cc1101_Select();    // Select CC1101
  wait_Miso();        // Wait until MISO goes low
  SPI.transfer(addr); // Send register address
  for (i = 0; i < len; i++)
    buffer[i] = SPI.transfer(0x00); // Read result byte by byte
  cc1101_Deselect();                // Deselect CC1101
}

/**
 * reset
 * 
 * Reset CC1101
 */
void CC1101::reset(void)
{
  cc1101_Deselect(); // Deselect CC1101
  delayMicroseconds(5);
  cc1101_Select(); // Select CC1101
  delayMicroseconds(10);
  cc1101_Deselect(); // Deselect CC1101
  delayMicroseconds(41);
  cc1101_Select(); // Select CC1101

  wait_Miso();               // Wait until MISO goes low
  SPI.transfer(CC1101_SRES); // Send reset command strobe
  wait_Miso();               // Wait until MISO goes low

  cc1101_Deselect(); // Deselect CC1101

  setCCregs(); // Reconfigure CC1101
}

/**
 * setCCregs
 * 
 * Configure CC1101 registers
 */
void CC1101::setCCregs(void)
{
  writeReg(CC1101_IOCFG2, CC1101_DEFVAL_IOCFG2);
  writeReg(CC1101_IOCFG1, CC1101_DEFVAL_IOCFG1);
  writeReg(CC1101_IOCFG0, CC1101_DEFVAL_IOCFG0);
  writeReg(CC1101_FIFOTHR, CC1101_DEFVAL_FIFOTHR);
  writeReg(CC1101_PKTLEN, CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1, CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);

  // Set default synchronization word
  setSyncWord(syncWord);

  // Set default device address
  setDevAddress(devAddress);

  // Set default frequency channel
  setChannel(channel);

  writeReg(CC1101_FSCTRL1, CC1101_DEFVAL_FSCTRL1);
  writeReg(CC1101_FSCTRL0, CC1101_DEFVAL_FSCTRL0);

  // Set default carrier frequency = 868 MHz
  setCarrierFreq(carrierFreq);

  // RF speed
  if (workMode == MODE_LOW_SPEED)
    writeReg(CC1101_MDMCFG4, CC1101_DEFVAL_MDMCFG4_4800);
  else
    writeReg(CC1101_MDMCFG4, CC1101_DEFVAL_MDMCFG4_38400);

  writeReg(CC1101_MDMCFG3, CC1101_DEFVAL_MDMCFG3);
  writeReg(CC1101_MDMCFG2, CC1101_DEFVAL_MDMCFG2);
  writeReg(CC1101_MDMCFG1, CC1101_DEFVAL_MDMCFG1);
  writeReg(CC1101_MDMCFG0, CC1101_DEFVAL_MDMCFG0);
  writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
  writeReg(CC1101_MCSM2, CC1101_DEFVAL_MCSM2);
  writeReg(CC1101_MCSM1, CC1101_DEFVAL_MCSM1);
  writeReg(CC1101_MCSM0, CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_FOCCFG, CC1101_DEFVAL_FOCCFG);
  writeReg(CC1101_BSCFG, CC1101_DEFVAL_BSCFG);
  writeReg(CC1101_AGCCTRL2, CC1101_DEFVAL_AGCCTRL2);
  writeReg(CC1101_AGCCTRL1, CC1101_DEFVAL_AGCCTRL1);
  writeReg(CC1101_AGCCTRL0, CC1101_DEFVAL_AGCCTRL0);
  writeReg(CC1101_WOREVT1, CC1101_DEFVAL_WOREVT1);
  writeReg(CC1101_WOREVT0, CC1101_DEFVAL_WOREVT0);
  writeReg(CC1101_WORCTRL, CC1101_DEFVAL_WORCTRL);
  writeReg(CC1101_FREND1, CC1101_DEFVAL_FREND1);
  writeReg(CC1101_FREND0, CC1101_DEFVAL_FREND0);
  writeReg(CC1101_FSCAL3, CC1101_DEFVAL_FSCAL3);
  writeReg(CC1101_FSCAL2, CC1101_DEFVAL_FSCAL2);
  writeReg(CC1101_FSCAL1, CC1101_DEFVAL_FSCAL1);
  writeReg(CC1101_FSCAL0, CC1101_DEFVAL_FSCAL0);
  writeReg(CC1101_RCCTRL1, CC1101_DEFVAL_RCCTRL1);
  writeReg(CC1101_RCCTRL0, CC1101_DEFVAL_RCCTRL0);
  writeReg(CC1101_FSTEST, CC1101_DEFVAL_FSTEST);
  writeReg(CC1101_PTEST, CC1101_DEFVAL_PTEST);
  writeReg(CC1101_AGCTEST, CC1101_DEFVAL_AGCTEST);
  writeReg(CC1101_TEST2, CC1101_DEFVAL_TEST2);
  writeReg(CC1101_TEST1, CC1101_DEFVAL_TEST1);
  writeReg(CC1101_TEST0, CC1101_DEFVAL_TEST0);

  // Send empty packet
  CCPACKET packet;
  packet.length = 0;
  sendData(packet);
}

/**
 * init
 * 
 * Initialize CC1101 radio
 *
 * @param freq Carrier frequency
 * @param mode Working mode (speed, ...)
 */
void CC1101::init(uint8_t freq, uint8_t mode)
{
  carrierFreq = freq;
  workMode = mode;
#ifdef ESP32
  pinMode(SS, OUTPUT); // Make sure that the SS Pin is declared as an Output
#endif
  SPI.begin();                 // Initialize SPI interface
  pinMode(CC1101_GDO0, INPUT); // Config GDO0 as input
  pinMode(SS, OUTPUT);
  pinMode(CC1101Interrupt, INPUT);
  pinMode(CC1101_SCLK, INPUT);
  pinMode(CC1101_GDO2, INPUT);

  reset(); // Reset CC1101

  // Configure PATABLE
  setTxPowerAmp(PA_LowPower);
}

/**
 * setSyncWord
 * 
 * Set synchronization word
 * 
 * 'syncH'	Synchronization word - High byte
 * 'syncL'	Synchronization word - Low byte
 */
void CC1101::setSyncWord(uint8_t syncH, uint8_t syncL)
{
  writeReg(CC1101_SYNC1, syncH);
  writeReg(CC1101_SYNC0, syncL);
  syncWord[0] = syncH;
  syncWord[1] = syncL;
}

/**
 * setSyncWord (overriding method)
 * 
 * Set synchronization word
 * 
 * 'syncH'	Synchronization word - pointer to 2-byte array
 */
void CC1101::setSyncWord(byte *sync)
{
  CC1101::setSyncWord(sync[0], sync[1]);
}

/**
 * setDevAddress
 * 
 * Set device address
 * 
 * @param addr	Device address
 */
void CC1101::setDevAddress(byte addr)
{
  writeReg(CC1101_ADDR, addr);
  devAddress = addr;
}

/**
 * setChannel
 * 
 * Set frequency channel
 * 
 * 'chnl'	Frequency channel
 */
void CC1101::setChannel(byte chnl)
{
  writeReg(CC1101_CHANNR, chnl);
  channel = chnl;
}

/**
 * setCarrierFreq
 * 
 * Set carrier frequency
 * 
 * 'freq'	New carrier frequency
 */
void CC1101::setCarrierFreq(byte freq)
{
  switch (freq)
  {
  case CFREQ_915:
    writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_915);
    writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_915);
    writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_915);
    break;
  case CFREQ_433:
    writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_433);
    writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_433);
    writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_433);
    break;
  case CFREQ_918:
    writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_918);
    writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_918);
    writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_918);
    break;
  case CFREQ_300:
    writeReg(CC1101_FREQ2, 0x0B);
    writeReg(CC1101_FREQ1, 0x89);
    writeReg(CC1101_FREQ0, 0xD8);
    break;
  default:
    writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_868);
    writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_868);
    writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_868);
    break;
  }

  carrierFreq = freq;
}

/**
     * setModulation
     * 
     * Set modulation scheme
     * 
     * 'mod'	is the new modulation scheme, 
     * see MODULATION enum for options
     */
void CC1101::setModulation(byte mod)
{

  byte PA_Power_FS[CC1101_PATABLE_SIZE] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte PA_Power[CC1101_PATABLE_SIZE] = {0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  setPATable(PA_Power_FS, CC1101_PATABLE_SIZE); //PA for all non ASK modulation is this
  switch (mod)
  {
  case ASK_OOK:
    writeReg(CC1101_MDMCFG2, 0x32);
    writeReg(CC1101_FREND0, 0x11);
    setPATable(PA_Power, CC1101_PATABLE_SIZE);
    break;
  case FSK2:
    writeReg(CC1101_MDMCFG2, 0x02);
    writeReg(CC1101_FREND0, 0x10);
    writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
    break;
  case FSK4:
    writeReg(CC1101_MDMCFG2, 0x42);
    writeReg(CC1101_FREND0, 0x10);
    writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
    break;
  case GFSK:
    writeReg(CC1101_MDMCFG2, 0x12);
    writeReg(CC1101_FREND0, 0x10);
    writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
    break;
  case MSK:
    writeReg(CC1101_MDMCFG2, 0x72);
    writeReg(CC1101_DEVIATN, 0x40);
    writeReg(CC1101_FREND0, CC1101_DEFVAL_FREND0);
    break;
  default:
    writeReg(CC1101_MDMCFG2, CC1101_DEFVAL_MDMCFG2);
    writeReg(CC1101_FREND0, CC1101_DEFVAL_FREND0);
    writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
    break;
  }

  devModulation = mod;
}

/**
     * setDataRate
     * 
     * Set datarate in Kbps
     * 
     * 'rate'	is the new datarate, 
     */
void CC1101::setDataRate(uint8_t rate)
{
  //  TODO: Add relevant data rates
}

/**
     * setContinuousTx
     * 
     * Set tx mode (continuous or packet)
     * 
     * 'continous' is a boolean, if true, 
     * tx is continous, else it is packet
     */
void CC1101::setContinuousTx(bool continuous)
{
  if (continuous)
  {
    writeReg(CC1101_IOCFG2, 0x0B);
    writeReg(CC1101_IOCFG0, 0x0C);
    writeReg(CC1101_PKTCTRL0, 0x12);
    writeReg(CC1101_MDMCFG2, 0x30);
  }
  else
  {
    writeReg(CC1101_IOCFG2, CC1101_DEFVAL_IOCFG2);
    writeReg(CC1101_IOCFG0, CC1101_DEFVAL_IOCFG0);
    writeReg(CC1101_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);
    writeReg(CC1101_MDMCFG2, CC1101_DEFVAL_MDMCFG2);
  }
}

/**
     * setPATable with byte array, up to 8 entries
     * 
     * Set PATABLE value
     * 
     * @param paLevel array of amplification value bytes (up to 8 max)
     * return:
     *   true if valid input, else false
     */
bool CC1101::setPATable(uint8_t paLevel[], uint8_t len)
{
  if (len > 8)
    return false;
  else
    writeBurstReg(CC1101_PATABLE, paLevel, len);
}

/**
 * setPowerDownState
 * 
 * Put CC1101 into power-down state
 */
void CC1101::setPowerDownState()
{
  // Comming from RX state, we need to enter the IDLE state first
  cmdStrobe(CC1101_SIDLE);
  // Enter Power-down state
  cmdStrobe(CC1101_SPWD);
}

/**
 * sendDataSerial
 * 
 * Send data packets continuously via RF
 * 'txBuffer' is a byte buffer to be transmitted
 * 'size' is the size of the byte buffer
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 **/
bool CC1101::sendDataSerial(byte *txBuffer, uint8_t size)
{
  if (size <= 0)
    return false;

  // String resString = "";
  // MSB of the first byte needs to ready on the data line before strobing TX
  byte firstByte = txBuffer[0];

  if (((firstByte >> 7) & 0x01) == 0)
  {
    serial_Deselect();
    // resString += '0';
  }
  else
  {
    serial_Select();
    // resString += '1';
  }

  setTxState();

  // Transmit the first byte
  for (uint8_t i = 1; i < 8; i++)
  {
    wait_SCLK_high();

    if (((firstByte >> (7 - i)) & 0x01) == 0)
    {
      serial_Deselect();
      // resString += '0';
    }
    else
    {
      serial_Select();
      // resString += '1';
    }

    wait_SCLK_low();
  }

  // Transmite remaining bytes
  for (uint8_t buffIdx = 1; buffIdx < size; ++buffIdx)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      wait_SCLK_high();

      if (((txBuffer[buffIdx] >> (7 - i)) & 0x01) == 0)
      {
        serial_Deselect();
        // resString += '0';
      }
      else
      {
        serial_Select();
        // resString += '1';
      }

      wait_SCLK_low();
    }
  }
  // Serial.print(// resString);
  wait_SCLK_high();
  wait_SCLK_low();

  // Transmit 13 dummy bits (must be 26 if Manchester mode is enabled)
  serial_Deselect();
  for (uint8_t i = 0; i < 8; ++i)
  {
    wait_SCLK_high();
    wait_SCLK_low();
  }
  setIdleState();
  return true;
}

/**
 * sendData
 * 
 * Send data packet via RF
 * 
 * 'packet'	Packet to be transmitted. First byte is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool CC1101::sendData(CCPACKET packet)
{
  byte marcState;
  bool res = false;

  // Declare to be in Tx state. This will avoid receiving packets whilst
  // transmitting
  rfState = RFSTATE_TX;

  // Enter RX state
  setRxState();

  int tries = 0;
  // Check that the RX state has been entered
  while (tries++ < 1000 && ((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D)
  {
    if (marcState == 0x11) // RX_OVERFLOW
      flushRxFifo();       // flush receive queue
  }
  if (tries >= 1000)
  {
    // TODO: MarcState sometimes never enters the expected state; this is a hack workaround.
    return res;
  }
  // TODO: Modify this function to make sure it works properly with
  // continuous transmit mode, see:
  // https://e2e.ti.com/support/wireless-connectivity/other-wireless/f/667/t/157309
  // cc1101 datasheet page 33
  delayMicroseconds(500);

  if (packet.length > 0)
  {
    // Set data length at the first position of the TX FIFO
    writeReg(CC1101_TXFIFO, packet.length);
    // Write data into the TX FIFO
    writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);

    // CCA enabled: will enter TX state only if the channel is clear
    setTxState();
  }

  // Check that TX state is being entered (state = RXTX_SETTLING)
  marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
  if ((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15))
  {
    setIdleState(); // Enter IDLE state
    flushTxFifo();  // Flush Tx FIFO
    setRxState();   // Back to RX state

    // Declare to be in Rx state
    rfState = RFSTATE_RX;
    return res;
  }

  // Wait for the sync word to be transmitted
  wait_GDO0_high();

  // Wait until the end of the packet transmission
  wait_GDO0_low();

  // Check that the TX FIFO is empty
  if ((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
    res = true;

  setIdleState(); // Enter IDLE state
  flushTxFifo();  // Flush Tx FIFO

  // Enter back into RX state
  setRxState();

  // Declare to be in Rx state
  rfState = RFSTATE_RX;
  return res;
}

/**
 * receiveData
 * 
 * Read data packet from RX FIFO
 *
 * 'packet'	Container for the packet received
 * 
 * Return:
 * 	Amount of bytes received
 */
byte CC1101::receiveData(CCPACKET *packet)
{
  byte val;
  byte rxBytes = readStatusReg(CC1101_RXBYTES);

  // Any byte waiting to be read and no overflow?
  if (rxBytes & 0x7F && !(rxBytes & 0x80))
  {
    // Read data length
    packet->length = readConfigReg(CC1101_RXFIFO);
    // If packet is too long
    if (packet->length > CCPACKET_DATA_LEN)
      packet->length = 0; // Discard packet
    else
    {
      // Read data packet
      readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
      // Read RSSI
      packet->rssi = readConfigReg(CC1101_RXFIFO);
      // Read LQI and CRC_OK
      val = readConfigReg(CC1101_RXFIFO);
      packet->lqi = val & 0x7F;
      packet->crc_ok = bitRead(val, 7);
    }
  }
  else
    packet->length = 0;

  setIdleState(); // Enter IDLE state
  flushRxFifo();  // Flush Rx FIFO
  //cmdStrobe(CC1101_SCAL);

  // Back to RX state
  setRxState();

  return packet->length;
}

/**
 * setRxState
 * 
 * Enter Rx state
 */
void CC1101::setRxState(void)
{
  cmdStrobe(CC1101_SRX);
  rfState = RFSTATE_RX;
}

/**
 * setTxState
 * 
 * Enter Tx state
 */
void CC1101::setTxState(void)
{
  cmdStrobe(CC1101_STX);
  rfState = RFSTATE_TX;
}

/**
     * setWhitenData
     * 
     * Turn data whitening on or off
     * 
     * 'whiten' when true, whiten data, else do not
     */
void CC1101::setWhitenData(bool whiten)
{
  if (whiten)
    writeReg(CC1101_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);
  else
    writeReg(CC1101_PKTCTRL0, 0x45);
}

/**
     * set433HzAsk
     * 
     * Use paramaters from SmartRFStudio to 
     * set 433 MHz, ASK transmission type
     * 
     */
void CC1101::set433MHzAsk()
{
  byte PA_Power[CC1101_PATABLE_SIZE] = {0x00, PA_LongDistance, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  setPATable(PA_Power, CC1101_PATABLE_SIZE);
  writeReg(0x02, 0x06);
  writeReg(0x03, 0x47);
  writeReg(0x08, 0x01); //change to 0x05 to get CRC back
  writeReg(0x0B, 0x06);
  writeReg(0x0D, 0x10);
  writeReg(0x0E, 0xA7);
  writeReg(0x0F, 0x62);
  writeReg(0x10, 0xF5);
  writeReg(0x11, 0x83);
  writeReg(0x12, 0x30);
  writeReg(0x15, 0x15);
  writeReg(0x18, 0x18);
  writeReg(0x19, 0x14);
  writeReg(0x1D, 0x92);
  writeReg(0x20, 0xFB);
  writeReg(0x22, 0x11);
  writeReg(0x23, 0xE9);
  writeReg(0x24, 0x2A);
  writeReg(0x25, 0x00);
  writeReg(0x26, 0x1F);
  writeReg(0x2C, 0x81);
  writeReg(0x2D, 0x35);
  writeReg(0x2E, 0x09);

  /**
   *  Address Config = No address check 
      Base Frequency = 432.999817 
      CRC Autoflush = false 
      CRC Enable = false 
      Carrier Frequency = 432.999817 
      Channel Number = 0 
      Channel Spacing = 199.951172 
      Data Format = Normal mode 
      Data Rate = 1.19948 
      Deviation = 5.157471 
      Device Address = 0 
      Manchester Enable = false 
      Modulation Format = ASK/OOK 
      PA Ramping = false 
      Packet Length = 255 
      Packet Length Mode = Variable packet length mode. Packet length configured by the first byte after sync word 
      Preamble Count = 4 
      RX Filter BW = 58.035714 
      Sync Word Qualifier Mode = No preamble/sync 
      TX Power = 0 
      Whitening = false 
 */
}

void CC1101::set300MhzAsk()
{
  byte PA_Power[CC1101_PATABLE_SIZE] = {0x00, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  setPATable(PA_Power, CC1101_PATABLE_SIZE);
  pinMode(CC1101_GDO0, OUTPUT); // Config GDO0 as input for serial TX
  // continuous mode
writeReg(0x00, 0x0B);
writeReg(0x02, 0x0C);
writeReg(0x03, 0x47);
writeReg(0x08, 0x12);
writeReg(0x0B, 0x06);
writeReg(CC1101_FREQ2, 0x0B);
writeReg(CC1101_FREQ1, 0x89);
writeReg(CC1101_FREQ0, 0xD8);
writeReg(CC1101_MDMCFG4, 0xF6);
writeReg(CC1101_MDMCFG3, 0x4E);
writeReg(0x12, 0x30);
writeReg(0x15, 0x15);
writeReg(0x18, 0x18);
writeReg(0x19, 0x16);
writeReg(0x20, 0xFB);
writeReg(0x22, 0x11);
writeReg(0x23, 0xE9);
writeReg(0x24, 0x2A);
writeReg(0x25, 0x00);
writeReg(0x26, 0x1F);
writeReg(0x2C, 0x81);
writeReg(0x2D, 0x35);
writeReg(0x2E, 0x00);


  /**
  Address Config = No address check 
  Base Frequency = 299.999756 
  CRC Autoflush = false 
  CRC Enable = false 
  Carrier Frequency = 299.999756 
  Channel Number = 0 
  Channel Spacing = 199.951172 
  Data Format = Synchronous serial mode 
  Data Rate = 2.07043
  Deviation = 5.157471 
  Device Address = 0 
  Manchester Enable = false 
  Modulated = false 
  Modulation Format = ASK/OOK 
  PA Ramping = false 
  Packet Length = 255 
  Packet Length Mode = Infinite packet length mode 
  Preamble Count = 4 
  RX Filter BW = 58.035714 
  Sync Word Qualifier Mode = No preamble/sync 
  TX Power = 10 
  Whitening = false
   **/
}