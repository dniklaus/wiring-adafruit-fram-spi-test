/*
 * main.cpp
 *
 *  Created on: 15.03.2017
 *      Author: niklausd
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_FRAM_SPI.h>

// PlatformIO libraries
#include <SerialCommand.h>  // pio lib install 173, lib details see https://github.com/kroimon/Arduino-SerialCommand
#include <Timer.h>          // pio lib install 1699, lib details see https://github.com/dniklaus/wiring-timer

// private libraries
#include <DbgCliNode.h>
#include <DbgCliTopic.h>
#include <DbgCliCommand.h>
#include <DbgTracePort.h>
#include <DbgTraceContext.h>
#include <DbgTraceOut.h>
#include <DbgPrintConsole.h>
#include <DbgTraceLevel.h>
#include <AppDebug.h>
#include <ProductDebug.h>
#include <RamUtils.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED 13
#endif

SerialCommand* sCmd = 0;

/* Example code to interrogate Adafruit SPI FRAM breakout for address size and storage capacity */

/* NOTE: This sketch will overwrite data already on the FRAM breakout */

uint8_t FRAM_CS = 10;
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI();  // use hardware SPI

uint8_t FRAM_SCK = 13;
uint8_t FRAM_MISO = 12;
uint8_t FRAM_MOSI = 11;
//Or use software SPI, any pins!
//Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

uint8_t           addrSizeInBytes = 2; //Default to address size of two bytes
uint32_t          memSize;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

int32_t readBack(uint32_t addr, int32_t data)
{
  int32_t check = !data;
  int32_t wrapCheck, backup;
  fram.read(addr, (uint8_t*) &backup, sizeof(int32_t));
  fram.writeEnable(true);
  fram.write(addr, (uint8_t*) &data, sizeof(int32_t));
  fram.writeEnable(false);
  fram.read(addr, (uint8_t*) &check, sizeof(int32_t));
  fram.read(0, (uint8_t*) &wrapCheck, sizeof(int32_t));
  fram.writeEnable(true);
  fram.write(addr, (uint8_t*) &backup, sizeof(int32_t));
  fram.writeEnable(false);
  // Check for warparound, address 0 will work anyway
  if (wrapCheck == check)
  {
    check = 0;
  }
  return check;
}

bool testAddrSize(uint8_t addrSize)
{
  fram.setAddressSize(addrSize);
  if (readBack(4, 0xbeefbead) == 0xbeefbead)
  {
    return true;
  }
  return false;
}

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 0);

  setupProdDebugEnv();

  if (fram.begin(FRAM_CS, addrSizeInBytes))
  {
    Serial.println("Found SPI FRAM");
  }
  else
  {
    Serial.println("No SPI FRAM found ... check your connections\r\n");
    while (1)
      ;
  }

  if (testAddrSize(2))
  {
    addrSizeInBytes = 2;
  }
  else if (testAddrSize(3))
  {
    addrSizeInBytes = 3;
  }
  else if (testAddrSize(4))
  {
    addrSizeInBytes = 4;
  }
  else
  {
    Serial.println(
        "SPI FRAM can not be read/written with any address size\r\n");
    while (1)
    {
      ;
    }
  }

  memSize = 0;
  while (readBack(memSize, memSize) == memSize)
  {
    memSize += 256;
    //Serial.print("Block: #"); Serial.println(memSize/256);
  }

  Serial.print("SPI FRAM address size is ");
  Serial.print(addrSizeInBytes);
  Serial.println(" bytes.");
  Serial.println("SPI FRAM capacity appears to be..");
  Serial.print(memSize); Serial.println(" bytes");
  Serial.print(memSize/0x400); Serial.println(" kilobytes");
  Serial.print((memSize*8)/0x400); Serial.println(" kilobits");
  if (memSize >= (0x100000/8))
  {
    Serial.print((memSize*8)/0x100000); Serial.println(" megabits");
  }
}

void loop()
{
  if (0 != sCmd)
  {
    sCmd->readSerial();     // process serial commands
  }
  yield();                  // process Timers
}
