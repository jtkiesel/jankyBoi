// mt_message.h
//
// Author: Justin Marple, Team BNS
// Contact: justinjmarple@gmail.com
// Date: 2/4/2018
//
// The mt_message portion of the Xsens library contains code for sending,
// receiving and parsing data from the MTi IMU.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef __MT_MESSAGE_H__
#define __MT_MESSAGE_H__

#include "API.h"
#include <stdint.h>

// A MTMessage is structured as follows:
//  Preamble - 1 Byte - 0xFA
//  BID - 1 Byte - 0xFF
//  MID - 1 Byte - Message Identifier
//  LEN - 1 Byte - Standard length: 0-254, Extended: 255
//  DATA - 0 to 254 Bytes
//  CHECKSUM - 1 Byte - Checksum of the message
struct MTMessage
{
  int mid;
  int len;
  uint8_t data[256];
};

struct MTData2
{
  //double XDI_Temperature;
  //_ XDI_UtcTime;
  uint16_t XDI_PacketCounter;
  uint32_t XDI_SampleTimeFine;
  //uint32_t XDI_SampleTimeCoarse;
  //double XDI_Quaternion[4];
  //double XDI_EulerAngles[3];
  //double XDI_RotationMatrix[9];
  //uint32_t XDI_BaroPressure;
  double XDI_DeltaV[3];
  double XDI_DeltaQ[4];
  double XDI_Acceleration[3];
  //double XDI_FreeAcceleration[3];
  //double XDI_AccelerationHR[3];
  double XDI_RateOfTurn[3];
  //double XDI_RateOfTurnHR[3];
  //_ XDI_RawAccGyrMagTemp;
  //int16_t XDI_RawGyroTemp[2];
  double XDI_MagneticField[3];
  //uint8_t XDI_StatusByte;
  uint32_t XDI_StatusWord;
  // double XDI_PositionEcef[3];
  // double XDI_LatLon[2];
  // double XDI_AltitudeEllipsoid;
  // double XDI_VelocityXYZ[3];

  // Mutex so that data can be updated safetly
  Mutex mutex;
};

#define XDI_PACKETCOUNTER 0x1020
#define XDI_SAMPLETIMEFINE 0x1060
#define XDI_DELTAQ 0x8030
#define XDI_RATEOFTURN 0x8020
#define XDI_MAGNETICFIELD 0xC020
#define XDI_DELTAV 0x4010
#define XDI_ACCELERATION 0x4020
#define XDI_STATUSWORD 0xE020

enum MTBaudrate
{
  BAUD_4M0000 = 0x0D,
  BAUD_3M6864 = 0x0E,
  BAUD_2M0000 = 0x0C,
  BAUD_921K6 = 0x0A,
  BAUD_460K8 = 0x00,
  BAUD_230K4 = 0x01,
  BAUD_115K2 = 0x02,
  BAUD_76K6 = 0x03,
  BAUD_57K6 = 0x04,
  BAUD_38K4 = 0x05,
  BAUD_28K8 = 0x06,
  BAUD_19K2 = 0x07,
  BAUD_14K4 = 0x08,
  BAUD_9K6 = 0x09,
  BAUD_4K8 = 0x0B
};

// Blocking function that waits until it recieves a complete message
// Returns 0 if message is succesful
// Returns 1 if checksum error
int getMTMessage(struct MTMessage* msg, FILE* usart);

// Sends a message over a given UART port to an xsens device
int sendMTMessage(struct MTMessage* msg, FILE* usart);

// Switch the active state of the device from Measurement State to Config State.
// This message can also be used in Config State to confirm that Config State is
// currently the active state.
void mtGoToConfig(FILE* usart);

// Switch the active state of the device from Config State to Measurement State.
// The current configuration settings are used to start the measurement.
void mtGoToMeasurement(FILE* usart);

// Sending this message will cause the MT to reset and to activate the WakeUp
// procedure. An acknowledge message will be sent to confirm reception of the
// Reset message.
void mtReset(FILE* usart);

// Request to send the device identifier (or serial number). MT acknowledges by
// sending the DeviceID message
void mtReqDID(FILE* usart);

// Request to send the product code. MT acknowledges by sending the ProductCode
// message
void mtReqProductCode(FILE* usart);

// Request to send the hardware revision of the device. MT acknowledges by
// sending HardwareRev message.
void mtReqHardwareVersion(FILE* usart);

// Request to send the firmware revision of the device. MT acknowledges by
// sending FirmwareRev message
void mtReqFWRev(FILE* usart);

// Runs the built-in self test.
void mtRunSelfTest(FILE* usart);

// Request the baud rate of the device.
void mtReqBaudrate(FILE* usart);

// This message changes the baud rate of the communication interface (RS-232 or
// RS-422). The new baudrate will be stored in non-volatile memory and will
// become active after issuing the Reset message or power cycle.
void mtSetBaudrate(FILE* usart, enum MTBaudrate baudrate);

// Request the current error mode.
void mtReqErrorMode(FILE* usart);

// Set the error mode to a specific ERRORMODE.
void mtSetErrorMode(FILE* usart, unsigned int errormode);

// Returns 1 if the message is a MTData2 message
int isMTData2(struct MTMessage* msg);

// Takes a MTMessage and parses it for MTData2 data
void parseMTData2(struct MTMessage* msg, struct MTData2* data);
#endif
