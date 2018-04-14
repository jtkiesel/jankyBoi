// xsens.h
//
// Author: Justin Marple, Team BNS
// Contact: justinjmarple@gmail.com
// Date: 2/4/2018
//
// The xsens portion of the Xsens library contains code for setting up and
// gathering useful data from the MTi IMU's.  This is done by using MTMessage
// function calls.
// NOTE: This library depends on having liblogger installed.  Install from
//       here https://github.com/JMarple/liblogger_pros
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef __XSENS_H__
#define __XSENS_H__

#include "API.h"
#include "mt_message.h"

// Depends on https://github.com/JMarple/liblogger_pros
#include "logger.h"

// A structure that holds data from the IMU
struct XsensVex
{
  // Accounts for a bias on the rateOfTurn output.  The RateOfTurn output
  // is in rad/sec, so heading_bias is also in rad/sec.
  double heading_bias[3];

  // Accumulated heading, in degrees.
  double heading[3];

  // PROS File handler for the UART port
  FILE* usart;

  // Task handler for the thread reading and writing data
  TaskHandle handle;

  // Last recorded packet off the MTI device.
  // NOTE: Use the mutex in MTdata2 to read data safetly
  struct MTData2 lastPacket;

  // Logger for capturing information within the library
  struct Logger log;
};

// Initializes the xsens structure and UART port on the cortex.
// Ensure baud_rate matches the set baud rate on your XSENS IMU
void xsens_init(struct XsensVex* x, FILE* usart, unsigned int baud_rate);

// Starts the main monitoring task that will put the XSENS into the
// measurement state and start saving packets.
void xsens_start_task(struct XsensVex* x);

// NOTE: Use function only when the robot is known to be perfectly still.
// This gathers N samples and gets the average "bias" of the gyroscope output.
// This will eliminate the majority of the drift seen on these IMU's.
// TODO: It appears the bias doesn't really change over time.  Can this be run
// once and the bias be hardcoded afterwords?
void xsens_calibrate(struct XsensVex* x, int numSamples);

// Resets the gyroscope's heading
void xsens_reset_heading(struct XsensVex* x,
  double pitch, double roll, double yaw);

// Returns the current angles on a given axis, accumulated by the gyroscope
double xsens_get_pitch(struct XsensVex* x);
double xsens_get_roll(struct XsensVex* x);
double xsens_get_yaw(struct XsensVex* x);

#endif
