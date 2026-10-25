// MPU6050 acquisition task (MPU6050_light 1.1.0).
//
// A dedicated FreeRTOS task is used because the 100 Hz update must keep its
// cadence while the main loop performs blocking DHT reads (~25 ms for a
// DHT11 start pulse + frame) and radio work. The task owns the I2C bus
// exclusively; other code only reads snapshots copied under a spinlock.
//
// Units: acceleration in g, angles in degrees. Axes are the MPU6050 chip
// axes. Roll = rotation about X, pitch = rotation about Y, from the library's
// complementary filter (gyro weight 0.98). Calibration (calcOffsets) assumes
// the board is stationary with +Z pointing up; it zeroes gyro bias and the
// accelerometer offsets so the boot orientation reads as level.
#pragma once

#include <stdint.h>

#include "vc_protocol.h"

struct ImuSnapshot {
  vc::proto::SensorStatus status = vc::proto::SensorStatus::Absent;
  float accelXG = 0, accelYG = 0, accelZG = 0, accelMagG = 0;
  float rollDeg = 0, pitchDeg = 0;
  uint32_t sampleMs = 0;     // millis() of the latest update
  float rateHz = 0;          // updates in the last complete second
  uint16_t maxGapMs = 0;     // largest gap between updates since last take()
  float peakAccelG = 0;      // max |a| since last take()
  uint32_t samples = 0;      // total updates since boot
  uint32_t overruns = 0;     // update took longer than one period
  uint32_t maxUpdateUs = 0;  // longest single update since boot
  uint32_t i2cErrors = 0;
};

namespace imu {

// Starts the task. period: target update period in ms.
bool start(int sda, int scl, uint32_t i2cHz, uint32_t periodMs, bool calibrate);
// Copy of the latest state; take() also resets the per-packet peak and gap.
ImuSnapshot peek();
ImuSnapshot take();
// Max |a| over the last `windowMs` (<= 320 ms) of 20 ms buckets.
float recentPeakG(uint32_t windowMs);

}  // namespace imu
