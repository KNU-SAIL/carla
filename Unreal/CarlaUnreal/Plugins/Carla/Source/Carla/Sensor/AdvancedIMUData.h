// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
// =============================================================================
// AdvancedIMUData.h
// =============================================================================
// Internal/debug output structure for the advanced inertial model.
// This provides Cbn, llh, velnav, wibb, fibb and debug intermediate values
// without modifying the existing CARLA IMU serialization format.
//
// For client-side access these values can be retrieved via:
//   1. The debug log (when EnableAdvancedIMUDebugLog is true)
//   2. Direct C++ access on the sensor actor (GetAdvancedIMUData())
//   3. Future: extended serializer (not yet implemented)
// =============================================================================

#pragma once

#include "Carla/Sensor/NavigationMath.h"

namespace carla {
namespace sensor {
namespace nav {

/// Debug snapshot of all intermediate values from one tick of the advanced
/// inertial model.  Written when bEnableAdvancedIMUDebugLog is true.
struct AdvancedIMUDebugData
{
  // ---- Raw inputs (after unit conversion to meters / rad/s) ----
  DVector3 RawPosEsu;        ///< CARLA world position [m] in ESU
  DVector3 RawVelEsu;        ///< CARLA world velocity [m/s] in ESU
  DVector3 RawAccEsu;        ///< CARLA world acceleration [m/s²] in ESU
  DVector3 RawAngVelEsu;     ///< CARLA angular velocity [rad/s] in ESU (LH)
  double   RawPitch;         ///< CARLA pitch [deg]
  double   RawYaw;           ///< CARLA yaw [deg]
  double   RawRoll;          ///< CARLA roll [deg]

  // ---- ENU intermediate values ----
  DVector3 PosEnu;           ///< Position in ENU [m]
  DVector3 VelEnu;           ///< Velocity in ENU [m/s]
  DVector3 AccEnu;           ///< Acceleration in ENU [m/s²]
  DVector3 AngVelFe;         ///< Angular velocity in ENU (RH) [rad/s]

  // ---- NED intermediate values ----
  DVector3 PosNed;           ///< Position in NED [m]

  // ---- Final outputs ----
  AdvancedInertialData Output;

  // ---- Tick metadata ----
  double   SimTime;          ///< CARLA simulation time [s]
  float    DeltaTime;        ///< Physics tick delta [s]
};

} // namespace nav
} // namespace sensor
} // namespace carla
