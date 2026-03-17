// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
// =============================================================================
// InertialMeasurementUnit.h
// =============================================================================
// CARLA IMU sensor with two operating modes:
//
//   1. Legacy mode (default):  Identical to original CARLA behaviour.
//      - Accelerometer from finite-difference + flat-Earth gravity
//      - Gyroscope from UE physics angular velocity
//      - Compass from forward-vector dot product
//
//   2. Advanced inertial model (enable_advanced_inertial_model = true):
//      - Full Earth-aware model matching the MATLAB carla2nav() pipeline.
//      - Outputs wibb (omega_ib^b) and fibb (f_ib^b) in body(FRD) frame,
//        with WGS-84 gravity, Earth rotation, Coriolis, and transport-rate
//        terms.
//      - Also computes Cbn, llh, velnav as internal/debug outputs.
//      - Existing accelerometer/gyroscope/compass fields are filled with the
//        advanced values (wibb for gyroscope, fibb for accelerometer), so the
//        serialization format does not change.
//
// See NavigationMath.h for the complete math documentation.
// =============================================================================

#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/NavigationMath.h"
#include "Carla/Sensor/AdvancedIMUData.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/Vector3D.h"
#include <util/enable-ue4-macros.h>

#include <array>
#include <deque>

#include "InertialMeasurementUnit.generated.h"

UCLASS()
class CARLA_API AInertialMeasurementUnit : public ASensor
{
  GENERATED_BODY()

public:

  AInertialMeasurementUnit(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription) override;

  void SetOwner(AActor *Owner) override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  const carla::geom::Vector3D ComputeAccelerometerNoise(
      const FVector &Accelerometer);

  const carla::geom::Vector3D ComputeGyroscopeNoise(
      const FVector &Gyroscope);

  /// Legacy accelerometer: measures linear acceleration in m/s^2
  carla::geom::Vector3D ComputeAccelerometer(const float DeltaTime);

  /// Legacy gyroscope: measures angular velocity in rad/sec
  carla::geom::Vector3D ComputeGyroscope();

  /// Legacy magnetometer: orientation with respect to the North in rad
  float ComputeCompass();

  void SetAccelerationStandardDeviation(const FVector &Vec);

  void SetGyroscopeStandardDeviation(const FVector &Vec);

  void SetGyroscopeBias(const FVector &Vec);

  void SetAccelerometerBias(const FVector &Vec);

  const FVector &GetAccelerationStandardDeviation() const;

  const FVector &GetGyroscopeStandardDeviation() const;

  const FVector &GetGyroscopeBias() const;

  const FVector &GetAccelerometerBias() const;

  double GetLatitudeRad() const { return LatitudeRad; }

  double GetLongitudeRad() const { return LongitudeRad; }

  double GetAltitudeMeters() const { return AltitudeMeters; }

  /// Returns the geodetic anchor (initial) latitude.
  double GetOriginLatitudeRad() const { return OriginLatitudeRad; }

  /// Returns the geodetic anchor (initial) longitude.
  double GetOriginLongitudeRad() const { return OriginLongitudeRad; }

  /// Returns the geodetic anchor (initial) altitude.
  double GetOriginAltitudeMeters() const { return OriginAltitudeMeters; }

  const carla::geom::Vector3D& GetAccelerometerValue() const;

  const carla::geom::Vector3D& GetGyroscopeValue() const;

  float GetCompassValue() const;

  /// Based on OpenDRIVE's lon and lat, North is in (0.0f, -1.0f, 0.0f)
  static const FVector CarlaNorthVector;

  // ═══════════════════════════════════════════════════════════════════════════
  // Advanced inertial model — public API
  // ═══════════════════════════════════════════════════════════════════════════

  /// Enable / disable the advanced Earth-aware inertial model.
  void SetAdvancedInertialModelEnabled(bool bEnabled);
  bool IsAdvancedInertialModelEnabled() const { return bEnableAdvancedInertialModel; }

  /// Set the geodetic reference center for the local-tangent frame.
  /// @param LatDeg  Latitude [deg]
  /// @param LonDeg  Longitude [deg]
  /// @param AltM    Altitude above WGS-84 ellipsoid [m]
  void SetCenterLlh(double LatDeg, double LonDeg, double AltM);

  /// Enable / disable verbose debug logging of all intermediate values.
  void SetAdvancedIMUDebugLog(bool bEnabled);
  bool IsAdvancedIMUDebugLogEnabled() const { return bEnableAdvancedIMUDebugLog; }

  /// Retrieve the last-computed advanced inertial data.
  /// Only valid when advanced mode is enabled.
  const carla::sensor::nav::AdvancedInertialData& GetAdvancedIMUData() const
  {
    return AdvancedData;
  }

  /// Retrieve the last debug snapshot (if debug logging is enabled).
  const carla::sensor::nav::AdvancedIMUDebugData& GetAdvancedIMUDebugData() const
  {
    return AdvancedDebug;
  }

  /// Get the geodetic reference center [deg, deg, m].
  void GetCenterLlh(double &OutLatDeg, double &OutLonDeg, double &OutAltM) const;

private:

  void BeginPlay() override;

  // ---- Advanced inertial model helpers (called from PostPhysTick) ----

  /// Gather raw CARLA/UE data and run the advanced inertial model.
  /// Populates AccelerometerValue, GyroscopeValue, CompassValue from
  /// the advanced pipeline (fibb, wibb, Cbn heading).
  void TickAdvancedInertialModel(float DeltaTime);

  /// Retrieve world-frame acceleration [cm/s²] from the physics engine.
  /// If the vehicle has a physics body, this computes finite-difference
  /// velocity or reads physics directly; otherwise returns zero.
  ///
  /// Source: UPrimitiveComponent::GetPhysicsLinearVelocity() differentiated
  /// over DeltaTime.  Documented because CARLA/UE does not expose a direct
  /// world-frame acceleration API for all actor types.
  FVector GetWorldAcceleration(float DeltaTime);

  /// Standard deviation for acceleration settings.
  FVector StdDevAccel = FVector::ZeroVector;

  /// Standard deviation for gyroscope settings.
  FVector StdDevGyro = FVector::ZeroVector;

  /// Bias for gyroscope settings.
  FVector BiasGyro = FVector::ZeroVector;

  /// Bias for accelerometer settings.
  FVector BiasAccel = FVector::ZeroVector;

  /// IMU reference latitude [rad].
  double LatitudeRad = 0.0;

  /// IMU reference longitude [rad].
  double LongitudeRad = 0.0;

  /// IMU reference altitude [m].
  double AltitudeMeters = 0.0;

  /// Anchor geodetic latitude [rad] (from configuration).
  double OriginLatitudeRad = 0.0;

  /// Anchor geodetic longitude [rad] (from configuration).
  double OriginLongitudeRad = 0.0;

  /// Anchor geodetic altitude [m] (from configuration).
  double OriginAltitudeMeters = 0.0;

  /// Anchor world position in meters (ESU frame).
  FVector OriginWorldPositionMeters = FVector::ZeroVector;

  /// Whether origin world position has been captured.
  bool bHasOriginWorld = false;

  /// Used to compute the acceleration (legacy mode + advanced velocity diff)
  std::array<FVector, 2> PrevLocation;

  /// Previous-tick world velocity [cm/s] for finite-difference acceleration
  FVector PrevWorldVelocity = FVector::ZeroVector;
  bool bHasPrevWorldVelocity = false;

  /// Used to compute the acceleration
  float PrevDeltaTime;

  /// Warmup tick counter for the finite-difference accelerometer.
  /// During the first ticks after spawn, PrevLocation history is unreliable
  /// because BeginPlay may fire before the actor reaches its final world
  /// position (e.g., when spawned with attach_to).  During warmup we
  /// re-seed PrevLocation every tick and return gravity-only output.
  /// See issue #8970.
  int AccelWarmupTicksRemaining = 2;

  /// Accelerometer value calculated after each PostPhysTick
  carla::geom::Vector3D AccelerometerValue;

  /// Gyroscope value calculated after each PostPhysTick
  carla::geom::Vector3D GyroscopeValue;

  /// Magnetometer value calculated after each PostPhysTick
  float CompassValue;

  // ═══════════════════════════════════════════════════════════════════════════
  // Advanced inertial model — configuration & state
  // ═══════════════════════════════════════════════════════════════════════════

  /// When true, the advanced Earth-aware model replaces the legacy model.
  /// Default: false (legacy mode).  Set via blueprint attribute
  /// "enable_advanced_inertial_model" or SetAdvancedInertialModelEnabled().
  bool bEnableAdvancedInertialModel = false;

  /// When true, a detailed debug log is emitted every tick.
  bool bEnableAdvancedIMUDebugLog = false;

  /// Geodetic reference center [deg, deg, m].
  /// Default: Seoul KAIST campus (approximate).
  /// Must be set to the CARLA map origin's real-world geodetic position.
  double CenterLatDeg  = 36.372;    // [deg] latitude
  double CenterLonDeg  = 127.363;   // [deg] longitude
  double CenterAltM    = 70.0;      // [m]   height above ellipsoid

  /// Latest advanced inertial output.
  carla::sensor::nav::AdvancedInertialData AdvancedData;

  /// Latest debug snapshot.
  carla::sensor::nav::AdvancedIMUDebugData AdvancedDebug;
};
