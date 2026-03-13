// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
// =============================================================================
// InertialMeasurementUnit.cpp
// =============================================================================
// Two-mode IMU sensor:
//   - Legacy mode: original CARLA flat-Earth approximation.
//   - Advanced mode: Earth-aware model reproducing MATLAB carla2nav().
//
// Output frame (both modes): FLU (Forward-Left-Up), matching ROS REP-103.
//
// Raw data sources (CARLA / UE):
//   - World position:      AActor::GetActorLocation()            [cm, ESU*]
//   - World velocity:      UPrimitiveComponent::GetPhysicsLinearVelocity()
//                                                                [cm/s, UE world]
//   - World acceleration:  2nd-order finite-difference of world position
//                                                                [cm/s², UE world]
//   - Angular velocity:    UPrimitiveComponent::GetPhysicsAngularVelocityInRadians()
//                          in world frame                         [rad/s, UE world]
//   - Orientation (PYR):   FRotator from GetActorRotation()      [deg]
//
// (*) CARLA world frame convention: +X = East, -Y = North, +Z = Up (left-handed UE).
//     We label this "ESU" (East-South-Up) before converting to right-handed ENU.
//
// Sensor mounting / lever-arm:
//   Both modes compute quantities at the sensor actor's own origin (the IMU
//   mount point).  If the IMU is attached to a vehicle with a non-zero
//   relative transform, the world position/velocity/acceleration from the
//   sensor actor already include the mount offset.
//   Explicit lever-arm correction (ω × r terms) is NOT applied because the
//   UE physics system already resolves quantities at the component origin.
//   If sub-centimetre lever-arm accuracy is required in the future, add a
//   correction term using the relative transform between the vehicle body
//   origin and the sensor actor origin.
// =============================================================================

#include "Carla/Sensor/InertialMeasurementUnit.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/WorldObserver.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Math/UnrealMathUtility.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include <util/enable-ue4-macros.h>

#include <limits>
#include <cmath>

// Based on OpenDRIVE's lon and lat
const FVector AInertialMeasurementUnit::CarlaNorthVector =
    FVector(0.0f, -1.0f, 0.0f);

AInertialMeasurementUnit::AInertialMeasurementUnit(
    const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PostPhysics;
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  PrevLocation = { FVector::ZeroVector, FVector::ZeroVector };
  // Initialized to something hight to minimize the artifacts
  // when the initial values are unknown
  PrevDeltaTime = std::numeric_limits<float>::max();
}

FActorDefinition AInertialMeasurementUnit::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeIMUDefinition();
}

void AInertialMeasurementUnit::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetIMU(ActorDescription, this);
}

void AInertialMeasurementUnit::SetOwner(AActor* OwningActor)
{
  Super::SetOwner(OwningActor);
}

// Returns the angular velocity of Actor, expressed in the actor's body frame
// (UE convention: X=Forward, Y=Right, Z=Up, i.e. FRU).
static FVector FIMU_GetActorAngularVelocityInRadians(
    AActor &Actor)
{
  const auto RootComponent = Cast<UPrimitiveComponent>(Actor.GetRootComponent());

  FVector AngularVelocity;

  if (RootComponent != nullptr) {
      const FQuat ActorGlobalRotation = RootComponent->GetComponentTransform().GetRotation();
      const FVector GlobalAngularVelocity = RootComponent->GetPhysicsAngularVelocityInRadians();
      AngularVelocity = ActorGlobalRotation.UnrotateVector(GlobalAngularVelocity);
  } else {
      AngularVelocity = FVector::ZeroVector;
  }

  return AngularVelocity;
}

const carla::geom::Vector3D AInertialMeasurementUnit::ComputeAccelerometerNoise(
    const FVector &Accelerometer)
{
  // Additive Gaussian noise: bias + N(0, stddev), parameters set by client
  constexpr float Mean = 0.0f;
  return carla::geom::Vector3D
  {
      (float)(Accelerometer.X + BiasAccel.X + RandomEngine->GetNormalDistribution(Mean, StdDevAccel.X)),
      (float)(Accelerometer.Y + BiasAccel.Y + RandomEngine->GetNormalDistribution(Mean, StdDevAccel.Y)),
      (float)(Accelerometer.Z + BiasAccel.Z + RandomEngine->GetNormalDistribution(Mean, StdDevAccel.Z))
  };
}

const carla::geom::Vector3D AInertialMeasurementUnit::ComputeGyroscopeNoise(
    const FVector &Gyroscope)
{
  // Additive Gaussian noise: bias + N(0, stddev), parameters set by client
  constexpr float Mean = 0.0f;
  return carla::geom::Vector3D
  {
      (float)(Gyroscope.X + BiasGyro.X + RandomEngine->GetNormalDistribution(Mean, StdDevGyro.X)),
      (float)(Gyroscope.Y + BiasGyro.Y + RandomEngine->GetNormalDistribution(Mean, StdDevGyro.Y)),
      (float)(Gyroscope.Z + BiasGyro.Z + RandomEngine->GetNormalDistribution(Mean, StdDevGyro.Z))
  };
}

carla::geom::Vector3D AInertialMeasurementUnit::ComputeAccelerometer(
    const float DeltaTime)
{
  // Used to convert from UE4's cm to meters
  constexpr float TO_METERS = 1e-2;
  // Earth's gravitational acceleration is approximately 9.81 m/s^2
  constexpr float GRAVITY = 9.81f;

  // 2nd derivative of the polynomic (quadratic) interpolation
  // using the point in current time and two previous steps:
  // d2[i] = -2.0*(y1/(h1*h2)-y2/((h2+h1)*h2)-y0/(h1*(h2+h1)))
  const FVector CurrentLocation = GetActorLocation();

  const FVector Y2 = PrevLocation[0];
  const FVector Y1 = PrevLocation[1];
  const FVector Y0 = CurrentLocation;
  const float H1 = DeltaTime;
  const float H2 = PrevDeltaTime;

  const float H1AndH2 = H2 + H1;
  const FVector A = Y1 / ( H1 * H2 );
  const FVector B = Y2 / ( H2 * (H1AndH2) );
  const FVector C = Y0 / ( H1 * (H1AndH2) );
  FVector FVectorAccelerometer = TO_METERS * -2.0f * ( A - B - C );

  // Update the previous locations
  PrevLocation[0] = PrevLocation[1];
  PrevLocation[1] = CurrentLocation;
  PrevDeltaTime = DeltaTime;

  // Add gravitational acceleration
  FVectorAccelerometer.Z += GRAVITY;

  // World ESU → sensor body FRU via world rotation (GetComponentTransform)
  FQuat ImuRotation =
      GetRootComponent()->GetComponentTransform().GetRotation();
  FVectorAccelerometer = ImuRotation.UnrotateVector(FVectorAccelerometer);

  // Sensor body FRU → FLU: negate Y (Right → Left)
  FVectorAccelerometer.Y = -FVectorAccelerometer.Y;

  // Apply noise and return as Vector3D [m/s²] in FLU
  const carla::geom::Vector3D Accelerometer =
      ComputeAccelerometerNoise(FVectorAccelerometer);

  return Accelerometer;
}

carla::geom::Vector3D AInertialMeasurementUnit::ComputeGyroscope()
{
  check(GetOwner() != nullptr);
  // Owner body-frame angular velocity [rad/s] in FRU
  const FVector AngularVelocity =
      FIMU_GetActorAngularVelocityInRadians(*GetOwner());

  // Owner body FRU → sensor body FRU via relative mount rotation
  const FQuat SensorLocalRotation =
      RootComponent->GetRelativeTransform().GetRotation();

  FVector FVectorGyroscope =
      SensorLocalRotation.RotateVector(AngularVelocity);

  // Sensor body FRU → FLU for angular velocity (pseudo-vector).
  // Under a reflection that flips Y (det = -1), a pseudo-vector
  // transforms as: v' = -R * v, so X and Z are negated (not Y).
  FVectorGyroscope.X = -FVectorGyroscope.X;
  FVectorGyroscope.Z = -FVectorGyroscope.Z;

  // Apply noise and return as Vector3D [rad/s] in FLU
  const carla::geom::Vector3D Gyroscope =
      ComputeGyroscopeNoise(FVectorGyroscope);

  return Gyroscope;
}

float AInertialMeasurementUnit::ComputeCompass()
{
  // Magnetometer: orientation with respect to the North in rad
  const FVector ForwVect = GetActorForwardVector().GetSafeNormal2D();
  const float DotProd = FVector::DotProduct(CarlaNorthVector, ForwVect);

  // We check if the dot product is higher than 1.0 due to numerical error
  if (DotProd >= 1.00f)
    return 0.0f;

  const float Compass = std::acos(DotProd);
  // Keep the angle between [0, 2pi)
  if (FVector::CrossProduct(CarlaNorthVector, ForwVect).Z < 0.0f)
    return carla::geom::Math::Pi2<float>() - Compass;

  return Compass;
}

void AInertialMeasurementUnit::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInertialMeasurementUnit::PostPhysTick);

  if (bEnableAdvancedInertialModel)
  {
    // ---- Advanced Earth-aware inertial model ----
    TickAdvancedInertialModel(DeltaTime);
  }
  else
  {
    // ---- Legacy flat-Earth model ----
    AccelerometerValue = ComputeAccelerometer(DeltaTime);
    GyroscopeValue = ComputeGyroscope();
    CompassValue = ComputeCompass();
  }

  auto DataStream = GetDataStream(*this);

  // ROS2
  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromIMU(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, AccelerometerValue, GyroscopeValue, CompassValue, this);
    }
    else
    {
      ROS2->ProcessDataFromIMU(DataStream.GetSensorType(), StreamId, DataStream.GetSensorTransform(), AccelerometerValue, GyroscopeValue, CompassValue, this);
    }
  }
  #endif

  {
    TRACE_CPUPROFILER_EVENT_SCOPE(AInertialMeasurementUnit::SerializeAndSend);
    DataStream.SerializeAndSend(*this, AccelerometerValue, GyroscopeValue, CompassValue);
  }
}

void AInertialMeasurementUnit::SetAccelerationStandardDeviation(const FVector &Vec)
{
  StdDevAccel = Vec;
}

void AInertialMeasurementUnit::SetGyroscopeStandardDeviation(const FVector &Vec)
{
  StdDevGyro = Vec;
}

void AInertialMeasurementUnit::SetGyroscopeBias(const FVector &Vec)
{
  BiasGyro = Vec;
}

void AInertialMeasurementUnit::SetAccelerometerBias(const FVector &Vec)
{
  BiasAccel = Vec;
}

const FVector &AInertialMeasurementUnit::GetAccelerationStandardDeviation() const
{
  return StdDevAccel;
}

const FVector &AInertialMeasurementUnit::GetGyroscopeStandardDeviation() const
{
  return StdDevGyro;
}

const FVector &AInertialMeasurementUnit::GetGyroscopeBias() const
{
  return BiasGyro;
}

const FVector &AInertialMeasurementUnit::GetAccelerometerBias() const
{
  return BiasAccel;
}

const carla::geom::Vector3D& AInertialMeasurementUnit::GetAccelerometerValue() const
{
  return AccelerometerValue;
}

const carla::geom::Vector3D& AInertialMeasurementUnit::GetGyroscopeValue() const
{
  return GyroscopeValue;
}

float AInertialMeasurementUnit::GetCompassValue() const
{
  return CompassValue;
}

void AInertialMeasurementUnit::BeginPlay()
{
  Super::BeginPlay();
}

// =============================================================================
// Advanced inertial model — configuration setters
// =============================================================================

void AInertialMeasurementUnit::SetAdvancedInertialModelEnabled(bool bEnabled)
{
  bEnableAdvancedInertialModel = bEnabled;
  if (bEnabled)
  {
    UE_LOG(LogCarla, Log,
      TEXT("IMU [%s]: Advanced inertial model ENABLED.  Center LLH = (%.6f, %.6f, %.1f)"),
      *GetName(), CenterLatDeg, CenterLonDeg, CenterAltM);
  }
  else
  {
    UE_LOG(LogCarla, Log,
      TEXT("IMU [%s]: Advanced inertial model DISABLED (legacy mode)."),
      *GetName());
  }
}

void AInertialMeasurementUnit::SetCenterLlh(double LatDeg, double LonDeg, double AltM)
{
  CenterLatDeg = LatDeg;
  CenterLonDeg = LonDeg;
  CenterAltM   = AltM;
  UE_LOG(LogCarla, Log,
    TEXT("IMU [%s]: Center LLH set to (%.6f deg, %.6f deg, %.1f m)"),
    *GetName(), CenterLatDeg, CenterLonDeg, CenterAltM);
}

void AInertialMeasurementUnit::GetCenterLlh(double &OutLatDeg, double &OutLonDeg, double &OutAltM) const
{
  OutLatDeg = CenterLatDeg;
  OutLonDeg = CenterLonDeg;
  OutAltM   = CenterAltM;
}

void AInertialMeasurementUnit::SetAdvancedIMUDebugLog(bool bEnabled)
{
  bEnableAdvancedIMUDebugLog = bEnabled;
}

// =============================================================================
// Advanced inertial model — world acceleration helper
// =============================================================================

FVector AInertialMeasurementUnit::GetWorldAcceleration(float DeltaTime)
{
  // Source: 2nd-derivative of position via quadratic (3-point) polynomial
  // interpolation — same numerical scheme as legacy ComputeAccelerometer(),
  // but WITHOUT gravity addition and WITHOUT body-frame rotation.
  //
  // Formula:
  //   d2[i] = -2.0 * ( y1/(h1*h2) - y2/((h2+h1)*h2) - y0/(h1*(h2+h1)) )
  //
  // Returns world-frame acceleration in [cm/s²] (ESU).
  // The caller is responsible for cm→m conversion.

  const FVector CurrentLocation = GetActorLocation();  // [cm] world (ESU)

  const FVector Y2 = PrevLocation[0];
  const FVector Y1 = PrevLocation[1];
  const FVector Y0 = CurrentLocation;
  const float H1 = DeltaTime;
  const float H2 = PrevDeltaTime;

  FVector Acc = FVector::ZeroVector;

  const float H1AndH2 = H2 + H1;
  if (H1 > 1e-8f && H2 > 1e-8f && H1AndH2 > 1e-8f)
  {
    const FVector A = Y1 / (H1 * H2);
    const FVector B = Y2 / (H2 * H1AndH2);
    const FVector C = Y0 / (H1 * H1AndH2);
    Acc = -2.0f * (A - B - C);  // [cm/s²] world frame (ESU)
  }

  // Update history — shared with legacy ComputeAccelerometer().
  // NOTE: PrevLocation[] and PrevDeltaTime are already updated by
  // legacy ComputeAccelerometer() when in legacy mode.  In advanced
  // mode, we update them here instead.
  PrevLocation[0] = PrevLocation[1];
  PrevLocation[1] = CurrentLocation;
  PrevDeltaTime = DeltaTime;

  return Acc;  // [cm/s²] world (ESU), no gravity, no body rotation
}

// =============================================================================
// Advanced inertial model — main tick
// =============================================================================

void AInertialMeasurementUnit::TickAdvancedInertialModel(float DeltaTime)
{
  using namespace carla::sensor::nav;

  // ---- 1. Gather raw CARLA/UE data ----
  // All positions / velocities are in UE world frame [cm] or [cm/s].
  // UE world frame is treated as ESU (+X=East, -Y=North, +Z=Up) per CARLA convention.

  // Position of the sensor actor in world [cm]
  const FVector UePos = GetActorLocation();

  // World-frame velocity [cm/s]  — from physics body of the *owner* (vehicle).
  // We use the owner's root component because the sensor itself may not have
  // its own rigid body.
  FVector UeVel = FVector::ZeroVector;
  {
    const AActor* Owner = GetOwner();
    if (Owner)
      UeVel = Owner->GetVelocity();  // [cm/s] world (ESU), at vehicle origin (not sensor mount)
  }

  // World-frame acceleration [cm/s²] — finite-difference of velocity.
  const FVector UeAcc = GetWorldAcceleration(DeltaTime);

  // World-frame angular velocity [rad/s] — from physics body.
  // NOTE: UE returns angular velocity in the WORLD frame (not body).
  FVector UeAngVelWorld = FVector::ZeroVector;
  {
    const AActor* Owner = GetOwner();
    if (Owner)
    {
      const auto RootComp = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
      if (RootComp)
        UeAngVelWorld = RootComp->GetPhysicsAngularVelocityInRadians();  // [rad/s] world
    }
  }

  // Actor orientation
  const FRotator UeRot = GetOwner() ? GetOwner()->GetActorRotation() : GetActorRotation();
  const double PitchDeg = static_cast<double>(UeRot.Pitch);
  const double YawDeg   = static_cast<double>(UeRot.Yaw);
  const double RollDeg  = static_cast<double>(UeRot.Roll);

  // ---- 2. Convert UE FVector to DVector3 (ESU) ----
  // Position [cm]
  const DVector3 PosEsu(UePos.X, UePos.Y, UePos.Z);

  // Velocity [cm/s]
  const DVector3 VelEsu(UeVel.X, UeVel.Y, UeVel.Z);

  // Acceleration [cm/s²]
  const DVector3 AccEsu(UeAcc.X, UeAcc.Y, UeAcc.Z);

  // Angular velocity [rad/s] world ESU
  const DVector3 AngVelEsuRad(
    UeAngVelWorld.X,
    UeAngVelWorld.Y,
    UeAngVelWorld.Z
  );

  // ---- 3. Run the advanced inertial model ----
  const AdvancedInertialData AdvancedIMU = ComputeAdvancedInertialModel(
    PosEsu, VelEsu, AccEsu, AngVelEsuRad,
    PitchDeg, YawDeg, RollDeg,
    CenterLatDeg, CenterLonDeg, CenterAltM);

  // ---- 4. Output: body(FRD) → body(FRU) → sensor(FRU) → sensor(FLU) ----
  // Apply sensor mount rotation: body frame → sensor local frame
  const FQuat SensorLocalRotation =
      RootComponent->GetRelativeTransform().GetRotation();

  // AccelerometerValue ← fibb (specific force) [m/s²], body FRD
  FVector FibbBody(
    static_cast<float>(AdvancedIMU.Fibb.X),
    static_cast<float>(AdvancedIMU.Fibb.Y),
    static_cast<float>(AdvancedIMU.Fibb.Z));
  // Body FRD → Body FRU (negate Z) so UE quaternion operates in its native frame
  FibbBody.Z = -FibbBody.Z;
  // Body FRU → Sensor mount frame (FRU)
  FVector FibbSensor = SensorLocalRotation.RotateVector(FibbBody);
  // Sensor FRU → Sensor FLU: negate Y
  FibbSensor.Y = -FibbSensor.Y;
  AccelerometerValue = ComputeAccelerometerNoise(FibbSensor);

  // GyroscopeValue ← wibb (true gyro) [rad/s], body FRD
  FVector WibbBody(
    static_cast<float>(AdvancedIMU.Wibb.X),
    static_cast<float>(AdvancedIMU.Wibb.Y),
    static_cast<float>(AdvancedIMU.Wibb.Z));
  // Body FRD → Body FRU (negate Z) so UE quaternion operates in its native frame
  WibbBody.Z = -WibbBody.Z;
  // Body FRU → Sensor mount frame (FRU)
  FVector WibbSensor = SensorLocalRotation.RotateVector(WibbBody);
  // Sensor FRU → Sensor FLU: negate Y
  WibbSensor.Y = -WibbSensor.Y;
  GyroscopeValue = ComputeGyroscopeNoise(WibbSensor);

  // CompassValue ← legacy compass
  CompassValue = ComputeCompass();

}
