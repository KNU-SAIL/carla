// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
// =============================================================================
// NavigationMath.h
// =============================================================================
// Helper functions for the advanced Earth-aware inertial measurement model.
//
// This module faithfully reproduces the MATLAB carla2nav() post-processing
// pipeline inside the UE runtime, using double precision throughout.
//
// -------  Frame convention summary  -------
//
//   Name            Axes                       Handedness
//   ----            ----                       ----------
//   CARLA world     +X = East, -Y = North,     Left-handed (UE)
//   (ESU)           +Z = Up
//
//   ENU             +X = East, +Y = North,     Right-handed
//                   +Z = Up
//
//   NED             +X = North, +Y = East,     Right-handed
//                   +Z = Down
//
//   Body (FRD)      +X = Forward, +Y = Right,  Right-handed
//                   +Z = Down
//
//   Body (FRU/UE)   +X = Forward, +Y = Right,  Left-handed (UE body)
//                   +Z = Up
// -------  DCM naming convention  ------
//   Cab  =  "attitude of frame b with respect to frame a"
//           i.e. transforms a vector FROM b TO a.
//           v_a = Cab * v_b
//
// -------  Key DCMs  -------
//   Cnb  : body(FRD) w.r.t. NED   →  v_n = Cnb * v_b
//   Cne  :  ECEF w.r.t. NED       →  v_n = Cne * v_e
//   Ce2fn : local NED at center w.r.t. ECEF  →  v_e = Ce2fn * v_fn
// 
// ------- Notations -------
//   fe: local-tangent ENU (fixed)
// =============================================================================

#pragma once

#include <cmath>
#include <array>

namespace carla {
namespace sensor {
namespace nav {

// ═══════════════════════════════════════════════════════════════════════════════
// Type aliases – double-precision 3-vectors and 3×3 matrices (row-major)
// ═══════════════════════════════════════════════════════════════════════════════

/// 3-element double vector
struct DVector3
{
  double X = 0.0; double Y = 0.0; double Z = 0.0;

  DVector3() = default;
  DVector3(double InX, double InY, double InZ) : X(InX), Y(InY), Z(InZ) {}

  DVector3 operator+(const DVector3 &B) const { return {X + B.X, Y + B.Y, Z + B.Z}; }
  DVector3 operator-(const DVector3 &B) const { return {X - B.X, Y - B.Y, Z - B.Z}; }
  DVector3 operator*(double S) const { return {X * S, Y * S, Z * S}; }
  DVector3& operator+=(const DVector3 &B) { X += B.X; Y += B.Y; Z += B.Z; return *this; }
  DVector3& operator-=(const DVector3 &B) { X -= B.X; Y -= B.Y; Z -= B.Z; return *this; }
  double Norm() const { return std::sqrt(X * X + Y * Y + Z * Z); }
};

inline DVector3 operator*(double S, const DVector3 &V) { return V * S; }

/// 3×3 double matrix, stored row-major: M[row][col]
struct DMatrix3
{
  double M[3][3] = {};

  DMatrix3() = default;

  /// Construct from rows
  DMatrix3(const DVector3 &Row0, const DVector3 &Row1, const DVector3 &Row2)
  {
    M[0][0] = Row0.X; M[0][1] = Row0.Y; M[0][2] = Row0.Z;
    M[1][0] = Row1.X; M[1][1] = Row1.Y; M[1][2] = Row1.Z;
    M[2][0] = Row2.X; M[2][1] = Row2.Y; M[2][2] = Row2.Z;
  }

  /// Matrix × vector
  DVector3 operator*(const DVector3 &V) const
  {
    return {
      M[0][0]*V.X + M[0][1]*V.Y + M[0][2]*V.Z,
      M[1][0]*V.X + M[1][1]*V.Y + M[1][2]*V.Z,
      M[2][0]*V.X + M[2][1]*V.Y + M[2][2]*V.Z
    };
  }

  /// Matrix × matrix
  DMatrix3 operator*(const DMatrix3 &B) const
  {
    DMatrix3 R;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
      {
        R.M[i][j] = 0.0;
        for (int k = 0; k < 3; ++k)
          R.M[i][j] += M[i][k] * B.M[k][j];
      }
    return R;
  }

  /// Transpose
  DMatrix3 T() const
  {
    DMatrix3 R;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        R.M[i][j] = M[j][i];
    return R;
  }

  static DMatrix3 Identity()
  {
    DMatrix3 I;
    I.M[0][0] = 1.0; I.M[1][1] = 1.0; I.M[2][2] = 1.0;
    return I;
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// WGS-84 Gravity Model output
// ═══════════════════════════════════════════════════════════════════════════════

struct Wgs84Params
{
  double R0;   ///< Geometric mean radius  √(RN*RE)
  double g;    ///< Local gravity [m/s²]
  double ER;   ///< Earth rotation rate [rad/s]
  double RN;   ///< Meridional radius of curvature
  double RE;   ///< Transverse radius of curvature
};

// ═══════════════════════════════════════════════════════════════════════════════
// Advanced Inertial Model output structure (double precision)
// ═══════════════════════════════════════════════════════════════════════════════

struct AdvancedInertialData
{
  DMatrix3 Cnb;        ///< Attitude of body(FRD) w.r.t. NED
  DVector3 Wibb;       ///< Angular rate [rad/s] (body-resolved)
  DVector3 Fibb;       ///< Specific force [m/s²] (body-resolved)
  DVector3 Llh;        ///< Geodetic position [rad, rad, m]
  DVector3 VelNav;     ///< Velocity w.r.t. Earth resolved in NED [m/s]
};

// ═══════════════════════════════════════════════════════════════════════════════
// Helper function declarations
// ═══════════════════════════════════════════════════════════════════════════════

// ---- Coordinate transforms --------------------------------------------------

/// Convert a vector from CARLA world ESU to right-handed ENU.
///   ESU (+X=East, -Y=North, +Z=Up) → ENU (+X=East, +Y=North, +Z=Up)
DVector3 ConvertEsuToEnu(const DVector3 &VecEsu);

/// Convert angular velocity from CARLA's left-handed rotation convention
/// about ESU axes to right-handed rotation about ENU axes.
DVector3 ConvertAngularVelocityLhsToRhs(const DVector3 &AngVelEsu);

/// Transform a vector from ENU to NED.
///   Cenu2ned = [0 1 0; 1 0 0; 0 0 -1]
DVector3 ConvertEnuToNed(const DVector3 &VecEnu);

/// Return the 3×3 DCM Cenu2ned.
DMatrix3 BuildCenu2ned();

// ---- Attitude matrices ------------------------------------------------------

/// DCM of NED w.r.t. ECEF.
///   v_e = Cen * v_n
DMatrix3 BuildCen(double LatRad, double LonRad);

/// Build the DCM Cuelocal2uebody from CARLA PYR angles (in degrees).
///   Reproduces the MATLAB Rz*Ry*Rx construction with sign flips:
///     Roll  → cos/sin(-Roll)  (right-handed about X)
///     Pitch → cos/sin(-Pitch) (right-handed about Y)
///     Yaw   → cos/sin(Yaw)   (left-handed about Z)
DMatrix3 BuildCuelocal2uebodyFromPYR(double PitchDeg, double YawDeg, double RollDeg);

/// Constant DCM: body(FRD) w.r.t. body(FRU) = diag([1 1 -1])
DMatrix3 BuildCbue2b();

/// Constant DCM: local-tangent UE (ESU) w.r.t. local-tangent ENU
///   = diag([1 -1 1])
DMatrix3 BuildCfe2uelocal();

/// Compute the full body-to-nav DCM:
///   Cnb = Cn2fe * Cfe2uelocal * Cuelocal2uebody * Cbue2b
/// where Cn2fe = Cne * Ce2fn * Cned2enu
DMatrix3 BuildCnb(const DVector3 &LlhRad,
                  const DVector3 &CenterLlhRad,
                  const DMatrix3 &Cuelocal2uebody);

/// Extract the composite Cfe2b = Cfe2uelocal * Cuelocal2uebody * Cbue2b
DMatrix3 BuildCfe2b(const DMatrix3 &Cuelocal2uebody);

// ---- Geodesy ----------------------------------------------------------------

/// WGS-84 gravity model.  Matches wgs84GravityModel.m exactly.
Wgs84Params ComputeWgs84Gravity(const DVector3 &LlhRad);

/// Convert NED position (relative to center) to geodetic LLH.
///   Matches ned2llh.m → llh2xyz → xyz2llh round-trip.
DVector3 NedToLlh(const DVector3 &NedPos, const DVector3 &CenterLlhDeg);

/// Convert geodetic LLH [rad, rad, m] to ECEF XYZ [m].
DVector3 Llh2Xyz(const DVector3 &LlhRad);

/// Convert ECEF XYZ [m] to geodetic LLH [rad, rad, m].
DVector3 Xyz2Llh(const DVector3 &Xyz);

// ---- Earth rotation / transport rate ----------------------------------------

/// Earth rotation vector in local ENU frame.
///   w_iefe = ER * [0; cos(lat); sin(lat)]
DVector3 ComputeEarthRateEnu(double LatRad, double EarthRate);

/// Transport-rate vector in NED frame.
///   wenn = [vE/(RE+h); -vN/(RN+h); -vE*tan(lat)/(RE+h)]
DVector3 ComputeTransportRate(double VelN, double VelE,
                              double Lat, double H,
                              double RN, double RE);

// ---- Utility ----------------------------------------------------------------

/// Skew-symmetric (cross-product) matrix of a 3-vector.
DMatrix3 SkewSymmetric(const DVector3 &V);

// ---- Core inertial computations ---------------------------------------------

/// Compute angular rate resolved in body frame [rad/s].
///   wibb = w_ife^b + w_feb^b
///   where w_ifeb = Cfe2b' * w_ifefe    (Earth rotation projected to body)
///         w_febb = Cfe2b' * w_febfe    (Angular velocity of body w.r.t. ENU resolvedin body)
DVector3 ComputeTrueGyro(const DMatrix3 &Cfe2b,
                         const DVector3 &EarthRateEnu,
                         const DVector3 &AngVelFeEnu);

/// Compute specific force f_ib^b [m/s²].
///   fibb = accBody + coriolisBody + gravityBody
///   gravityBody incorporates plumb-bob gravity with centrifugal correction.
///   coriolisBody     = Cfe2b' * skew(2 * w_ifefe) * vfebfe
///   accBody     = Cfe2b' * accfebfe
DVector3 ComputeSpecificForce(const DMatrix3 &Cnb,
                              const DMatrix3 &Cfe2b,
                              const DVector3 &AccEnu,
                              const DVector3 &VelEnu,
                              const DVector3 &EarthRateEnu,
                              double Lat, double H,
                              double Gravity, double EarthRate, double R0);

/// Iteratively solve for velocity w.r.t. NED (velnav) accounting for
/// transport-rate effects.
DVector3 ComputeVelNav(const DMatrix3 &Cn2fe,
                       const DVector3 &VelEnu,
                       const DVector3 &PosNed,
                       double Lat, double H,
                       double RN, double RE);

// ═══════════════════════════════════════════════════════════════════════════════
// Top-level advanced inertial model entry point
// ═══════════════════════════════════════════════════════════════════════════════

/// Compute all advanced inertial outputs from raw CARLA world data.
///
/// @param PosEsu    CARLA world position [cm] (will be converted to [m])
/// @param VelEsu    CARLA world velocity [cm/s] (will be converted to [m/s])
/// @param AccEsu    CARLA world acceleration [cm/s²] (will be converted to [m/s²])
/// @param AngVelEsu CARLA world angular velocity [rad/s] in ESU LH convention
/// @param PitchDeg  CARLA actor pitch [deg]  (rotation about right axis)
/// @param YawDeg    CARLA actor yaw [deg]    (rotation about up axis)
/// @param RollDeg   CARLA actor roll [deg]   (rotation about forward axis)
/// @param CenterLatDeg  Geodetic reference center latitude [deg]
/// @param CenterLonDeg  Geodetic reference center longitude [deg]
/// @param CenterAltM    Geodetic reference center altitude [m]
///
/// @return AdvancedInertialData with all computed navigation quantities.
AdvancedInertialData ComputeAdvancedInertialModel(
    const DVector3 &PosEsu,
    const DVector3 &VelEsu,
    const DVector3 &AccEsu,
    const DVector3 &AngVelEsu,
    double PitchDeg, double YawDeg, double RollDeg,
    double CenterLatDeg, double CenterLonDeg, double CenterAltM);

} // namespace nav
} // namespace sensor
} // namespace carla
