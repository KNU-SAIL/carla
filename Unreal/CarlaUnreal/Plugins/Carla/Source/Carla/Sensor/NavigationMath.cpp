// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
// =============================================================================
// NavigationMath.cpp
// =============================================================================
// Implementation of the Earth-aware inertial navigation math helpers.
// All internal math uses double precision.
// =============================================================================

#include "Carla/Sensor/NavigationMath.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace carla {
namespace sensor {
namespace nav {

// ═══════════════════════════════════════════════════════════════════════════════
// Constants
// ═══════════════════════════════════════════════════════════════════════════════
static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
static constexpr double kPi     = 3.14159265358979323846;

// WGS-84 ellipsoid constants
static constexpr double kWgs84_R  = 6378137.0;            // semi-major axis [m]
static constexpr double kWgs84_e  = 0.0818191908426;      // first eccentricity
static constexpr double kWgs84_ER = 7.292115e-5;          // Earth rate [rad/s]

// ═══════════════════════════════════════════════════════════════════════════════
// Coordinate transforms
// ═══════════════════════════════════════════════════════════════════════════════

DVector3 EsuToEnu(const DVector3 &VecEsu)
{
  return { VecEsu.X, -VecEsu.Y, VecEsu.Z };
}

DVector3 AngularVelocityLhsToRhs(const DVector3 &AngVelEsu)
{
  // Left-handed rotation about ESU axes.
  // to
  // right-handed rotation about ENU axes:

  return { -AngVelEsu.X, AngVelEsu.Y, -AngVelEsu.Z };
}

DVector3 ConvertEnuToNed(const DVector3 &VecEnu)
{
  return { VecEnu.Y, VecEnu.X, -VecEnu.Z };
}

DMatrix3 BuildCenu2ned()
{
  return DMatrix3(
    DVector3(0.0, 1.0, 0.0),
    DVector3(1.0, 0.0, 0.0),
    DVector3(0.0, 0.0, -1.0)
  );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Attitude matrices
// ═══════════════════════════════════════════════════════════════════════════════

DMatrix3 BuildCen(double LatRad, double LonRad)
{
  // Attitude of NED w.r.t. ECEF.
  const double sLat = std::sin(LatRad);
  const double cLat = std::cos(LatRad);
  const double sLon = std::sin(LonRad);
  const double cLon = std::cos(LonRad);

  return DMatrix3(
    DVector3(-sLat * cLon, -sLon, -cLat * cLon),
    DVector3(-sLat * sLon,  cLon, -cLat * sLon),
    DVector3( cLat,         0.0,  -sLat)
  );
}

DMatrix3 BuildCuelocal2uebodyFromPYR(double PitchDeg, double YawDeg, double RollDeg)
{
  // CARLA PYR system: Right-Right-Left handed Rule = DCM (-X, -Y, Z)
  // Attitude of body(UE) w.r.t local-tangent(UE)(ESU) 
  const double cr = std::cos(-RollDeg  * kDeg2Rad);
  const double sr = std::sin(-RollDeg  * kDeg2Rad);
  const double cp = std::cos(-PitchDeg * kDeg2Rad);
  const double sp = std::sin(-PitchDeg * kDeg2Rad);
  const double cy = std::cos( YawDeg   * kDeg2Rad);
  const double sy = std::sin( YawDeg   * kDeg2Rad);

  // Rx
  DMatrix3 Rx;
  Rx.M[0][0] = 1.0; Rx.M[0][1] = 0.0; Rx.M[0][2] = 0.0;
  Rx.M[1][0] = 0.0; Rx.M[1][1] = cr;  Rx.M[1][2] = -sr;
  Rx.M[2][0] = 0.0; Rx.M[2][1] = sr;  Rx.M[2][2] = cr;

  // Ry
  DMatrix3 Ry;
  Ry.M[0][0] = cp;  Ry.M[0][1] = 0.0; Ry.M[0][2] = sp;
  Ry.M[1][0] = 0.0; Ry.M[1][1] = 1.0; Ry.M[1][2] = 0.0;
  Ry.M[2][0] = -sp; Ry.M[2][1] = 0.0; Ry.M[2][2] = cp;

  // Rz
  DMatrix3 Rz;
  Rz.M[0][0] = cy;  Rz.M[0][1] = -sy; Rz.M[0][2] = 0.0;
  Rz.M[1][0] = sy;  Rz.M[1][1] = cy;  Rz.M[1][2] = 0.0;
  Rz.M[2][0] = 0.0; Rz.M[2][1] = 0.0; Rz.M[2][2] = 1.0;

  return Rz * Ry * Rx; 
}

DMatrix3 BuildCbue2b()
{
  // Attitude of body-Nav(FRD) w.r.t. body-UE(FRU) = diag([1, 1, -1])
  return DMatrix3(
    DVector3(1.0, 0.0,  0.0),
    DVector3(0.0, 1.0,  0.0),
    DVector3(0.0, 0.0, -1.0)
  );
}

DMatrix3 BuildCfe2uelocal()
{
  // Attitude of local-UE(ESU) w.r.t. local-Nav(ENU) = diag([1, -1, 1])
  return DMatrix3(
    DVector3(1.0,  0.0, 0.0),
    DVector3(0.0, -1.0, 0.0),
    DVector3(0.0,  0.0, 1.0)
  );
}

DMatrix3 BuildCnb(const DVector3 &LlhRad,
                  const DVector3 &CenterLlhRad,
                  const DMatrix3 &Cuelocal2uebody)
{
  // ---- Constant DCMs ----
  const DMatrix3 Cenu2ned  = BuildCenu2ned();         // NED w.r.t. ENU
  const DMatrix3 Cned2enu  = Cenu2ned.T();            // ENU w.r.t. NED
  const DMatrix3 Cbue2b    = BuildCbue2b();           // body-Nav(FRD) w.r.t. body-UE(FRU)
  const DMatrix3 Cfe2uelocal = BuildCfe2uelocal();    // local-UE(ESU) w.r.t. local-Nav(ENU)

  // ECEF ↔ NED rotations
  const DMatrix3 Ce2fn = BuildCen(CenterLlhRad.X, CenterLlhRad.Y);  // NED@center w.r.t. ECEF
  const DMatrix3 Cne   = BuildCen(LlhRad.X, LlhRad.Y).T();          // ECEF w.r.t. NED@vehicle

 
  // Cn2fe = (ECEF w.r.t. NED@veh) * (NED@center w.r.t. ECEF) * (ENU w.r.t. NED)
  // Result: Attitude of ENU@center w.r.t. NED@vehicle
  const DMatrix3 Cn2fe = Cne * Ce2fn * Cned2enu;

  // Cnb = (ENU@center w.r.t. NED@vehicle) * (local-Nav(ENU) w.r.t. ENU@center) 
  //       * (body-UE(FRU) w.r.t. local-Nav(ENU)) * (body(FRD) w.r.t. body(FRU))
  //   Result: Attitude of body(FRD) w.r.t. Nav(NED)
  const DMatrix3 Cnb = Cn2fe * Cfe2uelocal * Cuelocal2uebody * Cbue2b;
  return Cnb;
}

DMatrix3 BuildCfe2b(const DMatrix3 &Cuelocal2uebody)
{
  // Attitude of body(FRD) w.r.t. local-Nav(ENU)
  const DMatrix3 Cfe2uelocal = BuildCfe2uelocal();
  const DMatrix3 Cbue2b      = BuildCbue2b();
  return Cfe2uelocal * Cuelocal2uebody * Cbue2b;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Geodesy
// ═══════════════════════════════════════════════════════════════════════════════

Wgs84Params ComputeWgs84Gravity(const DVector3 &LlhRad)
{
  // Matches wgs84GravityModel.m exactly.
  const double lat = LlhRad.X;
  const double h   = LlhRad.Z;
  const double sinLat = std::sin(lat);
  const double sin2Lat = sinLat * sinLat;
  const double sin2LatDouble = std::sin(2.0 * lat);

  const double R  = kWgs84_R;
  const double e  = kWgs84_e;
  const double e2 = e * e;

  const double denom = std::pow(1.0 - e2 * sin2Lat, 1.5);
  const double RN = R * (1.0 - e2) / denom;               // meridional radius
  const double RE = R / std::sqrt(1.0 - e2 * sin2Lat);    // prime-vertical radius
  const double R0 = std::sqrt(RN * RE);                    // geometric-mean radius

  // Somigliana gravity at the ellipsoid surface
  const double g0 = 9.780318 * (1.0 + 5.3024e-3 * sin2Lat - 5.9e-6 * sin2LatDouble * sin2LatDouble);

  // Free-air corrected gravity at height h
  const double g = g0 / ((1.0 + h / R0) * (1.0 + h / R0));

  Wgs84Params P;
  P.R0 = R0;
  P.g  = g;
  P.ER = kWgs84_ER;
  P.RN = RN;
  P.RE = RE;
  return P;
}

DVector3 Llh2Xyz(const DVector3 &LlhRad)
{
  // Geodetic (lat, lon, h) [rad, rad, m] → ECEF (X, Y, Z) [m]
  const double lat = LlhRad.X;
  const double lon = LlhRad.Y;
  const double h   = LlhRad.Z;

  const double sinLat = std::sin(lat);
  const double cosLat = std::cos(lat);
  const double sinLon = std::sin(lon);
  const double cosLon = std::cos(lon);

  const double e2 = kWgs84_e * kWgs84_e;
  const double RE = kWgs84_R / std::sqrt(1.0 - e2 * sinLat * sinLat);

  return {
    (RE + h) * cosLat * cosLon,
    (RE + h) * cosLat * sinLon,
    (RE * (1.0 - e2) + h) * sinLat
  };
}

DVector3 Xyz2Llh(const DVector3 &Xyz)
{
  // ECEF (X, Y, Z) [m] → geodetic (lat, lon, h) [rad, rad, m]
  // Closed-form algorithm
  const double x = Xyz.X;
  const double y = Xyz.Y;
  const double z = Xyz.Z;
  const double x2 = x * x;
  const double y2 = y * y;
  const double z2 = z * z;

  const double a  = 6378137.0;             // earth radius [m]
  const double b  = 6356752.3142;          // earth semiminor [m]
  const double e  = std::sqrt(1.0 - (b / a) * (b / a));
  const double b2 = b * b;
  const double e2 = e * e;
  const double ep = e * (a / b);
  const double r  = std::sqrt(x2 + y2);
  const double r2 = r * r;
  const double E2 = a * a - b * b;
  const double F  = 54.0 * b2 * z2;
  const double G  = r2 + (1.0 - e2) * z2 - e2 * E2;
  const double c  = (e2 * e2 * F * r2) / (G * G * G);
  const double s  = std::cbrt(1.0 + c + std::sqrt(c * c + 2.0 * c));
  const double P  = F / (3.0 * (s + 1.0 / s + 1.0) * (s + 1.0 / s + 1.0) * G * G);
  const double Q  = std::sqrt(1.0 + 2.0 * e2 * e2 * P);
  const double ro = -(P * e2 * r) / (1.0 + Q)
                    + std::sqrt((a * a / 2.0) * (1.0 + 1.0 / Q)
                                - (P * (1.0 - e2) * z2) / (Q * (1.0 + Q))
                                - P * r2 / 2.0);
  const double tmp = (r - e2 * ro) * (r - e2 * ro);
  const double U  = std::sqrt(tmp + z2);
  const double V  = std::sqrt(tmp + (1.0 - e2) * z2);
  const double zo = (b2 * z) / (a * V);

  const double height = U * (1.0 - b2 / (a * V));
  const double lat    = std::atan((z + ep * ep * zo) / r);

  double lon;
  if (x >= 0.0)
  {
    lon = std::atan(y / x);
  }
  else if (y >= 0.0)
  {
    lon = kPi + std::atan(y / x);
  }
  else
  {
    lon = std::atan(y / x) - kPi;
  }

  return { lat, lon, height };
}

DVector3 NedToLlh(const DVector3 &NedPos, const DVector3 &CenterLlhDeg)
{
  const DVector3 CenterRad = {
    CenterLlhDeg.X * kDeg2Rad,
    CenterLlhDeg.Y * kDeg2Rad,
    CenterLlhDeg.Z   // height already in meters
  };

  // xyz = centerEcef + Cen * nedPos
  const DVector3 CenterEcef = Llh2Xyz(CenterRad);
  const DVector3 Offset = BuildCen(CenterRad.X, CenterRad.Y) * NedPos;
  const DVector3 Xyz = CenterEcef + Offset;

  return Xyz2Llh(Xyz);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Earth rotation / transport rate
// ═══════════════════════════════════════════════════════════════════════════════

DVector3 ComputeEarthRateEnu(double LatRad, double EarthRate)
{
  // Earth rotation of local-tangent frame w.r.t inertial frame resolved in local-tangent frame:
  // wifefe = ER * [0; cos(lat); sin(lat)]
  return { 0.0, EarthRate * std::cos(LatRad), EarthRate * std::sin(LatRad) };
}

DVector3 ComputeTransportRate(double VelN, double VelE,
                              double Lat, double H,
                              double RN, double RE)
{
  // Transport rate resolved in NED frame:
  // wenn = [vE/(RE+h); -vN/(RN+h); -vE*tan(lat)/(RE+h)]
  const double ReH = RE + H;
  const double RnH = RN + H;
  return {
     VelE / ReH,
    -VelN / RnH,
    -VelE * std::tan(Lat) / ReH
  };
}

// ═══════════════════════════════════════════════════════════════════════════════
// Utility
// ═══════════════════════════════════════════════════════════════════════════════

DMatrix3 SkewSymmetric(const DVector3 &V)
{
  return DMatrix3(
    DVector3( 0.0, -V.Z,  V.Y),
    DVector3( V.Z,  0.0, -V.X),
    DVector3(-V.Y,  V.X,  0.0)
  );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Core inertial computations
// ═══════════════════════════════════════════════════════════════════════════════

DVector3 ComputeTrueGyro(const DMatrix3 &Cfe2b,
                         const DVector3 &EarthRateEnu,
                         const DVector3 &AngVelFeEnu)
{
  //   wifeb: Earth rotation in body)
  //   wfebb: Angular velocity of body w.r.t. local-tangent frame resolved in body frame)
  //   wibb:  Angular velocity of body w.r.t. inertial frame resolved in body frame
  const DMatrix3 Cfe2bT = Cfe2b.T();
  const DVector3 Wifeb  = Cfe2bT * EarthRateEnu;
  const DVector3 Wfebb  = Cfe2bT * AngVelFeEnu;
  return Wifeb + Wfebb;
}

DVector3 ComputeSpecificForce(const DMatrix3 &Cnb,
                              const DMatrix3 &Cfe2b,
                              const DVector3 &AccEnu,
                              const DVector3 &VelEnu,
                              const DVector3 &EarthRateEnu,
                              double Lat, double H,
                              double Gravity, double EarthRate, double R0)
{
  // Gravity vector in NED with centrifugal correction
  // g_ned = [0; 0; g]
  // centrifugal_ned = ER^2 * (R0+h)/2 * [sin(2*lat); 0; 1+cos(2*lat)]
  // gravBody = Cnb^T * (g_ned - centrifugal_ned)
  const double sin2Lat = std::sin(2.0 * Lat);
  const double cos2Lat = std::cos(2.0 * Lat);
  const double CentFactor = EarthRate * EarthRate * (R0 + H) / 2.0;
  const DVector3 GravNed = { 0.0, 0.0, Gravity };
  const DVector3 CentNed = { CentFactor * sin2Lat, 0.0, CentFactor * (1.0 + cos2Lat) };
  const DMatrix3 CnbT = Cnb.T();
  const DVector3 GravBody = (CnbT * (GravNed - CentNed));

  // Coriolis in body frame
  // corlBody = Cfe2b' * skew(2 * w_ie^fe) * vel_fe
  const DMatrix3 Cfe2bT = Cfe2b.T();
  const DMatrix3 SkewW = SkewSymmetric(EarthRateEnu * 2.0);
  const DVector3 CorlBody = Cfe2bT * (SkewW * VelEnu);

  // Acceleration in body frame
  // accBody = Cfe2b' * acc_fe
  const DVector3 AccBody = Cfe2bT * AccEnu;

  // fibb = accBody + corlBody + gravBody
  return AccBody + CorlBody - GravBody;
}

DVector3 ComputeVelNav(const DMatrix3 &Cn2fe,
                       const DVector3 &VelEnu,
                       const DVector3 &PosNed,
                       double Lat, double H,
                       double RN, double RE)
{
  // MATLAB:
  //   velFe2bInNav = Cn2fe * carlaVelFe;
  //   vN = velFe2bInNav(1); vE = velFe2bInNav(2);
  //   wenn = [vE/(RE+h); -vN/(RN+h); -vE*tan(lat)/(RE+h)];
  //   oldwenn = [inf; inf; inf];
  //   while(norm(wenn - oldwenn) > 1e-10)
  //       oldwenn = wenn;
  //       velnav = velFe2bInNav - skewsymm(wenn) * carlaPosFn;
  //       vN = velnav(1); vE = velnav(2);
  //       wenn = [vE/(RE+h); -vN/(RN+h); -vE*tan(lat)/(RE+h)];
  //   end

  const DVector3 VelFe2bInNav = Cn2fe * VelEnu;

  double vN = VelFe2bInNav.X;
  double vE = VelFe2bInNav.Y;

  DVector3 Wenn = ComputeTransportRate(vN, vE, Lat, H, RN, RE);
  DVector3 OldWenn = { std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity() };

  DVector3 VelNav = VelFe2bInNav;

  constexpr int kMaxIter = 100;
  int iter = 0;
  while ((Wenn - OldWenn).Norm() > 1e-10)
  {
    OldWenn = Wenn;
    VelNav = VelFe2bInNav - SkewSymmetric(Wenn) * PosNed;
    vN = VelNav.X;
    vE = VelNav.Y;
    Wenn = ComputeTransportRate(vN, vE, Lat, H, RN, RE);
    ++iter;
  }

  return VelNav;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Top-level entry point
// ═══════════════════════════════════════════════════════════════════════════════

AdvancedInertialData ComputeAdvancedInertialModel(
    const DVector3 &PosEsu,
    const DVector3 &VelEsu,
    const DVector3 &AccEsu,
    const DVector3 &AngVelEsu,
    double PitchDeg, double YawDeg, double RollDeg,
    double CenterLatDeg, double CenterLonDeg, double CenterAltM)
{
  // -------- Unit conversion --------------------------------------------------
  // CARLA/UE uses centimeters.  Navigation math uses meters.
  constexpr double kCmToM = 0.01;
  const DVector3 PosEsuM = PosEsu * kCmToM;     // [m]
  const DVector3 VelEsuM = VelEsu * kCmToM;     // [m/s]
  const DVector3 AccEsuM = AccEsu * kCmToM;     // [m/s²]

  // Angular velocity: input is [rad/s] in ESU left-handed convention
  const DVector3 AngVelEsuRad = AngVelEsu;

  // ═══════════════════════════════════════════════════════════════════════════════
  // line 16~25 
  // ═══════════════════════════════════════════════════════════════════════════════

  // -------- ESU → ENU --------------------------------------------------------
  const DVector3 PosEnu    = EsuToEnu(PosEsuM);
  const DVector3 VelEnu    = EsuToEnu(VelEsuM);
  const DVector3 AccEnu    = EsuToEnu(AccEsuM);
  // Angular velocity handedness correction (LH ESU → RH ENU)
  const DVector3 AngVelFe  = AngularVelocityLhsToRhs(AngVelEsuRad);

  // -------- Reference geodetic center ----------------------------------------
  const DVector3 CenterLlhDeg = { CenterLatDeg, CenterLonDeg, CenterAltM };
  const DVector3 CenterLlhRad = { CenterLatDeg * kDeg2Rad,
                                   CenterLonDeg * kDeg2Rad,
                                   CenterAltM };

  // ═══════════════════════════════════════════════════════════════════════════════
  // line 26~67 
  // ═══════════════════════════════════════════════════════════════════════════════

  // -------- ENU → NED --------------------------------------------------------
  const DVector3 PosNed = ConvertEnuToNed(PosEnu);

  // -------- NED → LLH --------------------------------------------------------
  const DVector3 Llh = NedToLlh(PosNed, CenterLlhDeg);  // [rad, rad, m]
  const double Lat = Llh.X;
  const double H   = Llh.Z;

  // -------- Attitude chain ---------------------------------------------------
  const DMatrix3 Cuelocal2uebody = BuildCuelocal2uebodyFromPYR(PitchDeg, YawDeg, RollDeg);
  const DMatrix3 Cnb  = BuildCnb(Llh, CenterLlhRad, Cuelocal2uebody);
  const DMatrix3 Cfe2b = BuildCfe2b(Cuelocal2uebody);

  // -------- WGS-84 -----------------------------------------------------------
  const Wgs84Params Wgs = ComputeWgs84Gravity(Llh);

  // -------- Cn2fe (for velocity transform) -----------------------------------
  const DMatrix3 Cenu2ned = BuildCenu2ned();
  const DMatrix3 Cned2enu = Cenu2ned.T();
  const DMatrix3 Ce2fn = BuildCen(CenterLlhRad.X, CenterLlhRad.Y);
  const DMatrix3 Cne   = BuildCen(Llh.X, Llh.Y).T();
  const DMatrix3 Cn2fe = Cne * Ce2fn * Cned2enu;

  // ═══════════════════════════════════════════════════════════════════════════════
  // line 68~76 
  // ═══════════════════════════════════════════════════════════════════════════════

  // -------- True gyro --------------------------------------------------------
  const DVector3 EarthRateEnu = ComputeEarthRateEnu(CenterLatDeg * kDeg2Rad, Wgs.ER);
  const DVector3 Wibb = ComputeTrueGyro(Cfe2b, EarthRateEnu, AngVelFe);

  // ═══════════════════════════════════════════════════════════════════════════════
  // line 77~85 
  // ═══════════════════════════════════════════════════════════════════════════════

  // -------- Specific force ---------------------------------------------------
  const DVector3 Fibb = ComputeSpecificForce(
      Cnb, Cfe2b, AccEnu, VelEnu, EarthRateEnu,
      Lat, H, Wgs.g, Wgs.ER, Wgs.R0);
  
  // ═══════════════════════════════════════════════════════════════════════════════
  // line 86~99 
  // ═══════════════════════════════════════════════════════════════════════════════
  
  // -------- Velocity w.r.t. NED (iterative) ----------------------------------
  const DVector3 VelNav = ComputeVelNav(Cn2fe, VelEnu, PosNed, Lat, H, Wgs.RN, Wgs.RE);

  // -------- Pack result ------------------------------------------------------
  AdvancedInertialData Result;
  Result.Cnb     = Cnb;
  Result.Wibb    = Wibb;
  Result.Fibb    = Fibb;
  Result.Llh     = Llh;
  Result.VelNav  = VelNav;
  return Result;
}

} // namespace nav
} // namespace sensor
} // namespace carla
