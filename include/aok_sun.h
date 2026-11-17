// Sunrise and sunset for on-device schedules (NOAA's simplified algorithm).
// Accurate to a minute or two, which is plenty for blinds. Header-only and
// free of Arduino so the host tests can check it.
#pragma once

#include <cmath>

namespace aok {

constexpr double kPi = 3.14159265358979323846;
inline double rad(double d) { return d * kPi / 180.0; }
inline double deg(double r) { return r * 180.0 / kPi; }
inline double wrap(double v, double m) { v = std::fmod(v, m); return v < 0 ? v + m : v; }

// Minutes after local midnight of sunrise (rise = true) or sunset on day-of-year
// `yday` (0 = 1 January), at latitude/longitude in degrees (south and west
// negative), for a place `utcOffsetMin` minutes ahead of UTC. Returns -1 when
// the sun doesn't rise or set that day (polar day or night).
inline int sunMinutes(bool rise, int yday, double lat, double lon, int utcOffsetMin) {
  const double lngHour = lon / 15.0;
  const double t = (yday + 1) + (((rise ? 6.0 : 18.0) - lngHour) / 24.0);
  const double M = (0.9856 * t) - 3.289;
  const double L = wrap(M + (1.916 * std::sin(rad(M))) + (0.020 * std::sin(rad(2 * M))) + 282.634, 360.0);
  double RA = wrap(deg(std::atan(0.91764 * std::tan(rad(L)))), 360.0);
  RA += (std::floor(L / 90.0) * 90.0) - (std::floor(RA / 90.0) * 90.0);
  RA /= 15.0;
  const double sinDec = 0.39782 * std::sin(rad(L));
  const double cosDec = std::cos(std::asin(sinDec));
  const double cosH = (std::cos(rad(90.833)) - (sinDec * std::sin(rad(lat)))) / (cosDec * std::cos(rad(lat)));
  if (cosH > 1.0 || cosH < -1.0) return -1;
  const double H = (rise ? 360.0 - deg(std::acos(cosH)) : deg(std::acos(cosH))) / 15.0;
  const double T = H + RA - (0.06571 * t) - 6.622;
  const double ut = wrap(T - lngHour, 24.0);
  return static_cast<int>(std::lround(wrap(ut * 60.0 + utcOffsetMin, 1440.0)));
}

}  // namespace aok
