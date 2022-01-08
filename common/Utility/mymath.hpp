#ifndef MYMATH_HPP_
#define MYMATH_HPP_

extern "C" {
#define ARM_MATH_CM4
#include "arm_math.h"
}

namespace mymath {

constexpr static float const_rad2deg = 180.0f / PI;
constexpr static float const_deg2rad = PI / 180.0f;

inline float rad2deg(float _r) { return _r * const_rad2deg; };
inline float deg2rad(float _d) { return _d * const_deg2rad; };

inline float normalize_rad_0to2pi(float _d) {
	if (_d < 0.0f || _d >= 2.0f*PI)
	{
		int mod = static_cast<int>(_d / (2.0f*PI));
		_d -= (mod*2.0f*PI);
		if (_d < 0.0f) _d = _d + 2.0f*PI;
	}
	return _d;
}

inline float satf(float _x, float _u, float _l){
  return (_x > _u) ? _u : ((_x < _l) ? _l : _x);
}

inline float absf(float _x){
  return (_x < 0) ? -_x : _x; 
}

inline float sinf(float _x) { return arm_sin_f32(_x); };
inline float cosf(float _x) { return arm_cos_f32(_x); };

float atanf(float x);
float atan2f(float y, float x);

}; // namespace mymath

#endif