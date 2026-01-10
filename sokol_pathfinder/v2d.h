#pragma once
#ifndef V2D_STRUCT
#define V2D_STRUCT

//for sqrt
#include <cmath>

	template<typename T>
	struct v2d_generic {
		T x = 0, y = 0;

		v2d_generic() {}
		v2d_generic(T _x, T _y) : x(_x), y(_y) {}
		v2d_generic(const v2d_generic& v) : x(v.x), y(v.y) {}

		//float& operator[](int i) {
		//	if (i == 1) return y;
		//	if (i == 2) return z;
		//	return x;
		//}
		//
		//const float& operator[](int i) const {
		//	if (i == 1) return y;
		//	if (i == 2) return z;
		//	return x;
		//}

		v2d_generic& operator=(const v2d_generic& v) = default;

		T dot(const v2d_generic& v) const { return x * v.x + y * v.y; }
		T mag_sq() const { return dot(*this); }
		T mag() const { return std::sqrt(mag_sq()); }

		v2d_generic norm() const { T r = 1 / mag(); return operator*(r); }
		//v2d_generic cross(const v2d_generic& v) const {
		//	return {
		//		y * v.z - z * v.y,
		//		z * v.x - x * v.z,
		//		x * v.y - y * v.x
		//	};
		//}

		v2d_generic operator-() const { return { -x, -y }; }
		v2d_generic operator+(const v2d_generic& v) const { return { x + v.x, y + v.y }; }
		v2d_generic operator+(const T& s) const { return operator+({ s, s }); }
		v2d_generic operator-(const v2d_generic& v) const { return operator+(-v); }
		v2d_generic operator-(const T& s) const { return operator+(-s); }
		v2d_generic operator*(const v2d_generic& v) const { return { x * v.x, y * v.y }; }
		v2d_generic operator*(const T& s) const { return operator*({ s, s }); }
		v2d_generic operator/(const v2d_generic& v) const { return { x / v.x, y / v.y }; }
		v2d_generic operator/(const T& s) const { T r = 1 / s; return operator*({ r, r }); }//opt?

		//i never really use these, but whatever
		v2d_generic& operator+=(const v2d_generic& v) { *this = *this + v; return*this; }
		v2d_generic& operator+=(const T& v) { *this = *this + v; return*this; }
		v2d_generic& operator-=(const v2d_generic& v) { *this = *this - v; return*this; }
		v2d_generic& operator-=(const T& v) { *this = *this - v; return*this; }
		v2d_generic& operator*=(const v2d_generic& v) { *this = *this * v; return*this; }
		v2d_generic& operator*=(const T& v) { *this = *this * v; return*this; }
		v2d_generic& operator/=(const v2d_generic& v) { *this = *this / v; return*this; }
		v2d_generic& operator/=(const T& v) { *this = *this / v; return*this; }
	};

	template<typename T> v2d_generic<T> operator+(const T& s, const v2d_generic<T>& v) { return v + s; }
	template<typename T> v2d_generic<T> operator-(const T& s, const v2d_generic<T>& v) { return -v + s; }
	template<typename T> v2d_generic<T> operator*(const T& s, const v2d_generic<T>& v) { return v * s; }
	template<typename T> v2d_generic<T> operator/(const T& s, const v2d_generic<T>& v) { return { s / v.x, s / v.y }; }

	typedef v2d_generic<float> vf2d;

#endif//V3D_STRUCT