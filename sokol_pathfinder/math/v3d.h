#pragma once
#ifndef V3D_STRUCT
#define V3D_STRUCT

//for sqrt
#include <cmath>

template<typename T>
struct v3d_generic {
	T x=0, y=0, z=0;

	v3d_generic() {}
	v3d_generic(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
	v3d_generic(const v3d_generic& v) : x(v.x), y(v.y), z(v.z) {}

	float& operator[](int i) {
		if(i==1) return y;
		if(i==2) return z;
		return x;
	}

	const float& operator[](int i) const {
		if(i==1) return y;
		if(i==2) return z;
		return x;
	}

	v3d_generic& operator=(const v3d_generic& v)=default;

	T dot(const v3d_generic& v) const { return x*v.x+y*v.y+z*v.z; }
	T mag_sq() const { return dot(*this); }
	T mag() const { return std::sqrt(mag_sq()); }

	v3d_generic norm() const { T r=1/mag(); return operator*(r); }
	v3d_generic cross(const v3d_generic& v) const {
		return {
			y*v.z-z*v.y,
			z*v.x-x*v.z,
			x*v.y-y*v.x
		};
	}

	v3d_generic operator-() const { return {-x, -y, -z}; }
	v3d_generic operator+(const v3d_generic& v) const { return {x+v.x, y+v.y, z+v.z}; }
	v3d_generic operator+(const T& s) const { return operator+({s, s, s}); }
	v3d_generic operator-(const v3d_generic& v) const { return operator+(-v); }
	v3d_generic operator-(const T& s) const { return operator+(-s); }
	v3d_generic operator*(const v3d_generic& v) const { return {x*v.x, y*v.y, z*v.z}; }
	v3d_generic operator*(const T& s) const { return operator*({s, s, s}); }
	v3d_generic operator/(const v3d_generic& v) const { return {x/v.x, y/v.y, z/v.z}; }
	v3d_generic operator/(const T& s) const { T r=1/s; return operator*({r, r, r}); }//opt?

	//i never really use these, but whatever
	v3d_generic& operator+=(const v3d_generic& v) { *this=*this+v; return*this; }
	v3d_generic& operator+=(const T& v) { *this=*this+v; return*this; }
	v3d_generic& operator-=(const v3d_generic& v) { *this=*this-v; return*this; }
	v3d_generic& operator-=(const T& v) { *this=*this-v; return*this; }
	v3d_generic& operator*=(const v3d_generic& v) { *this=*this*v; return*this; }
	v3d_generic& operator*=(const T& v) { *this=*this*v; return*this; }
	v3d_generic& operator/=(const v3d_generic& v) { *this=*this/v; return*this; }
	v3d_generic& operator/=(const T& v) { *this=*this/v; return*this; }
};

template<typename T> v3d_generic<T> operator+(const T& s, const v3d_generic<T>& v) { return v+s; }
template<typename T> v3d_generic<T> operator-(const T& s, const v3d_generic<T>& v) { return -v+s; }
template<typename T> v3d_generic<T> operator*(const T& s, const v3d_generic<T>& v) { return v*s; }
template<typename T> v3d_generic<T> operator/(const T& s, const v3d_generic<T>& v) { return {s/v.x, s/v.y, s/v.z}; }

typedef v3d_generic<float> vf3d;
#endif//V3D_STRUCT