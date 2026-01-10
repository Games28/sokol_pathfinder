//column major
#pragma once
#ifndef MAT4_STRUCT_H
#define MAT4_STRUCT_H

//for memset
#include <string>

//for trig
#include <cmath>

//not sure what these macros did :3
#undef near
#undef far

#include "v3d.h"

#include "utils.h"

struct mat4 {
	float m[16];

	mat4() { std::memset(m, 0.f, sizeof(float)*16); }

	//row, col
	const float& operator()(int r, int c) const { return m[r+4*c]; }
	float& operator()(int r, int c) { return m[r+4*c]; }

	static mat4 makeIdentity() {
		mat4 c;
		c(0, 0)=1;
		c(1, 1)=1;
		c(2, 2)=1;
		c(3, 3)=1;
		return c;
	}

	//frobenius norm
	static float norm(const mat4& m) {
		float sum=0;
		for(int i=0; i<16; i++) sum+=m.m[i]*m.m[i];
		return std::sqrt(sum);
	}

	static mat4 sub(const mat4& lhs, const mat4& rhs) {
		mat4 r;
		for(int i=0; i<16; i++) r.m[i]=lhs.m[i]-rhs.m[i];
		return r;
	}

	//transform order is right to left: A then B = mul(B, A)
	static mat4 mul(const mat4& lhs, const mat4& rhs) {
		mat4 res;
		//basically a lot of corresponding dot products...
		for(int r=0; r<4; r++) {
			for(int c=0; c<4; c++) {
				res(r, c)=
					lhs(r, 0)*rhs(0, c)+
					lhs(r, 1)*rhs(1, c)+
					lhs(r, 2)*rhs(2, c)+
					lhs(r, 3)*rhs(3, c);
			}
		}
		return res;
	}

	static mat4 transpose(const mat4& a) {
		mat4 b;
		for(int r=0; r<4; r++) {
			for(int c=0; c<4; c++) {
				b(r, c)=a(c, r);
			}
		}
		return b;
	}

	//compute determinant of sub matrix
	static float minor(const mat4& m44, int row, int col) {
		float m33[9];
		int subr=0;
		for(int r=0; r<4; r++) {
			if(r==row) continue;
			int subc=0;
			for(int c=0; c<4; c++) {
				if(c==col) continue;
				m33[subr+3*subc]=m44(r, c);
				subc++;
			}
			subr++;
		}
		return
			m33[0+3*0]*(m33[1+3*1]*m33[2+3*2]-m33[1+3*2]*m33[2+3*1])-
			m33[0+3*1]*(m33[1+3*0]*m33[2+3*2]-m33[1+3*2]*m33[2+3*0])+
			m33[0+3*2]*(m33[1+3*0]*m33[2+3*1]-m33[1+3*1]*m33[2+3*0]);
	}

	static mat4 inverse(const mat4& a) {
		mat4 cofactors;
		for(int r=0; r<4; r++) {
			for(int c=0; c<4; c++) {
				cofactors(r, c)=((r+c)%2?-1:1)*mat4::minor(a, r, c);
			}
		}
		mat4 adjugate=mat4::transpose(cofactors);

		//default to identity if singular
		mat4 inv=mat4::makeIdentity();

		//get determinant using first row and cofactors
		float det=0;
		for(int c=0; c<4; c++) {
			det+=a(0, c)*cofactors(0, c);
		}
		if(det!=0) {
			//divide adjugate by determinant
			float recip=1/det;
			for(int i=0; i<16; i++) inv.m[i]=recip*adjugate.m[i];
		}

		return inv;
	}

	static mat4 makeTranslation(const vf3d& trans) {
		mat4 c=mat4::makeIdentity();
		c(0, 3)=trans.x;
		c(1, 3)=trans.y;
		c(2, 3)=trans.z;
		return c;
	}

	static mat4 makeScale(const vf3d& scl) {
		mat4 c;
		c(0, 0)=scl.x;
		c(1, 1)=scl.y;
		c(2, 2)=scl.z;
		c(3, 3)=1;
		return c;
	}

	static mat4 makeRotX(float t) {
		mat4 a;
		float c=std::cos(t), s=std::sin(t);
		a(0, 0)=1;
		a(1, 1)=c, a(1, 2)=-s;
		a(2, 1)=s, a(2, 2)=c;
		a(3, 3)=1;
		return a;
	}

	static mat4 makeRotY(float t) {
		mat4 a;
		float c=std::cos(t), s=std::sin(t);
		a(0, 0)=c, a(0, 2)=s;
		a(1, 1)=1;
		a(2, 0)=-s, a(2, 2)=c;
		a(3, 3)=1;
		return a;
	}

	static mat4 makeRotZ(float t) {
		mat4 a;
		float c=std::cos(t), s=std::sin(t);
		a(0, 0)=c, a(0, 1)=-s;
		a(1, 0)=s, a(1, 1)=c;
		a(2, 2)=1;
		a(3, 3)=1;
		return a;
	}

	//rodrigues axis-angle formula
	static mat4 makeRotAxis(const vf3d& axis, float theta) {
		float x=axis.x, y=axis.y, z=axis.z;
		float c=cos(theta);
		float s=sin(theta);
		float t=1-c;

		mat4 m;
		m(0, 0)=c+x*x*t, m(0, 1)=x*y*t-z*s, m(0, 2)=x*z*t+y*s;
		m(1, 0)=y*x*t+z*s, m(1, 1)=c+y*y*t, m(1, 2)=y*z*t-x*s;
		m(2, 0)=z*x*t-y*s, m(2, 1)=z*y*t-x*s, m(2, 2)=c+z*z*t;
		m(3, 3)=1;
		return m;
	}

	//from camera -> world
	//invert this for view matrix.
	static mat4 makeLookAt(const vf3d& eye, const vf3d& target, vf3d up) {
		//coordinate axes from RHR
		vf3d fwd=(target-eye).norm();
		vf3d rgt=fwd.cross(up).norm();
		up=rgt.cross(fwd);

		//column vectors + translation
		mat4 a;
		a(0, 0)=rgt.x, a(0, 1)=up.x, a(0, 2)=-fwd.x, a(0, 3)=eye.x;
		a(1, 0)=rgt.y, a(1, 1)=up.y, a(1, 2)=-fwd.y, a(1, 3)=eye.y;
		a(2, 0)=rgt.z, a(2, 1)=up.z, a(2, 2)=-fwd.z, a(2, 3)=eye.z;
		a(3, 3)=1;
		return a;
	}

	//fov=degrees & asp=width/height
	//https://gamedev.stackexchange.com/questions/120338
	static mat4 makePerspective(float fov_deg, float asp, float near, float far) {
		float fov_rad=fov_deg/180*Pi;
		float inv_tan=1/std::tan(fov_rad/2);
		float inv_nearfar=1/(near-far);
		mat4 a;
		a(0, 0)=inv_tan/asp;
		a(1, 1)=inv_tan;
		a(2, 2)=(far+near)*inv_nearfar, a(2, 3)=2*far*near*inv_nearfar;
		a(3, 2)=-1;
		return a;
	}
};

//mat4 * vec4 implies homogeneous coordinates
//  but im not going to make a vec4 struct for this.
vf3d matMulVec(const mat4& m, const vf3d& v, float& w) {
	float x=m(0, 0)*v.x+m(0, 1)*v.y+m(0, 2)*v.z+m(0, 3)*w;
	float y=m(1, 0)*v.x+m(1, 1)*v.y+m(1, 2)*v.z+m(1, 3)*w;
	float z=m(2, 0)*v.x+m(2, 1)*v.y+m(2, 2)*v.z+m(2, 3)*w;
	w=m(3, 0)*v.x+m(3, 1)*v.y+m(3, 2)*v.z+m(3, 3)*w;

	return {x, y, z};
}


#endif