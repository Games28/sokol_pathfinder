#pragma once
#ifndef MESH_STRUCT_H
#define MESH_STRUCT_H

#include "math/v3d.h"

#include <fstream>
#include <vector>
#include <sstream>
#include <unordered_map>
//for hash
#include <functional>

#include "return_code.h"

#include "utils.h"

struct Mesh {
	struct v2d_t { float u=0, v=0; };

	struct Vertex {
		vf3d pos, norm;
		v2d_t tex;
	};
	std::vector<Vertex> verts;

	struct IndexTriangle {
		int a, b, c;
	};
	std::vector<IndexTriangle> tris;
	

	sg_buffer vbuf{SG_INVALID_ID};
	sg_buffer ibuf{SG_INVALID_ID};



	void updateVertexBuffer() {
		//free old
		if(vbuf.id!=SG_INVALID_ID) sg_destroy_buffer(vbuf);

		//send data to "gpu"
		sg_buffer_desc vbuf_desc{};
		vbuf_desc.usage.index_buffer=false;
		vbuf_desc.usage.vertex_buffer=true;
		vbuf_desc.data.ptr=verts.data();
		vbuf_desc.data.size=sizeof(Vertex)*verts.size();
		vbuf=sg_make_buffer(vbuf_desc);
	}

	void updateIndexBuffer() {
		//free old
		if(ibuf.id!=SG_INVALID_ID) sg_destroy_buffer(ibuf);

		//alloc temp
		const int num_tris=tris.size();
		const int num_indexes=3*num_tris;
		std::uint32_t* ibuf_data=new std::uint32_t[num_indexes];

		//transfer to temp
		for(int i=0; i<num_tris; i++) {
			const auto& t=tris[i];
			ibuf_data[3*i]=t.a;
			ibuf_data[1+3*i]=t.b;
			ibuf_data[2+3*i]=t.c;
		}

		//send temp to "gpu"
		sg_buffer_desc ibuf_desc{};
		ibuf_desc.usage.vertex_buffer=false;
		ibuf_desc.usage.index_buffer=true;
		ibuf_desc.data.ptr=ibuf_data;
		ibuf_desc.data.size=sizeof(std::uint32_t)*num_indexes;
		ibuf=sg_make_buffer(ibuf_desc);

		//free temp
		delete[] ibuf_data;
	}

	float rayIntersectTri(const vf3d& orig, const vf3d& dir, const vf3d& t0, const vf3d& t1, const vf3d& t2,
		float* uptr = nullptr, float* vptr = nullptr)
	{
		static const float epsilon = 1e-6f;

		vf3d a = dir;
		vf3d b = t0 - t1;
		vf3d c = t0 - t2;
		vf3d d = t0 - orig;
		vf3d bxc = b.cross(c);
		float det = a.dot(bxc);

		//parallel
		if (std::abs(det) < epsilon) return -1;

		vf3d f = c.cross(a) / det;
		float u = f.dot(d);
		if (uptr) *uptr = u;

		vf3d g = a.cross(b) / det;
		float v = g.dot(d);
		if (vptr) *vptr = v;

		//within unit uv triangle
		if (u < 0 || u>1) return -1;
		if (v < 0 || v>1) return -1;
		if (u + v > 1) return -1;

		//get t
		vf3d e = bxc / det;
		float t = e.dot(d);

		//behind ray
		if (t < 0) return -1;

		return t;
	}

	static vf3d getClosePt(const vf3d& pt, const vf3d& t0, const vf3d& t1, const vf3d& t2)
	{
		vf3d ab = t1 - t0;
		vf3d ac = t2 - t0;

		vf3d ap = pt - t0;
		float d1 = ab.dot(ap);
		float d2 = ac.dot(ap);
		if (d1 <= 0 && d2 <= 0) return t0;

		vf3d bp = pt - t1;
		float d3 = ab.dot(bp);
		float d4 = ac.dot(bp);
		if (d3 >= 0 && d4 <= d3) return t1;

		vf3d cp = pt - t2;
		float d5 = ab.dot(cp);
		float d6 = ac.dot(cp);
		if (d6 >= 0 && d5 <= d6) return t2;

		float vc = d1 * d4 - d3 * d2;
		if (vc <= 0 && d1 >= 0 && d3 <= 0) {
			float v = d1 / (d1 - d3);
			return t0 + v * ab;
		}

		float vb = d5 * d2 - d1 * d6;
		if (vb <= 0 && d2 >= 0 && d6 <= 0) {
			float v = d2 / (d2 - d6);
			return t0 + v * ac;
		}

		float va = d3 * d6 - d5 * d4;
		if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
			float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return t1 + v * (t2 - t1);
		}

		float denom = 1 / (va + vb + vc);
		float v = vb * denom;
		float w = vc * denom;
		return t0 + v * ab + w * ac;

	}



	static Mesh makeCube() {
		Mesh m;

		m.verts={
			//back
			{{-1, -1, -1}, {0, 0, -1}, {1, 1}},
			{{1, -1, -1}, {0, 0, -1}, {0, 1}},
			{{-1, 1, -1}, {0, 0, -1}, {1, 0}},
			{{1, 1, -1}, {0, 0, -1}, {0, 0}},
			//front
			{{-1, -1, 1}, {0, 0, 1}, {0, 1}},
			{{1, -1, 1}, {0, 0, 1}, {1, 1}},
			{{-1, 1, 1}, {0, 0, 1}, {0, 0}},
			{{1, 1, 1}, {0, 0, 1}, {1, 0}},
			//left
			{{-1, -1, -1}, {-1, 0, 0}, {0, 1}},
			{{-1, 1, -1}, {-1, 0, 0}, {0, 0}},
			{{-1, -1, 1}, {-1, 0, 0}, {1, 1}},
			{{-1, 1, 1}, {-1, 0, 0}, {1, 0}},
			//right
			{{1, -1, -1}, {1, 0, 0}, {1, 1}},
			{{1, 1, -1}, {1, 0, 0}, {1, 0}},
			{{1, -1, 1}, {1, 0, 0}, {0, 1}},
			{{1, 1, 1}, {1, 0, 0}, {0, 0}},
			//bottom
			{{-1, -1, -1}, {0, -1, 0}, {0, 1}},
			{{-1, -1, 1}, {0, -1, 0}, {0, 0}},
			{{1, -1, -1}, {0, -1, 0}, {1, 1}},
			{{1, -1, 1}, {0, -1, 0}, {1, 0}},
			//top
			{{-1, 1, -1}, {0, 1, 0}, {0, 0}},
			{{-1, 1, 1}, {0, 1, 0}, {0, 1}},
			{{1, 1, -1}, {0, 1, 0}, {1, 0}},
			{{1, 1, 1}, {0, 1, 0}, {1, 1}}
		};

		m.tris={
			{0, 2, 1}, {1, 2, 3},//back
			{4, 5, 6}, {5, 7, 6},//front
			{8, 10, 9}, {9, 10, 11},//left
			{12, 13, 14}, {13, 15, 14},//right
			{16, 18, 17}, {17, 18, 19},//bottom
			{20, 21, 22}, {21, 23, 22}//top
		};

		m.updateVertexBuffer();
		m.updateIndexBuffer();

		return m;
	}

#pragma region SOLIDS OF REVOLUTION
	static Mesh makeTorus(float rad_xz, int num_xz, float rad_y, int num_y) {
		Mesh m;

		//note: loop order matters here.
		for(int i=0; i<num_xz; i++) {
			float u=i/(num_xz-1.f);
			float theta=2*Pi*u;

			//offset from big radius
			float dx=std::sin(theta);
			float ox=rad_xz*dx;
			float dz=std::cos(theta);
			float oz=rad_xz*dz;

			for(int j=0; j<num_y; j++) {
				float v=j/(num_y-1.f);
				float phi=2*Pi*v;

				//scale xz by little radius
				float dr=std::sin(phi);
				float nx=dx*dr;
				float ny=std::cos(phi);
				float nz=dz*dr;
				float x=ox+rad_y*nx;
				float z=oz+rad_y*nz;
				float y=rad_y*ny;

				m.verts.push_back({{x, y, z}, {nx, ny, nz}, {u, v}});
			}
		}

		//tessellate grid
		auto ix=[&] (int i, int j) { return j+num_y*i; };
		for(int i=1; i<num_xz; i++) {
			for(int j=1; j<num_y; j++) {
				const auto& a=ix(i-1, j-1);
				const auto& b=ix(i, j-1);
				const auto& c=ix(i-1, j);
				const auto& d=ix(i, j);
				m.tris.push_back({a, c, b});
				m.tris.push_back({b, c, d});
			}
		}

		m.updateVertexBuffer();
		m.updateIndexBuffer();

		return m;
	}

	static Mesh makeUVSphere(float rad, int num_xz, int num_y) {
		Mesh m;

		//note: loop order matters here.
		for(int i=0; i<num_xz; i++) {
			float u=i/(num_xz-1.f);
			float theta=2*Pi*u;

			float dx=std::sin(theta);
			float dz=std::cos(theta);

			for(int j=0; j<num_y; j++) {
				float v=j/(num_y-1.f);
				float phi=Pi*v;

				float dr=std::sin(phi);
				float nx=dx*dr;
				float ny=std::cos(phi);
				float nz=dz*dr;

				float x=rad*nx;
				float y=rad*ny;
				float z=rad*nz;

				m.verts.push_back({{x, y, z}, {nx, ny, nz}, {u, v}});
			}
		}

		//tessellate grid
		auto ix=[&] (int i, int j) { return j+num_y*i; };
		for(int i=1; i<num_xz; i++) {
			for(int j=1; j<num_y; j++) {
				const auto& a=ix(i-1, j-1);
				const auto& b=ix(i, j-1);
				const auto& c=ix(i-1, j);
				const auto& d=ix(i, j);
				m.tris.push_back({a, c, b});
				m.tris.push_back({b, c, d});
			}
		}

		m.updateVertexBuffer();
		m.updateIndexBuffer();

		return m;
	}

	static Mesh makeCylinder(float rad, int num, float hgt) {
		Mesh m;

		//toppole, topedge, topside, btmside, btmedge, btmpole

		float y_top=hgt/2, y_btm=-y_top;
		float v_top=std::atan(2*rad/hgt)/Pi, v_btm=1-v_top;

		//note: push_back order matters here.
		for(int i=0; i<num; i++) {
			float u=i/(num-1.f);
			float theta=2*Pi*u;

			float nx=std::sin(theta);
			float nz=std::cos(theta);

			float x=rad*nx;
			float z=rad*nz;

			m.verts.push_back({{0, y_top, 0}, {0, 1, 0}, {u, 0}});
			m.verts.push_back({{x, y_top, z}, {0, 1, 0}, {u, v_top}});
			m.verts.push_back({{x, y_top, z}, {nx, 0, nz}, {u, v_top}});
			m.verts.push_back({{x, y_btm, z}, {nx, 0, nz}, {u, v_btm}});
			m.verts.push_back({{x, y_btm, z}, {0, -1, 0}, {u, v_btm}});
			m.verts.push_back({{0, y_btm, 0}, {0, -1, 0}, {u, 1}});
		}

		auto ix_tp=[] (int i) { return 6*i; };
		auto ix_te=[] (int i) { return 1+6*i; };
		auto ix_ts=[] (int i) { return 2+6*i; };
		auto ix_bs=[] (int i) { return 3+6*i; };
		auto ix_be=[] (int i) { return 4+6*i; };
		auto ix_bp=[] (int i) { return 5+6*i; };

		//lid
		for(int i=0; i<num; i++) {
			m.tris.push_back({ix_tp(i), ix_te(i), ix_te((i+1)%num)});
		}

		//sides
		for(int i=0; i<num; i++) {
			int ts=ix_ts(i);
			int bs=ix_bs(i);
			int ts_next=ix_ts((i+1)%num);
			int bs_next=ix_bs((i+1)%num);
			m.tris.push_back({ts, bs, ts_next});
			m.tris.push_back({bs, bs_next, ts_next});
		}

		//base
		for(int i=0; i<num; i++) {
			m.tris.push_back({ix_bp(i), ix_be(i), ix_be((i+num-1)%num)});
		}

		m.updateVertexBuffer();
		m.updateIndexBuffer();

		return m;
	}

	static Mesh makeCone(float rad, int num, float hgt) {
		Mesh m;

		//topside, btmside, btmedge, btmpole

		float s=std::sqrt(rad*rad+hgt*hgt);
		float hs=hgt/s;
		float rs=rad/s;
		float ny=rad/s;

		float y_top=hgt/2, y_btm=-y_top;
		float v_btm=s/(s+rad);

		//note: push_back order matters here.
		for(int i=0; i<num; i++) {
			float u=i/(num-1.f);
			float theta=2*Pi*u;

			float dx=std::sin(theta);
			float dz=std::cos(theta);

			float nx=hs*dx;
			float nz=hs*dz;

			float x=rad*dx;
			float z=rad*dz;

			m.verts.push_back({{0, y_top, 0}, {nx, ny, nz}, {u, 0}});
			m.verts.push_back({{x, y_btm, z}, {nx, ny, nz}, {u, v_btm}});
			m.verts.push_back({{x, y_btm, z}, {0, -1, 0}, {u, v_btm}});
			m.verts.push_back({{0, y_btm, 0}, {0, -1, 0}, {u, 1}});
		}

		auto ix_ts=[] (int i) { return 4*i; };
		auto ix_bs=[] (int i) { return 1+4*i; };
		auto ix_be=[] (int i) { return 2+4*i; };
		auto ix_bp=[] (int i) { return 3+4*i; };

		//side
		for(int i=0; i<num; i++) {
			m.tris.push_back({ix_ts(i), ix_bs(i), ix_bs((i+1)%num)});
		}

		//base
		for(int i=0; i<num; i++) {
			m.tris.push_back({ix_bp(i), ix_be(i), ix_be((i+num-1)%num)});
		}

		m.updateVertexBuffer();
		m.updateIndexBuffer();

		return m;
	}
#pragma endregion

	[[nodiscard]] static ReturnCode loadFromOBJ(Mesh& m, const std::string& filename) {
		m=Mesh{};

		std::ifstream file(filename);
		if(file.fail()) return {false, "invalid filename"};

		std::vector<vf3d> vertex_pos;
		std::vector<vf3d> vertex_norm;
		std::vector<v2d_t> vertex_tex{{0, 0}};

		struct vtn_t {
			int v=0, t=0, n=0;

			bool operator==(const vtn_t& x) const {
				return x.v==v&&x.t==t&&x.n==n;
			}
		};

		struct vtn_t_hash {
			std::size_t operator()(const vtn_t& x) const {
				std::size_t v=std::hash<int>()(x.v);
				std::size_t t=std::hash<int>()(x.t);
				std::size_t n=std::hash<int>()(x.n);
				return v^(t<<1)^(n<<2);
			}
		};

		std::unordered_map<vtn_t, int, vtn_t_hash> vtn2ix;

		for(std::string line_str; std::getline(file, line_str);) {
			std::stringstream line_ss(line_str);

			std::string type; line_ss>>type;
			if(type=="v") {
				vf3d v;
				line_ss>>v.x>>v.y>>v.z;

				vertex_pos.push_back(v);
			} else if(type=="vn") {
				vf3d n;
				line_ss>>n.x>>n.y>>n.z;

				//ensure unit
				vertex_norm.push_back(n.norm());
			} else if(type=="vt") {
				v2d_t t;
				line_ss>>t.u>>t.v;

				vertex_tex.push_back(t);
			} else if(type=="f") {
				//parse vtns until fail
				std::vector<vtn_t> vtns;
				for(std::string vtn_str; line_ss>>vtn_str;) {
					std::stringstream vtn_ss(vtn_str);

					int v=-1, t=0, n=-1;

					std::string v_str;
					std::getline(vtn_ss, v_str, '/');
					std::stringstream v_ss(v_str);
					if(!(v_ss>>v)) v=-1;

					std::string t_str;
					if(std::getline(vtn_ss, t_str, '/')) {
						std::stringstream t_ss(t_str);
						if(!(t_ss>>t)) t=0;
					}

					std::string n_str;
					if(std::getline(vtn_ss, n_str, '/')) {
						std::stringstream n_ss(n_str);
						if(!(n_ss>>n)) n=-1;
					}

					if(v==-1) return {false, "invalid face vertex index"};
					if(n==-1) return {false, "invalid face normal index"};

					//obj 1-based indexing
					vtns.push_back({v-1, t, n-1});
				}

				//add vertexes
				std::vector<int> indexes;
				for(const auto& vtn:vtns) {
					//use vertex if it exists
					int ix=0;
					auto it=vtn2ix.find(vtn);
					if(it!=vtn2ix.end()) ix=it->second;
					//otherwise, make new one.
					else {
						ix=m.verts.size();
						vtn2ix.insert({vtn, ix});
						m.verts.push_back({
							vertex_pos[vtn.v],
							vertex_norm[vtn.n],
							vertex_tex[vtn.t]
							});
					}
					indexes.push_back(ix);
				}

				//triangulate polygon
				for(int i=2; i<indexes.size(); i++) {
					m.tris.push_back({
						indexes[0],
						indexes[i-1],
						indexes[i]
						});
				}
			}

		}
		
		file.close();

		m.updateVertexBuffer();
		m.updateIndexBuffer();

		return {true, "success"};
	}
};
#endif