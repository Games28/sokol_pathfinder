#pragma once
#ifndef LINEMESH_STRUCT_H
#define LINEMESH_STRUCT_H

#include "math/v3d.h"

#include <vector>

#include <set>

struct LineMesh {
	struct Vertex {
		vf3d pos;
		sg_color col;
	};
	std::vector<Vertex> verts;

	struct IndexLine {
		int a = 0, b = 0;

		IndexLine() {}

		//sorting constructor
		IndexLine(int a_, int b_) {
			if (a < b) a = a_, b = b_;
			else a = b_, b = a_;
		}

		//for set
		bool operator<(const IndexLine& l) const {
			if (l.a == a) return b < l.b;
			return a < l.a;
		}
	};
	std::vector<IndexLine> lines;

	sg_buffer vbuf{ SG_INVALID_ID };
	sg_buffer ibuf{ SG_INVALID_ID };

	void updateVertexBuffer() {
		//free old
		if (vbuf.id != SG_INVALID_ID) sg_destroy_buffer(vbuf);

		//send data to "gpu"
		sg_buffer_desc vbuf_desc{};
		vbuf_desc.usage.index_buffer = false;
		vbuf_desc.usage.vertex_buffer = true;
		vbuf_desc.data.ptr = verts.data();
		vbuf_desc.data.size = sizeof(Vertex) * verts.size();
		vbuf = sg_make_buffer(vbuf_desc);
	}

	void updateIndexBuffer() {
		//free old
		if (ibuf.id != SG_INVALID_ID) sg_destroy_buffer(ibuf);

		//alloc temp
		const int num_lines = lines.size();
		const int num_indexes = 2 * num_lines;
		std::uint32_t* ibuf_data = new std::uint32_t[num_indexes];

		//transfer to temp
		for (int i = 0; i < num_lines; i++) {
			const auto& l = lines[i];
			ibuf_data[2 * i] = l.a;
			ibuf_data[1 + 2 * i] = l.b;
		}

		//send temp to "gpu"
		sg_buffer_desc ibuf_desc{};
		ibuf_desc.usage.vertex_buffer = false;
		ibuf_desc.usage.index_buffer = true;
		ibuf_desc.data.ptr = ibuf_data;
		ibuf_desc.data.size = sizeof(std::uint32_t) * num_indexes;
		ibuf = sg_make_buffer(ibuf_desc);

		//free temp
		delete[] ibuf_data;
	}

	void randomizeColors() {
		for (auto& v : verts) {
			v.col = { randFloat(), randFloat(), randFloat(), 1 };
		}
	}

	static LineMesh makeFromMesh(const Mesh& m) {
		LineMesh l;

		//copy over vertexes as white
		const sg_color white{ 1, 1, 1, 1 };
		for (const auto& v : m.verts) {
			l.verts.push_back({ v.pos, white });
		}

		//convert tris to lines
		//  with uniqueness in mind
		std::set<IndexLine> line_set;
		for (const auto& t : m.tris) {
			//i dont like using emplace
			//  b/c of the compiler warnings
			line_set.insert(IndexLine(t.a, t.b));
			line_set.insert(IndexLine(t.b, t.c));
			line_set.insert(IndexLine(t.c, t.a));
		}

		//copy set to vector
		l.lines.insert(l.lines.begin(),
			line_set.begin(), line_set.end()
		);

		l.updateVertexBuffer();
		l.updateIndexBuffer();

		return l;
	}
};
#endif
