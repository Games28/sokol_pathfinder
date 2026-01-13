#pragma once
#ifndef SHAPE_STRUCT_H
#define SHAPE_STRUCT_H

#include "mesh.h"

#include "linemesh.h"

#include "math/mat4.h"
#include "AABB3.h"

struct Object {
	Mesh mesh;

	LineMesh linemesh;

	sg_view tex{};
	
	bool draggable = false;
	bool isbillboard = false;

	vf3d translation, rotation, scale{ 1, 1, 1 };

	mat4 model = mat4::makeIdentity();
	int num_x = 0, num_y = 0;
	int num_ttl = 0;

	float anim_timer = 0;
	int anim = 0;

	Object() {}

	Object(const Mesh& m, sg_view t) {
		mesh = m;
		linemesh = LineMesh::makeFromMesh(m);
		linemesh.randomizeColors();
		linemesh.updateVertexBuffer();
		tex = t;
	}

	void updateMatrixes() {
		//xyz euler angles?
		mat4 rot_x = mat4::makeRotX(rotation.x);
		mat4 rot_y = mat4::makeRotY(rotation.y);
		mat4 rot_z = mat4::makeRotZ(rotation.z);
		mat4 rot = mat4::mul(rot_z, mat4::mul(rot_y, rot_x));

		mat4 scl = mat4::makeScale(scale);

		mat4 trans = mat4::makeTranslation(translation);

		//combine & invert
		model = mat4::mul(trans, mat4::mul(rot, scl));
	}

	AABB3 getAABB() const
	{
		float w = 1;
		AABB3 box;
		for (const auto& v : mesh.verts)
		{
			box.fitToEnclose(matMulVec(model ,v.pos, w));
		}

		return box;
	}
};
#endif
