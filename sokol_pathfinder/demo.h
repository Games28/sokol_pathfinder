#define SOKOL_GLCORE
#include "sokol_engine.h"
#include "sokol/sokol_gfx.h"
#include "sokol/sokol_glue.h"
#include <iostream>

#include "shd.glsl.h"

#include "math/v3d.h"
#include "v2d.h"
#include "math/mat4.h"

#include "mesh.h"
#include "AABB.h"
#include "AABB3.h"
#include "poisson_disc.h"
#include "Triangulate.h"
#include "Graph.h"

//for time
#include <ctime>

#include "texture_utils.h"
#include "Object.h"

#define MAX_PARTICLES (512 * 1024)
#define NUM_PARTICLES_EMITTED_PER_FRAME (10)

//y p => x y z
//0 0 => 0 0 1
static vf3d polar3D(float yaw, float pitch) {
	return {
		std::sin(yaw)*std::cos(pitch),
		std::sin(pitch),
		std::cos(yaw)*std::cos(pitch)
	};
}


struct
{
	vf3d pos{ 0,2,2 };
	vf3d dir;
	float yaw = 0;
	float pitch = 0;
	mat4 proj, view;
	mat4 view_proj;
}cam;

struct Light
{
	vf3d pos;
	sg_color col;

};

struct NodeInfo
{
	vf3d translation, rotation, scale{ 1, 1, 1 };
	mat4 model = mat4::makeIdentity();
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
};

struct Demo : SokolEngine {
	sg_pipeline default_pip{};
	sg_pipeline line_pip{};
	
	Graph graph;
	sg_sampler sampler{};
	bool render_outlines = false;

	mat4 cam_view_proj;

	std::vector<Light> lights;
	Light* mainlight;

	std::vector<Object> objects;
	std::vector<Object> Nodes;
	std::vector<NodeInfo> nodeinfo;

	LineMesh nodelines;

	Node* from_node = nullptr;
	Node* to_node = nullptr;
	Object bb_Node;
	float node_dt = 0;
 	
	const std::vector<std::string> Structurefilenames{
		"assets/models/desert.txt",
		"assets/models/house.txt",
	};

	const std::vector<std::string> texturefilenames{
		"assets/poust_1.png",
		//"assets/sandtexture.png",
		"assets/sandtexture.png",
		
	};

	sg_view tex_blank{};
	sg_view tex_uv{};

	sg_view gui_image{};

	sg_pass_action display_pass_action{};

	Object platform;

	

	static inline uint32_t xorshift32(void) {
		static uint32_t x = 0x12345678;
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		return x;
	}

#pragma region SETUP HELPERS
	void setupEnvironment() {
		sg_desc desc{};
		desc.environment=sglue_environment();
		sg_setup(desc);
	}

	void setupLights()
	{
		//white
		lights.push_back({ {-1,3,1},{1,1,1,1} });
		mainlight = &lights.back();
		
	}
	
	//primitive textures to debug with
	void setupTextures() {
		tex_blank=makeBlankTexture();
		tex_uv=makeUVTexture(512, 512);
	}

	//if texture loading fails, default to uv tex.
	sg_view getTexture(const std::string& filename) {
		sg_view tex;
		auto status=makeTextureFromFile(tex, filename);
		if(!status.valid) tex=tex_uv;
		return tex;
	}

	void setupSampler() {
		sg_sampler_desc sampler_desc{};
		sampler=sg_make_sampler(sampler_desc);
	}

	void setupLinePipeline()
	{
		sg_pipeline_desc pip_desc{};
		pip_desc.layout.attrs[ATTR_line_v_pos].format = SG_VERTEXFORMAT_FLOAT3;
		pip_desc.layout.attrs[ATTR_line_v_col].format = SG_VERTEXFORMAT_FLOAT4;
		pip_desc.shader = sg_make_shader(line_shader_desc(sg_query_backend()));
		pip_desc.primitive_type = SG_PRIMITIVETYPE_LINES;
		pip_desc.index_type = SG_INDEXTYPE_UINT32;
		pip_desc.depth.write_enabled = true;
		pip_desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
		line_pip = sg_make_pipeline(pip_desc);
	}

	void setupObjects()
	{
		std::vector<vf3d> coords
		{
			{0.0f, 0.0f, 0.0f},
			//{-4.0f, 1.67f, 4.0f}, 
			//{51.31f, 1.67f, 41.95f}, 
			//{62.04f, 1.67f, -5.12f}, 
			//{12.09f, 2.16f, -38.03f},
			//{-38.35f, 2.63f, -33.38f},
			//{-52.99f, 1.43f, 16.84f},
		};

		std::vector<std::uint32_t> colors
		{
			0xFFFFFFFF,  //white
			0xFF0000ff,  //blue
			0xFFffFFff,   //white
			0xFFff0000,   //red
			0xFFffffff,   //white
			0xFF00FF00,   // green
			0xFF0000ff,   //blue
		};

	

		for (int i = 0; i < coords.size(); i++)
		{
			Object b;
			Mesh& m = b.mesh;
			auto status = Mesh::loadFromOBJ(m, Structurefilenames[1]);
			if (!status.valid) m = Mesh::makeCube();
			 b.scale = { 1,1,1 };
			
			b.translation = coords[i];

			b.updateMatrixes();
			b.tex = makeColorTexture(colors[i]);
			objects.push_back(Object(m,b.tex));

		}
	}

	void setupPlatform() {
		Object b;
		Mesh& m = b.mesh;
		auto status = Mesh::loadFromOBJ(m, Structurefilenames[0]);
		if (!status.valid) m = Mesh::makeCube();
		b.scale = { 1,1,1 };

		b.translation = { 2,0,-2 };

		b.updateMatrixes();
		b.tex = getTexture(texturefilenames[0]);
		platform = Object(m,b.tex);
	}

	void setupBillboard() {
		Object obj;
		Mesh& m=obj.mesh;
		m.verts={
			{{-.5f, .5f, 0}, {0, 0, 1}, {0, 0}},//tl
			{{.5f, .5f, 0}, {0, 0, 1}, {1, 0}},//tr
			{{-.5f, -.5f, 0}, {0, 0, 1}, {0, 1}},//bl
			{{.5f, -.5f, 0}, {0, 0, 1}, {1, 1}}//br
		};
		m.tris={
			{0, 2, 1},
			{1, 2, 3}
		};
		m.updateVertexBuffer();
		m.updateIndexBuffer();

		obj.translation={0, 5, 0};
		obj.isbillboard = true;
		obj.draggable = true;

		obj.tex=getTexture("assets/spritesheet.png");
		obj.num_x=4, obj.num_y=4;
		obj.num_ttl= obj.num_x*obj.num_y;
		objects.push_back(obj);
	}

	void setupNodeBillboards()
	{
		int count = 0;

		//for (auto& n : graph.nodes)
		{
			
			Object obj;
			Mesh& m = obj.mesh;
			m.verts = {
				{{-.5f, .5f, 0}, {0, 0, 1}, {0, 0}},//tl
				{{.5f, .5f, 0}, {0, 0, 1}, {1, 0}},//tr
				{{-.5f, -.5f, 0}, {0, 0, 1}, {0, 1}},//bl
				{{.5f, -.5f, 0}, {0, 0, 1}, {1, 1}}//br
			};
			m.tris = {
				{0, 2, 1},
				{1, 2, 3}
			};
			m.updateVertexBuffer();
			m.updateIndexBuffer();
			obj.scale = { 0.4f,0.4f,0.4f };
			obj.translation = { 0,0,0 };
			obj.isbillboard = true;
			obj.updateMatrixes();
			obj.tex = tex_uv;
			obj.num_x = 4, obj.num_y = 4;
			obj.num_ttl = obj.num_x * obj.num_y;
			//Nodes.push_back(obj);
			bb_Node = obj;
			
		}
		
		
	}

	void setupNodeLineLinks ()
	{
		std::unordered_map<Node*, int> lookup;
		int i = 0;
		nodelines.verts.clear();
		for (const auto& n : graph.nodes)
		{
			LineMesh::Vertex v1;
			v1.pos = nodeinfo[i].translation;
			v1.col = { 1, 1, 1, 1 };
			nodelines.verts.push_back(v1);
			lookup[n] = i++;
		}
		int u = 0;
		nodelines.updateVertexBuffer();

		nodelines.lines.clear();
		for (const auto& n : graph.nodes)
		{
			for (const auto& o : n->links)
			{
				nodelines.lines.push_back(LineMesh::IndexLine(lookup[n], lookup[o]));
			}
		}
		nodelines.updateIndexBuffer();

	}

	float rayIntersectNode(const vf3d& orig, const vf3d& dir, const vf3d& t0, const vf3d& t1, const vf3d& t2,
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

	void graphToNode()
	{
		for (auto& g : graph.nodes)
		{
			NodeInfo node;
			node.translation = g->pos;
			node.scale = { 0.4f,0.4f, 0.4f };
			node.updateMatrixes();
			nodeinfo.push_back(node);
		}
	}

	//clear to bluish
	void setupDisplayPassAction() {
		display_pass_action.colors[0].load_action=SG_LOADACTION_CLEAR;
		display_pass_action.colors[0].clear_value={.25f, .45f, .65f, 1.f};
	}

	void setupDefaultPipeline() {
		sg_pipeline_desc pipeline_desc{};
		pipeline_desc.layout.attrs[ATTR_default_v_pos].format=SG_VERTEXFORMAT_FLOAT3;
		pipeline_desc.layout.attrs[ATTR_default_v_norm].format=SG_VERTEXFORMAT_FLOAT3;
		pipeline_desc.layout.attrs[ATTR_default_v_uv].format=SG_VERTEXFORMAT_FLOAT2;
		pipeline_desc.shader=sg_make_shader(default_shader_desc(sg_query_backend()));
		pipeline_desc.index_type=SG_INDEXTYPE_UINT32;
		pipeline_desc.cull_mode=SG_CULLMODE_FRONT;
		pipeline_desc.depth.write_enabled=true;
		pipeline_desc.depth.compare=SG_COMPAREFUNC_LESS_EQUAL;
		default_pip=sg_make_pipeline(pipeline_desc);
	}

	float nodeBasedintersectRay(Object& obj, const vf3d& orig_local, const vf3d& dir_local)
	{
		float w = 1;
		
		float record = -1;
		for (const auto& t : obj.mesh.tris) {
			float w = 1;
			
			vf3d p0 = matMulVec(obj.model, obj.mesh.verts[t.a].pos, w);
			vf3d p1 = matMulVec(obj.model, obj.mesh.verts[t.b].pos, w);
			vf3d p2 = matMulVec(obj.model, obj.mesh.verts[t.c].pos, w);
			float dist = obj.mesh.rayIntersectTri(
				orig_local,
				dir_local,
				p0,
				p1,
				p2
			);

			
			if (dist > 0)
			{
				//sort while iterating
				if (record < 0 || dist < record) record = dist;
			}


		}

		return record;
	}

	bool contains(Object& obj, const vf3d& orig_local)
	{
		vf3d dir = vf3d(
			.5f - randFloat(),
			.5f - randFloat(),
			.5f - randFloat()
		).norm();

		
		float w = 1;
		int num = 0;
		for (const auto& t : obj.mesh.tris) {
			
			vf3d p0 = matMulVec(obj.model, obj.mesh.verts[t.a].pos, w);
			vf3d p1 = matMulVec(obj.model, obj.mesh.verts[t.b].pos, w);
			vf3d p2 = matMulVec(obj.model, obj.mesh.verts[t.c].pos, w);

			float dist = rayIntersectNode(
				orig_local,
				dir,
				p0,
				p1,
				p2
			);


			if (dist > 0)
			{
				num++;
			}


		}
		return num % 2;
	}

	//setup nodes, aabb of terrain, triangluation
	void setupNodes()
	{
		Object& terrian = platform;
		AABB3 bounds = terrian.getAABB();
		auto xz_pts = poissonDiscSample({ {bounds.min.x,bounds.min.z}, {bounds.max.x,bounds.max.z} }, 2);

	
		//project pts on to terrain
		std::unordered_map<vf2d*, Node*> xz2way;
		for (auto& p : xz_pts)
		{
			vf3d orig(p.x, bounds.min.y - .1f, p.y);
			vf3d dir(0, 1, 0);
			float dist = nodeBasedintersectRay(terrian, orig,dir);
			graph.nodes.push_back(new Node(orig + (.2f + dist) * dir));
			xz2way[&p] = graph.nodes.back();
		}

		//triangulate and add links
		auto tris = delaunay::triangulate(xz_pts);
		auto edges = delaunay::extractEdges(tris);
		for (const auto& e : edges) {
			auto a = xz2way[&xz_pts[e.p[0]]];
			auto b = xz2way[&xz_pts[e.p[1]]];
			graph.addLink(a, b);
			graph.addLink(b, a);
		}


		/// check for finishing errors when objects are added
		//remove any nodes in the way of objects
		for (auto it = graph.nodes.begin(); it != graph.nodes.end();)
		{
			auto& n = *it;
			//check if inside any meshes
			bool blocked = false;
			for (auto& obj : objects)
			{
				if (contains(obj, n->pos))
				{
					blocked = true;
					break;
				}
			}
			if (blocked)
			{
				//remove corresponding links
				for (auto& o : graph.nodes)
				{
					auto oit = std::find(o->links.begin(), o->links.end(), n);
					if (oit != o->links.end()) o->links.erase(oit);
				}
		
				delete n;
				it = graph.nodes.erase(it);
			}
			else
			{
				it++;
			}
		
		}

		
	}




#pragma endregion

	void userCreate() override {
		setupEnvironment();

		setupLinePipeline();

		setupTextures();

		setupSampler();

		setupLights();
		
		setupPlatform();
		
		setupObjects();

		setupNodes();
		graphToNode();

		setupNodeLineLinks();

		setupNodeBillboards();

		

		setupBillboard();

		
		setupDisplayPassAction();

		setupDefaultPipeline();
	}

#pragma region UPDATE HELPERS
	
	void updateCameraMatrixes() {
		mat4 look_at = mat4::makeLookAt(cam.pos, cam.pos + cam.dir, { 0, 1, 0 });
		cam.view = mat4::inverse(look_at);

		//cam proj can change with window resize
		float asp = sapp_widthf() / sapp_heightf();
		cam.proj = mat4::makePerspective(90, asp, .001f, 1000.f);

		cam.view_proj = mat4::mul(cam.proj, cam.view);
	}

	void handleCameraLooking(float dt) {
		//left/right
		if(getKey(SAPP_KEYCODE_LEFT).held) cam.yaw+=dt;
		if(getKey(SAPP_KEYCODE_RIGHT).held) cam.yaw-=dt;

		//up/down
		if(getKey(SAPP_KEYCODE_UP).held) cam.pitch+=dt;
		if(getKey(SAPP_KEYCODE_DOWN).held) cam.pitch-=dt;

		//clamp camera pitch
		if(cam.pitch>Pi/2) cam.pitch=Pi/2-.001f;
		if(cam.pitch<-Pi/2) cam.pitch=.001f-Pi/2;

		
	}

	void handleCameraMovement(float dt) {

		
		
		//move up, down
		if (getKey(SAPP_KEYCODE_SPACE).held) cam.pos.y += 4.f * dt;
		if (getKey(SAPP_KEYCODE_LEFT_SHIFT).held) cam.pos.y -= 4.f * dt;
		

		//move forward, backward
		vf3d fb_dir(std::sin(cam.yaw), 0, std::cos(cam.yaw));
		if(getKey(SAPP_KEYCODE_W).held) cam.pos+=5.f*dt*fb_dir;
		if(getKey(SAPP_KEYCODE_S).held) cam.pos-=3.f*dt*fb_dir;

		//move left, right
		vf3d lr_dir(fb_dir.z, 0, -fb_dir.x);
		if(getKey(SAPP_KEYCODE_A).held) cam.pos+=4.f*dt*lr_dir;
		if(getKey(SAPP_KEYCODE_D).held) cam.pos-=4.f*dt*lr_dir;

	}

	void handleUserInput(float dt) {
		handleCameraLooking(dt);
		//polar to cartesian
		cam.dir=polar3D(cam.yaw, cam.pitch);
		if (getKey(SAPP_KEYCODE_R).held) mainlight->pos = cam.pos;
		//toggle shape outlines
		if (getKey(SAPP_KEYCODE_O).pressed) render_outlines ^= true;

		handleCameraMovement(dt);
	}


	//make billboard always point at camera.
	void updateBillboard(Object& obj, float dt) {
		//move with player 
		vf3d eye_pos= obj.translation;
		vf3d target=cam.pos;

		vf3d y_axis(0, 1, 0);
		vf3d z_axis=(target-eye_pos).norm();
		vf3d x_axis=y_axis.cross(z_axis).norm();
		y_axis=z_axis.cross(x_axis);
		
		//slightly different than makeLookAt.
		mat4& m= obj.model;
		m(0, 0)=x_axis.x, m(0, 1)=y_axis.x, m(0, 2)=z_axis.x, m(0, 3)=eye_pos.x;
		m(1, 0)=x_axis.y, m(1, 1)=y_axis.y, m(1, 2)=z_axis.y, m(1, 3)=eye_pos.y;
		m(2, 0)=x_axis.z, m(2, 1)=y_axis.z, m(2, 2)=z_axis.z, m(2, 3)=eye_pos.z;
		m(3, 3)=1;
		
		float angle = atan2f(z_axis.z, z_axis.x);
		//
		//int i = 0;
		//
		if (angle < -0.70 && angle > -2.35 )
		{
			obj.anim = 1; //front
		}
		if (angle < -2.35 && angle < 2.35)
		{
			obj.anim = 4; //left
		}
		if (angle > -0.70 && angle < 0.70)
		{
			obj.anim = 8; //right
		}
		if (angle > 0.70 && angle < 2.35) 
		{
			obj.anim = 12; //back
		}
		//obj.anim_timer-=dt;
		//if(obj.anim_timer<0) {
		//	obj.anim_timer+=.5f;
		//
		//	//increment animation index and wrap
		//	obj.anim++;
		//	obj.anim%=obj.num_ttl;
		//}
	}

	void updateNodeBillboard(NodeInfo& node, float dt) {
		//move with player 
		vf3d eye_pos = node.translation;
		vf3d target = cam.pos;

		vf3d y_axis(0, 1, 0);
		vf3d z_axis = (target - eye_pos).norm();
		vf3d x_axis = y_axis.cross(z_axis).norm();
		y_axis = z_axis.cross(x_axis);

		//slightly different than makeLookAt.
		mat4& m = node.model;
		m(0, 0) = x_axis.x, m(0, 1) = y_axis.x, m(0, 2) = z_axis.x, m(0, 3) = eye_pos.x;
		m(1, 0) = x_axis.y, m(1, 1) = y_axis.y, m(1, 2) = z_axis.y, m(1, 3) = eye_pos.y;
		m(2, 0) = x_axis.z, m(2, 1) = y_axis.z, m(2, 2) = z_axis.z, m(2, 3) = eye_pos.z;
		m(3, 3) = 1;

		
	}

	

	
	

#pragma endregion

	void userUpdate(float dt) {
		
		handleUserInput(dt);
		
	
		updateCameraMatrixes();

		for (auto& obj : objects)
		{
			if (obj.isbillboard)
			{
				updateBillboard(obj, dt);
				
			}
			
		}

		if (render_outlines)
		{
			renderObjectOutlines();
		}

		for (auto& n : nodeinfo)
		{
			n.updateMatrixes();
			updateNodeBillboard(n, dt);
			
		}
		
	}

#pragma region RENDER HELPERS

	void renderNodes(Object& obj, const mat4& view_proj)
	{
		sg_apply_pipeline(default_pip);
		sg_bindings bind{};
		bind.vertex_buffers[0] = obj.mesh.vbuf;
		bind.index_buffer = obj.mesh.ibuf;
		bind.samplers[SMP_default_smp] = sampler;
		bind.views[VIEW_default_tex] = obj.tex;
		sg_apply_bindings(bind);

		//pass transformation matrix
		mat4 mvp = mat4::mul(view_proj, obj.model);
		vs_params_t vs_params{};
		std::memcpy(vs_params.u_mvp, mvp.m, sizeof(mvp.m));
		sg_apply_uniforms(UB_vs_params, SG_RANGE(vs_params));

		//which region of texture to sample?

		fs_params_t fs_params{};
		fs_params.u_tl[0] = 0, fs_params.u_tl[1] = 0;
		fs_params.u_br[0] = 1, fs_params.u_br[1] = 1;
		sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));

		sg_draw(0, 3 * obj.mesh.tris.size(), 1);
	}

	void node_billboard_render(NodeInfo& node,const mat4& view_proj)
	{
		sg_apply_pipeline(default_pip);
		sg_bindings bind{};
		bind.vertex_buffers[0] = bb_Node.mesh.vbuf;
		bind.index_buffer = bb_Node.mesh.ibuf;
		bind.samplers[SMP_default_smp] = sampler;
		bind.views[VIEW_default_tex] = bb_Node.tex;
		sg_apply_bindings(bind);

		//pass transformation matrix
		mat4 mvp = mat4::mul(view_proj, node.model);
		vs_params_t vs_params{};
		std::memcpy(vs_params.u_mvp, mvp.m, sizeof(mvp.m));
		sg_apply_uniforms(UB_vs_params, SG_RANGE(vs_params));

		//which region of texture to sample?

		fs_params_t fs_params{};
		fs_params.u_tl[0] = 0, fs_params.u_tl[1] = 0;
		fs_params.u_br[0] = 1, fs_params.u_br[1] = 1;
		sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));

		sg_draw(0, 3 * bb_Node.mesh.tris.size(), 1);
	}

	void renderObjects(Object& obj,const mat4& view_proj) {
		sg_apply_pipeline(default_pip);
		sg_bindings bind{};
		bind.vertex_buffers[0]=obj.mesh.vbuf;
		bind.index_buffer= obj.mesh.ibuf;
		bind.samplers[SMP_default_smp]=sampler;
		bind.views[VIEW_default_tex] = obj.tex;
		
		sg_apply_bindings(bind);

		//pass transformation matrix
		mat4 mvp=mat4::mul(view_proj, obj.model);
		vs_params_t vs_params{};
		std::memcpy(vs_params.u_model, obj.model.m, sizeof(vs_params.u_model));
		std::memcpy(vs_params.u_mvp, mvp.m, sizeof(mvp.m));
		sg_apply_uniforms(UB_vs_params, SG_RANGE(vs_params));

		//render entire texture.
		//fs_params_t fs_params{};
		//lighting test
		fs_params_t fs_params{};
		{

			fs_params.u_num_lights = lights.size();
			int idx = 0;
			for (const auto& l : lights)
			{
				fs_params.u_light_pos[idx][0] = l.pos.x;
				fs_params.u_light_pos[idx][1] = l.pos.y;
				fs_params.u_light_pos[idx][2] = l.pos.z;
				fs_params.u_light_col[idx][0] = l.col.r;
				fs_params.u_light_col[idx][1] = l.col.g;
				fs_params.u_light_col[idx][2] = l.col.b;
				idx++;
			}
		}

		fs_params.u_view_pos[0] = cam.pos.x;
		fs_params.u_view_pos[1] = cam.pos.y;
		fs_params.u_view_pos[2] = cam.pos.z;
		//sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));


		fs_params.u_tl[0]=0, fs_params.u_tl[1]=0;
		fs_params.u_br[0]=1, fs_params.u_br[1]=1;
		sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));

		sg_draw(0, 3* obj.mesh.tris.size(), 1);
	}

	

	void renderObjectOutlines()
	{
		
		sg_apply_pipeline(line_pip);
		
		sg_bindings bind{};
		bind.vertex_buffers[0] = nodelines.vbuf;
		bind.index_buffer = nodelines.ibuf;
		sg_apply_bindings(bind);
		
		vs_line_params_t vs_line_params{};
		mat4 mvp = mat4::mul(cam.view_proj, mat4::makeIdentity());
		std::memcpy(vs_line_params.u_mvp, mvp.m, sizeof(vs_line_params.u_mvp));
		sg_apply_uniforms(UB_vs_line_params, SG_RANGE(vs_line_params));
		
		sg_draw(0, 2 * nodelines.lines.size(), 1);
		
		 
	}
	

#pragma endregion
	
	void userRender() {
		sg_pass pass{};
		pass.action=display_pass_action;
		pass.swapchain=sglue_swapchain();
		sg_begin_pass(pass);

		//camera transformation matrix
		mat4 look_at=mat4::makeLookAt(cam.pos, cam.pos+cam.dir, {0, 1, 0});
		
		mat4 cam_view=mat4::inverse(look_at);


		//perspective
		mat4 cam_proj=mat4::makePerspective(90.f, sapp_widthf()/sapp_heightf(), .001f, 1000);
		
		//premultiply transform
		 cam_view_proj=mat4::mul(cam_proj, cam_view);

		sg_apply_pipeline(default_pip);

		
		renderObjects(platform, cam.view_proj);

		for (auto& obj : objects)
		{
			
			renderObjects(obj, cam.view_proj);
			
		}
		

		for (auto& n : nodeinfo)
		{
			node_billboard_render(n, cam.view_proj);
		}

		renderObjectOutlines();
		
		sg_end_pass();
		
		sg_commit();
	}
};