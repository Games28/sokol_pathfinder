#include "linemesh.h"
struct AABB3 {
   
    vf3d min{ INFINITY,INFINITY,INFINITY }, max = -min;

    vf3d getMin() const
    {
        return min;
    }

    vf3d getMax() const
    {
        return max;
    }

    vf3d getCenter() const
    {
        return (min + max) / 2;
    }

  


    bool contains(const vf3d& p) const
    {
        if (p.x<min.x || p.x>max.x) return false;
        if (p.y<min.y || p.y>max.y) return false;
        if (p.z<min.z || p.z>max.z) return false;
        return true;
    }

    bool overlaps(const AABB3& o) const {
        if (min.x > o.max.x || max.x < o.min.x) return false;
        if (min.y > o.max.y || max.y < o.min.y) return false;
        if (min.z > o.max.z || max.z < o.min.z) return false;
        return true;
    }

    void fitToEnclose(const vf3d& p) {
        if (p.x < min.x) min.x = p.x;
        if (p.y < min.y) min.y = p.y;
        if (p.z < min.z) min.z = p.z;
        if (p.x > max.x) max.x = p.x;
        if (p.y > max.y) max.y = p.y;
        if (p.z > max.z) max.z = p.z;
    }

} aabb3;


bool rayIntersectBox(const vf3d& orig, const vf3d& dir, const AABB3& box) {
    const float epsilon = 1e-6f;
    float tmin = -INFINITY;
    float tmax = INFINITY;

    //x axis
    if (std::abs(dir.x) > epsilon) {
        float inv_d = 1 / dir.x;
        float t1 = inv_d * (box.min.x - orig.x);
        float t2 = inv_d * (box.max.x - orig.x);
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
    }
    else if (orig.x<box.min.x || orig.x>box.max.x) return false;

    //y axis
    if (std::abs(dir.y) > epsilon) {
        float inv_d = 1 / dir.y;
        float t3 = inv_d * (box.min.y - orig.y);
        float t4 = inv_d * (box.max.y - orig.y);
        if (t3 > t4) std::swap(t3, t4);
        tmin = std::max(tmin, t3);
        tmax = std::min(tmax, t4);
    }
    else if (orig.y<box.min.y || orig.y>box.max.y) return false;

    //z axis
    if (std::abs(dir.z) > epsilon) {
        float inv_d = 1 / dir.z;
        float t5 = inv_d * (box.min.z - orig.z);
        float t6 = inv_d * (box.max.z - orig.z);
        if (t5 > t6) std::swap(t5, t6);
        tmin = std::max(tmin, t5);
        tmax = std::min(tmax, t6);
    }
    else if (orig.z<box.min.z || orig.z>box.max.z) return false;

    if (tmax<0 || tmin>tmax) return false;

    return true;
}


struct
{
    LineMesh linemesh;
    mat4 model;
}aabb_render;

void setupAABBRender() {
    LineMesh l;

    sg_color white{ 1, 1, 1, 1 };
    l.verts = {
        {{0, 0, 0}, white},
        {{1, 0, 0}, white},
        {{0, 1, 0}, white},
        {{1, 1, 0}, white},
        {{0, 0, 1}, white},
        {{1, 0, 1}, white},
        {{0, 1, 1}, white},
        {{1, 1, 1}, white}
    };

    l.updateVertexBuffer();

    l.lines = {
        {0, 1},
        {0, 2},
        {0, 4},
        {1, 3},
        {1, 5},
        {2, 3},
        {2, 6},
        {3, 7},
        {4, 5},
        {4, 6},
        {5, 7},
        {6, 7}
    };

    l.updateIndexBuffer();

    aabb_render.linemesh = l;
}




void renderAABB(sg_pipeline& line_pip,const vf3d& a, const vf3d& b, mat4 view_proj) {
    mat4 trans = mat4::makeTranslation(a);
    mat4 scl = mat4::makeScale(b - a);
    mat4 model = mat4::mul(trans, scl);

    sg_apply_pipeline(line_pip);

    sg_bindings bind{};
    bind.vertex_buffers[0] = aabb_render.linemesh.vbuf;
    bind.index_buffer = aabb_render.linemesh.ibuf;
    sg_apply_bindings(bind);

    vs_line_params_t vs_line_params{};
    mat4 mvp = mat4::mul(view_proj, model);
    std::memcpy(vs_line_params.u_mvp, mvp.m, sizeof(mvp.m));
    sg_apply_uniforms(UB_vs_line_params, SG_RANGE(vs_line_params));

    sg_draw(0, 2 * aabb_render.linemesh.lines.size(), 1);
}
