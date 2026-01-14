#pragma once
#include <list>
#include <vector>
#include <set>

namespace delaunay
{
	struct Triangle
	{
		int p[3]{ 0,0,0 };

		Triangle() {}

		Triangle(int a, int b, int c)
		{
			if (a > b) std::swap(a, b);
			if (b > c) std::swap(b, c);
			if (a > b) std::swap(a, b);
			p[0] = a, p[1] = b, p[2] = c;
		}

		bool contains(int a) const
		{
			for (int i = 0; i < 3; i++)
			{
				if (p[i] == a) return true;
			}
			return false;
		}

		bool equals(const Triangle& t) const
		{
			for (int i = 0; i < 3; i++)
			{
				if (t.p[i] != p[i]) return false;
			}

			return true;
		}

		void getEdge(int e, int& p0, int& p1) const
		{
			p0 = p[e], p1 = p[(e + 1) % 3];
		}

		static bool edgeShared(
			const Triangle& t0, const int& e0,
			const Triangle& t1, const int& e1)
		{
			int v00 = 0, v01 = 0, v10 = 0, v11 = 0;
			t0.getEdge(e0, v00, v01);
			t1.getEdge(e1, v10, v11);
			if (v00 == v10 && v01 == v11) return true;
			if (v00 == v11 && v01 == v10) return true;
			return false;
		}


	};

	struct Edge
	{
		int p[2]{ 0,0 };

		Edge() {}

		Edge(int a, int b)
		{
			if (a > b) std::swap(a, b);
			p[0] = a, p[1] = b;
		}

		bool operator<(const Edge& e) const
		{
			if (p[0] == e.p[0]) return p[1] < e.p[1];
			return p[0] < e.p[0];
		}

	};

	std::list<Triangle> triangulate( const std::vector<vf2d>& pts_ref)
	{
		std::list<Triangle> tris;
		std::vector<vf2d> pts = pts_ref;
		int st_a = pts.size();
		{
			//make super Triangle
			AABB2 box;
			for (const auto& p : pts_ref)
			{
				box.fitToEnclose(p);
			}
			box.min -= 1, box.max += 1;
			float x = box.min.x, y = box.min.y;
			float w = box.max.x - x, h = box.max.y - y;
			pts.emplace_back(x - h, y);
			pts.emplace_back(x + w / 2, y + h + w / 2);
			pts.emplace_back(x + w + h, y);
			tris.emplace_back(st_a, st_a + 1, st_a + 2);
		}

		//prependicular bisector
		auto getCircumcircle = [](const vf2d& a, const vf2d& b, const vf2d& c, vf2d& p, float& r_sq)
			{
				float D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
				if (std::abs(D) < 1e-6f) return false;

				float a_sq = a.x * a.x + a.y * a.y;
				float b_sq = b.x * b.x + b.y * b.y;
				float c_sq = c.x * c.x + c.y * c.y;

				p.x = (a_sq * (b.y - c.y) + b_sq * (c.y - a.y) + c_sq * (a.y - b.y)) / D;
				p.y = (a_sq * (c.x - b.x) + b_sq * (a.x - c.x) + c_sq * (b.x - a.x)) / D;
				r_sq = (p - a).mag();

				return true;
			};

		//add all the points one at a time to the triangulation
		for (int p = 0; p < st_a; p++) {
			const auto& pt = pts[p];

			//first find all the triangles that are no longer valid due to the insertion
			std::list<Triangle> bad_tris;
			for (const auto& t : tris) {
				//dont check tri pts
				if (t.contains(p)) continue;

				vf2d pos;
				float rad_sq;
				bool cc = getCircumcircle(pts[t.p[0]], pts[t.p[1]], pts[t.p[2]], pos, rad_sq);
				if (cc && (pos - pt).mag() < rad_sq) bad_tris.push_back(t);
			}

			//find the boundary of the polygonal hole
			std::list<Edge> polygon;
			for (const auto& bt : bad_tris) {
				//for each edge in tri
				for (int i = 0; i < 3; i++) {
					//if edge NOT shared by any other tri in bad_tris
					bool shared = false;
					for (const auto& o_bt : bad_tris) {
						if (&o_bt == &bt) continue;
						for (int j = 0; j < 3; j++) {
							if (Triangle::edgeShared(bt, i, o_bt, j)) {
								shared = true;
								break;
							}
						}
						if (shared) break;
					}
					//add edge to polygon
					if (!shared) {
						int a, b;
						bt.getEdge(i, a, b);
						polygon.emplace_back(a, b);
					}
				}

				//remove them from the data structure
				for (auto it = tris.begin(); it != tris.end();) {
					if (it->equals(bt)) {
						it = tris.erase(it);
					}
					else it++;
				}
			}

			//re-triangulate the polygonal hole
			for (const auto& e : polygon) {
				//form a triangle from edge to point
				tris.emplace_back(e.p[0], e.p[1], p);
			}
		}

		//done inserting points, now clean up
		for (auto it = tris.begin(); it != tris.end();) {
			//if tri has super tri vert, remove
			bool contains = false;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					if (it->p[i] == st_a + j) {
						contains = true;
						break;
					}
				}
				if (contains) break;
			}
			if (contains) {
				it = tris.erase(it);
			}
			else it++;
		}

		return tris;

	}


	//for connecting things up.
	template<typename TriContainer>
	std::set<Edge> extractEdges(const TriContainer& tris) {
		std::set<Edge> edges;
		for (const auto& t : tris) {
			for (int i = 0; i < 3; i++) {
				int a, b;
				t.getEdge(i, a, b);
				edges.emplace(a, b);
			}
		}
		return edges;
	}
}