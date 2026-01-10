#pragma once
#include <vector>


//polar to cartesian helper
template<typename V2D>
V2D polar_generic(float rad, float angle) {
	return rad * V2D(std::cosf(angle), std::sinf(angle));
}

vf2d polar(float rad, float angle) {
	return polar_generic<vf2d>(rad, angle);
}

std::vector<vf2d> poissonDiscSample(const AABB2& box, float rad)
{
	//determine spacing
	float cell_size = rad / std::sqrt(2);
	int w = 1 + (box.max.x - box.min.x) / cell_size;
	int h = 1 + (box.max.y - box.min.y) / cell_size;
	vf2d** grid = new vf2d * [w * h];
	for (int i = 0; i < w * h; i++) grid[i] = nullptr;

	//where can i spawn from?
	std::vector<vf2d> spawn_pts{ box.getCenter() };

	//as long as there are spawnable pts,
	std::vector<vf2d> pts;
	while (spawn_pts.size()) {
		//choose random spawn pt
		auto it = spawn_pts.begin();
		std::advance(it, rand() % spawn_pts.size());
		const auto& spawn = *it;

		//try n times to add pt
		int k = 0;
		const int samples = 20;
		for (; k < samples; k++) {
			float angle = randFloat(2 * Pi);
			float dist = randFloat(rad, 2 * rad);
			vf2d cand = spawn + polar(dist, angle);
			if (!box.contains(cand)) continue;

			//check 3x3 region around candidate
			bool valid = true;
			int ci = (cand.x - box.min.x) / cell_size;
			int cj = (cand.y - box.min.y) / cell_size;
			int si = std::max(0, ci - 2);
			int sj = std::max(0, cj - 2);
			int ei = std::min(w - 1, ci + 2);
			int ej = std::min(h - 1, cj + 2);
			for (int i = si; i <= ei; i++) {
				for (int j = sj; j <= ej; j++) {
					//if there is a point, and its too close,
					const auto& idx = grid[i + w * j];
					if (idx && (*idx - cand).mag() < rad * rad) {
						//invalidate it
						valid = false;
						break;
					}
				}
				if (!valid) break;
			}

			//if no points too close, add the sucker
			if (valid) {
				if (ci < 0 || cj < 0 || ci >= w || cj >= h) continue;
				pts.push_back(cand);
				grid[ci + w * cj] = &pts.back();
				spawn_pts.push_back(cand);
				break;
			}
		}

		//not spawnable enough, remove
		if (k == samples) spawn_pts.erase(it);
	}

	delete[] grid;

	return pts;
}

