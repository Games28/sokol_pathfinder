#pragma once
#ifndef AABB2_H
#define AABB2_H

struct AABB2 {
	vf2d min = { INFINITY, INFINITY }, max = -min;

	vf2d getCenter() const {
		return (min + max) / 2;
	}

	void fitToEnclose(const vf2d& v) {
		if (v.x < min.x) min.x = v.x;
		if (v.y < min.y) min.y = v.y;
		if (v.x > max.x) max.x = v.x;
		if (v.y > max.y) max.y = v.y;
	}

	bool contains(const vf2d& v) const {
		if (v.x<min.x || v.x>max.x) return false;
		if (v.y<min.y || v.y>max.y) return false;
		return true;
	}

	bool overlaps(const AABB2& o) const {
		if (min.x > o.max.x || max.x < o.min.x) return false;
		if (min.y > o.max.y || max.y < o.min.y) return false;
		return true;
	}
};



#endif // !AABB2_h

