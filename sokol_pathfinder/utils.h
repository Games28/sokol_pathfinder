#pragma once
#ifndef UTILS_HEADER_H
#define UTILS_HEADER_H

//for rand
#include <cstdlib>

constexpr float Pi=3.1415927f;

static float randFloat(float b=1, float a=0) {
	static const float rand_max=RAND_MAX;
	float t=std::rand()/rand_max;
	return a+t*(b-a);
}
#endif