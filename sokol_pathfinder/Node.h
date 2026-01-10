#pragma once
#include <list>

struct Node
{
	std::list<Node*> links;
	vf3d pos;
	int id = -1;
	float g_cost = 0, h_cost = 0, f_cost = 0;
	Node* parent = nullptr;

	Node() = delete;

	Node(vf3d p) : pos(p) {}

	//dont copy links?
	//negative id to differentiate original, and copy
	Node(const Node& n) : pos(n.pos), id(-n.id) {}
};
