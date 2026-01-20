#pragma once
#include "Node.h"

#include <list>
#include <unordered_map>

class Graph
{
	void copyFrom(const Graph&), clear();

public:
	std::list<Node*> nodes;

	Graph() {}

	Graph(const Graph& g)
	{
		copyFrom(g);
	}

	~Graph()
	{
		clear();
	}

	Graph& operator=(const Graph& g)
	{
		if (&g == this) return *this;
		clear();
		copyFrom(g);

		return *this;
	}

	bool contains(const Node* o) const
	{
		if (!o) return false;
		
		for (const auto& n : nodes)
		{
			if (n == o) return true;
		}

		return false;
	}

	bool addLink(Node* a, Node* b)
	{
		//is link invalid?
		if (!a || !b || a == b) return false;

		//does this graph have those nodes;
		if (!contains(a) || !contains(b)) return false;

		//doe it already exist?
		for (const auto& n : a->links)
		{
			if (n == b) return false;
		}

			a->links.emplace_back(b);

			return true;
		
	}

	void removeNode(Node* said)
	{
		//find
		auto it = std::find(nodes.begin(), nodes.end(), said);
		if (it != nodes.end())
		{
			//remove other nodes link
			for (const auto& n : nodes)
			{
				auto nit = std::find(n->links.begin(), n->links.end(), said);
				if (nit != nodes.end()) n->links.erase(nit);
			}

			//deallocate
			delete* it;
			//remove
			nodes.erase(it);

		}
	}

	//doesnt change structure pre se, but changes node values...(const-ish)
	[[nodiscard]] std::vector<Node*> route(Node* from, Node* to) const
	{
		std::vector<Node*> path;
		if (!from || !to || from == to) return path;

		from->g_cost = 0;

		for (const auto& n : nodes)
		{
			//compute h costs
			n->h_cost = (n->pos - to->pos).mag();
			//reset parents
			n->parent = nullptr;
		}

		std::list<Node*> open{ from }, closed;

		while (open.size())
		{
			//node in OPEN with lowest f_cost
			float record = INFINITY;
			auto curr_it = open.end();
			for (auto oit = open.begin(); oit != open.end(); oit++)
			{
				const auto& o = *oit;
				if (o->f_cost < record)
				{
					record = o->f_cost;
					curr_it = oit;
				}
			}

			//remove from OPEN, add to CLOSED
			open.erase(curr_it);
			Node* curr = *curr_it;
			closed.emplace_back(curr);

			//path found
			if (curr == to) break;

			for (const auto& nbr : curr->links)
			{
				//skip if neighbor in CLOSED
				bool found = false;
				for (const auto& c : closed)
				{
					if (c == nbr)
					{
						found = true;
						break;
					}
				}

				if (found) continue;

				//new path shorter OR neighbor NOT in OPEN
				float new_g_cost = curr->g_cost + (nbr->pos - curr->pos).mag();
				bool shorter = new_g_cost < nbr->g_cost;
				bool in_open = false;
				for (const auto& o : open)
				{
					if (o == nbr)
					{
						in_open = true;
						break;
					}
				}

				if (shorter || in_open)
				{
					nbr->g_cost = new_g_cost;
					nbr->f_cost = nbr->g_cost + nbr->h_cost;
					nbr->parent = curr;
					if (!in_open) open.emplace_back(nbr);
				}
			}
		}

		//traverse backwards
		bool valid = false;
		for (Node* curr = to; curr; curr = curr->parent)
		{
			path.emplace_back(curr);
			if (curr == from)
			{
				valid = true;
				break;
			}
		}

		//ensure path contains to and from
		if (valid) std::reverse(path.begin(), path.end());
		else path.clear();

		return path;
	}

};

void Graph::copyFrom(const Graph& g)
{
	//copy voer node positions
	std::unordered_map<Node*, Node*> g2me;
	for (const auto& gn : g.nodes)
	{
		nodes.push_back(new Node(gn->pos));
		g2me[gn] = nodes.back();
	}

	//copy over links
	for (const auto& gn : g.nodes)
	{
		Node* n = g2me[gn];
		for (const auto& go : gn->links)
		{
			n->links.push_back(g2me[go]);
		}
	}
}

void Graph::clear()
{
	for (const auto& n : nodes)
	{
		delete n;
	}

	nodes.clear();
}


