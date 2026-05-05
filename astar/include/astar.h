#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <iostream>
#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

// 方向数组：分别表示上、下、左、右、左上、右上、左下、右下
const int dirs[8][4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {-1, 1}, {1, 1}};

typedef struct node {
	int x, y; // x/y表示坐标（矩阵下标）
	double f, g, h; // f：从起点经过当前结点到达目标结点的总估计代价，g：从起点到当前结点的实际代价，h：从当前结点到目标结点的估计代价
	int px, py; // 当前结点的父结点的坐标（矩阵的下标）
	
	node() : x(0), y(0), f(0), g(0), h(0), px(-1), py(-1) {}
	node(int _x, int _y) : x(_x), y(_y), f(0), g(0), h(0), px(-1), py(-1) {}
};

typedef struct cmp_node {
	bool operator()(const node& a, const node& b)
	{
		return a.f > b.f;
	}
};

void pint_path(const std::vector<std::vector<int>> &map, const std::vector<std::pair<int, int>> &path);

bool astar_find_path(const std::vector<std::vector<int>> &map, 
	int sx, int sy, int ex, int ey, 
	std::vector<std::pair<int, int>> &path, 
	bool allow_diag);

#endif // _ASTAR_H_
