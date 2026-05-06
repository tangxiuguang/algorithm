#include "../include/astar.h"
#include "../../common/include/common.h"

void pint_path(const std::vector<std::vector<int>> &map, const std::vector<std::pair<int, int>> &path) {
	int rows = map.size();
	if (0 >= rows) {
		return;
	}
	int cols = map[0].size();
	std::vector<std::vector<char>> grid(rows, std::vector<char>(cols));
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			grid[i][j] = map[i][j] == 1 ? '#' : '.';
		}
	}
	
	for (std::vector<std::pair<int, int>>::const_iterator iter = path.begin(); iter != path.end(); iter++) {
		grid[iter->first][iter->second] = '*';
	}
	
	if (!path.empty()) {
		grid[path.front().first][path.front().second] = 'S';
		grid[path.back().first][path.back().second] = 'E';
	}
	
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			std::cout << grid[i][j] << ' ';
		}
		std::cout << std::endl;
	}
}

bool astar_find_path(const std::vector<std::vector<int>> &map, 
	int sx, int sy, int ex, int ey, 
	std::vector<std::pair<int, int>> &path, 
	bool allow_diag) {
	int rows = map.size();
	if (0 == rows) {
		return false;
	}
	int cols = map[0].size();
	
	path.clear();
	std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));
	std::priority_queue<node, std::vector<node>, cmp_node> openq;
	std::vector<std::vector<node>> node_grid(rows, std::vector<node>(cols));
	
	node start(sx, sy);
	start.h = allow_diag ? chebyshev(sx, sy, ex, ey) : manhattan(sx, sy, ex, ey);
	start.f = start.h;
	int dir_cnt = allow_diag ? 8 : 4;
	
	openq.push(start);
	node_grid[sx][sy] = start;
	
	while (!openq.empty()) {
		node cur = openq.top();
		openq.pop();
		
		int cx = cur.x;
		int cy = cur.y;
		
		if (cx == ex && cy == ey) {
			int x = ex;
			int y = ey;
			while (-1 != x && -1 != y) {
				path.emplace_back(x, y);
				int nx = node_grid[x][y].px;
				int ny = node_grid[x][y].py;
				x = nx;
				y = ny;
			}
			std::reverse(path.begin(), path.end());
			return true;
		}

		if (closed[cx][cy]) continue;
		closed[cx][cy] = true;
		
		for (int d = 0; d < dir_cnt; d++) {
			int nx = cx + dirs[d][0];
			int ny = cy + dirs[d][1];
			
			if (0 > nx || nx >= rows || 0 > ny || ny >= cols) {
				continue;
			}
			if (1 == map[nx][ny]) {
				continue;
			}
			if (closed[nx][ny]) {
				continue;
			}
			double step_cost = 1.0; // 直走代价1.0，斜走代价根号2
			if (allow_diag && 0 != dirs[d][0] && 0 != dirs[d][1]) {
				step_cost = sqrt(2.0);
			}
			double new_g = cur.g + step_cost;
			double new_h = allow_diag ? chebyshev(nx, ny, ex, ey) : manhattan(nx, ny, ex, ey);
			double new_f = new_g + new_h;
			if (new_f < node_grid[nx][ny].f || fabs(node_grid[nx][ny].f) < 1e-6) {
				node next(nx, ny);
				next.f = new_f;
				next.g = new_g;
				next.h = new_h;
				next.px = cx;
				next.py = cy;
				node_grid[nx][ny] = next;
				openq.push(next);
			}
		}
	}
	
	return false;
}