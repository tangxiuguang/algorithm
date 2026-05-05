#include <iostream>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

void dijkstra(int start, int n, const std::vector<std::vector<std::pair<int, int>>>& adj, std::vector<std::pair<int, int>>& path, bool single_source = true);
void print_path(int start, int step, int n, const std::vector<std::pair<int, int>>& path, bool single_source = true);
