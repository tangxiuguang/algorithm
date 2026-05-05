#include "../include/dijkstra.h"

int main()
{
	int n = 6;//node number
    std::vector<std::vector<std::pair<int, int>>> adj(n);

    adj[0].emplace_back(1, 4);  // 0 ↔ 1，权重 4
    adj[0].emplace_back(2, 5);  // 0 ↔ 2，权重 5

    adj[1].emplace_back(0, 4);
    adj[1].emplace_back(2, 11);  // 1 ↔ 2，权重 11
    adj[1].emplace_back(3, 9);  // 1 ↔ 3，权重 9
    adj[1].emplace_back(4, 7);  // 1 ↔ 4，权重 7

    adj[2].emplace_back(0, 5);
    adj[2].emplace_back(1, 11);
    adj[2].emplace_back(4, 3);  // 2 ↔ 3，权重 3

    adj[3].emplace_back(1, 9);
    adj[3].emplace_back(4, 13);  // 3 ↔ 4，权重 13
    adj[3].emplace_back(5, 2);  // 3 ↔ 5，权重 2

    adj[4].emplace_back(1, 7);
    adj[4].emplace_back(2, 3);
    adj[4].emplace_back(3, 13);
    adj[4].emplace_back(5, 6);  // 4 ↔ 5，权重 6

    adj[5].emplace_back(3, 2);
    adj[5].emplace_back(4, 6);

    adj[5].emplace_back(7, 2);	// 异常测试

    std::vector<std::pair<int, int>> diskstra_path_ex;
    dijkstra(6 % adj.size(), adj.size(), adj, diskstra_path_ex);
    dijkstra(1 % adj.size(), adj.size(), adj, diskstra_path_ex);
    dijkstra(2 % adj.size(), adj.size(), adj, diskstra_path_ex);
    dijkstra(3 % adj.size(), adj.size(), adj, diskstra_path_ex);
    dijkstra(4 % adj.size(), adj.size(), adj, diskstra_path_ex);
    dijkstra(5 % adj.size(), adj.size(), adj, diskstra_path_ex);
    return 0;
}
