#include "../../include/graph/dijkstra.h"

void dijkstra(int start, int n, const std::vector<std::vector<std::pair<int, int>>>& adj, std::vector<std::pair<int, int>>& path, bool single_source)
{
    if (0 > start || 0 >= n || 0 >= adj.size()) {
        return;
    }
    path.clear();
    path.resize(adj.size(), std::make_pair(INT_MAX/*距离*/, -1/*前趋*/)); // 重置path大小为n，统一初始化所有元素为(-1,-1)
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
    path[start].first = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        std::pair<int, int> qhead = pq.top();
        pq.pop();

        if (qhead.first > path[qhead.second].first) continue;

        for (std::vector<std::pair<int, int>>::const_iterator iter = adj[qhead.second].begin(); iter != adj[qhead.second].end(); iter++) {
            int v = iter->first;
            int w = iter->second;
            if (0 > v || v >= adj.size()) continue; // 防止输入数据异常导致越界
            if (path[v].first > path[qhead.second].first + w) {
                path[v].first = path[qhead.second].first + w;
                pq.emplace(path[v].first, v);
                path[v].second = qhead.second;
            }
        }
    }
    print_path(start, n, adj.size(), path);
    return;
}

/*
* start：开始节点下标
* step：计算的结点个数
* n：结点总数
* path：计算好的路径
* single_source：是否为单源路径
*/
void print_path(int start, int step, int n, const std::vector<std::pair<int, int>>& path, bool single_source)
{
    for (int i = 0; i < step; i++) {
        int index = (i + start) % n;
        std::cout << "Node " << (char)(start + 65) << " to node " << (char)(index + 65) << " value:"; // 65是把数字0转为字母A
        if (INT_MAX == path[index].first)
            std::cout << "unreachable";
        else
            std::cout << path[index].first << ", path:";

        std::vector<int> res;
        for (int cur = index; cur != -1; cur = path[cur].second)
            res.push_back(cur);
        if (single_source) std::reverse(res.begin(), res.end());

        for (int j = 0; j < res.size(); j++) {
            if (0 < j) std::cout << "-->";
            std::cout << (char)(res[j] + 65);
        }
        std::cout << std::endl << std::endl;
    }
    std::cout << std::endl;
}
