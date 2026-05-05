#include "../../common/include/common.h"
#include "../include/astar.h"

int main()
{
    // 0可行 1障碍
    std::vector<std::vector<int>> maze = {
        {0,0,0,0,0,0},
        {0,1,1,1,0,0},
        {0,0,0,1,0,0},
        {1,1,0,0,0,0},
        {0,0,1,1,1,0},
        {0,0,0,0,0,0}
    };

    int sx = 0, sy = 0;
    int ex = 5, ey = 5;

    std::vector<std::pair<int, int>> path;

    std::cout << "===== 4方向寻路 =====" << std::endl;
    astar_find_path(maze, sx, sy, ex, ey, path, false);
    pint_path(maze, path);

    std::cout << "\n===== 8方向寻路 =====" << std::endl;
    astar_find_path(maze, sx, sy, ex, ey, path, true);
    pint_path(maze, path);
}