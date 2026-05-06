#include "../include/dwa.h"
#include <cmath> // 关键：引入 cmath 头文件，用于 M_PI 定义

int main() {
    // 初始化 DWA 参数
    DWA dwa(0.0, 0.5, -1.0, 1.0, 0.05, 0.1, 1.0, 0.6, 0.2, 0.2); // 

    // 机器人初始状态（起点 (0,0)，朝向 0 弧度）
    RobotState current_state = { 0.0, 0.0, 0.0, 0.0, 0.0 }; // x,y,朝向角，线速度，角速度

    // 目标点 (5.0, 5.0)
    std::pair<double, double> goal = { 5.0, 5.0 };

    // 障碍物列表
    std::vector<Obstacle> obstacles = { {1.5, 1.5}, {3.0, 3.5}, {4.0, 1.0} };

    // 模拟机器人运动（1000 步，每步 0.1s）
    std::cout << "DWA Test: Robot moving to (" << goal.first << "," << goal.second << ")\n"; // 修复：补充 goal.second
    for (int i = 0; i < 1000; i++) {
        auto optimal_speed = dwa.calculateOptimalSpeed(current_state, goal, obstacles);
        double v = optimal_speed.first, w = optimal_speed.second;

        // 更新机器人状态
        double time_step = 0.1;
        current_state.x += v * cos(current_state.theta) * time_step;
        current_state.y += v * sin(current_state.theta) * time_step;
        current_state.theta += w * time_step;

        // 角度归一化（修正语法错误：current 改为 current_state.theta）
        if (current_state.theta > std::acos(-1)/*M_PI*/) current_state.theta -= 2 * std::acos(-1);
        if (current_state.theta < -std::acos(-1)) current_state.theta += 2 * std::acos(-1);

        // 输出状态（修复语法错误：补充 i、current_state.y，修正输出格式）
        std::cout << "Step" << i << ": x=" << current_state.x << ", y=" << current_state.y << ", theta=" << current_state.theta << ", v=" << v << ", w=" << w << "\n";
        
        // 判断是否到达目标（修复语法错误：补充括号、修正逻辑）
        if (sqrt(pow(current_state.x - goal.first, 2) + pow(current_state.y - goal.second, 2)) < 0.3) {
            std::cout << "Goal reached!\n";
            break;
        }
    }
    return 0;
}
