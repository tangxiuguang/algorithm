#ifndef _DWA_H_
#define _DWA_H_

#include <cmath>
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>

// 机器人状态（x,y坐标、朝向θ、线速度v、角速度ω）
struct RobotState {
    double x;     // x坐标
    double y;     // y坐标
    double theta; // 朝向角（弧度）
    double v;     // 线速度（m/s）
    double w;     // 角速度（rad/s）
};

// 障碍物（x,y坐标）
struct Obstacle {
    double x;
    double y;
};

// DWA算法类（封装核心逻辑）
class DWA {
public:
    // 构造函数（可调整参数适配机器人）
    DWA(double v_min = 0.0, double v_max = 0.5, double w_min = -1.0, double w_max = 1.0,
        double v_step = 0.05, double w_step = 0.1, double predict_time = 1.0,
        double alpha = 0.6, double beta = 0.2, double gamma = 0.2);

    // 核心：计算最优速度（v, ω）
    std::pair<double, double> calculateOptimalSpeed(const RobotState& current_state,
                                                   const std::pair<double, double>& goal,
                                                   const std::vector<Obstacle>& obstacles);

private:
    // 速度窗口采样（生成候选v、ω）
    std::vector<std::pair<double, double>> sampleSpeedWindow();

    // 轨迹预测（根据候选速度预测未来轨迹）
    std::vector<RobotState> predictTrajectory(const RobotState& current_state, double v, double w);

    // 评价函数（评分越高，速度越优）
    double evaluateSpeed(const std::vector<RobotState>& trajectory,
                        const std::pair<double, double>& goal,
                        const std::vector<Obstacle>& obstacles);

    // 碰撞检测（判断轨迹是否碰撞）
    bool isCollision(const std::vector<RobotState>& trajectory, const std::vector<Obstacle>& obstacles);

    // 核心参数
    double v_min, v_max;       // 线速度范围
    double w_min, w_max;       // 角速度范围
    double v_step, w_step;     // 速度采样步长
    double predict_time;       // 轨迹预测时间
    double alpha, beta, gamma; // 评价函数权重
    double safe_distance;      // 避障安全距离（0.2m）
};

#endif // _DWA_H_
