// DWA.cpp（实现文件，对应上方头文件声明，直接复制可用，无多余格式）
#include "DWA.h"

// 构造函数：初始化参数，设置默认安全距离
DWA::DWA(double v_min, double v_max, double w_min, double w_max,
    double v_step, double w_step, double predict_time,
    double alpha, double beta, double gamma)
    : v_min(v_min), v_max(v_max), w_min(w_min), w_max(w_max),
    v_step(v_step), w_step(w_step), predict_time(predict_time),
    alpha(alpha), beta(beta), gamma(gamma), safe_distance(0.2) {} // 安全距离默认 0.2m

// 1. 速度窗口采样：生成所有候选速度（v, ω）组合
std::vector<std::pair<double, double>> DWA::sampleSpeedWindow() {
    std::vector<std::pair<double, double>> speed_window;
    // 遍历所有线速度、角速度组合，生成候选窗口
    for (double v = v_min; v <= v_max; v += v_step) {
        for (double w = w_min; w <= w_max; w += w_step) {
            speed_window.emplace_back(v, w);
        }
    }
    return speed_window;
}

// 2. 轨迹预测：根据当前状态和候选速度，预测未来轨迹
std::vector<RobotState> DWA::predictTrajectory(const RobotState& current_state, double v, double w) {
    std::vector<RobotState> trajectory;
    RobotState current = current_state; // 从当前状态开始预测
    double t = 0.0;
    // 按固定步长预测，直到达到预测时间
    while (t < predict_time) {
        // 更新位置：x = x + v*cos (theta)dt，y = y + vsin(theta)dt
        current.x += v * cos(current.theta) * 0.01; //dt=0.01s，避免步长过大
        current.y += v * sin(current.theta) * 0.01;

        // 更新朝向角：theta = theta + wdt
        current.theta += w * 0.01;

        // 更新当前速度（候选速度）
        current.v = v;
        current.w = w;

        // 加入轨迹集合
        trajectory.push_back(current);
        // 累加时间
        t += 0.01;
    }
    return trajectory;
}

// 3. 碰撞检测：判断轨迹是否与障碍物碰撞
bool DWA::isCollision(const std::vector<RobotState>& trajectory, const std::vector<Obstacle>& obstacles) {
    // 遍历轨迹上的每个点，判断是否与任意障碍物距离小于安全距离
    for (const auto& state : trajectory) {
        for (const auto& obs : obstacles) {
            // 计算轨迹点与障碍物的距离
            double dist = sqrt(pow(state.x - obs.x, 2) + pow(state.y - obs.y, 2));
            if (dist < safe_distance) { // 小于安全距离，判定为碰撞
                return true;
            }
        }
    }
    return false; // 无碰撞
}

// 4. 评价函数：计算候选速度的评分（评分越高越优）
double DWA::evaluateSpeed(const std::vector<RobotState>& trajectory,
    const std::pair<double, double>& goal,
    const std::vector<Obstacle>& obstacles) {
    // 1. 目标代价：轨迹终点到目标的距离（越小评分越高）
    const auto& end_state = trajectory.back();
    double goal_dist = sqrt(pow(end_state.x - goal.first, 2) + pow(end_state.y - goal.second, 2));
    double goal_score = 1.0 / (goal_dist + 1e-6); // 避免除零

    // 2. 安全代价：无碰撞则得分，有碰撞得 0 分
    double safe_score = isCollision(trajectory, obstacles) ? 0.0 : 1.0;

    // 3. 速度代价：线速度越接近最大速度，评分越高
    double speed_score = end_state.v / v_max;

    // 加权计算总评分（alpha、beta、gamma 为权重，从构造函数传入）
    return alpha * goal_score + beta * safe_score + gamma * speed_score;
}

// 5. 核心：计算最优速度（筛选评分最高、无碰撞的速度）
std::pair<double, double> DWA::calculateOptimalSpeed(const RobotState& current_state,
    const std::pair<double, double>& goal,
    const std::vector<Obstacle>& obstacles) {
    std::vector<std::pair<double, double>> speed_window = sampleSpeedWindow(); // 获取所有候选速度
    double max_score = -1.0;
    std::pair<double, double> optimal_speed = { 0.0, 0.0 }; // 默认初始速度

    // 遍历所有候选速度，筛选最优解
    for (std::vector<std::pair<double, double>>::iterator iter = speed_window.begin(); iter != speed_window.end(); iter++) {
        std::vector<RobotState> trajectory(std::move(predictTrajectory(current_state, iter->first, iter->second)));
        // 跳过有碰撞的候选速度
        if (isCollision(trajectory, obstacles)) {
            continue;
        }
        // 计算当前速度的评分
        double score = evaluateSpeed(trajectory, goal, obstacles);
        // 更新最优速度（评分更高则替换）
        if (score > max_score) {
            max_score = score;
            optimal_speed = { iter->first, iter->second };
        }
    }
    return optimal_speed;
}
