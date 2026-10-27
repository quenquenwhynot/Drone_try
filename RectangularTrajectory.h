#ifndef RECTANGULAR_TRAJECTORY_H
#define RECTANGULAR_TRAJECTORY_H

#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <vector>

// 抽象基类（与 CircularTrajectory.h 保持一致）
class Trajectory {
public:
    virtual geometry_msgs::PoseStamped calculate_pose(double time) = 0;
    virtual bool is_finished(double time) = 0;
};

// 矩形轨迹类：沿矩形飞行一圈后降落
class RectangularTrajectory : public Trajectory {
    double width;           // 矩形宽（X方向）
    double height_rect;     // 矩形长（Y方向）
    double start_x;         // 起始点X
    double start_y;         // 起始点Y
    double altitude;        // 飞行高度
    double speed;           // 飞行速度（m/s）
    double perimeter;       // 矩形周长
    double total_time;      // 飞完一圈所需时间

    // 矩形四个顶点（按顺序）
    std::vector<std::pair<double,double>> waypoints;

public:
    // 参数：矩形宽、矩形长、起始点X、起始点Y、飞行高度、速度
    RectangularTrajectory(double w, double h, double sx, double sy, double z, double v)
        : width(w), height_rect(h), start_x(sx), start_y(sy), altitude(z), speed(v)
    {
        // 按顺时针定义四个顶点（起点 → 右 → 右上 → 左上 → 回起点）
        waypoints = {
            {start_x,         start_y},
            {start_x + width, start_y},
            {start_x + width, start_y + height_rect},
            {start_x,         start_y + height_rect},
            {start_x,         start_y}   // 回到起点，闭合一圈
        };

        perimeter = 2.0 * (width + height_rect);
        total_time = perimeter / speed;
    }

    // 根据时间计算当前在矩形轨迹上的位置
    geometry_msgs::PoseStamped calculate_pose(double time) override {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        // 超出一圈时间后，停在起始点（准备降落）
        if (time >= total_time) {
            pose.pose.position.x = start_x;
            pose.pose.position.y = start_y;
            pose.pose.position.z = altitude;
            return pose;
        }

        // 计算当前已走的路程
        double dist_traveled = speed * time;

        // 四段边长
        double seg_len[4] = { width, height_rect, width, height_rect };

        double accumulated = 0.0;
        for (int i = 0; i < 4; ++i) {
            if (dist_traveled <= accumulated + seg_len[i]) {
                // 在第 i 段上
                double t = (dist_traveled - accumulated) / seg_len[i];
                double x0 = waypoints[i].first;
                double y0 = waypoints[i].second;
                double x1 = waypoints[i + 1].first;
                double y1 = waypoints[i + 1].second;
                pose.pose.position.x = x0 + t * (x1 - x0);
                pose.pose.position.y = y0 + t * (y1 - y0);
                pose.pose.position.z = altitude;
                return pose;
            }
            accumulated += seg_len[i];
        }

        // 兜底：返回起始点
        pose.pose.position.x = start_x;
        pose.pose.position.y = start_y;
        pose.pose.position.z = altitude;
        return pose;
    }

    // 是否已飞完一圈
    bool is_finished(double time) override {
        return time >= total_time;
    }

    double get_total_time() const { return total_time; }
};

#endif // RECTANGULAR_TRAJECTORY_H
