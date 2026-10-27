#ifndef CIRCULAR_TRAJECTORY_H
#define CIRCULAR_TRAJECTORY_H

class Trajectory {
public:
    virtual geometry_msgs::PoseStamped calculate_pose(double time) = 0;
};

class CircularTrajectory : public Trajectory {
    double radius, center_x, center_y, altitude, angular_velocity;
public:
    CircularTrajectory(double r, double x, double y, double z, double w)
        : radius(r), center_x(x), center_y(y), altitude(z), angular_velocity(w) {}

    geometry_msgs::PoseStamped calculate_pose(double time) override {
        geometry_msgs::PoseStamped pose;
        double theta = angular_velocity * time;
        pose.pose.position.x = center_x + radius * cos(theta);
        pose.pose.position.y = center_y + radius * sin(theta);
        pose.pose.position.z = altitude;
        return pose;
    }
};
#endif // CIRCULAR_TRAJECTORY_H