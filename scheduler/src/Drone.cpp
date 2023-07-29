//
// Created by andykong on 2023/7/15.
//
#include "scheduler/Drone.h"

void Drone::init()
{
    position = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    qtn_orientation = Eigen::Quaterniond::Identity();
    angular_orientation = Eigen::Vector3d::Zero();
    angular_velocity = Eigen::Vector3d::Zero();
    position_accumulative_error = Eigen::Vector3d::Zero();
}

void Drone::setPosition(const Eigen::Vector3d & _position)
{
    position = _position;
}

void Drone::setVelocity(const Eigen::Vector3d & _velocity)
{
    velocity = _velocity;
}

void Drone::setQtnOrientation(const Eigen::Quaterniond & _qtn_orientation)
{
    qtn_orientation = _qtn_orientation;
    angular_orientation = ToEulerAngles(_qtn_orientation);
}

void Drone::setAngularOrientation(const Eigen::Vector3d & _angular_orientation)
{
    angular_orientation = _angular_orientation;
}

void Drone::setAngularVelocity(const Eigen::Vector3d & _angular_velocity)
{
    angular_velocity = _angular_velocity;
}

void Drone::setHeight(const float _height)
{
     height = _height;
}

void Drone_img::setDepth(const uint8_t _img_x,const uint8_t _img_y,const float _depth)
{
    img.x = _img_x;
    img.y = _img_y;
    depth = _depth;
    plane_depth = sqrtf(depth*depth + drone.getHeight() * drone.getHeight());
}
float Drone_img::getDis() const
{
    float plane_depth = sqrtf(depth*depth + drone.getHeight() * drone.getHeight());
    return plane_depth;
}
cv::Point Drone_img::getPoint() const
{
    return img;
}
Eigen::Vector3d Drone::getPosition() const
{
    return position;
}

Eigen::Vector3d Drone::getVelocity() const
{
    return velocity;
}

Eigen::Quaterniond Drone::getQtnOrientation() const
{
    return qtn_orientation;
}

Eigen::Vector3d Drone::getAngularOrientation() const
{
    return angular_orientation;
}

Eigen::Vector3d Drone::getAngularVelocity() const
{
    return angular_velocity;
}

float Drone::getHeight() const
{
    return height;
}

Eigen::Vector3d Drone::ToEulerAngles(const Eigen::Quaterniond& q) {
    Eigen::Vector3d angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

DronePose Drone::getPose() const {
    DronePose pose;
    pose.position = position;
    pose.angular_orientation = angular_orientation;
    return pose;
}

DroneTwist Drone::getTwist() const {
    DroneTwist twist;
    twist.velocity = velocity;
    twist.angular_velocity = angular_velocity;
    return twist;
}

//DroneImage Drone::getDepth() const {
//    DroneImage img;
//    img.img_x = img_x;
//    img.img_y = img_y;
//    img.depth = depth;
//    img.plane_depth = plane_depth;
//    return img;
//}

void Drone::setAccumulativeError(const Eigen::Vector3d &_convinced_position) {
    position_accumulative_error = _convinced_position - position;
}

