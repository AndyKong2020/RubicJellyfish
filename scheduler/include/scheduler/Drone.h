//
// Created by andykong on 2023/7/15.
//

#ifndef SRC_DRONE_H
#define SRC_DRONE_H

#include <iostream>
#include <Eigen/Core>
#include <boost/serialization/singleton.hpp>
#include <Eigen/Geometry>
#include <opencv4/opencv2/core/types.hpp>

#define drone Drone::get_mutable_instance()


typedef struct
{
    Eigen::Vector3d position;
    Eigen::Vector3d angular_orientation;
} DronePose;

typedef struct
{
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
} DroneTwist;

class Drone: public boost::serialization::singleton<Drone>
{
public:
    void init();
    void setPosition(const Eigen::Vector3d & _position);
    void setVelocity(const Eigen::Vector3d & _velocity);
    void setQtnOrientation(const Eigen::Quaterniond & _qtn_orientation);
    void setAngularOrientation(const Eigen::Vector3d & _angular_orientation);
    void setAngularVelocity(const Eigen::Vector3d & _angular_velocity);
    void setAccumulativeError(const Eigen::Vector3d & _convinced_position);
    void setHeight(const double & _height);
    static Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q);
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Quaterniond getQtnOrientation() const;
    Eigen::Vector3d getAngularOrientation() const;
    Eigen::Vector3d getAngularVelocity() const;
    Eigen::Vector3d getAccumulativeError() const;
    DronePose getPose() const;
    DroneTwist getTwist() const;
    double getHeight() const;
private:
    Eigen::Vector3d position_accumulative_error;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond qtn_orientation;
    Eigen::Vector3d angular_orientation;
    Eigen::Vector3d angular_velocity;
    double height;
};
#endif //SRC_DRONE_H
