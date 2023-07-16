//
// Created by andykong on 2023/7/15.
//

#ifndef SRC_DRONE_H
#define SRC_DRONE_H

#include <Eigen/Core>
#include <boost/serialization/singleton.hpp>
#include <Eigen/Geometry>


class Drone: public boost::serialization::singleton<Drone>
{
public:
    void init();
    void setPosition(const Eigen::Vector3d & _position);
    void setVelocity(const Eigen::Vector3d & _velocity);
    void setQtnOrientation(const Eigen::Quaterniond & _qtn_orientation);
    void setAngularOrientation(const Eigen::Vector3d & _angular_orientation);
    void setAngularVelocity(const Eigen::Vector3d & _angular_velocity);
    static Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q);
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Quaterniond getQtnOrientation() const;
    Eigen::Vector3d getAngularOrientation() const;
    Eigen::Vector3d getAngularVelocity() const;
private:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond qtn_orientation;
    Eigen::Vector3d angular_orientation;
    Eigen::Vector3d angular_velocity;
};
#endif //SRC_DRONE_H
