#ifndef DRONEMATHS_H
#define DRONEMATHS_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <BasicLinearAlgebra.h>

// DroneMaths.h
#pragma once

namespace DroneMaths {

class DroneKinematics {

public:
    // Convert Euler angles to rotation matrix form using Z-Y-X convention
    void euler_to_rotation(double roll, double pitch, double yaw, double R[3][3]);
    BLA::Matrix<3,3> euler_to_rotation(double roll, double pitch, double yaw);

    // Convert rotation matrix to Euler angles using Z-Y-X convention
    void rotation_to_euler(double R[3][3], double &roll, double &pitch, double &yaw);
    BLA::Matrix<3,1> rotation_to_euler(const BLA::Matrix<3,3> &R);

    // Convert Euler angles to quaternion
    void euler_to_quat(double roll, double pitch, double yaw, double q[4]);
    BLA::Matrix<4> euler_to_quat(double roll, double pitch, double yaw);

    // Convert quaternion to Euler angles
    void quat_to_euler(double q[4], double &roll, double &pitch, double &yaw);
    BLA::Matrix<3,1> quat_to_euler(const BLA::Matrix<4> &q);

    // Normalize a quaternion
    void quat_normalize(double q[4]);
    void quat_normalize(BLA::Matrix<4> &q);

};

}

#endif // DRONEMATHS_H