#include "DroneMaths.h"

namespace DroneMaths {

    void DroneKinematics::euler_to_rotation(double roll, double pitch, double yaw, double R[3][3]) {
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);

        /* 
            Rotation matrix using Z-Y-X convention where,
            φ = roll, θ = pitch, ψ = yaw

            Cψ*Cθ   Cψ*Sθ*Sφ − Sψ*Cφ    Cψ*Sθ*Cφ + Sψ*Sφ
            Sψ*Cθ   Sψ*Sθ*Sφ + Cψ*Cφ    Sψ*Sθ*Cφ − Cψ*Sφ
            −Sθ         Cθ*Sφ               Cθ Cφ
        */

        R[0][0] = cy*cp;
        R[0][1] = cy*sp*sr - sy*cr;
        R[0][2] = cy*sp*cr + sy*sr;

        R[1][0] = sy*cp;
        R[1][1] = sy*sp*sr + cy*cr;
        R[1][2] = sy*sp*cr - cy*sr;
        
        R[2][0] = -sp;
        R[2][1] = cp*sr;
        R[2][2] = cp*cr;
    }

    BLA::Matrix<3,3> DroneKinematics::euler_to_rotation(double roll, double pitch, double yaw) {
        BLA::Matrix<3,3> R;
        double rot[3][3];
        euler_to_rotation(roll, pitch, yaw, rot);

        R(0,0) = rot[0][0]; R(0,1) = rot[0][1]; R(0,2) = rot[0][2];
        R(1,0) = rot[1][0]; R(1,1) = rot[1][1]; R(1,2) = rot[1][2];
        R(2,0) = rot[2][0]; R(2,1) = rot[2][1]; R(2,2) = rot[2][2];
        return R;
    }

    void DroneKinematics::rotation_to_euler(double R[3][3], double &roll, double &pitch, double &yaw) {
        pitch = -asin(R[2][0]);
        roll = atan2(R[2][1], R[2][2]);
        yaw = atan2(R[1][0], R[0][0]);
    }

    void DroneKinematics::euler_to_quat(double roll, double pitch, double yaw, double q[4]) {
        // Abbreviations for the various angular functions
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        q[0] = cr * cp * cy + sr * sp * sy;
        q[1] = sr * cp * cy - cr * sp * sy;
        q[2] = cr * sp * cy + sr * cp * sy;
        q[3] = cr * cp * sy - sr * sp * cy;
    }

    BLA::Matrix<4> DroneKinematics::euler_to_quat(double roll, double pitch, double yaw) {
        BLA::Matrix<4> q;
        double quat[4];
        euler_to_quat(roll, pitch, yaw, quat);

        q(0) = quat[0];
        q(1) = quat[1];
        q(2) = quat[2];
        q(3) = quat[3];
        return q;
    }

    void DroneKinematics::quat_normalize(double q[4]) {
        double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }

    void DroneKinematics::quat_normalize(BLA::Matrix<4> &q) {
        double norm = sqrt(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
        q(0) /= norm;
        q(1) /= norm;
        q(2) /= norm;
        q(3) /= norm;
    }

}