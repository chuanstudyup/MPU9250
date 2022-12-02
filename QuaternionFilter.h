#ifndef QUATERNIONFILTER_H
#define QUATERNIONFILTER_H

#include <cmath>
#include "ArduTime.h"
#include "./Eigen/Dense"

using namespace Eigen;

enum class QuatFilterSel {
    NONE,
    MADGWICK,
    MAHONY,
    EKF
};

class QuaternionFilter {
    // for madgwick
    float GyroMeasError = M_PI * (10.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = M_PI * (0.f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // for mahony
    float Kp = 5.0;
    float Ki = 0.0;
    
    // for ekf
    Matrix4d P = Matrix4d::Identity() * 0.000001;
    Matrix4d Q = Matrix4d::Identity() * 0.0001;
    Matrix3d Ra = Matrix3d::Identity() * 0.1;
    Matrix3d Rm = Matrix3d::Identity() * 0.5;
	
    QuatFilterSel filter_sel{QuatFilterSel::MADGWICK};
    double deltaT{0.};
    uint32_t newTime{0}, oldTime{0};

public:
    void select_filter(QuatFilterSel sel) {
        filter_sel = sel;
    }

    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        newTime = micros();
        deltaT = newTime - oldTime;
        deltaT = fabs(deltaT * 0.001 * 0.001);

        switch (filter_sel) {
            case QuatFilterSel::MADGWICK:
                if(madgwick(ax, ay, az, gx, gy, gz, mx, my, mz, q))
                        oldTime = newTime;  
                break;
            case QuatFilterSel::MAHONY:
                if(mahony(ax, ay, az, gx, gy, gz, mx, my, mz, q))
                        oldTime = newTime;
                break;
            case QuatFilterSel::EKF:
                if(ekf(ax, ay, az, gx, gy, gz, mx, my, mz, q))
                        oldTime = newTime;
                break;
            default:
                no_filter(ax, ay, az, gx, gy, gz, mx, my, mz, q);
                oldTime = newTime;
                break;
        }
    }

    void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
        q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * deltaT;
        q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * deltaT;
        q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * deltaT;
        q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * deltaT;
        float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }

    // Madgwick Quaternion Update
    bool madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // short name local variable for readability
        double recipNorm;
        double s0, s1, s2, s3;
        double qDot1, qDot2, qDot3, qDot4;
        double hx, hy;
        //double w_err_x, w_err_y, w_err_z;
        //static double w_bx = 0.0, w_by = 0.0, w_bz = 0.0; 
        double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Normalise accelerometer measurement
        double a_norm = ax * ax + ay * ay + az * az;
        if (a_norm == 0.) return false;  // handle NaN
        recipNorm = 1.0 / sqrt(a_norm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        double m_norm = mx * mx + my * my + mz * mz;
        if (m_norm == 0.) return false;  // handle NaN
        recipNorm = 1.0 / sqrt(m_norm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
/*        
        //Compute angular estimated direction of the gyroscope error
        w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
        w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
        w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;
        
        //Compute and remove the gyroscope baises
        w_bx += w_err_x * deltaT * zeta;
        w_by += w_err_y * deltaT * zeta;
        w_bz += w_err_z * deltaT * zeta;
        gx -= w_bx;
        gy -= w_by;
        gz -= w_bz;
*/         
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * deltaT;
        q1 += qDot2 * deltaT;
        q2 += qDot3 * deltaT;
        q3 += qDot4 * deltaT;

        // Normalise quaternion
        recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
        return true;
    }

    // Mahony accelleration filter
    // Mahony scheme uses proportional and integral filtering on
    // the error between estimated reference vector (gravity) and measured one.
    // Madgwick's implementation of Mayhony's AHRS algorithm.
    // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    // Free parameters in the Mahony filter and fusion scheme,
    // Kp for proportional feedback, Ki for integral
    // float Kp = 30.0;
    // float Ki = 0.0;
    // with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
    // with MPU-6050, some instability observed at Kp=100 Now set to 30.
    bool mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        float recipNorm;
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;  //error terms
        static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
        float tmp;
        
        // auxiliary variables to reduce number of repeated operations
        float q0q0 = q[0]*q[0];
        float q0q1 = q[0]*q[1];
        float q0q2 = q[0]*q[2];
        float q0q3 = q[0]*q[3];
        float q1q1 = q[1]*q[1];
        float q1q2 = q[1]*q[2];
        float q1q3 = q[1]*q[3];
        float q2q2 = q[2]*q[2];
        float q2q3 = q[2]*q[3];
        float q3q3 = q[3]*q[3];
        
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        tmp = ax * ax + ay * ay + az * az;
        if (tmp == 0.f) return false;
        
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        tmp = mx * mx + my * my + mz * mz;
        if(tmp == 0.f) return false;
        recipNorm = 1.0 / sqrt(tmp);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // compute reference direction of magnetic field
        //这里计算得到的是地磁计在理论地磁坐标系下的机体上三个轴的分量
        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);   

        //bx计算的是当前航向角和磁北的夹角，也就是北天东坐标下的航向角
        //当罗盘水平旋转的时候，航向角在0-360之间变化
        bx = sqrt((hx*hx) + (hy*hy));
        bz = hz;
        
        //地磁计在n系下磁向量转换到b系下，反向使用DCM得到
        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
        
        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy) + (my*wz - mz*wy);
        ey = (az * vx - ax * vz) + (mz*wx - mx*wz);
        ez = (ax * vy - ay * vx) + (mx*my - my*wx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
        ix += Ki * ex * deltaT;  // integral error scaled by Ki
        iy += Ki * ey * deltaT;
        iz += Ki * ez * deltaT;
        gx += ix;  // apply integral feedback
        gy += iy;
        gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;

        // Integrate rate of change of quaternion, q cross gyro term
        deltaT = 0.5 * deltaT;
        gx *= deltaT;  // pre-multiply common factors
        gy *= deltaT;
        gz *= deltaT;
        q[0] += (-q[1] * gx - q[2] * gy - q[3] * gz);
        q[1] += (q[0] * gx + q[2] * gz - q[3] * gy);
        q[2] += (q[0] * gy - q[1] * gz + q[3] * gx);
        q[3] += (q[0] * gz + q[1] * gy - q[2] * gx);

        // renormalise quaternion
        recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] = q[0] * recipNorm;
        q[1] = q[1] * recipNorm;
        q[2] = q[2] * recipNorm;
        q[3] = q[3] * recipNorm;
        
        return true;
    }
    
    bool ekf(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        float tmp;
        
        //auxiliary variables to reduce number of repeated operations
        //float q0q0 = q[0]*q[0];
        float q0q1 = q[0]*q[1];
        float q0q2 = q[0]*q[2];
        float q0q3 = q[0]*q[3];
        float q1q1 = q[1]*q[1];
        float q1q2 = q[1]*q[2];
        float q1q3 = q[1]*q[3];
        float q2q2 = q[2]*q[2];
        float q2q3 = q[2]*q[3];
        float q3q3 = q[3]*q[3];
        
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        Vector3d Za(ax,ay,az); 
        tmp = Za.norm();
        if (tmp == 0.f) return false;
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        Za = Za/tmp;
       
        Vector3d Zm(mx,my,mz); 
        tmp = Zm.norm();
        if(tmp == 0.f) return false;
        Zm = Zm/tmp;
        
        //compute reference direction of magnetic field
        //这里计算得到的是地磁计在理论地磁坐标系下的机体上三个轴的分量
        float hx = 2*Zm(0)*(0.5 - q2q2 - q3q3) + 2*Zm(1)*(q1q2 - q0q3) + 2*Zm(2)*(q1q3 + q0q2);
        float hy = 2*Zm(0)*(q1q2 + q0q3) + 2*Zm(1)*(0.5 - q1q1 - q3q3) + 2*Zm(2)*(q2q3 - q0q1);
        float hz = 2*Zm(0)*(q1q3 - q0q2) + 2*Zm(1)*(q2q3 + q0q1) + 2*Zm(2)*(0.5 - q1q1 - q2q2);   

        //bx计算的是当前航向角和磁北的夹角，也就是北天东坐标下的航向角
        //当罗盘水平旋转的时候，航向角在0-360之间变化
        float bx = sqrt((hx*hx) + (hy*hy));
        float bz = hz; 

        Vector4d x(q[0],q[1],q[2],q[3]);
        Matrix4d Ak;
        Ak << 0,  -gx, -gy, -gz,
              gx,  0,   gz, -gy,
              gy, -gz,  0,   gx,
              gz,  gy, -gx,  0;
        Ak *= 0.5f * deltaT;
        Ak += Matrix4d::Identity();
        x = Ak * x;
        x = x/x.norm();
		
        Matrix4d Ppre = Ak * P * Ak.transpose() + Q;
        
        Vector3d R1(x(0)*x(0)+x(1)*x(1)-x(2)*x(2)-x(3)*x(3),
                   2*(x(1)*x(2)-x(0)*x(3)),
                   2*(x(1)*x(3)+x(0)*x(2)));
        
        Vector3d R3(2*(x(1)*x(3)-x(0)*x(2)),
                    2*(x(0)*x(1)+x(2)*x(3)),
                    x(0)*x(0)-x(1)*x(1)-x(2)*x(2)+x(3)*x(3));
                    
        MatrixXd J1(3,4),J3(3,4);
        J1 << x(0), x(1), -x(2), -x(3),
             -x(3), x(2),  x(1), -x(0),
              x(2), x(3),  x(0),  x(1);
        J1 = 2*J1;
        J3 <<  -x(2),  x(3), -x(0),  x(1),
                x(1),  x(0),  x(3),  x(2),
                x(0), -x(1), -x(2),  x(3);
        J3 = 2*J3;
        
        MatrixXd Kk(4,3);
        Kk = Ppre*J3.transpose()*((J3*Ppre*J3.transpose()+Ra).inverse());
        Vector4d qe1 = Kk*(Za - R3);
        Matrix4d Pk1 = (Matrix4d::Identity()-Kk*J3)*Ppre;
        
        Vector3d h2 = bx * R1 + bz * R3;
        MatrixXd Hk2(3,4);
        Hk2 = bx * J1 + bz * J3;
        Kk = Ppre*Hk2.transpose()*((Hk2*Ppre*Hk2.transpose()+Rm).inverse());
        Vector4d qe2 = Kk*(Zm - h2);
        P = (Matrix4d::Identity()-Kk*Hk2)*Pk1;
        x = x + qe1 + qe2;
        
        // renormalise quaternion
        x = x/x.norm();
        q[0] = x(0);
        q[1] = x(1);
        q[2] = x(2);
        q[3] = x(3);
        return true;
    }

};

#endif  // QUATERNIONFILTER_H
