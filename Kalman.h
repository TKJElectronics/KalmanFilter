/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _Kalman_h
#define _Kalman_h

class Kalman {
public:
    Kalman() {
        /* We will set the varibles like so, these can also be tuned by the user */
        Q_angle = 0.001;
        Q_gyroBias = 0.003;
        R_angle = 0.03;
        bias = 0; // Reset bias
        P[0][0] = 1; // Since we don't know the initial angle and bias we set it like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 1;
    };
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    double getAngle(double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/08/a-practical-approach-to-kalman-filter-and-how-to-implement-it
                        
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        rate = newRate - bias;
        angle += dt * rate;
        
        // Update estimation error covariance - Project the error covariance ahead
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_gyroBias * dt;
        
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        S = P[0][0] + R_angle;
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;
        
        // Calculate estimation error covariance - Update the error covariance
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];
        
        return angle;
    };
    void setAngle(double newAngle) { angle = newAngle; } // Used to set angle, this should be set as the starting angle
    double getRate() { return rate; }; // Return the unbiased rate
    
    /* These are used to tune the Kalman filter */
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQgyroBias(double newQ_gyroBias) { Q_gyroBias = newQ_gyroBias; };
    void setRangle(double newR_angle) { R_angle = newR_angle; };
    
private:
    /* Kalman filter variables */
    double Q_angle; // Process noise covariance for the accelerometer - (w = process noise)
    double Q_gyroBias; // Process noise covariance for the gyro bias - (w = process noise)
    double R_angle; // Measurement noise covariance - this is actually the variance of the measurement noise - (v = measurement noise)
    
    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    
    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 matrix
    double y; // Angle error - 1x1 matrix
    double S; // Estimate error - 1x1 matrix
};

#endif
