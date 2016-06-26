//
//  MahonySensorFusion.m
//  Fused
//
//  Implementation of Mahony IMU and AHRS algorithms.
//  See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
//  Date       Author          Notes
//  --------------------------------------------------------------------------------------------------
//  29/09/2011	SOH Madgwick    Initial release
//  02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//  02/06/2016  Brian Lambert   Ported to Objective-C.
//

#include <math.h>
#import "MahonySensorFusion.h"

// Inverse square root.
static inline float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
//    float xhalf = 0.5f * x;
//    int i = *(int*)&x;              // get bits for floating value
//    i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
//    x = *(float*)&i;                // convert bits back to float
//    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
//    return x;
}

// MahonySensorFusion implementation.
@implementation MahonySensorFusion
{
@private
    // Sample frequency.
    float _sampleFrequencyHz;

    // 2 * proportional gain (Kp).
    float _twoKp;

    // 2 * integral gain (Ki).
    float _twoKi;
    
    // Integral error terms scaled by Ki.
    float _integralFBx;
    float _integralFBy;
    float _integralFBz;
}

// Class initializer.
- (nullable instancetype)initWithSampleFrequencyHz:(float)sampleFrequencyHz
                                             twoKp:(float)twoKp
                                             twoKi:(float)twoKi
{
    // Initialize superclass.
    self = [super init];
    
    // Handle errors.
    if (!self)
    {
        return nil;
    }
    
    _sampleFrequencyHz = sampleFrequencyHz;
    _twoKp = twoKp;
    _twoKi = twoKi;
    _q0 = 1.0f;
    _q1 = 0.0f;
    _q2 = 0.0f;
    _q3 = 0.0f;
    
    // Done.
    return self;
}

// Update with gyroscope and accelerometer.
- (void)updateWithGyroscopeX:(float)gx
                  gyroscopeY:(float)gy
                  gyroscopeZ:(float)gz
              accelerometerX:(float)ax
              accelerometerY:(float)ay
              accelerometerZ:(float)az
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = _q1 * _q3 - _q0 * _q2;
        halfvy = _q0 * _q1 + _q2 * _q3;
        halfvz = _q0 * _q0 - 0.5f + _q3 * _q3;
        
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        
        // Compute and apply integral feedback if enabled
        if (_twoKi > 0.0f)
        {
            _integralFBx += _twoKi * halfex * (1.0f / _sampleFrequencyHz);	// integral error scaled by Ki
            _integralFBy += _twoKi * halfey * (1.0f / _sampleFrequencyHz);
            _integralFBz += _twoKi * halfez * (1.0f / _sampleFrequencyHz);
            gx += _integralFBx;	// apply integral feedback
            gy += _integralFBy;
            gz += _integralFBz;
        }
        else
        {
            _integralFBx = 0.0f;	// prevent integral windup
            _integralFBy = 0.0f;
            _integralFBz = 0.0f;
        }
        
        // Apply proportional feedback
        gx += _twoKp * halfex;
        gy += _twoKp * halfey;
        gz += _twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / _sampleFrequencyHz));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / _sampleFrequencyHz));
    gz *= (0.5f * (1.0f / _sampleFrequencyHz));
    qa = _q0;
    qb = _q1;
    qc = _q2;
    _q0 += (-qb * gx - qc * gy - _q3 * gz);
    _q1 += (qa * gx + qc * gz - _q3 * gy);
    _q2 += (qa * gy - qb * gz + _q3 * gx);
    _q3 += (qa * gz + qb * gy - qc * gx);
    
    // Normalise quaternion
    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

// Update with gyroscope, accelerometer and magnetometer.
- (void)updateWithGyroscopeX:(float)gx
                  gyroscopeY:(float)gy
                  gyroscopeZ:(float)gz
              accelerometerX:(float)ax
              accelerometerY:(float)ay
              accelerometerZ:(float)az
               magnetometerX:(float)mx
               magnetometerY:(float)my
               magnetometerZ:(float)mz
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        [self updateWithGyroscopeX:gx
                        gyroscopeY:gy
                        gyroscopeZ:gz
                    accelerometerX:ax
                    accelerometerY:ay
                    accelerometerZ:az];
        return;
    }
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = _q0 * _q0;
        q0q1 = _q0 * _q1;
        q0q2 = _q0 * _q2;
        q0q3 = _q0 * _q3;
        q1q1 = _q1 * _q1;
        q1q2 = _q1 * _q2;
        q1q3 = _q1 * _q3;
        q2q2 = _q2 * _q2;
        q2q3 = _q2 * _q3;
        q3q3 = _q3 * _q3;
        
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        
        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        
        // Compute and apply integral feedback if enabled
        if (_twoKi > 0.0f)
        {
            _integralFBx += _twoKi * halfex * (1.0f / _sampleFrequencyHz);	// integral error scaled by Ki
            _integralFBy += _twoKi * halfey * (1.0f / _sampleFrequencyHz);
            _integralFBz += _twoKi * halfez * (1.0f / _sampleFrequencyHz);
            gx += _integralFBx;	// apply integral feedback
            gy += _integralFBy;
            gz += _integralFBz;
        }
        else
        {
            _integralFBx = 0.0f;	// prevent integral windup
            _integralFBy = 0.0f;
            _integralFBz = 0.0f;
        }
        
        // Apply proportional feedback
        gx += _twoKp * halfex;
        gy += _twoKp * halfey;
        gz += _twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / _sampleFrequencyHz));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / _sampleFrequencyHz));
    gz *= (0.5f * (1.0f / _sampleFrequencyHz));
    qa = _q0;
    qb = _q1;
    qc = _q2;
    _q0 += (-qb * gx - qc * gy - _q3 * gz);
    _q1 += (qa * gx + qc * gz - _q3 * gy);
    _q2 += (qa * gy - qb * gz + _q3 * gx);
    _q3 += (qa * gz + qb * gy - qc * gx);
    
    // Normalise quaternion
    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

@end