//
//  MahonySensorFusion.h
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

@import Foundation;

// MahonySensorFusion interface.
@interface MahonySensorFusion : NSObject

// Quaternion properties.
@property (nonatomic, readonly) float q0;
@property (nonatomic, readonly) float q1;
@property (nonatomic, readonly) float q2;
@property (nonatomic, readonly) float q3;

// Class initializer.
- (nullable instancetype)initWithSampleFrequencyHz:(float)sampleFrequencyHz
                                             twoKp:(float)twoKp
                                             twoKi:(float)twoKi;

// Update with gyroscope and accelerometer.
- (void)updateWithGyroscopeX:(float)gx
                  gyroscopeY:(float)gy
                  gyroscopeZ:(float)gz
              accelerometerX:(float)ax
              accelerometerY:(float)ay
              accelerometerZ:(float)az;

// Update with gyroscope, accelerometer and magnetometer.
- (void)updateWithGyroscopeX:(float)gx
                  gyroscopeY:(float)gy
                  gyroscopeZ:(float)gz
              accelerometerX:(float)ax
              accelerometerY:(float)ay
              accelerometerZ:(float)az
               magnetometerX:(float)mx
               magnetometerY:(float)my
               magnetometerZ:(float)mz;

@end
