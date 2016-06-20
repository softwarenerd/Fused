//
//  SensorFusionProcessor.h
//  Fused
//
//  Created by Brian Lambert on 6/20/16.
//  Copyright © 2016 Brian Lambert. All rights reserved.
//

// SensorFusionProcessor protocol.
@protocol SensorFusionProcessor <NSObject>

// Quaternion properties.
@property (nonatomic, readonly) float q0;
@property (nonatomic, readonly) float q1;
@property (nonatomic, readonly) float q2;
@property (nonatomic, readonly) float q3;

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
