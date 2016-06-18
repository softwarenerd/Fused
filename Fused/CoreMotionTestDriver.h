//
//  CoreMotionTestDriver.h
//  Fused
//
//  Created by Brian Lambert on 6/7/16.
//
//  CoreMotionTestDriver uses CoreMotion device motion updates as IMU input data to
//  Fused. It then uses the resulting quaternion to calculate Euler angles.
//

#import <Foundation/Foundation.h>
#import "CoreMotionTestDriverDelegate.h"

// CoreMotionTestDriver interface.
@interface CoreMotionTestDriver : NSObject

// Properties.
@property (nonatomic, weak, nullable) id<CoreMotionTestDriverDelegate> delegate;

// Class initializer.
- (nullable instancetype)initMadgwickSensorFusionWithSampleFrequencyHz:(float)sampleFrequencyHz
                                                                  beta:(float)beta;

// Converts from radians to degrees.
+ (float)degreesFromRadians:(float)radians;

// Converts from degrees to radians.
+ (float)radiansFromDegrees:(float)degrees;

// Calculates Euler angles from quaternion.
+ (void)calculateEulerAnglesFromQuaternionQ0:(float)q0
                                          q1:(float)q1
                                          q2:(float)q2
                                          q3:(float)q3
                                        roll:(nonnull float *)roll
                                       pitch:(nonnull float *)pitch
                                         yaw:(nonnull float *)yaw;

// Starts the driver.
- (void)start;

// Stop the driver.
- (void)stop;

@end
