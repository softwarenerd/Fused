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

// Converts from radians to degrees.
+ (float)degreesFromRadians:(float)radians;

// Converts from degrees to radians.
+ (float)radiansFromDegrees:(float)degrees;

// Calculates Euler angles from quaternion.
+ (void)calculateEulerAnglesFromQuaternionQ0:(float)q0
                                          Q1:(float)q1
                                          Q2:(float)q2
                                          Q3:(float)q3
                                        roll:(float *)roll
                                       pitch:(float *)pitch
                                         yaw:(float *)yaw;

// Class initializer.
- (nullable instancetype)initSampleFrequencyHz:(float)sampleFrequencyHz
                                          beta:(float)beta;

// Starts the driver.
- (void)start;

// Stop the driver.
- (void)stop;

@end
