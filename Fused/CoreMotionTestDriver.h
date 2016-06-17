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
- (nullable instancetype)initSampleFrequencyHz:(float)sampleFrequencyHz
                                          beta:(float)beta;

// Starts the driver.
- (void)start;

// Stop the driver.
- (void)stop;

@end