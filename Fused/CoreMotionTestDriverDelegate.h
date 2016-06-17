//
//  CoreMotionTestDriverDelegate.h
//  Fused
//
//  Created by Brian Lambert on 6/11/16.
//  Copyright © 2016 Softwarenerd. All rights reserved.
//

// Forward declarations.
@class CoreMotionTestDriver;

// CoreMotionTestDriverDelegate protocol.
@protocol CoreMotionTestDriverDelegate <NSObject>
@required

// Notifies the delegate of an update.
- (void)CoreMotionTestDriver:(CoreMotionTestDriver *)CoreMotionTestDriver
                 didUpdateGyroscopeX:(float)gx
                          gyroscopeY:(float)gy
                          gyroscopeZ:(float)gz
                      accelerometerX:(float)ax
                      accelerometerY:(float)ay
                      accelerometerZ:(float)az
                       magnetometerX:(float)mx
                       magnetometerY:(float)my
                       magnetometerZ:(float)mz
                         quaternion0:(float)q0
                         quaternion1:(float)q1
                         quaternion2:(float)q2
                         quaternion3:(float)q3
                                roll:(float)roll
                               pitch:(float)pitch
                                 yaw:(float)yaw
                      coreMotionRoll:(float)coreMotionRoll
                     coreMotionPitch:(float)coreMotionPitch
                       coreMotionYaw:(float)coreMotionYaw;

@end
