//
//  CoreMotionTestDriver.m
//  Fused
//
//  Created by Brian Lambert on 6/7/16.
//
//  CoreMotionTestDriver uses CoreMotion device motion updates as IMU input data to
//  Fused. It then uses the resulting quaternion to calculate Euler angles.
//

#import <CoreMotion/CoreMotion.h>
#import "CoreMotionTestDriver.h"
#import "MadgwickSensorFusion.h"
#import "MahonySensorFusion.h"

// CoreMotionTestDriver (Internal) interface.
@interface CoreMotionTestDriver (Internal)

// Common initialization.
- (void)commonInitializationWithCoreMotionSamplingFrequencyHz:(NSUInteger)coreMotionSamplingFrequencyHz
                          sensorFusionOversamplingFrequencyHz:(NSUInteger)sensorFusionOversamplingFrequencyHz;

@end

// CoreMotionTestDriver implementation.
@implementation CoreMotionTestDriver
{
@private
    // The motion manager.
    CMMotionManager * _motionManager;
    
    // The operation queue.
    NSOperationQueue * _operationQueue;
    
    // The Core Motion sampling frquency in Hz.
    NSUInteger _coreMotionSamplingFrequencyHz;
    
    // The sensor fusion oversampling frequency in Hz.
    NSUInteger _sensorFusionOversamplingFrequencyHz;
    
    // The Madgwick sensor fusion processor.
    MadgwickSensorFusion * _madgwickSensorFusion;
    
    // The Mahony sensor fusion processor.
    MahonySensorFusion * _mahonySensorFusion;
}

// Converts from radians to degrees.
+ (float)degreesFromRadians:(float)radians
{
    return radians * 180.0f / (float)M_PI;
}

// Converts from degrees to radians.
+ (float)radiansFromDegrees:(float)degrees
{
    return degrees * (float)M_PI / 180.0f;
}

// Calculates Euler angles from quaternion.
+ (void)testCalculateEulerAnglesFromQuaternionQ0:(float)q0
                                          q1:(float)q1
                                          q2:(float)q2
                                          q3:(float)q3
                                        roll:(nonnull float *)roll
                                       pitch:(nonnull float *)pitch
                                         yaw:(nonnull float *)yaw
{
    *yaw   = atan2f(q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2.0f * (q1 * q2 - q0 * q3));
    *pitch = asinf(2.0f * (q2 * q3 + q0 * q1));
    *roll  = atan2f(2.0f * (q0 * q2 - q1 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

// Calculates Euler angles from quaternion.
+ (void)calculateEulerAnglesFromQuaternionQ0:(float)q0
                                          q1:(float)q1
                                          q2:(float)q2
                                          q3:(float)q3
                                        roll:(nonnull float *)roll
                                       pitch:(nonnull float *)pitch
                                         yaw:(nonnull float *)yaw
{
    const float w2 = q0 * q0;
    const float x2 = q1 * q1;
    const float y2 = q2 * q2;
    const float z2 = q3 * q3;
    const float unitLength = w2 + x2 + y2 + z2;     // Normalised == 1, otherwise correction divisor.
    const float abcd = q0 * q1 + q2 * q3;
    const float eps = 1e-7f;                        // TODO: pick from your math lib instead of hardcoding.
    if (abcd > (0.5f - eps) * unitLength)
    {
        *roll = 0.0f;
        *pitch = (float)M_PI;
        *yaw = 2.0f * atan2f(q2, q0);
    }
    else if (abcd < (-0.5f + eps) * unitLength)
    {
        *roll  = 0.0f;
        *pitch = (float)-M_PI;
        *yaw   = -2.0f * atan2(q2, q0);
    }
    else
    {
        const float adbc = q0 * q3 - q1 * q2;
        const float acbd = q0 * q2 - q1 * q3;
        *roll  = atan2f(2.0f * acbd, 1.0f - 2.0f * (y2 + x2));
        *pitch = asinf(2.0f * abcd / unitLength);
        *yaw   = atan2f(2.0f * adbc, 1.0f - 2.0f * (z2 + x2));
    }
}

// Class initializer.
- (nullable instancetype)initMadgwickSensorFusionWithCoreMotionSamplingFrequencyHz:(NSUInteger)coreMotionSamplingFrequencyHz
                                               sensorFusionOversamplingFrequencyHz:(NSUInteger)sensorFusionOversamplingFrequencyHz
                                                                              beta:(float)beta
{
    // Initialize superclass.
    self = [super init];
    
    // Handle errors.
    if (!self)
    {
        return nil;
    }
    
    // Initialize.
    [self commonInitializationWithCoreMotionSamplingFrequencyHz:coreMotionSamplingFrequencyHz
                            sensorFusionOversamplingFrequencyHz:sensorFusionOversamplingFrequencyHz];
    
    // Allocate and initialize the Madgwick sensor fusion.
    _madgwickSensorFusion = [[MadgwickSensorFusion alloc] initWithSampleFrequencyHz:(float)sensorFusionOversamplingFrequencyHz
                                                                               beta:beta];
    
    // Done.
    return self;
}

// Class initializer.
- (nullable instancetype)initMahonySensorFusionWithCoreMotionSamplingFrequencyHz:(NSUInteger)coreMotionSamplingFrequencyHz
                                             sensorFusionOversamplingFrequencyHz:(NSUInteger)sensorFusionOversamplingFrequencyHz
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
    
    // Initialize.
    [self commonInitializationWithCoreMotionSamplingFrequencyHz:coreMotionSamplingFrequencyHz
                            sensorFusionOversamplingFrequencyHz:sensorFusionOversamplingFrequencyHz];
    
    // Allocate and initialize the Mahony sensor fusion.
    _mahonySensorFusion = [[MahonySensorFusion alloc] initWithSampleFrequencyHz:(float)sensorFusionOversamplingFrequencyHz
                                                                          twoKp:twoKp
                                                                          twoKi:twoKi];
    
    // Done.
    return self;
}

// Starts the driver.
- (void)start
{
    // The Madgwick device motion handler.
    CMDeviceMotionHandler madgwickhandler = ^(CMDeviceMotion * _Nullable motion, NSError * _Nullable error)
    {
        // Perform sensor fusion.
        for (NSUInteger i = 0; i < _sensorFusionOversamplingFrequencyHz / _coreMotionSamplingFrequencyHz; i++)
        {
            [_madgwickSensorFusion updateWithGyroscopeX:(float)[motion rotationRate].x
                                             gyroscopeY:(float)[motion rotationRate].y
                                             gyroscopeZ:(float)[motion rotationRate].z
                                         accelerometerX:(float)[motion gravity].x * -1.0    // Accelerometer angles are inverted.
                                         accelerometerY:(float)[motion gravity].y * -1.0    // Accelerometer angles are inverted.
                                         accelerometerZ:(float)[motion gravity].z * -1.0    // Accelerometer angles are inverted.
                                          magnetometerX:(float)[motion magneticField].field.x
                                          magnetometerY:(float)[motion magneticField].field.y
                                          magnetometerZ:(float)[motion magneticField].field.z];
        }
        
        // Calculate roll, pitch, yaw.
        float roll, pitch, yaw;
        [CoreMotionTestDriver testCalculateEulerAnglesFromQuaternionQ0:[_madgwickSensorFusion q0]
                                                                q1:[_madgwickSensorFusion q1]
                                                                q2:[_madgwickSensorFusion q2]
                                                                q3:[_madgwickSensorFusion q3]
                                                              roll:&roll
                                                             pitch:&pitch
                                                               yaw:&yaw];
        roll = [CoreMotionTestDriver degreesFromRadians:roll];
        pitch = [CoreMotionTestDriver degreesFromRadians:pitch];
        yaw = [CoreMotionTestDriver degreesFromRadians:yaw];
        
        // Obtain CoreMotion roll, pitch and yaw for comparison logging below.
        float coreMotionRoll = [CoreMotionTestDriver degreesFromRadians:[[motion attitude] roll]];
        float coreMotionPitch = [CoreMotionTestDriver degreesFromRadians:[[motion attitude] pitch]];
        float coreMotionYaw = [CoreMotionTestDriver degreesFromRadians:[[motion attitude] yaw]];
        
        // Notify the delegate.
        if ([[self delegate] respondsToSelector:@selector(coreMotionTestDriver:
                                                          didUpdateGyroscopeX:
                                                          gyroscopeY:
                                                          gyroscopeZ:
                                                          accelerometerX:
                                                          accelerometerY:
                                                          accelerometerZ:
                                                          magnetometerX:
                                                          magnetometerY:
                                                          magnetometerZ:
                                                          q0:
                                                          q1:
                                                          q2:
                                                          q3:
                                                          roll:
                                                          pitch:
                                                          yaw:
                                                          coreMotionRoll:
                                                          coreMotionPitch:
                                                          coreMotionYaw:)])
        {
            [[self delegate] coreMotionTestDriver:self
                              didUpdateGyroscopeX:[motion rotationRate].x
                                       gyroscopeY:[motion rotationRate].y
                                       gyroscopeZ:[motion rotationRate].z
                                   accelerometerX:[motion gravity].x * -1.0f
                                   accelerometerY:[motion gravity].y * -1.0f
                                   accelerometerZ:[motion gravity].z * -1.0f
                                    magnetometerX:[motion magneticField].field.x
                                    magnetometerY:[motion magneticField].field.y
                                    magnetometerZ:[motion magneticField].field.z
                                               q0:[_madgwickSensorFusion q0]
                                               q1:[_madgwickSensorFusion q1]
                                               q2:[_madgwickSensorFusion q2]
                                               q3:[_madgwickSensorFusion q3]
                                             roll:roll
                                            pitch:pitch
                                              yaw:yaw
                                   coreMotionRoll:coreMotionRoll
                                  coreMotionPitch:coreMotionPitch
                                    coreMotionYaw:coreMotionYaw];
        }
    };
    
    // The Mahony device motion handler.
    CMDeviceMotionHandler mahonyHandlerhandler = ^(CMDeviceMotion * _Nullable motion, NSError * _Nullable error)
    {
        // Perform sensor fusion.
        for (NSUInteger i = 0; i < _sensorFusionOversamplingFrequencyHz / _coreMotionSamplingFrequencyHz; i++)
        {
            [_mahonySensorFusion updateWithGyroscopeX:(float)[motion rotationRate].x
                                           gyroscopeY:(float)[motion rotationRate].y
                                           gyroscopeZ:(float)[motion rotationRate].z
                                       accelerometerX:(float)[motion gravity].x * -1.0    // Accelerometer angles inverted.
                                       accelerometerY:(float)[motion gravity].y * -1.0    // Accelerometer angles inverted.
                                       accelerometerZ:(float)[motion gravity].z * -1.0    // Accelerometer angles inverted.
                                        magnetometerX:(float)[motion magneticField].field.x
                                        magnetometerY:(float)[motion magneticField].field.y
                                        magnetometerZ:(float)[motion magneticField].field.z];
        }
        
        // Calculate roll, pitch, yaw.
        float roll, pitch, yaw;
        [CoreMotionTestDriver testCalculateEulerAnglesFromQuaternionQ0:[_mahonySensorFusion q0]
                                                                q1:[_mahonySensorFusion q1]
                                                                q2:[_mahonySensorFusion q2]
                                                                q3:[_mahonySensorFusion q3]
                                                              roll:&roll
                                                             pitch:&pitch
                                                               yaw:&yaw];
        roll = [CoreMotionTestDriver degreesFromRadians:roll];
        pitch = [CoreMotionTestDriver degreesFromRadians:pitch];
        yaw = [CoreMotionTestDriver degreesFromRadians:yaw];
        
        // Obtain CoreMotion roll, pitch and yaw for comparison logging below.
        float coreMotionRoll = [CoreMotionTestDriver degreesFromRadians:[[motion attitude] roll]];
        float coreMotionPitch = [CoreMotionTestDriver degreesFromRadians:[[motion attitude] pitch]];
        float coreMotionYaw = [CoreMotionTestDriver degreesFromRadians:[[motion attitude] yaw]];
        
        // Notify the delegate.
        if ([[self delegate] respondsToSelector:@selector(coreMotionTestDriver:
                                                          didUpdateGyroscopeX:
                                                          gyroscopeY:
                                                          gyroscopeZ:
                                                          accelerometerX:
                                                          accelerometerY:
                                                          accelerometerZ:
                                                          magnetometerX:
                                                          magnetometerY:
                                                          magnetometerZ:
                                                          q0:
                                                          q1:
                                                          q2:
                                                          q3:
                                                          roll:
                                                          pitch:
                                                          yaw:
                                                          coreMotionRoll:
                                                          coreMotionPitch:
                                                          coreMotionYaw:)])
        {
            [[self delegate] coreMotionTestDriver:self
                              didUpdateGyroscopeX:[motion rotationRate].x
                                       gyroscopeY:[motion rotationRate].y
                                       gyroscopeZ:[motion rotationRate].z
                                   accelerometerX:[motion gravity].x * -1.0f
                                   accelerometerY:[motion gravity].y * -1.0f
                                   accelerometerZ:[motion gravity].z * -1.0f
                                    magnetometerX:[motion magneticField].field.x
                                    magnetometerY:[motion magneticField].field.y
                                    magnetometerZ:[motion magneticField].field.z
                                               q0:[_mahonySensorFusion q0]
                                               q1:[_mahonySensorFusion q1]
                                               q2:[_mahonySensorFusion q2]
                                               q3:[_mahonySensorFusion q3]
                                             roll:roll
                                            pitch:pitch
                                              yaw:yaw
                                   coreMotionRoll:coreMotionRoll
                                  coreMotionPitch:coreMotionPitch
                                    coreMotionYaw:coreMotionYaw];
        }
    };
    
    // Start motion updates.
    [_motionManager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
                                                        toQueue:_operationQueue
                                                    withHandler:_madgwickSensorFusion ? madgwickhandler : mahonyHandlerhandler];
}

// Stops the driver.
- (void)stop
{
    // Stop motion updates.
    [_motionManager stopDeviceMotionUpdates];
}

@end

// CoreMotionTestDriver (Internal) implementation
@implementation CoreMotionTestDriver (Internal)

// Common initialization.
- (void)commonInitializationWithCoreMotionSamplingFrequencyHz:(NSUInteger)coreMotionSamplingFrequencyHz
                          sensorFusionOversamplingFrequencyHz:(NSUInteger)sensorFusionOversamplingFrequencyHz;
{
    // Initialize sampling rates.
    _coreMotionSamplingFrequencyHz = coreMotionSamplingFrequencyHz;
    _sensorFusionOversamplingFrequencyHz = MAX(coreMotionSamplingFrequencyHz, sensorFusionOversamplingFrequencyHz);
    
    // Allocate and initialize the motion manager.
    _motionManager = [[CMMotionManager alloc] init];
    [_motionManager setShowsDeviceMovementDisplay:YES];
    [_motionManager setDeviceMotionUpdateInterval:1.0 / (NSTimeInterval)coreMotionSamplingFrequencyHz];
    
    // Allocate and initialize the operation queue.
    _operationQueue = [[NSOperationQueue alloc] init];
    [_operationQueue setName:@"DeviceMotion"];
    [_operationQueue setMaxConcurrentOperationCount:1];
}

@end