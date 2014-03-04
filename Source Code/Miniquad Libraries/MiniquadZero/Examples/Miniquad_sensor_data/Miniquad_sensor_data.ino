#include <Wire.h>
#include <MiniquadZero.h>

// The global instance for MiniquadZero
MiniquadZero copter;

// CONFIG: Data displayment configuration
//#define DISPLAY_ROTATION
#define DISPLAY_QUATERNION
//#define DISPLAY_EULER_ANGLE
//#define DISPLAY_YPR
//#define DISPLAY_GRAVITY
//#define DISPLAY_LINEAR_ACCEL
//#define DISPLAY_WORLD_ACCEL
//#define DISPLAY_TEMPERATURE


void setup()
{
  // Wire initialization
  Wire.begin();
  
  // Miniquad Zero initialization
  copter.Initialize();
  
  // Serial initialization (just for data displayment)
  Serial.begin(115200);
}

void loop()
{
  // Refresh the DMP data to get new data from AccelGyro sensor (necessary)
  copter.RefreshDmpData();
  
#ifdef DISPLAY_ROTATION
  // Obtain the rotation speeds about each axis
  Rotation& rot = copter.GetRotation();
  Serial.print(rot.getX()); Serial.print("\t");
  Serial.print(rot.getY()); Serial.print("\t");
  Serial.print(rot.getZ()); Serial.print("\n");
#endif
  
#ifdef DISPLAY_QUATERNION
  // Obtain the Quaternion
  Quaternion& quat = copter.GetQuaternion();
  Serial.print(quat.w); Serial.print("\t");
  Serial.print(quat.x); Serial.print("\t");
  Serial.print(quat.y); Serial.print("\t");
  Serial.print(quat.z); Serial.print("\n");
#endif
  
#ifdef DISPLAY_EULER_ANGLE
  // Obtain the Euler angle
  EulerAngle& euler = copter.GetEulerAngle();
  Serial.print(euler.getPsi()); Serial.print("\t");
  Serial.print(euler.getTheta()); Serial.print("\t");
  Serial.print(euler.getPhi()); Serial.print("\n");
#endif
  
#ifdef DISPLAY_YPR
  // Obtain the yaw, pitch, roll angles
  YawPitchRoll& ypr = copter.GetYawPitchRoll();
  Serial.print(ypr.getYaw()); Serial.print("\t");
  Serial.print(ypr.getPitch()); Serial.print("\t");
  Serial.print(ypr.getRoll()); Serial.print("\n");
#endif
  
#ifdef DISPLAY_GRAVITY
  // Obtain the gravity components
  Gravity& grav = copter.GetGravity();
  Serial.print(grav.getX()); Serial.print("\t");
  Serial.print(grav.getY()); Serial.print("\t");
  Serial.print(grav.getZ()); Serial.print("\n");
#endif
  
#ifdef DISPLAY_LINEAR_ACCEL
  // Obtain the linear acceleration without gravity
  Acceleration& accel = copter.GetLinearAcceleration();
  Serial.print(accel.getX()); Serial.print("\t");
  Serial.print(accel.getY()); Serial.print("\t");
  Serial.print(accel.getZ()); Serial.print("\n");
#endif
  
#ifdef DISPLAY_WORLD_ACCEL
  // Obtain the world acceleration with gravity
  Acceleration& accelW = copter.GetWorldAcceleration();
  Serial.print(accelW.getX()); Serial.print("\t");
  Serial.print(accelW.getY()); Serial.print("\t");
  Serial.print(accelW.getZ()); Serial.print("\n");
#endif

#ifdef DISPLAY_TEMPERATURE
  // Obtain the temperature
  Serial.print(copter.GetTemperature()); Serial.print("\n");
#endif
}

