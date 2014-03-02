#include <Wire.h>
#include <MiniquadZero.h>

// The global instance for MiniquadZero
MiniquadZero copter;

// Temp data
float temp;
uint8_t throttle[4] = {0};
char charReader;
uint8_t receivedThrottle[4] = {0};
int receivedCount = 0;

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
  // === Read from sensor and send to the PC ===
  // Refresh DMP data
  copter.RefreshDmpData();
  
  // Package head
  Serial.write('$');
  Serial.write(2);
  
  // Obtain the Quaternion
  Quaternion& quat = copter.GetQuaternion();
  temp = quat.w;
  Serial.write((uint8_t*)(&(temp)),4);
  temp = quat.x;
  Serial.write((uint8_t*)(&(temp)),4);
  temp = quat.y;
  Serial.write((uint8_t*)(&(temp)),4);
  temp = quat.z;
  Serial.write((uint8_t*)(&(temp)),4);
  
  // Obtain the rotation speeds about each axis
  Rotation& rot = copter.GetRotation();
  temp = rot.getX();
  Serial.write((uint8_t*)(&(temp)),4);
  temp = rot.getY();
  Serial.write((uint8_t*)(&(temp)),4);
  temp = rot.getZ();
  Serial.write((uint8_t*)(&(temp)),4);
  
  // Obtain the linear acceleration without gravity
  Acceleration& accel = copter.GetWorldAcceleration();
  temp = accel.getX();
  Serial.write((uint8_t*)(&(temp)),4);
  temp = accel.getY();
  Serial.write((uint8_t*)(&(temp)),4);
  temp = accel.getZ();
  Serial.write((uint8_t*)(&(temp)),4);
  
  // Propeller throttle
  Serial.write(throttle[0]); Serial.write(0);
  Serial.write(throttle[1]); Serial.write(0);
  Serial.write(throttle[2]); Serial.write(0);
  Serial.write(throttle[3]); Serial.write(0);
  
  // Package ending
  Serial.print("\r\n");
  
  
  // === Set the throttles by data read from PC ===
  // Read from serial port
  charReader = Serial.read();
  while(charReader > -1)
  {
    // Message received
    switch(receivedCount)
    {
      case 0:
        if(charReader == '@') ++receivedCount;
        break;
      case 1:
        if(charReader == 4) ++receivedCount;
        else receivedCount = 0;
        break;
      case 3: case 5: case 7: case 9:
        if(charReader == 0) ++receivedCount;
        else receivedCount = 0;
        break;
      case 2: case 4: case 6: case 8:
        receivedThrottle[receivedCount/2-1] = charReader;
        ++receivedCount;
        break;
      case 10:
        if(charReader == '\r') ++receivedCount;
        else receivedCount = 0;
        break;
      case 11:
        if(charReader == '\n')
        {
          throttle[0] = receivedThrottle[0]; copter.PropellerSetSpeed(PROPELLER1, throttle[0]);
          throttle[1] = receivedThrottle[1]; copter.PropellerSetSpeed(PROPELLER2, throttle[1]);
          throttle[2] = receivedThrottle[2]; copter.PropellerSetSpeed(PROPELLER3, throttle[2]);
          throttle[3] = receivedThrottle[3]; copter.PropellerSetSpeed(PROPELLER4, throttle[3]);
          Serial.flush(); //while(Serial.read() > -1);
        }
        receivedCount = 0;
        break;
      default:
        receivedCount = 0;
    }
    charReader = Serial.read();
  }
  
  // Delay time
  //delay(2);
}

