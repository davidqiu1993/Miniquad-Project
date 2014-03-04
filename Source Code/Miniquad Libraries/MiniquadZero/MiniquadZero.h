// Copyright (C) 2013 Robot Club, Sun Yat-Sen University. All rights reserved. 
// Author: David Qiu (ÇñµÏ´Ï) <david@davidqiu.com>
// Update:
//		- 2013.6.30 : Created by David Qiu <david@davidqiu.com>
//		- 2013.7.10 : MPU6050 DMP functions added by David Qiu <david@davidqiu.com>
//		- 2013.8.13 : MPU6050 DMP rotation function taken into use by 
//                    David Qiu <david@davidqiu.com>
//		- 2013.8.14 : MPU6050 DMP data refreshment program routine changed 
//					  by David Qiu <david@davidqiu.com>
//
// This is a standard library for the quadaxis copter "Miniquad Zero" (C). The 
// following functions are included:
//		- Propeller motors controlling
//		- Gyroscope sensor monitoring (MPU6050)
//		- Accelerometer sensor monitoring (MPU6050)
//		- Temperature sensor monitoring (MPU6050)
//
// It is free to use this library within the Robot Club. The copyright belongs to 
// the Robot Club, and the right authorship of each part belongs to its contributers.
// === [ Robot Club ] ===

#ifndef _MINIQUADZERO_H_
#define _MINIQUADZERO_H_

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Miniquad_3dmath.h"


// Config: If the DMP data should be kept for calculation
#define MINIQUAD_DMP_KEEP_DATA

// Define: Miniquad-MPU6050 interrupt pin
#define MPU6050_INT_PIN (0)

// Define: MPU6050 sensor detection data transformation units and skewing
#define MPU6050_ACCELERATION_UNIT (16384.0f) // AFS_0: ([+-RawAccel] / (65536(int16)/2)) * 2g  ==>  65536/4 = 16384 per g
#define MPU6050_ROTATION_UNIT (131.0f) // FS_0: ([+-RawRotation] / (65536(int16)/2)) * 250 degree/s ==>  131 per degree/s
#define MPU6050_QUATERNION_UNIT (16384.0f) // Quaternion conversion unit
#define MPU6050_GRAVITY_UNIT (8192.0f) // Gravity conversion unit
#define MPU6050_TEMPERATURE_UNIT (340.0f) // 65536 / Range([RawTemp]) = 340 per degree Celsius
#define MPU6050_TEMPERATURE_SKEWING (-12412.0f) // -512 - (340 * 35) = -12412  <=>  0 degree Celsius

// Define: Propellers
#define PROPELLER1 (3)
#define PROPELLER2 (5)
#define PROPELLER3 (6)
#define PROPELLER4 (9)
#define PPL1 PROPELLER1
#define PPL2 PROPELLER2
#define PPL3 PROPELLER3
#define PPL4 PROPELLER4


// Global: MPU6050 interrupt (INT 1)
// @Function:		Signal that the DMP module of MPU6050 is ready
// @Contributor:	David Qiu (2013.7.1)
volatile bool MpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high
void MpuDataReady()
{
	MpuInterrupt = true;
}


// Class: Quadaxis copter "Miniquad Zero"
class MiniquadZero
{
public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Initialize the quadaxis copter.
	// @Contributor:	David Qiu (2013.6.30)
	void Initialize()
	{
		// Initialize the propeller motors
		pinMode(PROPELLER1, OUTPUT); analogWrite(PROPELLER1, 0);
		pinMode(PROPELLER2, OUTPUT); analogWrite(PROPELLER2, 0);
		pinMode(PROPELLER3, OUTPUT); analogWrite(PROPELLER3, 0);
		pinMode(PROPELLER4, OUTPUT); analogWrite(PROPELLER4, 0);

		// Initialize the MPU6050
		Wire.begin();
		_mpu.initialize();
		while(!_mpu.testConnection()) {;} // Stop: if MPU6050 connect failed

		// Initialize DMP for MPU6050
		switch (_mpu.dmpInitialize())
		{
		case 0: // Initialization successful
			// Turn on the DMP
			_mpu.setDMPEnabled(true);

			// Enable Arduino interrupt detection
			attachInterrupt(MPU6050_INT_PIN, MpuDataReady, RISING);
			_mpu.getIntStatus(); // ? test interrupt

			// Get expected DMP packet size for later comparison
			_mpuFIFOPacketSize = _mpu.dmpGetFIFOPacketSize();

			// Get the first set of data
			_refreshDMPData();
			break;
		case 1: // Initial memory load failed
			while (true) {;}
			break;
		case 2: // DMP configuration updates failed
			while (true) {;}
			break;
		default: // Unexpected exception
			while (true) {;}
			break;
		}
	}


	// @Params:			propeller_pin: The pin number of the propeller (PROPELLER1 ~ PROPELLER4)
	//					speed_level: The level of speed (0 ~ 255; for floating about 115)
	// @Return:			(void)
	// @Function:		Set the speed of a specific propeller.
	// @Contributor:	David Qiu (2013.6.30)
	void PropellerSetSpeed(int propeller_pin, char speed_level)
	{
		analogWrite(propeller_pin, speed_level);
	}

	// @Params:			speed_level: The level of speed (0 ~ 255; for floating about 115)
	// @Return:			(void)
	// @Function:		Set the speeds of all propeller to the same level.
	// @Contributor:	David Qiu (2013.6.30)
	void PropellerSetAllSpeeds(char speed_level)
	{
		analogWrite(PROPELLER1, speed_level);
		analogWrite(PROPELLER2, speed_level);
		analogWrite(PROPELLER3, speed_level);
		analogWrite(PROPELLER4, speed_level);
	}

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Set the speeds of all propeller to 0.
	// @Contributor:	David Qiu (2013.6.30)
	void PropellerStopAll()
	{
		analogWrite(PROPELLER1, 0);
		analogWrite(PROPELLER2, 0);
		analogWrite(PROPELLER3, 0);
		analogWrite(PROPELLER4, 0);
	}


	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Refresh the DMP necessary data of the quad copter
	// @Contributor:	David Qiu (2013.7.10)
	void RefreshDmpData()
	{
		_refreshDMPData();
	}

	// @Params:			(void)
	// @Return:			A float indicating the temperature (degree Celsius)
	// @Function:		Get the temperature from the temperature sensor.
	// @Contributor:	David Qiu (2013.6.30)
	float GetTemperature()
	{
		// The temperature sensor is -40 to +85 degrees Celsius.
		// It is a signed integer.
		// According to the datasheet: 
		//   340 per degrees Celsius, -512 at 35 degrees.
		// At 0 degrees: -512 - (340 * 35) = -12412
		return ((((float)_mpu.getTemperature()) - MPU6050_TEMPERATURE_SKEWING) / MPU6050_TEMPERATURE_UNIT);
	}

	// @Params:			(void)
	// @Return:			A Rotation& (!Reference) indicating the current raw rotation data of the quad copter
	// @Function:		Get the rotation data of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10), David Qiu (2013.8.13)
	Rotation& GetRotation()
	{
		#ifdef MINIQUAD_DMP_KEEP_DATA
		if(!_rotation_cal)
		{
		#endif // MINIQUAD_DMP_KEEP_DATA
			// Calculate the rotation
			_rotation.setX((float)_rot_Int16_raw.x / MPU6050_ROTATION_UNIT);
			_rotation.setY((float)_rot_Int16_raw.y / MPU6050_ROTATION_UNIT);
			_rotation.setZ((float)_rot_Int16_raw.z / MPU6050_ROTATION_UNIT);
		#ifdef MINIQUAD_DMP_KEEP_DATA
			_rotation_cal = true;
		}
		#endif // MINIQUAD_DMP_KEEP_DATA

		// Return the result
		return _rotation;
	}

	// @Params:			(void)
	// @Return:			A Quaternion& (!Reference) indicating the current rotation information of the quad copter
	// @Function:		Get the quaternion data of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10)
	Quaternion& GetQuaternion()
	{
		return _quaternion;
	}

	// @Params:			(void)
	// @Return:			A EulerAngle& (!Reference) indicating the current rotation information of the quad copter
	// @Function:		Get the Euler angle of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10)
	EulerAngle& GetEulerAngle()
	{
	#ifdef MINIQUAD_DMP_KEEP_DATA
		if(_eulerAngle_cal) return _eulerAngle; // return immediately if calculated
	#endif // !MINIQUAD_DMP_KEEP_DATA

		// Calculate the Euler angle
		_eulerAngle.setPsi((180/M_PI) * atan2(2*_quaternion.x*_quaternion.y - 2*_quaternion.w*_quaternion.z, 2*_quaternion.w*_quaternion.w + 2*_quaternion.x*_quaternion.x - 1));
		_eulerAngle.setTheta((180/M_PI) * (-asin(2*_quaternion.x*_quaternion.z + 2*_quaternion.w*_quaternion.y)));
		_eulerAngle.setPhi((180/M_PI) * atan2(2*_quaternion.y*_quaternion.z - 2*_quaternion.w*_quaternion.x, 2*_quaternion.w*_quaternion.w + 2*_quaternion.z*_quaternion.z - 1));
	#ifdef MINIQUAD_DMP_KEEP_DATA
		_eulerAngle_cal = true;
	#endif // MINIQUAD_DMP_KEEP_DATA

		// Return the result
		return _eulerAngle;
	}

	// @Params:			(void)
	// @Return:			A Gravity& (!Reference) indicating the current gravity components of the quad copter
	// @Function:		Get the gravity components of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10)
	Gravity& GetGravity()
	{
	#ifdef MINIQUAD_DMP_KEEP_DATA
		if(_gravity_cal) return _gravity;
	#endif // MINIQUAD_DMP_KEEP_DATA

		// Calculate the gravity
		_gravity.setX(2 * (_quaternion.x*_quaternion.z - _quaternion.w*_quaternion.y));
		_gravity.setY(2 * (_quaternion.w*_quaternion.x + _quaternion.y*_quaternion.z));
		_gravity.setZ(_quaternion.w*_quaternion.w - _quaternion.x*_quaternion.x - _quaternion.y*_quaternion.y + _quaternion.z*_quaternion.z);
	#ifdef MINIQUAD_DMP_KEEP_DATA
		_gravity_cal = true;
	#endif // MINIQUAD_DMP_KEEP_DATA

		// Return the result
		return _gravity;
	}

	// @Params:			(void)
	// @Return:			A YawPitchRoll& (!Reference) indicating the current attitude of the quad copter
	// @Function:		Get the yaw, pitch and roll angles of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10)
	YawPitchRoll& GetYawPitchRoll()
	{
	#ifdef MINIQUAD_DMP_KEEP_DATA
		if(!_gravity_cal)
		{
	#endif // MINIQUAD_DMP_KEEP_DATA
			GetGravity(); // get the current gravity
	#ifdef MINIQUAD_DMP_KEEP_DATA
		}
		if(!_ypr_cal)
		{
	#endif // MINIQUAD_DMP_KEEP_DATA
			// Calculate the yaw, pitch and roll angles
			_ypr.setYaw((180/M_PI) * atan2(2*_quaternion.x*_quaternion.y - 2*_quaternion.w*_quaternion.z, 2*_quaternion.w*_quaternion.w + 2*_quaternion.x*_quaternion.x - 1));
			_ypr.setPitch((180/M_PI) * atan(_gravity.getX() / sqrt(_gravity.getY()*_gravity.getY() + _gravity.getZ()*_gravity.getZ())));
			_ypr.setRoll((180/M_PI) * atan(_gravity.getY() / sqrt(_gravity.getX()*_gravity.getX() + _gravity.getZ()*_gravity.getZ())));
	#ifdef MINIQUAD_DMP_KEEP_DATA
			_ypr_cal = true;
		}
	#endif // MINIQUAD_DMP_KEEP_DATA

		// Return the result
		return _ypr;
	}

	// @Params:			(void)
	// @Return:			A Acceleration& (!Reference) indicating the acceleration without gravity of the quad copter
	// @Function:		Get the acceleration without gravity of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10)
	Acceleration& GetLinearAcceleration()
	{
	#ifdef MINIQUAD_DMP_KEEP_DATA
		if(!_gravity_cal)
		{
	#endif // MINIQUAD_DMP_KEEP_DATA
			GetGravity(); // get the current gravity
	#ifdef MINIQUAD_DMP_KEEP_DATA
		}
		if(!_acceleration_cal)
		{
	#endif // MINIQUAD_DMP_KEEP_DATA
			// Calculate the linear acceleration
			// get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
			_acceleration.setX((float)_accel_Int16_raw.x/MPU6050_GRAVITY_UNIT - _gravity.getX());
			_acceleration.setY((float)_accel_Int16_raw.y/MPU6050_GRAVITY_UNIT - _gravity.getY());
			_acceleration.setZ((float)_accel_Int16_raw.z/MPU6050_GRAVITY_UNIT - _gravity.getZ());
	#ifdef MINIQUAD_DMP_KEEP_DATA
			_acceleration_cal = true;
		}
	#endif // MINIQUAD_DMP_KEEP_DATA

		// Return the result
		return _acceleration;
	}

	// @Params:			(void)
	// @Return:			A Acceleration& (!Reference) indicating the world acceleration with gravity of the quad copter
	// @Function:		Get the world acceleration with gravity of the quad copter (DMP)
	// @Contributor:	David Qiu (2013.7.10)
	Acceleration& GetWorldAcceleration()
	{
	#ifdef MINIQUAD_DMP_KEEP_DATA
		if(!_acceleration_cal)
		{
	#endif // MINIQUAD_DMP_KEEP_DATA
			GetLinearAcceleration(); // calculate the linear acceleration
	#ifdef MINIQUAD_DMP_KEEP_DATA
		}
		if(!_accelerationW_cal)
		{
	#endif // MINIQUAD_DMP_KEEP_DATA
			// Calculate the world acceleration in Int16 form
			VectorInt16 accel_world;
			accel_world.x = (int16_t)(_acceleration.getX() * MPU6050_GRAVITY_UNIT);
			accel_world.y = (int16_t)(_acceleration.getY() * MPU6050_GRAVITY_UNIT);
			accel_world.z = (int16_t)(_acceleration.getZ() * MPU6050_GRAVITY_UNIT);
			accel_world.rotate(&_quaternion);
			
			// Convert the Int16 form into float form
			_accelerationW.setX((float)accel_world.x/MPU6050_GRAVITY_UNIT);
			_accelerationW.setY((float)accel_world.y/MPU6050_GRAVITY_UNIT);
			_accelerationW.setZ((float)accel_world.z/MPU6050_GRAVITY_UNIT);
	#ifdef MINIQUAD_DMP_KEEP_DATA
			_accelerationW_cal = true;
		}
	#endif // MINIQUAD_DMP_KEEP_DATA

		// Return the result
		return _accelerationW;
	}


protected:
	MPU6050 _mpu;					// The MPU6050
	uint8_t _mpuInterruptStatus;	// Holds actual interrupt status byte from MPU
	uint16_t _mpuFIFOPacketSize;	// Expected DMP packet size (default is 42 bytes)
	uint16_t _mpuFIFOCount;			// Count of all bytes currently in FIFO
	uint8_t _mpuFIFOBuffer[64];		// FIFO storage buffer

	Quaternion _quaternionReader;	// The quaternion obtained as the DMP data source (DMP)
	Quaternion _quaternion;			// The last correct quaternion obtained from the quaternion reader (DMP)
	EulerAngle _eulerAngle;			// The Euler Angle obtained (DMP)
	Gravity _gravity;				// The gravity components of x-, y-, z-axes obtained (DMP)
	YawPitchRoll _ypr;				// The yaw, pitch, roll angles obtained (DMP)
	VectorInt16 _rot_Int16_raw;		// The raw Int16-form rotation data obtained as data source (raw)
	Rotation _rotation;				// The rotation data obtained (DMP)
	VectorInt16 _accel_Int16_raw;	// The raw Int16-form acceleration data obtained as data source (DMP)
	Acceleration _acceleration;		// The linear acceleration without gravity (DMP)
	Acceleration _accelerationW;	// The world linear acceleration with gravity (DMP)
#ifdef MINIQUAD_DMP_KEEP_DATA
	bool _acceleration_cal;			// Indicates if the _acceleration has been calculated
	bool _accelerationW_cal;		// Indicates if the _accelerationW has been calculated
	bool _eulerAngle_cal;			// Indicates if the _eulerAngle has been calculated
	bool _ypr_cal;					// Indicates if the _ypr has been calculated
	bool _gravity_cal;				// Indicates if the _gravity has been calculated
	bool _rotation_cal;				// Indicates if the _rotation has been calculated
#endif // MINIQUAD_DMP_KEEP_DATA


	// @Params:			(void)
	// @Return:			(_mpuFIFOBuffer, _quaternion, _accel_Int16_raw)
	// @Function:		Get the FIFO bytes from the MPU6050, put them to the buffer and convert them
	//					into Quaternion and raw acceleration in Int16 form.
	// @Contributor:	David Qiu (2013.7.1), David Qiu (2013.8.14)
	void _refreshDMPData()
	{
		do 
		{
			// Wait for MPU6050 to interrupt
			while(!MpuInterrupt && _mpuFIFOCount < _mpuFIFOPacketSize) {;}

			// Reset interrupt flag
			MpuInterrupt = false;

			// Get current FIFO count
			_mpuFIFOCount = _mpu.getFIFOCount();
			_mpuInterruptStatus = _mpu.getIntStatus();

			// Check for interrupt status and FIFO overflow
			if ((_mpuInterruptStatus & 0x10) || _mpuFIFOCount == 1024)
			{
				// Reset the FIFO
				_mpu.resetFIFO();
			}

			// Check the status again
			_mpuInterruptStatus = _mpu.getIntStatus();
			_mpuFIFOCount = _mpu.getFIFOCount();
		} while (!(_mpuInterruptStatus & 0x02));
		
		// Wait for correct available data length, should be a VERY short wait
		while (_mpuFIFOCount < _mpuFIFOPacketSize) _mpuFIFOCount = _mpu.getFIFOCount();

		// Read a packet from FIFO
		_mpu.getFIFOBytes(_mpuFIFOBuffer, _mpuFIFOPacketSize);

		// Track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		_mpuFIFOCount -= _mpuFIFOPacketSize;

		// Default FIFO buffer data structure
		/* ================================================================================================ *
		| Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
		|                                                                                                  |
		| [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
		|   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
		|                                                                                                  |
		| [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
		|  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
		* ================================================================================================ */

		// Get Quaternion as DMP data source
		_quaternionReader.w = (float)((_mpuFIFOBuffer[0] << 8) + _mpuFIFOBuffer[1]) / MPU6050_QUATERNION_UNIT;
		_quaternionReader.x = (float)((_mpuFIFOBuffer[4] << 8) + _mpuFIFOBuffer[5]) / MPU6050_QUATERNION_UNIT;
		_quaternionReader.y = (float)((_mpuFIFOBuffer[8] << 8) + _mpuFIFOBuffer[9]) / MPU6050_QUATERNION_UNIT;
		_quaternionReader.z = (float)((_mpuFIFOBuffer[12] << 8) + _mpuFIFOBuffer[13]) / MPU6050_QUATERNION_UNIT;
		if(_quaternionReader.getMagnitude()<0.9 || _quaternionReader.getMagnitude()>1.1) return;
		_quaternion = _quaternionReader;

		// Get Rotation as DMP data source
		//_rot_Int16_raw.x = (_mpuFIFOBuffer[16] << 8) + _mpuFIFOBuffer[17];
		//_rot_Int16_raw.y = (_mpuFIFOBuffer[20] << 8) + _mpuFIFOBuffer[21];
		//_rot_Int16_raw.z = (_mpuFIFOBuffer[24] << 8) + _mpuFIFOBuffer[25];

		// Get Acceleration as DMP data source
		_accel_Int16_raw.x = (_mpuFIFOBuffer[28] << 8) + _mpuFIFOBuffer[29];
		_accel_Int16_raw.y = (_mpuFIFOBuffer[32] << 8) + _mpuFIFOBuffer[33];
		_accel_Int16_raw.z = (_mpuFIFOBuffer[36] << 8) + _mpuFIFOBuffer[37];

		// Get Rotation from raw MPU6050
		_mpu.getRotation(&(_rot_Int16_raw.x), &(_rot_Int16_raw.y), &(_rot_Int16_raw.z));

	#ifdef MINIQUAD_DMP_KEEP_DATA
		// Clear the calculation flags
		_acceleration_cal = false;
		_accelerationW_cal = false;
		_eulerAngle_cal = false;
		_ypr_cal = false;
		_gravity_cal = false;
		_rotation_cal = false;
	#endif // MINIQUAD_DMP_KEEP_DATA
	}
};

#endif // !_MINIQUADZERO_H_
