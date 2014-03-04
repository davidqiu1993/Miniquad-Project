// Copyright (C) 2013 Robot Club, Sun Yat-Sen University. All rights reserved. 
// Author: David Qiu (ÇñµÏ´Ï) <david@davidqiu.com>
// Update:
//		- 2013.6.30 : Created by David Qiu <david@davidqiu.com>
//		- 2013.7.2 : Classes YawPitchRoll and Gravity added by Jack Xu <503689341@qq.com>
//		- 2013.7.2 : 3D calculations for _FloatVector_3D added by David Qiu <david@davidqiu.com>
//		- 2013.7.4 : Some logical errors corrected and comments upgraded of the classes 
//					 _FloatVector_3D, EulerAngle, YawPitchRoll and Gravity; the class
//					 Acceleration added by David Qiu <david@davidqiu.com>
//		- 2013.7.9 : Class Rotation added by David Qiu <david@davidqiu.com>
//
// This is a standard library for the quadaxis copter "Miniquad Zero" (C). It includes
// the basic data structures needed for the further development.
//
// It is free to use this library within the Robot Club. The copyright belongs to 
// the Robot Club, and the right authorship of each part belongs to its contributers.
// === [ Robot Club ] ===

#ifndef _MINIQUAD_3DMATH_H_
#define _MINIQUAD_3DMATH_H_

#include "helper_3dmath.h"


// Class: 3D vector for floats. 
class _FloatVector_3D
{
protected:
	float _array[3]; // The float array

public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Default constructor.
	// @Contributor:	David Qiu (2013.7.2)
	_FloatVector_3D()
	{
		_array[0] = 0;
		_array[0] = 0;
		_array[0] = 0;
	}

	// @Params:			x: The first value
	//					y: The second value
	//					z: The third value
	// @Return:			(void)
	// @Function:		The constructor with initialization.
	// @Contributor:	David Qiu (2013.7.2)
	_FloatVector_3D(float x, float y, float z)
	{
		_array[0] = x;
		_array[1] = y;
		_array[2] = z;
	}

	// @Params:			(void)
	// @Return:			A float* indicating the starting address of the float array.
	// @Function:		Get the array of the float vector.
	// @Contributor:	David Qiu (2013.7.2)
	float* GetArray()
	{
		return _array;
	}

	// @Params:			(void)
	// @Return:			A float indicating the magnitude of the vector.
	// @Function:		Get the magnitude of the vector.
	// @Contributor:	David Qiu (2013.7.2)
	float GetMagnitude()
	{
		return sqrt(_array[0]*_array[0] + _array[1]*_array[1] + _array[2]*_array[2]);
	}

	// @Params:			(void)
	// @Return:			(Effect on itself)
	// @Function:		Normalize the vector.
	// @Contributor:	David Qiu (2013.7.2)
	void Normalize()
	{
		float moduli = GetMagnitude();
		_array[0] = _array[0] / moduli;
		_array[1] = _array[1] / moduli;
		_array[2] = _array[2] / moduli;
	}

	// @Params:			(void)
	// @Return:			A _FloatVector_3D indicating the returned normalized vector.
	// @Function:		Get a normalized vector in respect to this.
	// @Contributor:	David Qiu (2013.7.2)
	_FloatVector_3D GetNormalized()
	{
		_FloatVector_3D vec(*this);
		vec.Normalize();
		return vec;
	}

	// @Params:			q: The referred rotation Quaternion
	// @Return:			(Effect on itself)
	// @Function:		Rotate the vector.
	// @Contributor:	David Qiu (2013.7.2)
	void Rotate(Quaternion *q)
	{
		Quaternion p(0, _array[0], _array[1], _array[2]);

		// quaternion multiplication: q * p, stored back in p
		p = q -> getProduct(p);

		// quaternion multiplication: p * conj(q), stored back in p
		p = p.getProduct(q -> getConjugate());

		// p quaternion is now [0, x', y', z']
		_array[0] = p.x;
		_array[1] = p.y;
		_array[2] = p.z;
	}

	// @Params:			q: The referred rotation Quaternion
	// @Return:			A _FloatVector_3D indicating the returned rotated vector.
	// @Function:		Get a rotated vector in respect to this.
	// @Contributor:	David Qiu (2013.7.4)
	_FloatVector_3D GetRoteted(Quaternion *q)
	{
		_FloatVector_3D vec(*this);
		vec.Rotate(q);
		return vec;
	}
};


// Class: Rotation
class Rotation : public _FloatVector_3D
{
public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Default constructor.
	// @Contributor:	David Qiu (2013.7.9)
	Rotation() : _FloatVector_3D()
	{
		;
	}

	// @Params:			x: The initial value of rotation of x-axis direction (degree)
	//					y: The initial value of rotation of y-axis direction (degree)
	//					z: The initial value of rotation of z-axis direction (degree)
	// @Return:			(void)
	// @Function:		The constructor with initialization of rotations of x-, y-, z-axes directions.
	// @Contributor:	David Qiu (2013.7.9)
	Rotation(float x, float y, float z) : _FloatVector_3D(x, y, z)
	{
		;
	}

	// @Attribute:		X
	// @Params:			(void)
	// @Return:			A float indicating the current value of rotation of x-axis direction (degree)
	// @Function:		Get the rotation of x-axis direction
	// @Contributor:	David Qiu (2013.7.9)
	float getX()
	{
		return _array[0];
	}
	// @Attribute:		X
	// @Params:			value: The new value of rotation of x-axis direction (degree)
	// @Return:			(void)
	// @Function:		Set the rotation of x-axis direction (degree)
	// @Contributor:	David Qiu (2013.7.9)
	void setX(float value)
	{
		_array[0] = value;
	}

	// @Attribute:		Y
	// @Params:			(void)
	// @Return:			A float indicating the current value of rotation of y-axis direction (degree)
	// @Function:		Get the rotation of y-axis direction
	// @Contributor:	David Qiu (2013.7.10)
	float getY()
	{
		return _array[1];
	}
	// @Attribute:		Y
	// @Params:			value: The new value of rotation of y-axis direction (degree)
	// @Return:			(void)
	// @Function:		Set the rotation of y-axis direction (degree)
	// @Contributor:	David Qiu (2013.7.10)
	void setY(float value)
	{
		_array[1] = value;
	}

	// @Attribute:		Z
	// @Params:			(void)
	// @Return:			A float indicating the current value of rotation of z-axis direction (degree)
	// @Function:		Get the rotation of z-axis direction
	// @Contributor:	David Qiu (2013.7.10)
	float getZ()
	{
		return _array[2];
	}
	// @Attribute:		Z
	// @Params:			value: The new value of rotation of z-axis direction (degree)
	// @Return:			(void)
	// @Function:		Set the rotation of z-axis direction (degree)
	// @Contributor:	David Qiu (2013.7.10)
	void setZ(float value)
	{
		_array[2] = value;
	}
};


// Class: Euler angle
class EulerAngle : public _FloatVector_3D
{
public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Default constructor.
	// @Contributor:	David Qiu (2013.7.2)
	EulerAngle() : _FloatVector_3D()
	{
		;
	}

	// @Params:			psi: The initial value of Psi (degree)
	//					theta: The initial value of Theta (degree)
	//					phi: The initial value of Phi (degree)
	// @Return:			(void)
	// @Function:		The constructor with initialization of Psi, Theta and Phi.
	// @Contributor:	David Qiu (2013.7.2)
	EulerAngle(float psi, float theta, float phi) : _FloatVector_3D(psi, theta, phi)
	{
		;
	}

	// @Attribute:		Psi
	// @Params:			(void)
	// @Return:			A float indicating the current value of Psi (degree)
	// @Function:		Get Psi
	// @Contributor:	David Qiu (2013.7.4)
	float getPsi()
	{
		return _array[0];
	}
	// @Attribute:		Psi
	// @Params:			value: The new value of Psi (degree)
	// @Return:			(void)
	// @Function:		Set Psi
	// @Contributor:	David Qiu (2013.7.4)
	void setPsi(float value)
	{
		_array[0] = value;
	}

	// @Attribute:		Theta
	// @Params:			(void)
	// @Return:			A float indicating the current value of Theta (degree)
	// @Function:		Get Theta
	// @Contributor:	David Qiu (2013.7.4)
	float getTheta()
	{
		return _array[1];
	}
	// @Attribute:		Theta
	// @Params:			value: The new value of Theta (degree) 
	// @Return:			(void)
	// @Function:		Set Theta
	// @Contributor:	David Qiu (2013.7.4)
	void setTheta(float value)
	{
		_array[1] = value;
	}

	// @Attribute:		Phi
	// @Params:			(void)
	// @Return:			A float indicating the current value of Phi (degree)
	// @Function:		Get Phi
	// @Contributor:	David Qiu (2013.7.4)
	float getPhi()
	{
		return _array[2];
	}
	// @Attribute:		Phi
	// @Params:			value: The new value of Phi (degree)
	// @Return:			(void)
	// @Function:		Set Phi
	// @Contributor:	David Qiu (2013.7.4)
	void setPhi(float value)
	{
		_array[2] = value;
	}
};


// Class: Yaw, Pitch and Roll angles
class YawPitchRoll : public _FloatVector_3D
{
public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Default constructor.
	// @Contributor:	Jack Xu (2013.7.2)
	YawPitchRoll() : _FloatVector_3D()
	{
		;
	}

	// @Params:			yaw: The initial value of the yaw angle (degree)
	//					pitch: The initial value of the pitch angle (degree)
	//					roll: The initial value of the roll angle (degree)
	// @Return:			(void)
	// @Function:		The constructor with initialization of the yaw, pitch and roll angles.
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	YawPitchRoll(float yaw, float pitch, float roll) : _FloatVector_3D(yaw, pitch, roll)
	{
		;
	}

	// @Attribute:		Yaw
	// @Params:			(void)
	// @Return:			A float indicating the new value of the yaw angle (degree)
	// @Function:		Get the yaw angle
	// @Contributor:	Jack Xu (2013.7.2)
	float getYaw()
	{
		return _array[0];
	}
	// @Attribute:		Yaw
	// @Params:			value: The new value of the yaw angle (degree)
	// @Return:			(void)
	// @Function:		Set the yaw angle
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	void setYaw(float value)
	{
		_array[0] = value;
	}

	// @Attribute:		Pitch
	// @Params:			(void)
	// @Return:			A float indicating the current value of the pitch angle (degree)
	// @Function:		Get the pitch angle
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	float getPitch()
	{
		return _array[1];
	}
	// @Attribute:		Pitch
	// @Params:			value: The new value of the pitch angle (degree)
	// @Return:			(void)
	// @Function:		Set the pitch angle
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	void setPitch(float value)
	{
		_array[1] = value;
	}

	// @Attribute:		Roll
	// @Params:			(void)
	// @Return:			A float indicating the current value of the roll angle (degree)
	// @Function:		Get the roll angle
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	float getRoll()
	{
		return _array[2];
	}
	// @Attribute:		Roll
	// @Params:			value: The new value of the roll angle (degree)
	// @Return:			(void)
	// @Function:		Set the roll angle
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	void setRoll(float value)
	{
		_array[2] = value;
	}
};


// Class: Gravity components
class Gravity : public _FloatVector_3D
{
public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Default constructor.
	// @Contributor:	Jack Xu (2013.7.2)
	Gravity() : _FloatVector_3D()
	{
		;
	}

	// @Params:			x: The initial value of the gravity component on x-axis (g)
	//					y: The initial value of the gravity component on y-axis (g)
	//					z: the initial value of the gravity component on z-axis (g)
	// @Return:			(void)
	// @Function:		The constructor with initialization of the gravity components.
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	Gravity(float x, float y, float z) : _FloatVector_3D(x, y, z)
	{
		;
	}

	// @Attribute:		X
	// @Params:			(void)
	// @Return:			A float indicating the current value of the gravity component on x-axis (g)
	// @Function:		Get the gravity component on x-axis
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	float getX()
	{
		return _array[0];
	}
	// @Attribute:		X
	// @Params:			A float indicating the new value of the gravity component on x-axis (g)
	// @Return:			(void)
	// @Function:		Set the gravity component on x-axis
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	void setX(float value)
	{
		_array[0] = value;
	}

	// @Attribute:		Y
	// @Params:			(void)
	// @Return:			A float indicating the current value of the gravity component on y-axis (g)
	// @Function:		Get the gravity component on y-axis (g)
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	float getY()
	{
		return _array[1];
	}
	// @Attribute:		Y
	// @Params:			value: The new value of the gravity component on y-axis (g)
	// @Return:			(void)
	// @Function:		Get the gravity component on y-axis (g)
	// @Contributor:	Jack Xu (2013.7.2), David Qiu (2013.7.4)
	void setY(float value)
	{
		_array[1] = value;
	}

	// @Attribute:		Z
	// @Params:			(void)
	// @Return:			A float indicating the current value of the gravity component on z-axis (g)
	// @Function:		Get the gravity component on z-axis
	// @Contributor:	David Qiu (2013.7.4)
	float getZ()
	{
		return _array[2];
	}
	// @Attribute:		Z
	// @Params:			value: The new value of the gravity component on z-axis (g)
	// @Return:			(void)
	// @Function:		Set the gravity component on z-axis (g)
	// @Contributor:	David Qiu (2013.7.4)
	void setZ(float value)
	{
		_array[2] = value;
	}
};


// Class: Acceleration components
class Acceleration : public _FloatVector_3D
{
public:

	// @Params:			(void)
	// @Return:			(void)
	// @Function:		Default constructor.
	// @Contributor:	David Qiu (2013.7.4)
	Acceleration() : _FloatVector_3D()
	{
		;
	}

	// @Params:			x: The initial value of acceleration component on x-axis (g)
	//					y: The initial value of acceleration component on y-axis (g)
	//					z: The initial value of acceleration component on z-axis (g)
	// @Return:			(void)
	// @Function:		The constructor with initialization of acceleration components.
	// @Contributor:	David Qiu (2013.7.4)
	Acceleration(float x, float y, float z) : _FloatVector_3D(x, y, z)
	{
		;
	}

	// @Attribute:		X
	// @Params:			(void)
	// @Return:			A float indicating the current value of the acceleration component on x-axis (g)
	// @Function:		Get the acceleration component on x-axis
	// @Contributor:	David Qiu (2013.7.4)
	float getX()
	{
		return _array[0];
	}
	// @Attribute:		X
	// @Params:			value: The new value of the acceleration component on x-axis (g)
	// @Return:			(void)
	// @Function:		Set the acceleration component on x-axis
	// @Contributor:	David Qiu (2013.7.4)
	void setX(float value)
	{
		_array[0] = value;
	}

	// @Attribute:		Y
	// @Params:			(void)
	// @Return:			A float indicating the current value of the acceleration component on y-axis (g)
	// @Function:		Get the acceleration component on y-axis
	// @Contributor:	David Qiu (2013.7.4)
	float getY()
	{
		return _array[1];
	}
	// @Attribute:		Y
	// @Params:			value: The new value of the acceleration component on y-axis (g)
	// @Return:			(void)
	// @Function:		Get the acceleration component on y-axis
	// @Contributor:	David Qiu (2013.7.4)
	void setY(float value)
	{
		_array[1] = value;
	}

	// @Attribute:		Z
	// @Params:			(void)
	// @Return:			A float indicating the current value of the acceleration component on z-axis (g)
	// @Function:		Get the acceleration component on z-axis
	// @Contributor:	David Qiu (2013.7.4)
	float getZ()
	{
		return _array[2];
	}
	// @Attribute:		Z
	// @Params:			value: The new value of the acceleration component on z-axis (g)
	// @Return:			(void)
	// @Function:		Set the acceleration component on z-axis
	// @Contributor:	David Qiu (2013.7.4)
	void setZ(float value)
	{
		_array[2] = value;
	}
};

#endif // !_MINIQUAD_3DMATH_H_
