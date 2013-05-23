/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/05/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "math/SO3.h"

namespace dart {
namespace math {

#define LIEGROUP_EPS 10e-9

//==============================================================================
so3::so3()
	: mw(Eigen::Vector3d::Zero())
{
}

so3::so3(const Eigen::Vector3d& _w)
	: mw(_w)
{

}

so3::so3(double _w0, double _w1, double _w2)
{
	mw << _w0, _w1, _w2;
}

so3::~so3()
{
}

const so3& so3::operator=(const so3& _w)
{
    if (this != & _w)
    {
        mw = _w.mw;
    }

    return *this;
}

double& so3::operator()(int _i)
{
    assert(0 <= _i && _i <= 2);

    return mw[_i];
}

const double& so3::operator()(int _i) const
{
    assert(0 <= _i && _i <= 2);

    return mw[_i];
}

so3 so3::operator+(void) const
{
    return *this;
}

so3 so3::operator-(void) const
{
    return so3(-mw);
}

const so3& so3::operator+=(const so3& _w)
{
    mw += _w.mw;
    return *this;
}

const so3& so3::operator-=(const so3& _w)
{
    mw -= _w.mw;
    return *this;
}

const so3& so3::operator*=(double _c)
{
    mw *= _c;
    return *this;
}

so3 so3::operator+(const so3& _w) const
{
    return so3(mw + _w.mw);
}

so3 so3::operator-(const so3& _w) const
{
    return so3(mw - _w.mw);
}

so3 so3::operator*(double _c) const
{
    return so3(mw * _c);
}

void so3::setValues(double _w0, double _w1, double _w2)
{
    mw(0) = _w0;
    mw(1) = _w1;
    mw(2) = _w2;
}

void so3::setFromSkewSymmetrixMatrix(const Eigen::Matrix3d& _ssm)
{
    assert(_ssm(0,0) == 0);
    assert(_ssm(1,1) == 0);
    assert(_ssm(2,2) == 0);

    assert(_ssm(0,1) == _ssm(1,0));
    assert(_ssm(0,2) == _ssm(2,0));
    assert(_ssm(1,2) == _ssm(2,1));

    mw(0) = _ssm(2,1);
    mw(1) = _ssm(0,2);
    mw(2) = _ssm(1,0);
}

Eigen::Matrix3d so3::getSkewSymmetrixMatrix() const
{
    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

    result(0, 1) = -mw(2);
    result(1, 0) =  mw(2);
    result(0, 2) =  mw(1);
    result(2, 0) = -mw(1);
    result(1, 2) = -mw(0);
    result(2, 1) =  mw(0);

    return result;
}

void so3::setVector(const Eigen::Vector3d& _w)
{
    mw = _w;
}


so3 operator*(double _c, const so3& _w)
{
    return so3(_c * _w.mw);
}

//==============================================================================
SO3::SO3()
    : mRotation(Eigen::Matrix3d::Identity())
{
}

SO3::SO3(const Eigen::Matrix3d& _rotation)
	: mRotation(_rotation)
{
}

//SO3::SO3(const Eigen::Vector3d& _axisX,
//		 const Eigen::Vector3d& _axisY,
//		 const Eigen::Vector3d& _axisZ)
//{
//	mRotation.col(0) = _axisX;
//	mRotation.col(1) = _axisY;
//	mRotation.col(2) = _axisZ;
//}

SO3::SO3(double _R00, double _R01, double _R02,
		 double _R10, double _R11, double _R12,
		 double _R20, double _R21, double _R22)
{
	mRotation << _R00, _R01, _R02,
			_R10, _R11, _R12,
			_R20, _R21, _R22;
}

SO3::SO3(const so3& _w)
{
	setExp(_w);
}

SO3::~SO3()
{
}

double& SO3::operator()(int _i, int _j)
{
	assert(0 <= _i && _i <= 2);
	assert(0 <= _j && _j <= 2);

	return mRotation(_i, _j);
}

const double& SO3::operator()(int _i, int _j) const
{
	assert(0 <= _i && _i <= 2);
	assert(0 <= _j && _j <= 2);

	return mRotation(_i, _j);
}

/// @brief Substitution operator.
const SO3& SO3::operator=(const SO3& _R)
{
	if (this != &_R)
	{
		mRotation = _R.mRotation;
	}

	return *this;
}

const SO3& SO3::operator*=(const SO3 & _R)
{
	mRotation *= _R.mRotation;

	return *this;
}

//const SO3& SO3::operator/=(const SO3& _R)
//{
//	mR *= _R.mR.transpose();
//}

//const SO3& SO3::operator%=(const SO3& _R)
//{

//}

SO3 SO3::operator*(const SO3& _R) const
{
	return SO3(mRotation * _R.mRotation);
}

//SO3 SO3::operator/(const SO3& _R) const
//{

//}

//SO3 SO3::operator%(const SO3& _R) const
//{

//}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d& _q) const
{
	return mRotation * _q;
}

so3 SO3::operator*(const so3& _w) const
{
	return so3(mRotation * _w.mw);
}

bool SO3::operator==(const SO3& _R) const
{
	if (mRotation == _R.mRotation)
		return true;
	else
		return false;
}

void SO3::setValues(double _R00, double _R01, double _R02, double _R10, double _R11, double _R12, double _R20, double _R21, double _R22)
{
	mRotation << _R00, _R01, _R02,
			_R10, _R11, _R12,
			_R20, _R21, _R22;
}

//Eigen::Vector3d SO3::operator%(const Eigen::Vector3d& _q) const
//{

//}

void SO3::setExp(const so3& _S)
{
	const Eigen::Vector3d& s = _S.getVector();

	double s2[] = { s[0] * s[0], s[1] * s[1], s[2] * s[2] };
	double theta = sqrt(s2[0] + s2[1] + s2[2]);
	double st_t = 0.0;
	double ct_t = 0.0;

	if ( theta < LIEGROUP_EPS )
	{
		st_t = 1.0 - theta * theta / (double)6.0;
		ct_t = 0.5 - theta * theta / (double)24.0;
	} else
	{
		st_t = sin(theta) / theta;
		ct_t = (1.0 - cos(theta)) / theta / theta;
	}

	mRotation(0,0) = 1.0 - ct_t * (s2[1] + s2[2]);
	mRotation(1,0) = ct_t * s[0] * s[1] + st_t * s[2];
	mRotation(2,0) = ct_t * s[0] * s[2] - st_t * s[1];
	mRotation(0,1) = ct_t * s[0] * s[1] - st_t * s[2];
	mRotation(1,1) = 1.0 - ct_t * (s2[0] + s2[2]);
	mRotation(2,1) = ct_t * s[1] * s[2] + st_t * s[0];
	mRotation(0,2) = ct_t * s[0] * s[2] + st_t * s[1];
	mRotation(1,2) = ct_t * s[1] * s[2] - st_t * s[0];
	mRotation(2,2) = 1.0 - ct_t * (s2[0] + s2[1]);
}

void SO3::setExp(const so3& _S, double theta)
{
	const Eigen::Vector3d& s = _S.getVector();

	double s2[] = { s[0] * s[0], s[1] * s[1], s[2] * s[2] };

	if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > LIEGROUP_EPS )
	{
		setExp(theta * _S);
	}

	double st = sin(theta),
			vt = 1.0 - cos(theta),
			sts[] = { st * s[0], st * s[1], st * s[2] };

	mRotation(0,0) = 1.0 + vt * (s2[0] - 1.0);
	mRotation(1,0) = vt * s[0] * s[1] + sts[2];
	mRotation(2,0) = vt * s[0] * s[2] - sts[1];
	mRotation(0,1) = vt * s[0] * s[1] - sts[2];
	mRotation(1,1) = 1.0 + vt * (s2[1] - 1.0);
	mRotation(2,1) = vt * s[1] * s[2] + sts[0];
	mRotation(0,2) = vt * s[0] * s[2] + sts[1];
	mRotation(1,2) = vt * s[1] * s[2] - sts[0];
	mRotation(2,2) = 1.0 + vt * (s2[2] - 1.0);
}

} // namespace math
} // namespace dart
