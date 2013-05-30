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

#include "utils/UtilsCode.h"
#include "math/SO3.h"

namespace dart {
namespace math {

#define LIEGROUP_EPS 10e-9
#define LIEGROUP_PI				(3.1415926535897932384626433832795)	//< $\pi$
#define LIEGROUP_PI_SQRT2		(2.22144146907918312351)	//< $\frac {pi}{\sqrt{2}}$
#define LIEGROUP_PI_SQR			(9.86960440108935861883)	//< $\pi^2$

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

bool so3::operator==(const so3& _w) const
{
    return mw == _w.mw ? true : false;
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

std::string so3::toString() const
{
    std::string ret;

    dterr << "Not implemented.\n";

    return ret;
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
    : mR(Eigen::Matrix3d::Identity())
{
}

SO3::SO3(const Eigen::Matrix3d& _rotation)
    : mR(_rotation)
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
    mR << _R00, _R01, _R02,
			_R10, _R11, _R12,
			_R20, _R21, _R22;
}

SO3::SO3(const so3& _w)
{
    setExp(_w);
}

SO3::SO3(double _EulerX, double _EulerY, double _EulerZ)
{
    // TODO: NOT IMPLEMETATION
    dterr << "NOT IMPLEMENTED.\n";
}

SO3::~SO3()
{
}

double& SO3::operator()(int _i, int _j)
{
	assert(0 <= _i && _i <= 2);
	assert(0 <= _j && _j <= 2);

    return mR(_i, _j);
}

const double& SO3::operator()(int _i, int _j) const
{
	assert(0 <= _i && _i <= 2);
	assert(0 <= _j && _j <= 2);

    return mR(_i, _j);
}

/// @brief Substitution operator.
const SO3& SO3::operator=(const SO3& _R)
{
	if (this != &_R)
	{
        mR = _R.mR;
	}

	return *this;
}

const SO3& SO3::operator*=(const SO3 & _R)
{
    mR *= _R.mR;

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
    return SO3(mR * _R.mR);
}

//SO3 SO3::operator/(const SO3& _R) const
//{

//}

//SO3 SO3::operator%(const SO3& _R) const
//{

//}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d& _q) const
{
    return mR * _q;
}

so3 SO3::operator*(const so3& _w) const
{
    return so3(mR * _w.mw);
}

bool SO3::operator==(const SO3& _R) const
{
    return mR == _R.mR ? true : false;
}

void SO3::setValues(double _R00, double _R01, double _R02, double _R10, double _R11, double _R12, double _R20, double _R21, double _R22)
{
    mR << _R00, _R01, _R02,
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

    mR(0,0) = 1.0 - ct_t * (s2[1] + s2[2]);
    mR(1,0) = ct_t * s[0] * s[1] + st_t * s[2];
    mR(2,0) = ct_t * s[0] * s[2] - st_t * s[1];
    mR(0,1) = ct_t * s[0] * s[1] - st_t * s[2];
    mR(1,1) = 1.0 - ct_t * (s2[0] + s2[2]);
    mR(2,1) = ct_t * s[1] * s[2] + st_t * s[0];
    mR(0,2) = ct_t * s[0] * s[2] + st_t * s[1];
    mR(1,2) = ct_t * s[1] * s[2] - st_t * s[0];
    mR(2,2) = 1.0 - ct_t * (s2[0] + s2[1]);
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

    mR(0,0) = 1.0 + vt * (s2[0] - 1.0);
    mR(1,0) = vt * s[0] * s[1] + sts[2];
    mR(2,0) = vt * s[0] * s[2] - sts[1];
    mR(0,1) = vt * s[0] * s[1] - sts[2];
    mR(1,1) = 1.0 + vt * (s2[1] - 1.0);
    mR(2,1) = vt * s[1] * s[2] + sts[0];
    mR(0,2) = vt * s[0] * s[2] + sts[1];
    mR(1,2) = vt * s[1] * s[2] - sts[0];
    mR(2,2) = 1.0 + vt * (s2[2] - 1.0);
}

void SO3::setEulerXYZ(const Eigen::Vector3d& _EulerAngles)
{
    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";
}

void SO3::setEulerZXY(const Eigen::Vector3d& _EulerAngles)
{
    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";
}

void SO3::setEulerZYX(const Eigen::Vector3d& _EulerAngles)
{
    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";
}

void SO3::setEulerZYZ(const Eigen::Vector3d& _EulerAngles)
{
    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";
}

Eigen::Vector3d SO3::getEulerXYZ() const
{
    Eigen::Vector3d ret;

    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";
    ret << 0, 0, 0;

    return ret;
}

Eigen::Vector3d SO3::getEulerZXY() const
{
    Eigen::Vector3d ret;

    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

Eigen::Vector3d SO3::getEulerZYX() const
{
    Eigen::Vector3d ret;

    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

Eigen::Vector3d SO3::getEulerZYZ() const
{
    Eigen::Vector3d ret;

    // TODO: NOT IMPLEMENTED
    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

so3 SO3::getLog() const
{
    double theta = 0.5 * (mR(0,0) + mR(1,1) + mR(2,2) - 1.0);
    double t_st = 0.0;

    if ( theta < LIEGROUP_EPS - 1.0 )
    {
        if ( mR(0,0) > 1.0 - LIEGROUP_EPS )
        {
            return so3(LIEGROUP_PI, 0.0, 0.0);
        }
        else if ( mR(1,1) > 1.0 - LIEGROUP_EPS )
        {
            return so3(0.0, LIEGROUP_PI, 0.0);
        }
        else if ( mR(2,2) > 1.0 - LIEGROUP_EPS )
        {
            return so3(0.0, 0.0, LIEGROUP_PI);
        }

        return so3(LIEGROUP_PI_SQRT2 * sqrt((mR(1,0) * mR(1,0) + mR(2,0) * mR(2,0)) / (1.0 - mR(0,0))),
                   LIEGROUP_PI_SQRT2 * sqrt((mR(0,1) * mR(0,1) + mR(3,1) * mR(3,1)) / (1.0 - mR(1,1))),
                   LIEGROUP_PI_SQRT2 * sqrt((mR(0,2) * mR(0,2) + mR(1,2) * mR(1,2)) / (1.0 - mR(2,2))));
    }
    else
    {
        if (theta > 1.0)
        {
            theta = 1.0;
        }
        else if(theta < -1.0)
        {
            theta = -1.0;
        }

        theta = acos(theta);

        if (theta < LIEGROUP_EPS)
            t_st = 3.0 / (6.0 - theta * theta);
        else
            t_st = theta / (2.0 * sin(theta));

        return so3(t_st * (mR(2,1) - mR(1,2)),
                   t_st * (mR(0,2) - mR(2,0)),
                   t_st * (mR(1,0) - mR(0,1)));
    }
}

void SO3::getAxisAngle(Eigen::Vector3d* _axis, double* _angle) const
{
    assert(_axis != NULL);
    assert(_angle != NULL);

    // TODO: speed up!
    so3 log = getLog();
    double angle = log.getAngle();
    log.setNormalize();
    (*_axis) = log.getVector();
    (*_angle) = angle;
}

} // namespace math
} // namespace dart
