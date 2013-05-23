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

#include "math/SE3.h"

namespace dart {
namespace math {

#define LIEGROUP_EPS 10e-9

se3 Ad(const SE3& _T12, const se3& _vel2)
{
    // _T12 = | R p |, vel1 = | w1 |, _vel2 = | w2 |
    //        | 0 1 |         | v1 |          | v2 |
    //
    // vel1 = Ad(_T12, _vel2)
    //      = Ad(_T12) * _vel2
    //      = | R    0 | * | w2 |
    //        | [p]R R | * | v2 |
    //      = | Rw2         |
    //        | [p]Rw2 + Rv2 |
    // w1 = R * w2
    // v1 = [p]R * w2 + R * v2

    se3 vel1;

    vel1.setAngular(_T12.getRotation() * _vel2.getAngular());
    vel1.setLinear(_T12.getPosition().cross(vel1.getAngular().getVector())
                   + _T12.getRotation() * _vel2.getLinear());

    return vel1;
}

//se3 InvAd(const SE3& _T21, const se3& _vel2)
//{
//    se3 vel1;

//    // _T21 = | R p |, vel1 = | w1 |, _vel2 = | w2 |
//    //        | 0 1 |         | v1 |          | v2 |
//    //
//    // vel1 = Ad(_T12, _vel2)
//    //      = Ad(_T12) * _vel2
//    //      = | R    0 | * | w2 |
//    //        | [p]R R | * | v2 |
//    //      = | Rw2         |
//    //        | [p]Rw2 + Rv2 |
//    // w1 = R * w2
//    // v1 = [p]R * w2 + R * v2

//    return vel1;
//}

//==============================================================================
se3::se3()
    : mAngular(Eigen::Vector3d::Zero()),
      mLinear(Eigen::Vector3d::Zero())
{
}

se3::se3(const se3& _V)
    : mAngular(_V.mAngular),
      mLinear(_V.mLinear)
{
}

se3::se3(const Vector6d& _V)
    : mAngular(_V.segment(0,3)),
      mLinear(_V.segment(3,3))
{
}

se3::se3(double _w0, double _w1, double _w2,
         double _v0, double _v1, double _v2)
{
    mAngular.setValues(_w0, _w1, _w2);
    mLinear << _v0, _v1, _v2;
}

se3::se3(const so3& _w,
         const Eigen::Vector3d& _v)
    : mAngular(_w),
      mLinear(_v)
{
}

se3::se3(const so3& _w)
    : mAngular(_w),
      mLinear(Eigen::Vector3d::Zero())
{
}

se3::se3(const Eigen::Vector3d& _v)
    : mAngular(Eigen::Vector3d::Zero()),
      mLinear(_v)
{
}

se3::~se3()
{
}

const se3& se3::operator=(const se3& _V)
{
    if(this != &_V)
    {
        mLinear = _V.mLinear;
        mAngular = _V.mAngular;
    }
    
    return *this;
}

//double& se3::operator()(int _i)
//{
//    assert(0 <= _i && _i <= 5);

//    return mw[_i];
//}

//const double& se3::operator()(int _i) const
//{

//}

se3 se3::operator+(void) const
{
    return *this;
}

se3 se3::operator-(void) const
{
    return se3(-mAngular, -mLinear);
}

const se3& se3::operator+=(const se3& _V)
{
    mAngular += _V.mAngular;
    mLinear += _V.mLinear;

    return *this;
}

const se3& se3::operator-=(const se3& _V)
{
    mAngular -= _V.mAngular;
    mLinear -= _V.mLinear;

    return *this;
}

const se3& se3::operator*=(double _c)
{
    mAngular *= _c;
    mLinear *= _c;

    return *this;
}

se3 se3::operator+(const se3& _V) const
{
    return se3(mAngular + _V.mAngular, mLinear + _V.mLinear);
}

se3 se3::operator-(const se3& _V) const
{
    return se3(mAngular - _V.mAngular, mLinear - _V.mLinear);
}

se3 se3::operator*(double _c) const
{
    return se3(mAngular * _c, mLinear * _c);
}

void se3::setVector(const Vector6d& _V)
{
    mAngular.setVector(_V.segment(0,3));
    mLinear = _V.segment(3,3);
}

Vector6d se3::getVector() const
{
    Vector6d ret;

    ret.segment(0,3) = mAngular.getVector();
    ret.segment(3,3) = mLinear;

    return ret;
}

TSE3 se3::operator*(const SE3& _T) const
{
    // Let,
    //     this = | [w] v |, _T = | R p |
    //            |  0  0 |       | 0 1 |
    // Then,
    //     return  = | [w] v | * | R p |
    //               |  0  0 |   | 0 1 |
    //
    //             = | [w] * R   [w] * p + v |
    //               |       0             0 |

    //const Eigen::Matrix3d& wSkewSymmetric = mAngular.getSkewSymmetrixMatrix();

//    return TSE3(wSkewSymmetric * _T.getRotation().getMatrix(),
//                wSkewSymmetric * _T.getPosition() + mLinear);
    return TSE3(*this, _T);
}

//TSE3 se3::operator*(const TSE3& _dT) const
//{
//    // Let,
//    //     this = | [w] v |, _dT = | R p |
//    //            |  0  0 |        | 0 0 |
//    // Then,
//    //     return  = | [w] v | * | R p |
//    //               |  0  0 |   | 0 0 |
//    //
//    //             = | [w] * R   [w] * p |
//    //               |       0         0 |

//    const Eigen::Matrix3d& wSkewSymmetric = mAngular.getSkewSymmetrixMatrix();

//    return TSE3(wSkewSymmetric * _dT.getRotation(),
//                wSkewSymmetric * _dT.getPosition());
//}

se3 operator * (double _c, const se3& _V)
{
    return se3(_c * _V.mAngular, _c * _V.mLinear);
}

void se3::setAd(const SE3& _T12, const se3& _V2)
{
    //
    // Let,
    //     _T12 = | R12 p12 |,
    //            |   0   1 |
    //      mAngular = w1, mLinear = v1,
    //     _V.mAngular = w2, _V.mLinear = v2.
    // Then,
    //     w1 = R12 * w2
    //     v1 = p12 x (R12 * w2) + R12 * v2
    //

    const Eigen::Vector3d& v2 = _V2.getLinear();
    const so3& w2 = _V2.getAngular();
    const SO3& R12 = _T12.getRotation();
    const Eigen::Vector3d& p12 = _T12.getPosition();

    mAngular = R12 * w2;
    mLinear = p12.cross(mAngular.mw) + R12 * v2;
}

void se3::setInvAd(const SE3& _T21, const se3& _V2)
{
	//
	// Let,
	//     _T21 = | R21 p21 |,
	//            |   0   1 |
	//      mAngular = w1, mLinear = v1,
	//     _V.mAngular = w2, _V.mLinear = v2.
	//     T12 = _T21^{-1} = | R21^T   -R21^T * p21 | = | R12 p12 |
	//                       |     0              1 |   |   0   1 |
	//
	// Then,
	//     w1 = R12 * w2
	//        = R21^T * w2
	//     v1 = p12 x (R12 * w2) + R12 * v2
	//        = (-R21^T * p21) x (R21^T * w2) + R21^T * v2
	//        = -R21^T (p21 x w2) + R21^T * v2
	//

	const Eigen::Vector3d& v2 = _V2.getLinear();
	const so3& w2 = _V2.getAngular();
	const SO3& R12 = _T21.getRotation().getInverse();
	//const Eigen::Vector3d& p21 = _T21.getPosition();
	const Eigen::Vector3d& p12 = -(R12 * _T21.getPosition());

	mAngular = R12 * w2;
	//mLinear = -(R12 * p21.cross(w2)) + R12 * v2;
	mLinear = p12.cross(mAngular.mw) + R12 * v2;
}

void se3::setad(const se3& _V1, const se3& _V2)
{
	const Eigen::Vector3d& v1 = _V1.getLinear();
	const so3& w1 = _V1.getAngular();

	const Eigen::Vector3d& v2 = _V2.getLinear();
	const so3& w2 = _V2.getAngular();

	mAngular.setVector(w1.mw.cross(w2.mw));
	mLinear = v1.cross(w2.mw) + w1.mw.cross(v2);
}

//==============================================================================
dse3::dse3()
    : mLinear(Eigen::Vector3d::Zero()),
      mAngular(Eigen::Vector3d::Zero())
{
}

dse3::dse3(const dse3& _V)
    : mLinear(_V.mLinear),
      mAngular(_V.mAngular)
{
}

dse3::dse3(const Vector6d& _V)
    : mLinear(_V.segment(3,3)),
      mAngular(_V.segment(0,3))
{
}

dse3::dse3(double _m0, double _m1, double _m2,
           double _f0, double _f1, double _f2)
{
    mLinear << _f0, _f1, _f2;
    mAngular << _m0, _m1, _m2;
}

dse3::dse3(const Eigen::Vector3d& _angular,
           const Eigen::Vector3d& _linear)
    : mLinear(_linear),
      mAngular(_angular)
{
}

dse3::~dse3()
{
}

const dse3& dse3::operator = (const dse3& _F)
{
    if(this != &_F)
    {
        mLinear = _F.mLinear;
        mAngular = _F.mAngular;
    }
    
    return *this;
}

dse3 dse3::operator + (void) const
{
    return *this;
}

dse3 dse3::operator - (void) const
{
    return dse3(-mLinear, -mAngular);
}

const dse3& dse3::operator += (const dse3& _F)
{
    mLinear += _F.mLinear;
    mAngular += _F.mAngular;

    return *this;
}

const dse3& dse3::operator -= (const dse3& _F)
{
    mLinear -= _F.mLinear;
    mAngular -= _F.mAngular;

    return *this;
}

const dse3& dse3::operator *= (double _c)
{
    mLinear *= _c;
    mAngular *= _c;

    return *this;
}

dse3 dse3::operator + (const dse3& _F) const
{
    return dse3(mLinear + _F.mLinear, mAngular + _F.mAngular);
}

dse3 dse3::operator - (const dse3& _F) const
{
    return dse3(mLinear - _F.mLinear, mAngular - _F.mAngular);
}

dse3 dse3::operator * (double _c) const
{
    return dse3(mLinear * _c, mAngular * _c);
}

dse3 operator * (double _c, const dse3& _F)
{
    return dse3(_c * _F.mLinear, _c * _F.mAngular);
}

void dse3::setdAd(const SE3& _T12, const dse3& _F2)
{
    //
    // Let,
    //     _T12 = | R12 p12 |,
    //            |   0   1 |
    //      mAngular = m1, mLinear = f1,
    //     _V.mAngular = m2, _V.mLinear = f2.
    // Then,
    //     f1 = R12 * f2
    //     m1 = p12 x (R12 * f2) + R12 * m2
    //

    const Eigen::Vector3d& f2 = _F2.getLinear();
    const Eigen::Vector3d& m2 = _F2.getAngular();
    const SO3& R12 = _T12.getRotation();
    const Eigen::Vector3d& p12 = _T12.getPosition();

    mLinear = R12 * f2;
    mAngular = R12 * m2 + p12.cross(mLinear);
}

void dse3::setInvdAd(const SE3& _T21, const dse3& _F2)
{
	//
	// Let,
	//     _T21 = | R21 p21 |,
	//            |   0   1 |
	//      mAngular = m1, mLinear = f1,
	//     _V.mAngular = m2, _V.mLinear = f2.
	//     T12 = _T21^{-1} = | R21^T   -R21^T * p21 | = | R12 p12 |
	//                       |     0              1 |   |   0   1 |
	//
	// Then,
	//     f1 = R12 * f2
	//        = R21^T * m2
	//     fm = p12 x (R12 * f2) + R12 * m2
	//        = (-R21^T * p21) x (R21^T * f2) + R21^T * m2
	//        = -R21^T (p21 x f2) + R21^T * m2
	//

	const Eigen::Vector3d& f2 = _F2.getLinear();
	const Eigen::Vector3d& m2 = _F2.getAngular();
	const SO3& R12 = _T21.getRotation().getInverse();
	const Eigen::Vector3d& p12 = -(R12 * _T21.getPosition());

	mLinear = R12 * f2;
	mAngular = R12 * m2 + p12.cross(mLinear);
}

void dse3::setdad(const se3& _V, const dse3& _F)
{
	//
	// f' = -w x f
	// m' = -w x m - v x f
	//

	const Eigen::Vector3d& v = _V.getLinear();
	const so3& w = _V.getAngular();

	const Eigen::Vector3d& f = _F.getLinear();
	const Eigen::Vector3d& m = _F.getAngular();

	mLinear = -w.mw.cross(f);
	mAngular = -w.mw.cross(m) - v.cross(f);
}

//==============================================================================
SE3::SE3()
    : mRotation(SO3()),
      mPosition(Eigen::Vector3d::Zero())
{
}

SE3::SE3(const SE3& _T)
    : mRotation(_T.mRotation),
      mPosition(_T.mPosition)
{

}

SE3::SE3(const Eigen::Matrix4d& _T)
    : mRotation(_T.topLeftCorner<3,3>()),
      mPosition(_T.topRightCorner<3,1>())
{
    assert(_T(3,0) == 0);
    assert(_T(3,1) == 0);
    assert(_T(3,2) == 0);

    assert(_T(3,3) == 1);
}

SE3::SE3(const SO3& _R, const Eigen::Vector3d& _p)
    : mRotation(_R),
      mPosition(_p)
{
}

SE3::SE3(double _R00, double _R01, double _R02,
         double _R10, double _R11, double _R12,
         double _R20, double _R21, double _R22,
         double _p0, double _p1, double _p2)
    : mRotation(_R00, _R01, _R02,
                _R10, _R11, _R12,
                _R20, _R21, _R22)
{
    mPosition << _p0, _p1, _p2;
}

SE3::SE3(const so3& _w)
{
    setExp(_w);
}

SE3::SE3(const so3& _w, double _theta)
{
    setExp(_w, _theta);
}

SE3::SE3(const se3& _S)
{
    setExp(_S);
}

SE3::SE3(const se3& _S, double _theta)
{
    setExp(_S, _theta);
}

SE3::~SE3()
{
}

//double& SE3::operator()(int _i, int _j)
//{
//    assert(0 <= _i && _i <= 3);
//    assert(0 <= _j && _j <= 3);

//    // TODO: speed up?
//    return getMatrix()(_i, _j);
//}

const SE3& SE3::operator=(const SE3& _T)
{
    if (this != &_T)
    {
        mRotation = _T.mRotation;
        mPosition = _T.mPosition;
    }

    return *this;
}

const SE3& SE3::operator*=(const SE3& _T)
{
    // T12 = T1 * T2
    //     = | R1 p1 | * | R2 p2 |
    //       |  0  1 |   |  0  1 |
    //     = | R1 * R2   R1 * p2 + p1 |
    //       |       0              1 |
    //
    // p12 = R1 * p2 + p1
    // R12 = R1 * R2

    mPosition += mRotation * _T.mPosition;  // p12 = R1 * p2 + p1
    mRotation *= _T.mRotation;              // R12 = R1 * R2

    return *this;
}

//const SE3& SE3::operator/=(const SE3& _T)
//{

//}

//const SE3& SE3::operator%=(const SE3& _T)
//{

//}

SE3 SE3::operator*(const SE3& _T) const
{
    // this = | R1 p1 |, _T = | R2 p2 |
    //        |  0  1 |,      |  0  1 |
    //
    // return = this * _T
    //
    //        = | R1 p1 | * | R2 p2 |
    //          |  0  1 |   |  0  1 |
    //
    //        = | R1 * R2   R1 * p2 + p1 |
    //          |       0              1 |

    return SE3(mRotation * _T.mRotation,
               mRotation * _T.mPosition + mPosition);
}

TSE3 SE3::operator*(const TSE3& _dT) const
{
    // this = | R1 p1 |, _dT = | R2 p2 |
    //        |  0  1 |,       |  0  0 |
    //
    // return = this * _T
    //
    //        = | R1 p1 | * | R2 p2 |
    //          |  0  1 |   |  0  0 |
    //
    //        = | R1 * R2   R1 * p2 |
    //          |       0         0 |

    return TSE3( Ad(_dT.mT, _dT.mS), _dT.mT * (*this));
}

//SE3 SE3::operator/(const SE3& _T) const
//{

//}

//SE3 SE3::operator%(const SE3& _T) const
//{

//}

Eigen::Vector3d SE3::operator*(const Eigen::Vector3d& _p) const
{
    Eigen::Vector3d res = mRotation * _p + mPosition;

    return res;
}

bool SE3::operator==(const SE3& _T) const
{
    if (mRotation == _T.mRotation
            && mPosition == _T.mPosition)
        return true;
    else
        return false;
}

void SE3::setValues(double _R00, double _R01, double _R02,
					double _R10, double _R11, double _R12,
					double _R20, double _R21, double _R22,
					double _p0, double _p1, double _p2)
{
	mRotation.setValues(_R00, _R01, _R02,
						_R10, _R11, _R12,
						_R20, _R21, _R22);
	mPosition << _p0, _p1, _p2;
}

void SE3::setExp(const so3& _w)
{
	// TODO: use SE3::setExp(const se3& _s)
	mRotation.setExp(_w);
	mPosition.setZero();
}

void SE3::setExp(const so3& _w, double _theta)
{
	// TODO: use SE3::setExp(const se3& _s, double _theta)
	mRotation.setExp(_w, _theta);
	mPosition.setZero();
}

void SE3::setExp(const se3& _s)
{
	// TODO: Need document

	Vector6d _S = _s.getVector();

	double s2[] = { _S[0] * _S[0], _S[1] * _S[1], _S[2] * _S[2] };
	double theta = sqrt(s2[0] + s2[1] + s2[2]);
	double st_t;
	double ct_t;
	double vt_t;

	if ( theta < LIEGROUP_EPS )
	{
		st_t = 1.0 - theta * theta / (double)6.0;
		ct_t = 0.5 - theta * theta / (double)24.0;
		vt_t = (_S[0] * _S[3] + _S[1] * _S[4] + _S[2] * _S[5]) * (1.0 - theta * theta / (double)20.0) / (double)6.0;
	}
	else
	{
		double itheta = 1.0 / theta;
		st_t = sin(theta) * itheta;
		itheta *= itheta;
		ct_t = (1.0 - cos(theta)) * itheta;
		vt_t = (_S[0] * _S[3] + _S[1] * _S[4] + _S[2] * _S[5]) * (1.0 - st_t) * itheta;
	}

	mRotation(0,0) = 1.0 - ct_t * (s2[1] + s2[2]);
	mRotation(1,0) = ct_t * _S[0] * _S[1] + st_t * _S[2];
	mRotation(2,0) = ct_t * _S[0] * _S[2] - st_t * _S[1];

	mRotation(0,1) = ct_t * _S[0] * _S[1] - st_t * _S[2];
	mRotation(1,1) = 1.0 - ct_t * (s2[0] + s2[2]);
	mRotation(2,1) = ct_t * _S[1] * _S[2] + st_t * _S[0];

	mRotation(0,2) = ct_t * _S[0] * _S[2] + st_t * _S[1];
	mRotation(1,2) = ct_t * _S[1] * _S[2] - st_t * _S[0];
	mRotation(2,2) = 1.0 - ct_t * (s2[0] + s2[1]);

	mRotation(0,3) = st_t * _S[3] + vt_t * _S[0] + ct_t * (_S[1] * _S[5] - _S[2] * _S[4]);
	mRotation(1,3) = st_t * _S[4] + vt_t * _S[1] + ct_t * (_S[2] * _S[3] - _S[0] * _S[5]);
	mRotation(2,3) = st_t * _S[5] + vt_t * _S[2] + ct_t * (_S[0] * _S[4] - _S[1] * _S[3]);
}

void SE3::setExp(const se3& _S, double _theta)
{
	// TODO: Need document



//	double s2[] = { _S[0] * _S[0], _S[1] * _S[1], _S[2] * _S[2] };

//	if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > LIEGROUP_EPS )
//	{
//		return setExp(_theta * _S);
//	}

//	double st = sin(_theta),
//			vt = 1.0 - cos(_theta),
//			ut = (_theta - st) * (_S[0] * _S[3] + _S[1] * _S[4] + _S[2] * _S[5]),
//			sts[] = { st * _S[0], st * _S[1], st * _S[2] };

//	mRotation(0,0) = 1.0 + vt * (s2[0] - 1.0);
//	mRotation(1,0) = vt * _S[0] * _S[1] + sts[2];
//	mRotation(2,0) = vt * _S[0] * _S[2] - sts[1];

//	mRotation(0,1) = vt * _S[0] * _S[1] - sts[2];
//	mRotation(1,1) = 1.0 + vt * (s2[1] - 1.0);
//	mRotation(2,1) = vt * _S[1] * _S[2] + sts[0];

//	mRotation(0,2) = vt * _S[0] * _S[2] + sts[1];
//	mRotation(1,2) = vt * _S[1] * _S[2] - sts[0];
//	mRotation(2,2) = 1.0 + vt * (s2[2] - 1.0);

//	mRotation(0,3) = st * _S[3] + ut * _S[0] + vt * (_S[1] * _S[5] - _S[2] * _S[4]);
//	mRotation(1,3) = st * _S[4] + ut * _S[1] + vt * (_S[2] * _S[3] - _S[0] * _S[5]);
//	mRotation(2,3) = st * _S[5] + ut * _S[2] + vt * (_S[0] * _S[4] - _S[1] * _S[3]);
}

void SE3::setIdentity()
{
	mRotation.setIdentity();
	mPosition.setZero();
}

void SE3::setInverse()
{
	//
	// T = | R p |
	//     | 0 1 |
	// T^{-1} = | R^{T} -R^{T}p |
	//        = |     0       1 |
	//

	//mRotation = mRotation.transpose();		// R^{T}
	mRotation = mRotation.getInverse();		// R^{T}
	mPosition = -(mRotation * mPosition);	// -R^{T} * p
}

SE3 SE3::getInverse() const
{
	return SE3(mRotation.getInverse(), -(mRotation.getInverse() * mPosition));
}

//Eigen::Vector3d SE3::operator%(const Eigen::Vector3d& _q) const
//{

//}

Eigen::Matrix4d SE3::getMatrix() const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.topLeftCorner<3,3>() = mRotation.getMatrix();
    T.topRightCorner<3,1>() = mPosition;

    return T;
}




//==============================================================================
TSE3::TSE3()
{
}

TSE3::~TSE3()
{
}

TSE3::TSE3(const TSE3& _dT)
    : mS(_dT.mS),
      mT(_dT.mT)
{
}

TSE3::TSE3(const se3& _S, const SE3& _T)
    : mS(_S), mT(_T)
{
}

TSE3::TSE3(const SE3& _T, const se3& _S)
//    : mS()
{
}

const TSE3 &TSE3::operator=(const TSE3& _dT)
{
    if (this != &_dT)
    {
        mS = _dT.mS;
        mT = _dT.mT;
    }

    return (*this);
}

//const TSE3& TSE3::operator*=(const TSE3& _dT)
//{
//    // Let,
//    //     this = | R1 p1 |, _dT = | R2 p2 |
//    //            |  0  0 |        |  0  0 |
//    // Then,
//    //     this = | R1 p1 | * | R2 p2 |
//    //            |  0  0 |   |  0  0 |
//    //
//    //          = | R1 * R2   R1 * p2 |
//    //            |       0         0 |

//    mPosition = mRotation * _dT.mPosition;
//    mRotation = mRotation * _dT.mRotation;

//    return *this;
//}

const TSE3& TSE3::operator*=(const SE3& _T)
{
    // Let,
    //     this = | R1 p1 |, _T = | R2 p2 |
    //            |  0  0 |       |  0  1 |
    // Then,
    //     this = | R1 p1 | * | R2 p2 |
    //            |  0  0 |   |  0  1 |
    //
    //          = | R1 * R2   R1 * p2 + p1 |
    //            |       0              0 |

//    mPosition = mRotation * _T.mPosition + mPosition;
//    mRotation = mRotation * _T.mRotation.getMatrix();

    return  *this;
}

const TSE3& TSE3::operator*=(const se3& _S)
{
    // Let,
    //     this = | R1 p1 |, _S = | [w] v |
    //            |  0  0 |       |  0  0 |
    // Then,
    //     this = | R1 p1 | * | [w] v |
    //            |  0  0 |   |  0  0 |
    //
    //          = | R1 * [w]   R1 * v |
    //            |       0         0 |

//    mPosition = mRotation * _S.getLinear();
//    mRotation = mRotation * _S.getAngular().getSkewSymmetrixMatrix();

    return  *this;
}

//TSE3 TSE3::operator*(const TSE3& _dT) const
//{
//    // Let,
//    //     this = | R1 p1 |, _dT = | R2 p2 |
//    //            |  0  0 |        |  0  0 |
//    // Then,
//    //     return = | R1 p1 | * | R2 p2 |
//    //              |  0  0 |   |  0  0 |
//    //
//    //            = | R1 * R2   R1 * p2 |
//    //              |       0         0 |

//    return TSE3(mRotation * _dT.mRotation,
//                mRotation * _dT.mPosition);
//}

TSE3 TSE3::operator*(const SE3& _T) const
{
    // Let,
    //     this = | R1 p1 |, _T = | R2 p2 |
    //            |  0  0 |       |  0  1 |
    // Then,
    //     return = | R1 p1 | * | R2 p2 |
    //              |  0  0 |   |  0  1 |
    //
    //            = | R1 * R2   R1 * p2 + p1 |
    //              |       0              0 |

//    return TSE3(mRotation * _T.mRotation.getMatrix(),
//                mRotation * _T.mPosition + mPosition);
    return TSE3(mS, mT * _T);
}

TSE3 TSE3::operator*(const se3& _S) const
{
    // Let,
    //     this = | R1 p1 |, _S = | [w] v |
    //            |  0  0 |       |  0  0 |
    // Then,
    //     return  = | R1 p1 | * | [w] v |
    //               |  0  0 |   |  0  0 |
    //
    //             = | R1 * [w]   R1 * v |
    //               |       0         0 |

//    const Eigen::Matrix3d& wSkeySymemetrix
//            = _S.getAngular().getSkewSymmetrixMatrix();
//    const Eigen::Vector3d& v
//            = _S.getLinear();

//    return TSE3(mRotation * wSkeySymemetrix,
//                mRotation * v);
    return TSE3();
}

void TSE3::setZero()
{
	mS.setZero();
	mT.setIdentity();
}

} // namespace math
} // namespace dart
