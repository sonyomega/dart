/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/21/2013
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

#include "math/UtilsMath.h"
#include "math/LieGroup.h"
#include "dynamics/BallJoint.h"

namespace dart {
namespace dynamics {

#define BJOINT_EPS 1e-6

BallJoint::BallJoint()
    : Joint("Ball joint")
{
    mJointType = BALL;
    mDofs.push_back(&mCoordinate[0]);
    mDofs.push_back(&mCoordinate[1]);
    mDofs.push_back(&mCoordinate[2]);
    mS.setSize(3);
    mdS.setSize(3);

    // TODO: Temporary code
    mDampingCoefficient.resize(3, 0);
}

BallJoint::~BallJoint()
{
}

inline void BallJoint::_updateTransformation()
{
    // T
    math::so3 w(mCoordinate[0].get_q(),
            mCoordinate[1].get_q(),
            mCoordinate[2].get_q());
    mT = mT_ParentBodyToJoint
         * math::Exp(w)
         / mT_ChildBodyToJoint;
}

inline void BallJoint::_updateVelocity()
{
    // TODO: NEED TO CHECK !!
    //       NOT FINISHED.

    // S
    Eigen::Vector3d q;
    q << mCoordinate[0].get_q(), mCoordinate[1].get_q(), mCoordinate[2].get_q();
    double theta = q.norm();

    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d qss = math::makeSkewSymmetric(q);
    Eigen::Matrix3d qss2 =  qss*qss;

    if(theta < BJOINT_EPS)
        J = Eigen::Matrix3d::Identity() + 0.5*qss +  (1.0/6.0)*qss2;
    else
        J = Eigen::Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;

    math::se3 J0(J(0,0), J(0,1), J(0,2), 0, 0, 0);
    math::se3 J1(J(1,0), J(1,1), J(1,2), 0, 0, 0);
    math::se3 J2(J(2,0), J(2,1), J(2,2), 0, 0, 0);

    mS.setColumn(0, math::Ad(mT_ChildBodyToJoint, J0));
    mS.setColumn(1, math::Ad(mT_ChildBodyToJoint, J1));
    mS.setColumn(2, math::Ad(mT_ChildBodyToJoint, J2));

    // V = S * dq
    mV = mS * get_dq();
}

inline void BallJoint::_updateAcceleration()
{
    // TODO: NEED TO CHECK !!
    //       NOT FINISHED.

    // dS
    Eigen::Vector3d q;
    Eigen::Vector3d dq;
    q << mCoordinate[0].get_q(), mCoordinate[1].get_q(), mCoordinate[2].get_q();
    dq << mCoordinate[0].get_dq(), mCoordinate[1].get_dq(), mCoordinate[2].get_dq();
    double theta = q.norm();

    Matrix3d Jdot = Matrix3d::Zero();
    Matrix3d qss =  math::makeSkewSymmetric(q);
    Matrix3d qss2 =  qss*qss;
    Matrix3d qdss = math::makeSkewSymmetric(dq);
    double ttdot = q.dot(dq);   // theta*thetaDot
    double st = sin(theta);
    double ct = cos(theta);
    double t2 = theta*theta;
    double t3 = t2*theta;
    double t4 = t3*theta;
    double t5 = t4*theta;

    if (theta < BJOINT_EPS)
    {
        Jdot = 0.5*qdss + (1.0/6.0)*(qss*qdss + qdss*qss);
        Jdot += (-1.0/12)*ttdot*qss + (-1.0/60)*ttdot*qss2;
    }
    else
    {
        Jdot = ((1-ct)/t2)*qdss + ((theta-st)/t3)*(qss*qdss + qdss*qss);
        Jdot += ((theta*st + 2*ct - 2)/t4)*ttdot*qss + ((3*st - theta*ct - 2*theta)/t5)*ttdot*qss2;
    }

    math::se3 dJ0(Jdot(0,0), Jdot(0,1), Jdot(0,2), 0, 0, 0);
    math::se3 dJ1(Jdot(1,0), Jdot(1,1), Jdot(1,2), 0, 0, 0);
    math::se3 dJ2(Jdot(2,0), Jdot(2,1), Jdot(2,2), 0, 0, 0);

    mdS.setColumn(0, math::Ad(mT_ChildBodyToJoint, dJ0));
    mdS.setColumn(1, math::Ad(mT_ChildBodyToJoint, dJ1));
    mdS.setColumn(2, math::Ad(mT_ChildBodyToJoint, dJ2));

    // dV = dS * dq + S * ddq
    mdV = mdS * get_dq() + mS * get_ddq();
}


EulerXYZJoint::EulerXYZJoint()
    : Joint("EulerXYZ joint")
{
    mJointType = EULER_XYZ;
    mDofs.push_back(&mCoordinate[0]);
    mDofs.push_back(&mCoordinate[1]);
    mDofs.push_back(&mCoordinate[2]);
    mS.setSize(3);
    mdS.setSize(3);

    // TODO: Temporary code
    mDampingCoefficient.resize(3, 0);
}

EulerXYZJoint::~EulerXYZJoint()
{
}

inline void EulerXYZJoint::_updateTransformation()
{
    // T
    mT = mT_ParentBodyToJoint *
         math::EulerXYZ(math::Vec3(mCoordinate[0].get_q(),
                                   mCoordinate[1].get_q(),
                                   mCoordinate[2].get_q())) /
         mT_ChildBodyToJoint;
}

inline void EulerXYZJoint::_updateVelocity()
{
    // S
    double q0 = mCoordinate[0].get_q();
    double q1 = mCoordinate[1].get_q();
    double q2 = mCoordinate[2].get_q();

    //double c0 = cos(q0);
    double c1 = cos(q1);
    double c2 = cos(q2);

    //double s0 = sin(q0);
    double s1 = sin(q1);
    double s2 = sin(q2);

    // S = [    c1*c2, s2,  0
    //       -(c1*s2), c2,  0
    //             s1,  0,  1
    //              0,  0,  0
    //              0,  0,  0
    //              0,  0,  0 ];
    math::se3 J0(c1*c2, -(c1*s2),  s1, 0.0, 0.0, 0.0);
    math::se3 J1(   s2,       c2, 0.0, 0.0, 0.0, 0.0);
    math::se3 J2(  0.0,      0.0, 1.0, 0.0, 0.0, 0.0);

    mS.setColumn(0, math::Ad(mT_ChildBodyToJoint, J0));
    mS.setColumn(1, math::Ad(mT_ChildBodyToJoint, J1));
    mS.setColumn(2, math::Ad(mT_ChildBodyToJoint, J2));

    // V = S * dq
    mV = mS * get_dq();
}

inline void EulerXYZJoint::_updateAcceleration()
{
    // dS
    double q0 = mCoordinate[0].get_q();
    double q1 = mCoordinate[1].get_q();
    double q2 = mCoordinate[2].get_q();

    double dq0 = mCoordinate[0].get_dq();
    double dq1 = mCoordinate[1].get_dq();
    double dq2 = mCoordinate[2].get_dq();

    double c0 = cos(q0);
    double c1 = cos(q1);
    double c2 = cos(q2);

    double s0 = sin(q0);
    double s1 = sin(q1);
    double s2 = sin(q2);

    // dS = [  -(dq1*c2*s1) - dq2*c1*s2,    dq2*c2,  0
    //         -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2),  0
    //                           dq1*c1,         0,  0
    //                                0,         0,  0
    //                                0,         0,  0
    //                                0,         0,  0 ];

    math::se3 dJ0(-(dq1*c2*s1) - dq2*c1*s2, -(dq2*c1*c2) + dq1*s1*s2, dq1*c1, 0, 0, 0);
    math::se3 dJ1(                  dq2*c2,                -(dq2*s2),    0.0, 0.0, 0.0, 0.0);
    math::se3 dJ2(0.0);

    mdS.setColumn(0, math::Ad(mT_ChildBodyToJoint, dJ0));
    mdS.setColumn(1, math::Ad(mT_ChildBodyToJoint, dJ1));
    mdS.setColumn(2, math::Ad(mT_ChildBodyToJoint, dJ2));

    // dV = dS * dq + S * ddq
    mdV = mdS * get_dq() + mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
