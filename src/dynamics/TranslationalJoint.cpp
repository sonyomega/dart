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

#include "math/LieGroup.h"
#include "dynamics/TranslationalJoint.h"

namespace dart {
namespace dynamics {

TranslationalJoint::TranslationalJoint()
    : Joint()
{
    mName.assign("Translational joint");
    mJointType = TRANSLATIONAL;
    mDofs.push_back(&mCoordinate[0]);
    mDofs.push_back(&mCoordinate[1]);
    mDofs.push_back(&mCoordinate[2]);
    mS = Eigen::Matrix<double,6,3>::Zero();
    mdS = Eigen::Matrix<double,6,3>::Zero();

    // TODO: Temporary code
    mDampingCoefficient.resize(3, 0);
}

TranslationalJoint::~TranslationalJoint()
{
}

void TranslationalJoint::_updateTransformation()
{
    // T
    math::Vec3 v(mCoordinate[0].get_q(),
            mCoordinate[1].get_q(),
            mCoordinate[2].get_q());

    mT = mT_ParentBodyToJoint
         * math::ExpLinear(v)
         * math::Inv(mT_ChildBodyToJoint);
}

void TranslationalJoint::_updateVelocity()
{
    // S
    mS = Eigen::Matrix<double,6,3>::Zero();
    mS.bottomRows<3>() = Eigen::Matrix3d::Identity();

    // V = S * dq
    mV = mS * get_dq();
}

void TranslationalJoint::_updateAcceleration()
{
    // dS = 0
    mdS.setZero();

    // dV = dS * dq + S * ddq
    mdV = mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
