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
#include "dynamics/BodyNode.h"
#include "dynamics/PrismaticJoint.h"

namespace dart {

using namespace math;

namespace dynamics {

PrismaticJoint::PrismaticJoint(const Vec3& axis)
    : Joint("Revolute joint"),
      mDirectionVector(axis)
      //mDampingCoefficient(0.0)
{
    mJointType = REVOLUTE;
    mDofs.push_back(&mCoordinate);
    mS.setSize(1);
    mdS.setSize(1);

    // TODO: Temporary code
    mDampingCoefficient.resize(1, 0);
}

PrismaticJoint::~PrismaticJoint()
{
}

Vec3 PrismaticJoint::getAxisGlobal() const
{
    math::SE3 parentTransf;

    if (this->mParentBody != NULL)
        parentTransf = mParentBody->getTransformationWorld();

    return math::Rotate(parentTransf * mT_ParentBodyToJoint, mDirectionVector);
}

void PrismaticJoint::_updateTransformation()
{
    // T
    mT = mT_ParentBodyToJoint
         * Exp(mDirectionVector * mCoordinate.get_q())
         * Inv(mT_ChildBodyToJoint);
}

void PrismaticJoint::_updateVelocity()
{
    // S
    mS.setColumn(0, math::Ad(mT_ChildBodyToJoint, math::se3(mDirectionVector)));

    // V = S * dq
    mV = mS * get_dq();
    //mV.setAngular(mAxis * mCoordinate.get_q());
}

void PrismaticJoint::_updateAcceleration()
{
    // dS = 0
    mdS.setZero();

    // dV = dS * dq + S * ddq
    mdV = mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
