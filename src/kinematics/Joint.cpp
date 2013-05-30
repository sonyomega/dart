/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "renderer/RenderInterface.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"

namespace dart {
namespace kinematics {

Joint::Joint()
    : mName("Unknown joint"),
      mJointType(UNKNOWN),
      mParentBody(NULL),
      mChildBody(NULL),
      mT_ParentBodyToJoint(math::SE3()),
      mT_ChildBodyToJoint(math::SE3()),
      mT(math::SE3()),
      mV(math::se3()),
      mS(math::Jacobian()),
      mdV(math::se3()),
      mdS(math::Jacobian())
{
}

Joint::~Joint()
{
}

void Joint::setParentBody(BodyNode* _body)
{
    mParentBody = _body;

    // TODO: Use builder
    if (mParentBody != NULL)
    {
        mParentBody->addChildJoint(this);

        if (mChildBody != NULL)
        {
            mChildBody->setParentBody(mParentBody);
            mParentBody->addChildBody(mChildBody);
        }
    }
}

void Joint::setChildBody(BodyNode* _body)
{
    mChildBody = _body;

    // TODO: Use builder
    if (mChildBody != NULL)
    {
        mChildBody->setParentJoint(this);

        if (mParentBody != NULL)
        {
            mParentBody->addChildBody(mChildBody);
            mChildBody->setParentBody(mParentBody);
        }
    }
}

void Joint::setLocalTransformFromParentBody(const math::SE3& _T)
{
    mT_ParentBodyToJoint = _T;
}

void Joint::setLocalTransformFromChildBody(const math::SE3& _T)
{
    mT_ChildBodyToJoint = _T;
}

void Joint::updateKinematics(bool _firstDerivative,
                                  bool _secondDerivative)
{
    _updateTransformation();
    _updateVelocity();
    _updateAcceleration();
}

void Joint::applyGLTransform(renderer::RenderInterface* _ri)
{
    Eigen::Vector3d offset = mT.getPosition();
    Eigen::Vector3d axis;
    double rad;
    mT.getOrientation().getAxisAngle(&axis, &rad);

    _ri->translate(offset);
    _ri->rotate(axis, rad * 180.0 / M_PI);
}

} // namespace kinematics
} // namespace dart
