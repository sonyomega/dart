/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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
#include "renderer/RenderInterface.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"

namespace dart {
namespace kinematics {

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode()
    : //mVizShapes(std::vector<Shape*>(1, static_cast<Shape*>(NULL))),
      mVizShape(NULL),
      //mColShapes(std::vector<Shape*>(1, static_cast<Shape*>(NULL))),
      mColShape(NULL),
      mColliding(true),
      mSkeleton(NULL),
      mJointParent(NULL),
      mJointsChild(std::vector<Joint*>(0)),
      mParentBody(NULL),
      mChildBodies(std::vector<BodyNode*>(0)),
      mW(math::SE3()),
      mV(math::se3()),
      mdV(math::se3())
{
    mID = BodyNode::msBodyNodeCount++;
}

BodyNode::~BodyNode()
{
}

void BodyNode::addChildJoint(Joint* _joint)
{
    assert(_joint != NULL);

    mJointsChild.push_back(_joint);
}

Joint*BodyNode::getChildJoint(int _idx) const
{
    assert(0 <= _idx && _idx < mJointsChild.size());

    return mJointsChild[_idx];
}

void BodyNode::addChildBody(BodyNode* _body)
{
    assert(_body != NULL);

    mChildBodies.push_back(_body);
}

BodyNode*BodyNode::getChildNode(int _idx) const
{
    assert(0 <= _idx && _idx < mChildBodies.size());

    return mChildBodies[_idx];
}

BodyNode*BodyNode::getChildBody(int _idx) const
{
    assert(0 <= _idx && _idx < mChildBodies.size());

    return mChildBodies[_idx];
}

void BodyNode::updateForwardKinematics(bool _firstDerivative, bool _secondDerivative)
{
    _updateTransformation();

    if (_firstDerivative)
        _updateVelocity();

    if (_secondDerivative)
        _updateAcceleration();
}

void BodyNode::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor,
                    int _depth) const
{
    if (_ri == NULL)
        return;

    _ri->pushMatrix();

    // render the self geometry
    mJointParent->applyGLTransform(_ri);

    if (mVizShape != NULL)
    {
        _ri->pushName((unsigned)mID);
        _ri->pushMatrix();
        mVizShape->draw(_ri, _color, _useDefaultColor);
        _ri->popMatrix();
        _ri->popName();
    }

    // render the subtree
    for (unsigned int i = 0; i < mJointsChild.size(); i++)
        mJointsChild[i]->getChildNode()->draw(_ri, _color, _useDefaultColor);

    _ri->popMatrix();
}

void BodyNode::_updateTransformation()
{
    if (mParentBody)
        mW = mParentBody->getWorldTransformation();

    mW *= mJointParent->getLocalTransformation();
}

void BodyNode::_updateVelocity(bool _updateJacobian)
{
    // V(i) = Ad(T(i, i-1), V(i-1)) + S * dq
    if (mParentBody)
    {
        mV.setInvAd(mJointParent->getLocalTransformation(),
                    mParentBody->getBodyVelocity());
        mV += mJointParent->getLocalVelocity();
    }
    else
    {
        mV = mJointParent->getLocalVelocity();
    }

    dterr << "Not implemented: Jacobian update.\n";
}

void BodyNode::_updateAcceleration(bool _updateJacobianDeriv)
{
    // dV(i) = Ad(T(i, i-1), dV(i-1)) + ad(V(i), S * dq)
    //         + dS * dq + S * ddq
    if (mParentBody)
    {
        mdV.setInvAd(mJointParent->getLocalTransformation(),
                    mParentBody->getBodyAcceleration());
        mdV += math::ad(mV, mJointParent->getLocalVelocity());
        mdV += mJointParent->getLocalAcceleration();
    }
    else
    {
        mdV = math::ad(mV, mJointParent->getLocalVelocity())
              + mJointParent->getLocalAcceleration();
    }

    dterr << "Not implemented: JacobianDeriv update.\n";
}

} // namespace kinematics
} // namespace dart

