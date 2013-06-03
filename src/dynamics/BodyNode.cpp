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

#include <iostream>

#include "utils/UtilsCode.h"
#include "renderer/RenderInterface.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Joint.h"
#include "dynamics/Shape.h"

namespace dart {
namespace dynamics {

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode()
    : //mVizShapes(std::vector<Shape*>(1, static_cast<Shape*>(NULL))),
      mVizShape(NULL),
      //mColShapes(std::vector<Shape*>(1, static_cast<Shape*>(NULL))),
      mColShape(NULL),
      mColliding(true),
      mSkeleton(NULL),
      mParentJoint(NULL),
      mJointsChild(std::vector<Joint*>(0)),
      mParentBody(NULL),
      mChildBodies(std::vector<BodyNode*>(0)),
      mW(math::SE3()),
      mV(math::se3()),
      mdV(math::se3()),
      mGravityMode(true),
      mI(math::Inertia()),
      mF(math::dse3())
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

void BodyNode::setDependDofList()
{
    mDependentDofs.clear();
    if (mParentBody != NULL)
    {
        mDependentDofs.insert(mDependentDofs.end(),
                              mParentBody->mDependentDofs.begin(),
                              mParentBody->mDependentDofs.end());
    }

    for (int i = 0; i < getNumLocalDofs(); i++)
    {
        int dofID = getDof(i)->getSkelIndex();
        mDependentDofs.push_back(dofID);
    }

#if _DEBUG
    for (int i = 0; i < (int)mDependentDofs.size() - 1; i++)
    {
        int now = mDependentDofs[i];
        int next = mDependentDofs[i + 1];
        if (now > next)
        {
            cerr << "Array not sorted!!!" << endl;
            exit(0);
        }
    }
#endif
}

int BodyNode::getNumLocalDofs() const
{
    return mParentJoint->getNumDofs();
}

Dof* BodyNode::getDof(int _idx) const
{
    return mParentJoint->getDof(_idx);
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
    mParentJoint->applyGLTransform(_ri);

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
        mW = mParentBody->getWorldTransformation()
             * mParentJoint->getLocalTransformation();
    else
        mW = mParentJoint->getLocalTransformation();
}

void BodyNode::_updateVelocity(bool _updateJacobian)
{
    // V(i) = Ad(T(i, i-1), V(i-1)) + S * dq
    if (mParentBody)
    {
        mV.setInvAd(mParentJoint->getLocalTransformation(),
                    mParentBody->getBodyVelocity());
        mV += mParentJoint->getLocalVelocity();
    }
    else
    {
        mV = mParentJoint->getLocalVelocity();
    }

    // TODO:
    //dterr << "Not implemented: Jacobian update.\n";
}

void BodyNode::_updateAcceleration(bool _updateJacobianDeriv)
{
    // dV(i) = Ad(T(i, i-1), dV(i-1)) + ad(V(i), S * dq)
    //         + dS * dq + S * ddq
    if (mParentBody)
    {
        mdV.setInvAd(mParentJoint->getLocalTransformation(),
                    mParentBody->getBodyAcceleration());
        mdV += math::ad(mV, mParentJoint->getLocalVelocity());
        mdV += mParentJoint->getLocalAcceleration();
    }
    else
    {
        mdV = math::ad(mV, mParentJoint->getLocalVelocity())
              + mParentJoint->getLocalAcceleration();
    }

    //
    //dterr << "Not implemented: JacobianDeriv update.\n";
}

void BodyNode::setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                                       double _Ixy, double _Ixz, double _Iyz)
{
    setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy, _Ixz, _Iyz);
}

void BodyNode::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                                          double _Ixy, double _Ixz, double _Iyz)
{
    mI.setAngularMomentDiag(_Ixx, _Iyy, _Izz);
    mI.setAngularMomentOffDiag(_Ixy, _Ixz, _Iyz);
}

void BodyNode::setExternalForceLocal(const math::dse3& _FextLocal)
{
    mFext = _FextLocal;
}

void BodyNode::setExternalForceGlobal(const math::dse3& _FextWorld)
{
    mFext = _FextWorld;
}

void BodyNode::setExternalForceLocal(
        const Eigen::Vector3d& _posLocal,
        const Eigen::Vector3d& _linearForceGlobal)
{
}

void BodyNode::addExtForce(const Eigen::Vector3d& _offset, const Eigen::Vector3d& _force, bool _isOffsetLocal, bool _isForceLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNode::addExternalForceLocal(const math::dse3& _FextLocal)
{
    mFext += _FextLocal;
}

void BodyNode::addExternalForceGlobal(const math::dse3& _FextWorld)
{
    mFext += _FextWorld;
}

void BodyNode::addExternalForceLocal(
        const Eigen::Vector3d& _posLocal,
        const Eigen::Vector3d& _linearForceGlobal)
{
    dterr << "Not implemented.\n";
}

math::dse3 BodyNode::getExternalForceGlobal() const
{
    dterr << "Not implemented.\n";
}

void BodyNode::_updateBodyForce(const Eigen::Vector3d& _gravity)
{
    mFgravity = mI * math::InvAd(mW, math::Vec3(_gravity));

    mF = mI * mdV;                // Inertial force
    mF -= mFext;                  // External force
    mF -= mFgravity;              // Gravity force
    mF -= math::dad(mV, mI * mV); // Coriolis force

    for (std::vector<BodyNode*>::iterator itrBody = mChildBodies.begin();
         itrBody != mChildBodies.end();
         ++itrBody)
    {
        dynamics::Joint* childJoint = (*itrBody)->getParentJoint();
        assert(childJoint != NULL);
        BodyNode* bodyDyn = dynamic_cast<BodyNode*>(*itrBody);
        assert(bodyDyn != NULL);

        mF += math::InvdAd(childJoint->getLocalTransformation(),
                           bodyDyn->getBodyForce());
    }
}

void BodyNode::_updateGeneralizedForce()
{
    assert(mParentJoint != NULL);

    const math::Jacobian& J = mParentJoint->getLocalJacobian();

    assert(J.getSize() == mParentJoint->getNumDofs());

    mParentJoint->set_tau(J.getInnerProduct(mF));
}

void BodyNode::_updateDampingForce()
{
    dterr << "Not implemented.\n";
}

void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNode::clearExternalForces()
{
    dterr << "Not implemented.\n";
}

} // namespace dynamics
} // namespace dart

