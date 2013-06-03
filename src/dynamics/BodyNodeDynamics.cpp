/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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
#include "kinematics/Joint.h"
#include "dynamics/BodyNodeDynamics.h"

namespace dart {
namespace dynamics {

BodyNodeDynamics::BodyNodeDynamics()
    : mGravityMode(true),
      mI(math::Inertia()),
      mF(math::dse3())
{
}

BodyNodeDynamics::~BodyNodeDynamics()
{
}

void BodyNodeDynamics::setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                                       double _Ixy, double _Ixz, double _Iyz)
{
    setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy, _Ixz, _Iyz);
}

void BodyNodeDynamics::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                                          double _Ixy, double _Ixz, double _Iyz)
{
    mI.setAngularMomentDiag(_Ixx, _Iyy, _Izz);
    mI.setAngularMomentOffDiag(_Ixy, _Ixz, _Iyz);
}

void BodyNodeDynamics::setExternalForceLocal(const math::dse3& _FextLocal)
{
    mFext = _FextLocal;
}

void BodyNodeDynamics::setExternalForceGlobal(const math::dse3& _FextWorld)
{
    mFext = _FextWorld;
}

void BodyNodeDynamics::setExternalForceLocal(
        const Eigen::Vector3d& _posLocal,
        const Eigen::Vector3d& _linearForceGlobal)
{
}

void BodyNodeDynamics::addExtForce(const Eigen::Vector3d& _offset, const Eigen::Vector3d& _force, bool _isOffsetLocal, bool _isForceLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNodeDynamics::addExternalForceLocal(const math::dse3& _FextLocal)
{
    mFext += _FextLocal;
}

void BodyNodeDynamics::addExternalForceGlobal(const math::dse3& _FextWorld)
{
    mFext += _FextWorld;
}

void BodyNodeDynamics::addExternalForceLocal(
        const Eigen::Vector3d& _posLocal,
        const Eigen::Vector3d& _linearForceGlobal)
{
    dterr << "Not implemented.\n";
}

math::dse3 BodyNodeDynamics::getExternalForceGlobal() const
{
    dterr << "Not implemented.\n";
}

void BodyNodeDynamics::_updateBodyForce(const Eigen::Vector3d& _gravity)
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
        kinematics::Joint* childJoint = (*itrBody)->getParentJoint();
        assert(childJoint != NULL);
        BodyNodeDynamics* bodyDyn = dynamic_cast<BodyNodeDynamics*>(*itrBody);
        assert(bodyDyn != NULL);

        mF += math::InvdAd(childJoint->getLocalTransformation(),
                           bodyDyn->getBodyForce());
    }
}

void BodyNodeDynamics::_updateGeneralizedForce()
{
    assert(mParentJoint != NULL);

    const math::Jacobian& J = mParentJoint->getLocalJacobian();

    assert(J.getSize() == mParentJoint->getNumDofs());

    mParentJoint->set_tau(J.getInnerProduct(mF));
}

void BodyNodeDynamics::_updateDampingForce()
{
    dterr << "Not implemented.\n";
}

void BodyNodeDynamics::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNodeDynamics::clearExternalForces()
{
    dterr << "Not implemented.\n";
}

} // namespace dynamics
} // namespace dart
