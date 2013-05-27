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

#include "math/UtilsMath.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"

namespace dart {
namespace dynamics {

SkeletonDynamics::SkeletonDynamics()
    : kinematics::Skeleton(),
      mTotalMass(0.0)
{
}

SkeletonDynamics::~SkeletonDynamics()
{
}

void SkeletonDynamics::initDynamics()
{
//    mDynamicsBodies.clear();

//    for (std::vector<kinematics::BodyNode*>::iterator itrBody = mBodies.begin();
//         itrBody != mBodies.end();
//         ++itrBody)
//        mDynamicsBodies.push_back(dynamic_cast<BodyNodeDynamics*>(*itrBody));
}

void SkeletonDynamics::computeInverseDynamics(const Eigen::Vector3d& _gravity)
{
    _inverseDynamicsFwdRecursion();
    _inverseDynamicsBwdRecursion(_gravity);
}

void SkeletonDynamics::computeForwardDynamics(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void SkeletonDynamics::computeForwardDynamicsID(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    int n = getNumDofs();

    // Save current tau
    Eigen::VectorXd tau_old = get_tau();

    // Set ddq as zero
    set_ddq(Eigen::VectorXd::Zero(n));

    //
    mM = Eigen::Matrix4d::Zero(n,n);

    // M(q) * ddq + b(q,dq) = tau
    computeInverseDynamics(_gravity);
    Eigen::VectorXd b = get_tau();

    // Calcualtion M
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorXd basis = Eigen::VectorXd::Zero(n);
        basis(i) = 1;
        set_ddq(basis);
        computeInverseDynamics(_gravity);
        mM.col(i) = get_tau() - b;
    }

    //
    set_tau(tau_old);

    // TODO:
    mMInv = mM.inverse();
    //mMInv = mM.ldlt().solve(MatrixXd::Identity(n,n));

    Eigen::VectorXd new_ddq = mMInv * (tau_old - b);
    set_ddq(new_ddq);
}

void SkeletonDynamics::computeForwardDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void SkeletonDynamics::computeHybridDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void SkeletonDynamics::computeEquationsOfMotionID(
        const Eigen::Vector3d& _gravity)
{
}

void SkeletonDynamics::computeEquationsOfMotionRecursive(
        const Eigen::Vector3d& _gravity)
{
}

void SkeletonDynamics::_inverseDynamicsFwdRecursion()
{
    // Forward recursion
    for (std::vector<kinematics::BodyNode*>::iterator itrBody = mBodies.begin();
         itrBody != mBodies.end();
         ++itrBody)
    {
        (*itrBody)->_updateTransformation();
        (*itrBody)->_updateVelocity();
        (*itrBody)->_updateAcceleration();
    }
}

void SkeletonDynamics::_inverseDynamicsBwdRecursion(const Eigen::Vector3d& _gravity)
{
    // Backward recursion
    for (std::vector<kinematics::BodyNode*>::reverse_iterator ritrBody = mBodies.rbegin();
         ritrBody != mBodies.rend();
         ++ritrBody)
    {
        dynamics::BodyNodeDynamics* body
                = dynamic_cast<dynamics::BodyNodeDynamics*>(*ritrBody);

        body->_updateBodyForce(_gravity);
        body->_updateGeneralizedForce();
    }
}


} // namespace dynamics
} // namespace dart
