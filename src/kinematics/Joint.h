/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef DART_KINEMATICS_JOINT_H
#define DART_KINEMATICS_JOINT_H

#include "math/SE3.h"
#include "math/Jacobian.h"
#include "kinematics/System.h"

namespace dart {
namespace kinematics {

class BodyNode;

/// @brief
///
/// [Members]
/// T: local transformation (4x4 matrix)
/// S: local Jacobian (6xm matrix)
/// dS: localJacobianDerivative (6xm matrix)
/// q: generalized coordinates (configuration) (scalar)
/// dq: generalized velocity (scalar)
/// ddq: generalized acceleration (scalar)
/// tau: generalized force (torque) (scalar)
class Joint : public System
{
public:
    /// @brief
    enum JointType
    {
        UNKNOWN,
        WELD,          // 0 dof
        REVOLUTE,
        PRISMATIC,     // 1 dof
        UNIVERSAL,
        PLANAR,        // 2 dof
        TRANSLATIONAL,
        SPHERICAL,     // 3 dof
        FREE           // 6 dof
    };

public:
    /// @brief
    Joint();

    /// @brief
    virtual ~Joint();

public:
    /// @brief
    virtual void updateKinematics(bool _firstDerivative = true,
                        bool _secondDerivative = true) = 0;

public:
    /// @brief
    JointType getJointType() const { return mJointType; }

    /// @brief
    BodyNode* getParentNode() const { return mParentBody; }

    /// @brief
    BodyNode* getChildNode() const { return mChildBody; }

    /// @brief
    const math::SE3& getLocalTransformation() const { return mT; }

    /// @brief
    const math::se3& getLocalVelocity() const { return mV; }

    /// @brief
    const math::Jacobian& getLocalJacobian() const { return mS; }

    /// @brief
    const math::Jacobian& getLocalJacobianFirstDerivative() const
    { return mdS; }

public:
    /// @brief
    void setParentBody(BodyNode* _body);

    /// @brief
    void setChildBody(BodyNode* _body);

    /// @brief
    void setLocalTransformFromParentBody(const math::SE3& _T);

    /// @brief
    void setLocalTransformFromChildBody(const math::SE3& _T);

    /// @brief
    BodyNode* getParentBody() const { return mParentBody; }

    /// @brief
    BodyNode* getChildBody() const { return mChildBody; }

    /// @brief
    const math::SE3& getLocalTransformationFromParentBody() const
    { return mT_ParentBodyToJoint; }

    /// @brief
    const math::SE3& getLocalTransformationFromChildBody() const
    { return mT_ChildBodyToJoint; }

protected:
    /// @brief Type of joint e.g. ball, hinge etc.
    JointType mJointType;

    /// @brief
    BodyNode* mParentBody;

    /// @brief
    BodyNode* mChildBody;

    /// @brief
    math::SE3 mT_ParentBodyToJoint;

    /// @brief
    math::SE3 mT_ChildBodyToJoint;

protected:
    /// @brief Local transformation.
    math::SE3 mT;

    /// @brief Local generalized body velocity.
    math::se3 mV;

    /// @brief Local Jacobian.
    math::Jacobian mS;

    /// @brief Local generalized body acceleration.
    math::se3 mdV;

    /// @brief Time derivative of local Jacobian.
    math::Jacobian mdS;

    // Eigen3 aligned allocator
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

};

} // namespace kinematics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_JOINT_H

