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

#include "math/LieGroup.h"
#include "dynamics/System.h"

namespace dart {
namespace renderer { class RenderInterface; }
namespace dynamics {

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
class Joint : public GenCoordSystem
{
public:
    //--------------------------------------------------------------------------
    // Types
    //--------------------------------------------------------------------------
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
        BALL,          // 3 dof
        BALL_EULER,
        FREE           // 6 dof
    };

    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// @brief
    Joint(const std::string& _name = "");

    /// @brief
    virtual ~Joint();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setName(const std::string& _name) { mName = _name; }

    /// @brief
    const std::string& getName() const { return mName; }

    //--------------------------------------------------------------------------
    // Kinematical Properties
    //--------------------------------------------------------------------------
    /// @brief
    JointType getJointType() const { return mJointType; }

    /// @brief
    const math::SE3& getLocalTransformation() const { return mT; }

    /// @brief
    const math::Jacobian& getLocalJacobian() const { return mS; }

    /// @brief
    const math::se3& getLocalVelocity() const { return mV; }

    /// @brief
    const math::Jacobian& getLocalJacobianFirstDerivative() const
    { return mdS; }

    /// @brief
    const math::se3& getLocalAcceleration() const { return mdV; }

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setSkelIndex(int _idx){mSkelIndex= _idx;}

    /// @brief
    int getSkelIndex() const {return mSkelIndex;}

    /// @brief
    void setParentBody(BodyNode* _body);

    /// @brief
    void setChildBody(BodyNode* _body);

    /// @brief
    void setLocalTransformFromParentBody(const math::SE3& _T);

    /// @brief
    void setLocalTransformFromChildBody(const math::SE3& _T);

    /// @brief
    DEPRECATED BodyNode* getParentNode() const { return mParentBody; }
    BodyNode* getParentBody() const { return mParentBody; }

    /// @brief
    DEPRECATED BodyNode* getChildNode() const { return mChildBody; }
    BodyNode* getChildBody() const { return mChildBody; }

    /// @brief
    const math::SE3& getLocalTransformationFromParentBody() const
    { return mT_ParentBodyToJoint; }

    /// @brief
    const math::SE3& getLocalTransformationFromChildBody() const
    { return mT_ChildBodyToJoint; }

    // TODO: Not implemented.
    /// @brief
    virtual double getPotentialEnergy() const = 0;

    //--------------------------------------------------------------------------
    // Recursive Kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    void updateKinematics(bool _firstDerivative = true,
                          bool _secondDerivative = true);


    //void updateGlobalKinematics();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    void applyGLTransform(renderer::RenderInterface* _ri);

protected:
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    /// q --> T(q)
    virtual void _updateTransformation() = 0;

    /// @brief
    /// q, dq --> S(q), V(q, dq)
    /// V(q, dq) = S(q) * dq
    virtual void _updateVelocity() = 0;

    /// @brief
    /// dq, ddq, S(q) --> dS(q), dV(q, dq, ddq)
    /// dV(q, dq, ddq) = dS(q) * dq + S(q) * ddq
    virtual void _updateAcceleration() = 0;

    // TODO:
    /// @brief
    //virtual void _updateDampingForce() = 0;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    std::string mName;

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief Unique dof id in skeleton
    int mSkelIndex;

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

    //--------------------------------------------------------------------------
    // Kinematics variables
    //--------------------------------------------------------------------------
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

    //--------------------------------------------------------------------------
    // Dynamics variables
    //--------------------------------------------------------------------------
public:
    /// @brief
    std::vector<double> mDampingCoefficient;

    /// @brief
    void setDampingCoefficient(int _idx, double _d)
    {
        assert(0 <= _idx && _idx < getNumDofs());
        assert(_d >= 0.0);

        mDampingCoefficient[_idx] = _d;
    }

    /// @brief
    double getDampingCoefficient(int _idx) const
    {
        assert(0 <= _idx && _idx < getNumDofs());

        return mDampingCoefficient[_idx];
    }


    Eigen::VectorXd getDampingForce() const
    {
        int numDofs = getNumDofs();
        Eigen::VectorXd dampingForce(numDofs);

        for (int i = 0; i < numDofs; ++i)
            dampingForce(i) = -mDampingCoefficient[i] * getDof(i)->get_dq();

        return dampingForce;
    }

    /// @brief
    std::vector<double> mSpringStiffness;

private:

};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_JOINT_H

