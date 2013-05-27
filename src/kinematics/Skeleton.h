/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
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

#ifndef DART_KINEMATICS_SKELETON_H
#define DART_KINEMATICS_SKELETON_H

#include <vector>
#include <Eigen/Dense>

#include "utils/Deprecated.h"
#include "math/Jacobian.h"
#include "kinematics/System.h"

namespace dart {
namespace renderer { class RenderInterface; }
namespace kinematics {

class BodyNode;
class Joint;

/// @brief
class Skeleton : public System
{
public:
    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// @brief
    Skeleton();

    /// @brief
    virtual ~Skeleton();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void initKinematics();

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
    void setSelfCollidable(bool _selfCollidable)
    { mSelfCollidable = _selfCollidable; }

    /// @brief
    bool getSelfCollidable() const { return mSelfCollidable; }


    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    void addBody(BodyNode* _body, bool _addParentJoint = true);

    /// @brief
    void addJoint(Joint* _joint);

    /// @brief
    //void setRootBody(BodyNode* _body) { mRootBody = _body; }

    /// @brief
    // TODO: rename
    //int getNumBodies() const { return mBodies.size(); }
    int getNumNodes() const { return mBodies.size(); }

    /// @brief
    int getNumJoints() const { return mJoints.size(); }

    /// @brief
    // TODO: rename
    //BodyNode* getBody(int _idx) const { return mBodies[_idx]; }
    BodyNode* getNode(int _idx) const { return mBodies[_idx]; }

    /// @brief
    // TODO: rename
    //BodyNode* findBody(const std::string& _name) const;
    BodyNode* findBody(const std::string& _name) const;

    //--------------------------------------------------------------------------
    // Recursive Kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    /// @todo Use set_q(const Eigen::VectorXd& _state) instead.
    DEPRECATED void setPose(const Eigen::VectorXd& _pose,
                            bool bCalcTrans = true,
                            bool bCalcDeriv = true);

    /// @brief
    /// @todo Use get_q() instead.
    DEPRECATED Eigen::VectorXd getPose() const { return get_q(); }

    /// @brief
    /// @todo Use get_dq() instead.
    DEPRECATED Eigen::VectorXd getPoseVelocity() const { return get_dq(); }

    /// @brief (q, dq, ddq) --> (W, V, dV)
    void updateForwardKinematics(bool _firstDerivative = true,
                                 bool _secondDerivative = true);

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    math::Jacobian getJacobian() const;

    //--------------------------------------------------------------------------
    // Rendering
    //--------------------------------------------------------------------------
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true) const;

protected:
    //--------------------------------------------------------------------------
    // Sub-functions for Recursive Algorithms
    //--------------------------------------------------------------------------
    /// @brief Update joint kinematics (T, S, V, dS, dV)
    void _updateJointKinematics(bool _firstDerivative = true,
                                     bool _secondDerivative = true);

    /// @brief Update body kinematics (W, V, dV)
    void _updateBodyForwardKinematics(bool _firstDerivative = true,
                                      bool _secondDerivative = true);


protected:
    /// @brief
    std::string mName;

    /// @brief
    bool mSelfCollidable;

protected:
    //--------------------------------------------------------------------------
    // Structual Properties
    //--------------------------------------------------------------------------
    /// @brief
    // TODO: rename
    //BodyNode* mRootBody;
    BodyNode* mRoot;

    /// @brief
    std::vector<BodyNode*> mBodies;

    /// @brief
    std::vector<Joint*> mJoints;



private:
};

} // namespace kinematics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_SKELETON_H

