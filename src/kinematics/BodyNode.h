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

/*! \mainpage Dynamic Animation and Robotics Toolkits

DART is an open source library for developing kinematic and dynamic
applications in robotics and computer animation. DART features two
main components: a multibody dynamic simulator developed by Georgia
Tech Graphics Lab and a variety of motion planning algorithms
developed by Georgia Tech Humanoid Robotics Lab. This document focuses
only on the dynamic simulator.

The multibody dynamics simulator in DART is designed to aid
development of motion control algorithms. DART uses generalized
coordinates to represent articulated rigid body systems and computes
Lagrange’s equations derived from D’Alembert’s principle to describe
the dynamics of motion. In contrast to many popular physics engines
which view the simulator as a black box, DART provides full access to
internal kinematic and dynamic quantities, such as mass matrix,
Coriolis and centrifugal force, transformation matrices and their
derivatives, etc. DART also provides efficient computation of Jacobian
matrices for arbitrary body points and coordinate frames.

The contact and collision are handled using an implicit time-stepping,
velocity-based LCP (linear-complementarity problem) to guarantee
non-penetration, directional friction, and approximated Coulombs
friction cone conditions. The LCP problem is solved efficiently by
Lemke's algorithm. For the collision detection, DART directly uses FCL
package developed by UNC Gamma Lab.

In addition, DART supports various joint types (ball-and-socket,
universal, hinge, and prismatic joints) and arbitrary meshes. DART
also provides two explicit integration methods: first-order
Runge-Kutta and fourth-order Runge Kutta.

*/

#ifndef DART_KINEMATICS_BODYNODE_H
#define DART_KINEMATICS_BODYNODE_H

#include "math/SE3.h"

namespace dart {
namespace kinematics {

class Skeleton;
class Joint;
class Shape;

/// @brief BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
/// Mostly automatically constructed by FileInfoSkel.
/// @see FileInfoSkel.
///
/// [Members]
/// W: world transformation (4x4 matrix)
/// J: world Jacobian (6xn matrix)
/// dJ: world Jacobian derivative (6xn matrix)
/// V: generalized body velocity (6x1 vector)
/// dV: generalized body acceleration (6x1 vector)
/// F: generalized body force (6x1 vector)
/// I: generalized body inertia (6x6 matrix)
class BodyNode
{
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTORS AND DESTRUCTOR
    //--------------------------------------------------------------------------
    /// @brief
    BodyNode();

    /// @brief
    virtual ~BodyNode();

    //--------------------------------------------------------------------------
    // KINEMATICAL PROPERTIES
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // STRUCTURAL PROPERTIES
    //--------------------------------------------------------------------------
    /// @brief
    void setParentJoint(Joint* _joint) { mParentJoint = _joint; }

    /// @brief
    Joint* getParentJoint() const { return mParentJoint; }

    /// @brief
    void addChildJoint(Joint* _joint);

    /// @brief
    Joint* getChildJoint(int _idx) const;

    /// @brief
    const std::vector<Joint*>& getChildJoints() const { return mChildJoints; }

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    const math::SE3& getWorldTransformation() const { return mW; }

    /// @brief
    const math::se3& getBodyVelocity() const { return mV; }

    /// @brief
    const math::se3& getBodyAcceleration() const { return mdV; }

    /// @brief
    void updateWorldTransformation();

    /// @brief
    void updateBodyVelocity();

    /// @brief
    void updateBodyAcceleration();

protected:
    //--------------------------------------------------------------------------
    // Constant Properties
    //--------------------------------------------------------------------------
    /// @brief
    std::vector<Shape*> mVizShapes;

    /// @brief
    std::vector<Shape*> mColShapes;

    //--------------------------------------------------------------------------
    // Structual Properties
    //--------------------------------------------------------------------------
    /// @brief Pointer to the model this body node belongs to.
    Skeleton* mSkeleton;

    /// @brief
    Joint* mParentJoint;

    /// @brief
    std::vector<Joint*> mChildJoints;

    /// @brief
    BodyNode* mParentBody;

    /// @brief
    std::vector<BodyNode*> mChildBodies;

    //--------------------------------------------------------------------------
    // Variable Properties
    //--------------------------------------------------------------------------
    /// @brief World transformation.
    math::SE3 mW;

    /// @brief Generalized body velocity w.r.t. body frame.
    math::se3 mV;

    /// @brief Generalized body acceleration w.r.t. body frame.
    math::se3 mdV;

private:

};

} // namespace kinematics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_BODYNODE_H

