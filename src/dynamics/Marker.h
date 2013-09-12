/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
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

#ifndef DART_DYNAMICS_MARKER_H
#define DART_DYNAMICS_MARKER_H

#include <Eigen/Dense>

namespace dart {

namespace renderer {
class RenderInterface;
}

namespace dynamics {

#define MAX_MARKER_NAME 256

class BodyNode;

class Marker
{
public:
    enum ConstraintType
    {
        NO,
        HARD,
        SOFT
    };
    
    /// @brief
    Marker(const char* _name, Eigen::Vector3d& _offset, BodyNode* _node,
           ConstraintType _type = NO);

    /// @brief
    virtual ~Marker();

    /// @brief
    void draw(renderer::RenderInterface* _ri = NULL, bool _offset = true,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Identity(),
              bool _useDefaultColor = true) const;

    /// @brief
    Eigen::Vector3d getWorldCoords(); ///< get the world coordinates of mOffset

    /// @brief
    Eigen::Vector3d getLocalCoords() const;

    /// @brief
    void setLocalCoords(Eigen::Vector3d& _offset);

    /// @brief
    int getSkeletonIndex() const;

    /// @brief
    void setSkeletonIndex(int _idx);

    /// @brief
    int getID() const;

    /// @brief
    BodyNode* getNode() const;

    /// @brief
    const char* getName() const;

    // useful for IK
    /// @brief
    ConstraintType getConstraintType() const;

    /// @brief
    void setConstraintType(ConstraintType _type);
    
    
protected:
    /// @brief body link associated with.
    BodyNode* mNode;

    /// @brief local coordinates in the links.
    Eigen::Vector3d mOffset;

    /// @brief name of this marker, max length 256 characters.
    char mName[MAX_MARKER_NAME];

    /// @brief position in the model class marker vector.
    int mSkelIndex;

    /// @brief type of constraint.
    ConstraintType mType;
    
private:
    /// @brief a unique ID of this marker globally.
    int mID;

    /// @brief counts the number of markers globally.
    static int msMarkerCount;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace kinematics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_MARKER_H

