/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/06/2013
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

#ifndef DART_MATH_LIE_GROUPS_H
#define DART_MATH_LIE_GROUPS_H

#include <Eigen/Dense>
#include "math/UtilsMath.h"
#include "math/SE3.h"

// TODO: Let's use noalias().
// TODO: Not using Eigen?

namespace math
{

// TODO: Is here right place?
typedef Matrix<double, 6, 1> Vector6d;

class Inertia; // Inertia with cog offset (6x6 matrix)
class so3;
class SO3; // Special orthogonal group (3x3 rotation matrix)
class se3;
class dse3;
class SE3; // Special Euclidean group (4x4 transformation matrix)
class TSE3;


//==============================================================================
class Jacobian
{
public:
    // Aligned allocator for Eigen member variable.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /// @brief
    Jacobian();
    
    /// @brief
    explicit Jacobian(unsigned int _size);

    /// @brief
    explicit Jacobian(const Eigen::MatrixXd& _J);

    /// @brief
    virtual ~Jacobian();
    
public:
    se3 operator* (const Eigen::VectorXd& _qdot);

public:
    /// @brief
    void setSize(int _size) { mJ.resize(_size); }
    
    /// @brief
    unsigned int getSize() const { return mJ.size(); }
    
    /// @brief
    void setMatrix(const Eigen::MatrixXd& _J);
    
    /// @brief
    Eigen::MatrixXd getMatrix() const;
    
    /// @brief
    void setColumn(int _idx, const se3& _J) { mJ[_idx] = _J; }
    
    /// @brief
    se3 getColumn(int _idx);

    /// @brief
    Jacobian getColumns(int _idx, int _size);
    
    /// @brief
    void setLinear(int _idx, Eigen::Vector3d& _Jv) { mJ[_idx].setLinear(_Jv); }
    
    /// @brief
    Eigen::Vector3d getLinear(int _idx) const { return mJ[_idx].getLinear(); }
    
    /// @brief
    void setAngular(int _idx, so3& _Jw) { mJ[_idx].setAngular(_Jw); }
    
    /// @brief
    so3 getAngular(int _idx) const { return mJ[_idx].getAngular(); }
    
protected:
    /// @brief
    std::vector<se3> mJ;
    
private:
};

} // namespace math

#endif // DART_MATH_LIE_GROUPS_H
