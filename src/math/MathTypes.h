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

#ifndef DART_MATH_MATH_TYPES_H
#define DART_MATH_MATH_TYPES_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
// Epsilon definition
#define DART_EPSILON 1.0E-6
#define M_PI		 3.14159265358979323846
#define M_2PI		 6.28318530717958647693		// = 2 * pi
#define M_PI_SQR	 9.86960440108935861883		// = pi^2
#define M_RADIAN	 0.0174532925199432957692	// = pi / 180
#define M_DEGREE	 57.2957795130823208768		// = 18 / pi

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
namespace Eigen {

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > EIGEN_V_VEC2D;
typedef std::vector<Eigen::Vector3d>                                             EIGEN_V_VEC3D;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > EIGEN_V_VEC4D;

typedef std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> > EIGEN_V_MAT2D;
typedef std::vector<Eigen::Matrix3d>                                             EIGEN_V_MAT3D;
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > EIGEN_V_MAT4D;

typedef std::vector<std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > > EIGEN_VV_VEC2D;
typedef std::vector<std::vector<Eigen::Vector3d > >                                            EIGEN_VV_VEC3D;
typedef std::vector<std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > > EIGEN_VV_VEC4D;

typedef std::vector<std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> > > EIGEN_VV_MAT2D;
typedef std::vector<std::vector<Eigen::Matrix3d > >                                            EIGEN_VV_MAT3D;
typedef std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > > EIGEN_VV_MAT4D;

typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond > >              EIGEN_V_QUATD;
typedef std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > > EIGEN_VV_QUATD;

}

namespace dart {
namespace math {

typedef Eigen::Matrix6d Inertia;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> Jacobian;

} // namespace math
} // namespace dart

#endif // #ifndef DART_MATH_MATH_TYPES_H
