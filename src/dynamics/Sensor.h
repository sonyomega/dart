/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 08/12/2013
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

#ifndef DART_DYNAMICS_SENSOR_H
#define DART_DYNAMICS_SENSOR_H

#include <Eigen/Dense>

namespace dart {

namespace renderer {
class RenderInterface;
}

namespace dynamics {

/// @brief
class Sensor
{
public:
    /// @brief
    enum SensorType {
        P_UNDEFINED,
        ST_RAY
    };

    /// @brief
    Sensor(const std::string& _name = "Sensor");

    /// @brief
    virtual ~Sensor();

    /// @brief
    void setTransform(const Eigen::Isometry3d& _Transform);

    /// @brief
    const Eigen::Isometry3d& getTransform() const;

    /// @brief
    void setOffset(const Eigen::Vector3d& _offset);

    /// @brief
    Eigen::Vector3d getOffset() const;

    /// @brief
    SensorType getSensorType() const;

    /// @brief
    virtual void draw(renderer::RenderInterface* _ri = NULL,
                      const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                      bool _useDefaultColor = true) const {}

protected:
    /// @brief Sensor name
    std::string mName;

    /// @brief Type of primitive; unknown in the general case.
    SensorType mType;

    /// @brief Dimensions for bounding box.
    Eigen::Vector3d mDim;

    /// @brief Volume enclosed by the geometry.
    double mVolume;

    /// @brief The origin of this primitive in the bodynode frame.
    Eigen::Vector3d mOffset;

    /// @brief Local Geometric transformation of the Sensor w.r.t. parent frame.
    Eigen::Isometry3d mTransform;

    /// @brief
    static int mCounter;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_Sensor_H

