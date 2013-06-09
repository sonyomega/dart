/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_KINEMATICS_SYSTEM_H
#define DART_KINEMATICS_SYSTEM_H

#include <vector>
#include <Eigen/Dense>

#include "dynamics/Dof.h"

namespace dart {
namespace dynamics {

class GeneralizedCoordinate
{
public:
    GeneralizedCoordinate();
    virtual ~GeneralizedCoordinate();

protected:

private:

};

class ScalarCoordinate : public GeneralizedCoordinate
{
public:
    ScalarCoordinate();
    virtual ~ScalarCoordinate();

protected:
    double q;
    double dq;
    double ddq;
    double tau;

private:

};

class SO3Coordinate
{
public:
    SO3Coordinate();
    virtual ~SO3Coordinate();

protected:
    std::vector<math::SO3> R;
    std::vector<math::so3> w;
    std::vector<math::so3> dw;
    std::vector<math::dso3> m;

private:

};

class SE3Coordinate
{
public:
    SE3Coordinate();
    virtual ~SE3Coordinate();

protected:
    std::vector<math::SE3> T;
    std::vector<math::se3> V;
    std::vector<math::se3> dV;
    std::vector<math::dse3> F;

private:

};

/// @brief Generalized configuration of a system
class SystemConfiguration
{
public:
    SystemConfiguration();
    virtual ~SystemConfiguration();

    int getDof() const;

    std::vector<double> theta;
    std::vector<math::SO3> R;
    std::vector<math::SE3> T;

protected:

private:
};

/// @brief Generalized velocity of a system
class SystemVelocity
{
public:
    SystemVelocity();
    virtual ~SystemVelocity();

protected:
    std::vector<double> theta;
    std::vector<math::so3> w;
    std::vector<math::se3> V;

private:
};

typedef SystemVelocity SystemAcceleration;

class SystemForce
{
public:
    SystemForce();
    virtual ~SystemForce();

protected:
    std::vector<double> tau;
    std::vector<math::so3> m;
    std::vector<math::se3> F;

private:
};

/// @brief System is a base class for every classes that has Dofs.
class GenCoordSystem
{
public:
    /// @brief
    GenCoordSystem();

    /// @brief
    virtual ~GenCoordSystem();

    /// @brief
    DEPRECATED int getNumDofs() const { return mDofs.size(); }
    int getDOF() const { return mDofs.size(); }

    /// @brief
    const std::vector<GenCoord*>& getDofs() const { return mDofs; }

    /// @brief
    GenCoord* getDof(int _idx) const;

    /// @brief
    GenCoord* getDof(const std::string& _name) const;

    /// @brief Backup current state as initial state.
    void backupInitState();

    /// @brief Restore the stored initial state.
    void restoreInitState();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    void set_q(const Eigen::VectorXd& _q);     ///< Set generalized coordinate vector
    void set_dq(const Eigen::VectorXd& _dq);   ///< Set generalized velocity vector
    void set_ddq(const Eigen::VectorXd& _ddq); ///< Set generalized acceleration vector
    void set_tau(const Eigen::VectorXd& _tau); ///< Set generalized force vector (internal forces)

    void set_qMin(const Eigen::VectorXd& _qMin);
    void set_dqMin(const Eigen::VectorXd& _dqMin);
    void set_ddqMin(const Eigen::VectorXd& _ddqMin);
    void set_tauMin(const Eigen::VectorXd& _tauMin);

    void set_qMax(const Eigen::VectorXd& _qMax);
    void set_dqMax(const Eigen::VectorXd& _dqMax);
    void set_ddqMax(const Eigen::VectorXd& _ddqMax);
    void set_tauMax(const Eigen::VectorXd& _tauMax);

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    Eigen::VectorXd get_q() const;
    Eigen::VectorXd get_dq() const;
    Eigen::VectorXd get_ddq() const;
    Eigen::VectorXd get_tau() const;

    Eigen::VectorXd get_qMin() const;
    Eigen::VectorXd get_dqMin() const;
    Eigen::VectorXd get_ddqMin() const;
    Eigen::VectorXd get_tauMin() const;

    Eigen::VectorXd get_qMax() const;
    Eigen::VectorXd get_dqMax() const;
    Eigen::VectorXd get_ddqMax() const;
    Eigen::VectorXd get_tauMax() const;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
//    /// @brief
//    void setState(const Eigen::VectorXd& _state);

//    /// @brief
//    Eigen::VectorXd getState() const;

    SystemConfiguration getConfiguration() const;

protected:
    /// @brief Pointers to Dofs.
    std::vector<GenCoord*> mDofs;

    std::vector<GeneralizedCoordinate*> mGeneralizedCoordinates;

private:

};

} // namespace dynamics
} // namespace dart

#endif // DART_KINEMATICS_SYSTEM_H
