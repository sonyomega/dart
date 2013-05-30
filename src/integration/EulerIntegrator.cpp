/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Kristin Siu <kasiu@gatech.edu>
 * Date: 09/16/2011
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

#include "kinematics/System.h"
#include "integration/EulerIntegrator.h"

using namespace Eigen;

namespace dart {
namespace integration {

EulerIntegrator::EulerIntegrator()
    : Integrator()
{
}

EulerIntegrator::~EulerIntegrator()
{
}

void EulerIntegrator::integrate(IntegrableSystem* system, double dt) const
{
    // Explicit Euler Method
    VectorXd deriv = system->evalDeriv();
    system->setState(system->getState() + (dt * deriv));

    // Semi-implicit Euler Method
//    int n = system->getState().size() / 2;
//    VectorXd deriv = system->evalDeriv();
//    VectorXd newState = system->getState();

//    newState.segment(n,n) += dt * deriv.segment(n,n);
//    newState.segment(0,n) += dt * newState.segment(n,n);

//    system->setState(newState);
}

//void EulerIntegrator::integrate(kinematics::System* _system, double _dt) const
//{
//    //--------------------------------------------------------------------------
//    // Explicit Euler Method
//    _system->set_q(_system->get_q() + _dt * _system->get_q());
//    _system->set_dq(_system->get_dq() + _dt * _system->get_ddq());

//    //--------------------------------------------------------------------------
//    // Semi-implicit Euler Method
//    _system->set_dq(_system->get_dq() + _dt * _system->get_ddq());
//    _system->set_q(_system->get_q() + _dt * _system->get_dq());

//    //--------------------------------------------------------------------------
//    // Euler-Verlet Method
//    _system->set_dq(_system->get_dq() + _dt * _dt * _system->get_ddq());
//    _system->set_q(_system->get_q() + _system->get_dq());
//}

} // namespace integration
} // namespace dart
