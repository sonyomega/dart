/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/05/2013
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

#include "math/Jacobian.h"
#include "utils/UtilsCode.h"

namespace dart {
namespace math {

#define LIEGROUP_EPS 10e-9

//==============================================================================
Jacobian::Jacobian()
{
}

Jacobian::Jacobian(unsigned int _size)
{
    mJ.resize(_size);
}

Jacobian::Jacobian(const Eigen::MatrixXd& _J)
{
    setMatrix(_J);
}

Jacobian::~Jacobian()
{
}

void Jacobian::setMatrix(const Eigen::MatrixXd& _J)
{
    assert(_J.rows() == 6);

    setSize(_J.cols());

    for (int i = 0; i < getSize(); ++i)
        mJ[i].setVector(_J.col(i));
}

Eigen::MatrixXd Jacobian::getMatrix() const
{
    Eigen::MatrixXd J(6, getSize());

    for (int i = 0; i < getSize(); i++)
        J.col(i) = mJ[i].getVector();

    return J;
}

void Jacobian::setZero()
{
    for (int i = 0; i < getSize(); i++)
        mJ[i].setZero();
}

se3 Jacobian::getColumn(int _idx)
{
    return mJ[_idx];
}

Jacobian Jacobian::getColumns(int _idx, int _size)
{
    // TODO: NEED TEST
    assert(0 <= _idx);
    assert(_idx + _size <= mJ.size());

    Jacobian J(_size);

    for (int i = 0; i < _size; ++i)
        J.setColumn(i, getColumn(i));

    return J;
}

Jacobian Jacobian::getAdjointed(const SE3& _T) const
{
    Jacobian ret;

    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

Jacobian Jacobian::getAdjointedInv(const SE3& _Tinv) const
{
    Jacobian ret;

    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

se3 Jacobian::operator*(const Eigen::VectorXd& _qdot)
{
    assert(_qdot.size() == getSize());

    se3 ret;

    for (int i = 0; i < getSize(); ++i)
        ret = mJ[i] * _qdot(i);

    return ret;
}

Eigen::VectorXd Jacobian::getInnerProduct(const dse3& _F) const
{
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(getSize());

    assert(ret.size() == getSize());

    int size = getSize();
    for (int i = 0; i < size; ++i)
        ret(i) = mJ[i].innerProduct(_F);

    return ret;
}

} // namespace math
} // namespace dart

