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

#ifndef DART_KINEMATICS_DOF_H
#define DART_KINEMATICS_DOF_H

#include <cstring>
#include <limits>

namespace kinematics
{
#define MAX_DOF_NAME 128

class Joint;
class Transformation;

class Dof
{
public:
    Dof();
    Dof(double _val);
    Dof(double _val, double _min, double _max);
    Dof(double _val, const char* _name);
    Dof(double _val, const char* _name, double _min, double _max);

    virtual ~Dof() {}

    // some helper functions
    void setName(char* _n) { strcpy(mName, _n); }
    inline char* getName() { return mName; }

    void setValue(double _v);

    inline double getValue() const { return m_q; }

    inline double getMin() const { return mMin_q; }
    inline double getMax() const { return mMax_q; }
    inline void setMin(double _min) { mMin_q = _min; }
    inline void setMax(double _max) { mMax_q = _max; }

    inline int getSkelIndex() const { return mSkelIndex; }
    inline void setSkelIndex(int _idx) { mSkelIndex = _idx; }

    inline bool isVariable() const { return mVariable; }
    inline void setVariable() { mVariable = true; }

    inline void setTrans(Transformation *_t){ mTrans = _t; }
    inline Transformation* getTrans() const{ return mTrans; }

    inline void setJoint(Joint *_j) { mJoint = _j; }
    inline Joint *getJoint() const { return mJoint; }

    inline void set_q(double _q) { m_q = _q; }
    inline void set_dq(double _dq) { m_dq = _dq; }
    inline void set_ddq(double _ddq) { m_ddq = _ddq; }
    inline void setTorque(double _tau) { mTorque = _tau; }

    inline double get_q(void) const { return m_q; }
    inline double get_dq(void) const { return m_dq; }
    inline double get_ddq(void) const { return m_ddq; }
    inline double getTorque(void) const { return mTorque; }

    inline void setMin_q(double _min_q) { mMin_q = _min_q; }
    inline void setMin_dq(double _min_dq) { mMin_dq = _min_dq; }
    inline void setMin_ddq(double _min_ddq) { mMin_ddq = _min_ddq; }
    inline void setMinTorque(double _min_tau) { mMinTorque = _min_tau; }

    inline double getMin_q(void) const { return mMin_q; }
    inline double getMin_dq(void) const { return mMin_dq; }
    inline double getMin_ddq(void) const { return mMin_ddq; }
    inline double getMinTorque(void) const { return mMinTorque; }

    inline void setMax_q(double _max_q) { mMax_q = _max_q; }
    inline void setMax__dq(double _max_dq) { mMax_dq = _max_dq; }
    inline void setMax__ddq(double _max_ddq) { mMax_ddq = _max_ddq; }
    inline void setMaxTorque(double _max_tau) { mMaxTorque = _max_tau; }

    inline double getMax_q(void) const { return mMax_q; }
    inline double getMax_dq(void) const { return mMax_dq; }
    inline double getMax_ddq(void) const { return mMax_ddq; }
    inline double getMaxTorque(void) const { return mMaxTorque; }

protected:
    void _init(double _v = 0.0,
              const char* _name = "Unknown Dof",
              double _min = -std::numeric_limits<double>::infinity(),
              double _max = std::numeric_limits<double>::infinity());

    double m_q;     // Value of the joint angle
    double mMin_q;	// Min value allowed
    double mMax_q;	// Max value allowed

    double m_dq;
    double mMin_dq;
    double mMax_dq;

    double m_ddq;
    double mMin_ddq;
    double mMax_ddq;

    double mTorque;
    double mMinTorque;
    double mMaxTorque;

    char mName[MAX_DOF_NAME];
    int mSkelIndex; // Unique to dof in model

    Transformation* mTrans;	// Transformation associated with
    Joint* mJoint;	// Joint to which it belongs

    bool mVariable;	// True when it is a variable and included int he model
};
} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_DOF_H

