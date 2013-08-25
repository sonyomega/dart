/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/24/2013
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

#ifndef DART_UTILS_SKEL_PARSER_H
#define DART_UTILS_SKEL_PARSER_H

#include <vector>
#include <Eigen/Dense>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "math/LieGroup.h"

namespace dart {

namespace dynamics {
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class FreeJoint;
}

namespace dynamics {
class BodyNode;
class Skeleton;
}

namespace simulation {
class World;
}

namespace utils {

class ElementEnumerator;

#define SDF_OK 0
#define SDF_ERROR 1

#define DART_OK 0
#define DART_ERROR 1

#define VSK_OK 0
#define VSK_ERROR 1

std::string toString(bool _v);
std::string toString(int _v);
std::string toString(unsigned int _v);
std::string toString(float _v);
std::string toString(double _v);
std::string toString(char _v);
std::string toString(const Eigen::Vector2d& _v);
std::string toString(const Eigen::Vector3d& _v);
//std::string toString(const math::Vec3& _v);
//std::string toString(const math::so3& _v);
//std::string toString(const math::SO3& _v);
std::string toString(const math::SE3& _v);

bool            toBool(const std::string& _str);
int             toInt(const std::string& _str);
unsigned int    toUInt(const std::string& _str);
float           toFloat(const std::string& _str);
double          toDouble(const std::string& _str);
char            toChar(const std::string& _str);
Eigen::Vector2d toVector2d(const std::string& _str);
Eigen::Vector3d toVector3d(const std::string& _str);
//math::Vec3      toVector3d(const std::string& _str);
//math::so3       toVector3d(const std::string& _str);
//math::SO3       toSO3(const std::string& _str);
math::SE3       toSE3(const std::string& _str);

std::string getValueString(tinyxml2::XMLElement* _parentElement,
                    const std::string& _name);

bool getValueBool(tinyxml2::XMLElement* _parentElement,
                  const std::string& _name);

int getValueInt(tinyxml2::XMLElement* _parentElement,
                const std::string& _name);

unsigned int getValueUInt(tinyxml2::XMLElement* _parentElement,
                          const std::string& _name);

float getValueFloat(tinyxml2::XMLElement* _parentElement,
                    const std::string& _name);

double getValueDouble(tinyxml2::XMLElement* _parentElement,
                      const std::string& _name);

char getValueChar(tinyxml2::XMLElement* _parentElement,
                  const std::string& _name);

Eigen::Vector2d getValueVector2d(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name);

Eigen::Vector3d getValueVector3d(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name);

math::Vec3 getValueVec3(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name);

math::so3 getValueso3(tinyxml2::XMLElement* _parentElement,
                      const std::string& _name);

//math::SO3 getValueSO3(tinyxml2::XMLElement* _parentElement,
//                      const std::string& _name);

math::SE3 getValueSE3(tinyxml2::XMLElement* _parentElement,
                      const std::string& _name);

bool hasElement(tinyxml2::XMLElement* _parentElement,
                const std::string& _name);

tinyxml2::XMLElement* getElement(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name);
//------------------------------------------------------------------------------
// Parsing Helper Functions
//------------------------------------------------------------------------------
/// @brief
simulation::World* readSkelFile(const std::string& _filename);

/// @brief
simulation::World* readWorld(tinyxml2::XMLElement* _worldElement);

/// @brief
dynamics::Skeleton* readSkeleton(tinyxml2::XMLElement* _skeletonElement,
                                 simulation::World* _world);

/// @brief
dynamics::BodyNode* readBodyNode(tinyxml2::XMLElement* _bodyElement,
                                     dynamics::Skeleton* _skeleton);

/// @brief
dynamics::Joint* readJoint(tinyxml2::XMLElement* _jointElement,
                             dynamics::Skeleton* _skeleton);

/// @brief
dynamics::PrismaticJoint* readPrismaticJoint(
        tinyxml2::XMLElement* _prismaticJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::RevoluteJoint* readRevoluteJoint(
        tinyxml2::XMLElement* _revoluteJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::UniversalJoint* readUniversalJoint(
        tinyxml2::XMLElement* _universalJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::BallJoint* readBallJoint(
        tinyxml2::XMLElement* _ballJointElement,
        dynamics::Skeleton* _skeleton);

dart::dynamics::EulerJoint *readEulerJoint(
        tinyxml2::XMLElement* _eulerJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::TranslationalJoint* readTranslationalJoint(
        tinyxml2::XMLElement* _translationalJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::FreeJoint* readFreeJoint(
        tinyxml2::XMLElement* _freeJointElement,
        dynamics::Skeleton* _skeleton);

dart::dynamics::WeldJoint* readWeldJoint(
        tinyxml2::XMLElement* _weldJointElement,
        dynamics::Skeleton* _skeleton);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
/// @brief
class ElementEnumerator
{
public:
    /// @brief
    ElementEnumerator(tinyxml2::XMLElement* _parent, const char* const _name);

    /// @brief
    ~ElementEnumerator();

    /// @brief
    bool valid() const;

    /// @brief
    bool next();

    /// @brief
    tinyxml2::XMLElement* get() const { return m_current; }

    /// @brief
    tinyxml2::XMLElement* operator->() const { return m_current; }

    /// @brief
    tinyxml2::XMLElement& operator*() const { return *m_current; }

    /// @brief
    bool operator==(const ElementEnumerator& _rhs) const;

    /// @brief
    ElementEnumerator & operator=(const ElementEnumerator& _rhs);

private:
    /// @brief
    std::string m_name;

    /// @brief
    tinyxml2::XMLElement* m_parent;

    /// @brief
    tinyxml2::XMLElement* m_current;
};

} // namespace utils
} // namespace dart

#endif // DART_UTILS_SKEL_PARSER_H
