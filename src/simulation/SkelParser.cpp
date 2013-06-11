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


// Standard Library
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

// Local Files
#include "utils/UtilsCode.h"
#include "dynamics/BodyNode.h"
#include "dynamics/ShapeBox.h"
#include "dynamics/ShapeCylinder.h"
#include "dynamics/ShapeEllipsoid.h"
#include "dynamics/WeldJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/TranslationalJoint.h"
#include "dynamics/BallJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"
#include "simulation/SkelParser.h"

namespace dart {
namespace simulation {

//==============================================================================
//
//==============================================================================
ElementEnumerator::ElementEnumerator(tinyxml2::XMLElement* _parent,
                                     const char* const _name)
    : m_name(_name),
      m_parent(_parent),
      m_current(NULL)
{
}

ElementEnumerator::~ElementEnumerator()
{
}

bool ElementEnumerator::valid() const
{
    return m_current != NULL;
}

bool ElementEnumerator::next()
{
    if(!m_parent)
        return false;

    if(m_current)
        m_current = m_current->NextSiblingElement(m_name.c_str());
    else
        m_current = m_parent->FirstChildElement(m_name.c_str());

    if(!valid())
        m_parent = NULL;

    return valid();
}

bool ElementEnumerator::operator==(const ElementEnumerator& _rhs) const
{
    // If they point at the same node, then the names must match
    return (this->m_parent == _rhs.m_parent) &&
           (this->m_current == _rhs.m_current) &&
           (this->m_current != 0 || (this->m_name == _rhs.m_name));
}

ElementEnumerator& ElementEnumerator::operator=(const ElementEnumerator& _rhs)
{
    this->m_name = _rhs.m_name;
    this->m_parent = _rhs.m_parent;
    this->m_current = _rhs.m_current;
}

//==============================================================================
//
//==============================================================================
static void openXMLFile(tinyxml2::XMLDocument& doc,
                        const char* const filename)
{
    int const result = doc.LoadFile(filename);
    switch(result)
    {
        case tinyxml2::XML_SUCCESS:
            break;
        case tinyxml2::XML_ERROR_FILE_NOT_FOUND:
            throw std::runtime_error("File not found");
        case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
            throw std::runtime_error("File not found");
        default:
        {
            std::ostringstream oss;
            oss << "Parse error = " << result;
            throw std::runtime_error(oss.str());
        }
    };
}

static std::string getAttribute(tinyxml2::XMLElement * element,
                                const char* const name)
{
    const char* const result = element->Attribute(name);
    if( result == 0 )
    {
        std::ostringstream oss;
        oss << "Missing attribute " << name << " on " << element->Name();
        throw std::runtime_error(oss.str());
    }
    return std::string(result);
}

static void getAttribute(tinyxml2::XMLElement* element,
                         const char* const name,
                         double* d)
{
    int result = element->QueryDoubleAttribute(name, d);
    if( result != tinyxml2::XML_NO_ERROR )
    {
        std::ostringstream oss;
        oss << "Error parsing double attribute " << name << " on " << element->Name();
        throw std::runtime_error(oss.str());
    }
}

simulation::World* readSkelFile(const std::string& _filename)
{
    //--------------------------------------------------------------------------
    // Load xml and create Document
    tinyxml2::XMLDocument _dartFile;
    try
    {
        openXMLFile(_dartFile, _filename.c_str());
    }
    catch(std::exception const& e)
    {
        std::cout << "LoadFile Fails: " << e.what() << std::endl;
        return NULL;
    }

    //--------------------------------------------------------------------------
    // Load DART
    tinyxml2::XMLElement* dartElement = NULL;
    dartElement = _dartFile.FirstChildElement("skel");
    if (dartElement == NULL)
        return NULL;

    //--------------------------------------------------------------------------
    // Load World
    tinyxml2::XMLElement* worldElement = NULL;
    worldElement = dartElement->FirstChildElement("world");
    if (worldElement == NULL)
        return NULL;

    simulation::World* newWorld = readWorld(worldElement);

    return newWorld;
}

simulation::World* readWorld(tinyxml2::XMLElement* _worldElement)
{
    assert(_worldElement != NULL);

    // Create a world
    simulation::World* newWorld = new simulation::World;

    //--------------------------------------------------------------------------
    // Load physics
    tinyxml2::XMLElement* physicsElement = NULL;
    physicsElement = _worldElement->FirstChildElement("physics");
    if (physicsElement != NULL)
    {
        // Time step
        tinyxml2::XMLElement* timeStepElement = NULL;
        timeStepElement = physicsElement->FirstChildElement("time_step");
        if (timeStepElement != NULL)
        {
            std::string strTimeStep = timeStepElement->GetText();
            double timeStep = toDouble(strTimeStep);
            newWorld->setTimeStep(timeStep);
        }

        // Gravity
        tinyxml2::XMLElement* gravityElement = NULL;
        gravityElement = physicsElement->FirstChildElement("gravity");
        if (gravityElement != NULL)
        {
            std::string strGravity = gravityElement->GetText();
            Eigen::Vector3d gravity = toVector3d(strGravity);
            newWorld->setGravity(gravity);
        }
    }

    //--------------------------------------------------------------------------
    // Load skeletons
    ElementEnumerator skeletonElements(_worldElement, "skeleton");
    while (skeletonElements.next())
    {
        dynamics::Skeleton* newSkeleton
                = readSkeleton(skeletonElements.get(), newWorld);

        newWorld->addSkeleton(newSkeleton);
    }

    return newWorld;
}

dynamics::Skeleton* readSkeleton(tinyxml2::XMLElement* _skeletonElement,
                                 World* _world)
{
    assert(_skeletonElement != NULL);
    assert(_world != NULL);

    dynamics::Skeleton* newSkeleton = new dynamics::Skeleton;

    //--------------------------------------------------------------------------
    // Name attribute
    std::string name = getAttribute(_skeletonElement, "name");
    newSkeleton->setName(name);

    //--------------------------------------------------------------------------
    // transformation
    if (hasElement(_skeletonElement, "transformation"))
    {
        math::SE3 W = getValueSE3(_skeletonElement, "transformation");
        newSkeleton->setWorldTransformation(W, false);
    }

    //--------------------------------------------------------------------------
    // immobile attribute
    tinyxml2::XMLElement* immobileElement = NULL;
    immobileElement = _skeletonElement->FirstChildElement("immobile");
    if (immobileElement != NULL)
    {
        std::string stdImmobile = immobileElement->GetText();
        bool immobile = toBool(stdImmobile);
        newSkeleton->setImmobileState(immobile);
    }

    //--------------------------------------------------------------------------
    // Bodies
    ElementEnumerator bodies(_skeletonElement, "body");
    while (bodies.next())
    {
        dynamics::BodyNode* newBody
                = readBody(bodies.get(), newSkeleton);

        newSkeleton->addBody(newBody, false);
    }

    //--------------------------------------------------------------------------
    // Joints
    ElementEnumerator joints(_skeletonElement, "joint");
    while (joints.next())
    {
        dynamics::Joint* newJoint
                = readJoint(joints.get(), newSkeleton);

        newSkeleton->addJoint(newJoint);
    }

    return newSkeleton;
}

dynamics::BodyNode* readBody(tinyxml2::XMLElement* _bodyElement,
                                     dynamics::Skeleton* _skeleton)
{
    assert(_bodyElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BodyNode* newBody = new dynamics::BodyNode;

    // Name attribute
    std::string name = getAttribute(_bodyElement, "name");
    newBody->setName(name);

    //--------------------------------------------------------------------------
    // gravity
    if (hasElement(_bodyElement, "gravity"))
    {
        bool gravityMode = getValueBool(_bodyElement, "gravity");
        newBody->setGravityMode(gravityMode);
    }

    //--------------------------------------------------------------------------
    // self_collide
//    if (hasElement(_bodyElement, "self_collide"))
//    {
//        bool gravityMode = getValueBool(_bodyElement, "self_collide");
//    }

    //--------------------------------------------------------------------------
    // transformation
    if (hasElement(_bodyElement, "transformation"))
    {
        math::SE3 W = getValueSE3(_bodyElement, "transformation");
        newBody->setWorldTransformation(_skeleton->getWorldTransformation() * W);
    }

    // visualization_shape
    if (hasElement(_bodyElement, "visualization_shape"))
    {
        tinyxml2::XMLElement* vizElement
                = getElement(_bodyElement, "visualization_shape");

        dynamics::Shape* shape = NULL;

        // type
        assert(hasElement(vizElement, "geometry"));
        tinyxml2::XMLElement* geometryElement = getElement(vizElement, "geometry");

        // FIXME: Assume that type has only one shape type.
        if (hasElement(geometryElement, "box"))
        {
            tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

            Eigen::Vector3d size = getValueVector3d(boxElement, "size");

            shape = new dynamics::ShapeBox(size);
        }
        else if (hasElement(geometryElement, "ellipsoid"))
        {
            tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "ellipsoid");

            Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

            shape = new dynamics::ShapeEllipsoid(size);
        }
        else if (hasElement(geometryElement, "cylinder"))
        {
            tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

            double radius = getValueDouble(cylinderElement, "radius");
            double height = getValueDouble(cylinderElement, "height");

            shape = new dynamics::ShapeCylinder(radius, height);
        }
        else
        {
            dterr << "Unknown visualization shape.\n";
            assert(0);
        }
        newBody->setVisualizationShape(shape);

        // transformation
        if (hasElement(vizElement, "transformation"))
        {
            math::SE3 W = getValueSE3(vizElement, "transformation");
            Eigen::Affine3d AF;
            AF.matrix() = W.getEigenMatrix();
            shape->setTransform(AF);
        }

    }

    // collision_shape
    if (hasElement(_bodyElement, "collision_shape"))
    {
        tinyxml2::XMLElement* colElement
                = getElement(_bodyElement, "collision_shape");

        dynamics::Shape* shape = NULL;

        // type
        assert(hasElement(colElement, "geometry"));
        tinyxml2::XMLElement* geometryElement = getElement(colElement, "geometry");

        // FIXME: Assume that type has only one shape type.
        if (hasElement(geometryElement, "box"))
        {
            tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

            Eigen::Vector3d size = getValueVector3d(boxElement, "size");

            shape = new dynamics::ShapeBox(size);
        }
        else if (hasElement(geometryElement, "ellipsoid"))
        {
            tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "ellipsoid");

            Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

            shape = new dynamics::ShapeEllipsoid(size);
        }
        else if (hasElement(geometryElement, "cylinder"))
        {
            tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

            double radius = getValueDouble(cylinderElement, "radius");
            double height = getValueDouble(cylinderElement, "height");

            shape = new dynamics::ShapeCylinder(radius, height);
        }
        else
        {
            dterr << "Unknown visualization shape.\n";
            assert(0);
        }
        newBody->setCollisionShape(shape);

        // transformation
        if (hasElement(colElement, "transformation"))
        {
            math::SE3 W = getValueSE3(colElement, "transformation");
            Eigen::Affine3d AF;
            AF.matrix() = W.getEigenMatrix();
            shape->setTransform(AF);
        }
    }

    //--------------------------------------------------------------------------
    // inertia
    if (hasElement(_bodyElement, "inertia"))
    {
        tinyxml2::XMLElement* inertiaElement = getElement(_bodyElement, "inertia");

        math::Inertia I;

        // mass
        double mass = getValueDouble(inertiaElement, "mass");
        I.setMass(mass);

        // moment of inertia
        if (hasElement(inertiaElement, "moment_of_inertia")) {
            tinyxml2::XMLElement* moiElement
                    = getElement(inertiaElement, "moment_of_inertia");

            double ixx = getValueDouble(moiElement, "ixx");
            double iyy = getValueDouble(moiElement, "iyy");
            double izz = getValueDouble(moiElement, "izz");

            double ixy = getValueDouble(moiElement, "ixy");
            double ixz = getValueDouble(moiElement, "ixz");
            double iyz = getValueDouble(moiElement, "iyz");

            I.setAngularMomentDiag   (ixx, iyy, izz);
            I.setAngularMomentOffDiag(ixy, ixz, iyz);
        }
        else if (newBody->getVisualizationShape() != 0) {
            I = newBody->getVisualizationShape()->computeInertia2(mass);
        }

        // offset
        math::Vec3 offset = getValueVec3(inertiaElement, "offset");
        I.setOffset(offset);

        newBody->setInertia(I);
    }

    return newBody;
}

dynamics::Joint* readJoint(tinyxml2::XMLElement* _jointElement,
                            dynamics::Skeleton* _skeleton)
{
    assert(_jointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::Joint* newJoint = NULL;

    //--------------------------------------------------------------------------
    // Type attribute
    std::string type = getAttribute(_jointElement, "type");
    assert(!type.empty());
    if (type == std::string("weld"))
        newJoint = readWeldJoint(_jointElement, _skeleton);
    if (type == std::string("revolute"))
        newJoint = readRevoluteJoint(_jointElement, _skeleton);
    if (type == std::string("ball"))
        newJoint = readBallJoint(_jointElement, _skeleton);
    if (type == std::string("euler_xyz"))
        newJoint = readEulerXYZJoint(_jointElement, _skeleton);
    if (type == std::string("translational"))
        newJoint = readTranslationalJoint(_jointElement, _skeleton);
    if (type == std::string("free"))
        newJoint = readFreeJoint(_jointElement, _skeleton);
    assert(newJoint != NULL);

    //--------------------------------------------------------------------------
    // Name attribute
    std::string name = getAttribute(_jointElement, "name");
    newJoint->setName(name);

    //--------------------------------------------------------------------------
    // parent
    tinyxml2::XMLElement* parentElement = NULL;
    parentElement = _jointElement->FirstChildElement("parent");
    assert(parentElement != NULL);
    std::string strParent = parentElement->GetText();
    dynamics::BodyNode* parentBody = NULL;

    if (strParent == std::string("world"))
    {
        newJoint->setParentBody(NULL);
    }
    else
    {
        parentBody = _skeleton->findBody(strParent);
        if (parentBody == NULL)
        {
            dterr << "Can't find the parent body, "
              << strParent
              << ", of the joint, "
              << newJoint->getName()
              << ", in the skeleton, "
              << _skeleton->getName()
              << ". " << std::endl;
            assert(parentBody != NULL);
        }
        newJoint->setParentBody(parentBody);
    }

    //--------------------------------------------------------------------------
    // child
    tinyxml2::XMLElement* childElement = NULL;
    childElement = _jointElement->FirstChildElement("child");
    assert(childElement != NULL);
    std::string strChild = childElement->GetText();
    dynamics::BodyNode* childBody = _skeleton->findBody(strChild);
    assert(childBody != NULL && "Dart cannot find child body.");
    newJoint->setChildBody(childBody);

    //--------------------------------------------------------------------------
    // transformation
    tinyxml2::XMLElement* transformationElement = NULL;
    transformationElement = _jointElement->FirstChildElement("transformation");
    math::SE3 parentWorld;
    math::SE3 childToJoint;
    math::SE3 childWorld = childBody->getTransformationWorld();
    if (parentBody)
         parentWorld = parentBody->getTransformationWorld();
    if (transformationElement != NULL)
    {
        std::string strTransformation = transformationElement->GetText();
        childToJoint = toSE3(strTransformation);
    }
    math::SE3 parentToJoint = math::Inv(parentWorld)
                              * childWorld
                              * childToJoint;
    newJoint->setLocalTransformFromChildBody(childToJoint);
    newJoint->setLocalTransformFromParentBody(parentToJoint);

    return newJoint;
}

dynamics::WeldJoint*readWeldJoint(
        tinyxml2::XMLElement* _weldJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_weldJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::WeldJoint* newWeldJoint
            = new dynamics::WeldJoint;

    return newWeldJoint;
}

dynamics::RevoluteJoint*readRevoluteJoint(
        tinyxml2::XMLElement* _revoluteJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_revoluteJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::RevoluteJoint* newRevoluteJoint = new dynamics::RevoluteJoint;

    //--------------------------------------------------------------------------
    // axis
    tinyxml2::XMLElement* axisElement = NULL;
    axisElement = _revoluteJointElement->FirstChildElement("axis");
    assert(axisElement != NULL);
    tinyxml2::XMLElement* xyzElement = axisElement->FirstChildElement("xyz");
    assert(xyzElement != NULL);
    std::string strXYZ = xyzElement->GetText();
    math::so3 axis = toso3(strXYZ);
    newRevoluteJoint->setAxis(axis);

    return newRevoluteJoint;
}


dynamics::BallJoint*readBallJoint(
        tinyxml2::XMLElement* _ballJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_ballJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

    return newBallJoint;
}

dynamics::EulerXYZJoint* readEulerXYZJoint(
        tinyxml2::XMLElement* _eulerXYZJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_eulerXYZJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::EulerXYZJoint* newEulerXYZJoint = new dynamics::EulerXYZJoint;

    return newEulerXYZJoint;
}

dynamics::TranslationalJoint*readTranslationalJoint(
        tinyxml2::XMLElement* _translationalJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_translationalJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::TranslationalJoint* newTranslationalJoint
            = new dynamics::TranslationalJoint;

    return newTranslationalJoint;
}

dynamics::FreeJoint*readFreeJoint(
        tinyxml2::XMLElement* _freeJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_freeJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::FreeJoint* newFreeJoint
            = new dynamics::FreeJoint;

    return newFreeJoint;
}

std::string toString(bool _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(int _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(unsigned int _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(float _v)
{
    if (std::isfinite(_v))
        return boost::lexical_cast<std::string>(_v);
    else
        return std::string("0");
}

std::string toString(double _v)
{
    if (std::isfinite(_v))
        return boost::lexical_cast<std::string>(_v);
    else
        return std::string("0");
}

std::string toString(char _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(const Eigen::Vector2d& _v)
{
    Eigen::RowVector2d rowVector2d = _v.transpose();

    return boost::lexical_cast<std::string>(rowVector2d);
}

std::string toString(const Eigen::Vector3d& _v)
{
    Eigen::RowVector3d rowVector3d = _v.transpose();

    return boost::lexical_cast<std::string>(rowVector3d);
}

std::string toString(const math::Vec3& _v)
{
    std::ostringstream ostr;
    ostr.precision(6);

    ostr << _v[0] << " " << _v[1] << " " << _v[2];

    return ostr.str();
}

std::string toString(const math::so3& _v)
{
    std::ostringstream ostr;
    ostr.precision(6);

    ostr << _v(0) << " " << _v(1) << " " << _v(2);

    return ostr.str();
}

//std::string toString(const math::SO3& _v)
//{
//    return boost::lexical_cast<std::string>(_v);
//}

std::string toString(const math::SE3& _v)
{
    std::ostringstream ostr;
    ostr.precision(6);

    math::Vec3 XYZ = math::iEulerXYZ(_v);
    math::Vec3 position = _v.getPosition();

    ostr << XYZ[0] << " " << XYZ[1] << " " << XYZ[2];
    ostr << " ";
    ostr << position[0] << " " << position[1] << " " << position[2];

    return ostr.str();
}

bool toBool(const std::string& _str)
{
    return boost::lexical_cast<bool>(_str);
}

int toInt(const std::string& _str)
{
    return boost::lexical_cast<int>(_str);
}

unsigned int toUInt(const std::string& _str)
{
    return boost::lexical_cast<unsigned int>(_str);
}

float toFloat(const std::string& _str)
{
    return boost::lexical_cast<float>(_str);
}

double toDouble(const std::string& _str)
{
    return boost::lexical_cast<double>(_str);
}

char toChar(const std::string& _str)
{
    return boost::lexical_cast<char>(_str);
}

Eigen::Vector2d toVector2d(const std::string& _str)
{
    Eigen::Vector2d ret;

    std::vector<double> elements;
    std::vector<std::string> pieces;
    boost::split(pieces, _str, boost::is_any_of(" "));
    assert(pieces.size() == 2);

    for (int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            try
            {
                elements.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
            }
            catch(boost::bad_lexical_cast& e)
            {
                std::cerr << "value ["
                          << pieces[i]
                          << "] is not a valid double for Eigen::Vector2d["
                          << i
                          << std::endl;
            }
        }
    }

    ret(0) = elements[0];
    ret(1) = elements[1];

    return ret;
}

Eigen::Vector3d toVector3d(const std::string& _str)
{
    Eigen::Vector3d ret;

    std::vector<double> elements;
    std::vector<std::string> pieces;
    boost::split(pieces, _str, boost::is_any_of(" "));
    assert(pieces.size() == 3);

    for (int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            try
            {
                elements.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
            }
            catch(boost::bad_lexical_cast& e)
            {
                std::cerr << "value ["
                          << pieces[i]
                          << "] is not a valid double for Eigen::Vector3d["
                          << i
                          << "]"
                          << std::endl;
            }
        }
    }

    ret(0) = elements[0];
    ret(1) = elements[1];
    ret(2) = elements[2];

    return ret;
}

math::Vec3 toVec3(const std::string& _str)
{
    math::Vec3 ret;

    std::vector<double> elements;
    std::vector<std::string> pieces;
    boost::split(pieces, _str, boost::is_any_of(" "));
    assert(pieces.size() == 3);

    for (int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            try
            {
                elements.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
            }
            catch(boost::bad_lexical_cast& e)
            {
                std::cerr << "value ["
                          << pieces[i]
                          << "] is not a valid double for Eigen::Vector3d["
                          << i
                          << "]"
                          << std::endl;
            }
        }
    }

    ret[0] = elements[0];
    ret[1] = elements[1];
    ret[2] = elements[2];

    return ret;
}

math::so3 toso3(const std::string& _str)
{
    math::so3 ret;

    std::vector<double> elements;
    std::vector<std::string> pieces;
    boost::split(pieces, _str, boost::is_any_of(" "));
    assert(pieces.size() == 3);

    for (int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            try
            {
                elements.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
            }
            catch(boost::bad_lexical_cast& e)
            {
                std::cerr << "value ["
                          << pieces[i]
                          << "] is not a valid double for so3["
                          << i
                          << "]"
                          << std::endl;
            }
        }
    }

    ret(0) = elements[0];
    ret(1) = elements[1];
    ret(2) = elements[2];

    return ret;
}


//math::SO3 toSO3(const std::string& _str)
//{
//    math::SO3 ret;
//    Eigen::Vector3d EulerXYZ;

//    std::vector<double> elements;
//    std::vector<std::string> pieces;
//    boost::split(pieces, _str, boost::is_any_of(" "));
//    assert(pieces.size() == 3);

//    for (int i = 0; i < pieces.size(); ++i)
//    {
//        if (pieces[i] != "")
//        {
//            try
//            {
//                elements.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
//            }
//            catch(boost::bad_lexical_cast& e)
//            {
//                std::cerr << "value ["
//                          << pieces[i]
//                          << "] is not a valid double for EulerXYZ["
//                          << i
//                          << "]"
//                          << std::endl;
//            }
//        }
//    }

//    EulerXYZ(0) = elements[0];
//    EulerXYZ(1) = elements[1];
//    EulerXYZ(2) = elements[2];

//    ret.setEulerXYZ(EulerXYZ);

//    return ret;
//}


math::SE3 toSE3(const std::string& _str)
{
    std::vector<double> elements;
    std::vector<std::string> pieces;
    boost::split(pieces, _str, boost::is_any_of(" "));
    assert(pieces.size() == 6);

    for (int i = 0; i < pieces.size(); ++i)
    {
        if (pieces[i] != "")
        {
            try
            {
                elements.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
            }
            catch(boost::bad_lexical_cast& e)
            {
                std::cerr << "value ["
                          << pieces[i]
                          << "] is not a valid double for SE3["
                          << i
                          << "]"
                          << std::endl;
            }
        }
    }

    return math::EulerXYZ(math::Vec3(elements[0], elements[1], elements[2]),
                          math::Vec3(elements[3], elements[4], elements[5]));
}

std::string getValueString(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return str;
}

bool getValueBool(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toBool(str);
}

int getValueInt(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toInt(str);
}

unsigned int getValueUInt(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toUInt(str);
}

float getValueFloat(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toFloat(str);
}

double getValueDouble(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toDouble(str);
}

char getValueChar(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toChar(str);
}

Eigen::Vector2d getValueVector2d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector2d(str);
}

Eigen::Vector3d getValueVector3d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector3d(str);
}

math::Vec3 getValueVec3(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVec3(str);
}

math::so3 getValueso3(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toso3(str);
}

//math::SO3 getValueSO3(tinyxml2::XMLElement* _parentElement, const string& _name)
//{
//    assert(_parentElement != NULL);
//    assert(!_name.empty());

//    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

//    return toSO3(str);
//}

math::SE3 getValueSE3(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toSE3(str);
}

bool hasElement(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    return _parentElement->FirstChildElement(_name.c_str()) == NULL ? false : true;
}

tinyxml2::XMLElement* getElement(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name)
{
    assert(!_name.empty());

    return _parentElement->FirstChildElement(_name.c_str());
}






} // namespace simulation
} // namespace dart
