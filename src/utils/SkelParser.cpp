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
#include "common/Console.h"
#include "dynamics/BodyNode.h"
#include "dynamics/BoxShape.h"
#include "dynamics/CylinderShape.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/WeldJoint.h"
#include "dynamics/PrismaticJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/ScrewJoint.h"
#include "dynamics/TranslationalJoint.h"
#include "dynamics/BallJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/EulerJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"
#include "utils/SkelParser.h"

namespace dart {
namespace utils {

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
    return *this;
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
                                 simulation::World* _world)
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
        Eigen::Isometry3d W = getValueIsometry3d(_skeletonElement, "transformation");
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
                = readBodyNode(bodies.get(), newSkeleton);

        newSkeleton->addBodyNode(newBody, false);
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

dynamics::BodyNode* readBodyNode(tinyxml2::XMLElement* _bodyNodeElement,
                                 dynamics::Skeleton* _skeleton)
{
    assert(_bodyNodeElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BodyNode* newBodyNode = new dynamics::BodyNode;

    // Name attribute
    std::string name = getAttribute(_bodyNodeElement, "name");
    newBodyNode->setName(name);

    //--------------------------------------------------------------------------
    // gravity
    if (hasElement(_bodyNodeElement, "gravity"))
    {
        bool gravityMode = getValueBool(_bodyNodeElement, "gravity");
        newBodyNode->setGravityMode(gravityMode);
    }

    //--------------------------------------------------------------------------
    // self_collide
//    if (hasElement(_bodyElement, "self_collide"))
//    {
//        bool gravityMode = getValueBool(_bodyElement, "self_collide");
//    }

    //--------------------------------------------------------------------------
    // transformation
    if (hasElement(_bodyNodeElement, "transformation"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(_bodyNodeElement, "transformation");
        newBodyNode->setWorldTransform(_skeleton->getWorldTransformation() * W);
    }

    // visualization_shape
    if (hasElement(_bodyNodeElement, "visualization_shape"))
    {
        tinyxml2::XMLElement* vizElement
                = getElement(_bodyNodeElement, "visualization_shape");

        dynamics::Shape* shape = NULL;

        // type
        assert(hasElement(vizElement, "geometry"));
        tinyxml2::XMLElement* geometryElement = getElement(vizElement, "geometry");

        // FIXME: Assume that type has only one shape type.
        if (hasElement(geometryElement, "box"))
        {
            tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

            Eigen::Vector3d size = getValueVector3d(boxElement, "size");

            shape = new dynamics::BoxShape(size);
        }
        else if (hasElement(geometryElement, "ellipsoid"))
        {
            tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "ellipsoid");

            Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

            shape = new dynamics::EllipsoidShape(size);
        }
        else if (hasElement(geometryElement, "cylinder"))
        {
            tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

            double radius = getValueDouble(cylinderElement, "radius");
            double height = getValueDouble(cylinderElement, "height");

            shape = new dynamics::CylinderShape(radius, height);
        }
        else
        {
            dterr << "Unknown visualization shape.\n";
            assert(0);
        }
        newBodyNode->addVisualizationShape(shape);

        // transformation
        if (hasElement(vizElement, "transformation"))
        {
            Eigen::Isometry3d W = getValueIsometry3d(vizElement, "transformation");
            shape->setTransform(W);
        }

        // color
        if (hasElement(vizElement, "color"))
        {
            Eigen::Vector3d color = getValueVector3d(vizElement, "color");
            shape->setColor(color);
        }
    }

    // collision_shape
    if (hasElement(_bodyNodeElement, "collision_shape"))
    {
        tinyxml2::XMLElement* colElement
                = getElement(_bodyNodeElement, "collision_shape");

        dynamics::Shape* shape = NULL;

        // type
        assert(hasElement(colElement, "geometry"));
        tinyxml2::XMLElement* geometryElement = getElement(colElement, "geometry");

        // FIXME: Assume that type has only one shape type.
        if (hasElement(geometryElement, "box"))
        {
            tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

            Eigen::Vector3d size = getValueVector3d(boxElement, "size");

            shape = new dynamics::BoxShape(size);
        }
        else if (hasElement(geometryElement, "ellipsoid"))
        {
            tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "ellipsoid");

            Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

            shape = new dynamics::EllipsoidShape(size);
        }
        else if (hasElement(geometryElement, "cylinder"))
        {
            tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

            double radius = getValueDouble(cylinderElement, "radius");
            double height = getValueDouble(cylinderElement, "height");

            shape = new dynamics::CylinderShape(radius, height);
        }
        else
        {
            dterr << "Unknown visualization shape.\n";
            assert(0);
        }
        newBodyNode->addCollisionShape(shape);

        // transformation
        if (hasElement(colElement, "transformation"))
        {
            Eigen::Isometry3d W = getValueIsometry3d(colElement, "transformation");
            shape->setTransform(W);
        }
    }

    //--------------------------------------------------------------------------
    // inertia
    if (hasElement(_bodyNodeElement, "inertia"))
    {
        tinyxml2::XMLElement* inertiaElement = getElement(_bodyNodeElement, "inertia");

        // mass
        double mass = getValueDouble(inertiaElement, "mass");
        newBodyNode->setMass(mass);

        // moment of inertia
        if (hasElement(inertiaElement, "moment_of_inertia"))
        {
            tinyxml2::XMLElement* moiElement
                    = getElement(inertiaElement, "moment_of_inertia");

            double ixx = getValueDouble(moiElement, "ixx");
            double iyy = getValueDouble(moiElement, "iyy");
            double izz = getValueDouble(moiElement, "izz");

            double ixy = getValueDouble(moiElement, "ixy");
            double ixz = getValueDouble(moiElement, "ixz");
            double iyz = getValueDouble(moiElement, "iyz");

            newBodyNode->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
        }
        else if (newBodyNode->getVisualizationShape(0) != 0)
        {
            Eigen::Matrix3d Ic = newBodyNode->getVisualizationShape(0)->computeInertia(mass);

            newBodyNode->setMomentOfInertia(Ic(0,0), Ic(1,1), Ic(2,2),
                                            Ic(0,1), Ic(0,2), Ic(1,2));
        }

        // offset
        if (hasElement(inertiaElement, "offset"))
        {
            Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
            newBodyNode->setLocalCOM(offset);
        }
    }

    return newBodyNode;
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
    if (type == std::string("prismatic"))
        newJoint = readPrismaticJoint(_jointElement, _skeleton);
    if (type == std::string("revolute"))
        newJoint = readRevoluteJoint(_jointElement, _skeleton);
    if (type == std::string("universal"))
        newJoint = readUniversalJoint(_jointElement, _skeleton);
    if (type == std::string("ball"))
        newJoint = readBallJoint(_jointElement, _skeleton);
    if (type == std::string("euler"))
        newJoint = readEulerJoint(_jointElement, _skeleton);
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
    dynamics::BodyNode* parentBody = NULL;
    if (hasElement(_jointElement, "parent"))
    {
        std::string strParent = getValueString(_jointElement, "parent");

        if (strParent == std::string("world"))
        {
            newJoint->setParentBody(NULL);
        }
        else
        {
            parentBody = _skeleton->findBodyNode(strParent);
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
    }
    else
    {
        dterr << "No parent body.\n";
        assert(0);
    }

    //--------------------------------------------------------------------------
    // child
    dynamics::BodyNode* childBody = NULL;
    if (hasElement(_jointElement, "child"))
    {
        std::string strChild = getValueString(_jointElement, "child");
        childBody = _skeleton->findBodyNode(strChild);
        assert(childBody != NULL && "Dart cannot find child body.");
        newJoint->setChildBody(childBody);
    }
    else
    {
        dterr << "No child body.\n";
        assert(0);
    }

    //--------------------------------------------------------------------------
    // transformation
    Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
    assert(childBody != NULL);
    Eigen::Isometry3d childWorld = childBody->getWorldTransform();
    if (parentBody)
         parentWorld = parentBody->getWorldTransform();
    if (hasElement(_jointElement, "transformation"))
        childToJoint = getValueIsometry3d(_jointElement, "transformation");
    Eigen::Isometry3d parentToJoint = math::Inv(parentWorld)*childWorld*childToJoint;
    newJoint->setTransformFromChildBody(childToJoint);
    newJoint->setTransformFromParentBody(parentToJoint);

    return newJoint;
}

dynamics::WeldJoint*readWeldJoint(
        tinyxml2::XMLElement* _weldJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_weldJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::WeldJoint* newWeldJoint = new dynamics::WeldJoint;

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
    if (hasElement(_revoluteJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_revoluteJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newRevoluteJoint->setAxis(xyz);

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newRevoluteJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newRevoluteJoint->getDof(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newRevoluteJoint->getDof(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_revoluteJointElement, "init_pos"))
    {
        double init_pos = getValueDouble(_revoluteJointElement, "init_pos");
        Eigen::VectorXd ipos = Eigen::VectorXd(1);
        ipos << init_pos;
        newRevoluteJoint->set_q(ipos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_revoluteJointElement, "init_vel"))
    {
        double init_vel = getValueDouble(_revoluteJointElement, "init_vel");
        Eigen::VectorXd ivel = Eigen::VectorXd(1);
        ivel << init_vel;
        newRevoluteJoint->set_q(ivel);
    }

    return newRevoluteJoint;
}

dynamics::PrismaticJoint* readPrismaticJoint(
        tinyxml2::XMLElement* _prismaticJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_prismaticJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::PrismaticJoint* newPrismaticJoint = new dynamics::PrismaticJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_prismaticJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_prismaticJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newPrismaticJoint->setAxis(xyz);

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newPrismaticJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newPrismaticJoint->getDof(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newPrismaticJoint->getDof(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_prismaticJointElement, "init_pos"))
    {
        double init_pos = getValueDouble(_prismaticJointElement, "init_pos");
        Eigen::VectorXd ipos = Eigen::VectorXd(1);
        ipos << init_pos;
        newPrismaticJoint->set_q(ipos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_prismaticJointElement, "init_vel"))
    {
        double init_vel = getValueDouble(_prismaticJointElement, "init_vel");
        Eigen::VectorXd ivel = Eigen::VectorXd(1);
        ivel << init_vel;
        newPrismaticJoint->set_q(ivel);
    }

    return newPrismaticJoint;
}

dynamics::ScrewJoint* readScrewJoint(
        tinyxml2::XMLElement* _screwJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_screwJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::ScrewJoint* newScrewJoint = new dynamics::ScrewJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_screwJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_screwJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newScrewJoint->setAxis(xyz);

        // pitch
        if (hasElement(axisElement, "pitch"))
        {
            double pitch = getValueDouble(axisElement, "pitch");
            newScrewJoint->setPitch(pitch);
        }

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newScrewJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newScrewJoint->getDof(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newScrewJoint->getDof(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_screwJointElement, "init_pos"))
    {
        double init_pos = getValueDouble(_screwJointElement, "init_pos");
        Eigen::VectorXd ipos = Eigen::VectorXd(1);
        ipos << init_pos;
        newScrewJoint->set_q(ipos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_screwJointElement, "init_vel"))
    {
        double init_vel = getValueDouble(_screwJointElement, "init_vel");
        Eigen::VectorXd ivel = Eigen::VectorXd(1);
        ivel << init_vel;
        newScrewJoint->set_q(ivel);
    }

    return newScrewJoint;
}

dynamics::UniversalJoint* readUniversalJoint(
        tinyxml2::XMLElement* _universalJointElement,
        dynamics::Skeleton *_skeleton)
{
    assert(_universalJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::UniversalJoint* newUniversalJoint = new dynamics::UniversalJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_universalJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_universalJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newUniversalJoint->setAxis(0, xyz);

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newUniversalJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newUniversalJoint->getDof(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newUniversalJoint->getDof(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // axis2
    if (hasElement(_universalJointElement, "axis2"))
    {
        tinyxml2::XMLElement* axis2Element
                = getElement(_universalJointElement, "axis2");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
        newUniversalJoint->setAxis(1, xyz);

        // damping
        if (hasElement(axis2Element, "damping"))
        {
            double damping = getValueDouble(axis2Element, "damping");
            newUniversalJoint->setDampingCoefficient(1, damping);
        }

        // limit
        if (hasElement(axis2Element, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axis2Element, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newUniversalJoint->getDof(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newUniversalJoint->getDof(1)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_universalJointElement, "init_pos"))
    {
        Eigen::Vector2d init_pos = getValueVector2d(_universalJointElement, "init_pos");
        newUniversalJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_universalJointElement, "init_vel"))
    {
        Eigen::Vector2d init_vel = getValueVector2d(_universalJointElement, "init_vel");
        newUniversalJoint->set_q(init_vel);
    }

    return newUniversalJoint;
}

dynamics::BallJoint* readBallJoint(
        tinyxml2::XMLElement* _ballJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_ballJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_ballJointElement, "init_pos"))
    {
        Eigen::Vector3d init_pos = getValueVector3d(_ballJointElement, "init_pos");
        newBallJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_ballJointElement, "init_vel"))
    {
        Eigen::Vector3d init_vel = getValueVector3d(_ballJointElement, "init_vel");
        newBallJoint->set_q(init_vel);
    }

    return newBallJoint;
}

dynamics::EulerJoint* readEulerJoint(
        tinyxml2::XMLElement* _eulerJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_eulerJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::EulerJoint* newEulerJoint = new dynamics::EulerJoint;

    //--------------------------------------------------------------------------
    // axis order
    std::string order = getValueString(_eulerJointElement, "axis_order");
    if (order == "xyz")
    {
        newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_XYZ);
    }
    else if (order == "zyx")
    {
        newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_ZYX);
    }
    else
    {
        dterr << "Undefined Euler axis order\n";
        assert(0);
    }

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_eulerJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_eulerJointElement, "axis");

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newEulerJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newEulerJoint->getDof(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newEulerJoint->getDof(0)->set_qMax(upper);
            }
        }
    }

    //--------------------------------------------------------------------------
    // axis2
    if (hasElement(_eulerJointElement, "axis2"))
    {
        tinyxml2::XMLElement* axis2Element
                = getElement(_eulerJointElement, "axis2");

        // damping
        if (hasElement(axis2Element, "damping"))
        {
            double damping = getValueDouble(axis2Element, "damping");
            newEulerJoint->setDampingCoefficient(1, damping);
        }

        // limit
        if (hasElement(axis2Element, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axis2Element, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newEulerJoint->getDof(1)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newEulerJoint->getDof(1)->set_qMax(upper);
            }
        }
    }

    //--------------------------------------------------------------------------
    // axis3
    if (hasElement(_eulerJointElement, "axis3"))
    {
        tinyxml2::XMLElement* axis3Element
                = getElement(_eulerJointElement, "axis3");

        // damping
        if (hasElement(axis3Element, "damping"))
        {
            double damping = getValueDouble(axis3Element, "damping");
            newEulerJoint->setDampingCoefficient(2, damping);
        }

        // limit
        if (hasElement(axis3Element, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axis3Element, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newEulerJoint->getDof(2)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newEulerJoint->getDof(2)->set_qMax(upper);
            }
        }
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_eulerJointElement, "init_pos"))
    {
        Eigen::Vector3d init_pos = getValueVector3d(_eulerJointElement, "init_pos");
        newEulerJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_eulerJointElement, "init_vel"))
    {
        Eigen::Vector3d init_vel = getValueVector3d(_eulerJointElement, "init_vel");
        newEulerJoint->set_q(init_vel);
    }

    return newEulerJoint;
}

dynamics::TranslationalJoint*readTranslationalJoint(
        tinyxml2::XMLElement* _translationalJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_translationalJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::TranslationalJoint* newTranslationalJoint
            = new dynamics::TranslationalJoint;

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_translationalJointElement, "init_pos"))
    {
        Eigen::Vector3d init_pos = getValueVector3d(_translationalJointElement, "init_pos");
        newTranslationalJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_translationalJointElement, "init_vel"))
    {
        Eigen::Vector3d init_vel = getValueVector3d(_translationalJointElement, "init_vel");
        newTranslationalJoint->set_q(init_vel);
    }

    return newTranslationalJoint;
}

dynamics::FreeJoint*readFreeJoint(
        tinyxml2::XMLElement* _freeJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_freeJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_freeJointElement, "init_pos"))
    {
        Eigen::Vector6d init_pos = getValueVector6d(_freeJointElement, "init_pos");
        newFreeJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_freeJointElement, "init_vel"))
    {
        Eigen::Vector6d init_vel = getValueVector6d(_freeJointElement, "init_vel");
        newFreeJoint->set_q(init_vel);
    }

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
    //if (std::isfinite(_v))
        return boost::lexical_cast<std::string>(_v);
    //else
    //    return std::string("0");
}

std::string toString(double _v)
{
    //if (std::isfinite(_v))
        return boost::lexical_cast<std::string>(_v);
    //else
    //    return std::string("0");
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

std::string toString(const Eigen::Isometry3d& _v)
{
    std::ostringstream ostr;
    ostr.precision(6);

    Eigen::Vector3d XYZ = math::iEulerXYZ(_v);

    ostr << _v(0,3) << " " << _v(1,3) << " " << _v(2,3);
    ostr << " ";
    ostr << XYZ[0] << " " << XYZ[1] << " " << XYZ[2];

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

Eigen::Vector6d toVector6d(const std::string& _str)
{
    Eigen::Vector6d ret;

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
                          << "] is not a valid double for Eigen::Vector6d["
                          << i
                          << "]"
                          << std::endl;
            }
        }
    }

    ret(0) = elements[0];
    ret(1) = elements[1];
    ret(2) = elements[2];
    ret(3) = elements[3];
    ret(4) = elements[4];
    ret(5) = elements[5];

    return ret;
}

Eigen::Isometry3d toIsometry3d(const std::string& _str)
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

    return math::EulerXYZ(Eigen::Vector3d(elements[3], elements[4], elements[5]),
                          Eigen::Vector3d(elements[0], elements[1], elements[2]));
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

Eigen::Vector6d getValueVector6d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector6d(str);
}

Eigen::Vector3d getValueVec3(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector3d(str);
}

Eigen::Isometry3d getValueIsometry3d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != NULL);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toIsometry3d(str);
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

} // namespace utils
} // namespace dart
