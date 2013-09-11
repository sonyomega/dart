/**
 * @file Main.cpp
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief This application shows the creation of a kinematic skeleton from scratch without
 * the use of a model file. Run the program without arguments and you can use the buttons
 * {1,2} to move the corresponding joints. The key '-' will make the joints move in the negative
 * direction.
 */

#include "dynamics/BodyNode.h"
#include "dynamics/PrismaticJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "MyWindow.h"

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace dynamics;

/// Function headers
enum TypeOfDOF {
    DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
};

/* ********************************************************************************************* */
/// Add a DOF to a given joint
Joint* create1DOFJoint(BodyNode* parent, BodyNode* child, double val, double min, double max, int type) {

    // Create the transformation based on the type
    Joint* newJoint = NULL;
    if(type == DOF_X)
        newJoint = new PrismaticJoint(parent, child, Eigen::Vector3d(1.0, 0.0, 0.0));
    else if(type == DOF_Y)
        newJoint = new PrismaticJoint(parent, child, Eigen::Vector3d(0.0, 1.0, 0.0));
    else if(type == DOF_Z)
        newJoint = new PrismaticJoint(parent, child, Eigen::Vector3d(0.0, 0.0, 1.0));
    else if(type == DOF_YAW)
        newJoint = new RevoluteJoint(parent, child, Eigen::Vector3d(0.0, 0.0, 1.0));
    else if(type == DOF_PITCH)
        newJoint = new RevoluteJoint(parent, child, Eigen::Vector3d(0.0, 1.0, 0.0));
    else if(type == DOF_ROLL)
        newJoint = new RevoluteJoint(parent, child, Eigen::Vector3d(1.0, 0.0, 0.0));
    // Add the transformation to the joint, set the min/max values and set it to the skeleton
    newJoint->getGenCoord(0)->set_q(val);
    newJoint->getGenCoord(0)->set_qMin(min);
    newJoint->getGenCoord(0)->set_qMax(max);

    return newJoint;
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

    // Create Left Leg skeleton
    Skeleton LeftLegSkel;

    // Pointers to be used during the Skeleton building
    Matrix3d inertiaMatrix;
    inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    double mass = 1.0;

    // ***** BodyNode 1: Left Hip Yaw (LHY) ***** *
    BodyNode* node = new BodyNode("LHY");
    Joint* joint = create1DOFJoint(NULL, node, 0.0, 0.0, DART_PI, DOF_YAW);
    joint->setName("LHY");
    Shape* shape = new BoxShape(Vector3d(0.3, 0.3, 1.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    LeftLegSkel.addBodyNode(node);

    // ***** BodyNode 2: Left Hip Roll (LHR) whose parent is: LHY *****\

    BodyNode* parent_node = LeftLegSkel.findBodyNode("LHY");
    node = new BodyNode("LHR");
    joint = create1DOFJoint(parent_node, node, 0.0, 0.0, DART_PI, DOF_ROLL);
    joint->setName("LHR");
    Eigen::Isometry3d T(Eigen::Translation3d(0.0, 0.0, 0.5));
    joint->setTransformFromParentBody(T);
    shape = new BoxShape(Vector3d(0.3, 0.3, 1.0));
    shape->setOffset(Vector3d(0.0, 0.0, 0.5));
    node->setLocalCOM(shape->getOffset());
    node->setMass(mass);
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    LeftLegSkel.addBodyNode(node);

    // ***** BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR *****
    parent_node = LeftLegSkel.findBodyNode("LHR");
    node = new BodyNode("LHP");
    joint = create1DOFJoint(parent_node, node, 0.0, 0.0, DART_PI, DOF_ROLL);
    joint->setName("LHP");
    T = Eigen::Translation3d(0.0, 0.0, 1.0);
    joint->setTransformFromParentBody(T);
    shape = new BoxShape(Vector3d(0.3, 0.3, 1.0));
    shape->setOffset(Vector3d(0.0, 0.0, 0.5));
    node->setLocalCOM(shape->getOffset());
    node->setMass(mass);
    Shape* shape1 = new EllipsoidShape(Vector3d(0.3, 0.3, 1.0));
    shape1->setOffset(Vector3d(0.0, 0.0, 0.5));
    node->addVisualizationShape(shape1);
    node->addCollisionShape(shape);
    LeftLegSkel.addBodyNode(node);

    // Initialize the skeleton
    LeftLegSkel.initDynamics();

    // Window stuff
    MyWindow window(&LeftLegSkel);
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Skeleton example");
    glutMainLoop();

    return 0;
}
