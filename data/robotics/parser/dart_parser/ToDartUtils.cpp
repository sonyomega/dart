/**
 * @file ToDartUtils.cpp
 * @brief Functions that convert ModelInterface objects to their Dart counterparts
 * @author A. Huaman Quispe
 * @date 2012 / 10 / 12
 */
#include "DartLoader.h"
#include <dynamics/BodyNode.h>
#include <dynamics/Dof.h>
#include <dynamics/Joint.h>
#include <dynamics/Skeleton.h>
#include <dynamics/Shape.h>
#include <dynamics/WeldJoint.h>
#include <dynamics/RevoluteJoint.h>
#include <dynamics/PrismaticJoint.h>
#include <dynamics/TranslationalJoint.h>
#include <dynamics/BallJoint.h>
#include <dynamics/FreeJoint.h>
#include "../urdf_parser/urdf_parser.h"
#include "../urdfdom_headers/urdf_model/link.h"

// For continuous joint limit
#include <limits>

namespace dart {

/**
 * @function createDartRootJoint
 * @brief Create defined rootJoint between world and rootNode
 */
dynamics::Joint* DartLoader::createDartRootJoint( boost::shared_ptr<urdf::Joint> _jt,
                            dynamics::Skeleton* _skel ) {

    // Currently we support 2 root joints: Fixed and floating
    // other type of joints (revolute and prismatic) appear as fixed so don't use them by the time being
    // (joints located on nodes other than root can be anything)
  
    double x, y, z;
    double roll, pitch, yaw;
    double val;
    double vmin, vmax;
    int axis;
    double xAxis, yAxis, zAxis;
  
    // Parent is NULL (World).
    if( strcmp( (_jt->parent_link_name).c_str(), "world") != 0 ) {
        std::cout<< "Error, createDartRootJoint should receive a joint with world as parent. No creating joint" << std::endl;
        return NULL;
    }

    // Get child body Nodes
    dynamics::Joint* joint;
    std::string jointName = ( _jt->name);
    std::string childName = (_jt->child_link_name);
    dynamics::BodyNode* child = getNode( childName );

    joint = new dynamics::Joint( NULL, child, jointName.c_str() );

  
    // For FLOATING root joint we do NOT use createDartJoint since it automatically adds a transform xyzRPY (fixed)
    // before the DOF. This would make the root joint to behave as a fixed joint with the current setup
    val = 0;
    if( _jt->type == urdf::Joint::FLOATING ) {
    
        axis = GOLEM_FLOATING;

        // Define infinity limits
        vmin = -1*std::numeric_limits<double>::infinity();
        vmax = std::numeric_limits<double>::infinity();
        xAxis = 0; yAxis = 0; zAxis = 0;
    
        add_DOF( _skel, joint, val, vmin, vmax, axis, xAxis, yAxis, zAxis );
        return joint;
    }
  
    else {
        return createDartJoint( _jt, _skel );
    }

}


/**
 * @function createNewDartRootJoint
 * @brief Create a new floating joint if no joint is defined between world and rootNode
 */
dynamics::Joint* DartLoader::createNewDartRootJoint( dynamics::BodyNode* _node,
                               dynamics::Skeleton* _skel ) {

  
  // Parent will be NULL.   
  dynamics::Joint* rootJoint;
  if(debug) std::cout<<"[debug] Creating joint for root node: "<< _node->getName() <<std::endl;

  // This joint connects with the world 
  rootJoint = new dynamics::Joint( NULL, _node, "worldJoint" );
   
  // This will be a FREEEULER joint type. We have 3 DOF for rotation and 3 DOF for translation
  add_DOF( _skel, rootJoint, 0, 0, 0, GOLEM_FLOATING );

 
  return rootJoint;
}


/**
 * @function createDartJoint
 */
dynamics::Joint* DartLoader::createDartJoint( boost::shared_ptr<urdf::Joint> _jt,
                        dynamics::Skeleton* _skel ) {


  double x, y, z; 
  double roll, pitch, yaw;
  double val;
  double vmin, vmax;
  int axis;
  double xAxis, yAxis, zAxis;
  std::string jointName = ( _jt->name);

  dynamics::Joint* joint = NULL;
  
  // Add DOF if prismatic or revolute joint
  val = 0;
  if( _jt->type == urdf::Joint::REVOLUTE || 
      _jt->type == urdf::Joint::PRISMATIC || 
      _jt->type == urdf::Joint::CONTINUOUS ||
      _jt->type == urdf::Joint::FLOATING ) {
    
    // Revolute
    if(   _jt->type == urdf::Joint::REVOLUTE ) {  
      vmin = _jt->limits->lower;
      vmax = _jt->limits->upper;

      xAxis = _jt->axis.x; yAxis = _jt->axis.y; zAxis = _jt->axis.z;
      
      dynamics::RevoluteJoint* newRevoluteJoint = new dynamics::RevoluteJoint;
      newRevoluteJoint->setAxis(Eigen::Vector3d(xAxis, yAxis, zAxis));
      // TODO: vmin, vmax
      joint = newRevoluteJoint;
    }

    // Prismatic
    else if( _jt->type == urdf::Joint::PRISMATIC ) {
      vmin = _jt->limits->lower;
      vmax = _jt->limits->upper;

      xAxis = _jt->axis.x; yAxis = _jt->axis.y; zAxis = _jt->axis.z;

      dynamics::PrismaticJoint* newPrismaticJoint = new dynamics::PrismaticJoint;
      newPrismaticJoint->setAxis(Eigen::Vector3d(xAxis, yAxis, zAxis));
      // TODO: vmin, vmax
      joint = newPrismaticJoint;
    }
 
    // Continuous
    else if( _jt->type == urdf::Joint::CONTINUOUS ) {
      // Define infinity limits
      vmin = -1*std::numeric_limits<double>::infinity();
      vmax = std::numeric_limits<double>::infinity();

      xAxis = _jt->axis.x; yAxis = _jt->axis.y; zAxis = _jt->axis.z;

      dynamics::RevoluteJoint* newRevoluteJoint = new dynamics::RevoluteJoint;
      newRevoluteJoint->setAxis(Eigen::Vector3d(xAxis, yAxis, zAxis));
      // TODO: vmin, vmax
      joint = newRevoluteJoint;
    }

    // Floating
    else if( _jt->type == urdf::Joint::FLOATING ) {
      // Define infinity limits
      vmin = 0;
      vmax = 0;

      xAxis = 0; yAxis = 0; zAxis = 0;

      dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;
      joint = newFreeJoint;
    }
      
    //add_DOF( _skel, joint, val, vmin, vmax, axis, xAxis, yAxis, zAxis );
    
  }
  
  // Fixed, do not add DOF
  else if( _jt->type == urdf::Joint::FIXED ) {
    if(debug) std::cout<<"[debug] Fixed joint: "<< jointName <<std::endl;  
  }
  
  // None of the above
  else {
    std::cout<<"[createDartJoint] ERROR: Parsing "<<  jointName <<" joint: No PRISMATIC or REVOLUTE or CONTINUOUS or FIXED \n";
  }

  // Name
  joint->setName(jointName);
  
  // Get parent and child body Nodes
  std::string parentName = (_jt->parent_link_name);
  std::string childName =  (_jt->child_link_name);
  dynamics::BodyNode* parent = getNode( parentName );
  dynamics::BodyNode* child = getNode( childName );

  if( parent == NULL && parentName == "world" ) {
    joint->setParentBody(NULL);
    joint->setChildBody(child);
    std::cout<<"Creating joint between world and "<<childName<<std::endl;
  } else {
    joint->setParentBody(parent);
    joint->setChildBody(child);
  }

  // Add Rigid transform
  urdf::Pose p = _jt->parent_to_joint_origin_transform;
  x = p.position.x;
  y = p.position.y;
  z = p.position.z;
  p.rotation.getRPY( roll, pitch, yaw );
  math::SE3 parentWorld = math::SE3::Identity();
  math::SE3 childWorld = math::SE3::Identity();
  if (parent)
       parentWorld = parent->getTransformationWorld();
  if (child)
       parentWorld = child->getTransformationWorld();
  math::SE3 parentToJoint
          = math::EulerXYZ(Eigen::Vector3d(roll, pitch, yaw), Eigen::Vector3d(x, y, z));
  math::SE3 childToJoint
          = math::Inv(childWorld)*parentWorld*parentToJoint;
  joint->setLocalTransformFromChildBody(childToJoint);
  joint->setLocalTransformFromParentBody(parentToJoint);

  return joint;
}

/**
 * @function createDartNode
 */
dynamics::BodyNode* DartLoader::createDartNode( boost::shared_ptr<urdf::Link> _lk,
                            dynamics::Skeleton* _skel,
							std::string _rootToSkelPath ) {

  std::string lk_name = _lk->name;

  if( lk_name == "world" ) {
    std::cout << "[info] world is not parsed as a link" << std::endl;
    return NULL;
  }

  if(debug) std::cout<<"[debug] Creating dart node:"<< lk_name <<std::endl;
  
  dynamics::BodyNode* node = new dynamics::BodyNode(lk_name);
  
  // Mesh Loading
  //FIXME: Shouldn't mass and inertia default to 0?
  double mass = 0.1;
  Eigen::Matrix3d inertia = Eigen::MatrixXd::Identity(3,3);
  inertia *= 0.1;
  Eigen::Vector3d localCOM(0.0, 0.0, 0.0);
  
  // Load Inertial information
  if( _lk->inertial ) {
    boost::shared_ptr<urdf::Inertial>inert= (_lk->inertial);
    
    // Load mass
    mass = inert->mass;

    // Load Inertia matrix
    inertia(0,0) = inert->ixx;
    inertia(0,1) = -1*(inert->ixy);
    inertia(0,2) = -1*(inert->ixz);
    
    inertia(1,0) = -1*(inert->ixy);
    inertia(1,1) = (inert->iyy);
    inertia(1,2) = -1*(inert->iyz);
    
    inertia(2,0) = -1*(inert->ixz);
    inertia(2,1) = -1*(inert->iyz);
    inertia(2,2) = inert->izz;
    
    // Load local CoM
    localCOM << inert->origin.position.x, 
      inert->origin.position.y, 
      inert->origin.position.z;
    
    if( debug ) { std::cout<< "[debug] Mass is: "<< mass << std::endl; }
    if( debug ) { std::cout<< "[debug] Inertia is: \n"<< inertia << std::endl; }
  }

  // Set inertial information
  node->setMass(mass);
  node->setMomentOfInertia(inertia(0,0), inertia(1,1), inertia(2,2),
                           inertia(0,1), inertia(0,2), inertia(1,2));
  node->setCenterOfMass(localCOM);

  // Set visual information
  if( _lk->visual ) {
    if( add_VizShape( node, _lk->visual, _rootToSkelPath ) == false ) { std::cout<< "Error loading VizShape" <<std::endl; return NULL; }
  }
  else {
    // Set NULL
    dynamics::Shape* shape = NULL;
    node->setVisualizationShape( shape );
    if(debug) {  std::cout<< "No Visualization tag defined for node "<<node->getName() <<". Using NULL VizShape"<<std::endl; }
  }
  // Set collision information
  if( _lk->collision ) {
    if( add_ColShape( node, _lk->collision, _rootToSkelPath ) == false ) {  std::cout<< "Error loading ColShape" <<std::endl; return NULL; }
  }
  else {
    // Set NULL
    dynamics::Shape* shape = NULL;
    node->setCollisionShape( shape );
    if(debug) { std::cout<< "No Collision tag defined for node "<<node->getName() <<". Using NULL ColShape"<<std::endl; }
  }

  return node;
}

} // namespace dart
