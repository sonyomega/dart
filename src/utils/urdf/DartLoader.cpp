/**
 * @file DartLoader.cpp
 */

#include "DartLoader.h"
#include <map>

#include <urdf_parser/urdf_parser.h>
#include <urdf_world/world.h>
#include "urdf_world_parser.h"

#include <iostream>
#include <fstream>
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Joint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/PrismaticJoint.h"
#include "dynamics/WeldJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Shape.h"
#include "dynamics/BoxShape.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/CylinderShape.h"
#include "dynamics/MeshShape.h"
#include "simulation/World.h"

namespace dart {
namespace utils {

/**
 * @function parseSkeleton
 */
dynamics::Skeleton* DartLoader::parseSkeleton(std::string _urdfFileName) {

    boost::shared_ptr<urdf::ModelInterface> skeletonModel = urdf::parseURDF(readFileToString(_urdfFileName));
    if(!skeletonModel)
        return NULL;

    // Change path to a Unix-style path if given a Windows one
    // Windows can handle Unix-style paths (apparently)
    std::replace(_urdfFileName.begin(), _urdfFileName.end(), '\\' , '/' );
    std::string skelDirectory = _urdfFileName.substr(0, _urdfFileName.rfind("/") + 1);

    return modelInterfaceToSkeleton(skeletonModel, skelDirectory);
}

/**
 * @function parseWorld
 */
simulation::World* DartLoader::parseWorld(std::string _urdfFileName) {

    std::string xmlString = readFileToString(_urdfFileName);

    // Change path to a Unix-style path if given a Windows one
    // Windows can handle Unix-style paths (apparently)
    std::replace(_urdfFileName.begin(), _urdfFileName.end(), '\\' , '/');
    std::string worldDirectory = _urdfFileName.substr(0, _urdfFileName.rfind("/") + 1);
    
    boost::shared_ptr<urdf::World> worldInterface = urdf::parseWorldURDF(xmlString, worldDirectory);
    if(!worldInterface)
        return NULL;

    // Store paths from world to entities
    parseWorldToEntityPaths(xmlString);

    simulation::World* world = new simulation::World();

    for(unsigned int i = 0; i < worldInterface->models.size(); ++i) {
      std::string skeletonDirectory = worldDirectory + mWorld_To_Entity_Paths.find(worldInterface->models[i].model->getName())->second;
      dynamics::Skeleton* skeleton = modelInterfaceToSkeleton(worldInterface->models[i].model, skeletonDirectory);

      if(!skeleton) {
        std::cout << "[ERROR] Robot " << worldInterface->models[i].model->getName() << " was not correctly parsed. World is not loaded. Exiting!" << std::endl;
        return NULL; 
      }

      // Initialize position and RPY
      dynamics::Joint* rootJoint = skeleton->getRoot()->getParentJoint();
      urdf::Pose pose = worldInterface->models[i].origin;
      if(dynamic_cast<dynamics::FreeJoint*>(rootJoint)) {
          rootJoint->getGenCoord(0)->set_q(pose.position.x);
          rootJoint->getGenCoord(1)->set_q(pose.position.y);
          rootJoint->getGenCoord(2)->set_q(pose.position.z);
          double r, p, y;
          worldInterface->models[i].origin.rotation.getRPY(r, p, y);
          rootJoint->getGenCoord(3)->set_q(y);
          rootJoint->getGenCoord(4)->set_q(p);
          rootJoint->getGenCoord(5)->set_q(r);
      }
      else {
          rootJoint->setTransformFromParentBodyNode(toEigen(pose));
      }

      skeleton->initKinematics();
      skeleton->initDynamics();
      world->addSkeleton(skeleton);
    }
    return world;
}

/**
 * @function parseWorldToEntityPaths
 */
void DartLoader::parseWorldToEntityPaths(const std::string &_xml_string) {
    
    TiXmlDocument xml_doc;
    xml_doc.Parse(_xml_string.c_str());
    
    TiXmlElement *world_xml = xml_doc.FirstChildElement("world");
    
    if( !world_xml ) {
      return;
    }
        
    // Get all include filenames
    std::map<std::string, std::string> includedFiles;

    for( TiXmlElement* include_xml = world_xml->FirstChildElement("include");
      include_xml; include_xml = include_xml->NextSiblingElement("include") ) {
      
      const char *filename = include_xml->Attribute("filename");
      const char *model_name = include_xml->Attribute("model_name");
      std::string string_filename( filename );
      std::string string_filepath = string_filename.substr( 0, string_filename.rfind("/") + 1 );
      std::string string_model_name( model_name );

      includedFiles[string_model_name] = string_filepath;
    }
    
    // Get all entities
    for( TiXmlElement* entity_xml = world_xml->FirstChildElement("entity");
      entity_xml; entity_xml = entity_xml->NextSiblingElement("entity") ) {

      // Find model and name for entity, if not, error
      const char* entity_model = entity_xml->Attribute("model");
      const char* entity_name = entity_xml->Attribute("name");

      if( entity_name && entity_model ) {
        std::string string_entity_model( entity_model );  
        std::string string_entity_name( entity_name ); 
        // Find the model
        if( includedFiles.find( string_entity_model ) == includedFiles.end() ) {
          std::cout<<"[!] Did not find entity model included. Weird things may happen"<<std::endl;
          return;
        }
        // Add it
        else {
          std::string string_entity_filepath = includedFiles.find( string_entity_model )->second;
          mWorld_To_Entity_Paths[string_entity_name] = string_entity_filepath;
        }
      }
      // If no name or model is defined
      else {
        std::cout<< "[!] Entity was not defined. Weird things will happen" <<std::endl;
      }

    } // for all entities

}

/**
 * @function modelInterfaceToSkeleton
 * @brief Read the ModelInterface and spits out a Skeleton object
 */
dynamics::Skeleton* DartLoader::modelInterfaceToSkeleton(boost::shared_ptr<urdf::ModelInterface> _model, std::string _rootToSkelPath) {
  
    if( _rootToSkelPath.empty() ) {
        std::cout<< "[DartLoader] Absolute path to skeleton "<<_model->getName()<<" is not set. Probably I will crash!"<<std::endl;
    }

    dynamics::Skeleton* skeleton = new dynamics::Skeleton(_model->getName());
    dynamics::BodyNode* rootNode;
    dynamics::Joint* rootJoint;

    boost::shared_ptr<const urdf::Link> root = _model->getRoot();
    if(root->name == "world") {
        if(_model->getRoot()->child_links.size() != 1) { 
            std::cout<< "[ERROR] Not unique link attached to world." << std::endl; 
        }
        else {
            root = root->child_links[0];
            rootNode = createDartNode(root, _rootToSkelPath);
            rootJoint = createDartJoint(root->parent_joint, NULL, rootNode);
            if(!rootJoint)
                return NULL;
        }
    }
    else {
        rootNode = createDartNode(root, _rootToSkelPath);
        rootJoint = new dynamics::FreeJoint();
        rootJoint->setParentBodyNode(NULL);
        rootJoint->setChildBodyNode(rootNode);
        rootJoint->setName("rootJoint");
        rootJoint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
        rootJoint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
    }
    rootNode->setParentJoint(rootJoint);

    for(unsigned int i = 0; i < root->child_links.size(); i++) {
        createSkeletonRecursive(skeleton, root->child_links[i], rootNode, _rootToSkelPath);
    }
}

void DartLoader::createSkeletonRecursive(dynamics::Skeleton* _skel, boost::shared_ptr<const urdf::Link> _lk, dynamics::BodyNode* _parentNode, std::string _rootToSkelPath) {
  dynamics::BodyNode* node = createDartNode(_lk, _rootToSkelPath);
  dynamics::Joint* joint = createDartJoint(_lk->parent_joint, _parentNode, node);
  node->setParentJoint(joint);
  _parentNode->addChildJoint(joint);
  _skel->addBodyNode(node);
  _skel->addJoint(joint);
  
  for(unsigned int i = 0; i < _lk->child_links.size(); i++) {
      createSkeletonRecursive(_skel, _lk->child_links[i], node, _rootToSkelPath);
  }
}


/**
 * @function readXml
 */
std::string  DartLoader::readFileToString(std::string _xmlFile) {
  
  std::string xml_string;
  std::ifstream xml_file(_xmlFile.c_str());
  
  // Read xml
  while(xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  
  return xml_string;
}

/**
 * @function createDartJoint
 */
dynamics::Joint* DartLoader::createDartJoint(boost::shared_ptr<const urdf::Joint> _jt, dynamics::BodyNode* _parent, dynamics::BodyNode* _child)
{ 
  dynamics::Joint* joint;
  switch(_jt->type) {
  case urdf::Joint::REVOLUTE:
      joint = new dynamics::RevoluteJoint(_parent, _child, toEigen(_jt->axis));
      joint->getGenCoord(0)->set_qMin(_jt->limits->lower);
      joint->getGenCoord(1)->set_qMin(_jt->limits->upper);
  case urdf::Joint::CONTINUOUS:
      joint = new dynamics::RevoluteJoint(_parent, _child, toEigen(_jt->axis));
      break;
  case urdf::Joint::PRISMATIC:
      joint = new dynamics::PrismaticJoint(_parent, _child, toEigen(_jt->axis));
      joint->getGenCoord(0)->set_qMin(_jt->limits->lower);
      joint->getGenCoord(1)->set_qMin(_jt->limits->upper);
      break;
  case urdf::Joint::FIXED:
      joint = new dynamics::WeldJoint(_parent, _child);
      break;
  case urdf::Joint::FLOATING:
      joint = new dynamics::FreeJoint(_parent, _child);
      break;
  case urdf::Joint::PLANAR:
      std::cout << "Planar joint not supported." << std::endl;
      assert(false);
      return NULL;
  default:
      std::cout << "Unsupported joint type." << std::endl;
      assert(false);
      return NULL;
  }
  joint->setName(_jt->name);
  joint->setTransformFromParentBodyNode(toEigen(_jt->parent_to_joint_origin_transform));
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());

  return joint;
}

/**
 * @function createDartNode
 */
dynamics::BodyNode* DartLoader::createDartNode(boost::shared_ptr<const urdf::Link> _lk, std::string _rootToSkelPath ) {

  dynamics::BodyNode* node = new dynamics::BodyNode(_lk->name);
  
  // Mesh Loading
  //FIXME: Shouldn't mass and inertia default to 0?
  double mass = 0.1;
  Eigen::Matrix3d inertia = Eigen::MatrixXd::Identity(3,3);
  inertia *= 0.1;
  
  // Load Inertial information
  if(_lk->inertial) {
      node->setLocalCOM(toEigen(_lk->inertial->origin.position));
      node->setMass(_lk->inertial->mass);
      node->setMomentOfInertia(_lk->inertial->ixx, _lk->inertial->iyy, _lk->inertial->izz,
                               _lk->inertial->ixy, _lk->inertial->ixz, _lk->inertial->iyz);
  }

  // Set visual information
  for(unsigned int i = 0; i < _lk->visual_array.size(); i++) {
    if(dynamics::Shape* shape = createShape(_lk->visual_array[i], _rootToSkelPath)) {
      node->addVisualizationShape(shape);
    }
  }

  // Set collision information
  for(unsigned int i = 0; i < _lk->collision_array.size(); i++) {
    if(dynamics::Shape* shape = createShape(_lk->collision_array[i], _rootToSkelPath)) {
      node->addCollisionShape(shape);
    }
  }

  return node;
}


void setMaterial(dynamics::Shape* _shape, const urdf::Visual* _viz) {
  if(_viz->material) {
    _shape->setColor(Eigen::Vector3d(_viz->material->color.r, _viz->material->color.g, _viz->material->color.b));
  }
}

void setMaterial(dynamics::Shape* _shape, const urdf::Collision* _col) {
}

/**
 * @function createShape
 */
template <class VisualOrCollision>
dynamics::Shape* DartLoader::createShape(boost::shared_ptr<VisualOrCollision> _vizOrCol, std::string _rootToSkelPath) {
  dynamics::Shape* shape;

  // Sphere
  if(urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(_vizOrCol->geometry.get())) {
    shape = new dynamics::EllipsoidShape(2.0 * sphere->radius * Eigen::Vector3d::Ones());
  }

  // Box
  else if(urdf::Box* box = dynamic_cast<urdf::Box*>(_vizOrCol->geometry.get())) {
    shape = new dynamics::BoxShape(Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z));
  }

  // Cylinder
  else if(urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(_vizOrCol->geometry.get())) {
    shape = new dynamics::CylinderShape(cylinder->radius, cylinder->length);
  }

  // Mesh
  else if(urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(_vizOrCol->geometry.get())) {
    std::string fullPath = _rootToSkelPath + mesh->filename;
    const aiScene* model = dynamics::MeshShape::loadMesh( fullPath );
    
    if(!model) {
      std::cout<< "[add_Shape] [ERROR] Not loading model " << fullPath << " (NULL) \n";
      shape = NULL;
    } 
    else {
      shape = new dynamics::MeshShape(Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z), model);
    }
  }

  // Unknown geometry type
  else {
    std::cout << "[add_Shape] No MESH, BOX, CYLINDER OR SPHERE! Not loading this shape." << std::endl;
    return NULL;
  }

  shape->setLocalTransform(toEigen(_vizOrCol->origin));
  setMaterial(shape, _vizOrCol.get());
  return shape;
}

template dynamics::Shape* DartLoader::createShape<urdf::Visual>(boost::shared_ptr<urdf::Visual> _vizOrCol, std::string _rootToSkelPath);
template dynamics::Shape* DartLoader::createShape<urdf::Collision>(boost::shared_ptr<urdf::Collision> _vizOrCol, std::string _rootToSkelPath);

/**
 * @function pose2Affine3d
 */
Eigen::Isometry3d DartLoader::toEigen(const urdf::Pose& _pose) {
    Eigen::Quaterniond quat;
    _pose.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    Eigen::Isometry3d transform(quat);
    transform.translation() = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
    return transform;
}

Eigen::Vector3d DartLoader::toEigen(const urdf::Vector3& _vector) {
    return Eigen::Vector3d(_vector.x, _vector.y, _vector.z);
}

} // namespace utils
} // namespace dart
