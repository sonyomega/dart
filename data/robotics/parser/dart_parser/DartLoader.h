/**
 * @file DartLoader.h
 */

#ifndef DART_LOADER_H
#define DART_LOADER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <robotics/parser/urdfdom_headers/urdf_model/pose.h>
#include <robotics/parser/urdfdom_headers/urdf_model/link.h>
#include <robotics/parser/urdfdom_headers/urdf_model/color.h>

const bool debug = false;

namespace urdf {
class ModelInterface;
class Link;
class Joint;
}

namespace dart {

namespace dynamics {
class Joint;
class Skeleton;
class BodyNode;
}

namespace simulation {
class World;
}

// Type of DOF Enum
enum TypeOfDOF {
  GOLEM_X, GOLEM_Y, GOLEM_Z, 
  GOLEM_ROLL, GOLEM_PITCH, GOLEM_YAW,
  GOLEM_ARBITRARY_ROTATION,
  GOLEM_FLOATING
};

/**
 * @class DartLoader
 */
class DartLoader {
  
 public:
  DartLoader();
  ~DartLoader();
  
  dynamics::Skeleton* parseSkeleton( std::string _urdfFile,
					     std::string _rootToSkelPath = NULL );
  
  dynamics::Skeleton* parseRobot( std::string _urdfFile,
			       std::string _rootToRobotPath = NULL );
  dynamics::Skeleton* parseObject( std::string _urdfFile,
				 std::string _rootToObjectPath = NULL );
  simulation::World* parseWorld( std::string _urdfFile );
  
  void parseWorldToEntityPaths( const std::string &_xml_string );

  dynamics::Skeleton* modelInterfaceToSkeleton( boost::shared_ptr<urdf::ModelInterface> _model,
							std::string _rootToSkelPath = NULL );
  dynamics::Skeleton* modelInterfaceToRobot( boost::shared_ptr<urdf::ModelInterface> _model,
					  std::string _rootToRobotPath = NULL );
  dynamics::Skeleton* modelInterfaceToObject( boost::shared_ptr<urdf::ModelInterface> _model,
					   std::string _rootToObjectPath = NULL );
  
  // Utilities
  dynamics::BodyNode* getNode( std::string _nodeName );
  std::string readXmlToString( std::string _xmlFile );

  // Loader utils
//  void add_DOF(dynamics::Skeleton* _skel,
//           dynamics::Joint* _joint,
//	       double _val, double _min, double _max,
//	       int _DOF_TYPE,
//	       double _x = 0, double _y = 0, double _z = 0 );

  bool add_VizShape( dynamics::BodyNode* _node,
		     boost::shared_ptr<urdf::Visual> _viz,
		     std::string  _rootToSkelPath );
  bool add_ColShape( dynamics::BodyNode* _node,
		     boost::shared_ptr<urdf::Collision> _col,
		     std::string _rootToSkelPath );

  // ToDart utils
  dynamics::Joint* createDartRootJoint( boost::shared_ptr<urdf::Joint> _jt,
                      dynamics::Skeleton* _skel );
  dynamics::Joint* createNewDartRootJoint( dynamics::BodyNode* _node,
                         dynamics::Skeleton* _skel );


  dynamics::Joint* createDartJoint( boost::shared_ptr<urdf::Joint> _jt,
                      dynamics::Skeleton* _skel );
  dynamics::BodyNode* createDartNode( boost::shared_ptr<urdf::Link> _lk,
                          dynamics::Skeleton* _skel,
					      std::string _rootToSkelPath = NULL ); 

  // Useful helpers
  Eigen::Affine3d pose2Affine3d( urdf::Pose _pose );

  // Member variables
  std::vector<dynamics::BodyNode*> mNodes;
  std::vector<dynamics::Joint*> mJoints;
  
  std::string mRoot_To_World_Path;
  std::map<std::string, std::string> mWorld_To_Entity_Paths;
  
};

} // namespace dart

#endif /** DART_LOADER_H */
