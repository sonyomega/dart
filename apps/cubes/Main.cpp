#include "MyWindow.h"
#include "dynamics/Skeleton.h"
#include "utils/SkelParser.h"
#include "utils/Paths.h"

using namespace dart;
using namespace dynamics;
using namespace simulation;
using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
    // create and initialize the world
    dart::simulation::World *myWorld
            = dart::simulation::readSkelFile(
                  DART_DATA_PATH"/skel/cubes.skel");
    assert(myWorld != NULL);
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1'--'4': programmed interaction" << endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Boxes");
    glutMainLoop();

    return 0;
}
