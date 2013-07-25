#include "dart/common/Paths.h"
#include "dart/math/UtilsMath.h"
#include "dart/utils/SkelParser.h"
#include "dart/dynamics/Skeleton.h"
#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World *myWorld
            = dart::simulation::readSkelFile(
                  DART_DATA_PATH"/skel/test/spheres.skel");
    assert(myWorld != NULL);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'1'--'4': programmed interaction" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Spheres");
    glutMainLoop();

    return 0;
}
