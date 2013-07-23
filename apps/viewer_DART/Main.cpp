#include "common/Paths.h"
#include "utils/SkelParser.h"
#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World *myWorld
            = dart::simulation::readSkelFile(
                  DART_DATA_PATH"/skel/test/single_pendulum.skel");
                  //DART_DATA_PATH"/skel/test/double_pendulum.skel");
                  //DART_DATA_PATH"/skel/test/boxes.skel");
                  //DART_DATA_PATH"/skel/test/ball_joints.skel");
                  //DART_DATA_PATH"/skel/test/translational_joints.skel");
                  //DART_DATA_PATH"/skel/test/free_joints.skel");
                  //DART_DATA_PATH"/skel/test/serial_chain.skel");
                  //DART_DATA_PATH"/skel/test/drop.skel");
                  //DART_DATA_PATH"/skel/test/drop_unrotated_box.skel");
                  //DART_DATA_PATH"/skel/test/SimplePendulum.skel");
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
    window.initWindow(640, 480, "Boxes");
    glutMainLoop();

    return 0;
}