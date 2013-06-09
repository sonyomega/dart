#include "utils/Paths.h"
#include "simulation/ParserDART.h"
#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World *myWorld
            = dart::simulation::readDARTFile(
                  //DART_DATA_PATH"/dart/single_pendulum.dart");
                  //DART_DATA_PATH"/dart/double_pendulum.dart");
                  //DART_DATA_PATH"/dart/boxes.dart");
                  //DART_DATA_PATH"/dart/test/ball_joints.dart");
                  //DART_DATA_PATH"/dart/test/translational_joints.dart");
                  //DART_DATA_PATH"/dart/test/free_joints.dart");
                  //DART_DATA_PATH"/dart/test/serial_chain.dart");
                  //DART_DATA_PATH"/dart/test/drop.dart");
                  DART_DATA_PATH"/dart/test/drop_unrotated_box.dart");
                  //DART_DATA_PATH"/dart/test/SimplePendulum.dart");
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
