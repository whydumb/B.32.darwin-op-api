#include "walk_controller_mongodb.hpp"
#include <cstdlib>

using namespace webots;

int main(int argc, char **argv) {
  WalkMongoDB *controller = new WalkMongoDB();
  controller->run();
  delete controller;
  return EXIT_FAILURE;
}
