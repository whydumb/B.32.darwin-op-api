// main.cpp - MongoDB version (fixed)
#include "walk_controller_mongodb.hpp"
#include <cstdlib>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    cout << "MongoDB Integrated ROBOTIS OP2 Controller Starting..." << endl;
    
    Walk *controller = new Walk();
    
    try {
        controller->run();
    } catch (const exception& e) {
        cout << "Error occurred: " << e.what() << endl;
        delete controller;
        return EXIT_FAILURE;
    }
    
    delete controller;
    cout << "Controller terminated" << endl;
    
    return EXIT_SUCCESS;
}
