#include "walk_controller_mongodb.hpp"
#include <cstdlib>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    MongoDBWalkController *controller = new MongoDBWalkController();
    
    // MongoDB API에서 모션 데이터 로드
    string api_url = "http://localhost:3000/api";
    string motion_id = "4ea4dde7-4669-4cf0-9356-c16d57582f99";
    
    cout << "Loading motion from API..." << endl;
    controller->loadMotionFromAPI(api_url, motion_id);
    
    cout << "Starting MongoDB controlled walk..." << endl;
    controller->runWithMongoDB();
    
    delete controller;
    return EXIT_SUCCESS;
}
