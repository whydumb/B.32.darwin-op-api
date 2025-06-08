// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Description: MongoDB 실시간 명령 처리 컨트롤러 메인 함수

#include "walk_controller_mongodb.hpp"

#include <cstdlib>
#include <iostream>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
    cout << "Starting ROBOTIS OP2 MongoDB Real-Time Controller..." << endl;
    
    try {
        WalkMongoDB *controller = new WalkMongoDB();
        controller->run();
        delete controller;
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }
    catch (...) {
        cerr << "Unknown error occurred" << endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}