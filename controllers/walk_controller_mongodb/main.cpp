// main.cpp - MongoDB 버전 (수정됨)
#include "walk_controller_mongodb.hpp"
#include <cstdlib>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    cout << "MongoDB 통합 ROBOTIS OP2 컨트롤러 시작..." << endl;
    
    Walk *controller = new Walk();
    
    try {
        controller->run();
    } catch (const exception& e) {
        cout << "에러 발생: " << e.what() << endl;
        delete controller;
        return EXIT_FAILURE;
    }
    
    delete controller;
    cout << "컨트롤러 종료" << endl;
    
    return EXIT_SUCCESS;
}
