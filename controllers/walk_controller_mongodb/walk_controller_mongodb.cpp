// walk_controller_mongodb.cpp - MongoDB 기능 추가 버전
#include "walk_controller_mongodb.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

#ifdef USE_MONGODB
// MongoDB 유틸리티 함수들
string actionTypeToString(ActionType action) {
    switch(action) {
        case FORWARD: return "FORWARD";
        case BACKWARD: return "BACKWARD";
        case LEFT: return "LEFT";
        case RIGHT: return "RIGHT";
        case FORWARD_LEFT: return "FORWARD_LEFT";
        case FORWARD_RIGHT: return "FORWARD_RIGHT";
        case BACKWARD_LEFT: return "BACKWARD_LEFT";
        case BACKWARD_RIGHT: return "BACKWARD_RIGHT";
        case IDLE: return "IDLE";
        default: return "UNKNOWN";
    }
}

ActionType stringToActionType(const string& actionStr) {
    if (actionStr == "FORWARD") return FORWARD;
    if (actionStr == "BACKWARD") return BACKWARD;
    if (actionStr == "LEFT") return LEFT;
    if (actionStr == "RIGHT") return RIGHT;
    if (actionStr == "FORWARD_LEFT") return FORWARD_LEFT;
    if (actionStr == "FORWARD_RIGHT") return FORWARD_RIGHT;
    if (actionStr == "BACKWARD_LEFT") return BACKWARD_LEFT;
    if (actionStr == "BACKWARD_RIGHT") return BACKWARD_RIGHT;
    return IDLE;
}

// MongoDBController 구현
MongoDBController::MongoDBController() : connected(false) {
    try {
        instance = make_unique<mongocxx::instance>();
        mongocxx::uri uri{"mongodb://localhost:27017"};
        client = make_unique<mongocxx::client>(uri);
        
        database = (*client)["robot_control"];
        collection = database["current_action"];
        
        // 연결 테스트
        auto ping_cmd = bsoncxx::builder::stream::document{} << "ping" << 1 << bsoncxx::builder::stream::finalize;
        database.run_command(ping_cmd.view());
        
        connected = true;
        cout << "[MongoDB] 연결 성공!" << endl;
        
    } catch (const exception& e) {
        cout << "[MongoDB] 연결 실패: " << e.what() << endl;
        connected = false;
    }
}

optional<ActionType> MongoDBController::getCurrentAction() {
    if (!connected) return nullopt;
    
    try {
        auto opts = mongocxx::options::find{};
        opts.sort(bsoncxx::builder::stream::document{} << "timestamp" << -1 << bsoncxx::builder::stream::finalize);
        opts.limit(1);
        
        auto cursor = collection.find({}, opts);
        
        for (auto&& doc : cursor) {
            auto action_element = doc["action"];
            if (action_element && action_element.type() == bsoncxx::type::k_utf8) {
                string action_str = action_element.get_utf8().value.to_string();
                return stringToActionType(action_str);
            }
        }
        
        return nullopt;
        
    } catch (const exception& e) {
        cout << "[MongoDB] 읽기 에러: " << e.what() << endl;
        return nullopt;
    }
}

bool MongoDBController::saveAction(ActionType action, const string& source) {
    if (!connected) return false;
    
    try {
        auto timestamp = getCurrentTimestamp();
        
        auto doc = bsoncxx::builder::stream::document{}
            << "action" << actionTypeToString(action)
            << "timestamp" << bsoncxx::types::b_date{chrono::milliseconds{timestamp}}
            << "source" << source
            << "robot_id" << "webots_op2"
            << bsoncxx::builder::stream::finalize;
        
        auto result = collection.insert_one(doc.view());
        
        if (result) {
            cout << "[MongoDB] 액션 저장: " << actionTypeToString(action) << endl;
            return true;
        }
        
        return false;
        
    } catch (const exception& e) {
        cout << "[MongoDB] 저장 실패: " << e.what() << endl;
        return false;
    }
}

ActionType MongoDBController::stringToActionType(const string& actionStr) {
    return ::stringToActionType(actionStr);
}

string MongoDBController::actionTypeToString(ActionType action) {
    return ::actionTypeToString(action);
}

int64_t MongoDBController::getCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch());
    return timestamp.count();
}
#endif // USE_MONGODB

// Walk 클래스 구현
Walk::Walk() : Robot() {
    mTimeStep = getBasicTimeStep();
    
    // 기존 초기화
    getLED("HeadLed")->set(0xFF0000);
    getLED("EyeLed")->set(0x00FF00);
    
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);
    
    mGyro = getGyro("Gyro");
    mGyro->enable(mTimeStep);
    
    for (int i = 0; i < NMOTORS; i++) {
        mMotors[i] = getMotor(motorNames[i]);
        string sensorName = motorNames[i];
        sensorName.push_back('S');
        mPositionSensors[i] = getPositionSensor(sensorName);
        mPositionSensors[i]->enable(mTimeStep);
    }
    
    mKeyboard = getKeyboard();
    mKeyboard->enable(mTimeStep);
    
    mMotionManager = new RobotisOp2MotionManager(this);
    mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
    
#ifdef USE_MONGODB
    // MongoDB 관련 초기화 (조건부)
    mMongoController = make_unique<MongoDBController>();
    mIsWalking = false;
    mLastAction = IDLE;
    mActionCheckCounter = 0;
    initializeActionMapper();
#endif
}

Walk::~Walk() {
    delete mMotionManager;
    delete mGaitManager;
#ifdef USE_MONGODB
    // mMongoController는 unique_ptr이므로 자동 해제
#endif
}

#ifdef USE_MONGODB
void Walk::initializeActionMapper() {
    // x_amplitude, a_amplitude
    mActionMap[FORWARD] = make_pair(1.0, 0.0);
    mActionMap[BACKWARD] = make_pair(-1.0, 0.0);
    mActionMap[LEFT] = make_pair(0.0, 0.5);
    mActionMap[RIGHT] = make_pair(0.0, -0.5);
    mActionMap[FORWARD_LEFT] = make_pair(0.7, 0.3);
    mActionMap[FORWARD_RIGHT] = make_pair(0.7, -0.3);
    mActionMap[BACKWARD_LEFT] = make_pair(-0.7, 0.3);
    mActionMap[BACKWARD_RIGHT] = make_pair(-0.7, -0.3);
    mActionMap[IDLE] = make_pair(0.0, 0.0);
}

void Walk::executeAction(ActionType action) {
    if (mLastAction == action) return;
    
    cout << "[Action] 실행: " << actionTypeToString(action) << endl;
    
    auto it = mActionMap.find(action);
    if (it != mActionMap.end()) {
        double xAmplitude = it->second.first;
        double aAmplitude = it->second.second;
        
        if (action == IDLE) {
            if (mIsWalking) {
                mGaitManager->stop();
                mIsWalking = false;
            }
        } else {
            if (!mIsWalking) {
                mGaitManager->start();
                mIsWalking = true;
            }
            mGaitManager->setXAmplitude(xAmplitude);
            mGaitManager->setAAmplitude(aAmplitude);
        }
    }
    
    mLastAction = action;
}

void Walk::readAndExecuteFromMongoDB() {
    if (mMongoController && mMongoController->isConnected()) {
        auto actionOpt = mMongoController->getCurrentAction();
        if (actionOpt.has_value()) {
            executeAction(actionOpt.value());
        }
    }
}
#endif // USE_MONGODB

void Walk::myStep() {
    int ret = step(mTimeStep);
    if (ret == -1)
        exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
    double startTime = getTime();
    double s = (double)ms / 1000.0;
    while (s + startTime >= getTime())
        myStep();
}

void Walk::run() {
    cout << "=== ROBOTIS OP2 Walk Controller ===" << endl;
#ifdef USE_MONGODB
    cout << "MongoDB 지원: 활성화" << endl;
    cout << "키보드 제어: W/A/S/D (방향), Space (정지), 1-8 (테스트)" << endl;
#else
    cout << "MongoDB 지원: 비활성화" << endl;
    cout << "기본 키보드 제어만 사용" << endl;
#endif
    
    myStep();
    
    // 초기 위치
    mMotionManager->playPage(9);
    wait(200);
    
    while (true) {
        checkIfFallen();
        
#ifdef USE_MONGODB
        // MongoDB 액션 체크 (10번에 한 번)
        mActionCheckCounter++;
        if (mActionCheckCounter >= 10) {
            readAndExecuteFromMongoDB();
            mActionCheckCounter = 0;
        }
#endif
        
        // 키보드 입력 처리
        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
#ifdef USE_MONGODB
            ActionType keyAction = IDLE;
            bool saveToMongo = false;
            
            switch (key) {
                case ' ':
                    keyAction = IDLE;
                    saveToMongo = true;
                    break;
                case 'W':
                case 'w':
                case Keyboard::UP:
                    keyAction = FORWARD;
                    saveToMongo = true;
                    break;
                case 'S':
                case 's':
                case Keyboard::DOWN:
                    keyAction = BACKWARD;
                    saveToMongo = true;
                    break;
                case 'A':
                case 'a':
                case Keyboard::LEFT:
                    keyAction = LEFT;
                    saveToMongo = true;
                    break;
                case 'D':
                case 'd':
                case Keyboard::RIGHT:
                    keyAction = RIGHT;
                    saveToMongo = true;
                    break;
                case '1': keyAction = FORWARD; saveToMongo = true; break;
                case '2': keyAction = BACKWARD; saveToMongo = true; break;
                case '3': keyAction = LEFT; saveToMongo = true; break;
                case '4': keyAction = RIGHT; saveToMongo = true; break;
                case '5': keyAction = FORWARD_LEFT; saveToMongo = true; break;
                case '6': keyAction = FORWARD_RIGHT; saveToMongo = true; break;
                case '7': keyAction = BACKWARD_LEFT; saveToMongo = true; break;
                case '8': keyAction = BACKWARD_RIGHT; saveToMongo = true; break;
                case '0': keyAction = IDLE; saveToMongo = true; break;
            }
            
            if (saveToMongo) {
                executeAction(keyAction);
                if (mMongoController && mMongoController->isConnected()) {
                    mMongoController->saveAction(keyAction, "keyboard");
                }
            }
#else
            // MongoDB 없는 기본 키보드 처리
            switch (key) {
                case ' ':
                    cout << "[기본] 정지" << endl;
                    mGaitManager->stop();
                    break;
                case 'W':
                case 'w':
                case Keyboard::UP:
                    cout << "[기본] 전진" << endl;
                    mGaitManager->start();
                    mGaitManager->setXAmplitude(1.0);
                    mGaitManager->setAAmplitude(0.0);
                    break;
                // 기타 기본 동작들...
            }
#endif
        }
        
#ifdef USE_MONGODB
        // 걷기 동작 업데이트
        if (mIsWalking) {
            mGaitManager->step(mTimeStep);
        }
#endif
        
        myStep();
    }
}

void Walk::checkIfFallen() {
    static int fup = 0;
    static int fdown = 0;
    static const double acc_tolerance = 80.0;
    static const double acc_step = 100;
    
    const double *acc = mAccelerometer->getValues();
    if (acc[1] < 512.0 - acc_tolerance)
        fup++;
    else
        fup = 0;
    
    if (acc[1] > 512.0 + acc_tolerance)
        fdown++;
    else
        fdown = 0;
    
    if (fup > acc_step) {
        cout << "[낙상감지] 앞으로 넘어짐!" << endl;
        mMotionManager->playPage(10); // get up front
        mMotionManager->playPage(9);  // go to walking position
        fup = 0;
    }
    else if (fdown > acc_step) {
        cout << "[낙상감지] 뒤로 넘어짐!" << endl;
        mMotionManager->playPage(11); // get up back
        mMotionManager->playPage(9);  // go to walking position
        fdown = 0;
    }
}
