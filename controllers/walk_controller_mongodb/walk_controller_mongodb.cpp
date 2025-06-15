// walk_controller_mongodb.cpp
// MongoDB integrated Walk Controller for Webots (C API Version)

#include "walk_controller_mongodb.hpp"  // 헤더명 통일!

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
#include <iostream>
#include <string>
#include <cstring>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

// ActionType 변환 함수들
const char* actionTypeToString(ActionType action) {
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

ActionType stringToActionType(const char* actionStr) {
    if (strcmp(actionStr, "FORWARD") == 0) return FORWARD;
    if (strcmp(actionStr, "BACKWARD") == 0) return BACKWARD;
    if (strcmp(actionStr, "LEFT") == 0) return LEFT;
    if (strcmp(actionStr, "RIGHT") == 0) return RIGHT;
    if (strcmp(actionStr, "FORWARD_LEFT") == 0) return FORWARD_LEFT;
    if (strcmp(actionStr, "FORWARD_RIGHT") == 0) return FORWARD_RIGHT;
    if (strcmp(actionStr, "BACKWARD_LEFT") == 0) return BACKWARD_LEFT;
    if (strcmp(actionStr, "BACKWARD_RIGHT") == 0) return BACKWARD_RIGHT;
    return IDLE;
}

// CMongoController 구현
CMongoController::CMongoController() {
    // MongoDB C 드라이버 초기화
    mongoc_init();
    
    // 클라이언트 생성
    client = mongoc_client_new("mongodb://localhost:27017");
    if (!client) {
        cout << "MongoDB 클라이언트 생성 실패!" << endl;
        connected = false;
        return;
    }
    
    // 데이터베이스 및 컬렉션 설정
    database = mongoc_client_get_database(client, "robot_control");
    collection = mongoc_client_get_collection(client, "robot_control", "current_action");
    
    connected = true;
    cout << "MongoDB C API 연결 성공!" << endl;
}

CMongoController::~CMongoController() {
    if (connected) {
        mongoc_collection_destroy(collection);
        mongoc_database_destroy(database);
        mongoc_client_destroy(client);
        mongoc_cleanup();
    }
}

ActionType CMongoController::getCurrentAction() {
    if (!connected) return IDLE;
    
    bson_t *query = bson_new();
    bson_t *opts = bson_new();
    
    // 최신 문서 찾기 위한 정렬 옵션
    bson_t sort;
    bson_init(&sort);
    BSON_APPEND_INT32(&sort, "timestamp", -1);
    
    bson_t limit;
    bson_init(&limit);
    BSON_APPEND_INT32(&limit, "limit", 1);
    
    // 옵션 설정
    BSON_APPEND_DOCUMENT(opts, "sort", &sort);
    BSON_APPEND_DOCUMENT(opts, "limit", &limit);
    
    // 쿼리 실행
    mongoc_cursor_t *cursor = mongoc_collection_find_with_opts(collection, query, opts, NULL);
    
    ActionType result = IDLE;
    const bson_t *doc;
    
    if (mongoc_cursor_next(cursor, &doc)) {
        bson_iter_t iter;
        
        if (bson_iter_init_find(&iter, doc, "action")) {
            const char *action_str = bson_iter_utf8(&iter, NULL);
            result = stringToActionType(action_str);
        }
    } else {
        // 에러 체크
        bson_error_t error;
        if (mongoc_cursor_error(cursor, &error)) {
            cout << "MongoDB 쿼리 에러: " << error.message << endl;
        }
    }
    
    // 메모리 정리
    mongoc_cursor_destroy(cursor);
    bson_destroy(query);
    bson_destroy(opts);
    bson_destroy(&sort);
    bson_destroy(&limit);
    
    return result;
}

bool CMongoController::saveAction(ActionType action) {
    if (!connected) return false;
    
    bson_t *doc = bson_new();
    bson_error_t error;
    
    // 현재 시간을 타임스탬프로 사용
    int64_t timestamp = time(NULL) * 1000; // milliseconds
    
    // 문서 생성
    BSON_APPEND_UTF8(doc, "action", actionTypeToString(action));
    BSON_APPEND_DATE_TIME(doc, "timestamp", timestamp);
    
    // 삽입 실행
    bool success = mongoc_collection_insert_one(collection, doc, NULL, NULL, &error);
    
    if (success) {
        cout << "액션 저장 성공: " << actionTypeToString(action) << endl;
    } else {
        cout << "액션 저장 실패: " << error.message << endl;
    }
    
    bson_destroy(doc);
    return success;
}

ActionType CMongoController::stringToActionType(const char* actionStr) {
    return ::stringToActionType(actionStr); // 전역 함수 호출
}

const char* CMongoController::actionTypeToString(ActionType action) {
    return ::actionTypeToString(action); // 전역 함수 호출
}

// Walk 클래스 구현
Walk::Walk() : Robot() {
    mTimeStep = getBasicTimeStep();
    mIsWalking = false;
    mLastAction = IDLE;
    mActionCheckCounter = 0;
    
    getLED("HeadLed")->set(0xFF0000);
    getLED("EyeLed")->set(0x00FF00);
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);
    
    getGyro("Gyro")->enable(mTimeStep);
    
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
    
    // MongoDB 컨트롤러 초기화
    mMongoController = new CMongoController();
    
    // 액션 매핑 초기화
    initializeActionMapper();
}

Walk::~Walk() {
    delete mMongoController;
    delete mMotionManager;
    delete mGaitManager;
}

void Walk::initializeActionMapper() {
    mActionMap[FORWARD] = GaitParams(1.0, 0.0, true);
    mActionMap[BACKWARD] = GaitParams(-1.0, 0.0, true);
    mActionMap[LEFT] = GaitParams(0.0, 0.5, true);
    mActionMap[RIGHT] = GaitParams(0.0, -0.5, true);
    mActionMap[FORWARD_LEFT] = GaitParams(0.7, 0.3, true);
    mActionMap[FORWARD_RIGHT] = GaitParams(0.7, -0.3, true);
    mActionMap[BACKWARD_LEFT] = GaitParams(-0.7, 0.3, true);
    mActionMap[BACKWARD_RIGHT] = GaitParams(-0.7, -0.3, true);
    mActionMap[IDLE] = GaitParams(0.0, 0.0, false);
}

void Walk::executeAction(ActionType action) {
    if (mLastAction == action) return; // 같은 액션이면 무시
    
    cout << "액션 실행: " << actionTypeToString(action) << endl;
    
    auto it = mActionMap.find(action);
    if (it != mActionMap.end()) {
        const GaitParams& params = it->second;
        
        if (params.startGait && !mIsWalking) {
            mGaitManager->start();
            mIsWalking = true;
        } else if (!params.startGait && mIsWalking) {
            mGaitManager->stop();
            mIsWalking = false;
        }
        
        mGaitManager->setXAmplitude(params.xAmplitude);
        mGaitManager->setAAmplitude(params.aAmplitude);
    }
    
    mLastAction = action;
}

void Walk::readAndExecuteFromMongoDB() {
    if (mMongoController && mMongoController->isConnected()) {
        ActionType currentAction = mMongoController->getCurrentAction();
        executeAction(currentAction);
    }
}

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
    cout << "-------ROBOTIS OP2 MongoDB C API 제어-------" << endl;
    cout << "MongoDB에서 ActionType을 읽어 자동 제어" << endl;
    cout << "키보드 제어:" << endl;
    cout << "스페이스바: 정지, 화살표키: 방향, 숫자키: 테스트" << endl;
    
    myStep();
    
    // 초기 위치
    mMotionManager->playPage(9);
    wait(200);
    
    while (true) {
        checkIfFallen();
        
        // 10번에 한 번씩만 MongoDB 체크 (성능 최적화)
        mActionCheckCounter++;
        if (mActionCheckCounter >= 10) {
            readAndExecuteFromMongoDB();
            mActionCheckCounter = 0;
        }
        
        // 키보드 수동 제어
        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
            ActionType keyAction = IDLE;
            bool saveToMongo = false;
            
            switch (key) {
                case ' ':
                    keyAction = IDLE;
                    saveToMongo = true;
                    break;
                case Keyboard::UP:
                    keyAction = FORWARD;
                    saveToMongo = true;
                    break;
                case Keyboard::DOWN:
                    keyAction = BACKWARD;
                    saveToMongo = true;
                    break;
                case Keyboard::LEFT:
                    keyAction = LEFT;
                    saveToMongo = true;
                    break;
                case Keyboard::RIGHT:
                    keyAction = RIGHT;
                    saveToMongo = true;
                    break;
                // 테스트용 숫자 키
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
                    mMongoController->saveAction(keyAction);
                }
            }
        }
        
        if (mIsWalking) {
            mGaitManager->step(mTimeStep);
        }
        
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
        mMotionManager->playPage(10);
        mMotionManager->playPage(9);
        fup = 0;
    }
    else if (fdown > acc_step) {
        mMotionManager->playPage(11);
        mMotionManager->playPage(9);
        fdown = 0;
    }
}
