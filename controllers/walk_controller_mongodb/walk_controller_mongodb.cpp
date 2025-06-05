// walk_controller_mongodb.cpp - Replay 구현
#include "walk_controller_mongodb.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
    "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
    "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
    "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
    "AnkleL", "FootR", "FootL", "Neck", "Head"
};

WalkMongoDB::WalkMongoDB() : Robot() {
    mTimeStep = getBasicTimeStep();
    isWalking = false;
    isReplayActive = false;
    currentActionIndex = 0;
    pollInterval = 1000; // 1초마다 폴링
    
    // cURL 초기화
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    robotId = "robot_001";
    currentPlayerId = "";

    // Robot 하드웨어 초기화
    getLED("HeadLed")->set(0xFF0000);
    getLED("EyeLed")->set(0x00FF00);
    
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);
    
    mGyro = getGyro("Gyro");
    mGyro->enable(mTimeStep);
    
    mCamera = getCamera("Camera");
    mCamera->enable(mTimeStep);

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
    
    loadConfig();
    initializeMongoDB();
}

WalkMongoDB::~WalkMongoDB() {
    if (curl) {
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
    
    delete mMotionManager;
    delete mGaitManager;
}

void WalkMongoDB::loadConfig() {
    ifstream configFile("config.ini");
    string line;
    
    mongoApiUrl = "http://localhost:3000/api";
    
    while (getline(configFile, line)) {
        if (line.find("api_url") != string::npos) {
            size_t pos = line.find("=");
            if (pos != string::npos) {
                mongoApiUrl = line.substr(pos + 1);
                mongoApiUrl.erase(0, mongoApiUrl.find_first_not_of(" \t"));
                mongoApiUrl.erase(mongoApiUrl.find_last_not_of(" \t") + 1);
            }
        }
    }
    
    cout << "MongoDB API URL: " << mongoApiUrl << endl;
}

void WalkMongoDB::initializeMongoDB() {
    // 로봇 상태 등록
    Json::Value robotStatus;
    robotStatus["robotId"] = robotId;
    robotStatus["status"] = "online";
    robotStatus["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, robotStatus);
    
    string url = mongoApiUrl + "/robots/status";
    HTTPResponse response = httpRequest(url, "POST", jsonString);
    
    if (response.responseCode == 200) {
        cout << "Successfully registered robot status with MongoDB" << endl;
    } else {
        cout << "Failed to register robot status: " << response.responseCode << endl;
    }
}

size_t WalkMongoDB::WriteCallback(void* contents, size_t size, size_t nmemb, HTTPResponse* response) {
    size_t totalSize = size * nmemb;
    response->data.append((char*)contents, totalSize);
    return totalSize;
}

HTTPResponse WalkMongoDB::httpRequest(const string& url, const string& method, const string& postData) {
    HTTPResponse response;
    response.responseCode = 0;
    
    if (!curl) {
        return response;
    }
    
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    
    if (method == "POST") {
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
    } else if (method == "PUT") {
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
    }
    
    CURLcode res = curl_easy_perform(curl);
    
    if (res == CURLE_OK) {
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.responseCode);
    }
    
    curl_slist_free_all(headers);
    return response;
}

bool WalkMongoDB::loadReplay(const string& replayName, const string& playerId) {
    // MongoDB에서 리플레이 액션들 조회
    string url = mongoApiUrl + "/replays/" + replayName;
    if (!playerId.empty()) {
        url += "?player_id=" + playerId;
    }
    
    HTTPResponse response = httpRequest(url);
    
    if (response.responseCode != 200) {
        cout << "Failed to load replay: " << replayName << " (Code: " << response.responseCode << ")" << endl;
        return false;
    }
    
    Json::Value root;
    Json::Reader reader;
    
    if (!reader.parse(response.data, root)) {
        cout << "Failed to parse replay JSON" << endl;
        return false;
    }
    
    // 응답이 배열인 경우 (여러 액션들)
    if (root.isArray()) {
        replayActions = parseReplayActions(root);
    } else {
        // 단일 객체인 경우
        replayActions.clear();
        replayActions.push_back(jsonToReplayAction(root));
    }
    
    currentReplayName = replayName;
    currentPlayerId = playerId;
    currentActionIndex = 0;
    isReplayActive = true;
    replayStartTime = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    cout << "Loaded replay '" << replayName << "' with " << replayActions.size() << " actions" << endl;
    return true;
}

ReplayAction WalkMongoDB::jsonToReplayAction(const Json::Value& json) {
    ReplayAction action;
    action.id = json["_id"].asString();
    action.replay_name = json["replay_name"].asString();
    action.player_id = json["player_id"].asString();
    action.order = json["order"].asInt();
    action.timestamp = json["timestamp"].asInt64();
    action.action = json["action"].asString();
    action.yaw = json["yaw"].asDouble();
    action.pitch = json["pitch"].asDouble();
    action.duration = json["duration"].asInt();
    action.created_at = json["created_at"].asInt64();
    
    return action;
}

vector<ReplayAction> WalkMongoDB::parseReplayActions(const Json::Value& actionsArray) {
    vector<ReplayAction> actions;
    
    for (const auto& actionJson : actionsArray) {
        actions.push_back(jsonToReplayAction(actionJson));
    }
    
    // order 순으로 정렬
    sort(actions.begin(), actions.end(), [](const ReplayAction& a, const ReplayAction& b) {
        return a.order < b.order;
    });
    
    return actions;
}

void WalkMongoDB::executeReplay() {
    if (!isReplayActive || currentActionIndex >= replayActions.size()) {
        return;
    }
    
    ReplayAction& action = replayActions[currentActionIndex];
    
    cout << "Executing action " << (currentActionIndex + 1) << "/" << replayActions.size() 
         << ": " << action.action << " (duration: " << action.duration << "ms)" << endl;
    
    executeAction(action);
    
    currentActionIndex++;
    
    // 리플레이 완료 확인
    if (currentActionIndex >= replayActions.size()) {
        isReplayActive = false;
        cout << "Replay '" << currentReplayName << "' completed!" << endl;
        
        // 정지 상태로 전환
        executeStop();
    }
}

void WalkMongoDB::executeAction(const ReplayAction& action) {
    saveCurrentAction(); // 현재 실행 중인 액션 저장
    
    if (action.action == "forward") {
        executeForward(action.duration);
    }
    else if (action.action == "backward") {
        executeBackward(action.duration);
    }
    else if (action.action == "turn_left") {
        executeTurnLeft(action.yaw, action.duration);
    }
    else if (action.action == "turn_right") {
        executeTurnRight(action.yaw, action.duration);
    }
    else if (action.action == "stop") {
        executeStop();
    }
    else if (action.action == "motion") {
        // pitch를 모션 페이지 번호로 사용
        executeCustomMotion(static_cast<int>(action.pitch));
    }
    else {
        cout << "Unknown action: " << action.action << endl;
    }
}

void WalkMongoDB::executeForward(double duration) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(1.0);  // 전진
    mGaitManager->setAAmplitude(0.0);  // 직진
    wait(duration);
}

void WalkMongoDB::executeBackward(double duration) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(-1.0); // 후진
    mGaitManager->setAAmplitude(0.0);  // 직진
    wait(duration);
}

void WalkMongoDB::executeTurnLeft(double yaw, double duration) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(0.0);  // 제자리
    mGaitManager->setAAmplitude(0.5);  // 좌회전 (yaw 값 사용 가능)
    wait(duration);
}

void WalkMongoDB::executeTurnRight(double yaw, double duration) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(0.0);   // 제자리
    mGaitManager->setAAmplitude(-0.5);  // 우회전 (yaw 값 사용 가능)
    wait(duration);
}

void WalkMongoDB::executeStop() {
    if (isWalking) {
        mGaitManager->stop();
        isWalking = false;
        wait(200);
    }
}

void WalkMongoDB::executeCustomMotion(int motionPage) {
    mMotionManager->playPage(motionPage);
    wait(1000); // 모션 완료 대기
}

void WalkMongoDB::saveCurrentAction() {
    if (currentActionIndex < replayActions.size()) {
        ReplayAction& action = replayActions[currentActionIndex];
        
        // 현재 실행 로그 저장
        Json::Value executionLog;
        executionLog["robotId"] = robotId;
        executionLog["replay_name"] = action.replay_name;
        executionLog["player_id"] = action.player_id;
        executionLog["action_id"] = action.id;
        executionLog["order"] = action.order;
        executionLog["action"] = action.action;
        executionLog["executed_at"] = chrono::duration_cast<chrono::seconds>(
            chrono::system_clock::now().time_since_epoch()).count();
        executionLog["robot_timestamp"] = getTime();
        
        Json::StreamWriterBuilder builder;
        string jsonString = Json::writeString(builder, executionLog);
        
        string url = mongoApiUrl + "/execution-logs";
        httpRequest(url, "POST", jsonString);
    }
}

Json::Value WalkMongoDB::sensorDataToJson() {
    Json::Value data;
    data["robotId"] = robotId;
    data["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    data["robot_time"] = getTime();
    data["isWalking"] = isWalking;
    data["robotState"] = isWalking ? "walking" : "standing";
    data["currentReplay"] = currentReplayName;
    data["currentOrder"] = (currentActionIndex < replayActions.size()) ? 
                          replayActions[currentActionIndex].order : -1;
    
    // 센서 데이터 업데이트
    const double *acc = mAccelerometer->getValues();
    const double *gyro_vals = mGyro->getValues();
    
    Json::Value acc_array(Json::arrayValue);
    acc_array.append(acc[0]);
    acc_array.append(acc[1]);
    acc_array.append(acc[2]);
    data["accelerometer"] = acc_array;
    
    Json::Value gyro_array(Json::arrayValue);
    gyro_array.append(gyro_vals[0]);
    gyro_array.append(gyro_vals[1]);
    gyro_array.append(gyro_vals[2]);
    data["gyro"] = gyro_array;
    
    Json::Value motors(Json::arrayValue);
    for (int i = 0; i < NMOTORS; i++) {
        Json::Value motor;
        motor["name"] = motorNames[i];
        motor["position"] = mPositionSensors[i]->getValue();
        motors.append(motor);
    }
    data["motorPositions"] = motors;
    
    return data;
}

void WalkMongoDB::myStep() {
    int ret = step(mTimeStep);
    if (ret == -1)
        exit(EXIT_SUCCESS);
}

void WalkMongoDB::wait(int ms) {
    double startTime = getTime();
    double s = (double)ms / 1000.0;
    while (s + startTime >= getTime())
        myStep();
}

void WalkMongoDB::run() {
    cout << "-------Walk MongoDB Replay Controller-------" << endl;
    cout << "Robot controller with MongoDB Replay integration" << endl;
    cout << "API URL: " << mongoApiUrl << endl;
    
    // 첫 번째 센서 업데이트
    myStep();
    
    // 초기 자세로 이동
    mMotionManager->playPage(9);
    wait(200);
    
    // 기본 리플레이 로드 (설정에서 지정 가능)
    string defaultReplay = "track1";  // config에서 읽어올 수 있음
    cout << "Loading default replay: " << defaultReplay << endl;
    loadReplay(defaultReplay);
    
    auto lastSensorSaveTime = chrono::steady_clock::now();
    
    // 메인 제어 루프
    while (true) {
        checkIfFallen();
        
        auto currentTime = chrono::steady_clock::now();
        
        // 활성 리플레이 실행
        if (isReplayActive) {
            executeReplay();
        }
        
        // 센서 데이터 저장 (5초마다)
        if (chrono::duration_cast<chrono::milliseconds>(currentTime - lastSensorSaveTime).count() >= 5000) {
            Json::Value jsonData = sensorDataToJson();
            Json::StreamWriterBuilder builder;
            string jsonString = Json::writeString(builder, jsonData);
            
            string url = mongoApiUrl + "/sensor-data";
            httpRequest(url, "POST", jsonString);
            
            lastSensorSaveTime = currentTime;
        }
        
        // 로컬 키보드 제어 (새 리플레이 로드용)
        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
            switch (key) {
                case '1':  // track1 리플레이 로드
                    cout << "Loading track1 replay..." << endl;
                    executeStop();
                    loadReplay("track1");
                    break;
                case '2':  // track2 리플레이 로드
                    cout << "Loading track2 replay..." << endl;
                    executeStop();
                    loadReplay("track2");
                    break;
                case '3':  // track3 리플레이 로드
                    cout << "Loading track3 replay..." << endl;
                    executeStop();
                    loadReplay("track3");
                    break;
                case ' ':  // 리플레이 일시정지/재개
                    isReplayActive = !isReplayActive;
                    cout << "Replay " << (isReplayActive ? "resumed" : "paused") << endl;
                    if (!isReplayActive) {
                        executeStop();
                    }
                    break;
                case 'r':  // 리플레이 재시작
                    cout << "Restarting current replay..." << endl;
                    currentActionIndex = 0;
                    isReplayActive = true;
                    executeStop();
                    break;
                case 's':  // 정지
                    cout << "Stopping replay..." << endl;
                    isReplayActive = false;
                    executeStop();
                    break;
            }
        }
        
        if (isWalking) {
            mGaitManager->step(mTimeStep);
        }
        
        myStep();
    }
}

void WalkMongoDB::checkIfFallen() {
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

    // 로봇이 넘어진 경우
    if (fup > acc_step) {
        cout << "Robot fell forward - recovering..." << endl;
        isReplayActive = false;  // 리플레이 중단
        executeStop();
        mMotionManager->playPage(10);  // f_up
        mMotionManager->playPage(9);   // init position
        fup = 0;
        
        // 넘어짐 이벤트 로그
        Json::Value fallEvent;
        fallEvent["robotId"] = robotId;
        fallEvent["event"] = "fall_forward";
        fallEvent["timestamp"] = chrono::duration_cast<chrono::seconds>(
            chrono::system_clock::now().time_since_epoch()).count();
        fallEvent["replay_name"] = currentReplayName;
        fallEvent["action_order"] = (currentActionIndex < replayActions.size()) ? 
                                   replayActions[currentActionIndex].order : -1;
        
        Json::StreamWriterBuilder builder;
        string jsonString = Json::writeString(builder, fallEvent);
        
        string url = mongoApiUrl + "/events";
        httpRequest(url, "POST", jsonString);
    }
    else if (fdown > acc_step) {
        cout << "Robot fell backward - recovering..." << endl;
        isReplayActive = false;  // 리플레이 중단
        executeStop();
        mMotionManager->playPage(11);  // b_up
        mMotionManager->playPage(9);   // init position
        fdown = 0;
        
        // 넘어짐 이벤트 로그
        Json::Value fallEvent;
        fallEvent["robotId"] = robotId;
        fallEvent["event"] = "fall_backward";
        fallEvent["timestamp"] = chrono::duration_cast<chrono::seconds>(
            chrono::system_clock::now().time_since_epoch()).count();
        fallEvent["replay_name"] = currentReplayName;
        fallEvent["action_order"] = (currentActionIndex < replayActions.size()) ? 
                                   replayActions[currentActionIndex].order : -1;
        
        Json::StreamWriterBuilder builder;
        string jsonString = Json::writeString(builder, fallEvent);
        
        string url = mongoApiUrl + "/events";
        httpRequest(url, "POST", jsonString);
    }
}