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
    hasActiveCommand = false;
    commandPollInterval = 500; // 0.5초마다 폴링
    
    // cURL 초기화
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    robotId = "robot_001";

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
    
    lastCommandPoll = chrono::steady_clock::now();
    
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
        else if (line.find("robot_id") != string::npos) {
            size_t pos = line.find("=");
            if (pos != string::npos) {
                robotId = line.substr(pos + 1);
                robotId.erase(0, robotId.find_first_not_of(" \t"));
                robotId.erase(robotId.find_last_not_of(" \t") + 1);
            }
        }
        else if (line.find("poll_interval") != string::npos) {
            size_t pos = line.find("=");
            if (pos != string::npos) {
                string intervalStr = line.substr(pos + 1);
                intervalStr.erase(0, intervalStr.find_first_not_of(" \t"));
                commandPollInterval = stoi(intervalStr);
            }
        }
    }
    
    cout << "MongoDB API URL: " << mongoApiUrl << endl;
    cout << "Robot ID: " << robotId << endl;
    cout << "Poll Interval: " << commandPollInterval << "ms" << endl;
}

void WalkMongoDB::initializeMongoDB() {
    // 로봇 상태 등록
    Json::Value robotStatus;
    robotStatus["robotId"] = robotId;
    robotStatus["status"] = "online";
    robotStatus["capabilities"] = Json::Value(Json::arrayValue);
    robotStatus["capabilities"].append("walk_forward");
    robotStatus["capabilities"].append("walk_backward");
    robotStatus["capabilities"].append("turn_left");
    robotStatus["capabilities"].append("turn_right");
    robotStatus["capabilities"].append("stop");
    robotStatus["capabilities"].append("motion");
    robotStatus["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, robotStatus);
    
    string url = mongoApiUrl + "/robots/status";
    HTTPResponse response = httpRequest(url, "POST", jsonString);
    
    if (response.responseCode == 200) {
        cout << "Successfully registered robot with MongoDB" << endl;
    } else {
        cout << "Warning: Failed to register robot status: " << response.responseCode << endl;
        cout << "Robot will work in offline mode" << endl;
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
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L); // 5초 타임아웃
    
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

bool WalkMongoDB::pollForNewCommand() {
    // MongoDB에서 새로운 명령 조회
    string url = mongoApiUrl + "/commands/pending?robotId=" + robotId + "&limit=1";
    HTTPResponse response = httpRequest(url);
    
    if (response.responseCode != 200) {
        return false;
    }
    
    Json::Value root;
    Json::Reader reader;
    
    if (!reader.parse(response.data, root)) {
        cout << "Failed to parse command JSON" << endl;
        return false;
    }
    
    if (root.isArray() && root.size() > 0) {
        // 첫 번째 명령 선택 (가장 높은 우선순위)
        RobotCommand cmd = jsonToCommand(root[0]);
        
        // 만료된 명령 스킵
        if (isCommandExpired(cmd)) {
            updateCommandStatus(cmd.id, "expired");
            return false;
        }
        
        currentCommand = cmd;
        hasActiveCommand = true;
        commandStartTime = chrono::steady_clock::now();
        updateCommandStatus(currentCommand.id, "executing");
        
        cout << "New command received: " << currentCommand.command 
             << " (value: " << currentCommand.value 
             << ", duration: " << currentCommand.duration << "ms)" << endl;
        
        return true;
    }
    
    return false;
}

RobotCommand WalkMongoDB::jsonToCommand(const Json::Value& json) {
    RobotCommand cmd;
    
    cmd.id = json.get("_id", "unknown").asString();
    cmd.robotId = json.get("robotId", "").asString();
    cmd.command = json.get("command", "stop").asString();
    cmd.value = json.get("value", 0.0).asDouble();
    cmd.value2 = json.get("value2", 0.0).asDouble();
    cmd.duration = json.get("duration", 1000).asInt();
    cmd.priority = json.get("priority", "normal").asString();
    cmd.status = json.get("status", "pending").asString();
    cmd.timestamp = json.get("timestamp", 0).asInt64();
    cmd.expires_at = json.get("expires_at", 0).asInt64();
    cmd.continuous = json.get("continuous", false).asBool();
    
    return cmd;
}

bool WalkMongoDB::isCommandExpired(const RobotCommand& command) {
    if (command.expires_at == 0) return false; // 만료 시간 없음
    
    auto now = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    return now > command.expires_at;
}

bool WalkMongoDB::isCommandCompleted() {
    if (!hasActiveCommand) return true;
    
    // 연속 실행 명령은 완료되지 않음
    if (currentCommand.continuous) return false;
    
    // 지속시간 체크
    if (currentCommand.duration > 0) {
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(
            chrono::steady_clock::now() - commandStartTime).count();
        return elapsed >= currentCommand.duration;
    }
    
    return false;
}

void WalkMongoDB::executeCommand(const RobotCommand& command) {
    if (command.command == "walk_forward") {
        executeWalkForward(command.value);
        
    } else if (command.command == "walk_backward") {
        executeWalkBackward(command.value);
        
    } else if (command.command == "turn_left") {
        executeTurnLeft(command.value);
        
    } else if (command.command == "turn_right") {
        executeTurnRight(command.value);
        
    } else if (command.command == "stop") {
        executeStop();
        
    } else if (command.command == "motion") {
        executeMotion(static_cast<int>(command.value));
        
    } else if (command.command == "walk_direction") {
        // value: X 방향 속도, value2: 회전 속도
        if (!isWalking) {
            mGaitManager->start();
            isWalking = true;
            wait(200);
        }
        mGaitManager->setXAmplitude(max(-1.0, min(1.0, command.value)));
        mGaitManager->setAAmplitude(max(-1.0, min(1.0, command.value2)));
        
    } else if (command.command == "set_speed") {
        // 현재 방향 유지하면서 속도만 변경
        if (isWalking) {
            double currentX = mGaitManager->getXAmplitude();
            double currentA = mGaitManager->getAAmplitude();
            
            if (currentX != 0) {
                mGaitManager->setXAmplitude(currentX > 0 ? command.value : -command.value);
            }
            if (currentA != 0) {
                mGaitManager->setAAmplitude(currentA > 0 ? command.value : -command.value);
            }
        }
        
    } else if (command.command == "emergency_stop") {
        executeStop();
        clearPendingCommands();
        
    } else {
        cout << "Unknown command: " << command.command << endl;
    }
}

void WalkMongoDB::executeWalkForward(double speed) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(min(1.0, max(0.0, speed)));
    mGaitManager->setAAmplitude(0.0);
}

void WalkMongoDB::executeWalkBackward(double speed) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(-min(1.0, max(0.0, speed)));
    mGaitManager->setAAmplitude(0.0);
}

void WalkMongoDB::executeTurnLeft(double speed) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(min(1.0, max(0.0, speed)));
}

void WalkMongoDB::executeTurnRight(double speed) {
    if (!isWalking) {
        mGaitManager->start();
        isWalking = true;
        wait(200);
    }
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(-min(1.0, max(0.0, speed)));
}

void WalkMongoDB::executeStop() {
    if (isWalking) {
        mGaitManager->stop();
        isWalking = false;
        wait(200);
    }
}

void WalkMongoDB::executeMotion(int motionPage) {
    if (isWalking) {
        executeStop();
    }
    mMotionManager->playPage(motionPage);
    wait(1000); // 모션 완료 대기
}

void WalkMongoDB::updateCommandStatus(const string& commandId, const string& status) {
    Json::Value statusUpdate;
    statusUpdate["status"] = status;
    statusUpdate["updated_at"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    statusUpdate["robotId"] = robotId;
    
    if (status == "completed" || status == "executing") {
        statusUpdate["execution_time"] = getTime();
    }
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, statusUpdate);
    
    string url = mongoApiUrl + "/commands/" + commandId + "/status";
    HTTPResponse response = httpRequest(url, "PUT", jsonString);
    
    // 오프라인 모드에서는 에러 메시지 표시하지 않음
    if (response.responseCode != 200 && response.responseCode != 0) {
        cout << "Failed to update command status (offline mode)" << endl;
    }
}

void WalkMongoDB::clearPendingCommands() {
    string url = mongoApiUrl + "/commands/clear?robotId=" + robotId;
    HTTPResponse response = httpRequest(url, "POST", "{}");
    
    if (response.responseCode == 200) {
        cout << "Cleared all pending commands" << endl;
    }
}

Json::Value WalkMongoDB::sensorDataToJson() {
    Json::Value data;
    data["robotId"] = robotId;
    data["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    data["robot_time"] = getTime();
    data["isWalking"] = isWalking;
    data["hasActiveCommand"] = hasActiveCommand;
    data["robotState"] = isWalking ? "walking" : "standing";
    
    if (hasActiveCommand) {
        data["currentCommand"] = currentCommand.command;
        data["commandValue"] = currentCommand.value;
        data["commandDuration"] = currentCommand.duration;
    }
    
    // 센서 데이터
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
    
    // 걸음 상태
    if (isWalking) {
        data["walkSpeed"] = Json::Value(Json::objectValue);
        data["walkSpeed"]["x"] = mGaitManager->getXAmplitude();
        data["walkSpeed"]["a"] = mGaitManager->getAAmplitude();
    }
    
    return data;
}

void WalkMongoDB::saveSensorData() {
    Json::Value jsonData = sensorDataToJson();
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, jsonData);
    
    string url = mongoApiUrl + "/sensor-data";
    httpRequest(url, "POST", jsonString);
}

void WalkMongoDB::updateRobotStatus() {
    Json::Value status;
    status["robotId"] = robotId;
    status["status"] = "online";
    status["isWalking"] = isWalking;
    status["hasActiveCommand"] = hasActiveCommand;
    status["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, status);
    
    string url = mongoApiUrl + "/robots/status";
    httpRequest(url, "PUT", jsonString);
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
    cout << "=================================================" << endl;
    cout << "    Walk MongoDB Real-Time Controller v1.0" << endl;
    cout << "=================================================" << endl;
    cout << "Real-time command processing from MongoDB" << endl;
    cout << "API URL: " << mongoApiUrl << endl;
    cout << "Robot ID: " << robotId << endl;
    cout << "Poll Interval: " << commandPollInterval << "ms" << endl;
    cout << endl;
    cout << "Available Commands:" << endl;
    cout << "  walk_forward, walk_backward" << endl;
    cout << "  turn_left, turn_right" << endl;
    cout << "  stop, motion, walk_direction" << endl;
    cout << "  emergency_stop" << endl;
    cout << endl;
    cout << "Local Controls:" << endl;
    cout << "  SPACE - Emergency stop" << endl;
    cout << "  s     - Stop current action" << endl;
    cout << "  d     - Debug info" << endl;
    cout << "  q     - Quit" << endl;
    cout << "=================================================" << endl;
    
    // 초기화
    myStep();
    mMotionManager->playPage(9); // 초기 자세
    wait(2000);
    
    auto lastSensorSave = chrono::steady_clock::now();
    auto lastStatusUpdate = chrono::steady_clock::now();
    auto lastDebugOutput = chrono::steady_clock::now();
    
    cout << "\nStarting real-time command processing..." << endl;
    cout << "Waiting for commands from MongoDB..." << endl;
    
    while (true) {
        checkIfFallen();
        auto currentTime = chrono::steady_clock::now();
        
        // 새로운 명령 폴링
        auto timeSinceLastPoll = chrono::duration_cast<chrono::milliseconds>(
            currentTime - lastCommandPoll).count();
        
        if (timeSinceLastPoll >= commandPollInterval) {
            if (!hasActiveCommand) {
                pollForNewCommand();
            }
            lastCommandPoll = currentTime;
        }
        
        // 활성 명령 실행 및 완료 체크
        if (hasActiveCommand) {
            executeCommand(currentCommand);
            
            if (isCommandCompleted()) {
                updateCommandStatus(currentCommand.id, "completed");
                cout << "Command completed: " << currentCommand.command << endl;
                hasActiveCommand = false;
            }
        }
        
        // 주기적 센서 데이터 저장 (5초마다)
        if (chrono::duration_cast<chrono::milliseconds>(currentTime - lastSensorSave).count() >= 5000) {
            saveSensorData();
            lastSensorSave = currentTime;
        }
        
        // 로봇 상태 업데이트 (10초마다)
        if (chrono::duration_cast<chrono::milliseconds>(currentTime - lastStatusUpdate).count() >= 10000) {
            updateRobotStatus();
            lastStatusUpdate = currentTime;
        }
        
        // 상태 출력 (30초마다)
        if (chrono::duration_cast<chrono::milliseconds>(currentTime - lastDebugOutput).count() >= 30000) {
            cout << "Status: " << (isWalking ? "Walking" : "Standing")
                 << " | Active Command: " << (hasActiveCommand ? currentCommand.command : "None") << endl;
            lastDebugOutput = currentTime;
        }
        
        // 키보드 제어 (로컬 디버깅용)
        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
            switch (key) {
                case ' ':  // 긴급 정지
                    cout << "\n!!! EMERGENCY STOP !!!" << endl;
                    hasActiveCommand = false;
                    executeStop();
                    clearPendingCommands();
                    break;
                    
                case 's':  // 정지
                    cout << "\nManual stop" << endl;
                    hasActiveCommand = false;
                    executeStop();
                    break;
                    
                case 'd':  // 디버그 정보
                    cout << "\n=== DEBUG INFO ===" << endl;
                    cout << "Robot ID: " << robotId << endl;
                    cout << "Is Walking: " << (isWalking ? "Yes" : "No") << endl;
                    cout << "Has Active Command: " << (hasActiveCommand ? "Yes" : "No") << endl;
                    if (hasActiveCommand) {
                        cout << "Current Command: " << currentCommand.command << endl;
                        cout << "Command Value: " << currentCommand.value << endl;
                        cout << "Command Duration: " << currentCommand.duration << "ms" << endl;
                        cout << "Continuous: " << (currentCommand.continuous ? "Yes" : "No") << endl;
                        
                        auto elapsed = chrono::duration_cast<chrono::milliseconds>(
                            chrono::steady_clock::now() - commandStartTime).count();
                        cout << "Elapsed Time: " << elapsed << "ms" << endl;
                    }
                    if (isWalking) {
                        cout << "Walk X Amplitude: " << mGaitManager->getXAmplitude() << endl;
                        cout << "Walk A Amplitude: " << mGaitManager->getAAmplitude() << endl;
                    }
                    cout << "Poll Interval: " << commandPollInterval << "ms" << endl;
                    cout << "MongoDB URL: " << mongoApiUrl << endl;
                    cout << "=================" << endl;
                    break;
                    
                case 'q':  // 종료
                    cout << "\nShutting down robot..." << endl;
                    executeStop();
                    updateRobotStatus();
                    exit(EXIT_SUCCESS);
                    break;
            }
        }
        
        // Gait Manager 스텝 (걷고 있을 때만)
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
        hasActiveCommand = false;  // 현재 명령 중단
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
        fallEvent["robot_time"] = getTime();
        if (hasActiveCommand) {
            fallEvent["interrupted_command"] = currentCommand.command;
        }
        
        Json::StreamWriterBuilder builder;
        string jsonString = Json::writeString(builder, fallEvent);
        
        string url = mongoApiUrl + "/events";
        httpRequest(url, "POST", jsonString);
    }
    else if (fdown > acc_step) {
        cout << "Robot fell backward - recovering..." << endl;
        hasActiveCommand = false;  // 현재 명령 중단
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
        fallEvent["robot_time"] = getTime();
        if (hasActiveCommand) {
            fallEvent["interrupted_command"] = currentCommand.command;
        }
        
        Json::StreamWriterBuilder builder;
        string jsonString = Json::writeString(builder, fallEvent);
        
        string url = mongoApiUrl + "/events";
        httpRequest(url, "POST", jsonString);
    }
}