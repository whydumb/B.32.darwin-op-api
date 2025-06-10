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
#include <queue>
#include <exception>

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
    
    // Initialize connection management variables
    maxRetries = 3;
    retryDelay = 1000;
    isOnlineMode = true;
    robotFallen = false;
    commandsExecuted = 0;
    commandsFailed = 0;
    connectionFailures = 0;
    averageExecutionTime = 0.0;
    startTime = chrono::steady_clock::now();
    
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
    shutdown();
    
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
        else if (line.find("max_retries") != string::npos) {
            size_t pos = line.find("=");
            if (pos != string::npos) {
                string retriesStr = line.substr(pos + 1);
                retriesStr.erase(0, retriesStr.find_first_not_of(" \t"));
                maxRetries = stoi(retriesStr);
            }
        }
        else if (line.find("retry_delay") != string::npos) {
            size_t pos = line.find("=");
            if (pos != string::npos) {
                string delayStr = line.substr(pos + 1);
                delayStr.erase(0, delayStr.find_first_not_of(" \t"));
                retryDelay = stoi(delayStr);
            }
        }
    }
    
    cout << "MongoDB API URL: " << mongoApiUrl << endl;
    cout << "Robot ID: " << robotId << endl;
    cout << "Poll Interval: " << commandPollInterval << "ms" << endl;
    cout << "Max Retries: " << maxRetries << endl;
    cout << "Retry Delay: " << retryDelay << "ms" << endl;
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
    robotStatus["capabilities"].append("walk_direction");
    robotStatus["capabilities"].append("set_speed");
    robotStatus["capabilities"].append("emergency_stop");
    robotStatus["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    robotStatus["version"] = "2.0";
    robotStatus["uptime"] = 0;
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, robotStatus);
    
    string url = mongoApiUrl + "/robots/status";
    HTTPResponse response = httpRequestWithRetry(url, "POST", jsonString);
    
    if (response.responseCode == 200) {
        cout << "Successfully registered robot with MongoDB" << endl;
        isOnlineMode = true;
    } else {
        cout << "Warning: Failed to register robot status: " << response.responseCode << endl;
        cout << "Robot will work in offline mode" << endl;
        isOnlineMode = false;
    }
}

bool WalkMongoDB::checkConnectionHealth() {
    string url = mongoApiUrl + "/health";
    HTTPResponse response = httpRequest(url, "GET", "");
    return response.responseCode == 200;
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
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "User-Agent: RobotController/2.0");
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
    } else {
        cout << "cURL error: " << curl_easy_strerror(res) << endl;
    }
    
    curl_slist_free_all(headers);
    return response;
}

HTTPResponse WalkMongoDB::httpRequestWithRetry(const string& url, const string& method, const string& postData) {
    HTTPResponse response;
    
    for (int attempt = 0; attempt < maxRetries; attempt++) {
        response = httpRequest(url, method, postData);
        
        if (response.responseCode == 200) {
            if (!isOnlineMode) {
                cout << "Connection restored - switching to online mode" << endl;
                isOnlineMode = true;
                processOfflineQueue();
            }
            return response;
        }
        
        if (attempt < maxRetries - 1) {
            cout << "HTTP request failed (attempt " << (attempt + 1) 
                 << "/" << maxRetries << "), retrying in " << retryDelay << "ms..." << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay));
        }
    }
    
    if (isOnlineMode) {
        cout << "Connection lost - switching to offline mode" << endl;
        isOnlineMode = false;
        connectionFailures++;
    }
    
    return response;
}

void WalkMongoDB::processOfflineQueue() {
    int processedCount = 0;
    
    while (!offlineCommandQueue.empty()) {
        RobotCommand cmd = offlineCommandQueue.front();
        offlineCommandQueue.pop();
        
        if (!isCommandExpired(cmd)) {
            Json::Value cmdJson = commandToJson(cmd);
            Json::StreamWriterBuilder builder;
            string jsonString = Json::writeString(builder, cmdJson);
            
            string url = mongoApiUrl + "/commands";
            HTTPResponse response = httpRequest(url, "POST", jsonString);
            
            if (response.responseCode == 200) {
                processedCount++;
            } else {
                // Re-queue if failed
                offlineCommandQueue.push(cmd);
                break;
            }
        }
    }
    
    if (processedCount > 0) {
        cout << "Processed " << processedCount << " offline commands" << endl;
    }
}

void WalkMongoDB::queueCommandOffline(const RobotCommand& command) {
    if (offlineCommandQueue.size() >= 100) {
        cout << "Offline command queue full, dropping oldest command" << endl;
        offlineCommandQueue.pop();
    }
    offlineCommandQueue.push(command);
    cout << "Command queued for offline processing: " << command.command << endl;
}

bool WalkMongoDB::validateMovementValue(double value, const string& commandType) {
    if (commandType == "walk_forward" || commandType == "walk_backward" ||
        commandType == "turn_left" || commandType == "turn_right" ||
        commandType == "set_speed") {
        if (value < 0.0 || value > 1.0) {
            cout << "Warning: Invalid " << commandType << " value: " << value 
                 << " (must be 0.0-1.0)" << endl;
            return false;
        }
    }
    
    if (commandType == "walk_direction") {
        if (value < -1.0 || value > 1.0) {
            cout << "Warning: Invalid walk_direction value: " << value 
                 << " (must be -1.0 to 1.0)" << endl;
            return false;
        }
    }
    
    if (commandType == "motion") {
        int motionPage = static_cast<int>(value);
        if (motionPage < 1 || motionPage > 255) {
            cout << "Warning: Invalid motion page: " << motionPage 
                 << " (must be 1-255)" << endl;
            return false;
        }
    }
    
    return true;
}

bool WalkMongoDB::pollForNewCommand() {
    string url = mongoApiUrl + "/commands/pending?robotId=" + robotId + "&limit=1";
    HTTPResponse response = httpRequestWithRetry(url);
    
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
        RobotCommand cmd = jsonToCommand(root[0]);
        
        if (isCommandExpired(cmd)) {
            updateCommandStatus(cmd.id, "expired");
            return false;
        }
        
        if (!validateMovementValue(cmd.value, cmd.command)) {
            updateCommandStatus(cmd.id, "failed");
            commandsFailed++;
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

Json::Value WalkMongoDB::commandToJson(const RobotCommand& cmd) {
    Json::Value json;
    json["_id"] = cmd.id;
    json["robotId"] = cmd.robotId;
    json["command"] = cmd.command;
    json["value"] = cmd.value;
    json["value2"] = cmd.value2;
    json["duration"] = cmd.duration;
    json["priority"] = cmd.priority;
    json["status"] = cmd.status;
    json["timestamp"] = cmd.timestamp;
    json["expires_at"] = cmd.expires_at;
    json["continuous"] = cmd.continuous;
    return json;
}

bool WalkMongoDB::isCommandExpired(const RobotCommand& command) {
    if (command.expires_at == 0) return false;
    
    auto now = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    return now > command.expires_at;
}

bool WalkMongoDB::isCommandCompleted() {
    if (!hasActiveCommand) return true;
    
    if (currentCommand.continuous) return false;
    
    if (currentCommand.duration > 0) {
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(
            chrono::steady_clock::now() - commandStartTime).count();
        return elapsed >= currentCommand.duration;
    }
    
    return false;
}

double WalkMongoDB::calculateCommandProgress() {
    if (!hasActiveCommand || currentCommand.duration <= 0) {
        return 0.0;
    }
    
    auto elapsed = chrono::duration_cast<chrono::milliseconds>(
        chrono::steady_clock::now() - commandStartTime).count();
    
    return min(1.0, static_cast<double>(elapsed) / currentCommand.duration);
}

void WalkMongoDB::executeCommand(const RobotCommand& command) {
    if (!validateMovementValue(command.value, command.command)) {
        updateCommandStatus(command.id, "failed");
        commandsFailed++;
        return;
    }
    
    auto executionStart = chrono::steady_clock::now();
    
    cout << "Executing: " << command.command 
         << " (value: " << command.value << ")" << endl;
    
    try {
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
            if (!isWalking) {
                mGaitManager->start();
                isWalking = true;
                wait(200);
            }
            mGaitManager->setXAmplitude(max(-1.0, min(1.0, command.value)));
            mGaitManager->setAAmplitude(max(-1.0, min(1.0, command.value2)));
            
        } else if (command.command == "set_speed") {
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
            updateCommandStatus(command.id, "failed");
            commandsFailed++;
            return;
        }
        
        // Update execution metrics
        commandsExecuted++;
        auto executionTime = chrono::duration_cast<chrono::milliseconds>(
            chrono::steady_clock::now() - executionStart).count();
        
        averageExecutionTime = (averageExecutionTime * (commandsExecuted - 1) + executionTime) / commandsExecuted;
        
    } catch (const std::exception& e) {
        cout << "Error executing command: " << e.what() << endl;
        updateCommandStatus(command.id, "failed");
        commandsFailed++;
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
    wait(1000);
}

void WalkMongoDB::updateCommandStatus(const string& commandId, const string& status) {
    Json::Value statusUpdate;
    statusUpdate["status"] = status;
    statusUpdate["updated_at"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    statusUpdate["robotId"] = robotId;
    
    if (status == "completed" || status == "executing") {
        statusUpdate["execution_time"] = getTime();
        if (hasActiveCommand) {
            statusUpdate["progress"] = calculateCommandProgress();
        }
    }
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, statusUpdate);
    
    string url = mongoApiUrl + "/commands/" + commandId + "/status";
    
    if (isOnlineMode) {
        HTTPResponse response = httpRequestWithRetry(url, "PUT", jsonString);
        if (response.responseCode != 200) {
            cout << "Failed to update command status: " << status << endl;
        }
    } else {
        cout << "Offline mode - status update queued: " << status << endl;
    }
}

void WalkMongoDB::clearPendingCommands() {
    string url = mongoApiUrl + "/commands/clear?robotId=" + robotId;
    HTTPResponse response = httpRequestWithRetry(url, "POST", "{}");
    
    if (response.responseCode == 200) {
        cout << "Cleared all pending commands" << endl;
    } else {
        cout << "Failed to clear pending commands" << endl;
    }
}

Json::Value WalkMongoDB::sensorDataToJson() {
    Json::Value data;
    
    try {
        data["robotId"] = robotId;
        data["timestamp"] = chrono::duration_cast<chrono::seconds>(
            chrono::system_clock::now().time_since_epoch()).count();
        data["robot_time"] = getTime();
        data["isWalking"] = isWalking;
        data["hasActiveCommand"] = hasActiveCommand;
        data["robotState"] = isWalking ? "walking" : "standing";
        
        // Connection and queue status
        data["connectionStatus"] = isOnlineMode ? "online" : "offline";
        data["queuedCommands"] = static_cast<int>(offlineCommandQueue.size());
        data["robotFallen"] = robotFallen;
        
        // Performance metrics
        data["metrics"] = Json::Value(Json::objectValue);
        data["metrics"]["commandsExecuted"] = commandsExecuted;
        data["metrics"]["commandsFailed"] = commandsFailed;
        data["metrics"]["connectionFailures"] = connectionFailures;
        data["metrics"]["averageExecutionTime"] = averageExecutionTime;
        
        auto uptime = chrono::duration_cast<chrono::seconds>(
            chrono::steady_clock::now() - startTime).count();
        data["metrics"]["uptime"] = uptime;
        
        if (hasActiveCommand) {
            data["currentCommand"] = currentCommand.command;
            data["commandValue"] = currentCommand.value;
            data["commandDuration"] = currentCommand.duration;
            data["commandProgress"] = calculateCommandProgress();
            data["commandContinuous"] = currentCommand.continuous;
        }
        
        // Sensor data with error handling
        if (mAccelerometer) {
            const double *acc = mAccelerometer->getValues();
            if (acc) {
                Json::Value acc_array(Json::arrayValue);
                acc_array.append(acc[0]);
                acc_array.append(acc[1]);
                acc_array.append(acc[2]);
                data["accelerometer"] = acc_array;
            }
        }
        
        if (mGyro) {
            const double *gyro_vals = mGyro->getValues();
            if (gyro_vals) {
                Json::Value gyro_array(Json::arrayValue);
                gyro_array.append(gyro_vals[0]);
                gyro_array.append(gyro_vals[1]);
                gyro_array.append(gyro_vals[2]);
                data["gyro"] = gyro_array;
            }
        }
        
        // Walking parameters
        if (isWalking && mGaitManager) {
            data["walkSpeed"] = Json::Value(Json::objectValue);
            data["walkSpeed"]["x"] = mGaitManager->getXAmplitude();
            data["walkSpeed"]["a"] = mGaitManager->getAAmplitude();
        }
        
        // System health
        data["health"] = Json::Value(Json::objectValue);
        data["health"]["temperature"] = "normal";
        data["health"]["battery"] = "good";
        data["health"]["motors"] = "operational";
        
    } catch (const std::exception& e) {
        cout << "Error creating sensor data JSON: " << e.what() << endl;
        data["error"] = "Failed to collect sensor data";
        data["error_details"] = e.what();
    }
    
    return data;
}

void WalkMongoDB::saveSensorData() {
    Json::Value jsonData = sensorDataToJson();
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, jsonData);
    
    string url = mongoApiUrl + "/sensor-data";
    
    if (isOnlineMode) {
        HTTPResponse response = httpRequest(url, "POST", jsonString);
        if (response.responseCode != 200) {
            cout << "Failed to save sensor data (code: " << response.responseCode << ")" << endl;
        }
    }
}

void WalkMongoDB::updateRobotStatus() {
    Json::Value status;
    status["robotId"] = robotId;
    status["status"] = isOnlineMode ? "online" : "offline";
    status["isWalking"] = isWalking;
    status["hasActiveCommand"] = hasActiveCommand;
    status["timestamp"] = chrono::duration_cast<chrono::seconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    
    // Performance metrics
    auto uptime = chrono::duration_cast<chrono::seconds>(
        chrono::steady_clock::now() - startTime).count();
    status["uptime"] = uptime;
    status["queuedCommands"] = static_cast<int>(offlineCommandQueue.size());
    status["commandsExecuted"] = commandsExecuted;
    status["commandsFailed"] = commandsFailed;
    status["connectionFailures"] = connectionFailures;
    
    // Health status
    status["health"] = Json::Value(Json::objectValue);
    status["health"]["fallen"] = robotFallen;
    status["health"]["temperature"] = "normal";
    status["health"]["operational"] = true;
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, status);
    
    string url = mongoApiUrl + "/robots/status";
    
    if (isOnlineMode) {
        HTTPResponse response = httpRequestWithRetry(url, "PUT", jsonString);
        if (response.responseCode != 200) {
            cout << "Failed to update robot status" << endl;
        }
    } else {
        cout << "Offline mode - status update skipped" << endl;
    }
}

void WalkMongoDB::shutdown() {
    cout << "Initiating graceful shutdown..." << endl;
    
    // Stop all movement
    executeStop();
    
    // Update final command status
    if (hasActiveCommand) {
        updateCommandStatus(currentCommand.id, "interrupted");
    }
    
    // Update robot status to offline
    Json::Value status;
    status["robotId"] = robotId;
    status["status"] = "offline";
    
    Json::StreamWriterBuilder builder;
    string jsonString = Json::writeString(builder, status);
    
    string url = mongoApiUrl + "/robots/status";
    
    if (isOnlineMode) {
        HTTPResponse response = httpRequestWithRetry(url, "PUT", jsonString);
        if (response.responseCode != 200) {
            cout << "Failed to update robot status to offline" << endl;
        }
    } else {
        cout << "Offline mode - status update skipped" << endl;
    }
    
    // Clean up resources
    if (curl) {
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
    
    // Delete all commands
    clearPendingCommands();
    
    // Delete all sensor data
    deleteSensorData();
}    
