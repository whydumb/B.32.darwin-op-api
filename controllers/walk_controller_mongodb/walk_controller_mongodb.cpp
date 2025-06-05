// Fixed walk_controller_mongodb.cpp
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
// Simple string-based communication instead of JSON

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
    serverPort = 8080;
    isServerRunning = false;
    isWalking = false;

#ifdef _WIN32
    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cerr << "Failed to initialize Winsock" << endl;
    }
#endif

    // Initialize robot hardware
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
    
    setupSocketServer();
}

WalkMongoDB::~WalkMongoDB() {
    isServerRunning = false;
    if (serverSocket != INVALID_SOCKET) {
        closesocket(serverSocket);
    }
    
#ifdef _WIN32
    WSACleanup();
#endif

    delete mMotionManager;
    delete mGaitManager;
}

void WalkMongoDB::setupSocketServer() {
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == INVALID_SOCKET) {
        cerr << "Failed to create socket" << endl;
        return;
    }

    // Set socket options - Fixed for Windows
#ifdef _WIN32
    char opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == SOCKET_ERROR) {
        cerr << "Failed to set socket options: " << WSAGetLastError() << endl;
        closesocket(serverSocket);
        return;
    }
#else
    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        cerr << "Failed to set socket options" << endl;
        close(serverSocket);
        return;
    }
#endif

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(serverPort);

    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        cerr << "Failed to bind socket" << endl;
        closesocket(serverSocket);
        return;
    }

    if (listen(serverSocket, 5) == SOCKET_ERROR) {
        cerr << "Failed to listen on socket" << endl;
        closesocket(serverSocket);
        return;
    }

    isServerRunning = true;
    cout << "Socket server listening on port " << serverPort << endl;
}

void WalkMongoDB::handleClient(SOCKET clientSocket) {  // Fixed parameter type
    char buffer[1024];
    
    while (isServerRunning) {
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        
        if (bytesReceived <= 0) {
            break;  // Client disconnected or error
        }
        
        buffer[bytesReceived] = '\0';
        string receivedData(buffer);
        
        // Parse command
        try {
            CommandData cmd = deserializeCommand(receivedData);
            processCommand(cmd);
            
            // Send response with current sensor data
            sendSensorData(clientSocket);
        } catch (const exception& e) {
            cerr << "Error processing command: " << e.what() << endl;
        }
    }
    
    closesocket(clientSocket);
}

void WalkMongoDB::sendSensorData(SOCKET clientSocket) {
    // Update current sensor data
    const double *acc = mAccelerometer->getValues();
    const double *gyro_vals = mGyro->getValues();
    
    currentSensorData.accelerometer[0] = acc[0];
    currentSensorData.accelerometer[1] = acc[1];
    currentSensorData.accelerometer[2] = acc[2];
    
    currentSensorData.gyro[0] = gyro_vals[0];
    currentSensorData.gyro[1] = gyro_vals[1];
    currentSensorData.gyro[2] = gyro_vals[2];
    
    for (int i = 0; i < NMOTORS; i++) {
        currentSensorData.motorPositions[i] = mPositionSensors[i]->getValue();
    }
    
    currentSensorData.timestamp = getTime();
    currentSensorData.isWalking = isWalking;
    currentSensorData.robotState = isWalking ? "walking" : "standing";
    
    string jsonData = serializeSensorData();
    send(clientSocket, jsonData.c_str(), jsonData.length(), 0);
}

string WalkMongoDB::serializeSensorData() {
    ostringstream oss;
    oss << "SENSOR_DATA|";
    oss << currentSensorData.timestamp << "|";
    
    // Accelerometer data
    oss << currentSensorData.accelerometer[0] << "," 
        << currentSensorData.accelerometer[1] << "," 
        << currentSensorData.accelerometer[2] << "|";
    
    // Gyro data
    oss << currentSensorData.gyro[0] << "," 
        << currentSensorData.gyro[1] << "," 
        << currentSensorData.gyro[2] << "|";
    
    // Motor positions
    for (int i = 0; i < NMOTORS; i++) {
        oss << currentSensorData.motorPositions[i];
        if (i < NMOTORS - 1) oss << ",";
    }
    oss << "|";
    
    // Robot state
    oss << (currentSensorData.isWalking ? "1" : "0") << "|";
    oss << currentSensorData.robotState << "\n";
    
    return oss.str();
}

CommandData WalkMongoDB::deserializeCommand(const string& data) {
    CommandData cmd;
    istringstream iss(data);
    string token;
    vector<string> tokens;
    
    // Split by '|' delimiter
    while (getline(iss, token, '|')) {
        tokens.push_back(token);
    }
    
    if (tokens.size() < 2) {
        throw runtime_error("Invalid command format");
    }
    
    cmd.command = tokens[0];
    
    // Parse timestamp if available
    if (tokens.size() > 1 && !tokens[1].empty()) {
        cmd.timestamp = stod(tokens[1]);
    }
    
    // Parse parameters if available
    if (tokens.size() > 2 && !tokens[2].empty()) {
        istringstream paramStream(tokens[2]);
        string param;
        while (getline(paramStream, param, ',')) {
            if (!param.empty()) {
                cmd.parameters.push_back(stod(param));
            }
        }
    }
    
    return cmd;
}

void WalkMongoDB::processCommand(const CommandData& cmd) {
    cout << "Processing command: " << cmd.command << endl;
    
    if (cmd.command == "start_walking") {
        if (!isWalking) {
            mGaitManager->start();
            isWalking = true;
            wait(200);
        }
    }
    else if (cmd.command == "stop_walking") {
        if (isWalking) {
            mGaitManager->stop();
            isWalking = false;
            wait(200);
        }
    }
    else if (cmd.command == "move_forward" && cmd.parameters.size() > 0) {
        mGaitManager->setXAmplitude(cmd.parameters[0]);
    }
    else if (cmd.command == "turn" && cmd.parameters.size() > 0) {
        mGaitManager->setAAmplitude(cmd.parameters[0]);
    }
    else if (cmd.command == "play_motion" && cmd.parameters.size() > 0) {
        int motionPage = static_cast<int>(cmd.parameters[0]);
        mMotionManager->playPage(motionPage);
    }
    else if (cmd.command == "get_sensor_data") {
        // This will be handled by sending sensor data in response
    }
    else {
        cout << "Unknown command: " << cmd.command << endl;
    }
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
    cout << "-------Walk MongoDB Controller-------" << endl;
    cout << "Robot controller with MongoDB/Socket integration" << endl;
    cout << "Listening for network commands on port " << serverPort << endl;
    
    // First step to update sensors
    myStep();
    
    // Initialize robot position
    mMotionManager->playPage(9);  // init position
    wait(200);
    
    // Start network thread to handle clients
    thread networkThread([this]() {
        while (isServerRunning) {
            struct sockaddr_in clientAddr;
            socklen_t clientLen = sizeof(clientAddr);
            
            SOCKET clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
            if (clientSocket != INVALID_SOCKET) {
                cout << "Client connected" << endl;
                thread clientThread(&WalkMongoDB::handleClient, this, clientSocket);
                clientThread.detach();  // Handle client in separate thread
            }
        }
    });
    
    // Main control loop
    while (true) {
        checkIfFallen();
        
        // Handle keyboard input (for local control)
        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
            switch (key) {
                case ' ':  // Space bar
                    if (isWalking) {
                        mGaitManager->stop();
                        isWalking = false;
                        wait(200);
                    } else {
                        mGaitManager->start();
                        isWalking = true;
                        wait(200);
                    }
                    break;
                case Keyboard::UP:
                    mGaitManager->setXAmplitude(1.0);
                    break;
                case Keyboard::DOWN:
                    mGaitManager->setXAmplitude(-1.0);
                    break;
                case Keyboard::RIGHT:
                    mGaitManager->setAAmplitude(-0.5);
                    break;
                case Keyboard::LEFT:
                    mGaitManager->setAAmplitude(0.5);
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

    // Robot face is down
    if (fup > acc_step) {
        mMotionManager->playPage(10);  // f_up
        mMotionManager->playPage(9);   // init position
        fup = 0;
    }
    // Robot back is down
    else if (fdown > acc_step) {
        mMotionManager->playPage(11);  // b_up
        mMotionManager->playPage(9);   // init position
        fdown = 0;
    }
}