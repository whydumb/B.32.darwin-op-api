// walk_controller_mongodb.cpp - Complete corrected implementation
#include "walk_controller_mongodb.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constructor
Walk::Walk() : webots::Robot() {
    mTimeStep = 32;  // 32ms time step
    mIsWalking = false;
    
    // Initialize MongoDB objects (pointers)
    mMongoClient = nullptr;
    mDatabase = nullptr;
    mCollection = nullptr;
    
    // Initialize hardware pointers
    for (int i = 0; i < NMOTORS; i++) {
        mMotors[i] = nullptr;
        mPositionSensors[i] = nullptr;
    }
    mAccelerometer = nullptr;
    mKeyboard = nullptr;
    
    // Initialize ROBOTIS managers
    mMotionManager = nullptr;
    mGaitManager = nullptr;
    
    std::cout << "Walk controller initializing..." << std::endl;
    
    // Initialize MongoDB
    initializeMongoDB();
    
    // Initialize action mapper
    initializeActionMapper();
    
    std::cout << "Walk controller initialization complete!" << std::endl;
}

// Destructor
Walk::~Walk() {
    std::cout << "Walk controller shutting down" << std::endl;
    
    // Clean up MongoDB objects
    if (mCollection) {
        delete mCollection;
        mCollection = nullptr;
    }
    if (mDatabase) {
        delete mDatabase;
        mDatabase = nullptr;
    }
    if (mMongoClient) {
        delete mMongoClient;
        mMongoClient = nullptr;
    }
}

// Main execution function
void Walk::run() {
    std::cout << "Walk controller execution started" << std::endl;
    
    // Initialize keyboard
    mKeyboard = getKeyboard();
    if (mKeyboard) {
        mKeyboard->enable(mTimeStep);
    }
    
    // Initialize accelerometer
    mAccelerometer = getAccelerometer("accelerometer");
    if (mAccelerometer) {
        mAccelerometer->enable(mTimeStep);
    }
    
    // Motor names
    const char* motorNames[NMOTORS] = {
        "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL",
        "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL",
        "PelvR", "PelvL", "LegUpperR", "LegUpperL", 
        "LegLowerR", "LegLowerL", "AnkleR", "AnkleL",
        "FootR", "FootL", "Neck", "Head"
    };
    
    // Initialize motors
    for (int i = 0; i < NMOTORS; i++) {
        mMotors[i] = getMotor(motorNames[i]);
        std::string sensorName = std::string(motorNames[i]) + "S";
        mPositionSensors[i] = getPositionSensor(sensorName);
        if (mPositionSensors[i]) {
            mPositionSensors[i]->enable(mTimeStep);
        }
    }
    
    // Main loop
    while (step(mTimeStep) != -1) {
        // Handle keyboard input
        if (mKeyboard) {
            int key = mKeyboard->getKey();
            if (key == 'W' || key == 'w') {
                mIsWalking = true;
                std::cout << "Walking started!" << std::endl;
            } else if (key == 'S' || key == 's') {
                mIsWalking = false;
                std::cout << "Walking stopped!" << std::endl;
            } else if (key == 'M' || key == 'm') {
                // Read and execute from MongoDB
                readAndExecuteFromMongoDB();
            } else if (key == 'P' || key == 'p') {
                // MongoDB playback
                playbackFromMongoDB();
            }
        }
        
        // Check if fallen
        checkIfFallen();
        
        // Read and execute actions from MongoDB (automatic)
        if (mIsWalking) {
            readAndExecuteFromMongoDB();
        }
        
        // Status output (every 5 seconds)
        if (fmod(getTime(), 5.0) < 0.032) {
            std::cout << "Time: " << getTime() << "s, Walking: " << (mIsWalking ? "ON" : "OFF") << std::endl;
        }
    }
}

// Check if fallen
void Walk::checkIfFallen() {
    if (!mAccelerometer) return;
    
    const double* accel = mAccelerometer->getValues();
    double totalAccel = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    
    // If acceleration is too high, consider robot has fallen
    if (totalAccel > 15.0) {
        mIsWalking = false;
        std::cout << "Robot has fallen! Stopping walk." << std::endl;
    }
}

// Execute one step
void Walk::myStep() {
    step(mTimeStep);
}

// Wait function
void Walk::wait(int ms) {
    double startTime = getTime();
    while (getTime() - startTime < ms / 1000.0) {
        step(mTimeStep);
    }
}

// Initialize MongoDB
void Walk::initializeMongoDB() {
    try {
        std::cout << "Attempting MongoDB connection..." << std::endl;
        
        // Create MongoDB instance (as static variable)
        static mongocxx::instance instance{};
        
        // Create client
        mMongoClient = new mongocxx::client(mongocxx::uri{});
        
        // Create database
        mDatabase = new mongocxx::database((*mMongoClient)["robotics"]);
        
        // Create collection
        mCollection = new mongocxx::collection((*mDatabase)["movements"]);
        
        // Test connection
        auto result = mDatabase->run_command(bsoncxx::builder::stream::document{} 
                                           << "ping" << 1 
                                           << bsoncxx::builder::stream::finalize);
        
        std::cout << "MongoDB connection successful!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "MongoDB connection failed: " << e.what() << std::endl;
        std::cerr << "Continuing without MongoDB." << std::endl;
    }
}

// Initialize action mapper
void Walk::initializeActionMapper() {
    std::cout << "Initializing action mapper..." << std::endl;
    
    // Define basic actions
    mActionMap["forward"] = GaitParams(1.0, 0.0, true);      // Forward
    mActionMap["backward"] = GaitParams(-1.0, 0.0, true);    // Backward
    mActionMap["left"] = GaitParams(0.0, 0.5, true);         // Left turn
    mActionMap["right"] = GaitParams(0.0, -0.5, true);       // Right turn
    mActionMap["stop"] = GaitParams(0.0, 0.0, false);        // Stop
    
    std::cout << "Action mapper initialization complete" << std::endl;
}

// Apply action from MongoDB
void Walk::applyActionFromMongoDB(const std::string& actionStr) {
    auto it = mActionMap.find(actionStr);
    if (it != mActionMap.end()) {
        GaitParams params = it->second;
        
        std::cout << "Executing action: " << actionStr 
                  << " (x:" << params.xAmplitude 
                  << ", a:" << params.aAmplitude 
                  << ", start:" << params.startGait << ")" << std::endl;
        
        // Apply simple walking pattern
        if (params.startGait && mMotors[10] && mMotors[11]) {
            double time = getTime();
            double walkPhase = fmod(time, 2.0) * M_PI;  // 2 second cycle
            
            // Leg movement (simple sine wave)
            double legAmplitude = 0.3 * params.xAmplitude;
            mMotors[10]->setPosition(-0.3 + legAmplitude * sin(walkPhase));      // LegUpperR
            mMotors[11]->setPosition(-0.3 + legAmplitude * sin(walkPhase + M_PI)); // LegUpperL
            
            // Knee movement
            if (mMotors[12] && mMotors[13]) {
                mMotors[12]->setPosition(0.6 + 0.2 * sin(walkPhase + M_PI/4));    // LegLowerR
                mMotors[13]->setPosition(0.6 + 0.2 * sin(walkPhase + M_PI*5/4));  // LegLowerL
            }
            
            // Rotation (pelvis rotation)
            if (mMotors[6] && mMotors[7]) {
                double turnAmplitude = 0.2 * params.aAmplitude;
                mMotors[6]->setPosition(turnAmplitude * sin(walkPhase));  // PelvYR
                mMotors[7]->setPosition(-turnAmplitude * sin(walkPhase)); // PelvYL
            }
        }
    } else {
        std::cout << "Unknown action: " << actionStr << std::endl;
    }
}

// Read and execute from MongoDB
void Walk::readAndExecuteFromMongoDB() {
    if (!mCollection) return;
    
    try {
        // Query latest commands
        auto filter = bsoncxx::builder::stream::document{} 
                     << "timestamp" << bsoncxx::builder::stream::open_document
                     << "$gte" << (std::chrono::system_clock::now().time_since_epoch().count() - 5000000000LL) // Within 5 seconds
                     << bsoncxx::builder::stream::close_document
                     << bsoncxx::builder::stream::finalize;
        
        auto cursor = mCollection->find(filter.view());
        
        for (auto&& doc : cursor) {
            if (doc["action"]) {
                std::string action = std::string(doc["action"].get_string().value);
                applyActionFromMongoDB(action);
                break;  // Execute only the latest one
            }
        }
        
    } catch (const std::exception& e) {
        // Handle MongoDB errors silently
    }
}

// MongoDB playback
void Walk::playbackFromMongoDB() {
    if (!mCollection) {
        std::cout << "MongoDB is not connected." << std::endl;
        return;
    }
    
    try {
        std::cout << "Starting MongoDB playback..." << std::endl;
        
        auto cursor = mCollection->find({});
        
        for (auto&& doc : cursor) {
            if (doc["action"]) {
                std::string action = std::string(doc["action"].get_string().value);
                std::cout << "Playback: " << action << std::endl;
                applyActionFromMongoDB(action);
                wait(1000);  // Wait 1 second
            }
        }
        
        std::cout << "MongoDB playback complete" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "MongoDB playback error: " << e.what() << std::endl;
    }
}
