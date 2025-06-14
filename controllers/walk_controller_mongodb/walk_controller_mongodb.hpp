// walk_controller_mongodb.hpp
// MongoDB integrated Walk Controller Header for Webots
#ifndef WALK_MONGODB_HPP
#define WALK_MONGODB_HPP

#define NMOTORS 20

// Webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>

// MongoDB headers
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/instance.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>

// Standard C++ headers
#include <string>
#include <map>
#include <memory>
#include <iostream>

// Forward declarations for ROBOTIS managers
namespace managers {
    class RobotisOp2MotionManager;
    class RobotisOp2GaitManager;
}

// Forward declarations for Webots classes (optional, already included above)
namespace webots {
    class Motor;
    class PositionSensor;
    class LED;
    class Camera;
    class Accelerometer;
    class Gyro;
    class Keyboard;
    class Speaker;
}

// Gait parameters structure
struct GaitParams {
    double xAmplitude;  // Forward/backward movement (-1.0 ~ 1.0)
    double aAmplitude;  // Left/right rotation (-0.5 ~ 0.5)
    bool startGait;     // Whether to start walking
    
    // Constructor for easy initialization
    GaitParams(double x = 0.0, double a = 0.0, bool start = false) 
        : xAmplitude(x), aAmplitude(a), startGait(start) {}
};

class Walk : public webots::Robot {
public:
    // Constructor and destructor
    Walk();
    virtual ~Walk();
    
    // Main execution method
    void run();
    
    // Utility methods
    void checkIfFallen();

private:
    // Basic properties
    int mTimeStep;
    bool mIsWalking;
    
    // Basic methods
    void myStep();
    void wait(int ms);
    
    // MongoDB related methods
    void initializeMongoDB();
    void initializeActionMapper();
    void applyActionFromMongoDB(const std::string& actionStr);
    void readAndExecuteFromMongoDB();
    void playbackFromMongoDB();
    
    // Webots hardware interface
    webots::Motor *mMotors[NMOTORS];
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Accelerometer *mAccelerometer;
    webots::Keyboard *mKeyboard;
    
    // ROBOTIS managers
    managers::RobotisOp2MotionManager *mMotionManager;
    managers::RobotisOp2GaitManager *mGaitManager;
    
    // MongoDB related (using raw pointers for compatibility)
    mongocxx::client *mMongoClient;
    mongocxx::database *mDatabase;
    mongocxx::collection *mCollection;
    
    // Action mapping
    std::map<std::string, GaitParams> mActionMap;
};

#endif // WALK_MONGODB_HPP
