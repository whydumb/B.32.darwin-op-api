// walk_controller_mongodb.hpp
// MongoDB integrated Walk Controller Header for Webots (C API Version)
#ifndef WALK_MONGODB_HPP
#define WALK_MONGODB_HPP

#define NMOTORS 20

// Webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>

// MongoDB C API headers (더 안정적!)
#include <mongoc/mongoc.h>
#include <bson/bson.h>

// Standard C++ headers
#include <string>
#include <map>
#include <memory>
#include <iostream>
#include <cstring>
#include <ctime>

// Forward declarations for ROBOTIS managers
namespace managers {
    class RobotisOp2MotionManager;
    class RobotisOp2GaitManager;
}

// Forward declarations for Webots classes
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

// ActionType enum (Java 호환)
enum ActionType {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    IDLE
};

// Gait parameters structure
struct GaitParams {
    double xAmplitude;  // Forward/backward movement (-1.0 ~ 1.0)
    double aAmplitude;  // Left/right rotation (-0.5 ~ 0.5)
    bool startGait;     // Whether to start walking
    
    // Constructor for easy initialization
    GaitParams(double x = 0.0, double a = 0.0, bool start = false) 
        : xAmplitude(x), aAmplitude(a), startGait(start) {}
};

// C API MongoDB wrapper class
class CMongoController {
private:
    mongoc_client_t *client;
    mongoc_database_t *database;
    mongoc_collection_t *collection;
    bool connected;
    
public:
    CMongoController();
    ~CMongoController();
    
    bool isConnected() const { return connected; }
    ActionType getCurrentAction();
    bool saveAction(ActionType action);
    
private:
    ActionType stringToActionType(const char* actionStr);
    const char* actionTypeToString(ActionType action);
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
    ActionType mLastAction;
    int mActionCheckCounter;
    
    // Basic methods
    void myStep();
    void wait(int ms);
    
    // Action execution
    void executeAction(ActionType action);
    void initializeActionMapper();
    
    // MongoDB related methods
    void readAndExecuteFromMongoDB();
    
    // Webots hardware interface
    webots::Motor *mMotors[NMOTORS];
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Accelerometer *mAccelerometer;
    webots::Keyboard *mKeyboard;
    
    // ROBOTIS managers
    managers::RobotisOp2MotionManager *mMotionManager;
    managers::RobotisOp2GaitManager *mGaitManager;
    
    // MongoDB C API controller
    CMongoController *mMongoController;
    
    // Action mapping
    std::map<ActionType, GaitParams> mActionMap;
};

// Utility functions
const char* actionTypeToString(ActionType action);
ActionType stringToActionType(const char* actionStr);

#endif // WALK_MONGODB_HPP
