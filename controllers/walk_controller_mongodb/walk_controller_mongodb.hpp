// walk_controller.hpp - MongoDB 기능 추가 버전
#ifndef walk_controller_mongodb_HPP
#define walk_controller_mongodb_HPP

#define NMOTORS 20

// 기존 Webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Gyro.hpp>

// 기존 표준 라이브러리
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <map>

// MongoDB C++ API headers (새로 추가)
#ifdef USE_MONGODB
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>
#include <chrono>
#include <optional>
#endif

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
    class Accelerometer;
    class Gyro;
    class Keyboard;
}

// ActionType enum (MongoDB용)
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

#ifdef USE_MONGODB
// MongoDB Controller 클래스 (조건부 컴파일)
class MongoDBController {
private:
    std::unique_ptr<mongocxx::instance> instance;
    std::unique_ptr<mongocxx::client> client;
    mongocxx::database database;
    mongocxx::collection collection;
    bool connected;
    
public:
    MongoDBController();
    ~MongoDBController() = default;
    
    bool isConnected() const { return connected; }
    std::optional<ActionType> getCurrentAction();
    bool saveAction(ActionType action, const std::string& source = "manual");
    
private:
    ActionType stringToActionType(const std::string& actionStr);
    std::string actionTypeToString(ActionType action);
    std::int64_t getCurrentTimestamp();
};
#endif

class Walk : public webots::Robot {
public:
    Walk();
    virtual ~Walk();
    
    void run();
    void checkIfFallen();

private:
    int mTimeStep;
    
    // 기존 멤버 변수들
    webots::Motor *mMotors[NMOTORS];
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Accelerometer *mAccelerometer;
    webots::Gyro *mGyro;
    webots::Keyboard *mKeyboard;
    
    // ROBOTIS managers (기존)
    managers::RobotisOp2MotionManager *mMotionManager;
    managers::RobotisOp2GaitManager *mGaitManager;
    
    // MongoDB 관련 (새로 추가, 조건부)
#ifdef USE_MONGODB
    std::unique_ptr<MongoDBController> mMongoController;
    bool mIsWalking;
    ActionType mLastAction;
    int mActionCheckCounter;
    std::map<ActionType, std::pair<double, double>> mActionMap; // x, a amplitudes
    
    void executeAction(ActionType action);
    void readAndExecuteFromMongoDB();
    void initializeActionMapper();
#endif
    
    // 기존 메서드들
    void myStep();
    void wait(int ms);
};

// Utility functions (조건부)
#ifdef USE_MONGODB
std::string actionTypeToString(ActionType action);
ActionType stringToActionType(const std::string& actionStr);
#endif

#endif // WALK_CONTROLLER_HPP
