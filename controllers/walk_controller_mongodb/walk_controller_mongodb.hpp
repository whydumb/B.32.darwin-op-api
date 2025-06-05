// walk_controller_mongodb.hpp - Replay 기반 MongoDB 버전
#ifndef WALK_MONGODB_HPP
#define WALK_MONGODB_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>
#include <string>
#include <vector>
#include <curl/curl.h>
#include <json/json.h>

namespace managers {
    class RobotisOp2MotionManager;
    class RobotisOp2GaitManager;
}

namespace webots {
    class Motor;
    class PositionSensor;
    class LED;
    class Camera;
    class Accelerometer;
    class Gyro;
    class Keyboard;
}

struct ReplayAction {
    std::string id;
    std::string replay_name;
    std::string player_id;
    int order;
    long long timestamp;
    std::string action;
    double yaw;
    double pitch;
    int duration;
    long long created_at;
};

struct SensorData {
    double accelerometer[3];
    double gyro[3];
    double motorPositions[NMOTORS];
    double timestamp;
    bool isWalking;
    std::string robotState;
    std::string currentReplay;
    int currentOrder;
};

struct HTTPResponse {
    std::string data;
    long responseCode;
};

class WalkMongoDB : public webots::Robot {
public:
    WalkMongoDB();
    virtual ~WalkMongoDB();
    void run();
    void checkIfFallen();

private:
    int mTimeStep;

    // Robot control methods
    void myStep();
    void wait(int ms);
    
    // MongoDB/HTTP methods
    void initializeMongoDB();
    HTTPResponse httpRequest(const std::string& url, const std::string& method = "GET", 
                            const std::string& postData = "");
    
    // Replay methods
    bool loadReplay(const std::string& replayName, const std::string& playerId = "");
    void executeReplay();
    void executeAction(const ReplayAction& action);
    bool getNextAction();
    void saveCurrentAction();
    
    // Action execution methods
    void executeForward(double duration);
    void executeBackward(double duration);
    void executeTurnLeft(double yaw, double duration);
    void executeTurnRight(double yaw, double duration);
    void executeStop();
    void executeCustomMotion(int motionPage);
    
    // Data conversion methods
    Json::Value sensorDataToJson();
    ReplayAction jsonToReplayAction(const Json::Value& json);
    std::vector<ReplayAction> parseReplayActions(const Json::Value& actionsArray);
    Json::Value replayActionToJson(const ReplayAction& action);
    
    // Configuration
    void loadConfig();
    
    // Robot hardware
    webots::Motor *mMotors[NMOTORS];
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Accelerometer *mAccelerometer;
    webots::Gyro *mGyro;
    webots::Keyboard *mKeyboard;
    webots::Camera *mCamera;
    
    // Managers
    managers::RobotisOp2MotionManager *mMotionManager;
    managers::RobotisOp2GaitManager *mGaitManager;
    
    // MongoDB configuration
    std::string mongoApiUrl;
    std::string robotId;
    std::string currentPlayerId;
    int pollInterval;
    
    // Replay state
    std::string currentReplayName;
    std::vector<ReplayAction> replayActions;
    int currentActionIndex;
    bool isReplayActive;
    long long replayStartTime;
    
    // Robot state
    bool isWalking;
    SensorData currentSensorData;
    
    // HTTP client
    CURL* curl;
    
    // Callback for curl response
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, HTTPResponse* response);
};

#endif