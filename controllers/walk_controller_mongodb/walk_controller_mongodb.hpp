#ifndef WALK_CONTROLLER_MONGODB_HPP
#define WALK_CONTROLLER_MONGODB_HPP

#include <webots/Robot.hpp>
#include <string>
#include <vector>
#include <map>

#define NMOTORS 20

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
  class Speaker;
}

struct MotionFrame {
  std::vector<double> positions;
  int duration;
  std::string name;
};

struct MotionSequence {
  std::string id;
  std::string name;
  std::vector<MotionFrame> frames;
};

class MongoDBWalkController : public webots::Robot {
public:
  MongoDBWalkController();
  virtual ~MongoDBWalkController();
  
  // MongoDB API 연동 함수들
  bool loadMotionFromAPI(const std::string& api_url, const std::string& motion_id);
  bool saveMotionToAPI(const std::string& api_url, const MotionSequence& motion);
  
  // 실행 함수들
  void runWithMongoDB();
  void playMotionSequence(const MotionSequence& motion);
  void recordMotionSequence(const std::string& motion_name);
  
  // 센서 데이터 로깅
  void logSensorData();
  void sendSensorDataToAPI(const std::string& api_url);
  
  // 파일 저장/로드
  void saveMotionToFile();
  bool loadMotionFromFile(const std::string& filename);

private:
  int mTimeStep;
  
  // Webots 컴포넌트들
  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::Accelerometer *mAccelerometer;
  webots::Gyro *mGyro;
  webots::Keyboard *mKeyboard;
  webots::Camera *mCamera;
  
  // ROBOTIS 매니저들
  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager *mGaitManager;
  
  // MongoDB 데이터
  MotionSequence mCurrentMotion;
  std::map<std::string, double> mSensorData;
  
  // 헬퍼 함수들
  void myStep();
  void wait(int ms);
  std::string httpRequest(const std::string& url, const std::string& data = "");
  void initializeRobot();
  void checkIfFallen();
  MotionSequence parseMotionFromJson(const std::string& json);
  std::string motionToJson(const MotionSequence& motion);
};

#endif