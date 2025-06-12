// walk_controller_mongodb.hpp
// MongoDB 연동이 추가된 Walk Controller 헤더

#ifndef WALK_MONGODB_HPP
#define WALK_MONGODB_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <string>
#include <map>

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

// 동작 파라미터 구조체
struct GaitParams {
  double xAmplitude;  // 전진/후진 (-1.0 ~ 1.0)
  double aAmplitude;  // 좌우회전 (-0.5 ~ 0.5)
  bool startGait;     // 걷기 시작 여부
};

class Walk : public webots::Robot {
public:
  Walk();
  virtual ~Walk();
  void run();
  void checkIfFallen();

private:
  int mTimeStep;
  bool mIsWalking;

  // 기본 메서드
  void myStep();
  void wait(int ms);

  // MongoDB 관련 메서드
  void initializeMongoDB();
  void initializeActionMapper();
  void applyActionFromMongoDB(const std::string& actionStr);
  void readAndExecuteFromMongoDB();
  void playbackFromMongoDB();

  // Webots 하드웨어 인터페이스
  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::Accelerometer *mAccelerometer;
  webots::Keyboard *mKeyboard;

  // ROBOTIS 관리자
  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager *mGaitManager;

  // MongoDB 관련
  mongocxx::client mMongoClient;
  mongocxx::database mDatabase;
  mongocxx::collection mCollection;

  // ActionType 매핑
  std::map<std::string, GaitParams> mActionMap;
};

#endif
