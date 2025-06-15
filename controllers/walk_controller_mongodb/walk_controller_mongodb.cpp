// walk_controller_mongodb.cpp - MongoDB 기능 추가 버전
#include "walk_controller_mongodb.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#ifdef USE_MONGODB
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <bsoncxx/json.hpp>
#endif

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }



  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");

#ifdef USE_MONGODB
  // MongoDB 초기화 (매우 간단)
  mMongoInstance = make_unique<mongocxx::instance>();
  mMongoClient = make_unique<mongocxx::client>(mongocxx::uri{});
  mCollection = (*mMongoClient)["robot_control"]["current_action"];
  mMongoConnected = true;
  cout << "[MongoDB] 연결됨" << endl;
  mMongoCheckCounter = 0;
#endif
}

Walk::~Walk() {
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

#ifdef USE_MONGODB
string Walk::getMongoAction() {
  auto cursor = mCollection.find({});
  for (auto&& doc : cursor) {
    auto action_element = doc["action"];
    if (action_element && action_element.type() == bsoncxx::type::k_utf8) {
      return action_element.get_utf8().value.to_string();
    }
  }
  return "idle";
}

void Walk::executeMongoAction(const string& action) {
  if (action == "forward") {
    mGaitManager->setXAmplitude(1.0);
    mGaitManager->setAAmplitude(0.0);
  }
  else if (action == "backward") {
    mGaitManager->setXAmplitude(-1.0);
    mGaitManager->setAAmplitude(0.0);
  }
  else if (action == "left") {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.5);
  }
  else if (action == "right") {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(-0.5);
  }
  else { // "idle" 또는 기타
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
  }
}
#endif

void Walk::run() {
  cout << "-------MongoDB 전용 Walk Controller-------" << endl;
#ifdef USE_MONGODB
  cout << "MongoDB에서 action 읽어서 자동 제어" << endl;
#else
  cout << "MongoDB 비활성화 - 정지 상태" << endl;
#endif

  myStep();
  
  // 초기 위치
  mMotionManager->playPage(9);
  wait(200);

#ifdef USE_MONGODB
  // 자동으로 걷기 시작
  mGaitManager->start();
  bool isWalking = true;
  cout << "걷기 시작..." << endl;
#endif

  // 메인 루프 - MongoDB 데이터만으로 제어
  while (true) {
    checkIfFallen();

#ifdef USE_MONGODB
    // MongoDB에서 액션 읽고 바로 적용
    mMongoCheckCounter++;
    if (mMongoCheckCounter >= 10) {
      string mongoAction = getMongoAction();
      executeMongoAction(mongoAction);
      mMongoCheckCounter = 0;
    }
#endif

    mGaitManager->step(mTimeStep);
    myStep();
  }
}

void Walk::checkIfFallen() {
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

  if (fup > acc_step) {
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9);  // init position
    fup = 0;
  }
  else if (fdown > acc_step) {
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9);  // init position
    fdown = 0;
  }
}
