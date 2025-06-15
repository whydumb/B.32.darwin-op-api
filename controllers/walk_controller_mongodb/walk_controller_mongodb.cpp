// walk_controller_mongodb.cpp - MongoDB 기능 추가 버전 (수정됨)
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
#include <bsoncxx/builder/stream/document.hpp>
#endif

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

#ifdef USE_MONGODB
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
#endif

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
  try {
    // MongoDB 초기화
    mMongoInstance = make_unique<mongocxx::instance>();
    mMongoClient = make_unique<mongocxx::client>(mongocxx::uri{});
    mCollection = (*mMongoClient)["movement_tracker"]["movementtracker"];
    mMongoConnected = true;
    mMongoCheckCounter = 0;
    cout << "[MongoDB] 연결 성공 - movement_tracker.movementtracker 컬렉션" << endl;
  } catch (const exception& e) {
    mMongoConnected = false;
    cout << "[MongoDB] 연결 실패: " << e.what() << endl;
  }
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
  if (!mMongoConnected) {
    return "idle";
  }

  try {
    // replay_name이 "your_database_name"인 문서만 필터링
    auto filter = document{} << "replay_name" << "your_database_name" << finalize;
    
    // 첫 번째 매칭 문서 찾기
    auto maybe_result = mCollection.find_one(filter.view());
    
    if (maybe_result) {
      auto doc = maybe_result.value();
      
      // current_action 필드 추출
      auto action_element = doc["current_action"];
      
      if (action_element && action_element.type() == bsoncxx::type::k_utf8) {
        string action = action_element.get_utf8().value.to_string();
        cout << "[MongoDB] replay_name='your_database_name'에서 current_action: " << action << endl;
        return action;
      } else {
        cout << "[MongoDB] current_action 필드가 없거나 문자열이 아닙니다." << endl;
        return "idle";
      }
    } else {
      cout << "[MongoDB] replay_name 'your_database_name'인 문서를 찾을 수 없습니다." << endl;
      return "idle";
    }
    
  } catch (const exception& e) {
    cout << "[MongoDB] 쿼리 오류: " << e.what() << endl;
    return "idle";
  }
}

void Walk::executeMongoAction(const string& action) {
  cout << "[실행] 액션: " << action << endl;
  
  if (action == "forward") {
    mGaitManager->setXAmplitude(1.0);
    mGaitManager->setAAmplitude(0.0);
    cout << "→ 앞으로 걷기" << endl;
  }
  else if (action == "backward") {
    mGaitManager->setXAmplitude(-1.0);
    mGaitManager->setAAmplitude(0.0);
    cout << "→ 뒤로 걷기" << endl;
  }
  else if (action == "left") {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.5);
    cout << "→ 왼쪽으로 회전" << endl;
  }
  else if (action == "right") {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(-0.5);
    cout << "→ 오른쪽으로 회전" << endl;
  }
  else { // "idle" 또는 기타
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    cout << "→ 제자리 서기" << endl;
  }
}
#endif

void Walk::run() {
  cout << "=======================================" << endl;
  cout << "MongoDB 통합 ROBOTIS OP2 Walk Controller" << endl;
  cout << "=======================================" << endl;
  
#ifdef USE_MONGODB
  cout << "MongoDB에서 replay_name='your_database_name'의" << endl;
  cout << "current_action 값을 읽어서 자동 제어합니다." << endl;
#else
  cout << "MongoDB 비활성화 - 정지 상태" << endl;
#endif
  cout << "=======================================" << endl;

  myStep();
  
  // 초기 위치로 이동
  cout << "[초기화] 기본 자세로 이동중..." << endl;
  mMotionManager->playPage(9);
  wait(200);

#ifdef USE_MONGODB
  if (mMongoConnected) {
    // 자동으로 걷기 시작
    mGaitManager->start();
    cout << "[시작] 걷기 모드 활성화" << endl;
    cout << "[대기] MongoDB에서 명령 대기중..." << endl;
  } else {
    cout << "[오류] MongoDB 연결 실패 - 정지 상태" << endl;
  }
#endif

  // 메인 루프 - MongoDB 데이터로 제어
  while (true) {
    checkIfFallen();

#ifdef USE_MONGODB
    if (mMongoConnected) {
      // 10번의 스텝마다 MongoDB 체크 (약 160ms마다)
      mMongoCheckCounter++;
      if (mMongoCheckCounter >= 10) {
        string mongoAction = getMongoAction();
        executeMongoAction(mongoAction);
        mMongoCheckCounter = 0;
      }
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
    cout << "[낙상감지] 앞으로 넘어짐 - 복구중..." << endl;
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9);  // init position
    fup = 0;
  }
  else if (fdown > acc_step) {
    cout << "[낙상감지] 뒤로 넘어짐 - 복구중..." << endl;
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9);  // init position
    fdown = 0;
  }
}
