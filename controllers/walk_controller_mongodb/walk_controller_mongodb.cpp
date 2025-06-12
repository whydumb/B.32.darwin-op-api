// walk_controller_mongodb.cpp
// MongoDB 연동 추가된 Walk Controller

#include "walk_controller_mongodb.hpp"

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <map>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  getGyro("Gyro")->enable(mTimeStep);

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

  // MongoDB 초기화
  initializeMongoDB();
  
  // ActionType 매핑 초기화
  initializeActionMapper();
}

Walk::~Walk() {
}

void Walk::initializeMongoDB() {
  try {
    mongocxx::instance inst{};
    mMongoClient = mongocxx::client{mongocxx::uri{}};
    mDatabase = mMongoClient["movement_tracker"];
    mCollection = mDatabase["movementracker"];
    
    cout << "MongoDB 연결 성공!" << endl;
  } catch (const std::exception& e) {
    cout << "MongoDB 연결 실패: " << e.what() << endl;
  }
}

void Walk::initializeActionMapper() {
  // ActionType별 매핑 설정
  mActionMap["forward"] = {1.0, 0.0, true};         // ↑: 전진
  mActionMap["backward"] = {-1.0, 0.0, true};       // ↓: 후진
  mActionMap["left"] = {0.0, 0.5, true};            // ←: 좌회전
  mActionMap["right"] = {0.0, -0.5, true};          // →: 우회전
  mActionMap["forward_left"] = {1.0, 0.5, true};    // ↑←: 전진+좌회전
  mActionMap["forward_right"] = {1.0, -0.5, true};  // ↑→: 전진+우회전
  mActionMap["backward_left"] = {-1.0, 0.5, true};  // ↓←: 후진+좌회전
  mActionMap["backward_right"] = {-1.0, -0.5, true}; // ↓→: 후진+우회전
  mActionMap["idle"] = {0.0, 0.0, false};           // 정지
}

void Walk::applyActionFromMongoDB(const string& actionStr) {
  auto it = mActionMap.find(actionStr);
  if (it != mActionMap.end()) {
    const GaitParams& params = it->second;
    
    // X축 진폭 설정 (전진/후진)
    mGaitManager->setXAmplitude(params.xAmplitude);
    
    // A축 진폭 설정 (좌우회전)
    mGaitManager->setAAmplitude(params.aAmplitude);
    
    // 걷기 시작/정지
    if (params.startGait && !mIsWalking) {
      mGaitManager->start();
      mIsWalking = true;
      wait(200);
    } else if (!params.startGait && mIsWalking) {
      mGaitManager->stop();
      mIsWalking = false;
      wait(200);
    }
    
    cout << "MongoDB Action 적용: " << actionStr 
         << " -> X: " << params.xAmplitude 
         << ", A: " << params.aAmplitude 
         << ", Gait: " << (params.startGait ? "Start" : "Stop") << endl;
  } else {
    cout << "알 수 없는 액션: " << actionStr << endl;
  }
}

void Walk::readAndExecuteFromMongoDB() {
  try {
    // 최신 문서부터 가져오기 (시간 순서대로)
    auto cursor = mCollection.find({}, mongocxx::options::find{}.sort(
      bsoncxx::builder::stream::document{} << "created_at" << -1 << bsoncxx::builder::stream::finalize
    ).limit(1));
    
    for (auto&& doc : cursor) {
      auto view = doc.view();
      
      // action 필드 읽기
      if (view["action"]) {
        string actionStr = view["action"].get_utf8().value.to_string();
        cout << "MongoDB에서 읽은 액션: " << actionStr << endl;
        applyActionFromMongoDB(actionStr);
        
        // 다른 데이터도 출력 (디버깅용)
        if (view["yaw"]) {
          cout << "Yaw: " << view["yaw"].get_double().value << endl;
        }
        if (view["pitch"]) {
          cout << "Pitch: " << view["pitch"].get_double().value << endl;
        }
        if (view["status"]) {
          cout << "Status: " << view["status"].get_utf8().value.to_string() << endl;
        }
      }
    }
  } catch (const std::exception& e) {
    cout << "MongoDB 읽기 오류: " << e.what() << endl;
  }
}

void Walk::playbackFromMongoDB() {
  try {
    cout << "MongoDB에서 모든 동작 재생 시작..." << endl;
    
    // 시간순으로 모든 문서 가져오기
    auto cursor = mCollection.find({}, mongocxx::options::find{}.sort(
      bsoncxx::builder::stream::document{} << "created_at" << 1 << bsoncxx::builder::stream::finalize
    ));
    
    for (auto&& doc : cursor) {
      auto view = doc.view();
      
      if (view["action"]) {
        string actionStr = view["action"].get_utf8().value.to_string();
        
        // 지속시간 읽기 (있는 경우)
        int duration = 3; // 기본값
        if (view["duration"]) {
          duration = view["duration"].get_int32().value;
        }
        
        cout << "재생 중: " << actionStr << " (지속시간: " << duration << "초)" << endl;
        
        // 액션 적용
        applyActionFromMongoDB(actionStr);
        
        // 지속시간만큼 대기
        for (int i = 0; i < duration; i++) {
          wait(1000); // 1초씩 대기
          mGaitManager->step(mTimeStep);
          myStep();
          
          // ESC 키를 누르면 재생 중단
          if (mKeyboard->getKey() == 27) { // ESC 키
            cout << "재생 중단됨" << endl;
            return;
          }
        }
      }
    }
    
    cout << "MongoDB 재생 완료" << endl;
  } catch (const std::exception& e) {
    cout << "MongoDB 재생 오류: " << e.what() << endl;
  }
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

void Walk::run() {
  cout << "-------ROBOTIS OP2 MongoDB 보행 컨트롤러-------" << endl;
  cout << "키보드 명령:" << endl;
  cout << "  스페이스바: 보행 시작/정지" << endl;
  cout << "  화살표 키: 로봇 움직임" << endl;
  cout << "  M: MongoDB에서 최신 액션 읽기" << endl;
  cout << "  P: MongoDB 전체 재생" << endl;
  cout << "  ESC: 재생 중단" << endl;

  myStep();

  // 초기 위치
  mMotionManager->playPage(9);
  wait(200);

  mIsWalking = false;

  while (true) {
    checkIfFallen();

    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);

    // 키보드 입력 처리
    int key = 0;
    while ((key = mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // 스페이스바 - 보행 토글
          if (mIsWalking) {
            mGaitManager->stop();
            mIsWalking = false;
            wait(200);
            cout << "보행 정지" << endl;
          } else {
            mGaitManager->start();
            mIsWalking = true;
            wait(200);
            cout << "보행 시작" << endl;
          }
          break;
          
        // 기존 화살표 키 제어
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
          
        // MongoDB 명령
        case 'M':
        case 'm':
          cout << "MongoDB에서 최신 액션 읽는 중..." << endl;
          readAndExecuteFromMongoDB();
          break;
          
        case 'P':
        case 'p':
          cout << "MongoDB 전체 재생 시작..." << endl;
          playbackFromMongoDB();
          break;
          
        case 27: // ESC
          cout << "프로그램 종료" << endl;
          exit(EXIT_SUCCESS);
          break;
      }
    }

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
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // 초기 위치
    fup = 0;
  } else if (fdown > acc_step) {
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // 초기 위치
    fdown = 0;
  }
}
