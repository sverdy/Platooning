#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

extern DeviceDriverSet_Motor AppMotor;
#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

#define MAX_DISTANCE 20
#define DISTANCE_SECU 20
#define TEMPS_INTERM 500000 //en us
#define DISTANCE_DANGER 10


ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_ULTRASONIC myUltrasonic;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/**
 * @brief Asservissement de la vitesse
 * 
 * @param distance 
 * @param new_speed 
 * @return uint8_t 
 */
static uint8_t asservissement_vitesse(float distance, uint8_t new_speed)
{
  if (distance < DISTANCE_DANGER)
  {
    new_speed = 0;
    return new_speed ;
  }
  else
  {
    if (distance > DISTANCE_SECU)
    {
      if (new_speed < 230)
      {
        new_speed += 10;
      }
      else
      {
        new_speed = 240;
      }
      return new_speed;
    }
    else if (distance < DISTANCE_SECU)
    {
      if (new_speed > 20)
      {
        new_speed -= 10;
      }
      else
      {
        new_speed = 25;
      }
      return new_speed;
    }
  }
}


/*运动方向控制序列*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};

Application_xxx Application_SmartRobotCarxxx0;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0)
  {
    /*清空串口缓存...*/
  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
}

static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}

static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //前进
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}

static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;


  Kp = 10;
  UpperLimit = 255;

  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {

      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
                                                 }
    else
    { //前进时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */

      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
      /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
     /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
     /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
      directionRecord = 10;
    break;
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{
  { /*R循迹状态更新*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    //ITR20001 检测小车是否离开地面
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }
}

uint8_t ApplicationFunctionSet::ApplicationFunctionSet_Tracking(uint8_t vitesse_courante)
{
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) //车子离开地面了？
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return 0;
    }
    float getAnaloguexxx_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    float getAnaloguexxx_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    float getAnaloguexxx_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    
    if (function_xxx(getAnaloguexxx_M, TrackingDetection_S, TrackingDetection_E))
    {
      /*控制左右电机转动：实现匀速直行*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, vitesse_courante);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(getAnaloguexxx_R, TrackingDetection_S, TrackingDetection_E))
    {
      /*控制左右电机转动：前右*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, vitesse_courante);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(getAnaloguexxx_L, TrackingDetection_S, TrackingDetection_E))
    {
      /*控制左右电机转动：前左*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, vitesse_courante);
      timestamp = true;
      BlindDetection = true;
    }
    else //不在黑线上的时候。。。
    {
      if (timestamp == true) //获取时间戳 timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      unsigned long m=millis();
      /*Blind Detection*/
      if ((function_xxx((m - MotorRL_time), 0, 200) || function_xxx((m - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      }
      else if (((function_xxx((m - MotorRL_time), 200, 1600))) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      }
      else if ((function_xxx((m - MotorRL_time), 3000, 3500))) // Blind Detection ...s ?
      {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  }
  else if (false == timestamp)
  {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
  return vitesse_courante;
}

uint8_t ApplicationFunctionSet::ApplicationFunctionSet_Platooning(uint8_t vitesse_courante) {

  static float distance;

  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&distance);
  Serial.print((String)(distance));

  return asservissement_vitesse(distance, vitesse_courante);
}
