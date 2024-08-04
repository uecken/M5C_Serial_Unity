#ifdef DEBUG_LOG
#include <epp_log.h>
#endif 

//#include <M5StickCPlus.h>
#ifndef _M5STICKC_H_
#include <M5StickC.h>
#endif
#include <vector>
#include <EEPROM.h>


//Select BTserial or MotionController
//#define BTSerial
#define MotionCont
#ifdef BTSerial
  #include "BluetoothSerial.h"
  BluetoothSerial bts;
#elif defined(MotionCont)
  //#include <AccelRingBuffer.hpp>
  //copilotでRingBufferの加速度インスタンス作成
  #include <MotionController.hpp>
  const uint8_t Fs = 50;
  const float Ts = (1000/Fs);
  MotionController mc(10,Fs);
  AccelRingBuffer acc_buffer(20);
#endif

#include <IMUReader.h>
IMUReader imur;




//Refered to AxisOrange https://github.com/naninunenoy/AxisOrange
#define TASK_DEFAULT_CORE_ID 1
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_IMU "IMUTask"
#define TASK_NAME_WRITE_SESSION "WriteSessionTask"
#define TASK_NAME_READ_SESSION "ReadSessionTask"
#define TASK_NAME_BUTTON "ButtonTask"
#define TASK_NAME_HID "HIDTask"
#define TASK_SLEEP_IMU Ts // = 1000[ms] / 100[Hz    //1000/13=76.9Hz
#define TASK_SLEEP_HID_SESSION 30 // = 1000[ms] / 100[Hz]
uint8_t task_sleep_hid_session_plus = 0;
#define TASK_SLEEP_WRITE_SESSION 151 // = 1000[ms] / 100[Hz]
#define TASK_SLEEP_READ_SESSION 201 // = 1000[ms] / 10[Hz]
#define TASK_SLEEP_BUTTON 15 // = 1000[ms] / 20[Hz]
#define MUTEX_DEFAULT_WAIT 10000UL
static void ImuLoop(void* arg);
static void ReadSessionLoop(void* arg);
static void WriteSessionLoop(void* arg);
static void ButtonLoop(void* arg);
static void hidSessionLoop(void* arg);
static SemaphoreHandle_t serWriteMutex = NULL;
static SemaphoreHandle_t imuDataMutex = NULL;
static SemaphoreHandle_t btnDataMutex = NULL;


uint8_t event=0;
#define NO_EVENT 0
#define INPUT_EVENT 1
#define INPUT1_EVENT 2
#define INPUT2_EVENT 3
#define INPUT3_EVENT 4
#define REGISTRATION_ROLL_EVENT 5
#define REGISTRATION_PITCH_EVENT 6
#define REGISTRATION_ROLL_PITCH_EVENT 7
#define xxxxx_EVENT 8
bool events_bool[3]; //events[0] = INPUT1_EVENT, events[1] = INPUT2_EVENT, events[2] = INPUT3_EVENT






const float CONVERT_G_TO_MS2 = 9.80665f;
const boolean left_axis_trans = true; //Unityは左手系、M5Stackは右手系

void calibrateMPU6886();
int split(String,char,String*);

float accX = 0.0F,accY = 0.0F,accZ = 0.0F;
float gyroX = 0.0F,gyroY = 0.0F,gyroZ = 0.0F;
float pitch = 0.0F,roll  = 0.0F,yaw   = 0.0F;
float unwrpRoll,unwrpYaw;
float accABS;
float* q_array = new float[4]; //quaternion
float aOX = -0.003, aOY = +0.018, aOZ =  0.085 ;  //-0.00   0.01   0.07 
float gOX = 3.516, gOY = -6.023 , gOZ = -5.14;  //3.36   9.66   4.11
float pO=0 , rO=0 , yO=-8.5, yO2=0;
float roll_prev,pitch_prev,yaw_prev;


boolean DEBUG_EEPROM = true;
boolean DEBUG_HID = true;
boolean BLEHID_ENABLE = true;


//===========Player Key settings2=============
struct pk2{
  uint16_t index;

  //===Trigger==
  uint8_t button_idx; //0~2
  short rpy[3]; //6byte
  float quatarnion[4]; //16byte
  float accX_triger;  
  float accY_triger;  
  float accZ_triger;   
  float accABS_trigger;
  float gyroX_triger;  
  float gyroY_triger;  
  float gyroZ_triger;
  float gyroABS_trigger;
  
  //==Outoput==
  uint8_t inputs_msg[20]; //ASCI+Key&Mouse or Gamepad

  //==Option==
  uint8_t hid_input_interval; //default:20ms
  uint8_t msg_format; //0:deafult 1:Gamepad
};



//===========Player Key settings===========
struct pk{ //44byte (padding 2Byte?)
  //mode設定
  uint8_t mode;

  //Priority設定
  //uint8_t skill_priority; //1~255 1:high , 255:low
  //Index設定
  uint8_t index; //1~255

  char rpy_priority;  //rpy 1byte //notused

  //モーション始端
  short roll_min;    // 2byte //notused
  short roll_max;    // 2byte //notused
  short pitch_min;    // 2byte //notused
  short pitch_max;    // 2byte //notused

  short rpy[3]; //6byte
  float quatanion[4]; //16byte

  //モーション終端
  /* 
  short end_roll_min;    // 2byte
  short end_roll_max;    // 2byte
  short end_pitch_min;    // 2byte
  short end_pitch_max;    // 2byte
  */

  //入力値
  char hid_input;   // 1byte
  char hid_inputs[8];   // 8byte

  uint8_t hid_input_interval; //コマンド入力間隔 1byte
  float hid_input_acc_threshold; //最終入力値の絶対加速度閾値 2byte
};

// roll -180 ~ +180, pitch -90 ~ +90, yaw -180 ~ +180

// ======== Street Fighter, MODE_STREET_FIGHTER_RANGE用, RightDisplay ==========
pk pk5 = {MODE_STREET_FIGHTER_RANGE,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
pk pk6 = {MODE_STREET_FIGHTER_RANGE,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_LEFT_ARROW,'a','\0'},50,3}; //波動拳
pk pk7 = {MODE_STREET_FIGHTER_RANGE,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk pk8 = {MODE_STREET_FIGHTER_RANGE,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{KEY_LEFT_ARROW,KEY_DOWN_ARROW,KEY_LEFT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
//pk pk9 = {5,'r',150,210,-30,30,'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
//pk pk10 = {5,'r',150,210,-30,30,'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)

pk p2_pk5 = {MODE_STREET_FIGHTER_RANGE,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3}; //波動拳
pk p2_pk6 = {MODE_STREET_FIGHTER_RANGE,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_LEFT_ARROW,'y','\0'},50,3}; //波動拳
pk p2_pk7 = {MODE_STREET_FIGHTER_RANGE,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{P2_KEY_RIGHT_ARROW,P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk p2_pk8 = {MODE_STREET_FIGHTER_RANGE,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{P2_KEY_LEFT_ARROW,P2_KEY_DOWN_ARROW,P2_KEY_LEFT_ARROW,'y','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)



// ======== Street Fighter, MODE_STREET_FIGHTER_DIST用, LeftDisplay ==========
pk pk5_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',0,0,0,0,{180,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
pk pk6_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',0,0,0,0,{180,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_LEFT_ARROW,'a','\0'},50,3}; //波動拳
pk pk7_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',0,0,0,0,{0,0,0},{0,0,0,0},'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk pk8_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',0,0,0,0,{0,0,0},{0,0,0,0},'1',{KEY_LEFT_ARROW,KEY_DOWN_ARROW,KEY_LEFT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk pk_a_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',0,0,0,0,{-90,0,0},{0,0,0,0},'1',{'v','a','\0'},50,2}; //パンチ

pk p2_pk5_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',-30,30,-30,30,{180,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3}; //波動拳
pk p2_pk6_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',-30,30,-30,30,{180,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_LEFT_ARROW,'y','\0'},50,3}; //波動拳
pk p2_pk7_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',150,210,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_RIGHT_ARROW,P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk p2_pk8_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',150,210,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_LEFT_ARROW,P2_KEY_DOWN_ARROW,P2_KEY_LEFT_ARROW,'y','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk p2_pk_y_leftdisp_dist = {MODE_STREET_FIGHTER_DIST,5,'r',-30,30,-30,30,{-90,0,0},{0,0,0,0},'1',{'v','y','\0'},50,2}; //パンチ




/*
pk pk2 = {'r',45,90,'2'};
pk pk3 = {'p',0,45,'3'};
pk pk4 = {'p',45,90,'4'a};
pk pk5 = {'p',45,90,'5'a};
*/
//pk pk_end = {'0',0,0,'0'}; //Do not use banpei
//pk pk_array[100] = {pk1,pk2,pk3,pk4,pk_end}; //Cannot use vector
//7pk pk_array2[100];
//uint8_t pk_size = sizeof(pk_array)/sizeof(pk_array[0]);

std::vector<pk> pk_vector{pk5,pk6};

std::vector<pk> pk_vector_sf_left_character{pk5,pk7};
std::vector<pk> pk_vector_sf_right_character{pk6,pk8};

std::vector<pk> p2_pk_vector_sf_left_character{p2_pk5,p2_pk7};
std::vector<pk> p2_pk_vector_sf_right_character{p2_pk6,p2_pk8};

std::vector<pk> pk_vector_sf_left_character_dist{pk5_leftdisp_dist,pk7_leftdisp_dist,pk_a_leftdisp_dist};
std::vector<pk> pk_vector_sf_right_character_dist{pk6_leftdisp_dist,pk8_leftdisp_dist,pk_a_leftdisp_dist};

std::vector<pk> p2_pk_vector_sf_left_character_dist{p2_pk5_leftdisp_dist,p2_pk7_leftdisp_dist,p2_pk_y_leftdisp_dist};
std::vector<pk> p2_pk_vector_sf_right_character_dist{p2_pk6_leftdisp_dist,p2_pk8_leftdisp_dist,p2_pk_y_leftdisp_dist};



std::vector<pk> pk_references;


//std::vector<pk> pk_vector_preset1{pk1,pk2,pk3,pk4,pk5};
//std::vector<pk> pk_vector_preset_FEZ{pk5};
std::vector<pk> pk_vector_preset_Action{pk5};

pk getClosestPK(std::vector<pk> pk_refecences);

struct message{
  short rpy[3]; //6byte
  char content[50]; //50byte
};
std::vector<message> message_vector;

//std::vector<pk> pk_vector;
//void flushPKarray(pk*);
//void readPKarray();
const uint16_t EEPROM_MOTIONCONT_START = 0;
const uint16_t EEPROM_MOTIONCONT_SIZE = 800;
//const uint16_t EEPROM_PRES1_START = 400;
//const uint16_t EEPROM_PRES1_SIZE = 400;
//const uint16_t EEPROM_PRES2_START = 800;
//const uint16_t EEPROM_PRES2_SIZE = 400;

const uint16_t EEPROM_CAL_SPACE_START = 800;
const uint16_t EEPROM_CAL_SPACE_SIZE = 100;
const uint16_t EEPROM_MESSAGE_SPACE_START = 900;
const uint16_t EEPROM_MESSAGE_SPACE_SIZE = 3190; //3190Byte/56Byte=56 message
//const uint16_t EEPROM_MESSAGE_SPACE_SIZE = 90000; //90KB

const uint16_t EEPROM_MAX_SIZE=4094;

//const uint16_t EEPROM_SIZE=EEPROM_MOTIONCONT_SIZE+EEPROM_PRES1_SIZE+EEPROM_PRES2_SIZE+EEPROM_CAL_SPACE_SIZE+EEPROM_CAL_SPACE_SIZE;
//const uint16_t EEPROM_SIZE=EEPROM_MOTIONCONT_SIZE+EEPROM_PRES1_SIZE+EEPROM_CAL_SPACE_SIZE+EEPROM_MESSAGE_SPACE_SIZE;
const uint16_t EEPROM_SIZE=EEPROM_MOTIONCONT_SIZE+EEPROM_CAL_SPACE_SIZE+EEPROM_MESSAGE_SPACE_SIZE;


//==============Player Key settings end==============

//==== Cannot Use EEPROMClass. =====
/*==== The error is [E][EEPROM.cpp:199] commit(): error in write ====
  ==== Use Only EEPROM ==== 
EEPROMClass TEST_ER('eeprom1');
EEPROMClass USER1_ER('eeprom2');
EEPROMClass USER2_ER('eeprom3');
EEPROMClass PRES1_ER('eeprom4');
EEPROMClass PRES2_ER('eeprom5');
*/

void flushPKvector(std::vector<pk>,String);
void readPKvector(String);
void deleteEEPROM(String);
void remove_crlf(std::string&);
//void serialprint_pk_references(std::vector<pk>&,String);
void serialprint_pk_references(String);
void serialprint_motion_messages(String);

void flushMessageVector(std::vector<message> message_vector,String preset_name);
void readMessageVector(String preset_name);
message getClosestMessage(std::vector<message> message_refs);
void serialOutput(pk &pk , String msg_type);
void unityAppInit();


String input_serial;
char input_serial_char;
String input_serial_str;
uint8_t num_powerbtn_click=1;
//boolean serial_ON = false; //mcクラスで管理
//unsigned long Tcur,Tpre;


TaskHandle_t taskHandleIMU = NULL;

void setup() {


  #ifdef BTSerial
   bts.begin("M5C_BTSerial");
  #elif defined(MotionCont)
   //mc.mode=MODE_MOTION_MASSAGE;
   mc.mode=MODE_STREET_FIGHTER_DIST;
   mc.begin();
  #endif

  serWriteMutex = xSemaphoreCreateMutex();
  imuDataMutex = xSemaphoreCreateMutex();
  btnDataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(ImuLoop, TASK_NAME_IMU, TASK_STACK_DEPTH, NULL, 2, &taskHandleIMU, TASK_DEFAULT_CORE_ID); 
  xTaskCreatePinnedToCore(WriteSessionLoop, TASK_NAME_WRITE_SESSION, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ReadSessionLoop, TASK_NAME_READ_SESSION, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ButtonLoop, TASK_NAME_BUTTON, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(hidSessionLoop, TASK_NAME_HID, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);


  //calibrate
  /*if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
    //calibrateMPU6886();
    xSemaphoreGive(imuDataMutex);
  }*/
 
  //保存したPlayerkeyベクトルの読みだし
  if(EEPROM_SIZE>EEPROM_MAX_SIZE){
    if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over 4095(EEPROM_MAX_SIZE)");
  }
  else{
    if(!EEPROM.begin(EEPROM_SIZE)){
      Serial.println("ERROR:EEPROM begin failed"); //EEPROM.begin(size)で確保できる最大サイズは通常、最大4095バイト（4KB未満）
    }else{
      Serial.println("ERROR:EEPROM begin succesed"); //EEPROM.begin(size)で確保できる最大サイズは通常、最大4095バイト（4KB未満）
    }
    //readPKvector("MOTION_CONT");
  }

  //EEPROMから読んだCalibration値を読み出し
  uint16_t n=EEPROM_CAL_SPACE_START;
  uint8_t size;
  EEPROM.get(n,size); n+=sizeof(size);
  EEPROM.get(n,aOX); n+=sizeof(aOX);
  EEPROM.get(n,aOY); n+=sizeof(aOY);
  EEPROM.get(n,aOZ); n+=sizeof(aOZ);
  EEPROM.get(n,gOX); n+=sizeof(gOX);
  EEPROM.get(n,gOY); n+=sizeof(gOY);
  EEPROM.get(n,gOZ); n+=sizeof(gOZ);

  Serial.printf("Read Cal:%d, %2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f\r\n", size, aOX, aOY, aOZ,gOX, gOY, gOZ);
  if((abs(aOX) < 1 & abs(aOY) < 1 & abs(aOZ) < 1) & (abs(aOX) != 0 & abs(aOY) != 0 & abs(aOZ) != 0)){
    Serial.printf("Read Cal:%d, %2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f\r\n", size, aOX, aOY, aOZ,gOX, gOY, gOZ);
  }else{
    Serial.println("Not calibrated properly. Reseted EEPROM. Push M5 button 1 seconds and release to calibrate.");
    deleteEEPROM("ALL");
  }
   

  if(LEFT_DISPLAY){
    Serial.println("LEFT_DISPLAY,");
  }
 
  vTaskDelay(500);
  mc.setInitialQuat(q_array);
  //TEST_ER.begin(EEPROM_SIZE/5); //8Byte * 100/20;
  /*
  USER1_ER.begin(EEPROM_SIZE/5);
  USER2_ER.begin(EEPROM_SIZE/5); 
  PRES1_ER.begin(EEPROM_SIZE/5); 
  PRES2_ER.begin(EEPROM_SIZE/5); 
  */
  //flushPKarray(pk_array);
  //readPKarray();
  
  //flushPKvector(pk_vector);
  //readPKvector();

/*
EEPROM.put(0,pk_array);
EEPROM.commit();
pk pk_array;

EEPROM.get(1,pk_array);
int i =0;
Serial.println(String(pk_array[i].rpy_priority) +"," +String(pk_array[i].roll_min) + "," + String(pk_array[i].roll_max) + "," +String(pk_array[i].hid_input));
*/

}

void loop() {
  vTaskDelay(1000);
  }


static void ImuLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();
      roll_prev = roll;
      pitch_prev = pitch;
      yaw_prev = yaw;
      
    //if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
      M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
      M5.IMU.getAccelData(&accX,&accY,&accZ);
      accX -= aOX; accY -= aOY; accZ -= aOZ; 
      //accABS = sqrt(accX*accX + accY*accY + accZ*accZ);
      gyroX -=gOX; gyroY -=gOY; gyroZ -=gOZ;


      //クォータニオンの初期化. 
      //Roll/PitchもOffsetされるが、MafoniFilterによりすぐに正しい値となる
      //そのためYawだけOffsetされることになる
      if(mc.initial_quat_offset_enable){
        //mc.quatMultiply(q_array,mc.initial_quat);

        float Q[4] = {1.0,0.0,0.0,0.0}; //wxyz,M5Cにおける初期クォータニオン. MahonyFilterで設定されている
        M5.IMU.setQuaternion(Q);
        Serial.printf("%f,%f,%f,%f",Q[0],Q[1],Q[2],Q[3]);
        mc.initial_quat_offset_enable = false;
      }

      q_array = M5.IMU.getAhrsData(&pitch,&roll,&yaw,aOX,aOY,aOZ,gOX,gOY,gOZ); //w,x,y,z

      if(mc.q_offset_enable){
        mc.quatMultiply(q_array,mc.q_offset);
      }

      //q_array = M5.IMU.getAhrsData2(&pitch,&roll,&yaw,aOX,aOY,aOZ,gOX,gOY,gOZ);
      pitch -=pO;  roll -= rO;  yaw -= yO;//★最小値-188.5。原因はgyroZの補正ができていないからか？一時的に+8.5して補正する。
      if(LEFT_DISPLAY){
        pitch = -pitch;
        roll = -roll;
        yaw = -yaw;
      }
      

      mc.addSensorData(accX,accY,accZ,gyroX,gyroY,gyroZ,roll,pitch,yaw);

    //}
    //xSemaphoreGive(imuDataMutex);

    acc_buffer.add(accX, accY, accZ);
    unwrpRoll = imur.unwrappingRoll(roll,pitch);
    unwrpYaw = imur.unwrappingYaw(yaw);

    int32_t sleep = TASK_SLEEP_IMU - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void hidSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();

    //if (xSemaphoreTake(serWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 line serial Input

      //姿勢入力判定
      /*
      if(event==INPUT_EVENT){
        if(pk1.roll_min < roll && roll < pk1.roll_max) Serial.println(String("In,")+String(pk1.hid_input)); // NG "In,"+pk1.input. Buffer Over
        else if(pk2.roll_min < pitch && pitch < pk2.roll_max) Serial.println(String("In,")+String(pk2.hid_input));
        M5.Lcd.setCursor(30, 120);
        M5.Lcd.println(pk1.hid_input);
        event = NO_EVENT;
      }*/
      if(mc.mode==MODE_TEST_BUTTONB){
        if(event==INPUT_EVENT){
          //登録キーから判定
          for(uint8_t i = 0; i < pk_vector.size(); i++){
            /*
            int angle = -999;
            switch(pk_vector[i].rpy_priority){
              case 'r' : angle = roll; break;
              case 'p' : angle = pitch; break;
            }
            */
            //Rollが180度をまたぐ場合
            if(pk_vector[i].roll_min < 180 && pk_vector[i].roll_max > 180){
            //例えばRoll 150 ~ 210の場合、150°以上または-150°以下の場合に入力を受け付ける
              if((pk_vector[i].roll_min < roll) && (-1 *(180 - (180 - pk_vector[i].roll_max)) < roll )){
                mc.execSF_HIDInputs(pk_vector[i].hid_inputs,(float)pk_vector[i].hid_input_acc_threshold,&acc_buffer);
              }
            } 
            
            //通常の場合
            else if(pk_vector[i].roll_min < roll && roll < pk_vector[i].roll_max){
              if(mc.serial_ON)Serial.println(String("In,")+String(pk_vector[i].hid_input) + ","+String(pk_vector[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              M5.Lcd.setCursor(10,60);
              M5.Lcd.println(String("In,")+String(pk_vector[i].hid_input) + ","+String(pk_vector[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              mc.execSF_HIDInputs(pk_vector[i].hid_inputs,(float)pk_vector[i].hid_input_acc_threshold,&acc_buffer);
              break;
            }
          }
        }
      }

      std::vector<pk> pks;
      pk pk_selected;
      pks = pk_references;
      if(mc.mode==MODE_MOTION_CONTROLLER){
        if(events_bool[0]==true){
          pk_selected = getClosestPK(pks);
          if(BLEHID_ENABLE)mc.inputKey(pk_selected.hid_input);
          //if(DEBUG_HID)Serial.printf("selected_pk,%c,%s,%d,%d, current_rp:%f,%f \r\n",pk_selected.hid_input,pk_selected.hid_inputs,pk_selected.rpy[0],pk_selected.rpy[1],roll,pitch);
          if(DEBUG_HID) serialOutput(pk_selected, "SELECTED_PK");
          //if(DEBUG_HID)Serial.printf("selected_pk,%c,%d,%d \r\n",pk_selected.hid_input,pk_selected.rpy[0],pk_selected.rpy[1]);
          events_bool[0]=false;
        }
      }
      else if(mc.mode==MODE_MOTION_MASSAGE){
        if(events_bool[0]==true){
          //if(DEBUG_HID)Serial.println("Sending_MASSAGE");
          message message_selected = getClosestMessage(message_vector);


          //送信に7ms/文字数 必要
          if(BLEHID_ENABLE)mc.inputKeys(message_selected.content);
          //if(BLEHID_ENABLE)bleCombo.write(reinterpret_cast<const uint8_t*>(message_selected.content), sizeof(message_selected.content));

          
          //if(DEBUG_HID)Serial.printf("MOTION_MESSAGE_selected,%s \n",message_selected.content);
          if(DEBUG_HID)Serial.printf("MOTION_MESSAGE_selected,%s,%d,%d,%d \r\n",message_selected.content, message_selected.rpy[0], message_selected.rpy[1], message_selected.rpy[2]);
          //描画を入れると5回に一度程度落ちる。
          //Userassert failed: xQueueGenericSend queue.c:832 (pxQueue->pcHead != ((void *)0) || pxQueue->u.xSemaphore.xMutexHolder == ((void *)0) || pxQueue->u.xSemaphore.xMutexHolder == xTaskGetCurrentTaskHandle())

          //M5.Lcd.fillScreen(BLACK);
          //M5.Lcd.setCursor(5, 35);
          //String msg_space = message_selected.content + "                   ";
          //M5.Lcd.println(message_selected.content);
          
          events_bool[0]=false;
        }
      /*
      else if(mc.mode==MODE_MOTION_MASSAGE){
        if(events_bool[0]==true){
          pk_selected = getClosestPK(pks);
          if(BLEHID_ENABLE)mc.inputKeys(pk_selected.hid_inputs);
          if(DEBUG_HID)Serial.printf("selected_pk:%c,%s,%d,%d, current_rp:%f,%f \r\n",pk_selected.hid_input,pk_selected.hid_inputs,pk_selected.rpy[0],pk_selected.rpy[1],roll,pitch);
          events_bool[0]=false;
        }
      */
      }
      else if(mc.mode==MODE_MOUSE){
        float mouse_scale=5;
        float pitch_delta = mouse_scale*mc.mouse_scale2*(pitch - pitch_prev);
        float yaw_delta = mouse_scale*mc.mouse_scale2*(yaw - yaw_prev);
        if(events_bool[0]==true && events_bool[1]==true){
          if(!LEFT_DISPLAY){
            
          }
          else if(LEFT_DISPLAY){
            //pitch変化が負の場合スクロールアップ、正の場合スクロールダウン
            if(pitch_delta < 0){
              bleCombo.move(0,0,1);
            }else if(pitch_delta > 0){
              bleCombo.move(0,0,-1);
            }
          }
        }else if(events_bool[0]==true){
          if(!LEFT_DISPLAY){
            
          }
          else if(LEFT_DISPLAY){
            //pitch_deltaがマイナスの場合、マウスの上下移動が逆になるので符号を反転させる
            mc.moveMouse(yaw_delta,pitch_delta);
          }
        }else if(events_bool[1]==true){
          if(!LEFT_DISPLAY){
            
          }
          else if(LEFT_DISPLAY){
            //bleCombo.press(MOUSE_LEFT);
            bleCombo.click(MOUSE_LEFT);
            /*
            float exponent = 1.5;
            float mouse_scale=5;
            float pitch_delta = mouse_scale*(pitch - pitch_prev);
            float yaw_delta = mouse_scale*(yaw - yaw_prev);
            //pitch_deltaがマイナスの場合、マウスの上下移動が逆になるので符号を反転させる
            float xMove = (yaw_delta < 0) ? -1 * round(pow(yaw_delta,exponent)) : round(pow(yaw_delta,exponent));
            float yMove = (pitch_delta < 0) ? -1 * round(pow(pitch_delta,exponent)) : round(pow(pitch_delta,exponent));

            mc.moveMouse(xMove,yMove);
            //mc.moveMouse(yaw_delta,pitch_delta);
            */
          }
        }else if(events_bool[2]==true){
            bleCombo.click(MOUSE_RIGHT);
        }
      }else if(mc.mode==MODE_STREET_FIGHTER_DIST){
        std::vector<pk> pks;
        pk pk_selected;
        static boolean prev_events_bool[3] = {false,false,false};

        //キャラ移動設定
        char l;
        char r;
        char u;
        char d;
        if(mc.current_game_mode==MODE_SF_P1){
          l = KEY_LEFT_ARROW;
          r = KEY_RIGHT_ARROW;
          u = KEY_UP_ARROW;
          d = KEY_DOWN_ARROW;
        }else if(mc.current_game_mode==MODE_SF_P2){
          l='j'; r='l'; u='i'; d='k'; 
        }


        //キーボードマウス入力(button2の場合)
        if(events_bool[2]==true){
          //roll=-90度が基準
          /*
          if((roll<-120 & roll>-180) || (roll>90 && roll<180)) mc.inputKey(l);
          else if((roll>-60 & roll<0) || (roll>0 && roll<90)) mc.inputKey(r);
          //else{mc.releaseKey(l);mc.releaseKey(r);}

          //pitch=0度が基準
          if(pitch<-40) mc.inputKey(u);
          else if(pitch>40) mc.inputKey(d);
          //else{mc.releaseKey(u);mc.releaseKey(d);}
          */

          /*
          if((roll<-120 && roll>-180) || (roll>90 && roll<180)) mc.pressKey(l);
          else {mc.releaseKey(l);}
          if((roll>-60 && roll<0) || (roll>0 && roll<90)) mc.pressKey(r);
          else{mc.releaseKey(r);}

          if(pitch<-40) mc.pressKey(u);

          else if(pitch>40) mc.pressKey(d);
          else{mc.releaseKey(u);mc.releaseKey(d);}
          */

          
          //2キー選択
          //if(pitch>30 && roll<-120){ mc.pressKey(d); mc.pressKey(l);}
          //else if(pitch>30 && roll>-60){ mc.pressKey(d); mc.pressKey(r);}

          //手前に倒すと下ボタン
          if(pitch<-30 && roll<-120){ mc.pressKey(d); vTaskDelay(30); mc.pressKey(l);}
          else if(pitch<-30 && roll>-60){ mc.pressKey(d); vTaskDelay(30); mc.pressKey(r);}
          else{
            //1キー選択
            //pitch=0度が基準
            if(pitch<-30) {mc.pressKey(d); }//mc.pressKey(u); }
            else if(pitch>30) {mc.pressKey(u); }//mc.pressKey(d);}
            else{mc.releaseKey(u);mc.releaseKey(d);}          

            //roll=-90度が基準
            if((roll<-120 && roll>-180) || (roll>90 && roll<180)) mc.pressKey(l);
            else if((roll>-60 && roll<0) || (roll>0 && roll<90)) mc.pressKey(r);
            else{mc.releaseKey(l);mc.releaseKey(r);}
          }
          prev_events_bool[2] = true;

          //キャラ移動しながらボタン押す事で、必殺技を出せるようにする
          if(events_bool[0]==true || events_bool[1]==true){
            vTaskDelay(30);
            mc.keyboardReleaseAll();
            vTaskDelay(30);

            if(mc.current_game_mode==MODE_SF_P1){
              //mc.inputKey('a');
              mc.pressKey('a');
              //mc.pressKey('h');
              vTaskDelay(30);
            }else
            if(mc.current_game_mode==MODE_SF_P2){
              //mc.inputKey('y');
              mc.pressKey('y');
              //mc.pressKey('h');
              vTaskDelay(30);
            }
            mc.keyboardReleaseAll();
          }
        }


        //キーボード設定
        if(events_bool[0]==true){//キャラが左に居る場合
          if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_left_character_dist;}
          else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_left_character_dist;}
        }else if(events_bool[1]==true){ //キャラが右に居る場合
          if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_right_character_dist;}
          else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_right_character_dist;}
        }

        //キーボード入力(マウス押してない場合)
        if((events_bool[0]==true || events_bool[1]==true) && events_bool[2]==false){  
          //キー判定
          pk_selected = getClosestPK(pks);
          if(BLEHID_ENABLE)mc.execSF_HIDInputs(pk_selected.hid_inputs,(float)pk_selected.hid_input_acc_threshold,&acc_buffer);
          //if(DEBUG_HID)Serial.printf("SF,selected_pk:%s,%d,%d, current_rp:%f,%f \r\n",pk_selected.hid_inputs,pk_selected.rpy[0],pk_selected.rpy[1],roll,pitch);
          //unityでsplitするため、selected_pkにする必要がある
          //if(DEBUG_HID)Serial.printf("selected_pk,%c,%d,%d, current_rp:%f,%f \r\n",pk_selected.hid_input,pk_selected.rpy[0],pk_selected.rpy[1],roll,pitch);
          if(DEBUG_HID)serialOutput(pk_selected,"SELECTED_PK");

          //if(DEBUG_HID)Serial.printf("selected_pk:%c,%d,%d \r\n",pk_selected.hid_input,pk_selected.rpy[0],pk_selected.rpy[1]);
          events_bool[0]=false;
          events_bool[1]=false;
        }//else

        //Release
        if (prev_events_bool[2] == true && events_bool[2]==false){
          mc.releaseKey(l);mc.releaseKey(r);mc.releaseKey(u);mc.releaseKey(d);
        }

        //button2をreleaesした時に、該当プレイヤーのキャラ移動停止
        /*
        else
        if (mc.prev_button_state[2]==true && events_bool[2]==false){
          mc.releaseKey(l);mc.releaseKey(r);mc.releaseKey(u);mc.releaseKey(d);
        }
        */

      }else if(mc.mode==MODE_STREET_FIGHTER_RANGE){
        std::vector<pk> pks;
        if(events_bool[0]==true){
          if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_left_character;}
          else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_left_character;}

          for(uint8_t i = 0; i < pks.size(); i++){
            //Rollが180度をまたぐ場合
            if(pks[i].roll_min < 180 && pks[i].roll_max > 180){
            //例えばRoll 150 ~ 210の場合、150°以上または-150°以下の場合に入力を受け付ける
              if((pks[i].roll_min < roll) && (-1 *(180 - (pks[i].roll_max - 180)) < roll )){
                mc.execSF_HIDInputs(pks[i].hid_inputs,(float)pks[i].hid_input_acc_threshold,&acc_buffer);
              }
            }
            
            //通常の場合
            else if(pks[i].roll_min < roll && roll < pks[i].roll_max){
              if(mc.serial_ON)Serial.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              M5.Lcd.setCursor(10,60);
              M5.Lcd.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              mc.execSF_HIDInputs(pks[i].hid_inputs,(float)pks[i].hid_input_acc_threshold,&acc_buffer);
              break;
            }
          }
          events_bool[0]=false;
        }
        else if(events_bool[1]==true){
          if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_right_character;}
          else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_right_character;}

          for(uint8_t i = 0; i < pks.size(); i++){
            //Rollが180度をまたぐ場合
            if(pks[i].roll_min < 180 && pks[i].roll_max > 180){
            //例えばRoll 150 ~ 210の場合、150°以上または-150°以下の場合に入力を受け付ける
              if((pks[i].roll_min < roll) && (-1 *(180 - (pks[i].roll_max - 180)) < roll )){
                mc.execSF_HIDInputs(pks[i].hid_inputs,(float)pks[i].hid_input_acc_threshold,&acc_buffer);
              }
            }
            
            //通常の場合
            else if(pks[i].roll_min < roll && roll < pks[i].roll_max){
              if(mc.serial_ON)Serial.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              M5.Lcd.setCursor(10,60);
              M5.Lcd.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              mc.execSF_HIDInputs(pks[i].hid_inputs,(float)pks[i].hid_input_acc_threshold,&acc_buffer);
              break;
            }
          }
          events_bool[1]=false;
        }
      }

/*
      if(event==REGISTRATION_ROLL_EVENT){
        char input_key;
        if(input_serial_char!=NULL) input_key = input_serial_char;
        else input_key = num_powerbtn_click;
        
        pk pk_reg = {'r',roll-22.5,roll+22.5,input_key};
        pk_vector.push_back(pk_reg);

        M5.Lcd.println(Serial.readStringUntil('/0'));
        
        for(uint8_t i = 0; i < pk_vector.size(); i++){
          M5.Lcd.setCursor(10,80+i*7);
          M5.Lcd.println(String(pk_vector[i].rpy_priority) +"," +String(pk_vector[i].roll_min) + "," + String(pk_vector[i].roll_max) + "," +String(pk_vector[i].hid_input));
        }

        input_serial_char = NULL;
        num_powerbtn_click = 1;
      }
      else if(event==REGISTRATION_PITCH_EVENT){
        char input_key = 'p';
        pk pk_reg = {'p',pitch-22.5,pitch+22.5,input_key};
        pk_vector.push_back(pk_reg);
      }
*/

      //modeに依存せず登録
      if(event==REGISTRATION_ROLL_PITCH_EVENT){
        //pk pk5 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
        String s = input_serial;
        if(s==""){
          Serial.println("No innout");
        }else{
          if(mc.mode==MODE_MOTION_MASSAGE){
            char cs[50];
            input_serial.toCharArray(cs,50);
            //message message = { {roll,pitch,yaw}, cs}; //C++では配列に直接配列コピーできない
            message message;
            message.rpy[0] = roll;
            message.rpy[1] = pitch;
            message.rpy[2] = yaw;
            strncpy(message.content, cs, sizeof(message.content) - 1);
            message.content[sizeof(message.content) - 1] = '\0'; // null終端文字を確実に追加
            message_vector.push_back(message);
          }else{
            //単一文字の場合
            //pk pk_reg = {MODE_STREET_FIGHTER_RANGE,6,'r',30,60,-30,30,{roll,pitch,yaw},{q_array[0],q_array[1],q_array[2],q_array[3]},input_key,{input_keys,'\0'},50,0};
            uint8_t index = sizeof(pk_references);
            pk pk_reg = {MODE_STREET_FIGHTER_RANGE,index,'r',30,60,-30,30,{roll,pitch,yaw},{q_array[0],q_array[1],q_array[2],q_array[3]},s[0], { s[0],s[1],s[2],s[3],s[4],s[5],s[6],'\0'},50,0};
            pk_references.push_back(pk_reg);
          }
        }
        //連続文字の場合
          //後日実装
        
        input_serial = "";
        input_serial_char =  NULL;
      }

      if(mc.mode!=MODE_STREET_FIGHTER_DIST){
        mc.keyboardReleaseAll(); //マルチプレイはPC共有のためReleaseAll()は使用禁止
      }
      event = NO_EVENT;
    //}
    //xSemaphoreGive(serWriteMutex);

    // idle
    int32_t sleep = TASK_SLEEP_HID_SESSION+task_sleep_hid_session_plus - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void WriteSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();

    //if (xSemaphoreTake(serWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 lin serial Input
      if(event==NO_EVENT){
        if(left_axis_trans){//左手系
          //if(mc.serial_ON)Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          if(mc.serial_ON)Serial.printf("sensor_data,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          #ifdef BTSerial 
            //if(mc.serial_ON)bts.printf("%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
            if(mc.serial_ON)bts.printf("%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, yaw-yO2,q_array[0],q_array[1],q_array[2],q_array[3]);

          #endif
        }else{//右手系
          Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, yaw-yO2);
        }
        M5.Lcd.setCursor(30, 60);
        if((abs(aOX) < 1 & abs(aOY) < 1 & abs(aOZ) < 1) & (abs(aOX) != 0 & abs(aOY) != 0 & abs(aOZ) != 0)){
          M5.Lcd.println(String(pitch) + " " + String(roll) + " " + String(yaw-yO2));
        }else{
          M5.lcd.println("Not calibrated properly");
        }
        //M5.Lcd.println(String(pitch) + " " + String(unwrpRoll) + " " + String(unwrpYaw-yO2));   
        }

        //if(mc.serial_ON)Serial.println(acc_buffer.getMaxAbsAccel(5));
    //}
    //xSemaphoreGive(serWriteMutex);

    // idle
    int32_t sleep = TASK_SLEEP_WRITE_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

String inputs[10] = {"\0"};
static void ReadSessionLoop(void* arg){
  while (1) {
    uint32_t entryTime = millis();
    if(Serial.available()){//受信データ確認
      //M5.Lcd.setCursor(10, 110);
      
      input_serial = Serial.readStringUntil('/0');
      input_serial.trim(); //remove (" ","\t","\v","\f","\r","\n") not split
      input_serial_char = input_serial[0];
      Serial.println(input_serial);
      //M5.Lcd.println(input_serial);

      int num_length ;
      if(input_serial.indexOf(',')>0){//.が存在する場合
        num_length = split(input_serial,',',inputs);
        String command = String(inputs[0]);
        //String value = String(inputs[1]);
        Serial.println("split:" + command + "," + inputs[1]);
        if(command == "FLUSH") flushPKvector(pk_references,inputs[1]);
        else if(command == "READ") readPKvector(inputs[1]);
        else if(command == "DELETE") deleteEEPROM(inputs[1]);
        else if(command == "SERIAL" && inputs[1] == "ON") mc.serialON();
        else if(command == "SERIAL" && inputs[1] == "OFF") mc.serialOFF();
        else if(command == "UNITY" && inputs[1] == "START") unityAppInit();
      }else if(mc.mode==MODE_MOTION_MASSAGE){
        if(input_serial == "FLUSH") flushMessageVector(message_vector,"MESSAGE");
        else if(input_serial == "READ")readMessageVector("MESSAGE");
        else if(input_serial == "DELETE") deleteEEPROM("MESSAGE");        
        else if(input_serial == "SHOW") Serial.println("not implemented now");
        else if(input_serial == "SHOW_REFERENCES") serialprint_motion_messages("csv");
        else if(input_serial == "REBOOT") ESP.restart();
        else if(input_serial == "RESET") {deleteEEPROM("ALL"); ESP.restart();}
        else if(input_serial == "QOFFSET") {mc.set_qoffset(q_array);}
        else if(input_serial == "QINIT") {mc.initial_quat_offset_enable = true;}  //本来は起動時して姿勢が変わる前に実行
      }else if(mc.mode!=MODE_MOTION_MASSAGE){
        if(input_serial == "FLUSH") flushPKvector(pk_references,"MOTION_CONT");
        else if(input_serial == "READ")readPKvector("MOTION_CONT");
        else if(input_serial == "DELETE") deleteEEPROM("MOTION_CONT");        
        else if(input_serial == "SHOW") serialprint_pk_references("");
        else if(input_serial == "SHOW_REFERENCES") serialprint_pk_references("csv");
        else if(input_serial == "REBOOT") ESP.restart();
        else if(input_serial == "RESET") {deleteEEPROM("ALL"); ESP.restart();}
        else if(input_serial == "QOFFSET") {mc.set_qoffset(q_array);}
        else if(input_serial == "QINIT") {mc.initial_quat_offset_enable = true;} //本来は起動時して姿勢が変わる前に実行
      }


      /*
      if(input_serial == "FLUSH") flushPKvector(pk_vector,"MOTION_CONT");
      else if(input_serial == String("READ")){
        Serial.println("READEEPROOO");
        readPKvector("MOTION_CONT");
      }
      else if(input_serial == "DELETE") deleteEEPROM("MOTION_CONT");
      */

    }
    #ifdef BTSerial    
    if(bts.available()){
      M5.Lcd.setCursor(10, 0);
      //M5.Lcd.println(bts.read());　//Byte受信
      
      M5.Lcd.println(bts.readStringUntil('/0'));
    }
    #endif
    // idle
    int32_t sleep = TASK_SLEEP_READ_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void ButtonLoop(void* arg) {
  uint8_t btnFlag = 0;
  while (1) {
    uint32_t entryTime = millis();

    M5.update();
    if(M5.BtnA.wasReleasefor(2000)){
      yO2 = yaw; //yaw軸の手動補正
    }
    else if(M5.BtnA.wasReleasefor(1000)){
      if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
        calibrateMPU6886();
        xSemaphoreGive(imuDataMutex);
      }
    }
    else if(M5.BtnA.wasReleasefor(500)){
      mc.changeSerialONOFF(false);
    }
    else if(M5.BtnA.wasReleasefor(10)){
      mc.changeGameMode(false);
    }

    if (M5.BtnB.wasReleasefor(3000)){
      //event = REGISTRATION_ROLL_EVENT;
      mc.reConnect();
    }
    else if (M5.BtnB.wasReleasefor(2000)){
      //event = REGISTRATION_ROLL_EVENT;
    }    
    else if (M5.BtnB.wasReleasefor(1000)){
      event = REGISTRATION_ROLL_PITCH_EVENT;

      digitalWrite(10, HIGH);
      vTaskDelay(100);
      digitalWrite(10, LOW);
      vTaskDelay(100);
      digitalWrite(10, HIGH); 
    }
    else if(M5.BtnB.wasReleasefor(10)){
      task_sleep_hid_session_plus += 5;
      task_sleep_hid_session_plus = task_sleep_hid_session_plus % 50;
      //BLEHID_ENABLE = !BLEHID_ENABLE;
      //event = INPUT_EVENT;
    }
    else{          
    }

    uint8_t axpButton = M5.Axp.GetBtnPress();
    //電源ボタンを1秒以上押した場合
    if(axpButton==1){
      mc.mouse_scale2 += 1;
      Serial.println(mc.mouse_scale2);
      if(mc.mouse_scale2 > 5) mc.mouse_scale2 = 1;
    }
    //電源ボタンを1秒未満押してから離した場合
    if(axpButton==2){
      mc.changeMCMode(false);
      if(mc.mode==MODE_STREET_FIGHTER_DIST){
        Serial.println("SF mode now");
        bleCombo.setDelay(15);
      }else if(mc.mode==MODE_MOTION_CONTROLLER){
        readPKvector("MOTION_CONT");
        Serial.println("Init Unity Roll and Pitch,");
        //vTaskDelay(300);
        serialprint_pk_references("csv");
        bleCombo.setDelay(7);
      }else if(mc.mode==MODE_MOTION_MASSAGE){
        readMessageVector("MESSAGE");
        Serial.println("Init Unity Roll and Pitch,");
        //vTaskDelay(300);
        serialprint_motion_messages("csv");
        bleCombo.setDelay(7);
      }
    }

    //===ボタン判定===
    for(uint8_t i=0; i<sizeof(events_bool); i++){
      //ボタン押下判定
      if(mc.prev_button_state[i] == false && mc.switchRead(mc.button[i])==true){
          mc.prev_button_state[i] = true;
          events_bool[i] = true;
          //Serial.println("..");
        }
      //ボタン離す判定
      else if(mc.prev_button_state[i] == true && mc.switchRead(mc.button[i])==false){
          mc.prev_button_state[i] = false;
          events_bool[i] = false;
          //Serial.println("!");
      }
    }
    


/*
    if(M5.Axp.GetBtnPress()==2)  //First, read num of element
  int n =0;
  uint8_t num_element;
  EEPROM.get(n,num_element);
  n = sizeof(uint8_t);

  //{ //Instanious Click   // Unexpected "emptyRxFifo(): RxEmpty(2) call on TxBuffer? dq=0" outputs.
      num_powerbtn_click +=1;
    }
*/
    // idle
    int32_t sleep = TASK_SLEEP_BUTTON - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}



void flushPKvector(std::vector<pk> pk_vector,String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="MOTION_CONT"){ n = EEPROM_MOTIONCONT_START; size = EEPROM_MOTIONCONT_SIZE;}
  //else if(preset_name=="PRES1"){ n = EEPROM_PRES1_START; size = EEPROM_PRES1_SIZE;}
  //else if(preset_name=="PRES2"){ n = EEPROM_PRES2_START; size = EEPROM_PRES2_SIZE;}

  //==First, write num of element
  uint8_t num_element = pk_vector.size();
  EEPROM.put(n,num_element);
  Serial.println("write:index_byte"+String(n)+",num_element"+String(num_element));
  //n += sizeof(uint8_t) * 2;
  n += sizeof(uint8_t);

  //==Second, write pk array;
  for (int i=0 ; i < pk_vector.size(); i++){
    if(EEPROM_SIZE< (n + sizeof(pk))){
      if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over, stopped flush.");
    }
    else{
      EEPROM.put(n,pk_vector[i]);
      if(DEBUG_EEPROM)Serial.println("write:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_priority));
      n += sizeof(pk);
    }
  }
  EEPROM.commit();
}

void flushMessageVector(std::vector<message> message_vector,String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="MESSAGE"){
    n = EEPROM_MESSAGE_SPACE_START; size = EEPROM_MESSAGE_SPACE_SIZE;  
  }else{
  }

  //==First, write num of element
  uint8_t num_element = message_vector.size();
  EEPROM.put(n,num_element);
  Serial.println("write:index_byte"+String(n)+",num_element"+String(num_element));
  //n += sizeof(uint8_t) * 2;
  n += sizeof(uint8_t);

  //==Second, write pk array;
  for (int i=0 ; i < message_vector.size(); i++){
    if(EEPROM_SIZE< (n + sizeof(message))){
      if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over, stopped flush.");
    }
    else{
      EEPROM.put(n,message_vector[i]);
      if(DEBUG_EEPROM) Serial.println("write:"+String(message_vector[i].content));
      n += sizeof(message);
    }
  }
  EEPROM.commit();
}


/*void readPKvector(String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="MOTION_CONT") n = EEPROM_MOTIONCONT_START; //size = EEPROM_MOTIONCONT_SIZE;
  else if(preset_name=="PRES1") n = EEPROM_PRES1_START; //size = EEPROM_PRES1_SIZE;
  else if(preset_name=="PRES2") n = EEPROM_PRES2_START; //size = EEPROM_PRES2_SIZE;
  else if(preset_name=="ACTION") {pk_vector = pk_vector_preset_Action; return;}

  //==First, read num of element1
  // 要素数の読み出し
  uint8_t num_element;
  EEPROM.get(n, num_element);
  Serial.println("read:index_byte" + String(n) + ",num_element" + String(num_element));
  n += sizeof(uint8_t);
  
  //==Second, read pk array.
  pk pk;
  pk_vector = {};
  for (int i=0; i < num_element; i++){
    EEPROM.get(n,pk);
    pk_vector.push_back(pk);
    //if(pk_vector[i].rpy_priority=='0')break;
    if(DEBUG_EEPROM)Serial.println("read:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_priority));
    n += sizeof(pk);
  }
}*/

void readPKvector(String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="MOTION_CONT"){ n = EEPROM_MOTIONCONT_START; size = EEPROM_MOTIONCONT_SIZE;}
  //else if(preset_name=="MESSAGE"){ n = EEPROM_MESSAGE_SPACE_START; size = EEPROM_MESSAGE_SPACE_SIZE;}
  /*
  else if(preset_name=="STREET_FIGHTER"){ 
    pk_references.push_back(pk5);
    pk_references.push_back(pk7);
  }*/

  // 要素数の読み出し
  uint8_t num_element;
  EEPROM.get(n, num_element);
  Serial.println("read:index_byte" + String(n) + ",num_element" + String(num_element));
  n += sizeof(uint8_t);
  
  // pk構造体の読み出し
  pk pk;
  pk_references = {};
  for (int i = 0; i < num_element; i++){
    if(EEPROM_SIZE< (n + sizeof(pk))){
      if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over, stopped read.");
    }else{
      EEPROM.get(n, pk);
      pk_references.push_back(pk);
      if(DEBUG_EEPROM)Serial.println("read:" + String(n) + "," + String(pk_references[i].hid_input) + "," + String(pk_references[i].rpy_priority));
      n += sizeof(pk);
    }
  }
}


void readMessageVector(String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="MESSAGE"){ n = EEPROM_MESSAGE_SPACE_START; size = EEPROM_MESSAGE_SPACE_SIZE;}

  // 要素数の読み出し
  uint8_t num_element;
  EEPROM.get(n, num_element);
  Serial.println("read:index_byte" + String(n) + ",num_element" + String(num_element));
  n += sizeof(uint8_t);
  
  // 構造体の読み出し
  message message;
  message_vector = {};
  for (int i = 0; i < num_element; i++){
    if(EEPROM_SIZE< (n + sizeof(message))){
      if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over, stopped read.");
    }else{
      EEPROM.get(n, message);
      message_vector.push_back(message);
      if(DEBUG_EEPROM)Serial.println("index:" + String(n)+",read:"+String(message.content));
      n += sizeof(message);
    }
  }
}

void deleteEEPROM(String preset_name){
  EEPROM.put(0,0);// Set num_element.
  //Serial.println(String(EEPROM.length())); 0
  //Serial.println(String(EEPROM.length()/sizeof(pk))); 0

  uint16_t size=0;
  uint16_t n;

  if(preset_name=="MOTION_CONT") {n = EEPROM_MOTIONCONT_START; size = EEPROM_MOTIONCONT_SIZE;}
  else if(preset_name=="MESSAGE"){ n = EEPROM_MESSAGE_SPACE_START; size = EEPROM_MESSAGE_SPACE_SIZE;}
  else if(preset_name=="ALL"){ n = 0; size = EEPROM_MAX_SIZE;}

  //要素数を0へ。
  EEPROM.put(n,0);
  n = n + sizeof(uint8_t);

  if(preset_name=="MESSAGE"){
    message null_message;
    for (int i=0 ; i < size/sizeof(message) ; i++){
      EEPROM.put(n,null_message);
      n += sizeof(message);
    }
    EEPROM.commit();
    message_vector = {};
  }else if(preset_name=="ALL"){
    //FLASHをすべて削除
    for (int i=0 ; i < size ; i++){
      EEPROM.put(n,0);
      n += 1;
    }
    EEPROM.commit();
  }else if(preset_name=="MOTION_CONT"){
    //pk pk_v = {'0',0,0,'0'};
    pk pk_v = {99,0,'r',1,2,3,4,{0,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3};
    for (int i=0 ; i < size/sizeof(pk) ; i++){
      EEPROM.put(n,pk_v);
      n += sizeof(pk);
    }
    EEPROM.commit();
    pk_references = {};
  }
}


/* == Flush and Read by array == 
void flushPKarray(pk *a){
  int n = 0;
  for (int i=0 ; i < SIZE_PK_array; i++){
    EEPROM.put(n,a[i]);
    n += sizeof(pk);
    if(a[i].rpy_priority=='0')break;
  }
  EEPROM.commit();
}

void readPKarray(){
  int n = 0;
  for (int i=0; i <  SIZE_PK_array; i++){
    EEPROM.get(n,pk_array2[i]);
    n += sizeof(pk);
    if(pk_array2[i].rpy_priority=='0')break;
    if(DEBUG_EEPROM)Serial.println("read:"+String(sizeof(pk))+","+String(pk_array2[i].hid_input) +""+String(pk_array2[i].rpy_priority));
  }
}
*/


//void serialprint_pk_references(std::vector<pk>& pk_vectol,String option){
void serialprint_pk_references(String option){

  for (int i=0 ; i < pk_references.size(); i++){
    //pk pk5 = {MODE_STREET_FIGHTER_RANGE,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
    if(option==""){
      if(DEBUG_EEPROM){    //Serial.println(String(pk_references[i].hid_input) +","+String(pk_references[i].roll_min) + "," + String(pk_references[i].roll_max)+","+ String(pk_references[i].rpy_priority));
        Serial.printf("Mode:%u, index: %u, RPY Priority: %c, Roll Min: %d, Roll Max: %d, Pitch Min: %d, Pitch Max: %d, Roll:%d, Pitch:%d, Yaw:%d, q0:%f, q1:%f, q2:%f, q3:%f, HID Input: %c, HID Inputs: %c%c%c%c%c%c%c%c, HID Input Interval:%c, HID Input Acc Threshold: %f \r\n",
          pk_references[i].mode,
          pk_references[i].index,
          pk_references[i].rpy_priority,
          pk_references[i].roll_min,
          pk_references[i].roll_max,
          pk_references[i].pitch_min,
          pk_references[i].pitch_max,
          pk_references[i].rpy[0], pk_references[i].rpy[1], pk_references[i].rpy[2],
          pk_references[i].quatanion[0], pk_references[i].quatanion[1], pk_references[i].quatanion[2], pk_references[i].quatanion[3],
          pk_references[i].hid_input,
          pk_references[i].hid_inputs[0], pk_references[i].hid_inputs[1], pk_references[i].hid_inputs[2], pk_references[i].hid_inputs[3],
          pk_references[i].hid_inputs[4], pk_references[i].hid_inputs[5], pk_references[i].hid_inputs[6], pk_references[i].hid_inputs[7],
          pk_references[i].hid_input_interval,
          pk_references[i].hid_input_acc_threshold);
        }
    }else if(option=="csv"){
          Serial.printf("pk_references,%u,%u,%c,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%c,%c%c%c%c%c%c%c%c,%c,%f \r\n",
          pk_references[i].mode,
          pk_references[i].index,
          pk_references[i].rpy_priority,
          pk_references[i].roll_min,
          pk_references[i].roll_max,
          pk_references[i].pitch_min,
          pk_references[i].pitch_max,
          pk_references[i].rpy[0], pk_references[i].rpy[1], pk_references[i].rpy[2],
          pk_references[i].quatanion[0], pk_references[i].quatanion[1], pk_references[i].quatanion[2], pk_references[i].quatanion[3],
          pk_references[i].hid_input,
          pk_references[i].hid_inputs[0], pk_references[i].hid_inputs[1], pk_references[i].hid_inputs[2], pk_references[i].hid_inputs[3],
          pk_references[i].hid_inputs[4], pk_references[i].hid_inputs[5], pk_references[i].hid_inputs[6], pk_references[i].hid_inputs[7],
          pk_references[i].hid_input_interval,
          pk_references[i].hid_input_acc_threshold);
    }
  }
}

void serialprint_motion_messages(String option){
  for (int i=0 ; i < message_vector.size(); i++){
    if(option==""){
      if(DEBUG_EEPROM){    //Serial.println(String(pk_references[i].hid_input) +","+String(pk_references[i].roll_min) + "," + String(pk_references[i].roll_max)+","+ String(pk_references[i].rpy_priority));
        Serial.printf("Message: %s, %d, %d, %d \r\n",
          message_vector[i].content,
          message_vector[i].rpy[0], message_vector[i].rpy[1], message_vector[i].rpy[2]);
        }
    }else if(option=="csv"){
          Serial.printf("MOTION_MESSAGE,%s,%d,%d,%d \r\n",
          message_vector[i].content,
          message_vector[i].rpy[0], message_vector[i].rpy[1], message_vector[i].rpy[2]);
    }
  }
}



void resetTask(TaskHandle_t *taskHandle) {
    if (eTaskGetState(*taskHandle) == eDeleted) {
        vTaskDelete(*taskHandle);  // タスクを削除
        // タスクを再作成
        xTaskCreatePinnedToCore(ImuLoop, TASK_NAME_IMU, TASK_STACK_DEPTH, NULL, 2, taskHandle, TASK_DEFAULT_CORE_ID);
    }
}

void calibrateMPU6886(){
  float gyroSumX,gyroSumY,gyroSumZ;
  float accSumX,accSumY,accSumZ;
  float calibCount = 500;

  Serial.println("Calibrating...");
  digitalWrite(10, LOW);
  vTaskDelay(1000);
  digitalWrite(10, HIGH);
  vTaskDelay(100);
  digitalWrite(10, LOW); 
  for(int i = 0; i < calibCount; i++){
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);
      gyroSumX += gyroX;
      gyroSumY += gyroY;
      gyroSumZ += gyroZ;
      accSumX += accX;
      accSumY += accY;
      accSumZ += accZ;
      vTaskDelay(10);
      //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", accX, accY, accZ);
  }
  gOX = gyroSumX/calibCount;
  gOY = gyroSumY/calibCount;
  gOZ = gyroSumZ/calibCount;
  aOX = accSumX/calibCount;
  aOY = accSumY/calibCount;
  aOZ = (accSumZ/calibCount) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定
  //aOZ = (accSumZ/calibCount) + 1.0;//重力加速度1G、つまりM5ボタンが下向きで行う想定
  //aOZ = (accSumZ/calibCount);//
  Serial.println("Calibrating...OK");
  Serial.printf("%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  //Serial.printf("%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  /*M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  */
  digitalWrite(10, HIGH);

  //EEPROMへ保存
  uint16_t n = 0; uint16_t size=0; 
  n = EEPROM_CAL_SPACE_START;
  size = EEPROM_CAL_SPACE_SIZE;
  
  //==First, write num of element
  uint8_t num_element = 6; //In case, acc,gyro,mg,quartanion , 13
  EEPROM.put(n,num_element);
  Serial.println("calibration: write:index_byte"+String(n)+",num_element"+String(num_element));
  //n += sizeof(uint8_t) * 2;
  n += sizeof(uint8_t);

  EEPROM.put(n,aOX); n+=sizeof(float);
  EEPROM.put(n,aOY); n+=sizeof(float);
  EEPROM.put(n,aOZ); n+=sizeof(float);
  EEPROM.put(n,gOX); n+=sizeof(float);
  EEPROM.put(n,gOY); n+=sizeof(float);
  EEPROM.put(n,gOZ); n+=sizeof(float);
  if (!EEPROM.commit()) {
      Serial.println("ERROR: calibration commit failed");
  } else {
      Serial.println("Calibration data committed to EEPROM");
  }
  
  resetTask(&taskHandleIMU);
  //==Second, write pk array;
  /*
  for (int i=0 ; i < num_element; i++){
    if(EEPROM_SIZE< (n + sizeof(pk))){
      if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over, stopped flush.");
    }
    else{
      EEPROM.put(n,pk_vector[i]);
      if(DEBUG_EEPROM)Serial.println("write:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_priority));
      n += sizeof(pk);
    }
  }
  */
}



pk getClosestPK(std::vector<pk> pk_refs){
          pk pk_selected;
          float roll_pitch_distance;
          float roll_pitch_distance_prev=999;
          short roll_ref;
          short pitch_ref;


          for(uint8_t i = 0; i < pk_refs.size(); i++){
            roll_ref = pk_refs[i].rpy[0];
            pitch_ref = pk_refs[i].rpy[1];
            //float yaw = pk_refs[i].rpy[2];

            //roll_refをオフセット
            /*
            if (roll_ref < -90) 
            {
                roll_re = 180 + (-1 * (-180 - roll));
            }
            roll = roll - 90; // roll=90°を中心にするためのオフセット
            */
            

            //roll範囲は -180° ~ 180°であり最小roll距離を求めたいので条件分岐する
            float roll_diff = roll - roll_ref;
            if(roll > 90 && roll_ref < -90){
              roll_diff = (180-roll) + (180 + roll_ref); // 1*(150) + (-180 - (-150)) = 30
            }else if(roll < -90 && roll_ref > 90){
              roll_diff =-(-180-roll)+ (180 - roll_ref); //-1*(-150) + (180 - 150) = 30
                                                    //(150,-50) & Ref(-130,-31)?
                                                    //(150,-50) & (100,0) 
                                                    //(170,-20) →(100,0) ×. (本来は(-130,-31))
                                                    //(-170,-20)→(150,-50)〇
            }else{
            }

          
            float pitch_diff = pitch - pitch_ref;
            
            //pk_refsの(roll,pitch)座標と現在の(roll,pitch)座標とのユークリッド距離が最小となるpk_referenceを選択する
            roll_pitch_distance = sqrt(roll_diff*roll_diff + pitch_diff*pitch_diff);
            if(roll_pitch_distance_prev > roll_pitch_distance){
              pk_selected = pk_refs[i];
              roll_pitch_distance_prev = roll_pitch_distance;
            }
          }
          return pk_selected;
}


message getClosestMessage(std::vector<message> message_refs){
          message message_selected;
          float roll_pitch_distance;
          float roll_pitch_distance_prev=999;
          short roll_ref;
          short pitch_ref;


          for(uint8_t i = 0; i < message_refs.size(); i++){
            roll_ref = message_refs[i].rpy[0];
            pitch_ref = message_refs[i].rpy[1];

            //roll範囲は -180° ~ 180°であり最小roll距離を求めたいので条件分岐する
            float roll_diff = roll - roll_ref;
            if(roll > 90 && roll_ref < -90){
              roll_diff = (180-roll) + (180 + roll_ref); // 1*(150) + (-180 - (-150)) = 30
            }else if(roll < -90 && roll_ref > 90){
              roll_diff =-(-180-roll)+ (180 - roll_ref); //-1*(-150) + (180 - 150) = 30
                                                    //(150,-50) & Ref(-130,-31)?
                                                    //(150,-50) & (100,0) 
                                                    //(170,-20) →(100,0) ×. (本来は(-130,-31))
                                                    //(-170,-20)→(150,-50)〇
            }else{
            }

          
            float pitch_diff = pitch - pitch_ref;
            
            //pk_refsの(roll,pitch)座標と現在の(roll,pitch)座標とのユークリッド距離が最小となるpk_referenceを選択する
            roll_pitch_distance = sqrt(roll_diff*roll_diff + pitch_diff*pitch_diff);
            if(roll_pitch_distance_prev > roll_pitch_distance){
              message_selected = message_refs[i];
              roll_pitch_distance_prev = roll_pitch_distance;
            }
          }
          return message_selected;
}



void serialOutput(pk &pk,String msg_type){
  if(msg_type=="SELECTED_PK"){
    Serial.printf("selected_pk,%d,%d,%d,%c,%s,current_rp,%f,%f \r\n",
    pk.rpy[0],
    pk.rpy[1],
    pk.rpy[2],
    pk.hid_input,
    pk.hid_inputs,

    roll,
    pitch);
  }
  //else if(msg_type=="OUTPUT_PK_REFERENCES"){ 
  //}
}


void unityAppInit(){
    Serial.println("UnityAppInit");
    mc.serialON();
    if(mc.mode == MODE_MOTION_CONTROLLER){
      readPKvector("MOTION_CONT");
      Serial.println("Init Unity Roll and Pitch,");
      serialprint_pk_references("csv");
    }else if(mc.mode == MODE_MOTION_MASSAGE){
      readMessageVector("MESSAGE");
      Serial.println("Init Unity Roll and Pitch,");
      serialprint_motion_messages("csv");
    }
}

void remove_crlf(std::string& s)
{
    size_t i, j;
    for( i = 0, j = 0; i < s.size(); i++ ){
        if( s[i] != '\r' && s[i] != '\n' ){
            s[j] = s[i];
            j++;
        }
    }
    s.resize(j);
}


// Thanks to https://algorithm.joho.info/arduino/string-split-delimiter/
int split(String data, char delimiter, String *dst){
    //文字列配列の初期化
    for (int j=0; j< sizeof(dst); j++){
      //Serial.println(dst(j));
      dst[j] = {""}; 
    }
    //msgs[3] = {"\0"};
  
    int index = 0;
    int arraySize = (sizeof(data)/sizeof((data)[0]));  
    int datalength = data.length();
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter ) {
            index++;
            if ( index > (arraySize - 1)) return -1;
        }
        else dst[index] += tmp; //区切り文字が来るまで1Byteづつ連結
        //Serial.print("dbg: "+dst[index]);
    }
    return (index + 1);
}
