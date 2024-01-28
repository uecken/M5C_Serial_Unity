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
  const uint8_t Fs = 75;
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
#define TASK_SLEEP_HID_SESSION 33 // = 1000[ms] / 100[Hz]
#define TASK_SLEEP_WRITE_SESSION 100 // = 1000[ms] / 100[Hz]
#define TASK_SLEEP_READ_SESSION 200 // = 1000[ms] / 10[Hz]
#define TASK_SLEEP_BUTTON 50 // = 1000[ms] / 20[Hz]
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
float* q_array = new float[4]; //quatanion
float aOX = -0.003, aOY = +0.018, aOZ =  0.085 ;  //-0.00   0.01   0.07 
float gOX = 3.516, gOY = -6.023 , gOZ = -5.14;  //3.36   9.66   4.11
float pO=0 , rO=0 , yO=-8.5, yO2=0;
float roll_prev,pitch_prev,yaw_prev;


boolean DEBUG_EEPROM = true;
boolean DEBUG_HID = true;
boolean BLEHID_ENABLE = true;


//===========Player Key settings===========
struct pk{ //44byte (padding 2Byte?)
  //mode設定
  uint8_t mode;

  //Priority設定
  uint8_t skill_priority; //1~255 1:high , 255:low
  char rpy_priority;  //rpy 1byte

  //モーション始端
  short roll_min;    // 2byte
  short roll_max;    // 2byte
  short pitch_min;    // 2byte
  short pitch_max;    // 2byte

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
//pk pk1 = {1,'r',30,45,30,45,'1',{'1','2','3','4','\0'},50,3};
pk pk5 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
pk pk6 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_LEFT_ARROW,'a','\0'},50,3}; //波動拳
pk pk7 = {MODE_STREET_FIGHTER,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk pk8 = {MODE_STREET_FIGHTER,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{KEY_LEFT_ARROW,KEY_DOWN_ARROW,KEY_LEFT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
//pk pk9 = {5,'r',150,210,-30,30,'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
//pk pk10 = {5,'r',150,210,-30,30,'1',{KEY_RIGHT_ARROW,KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)

pk p2_pk5 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3}; //波動拳
pk p2_pk6 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_LEFT_ARROW,'y','\0'},50,3}; //波動拳
pk p2_pk7 = {MODE_STREET_FIGHTER,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{P2_KEY_RIGHT_ARROW,P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)
pk p2_pk8 = {MODE_STREET_FIGHTER,5,'r',150,210,-30,30,{180,0,0},{0,0,0,0},'1',{P2_KEY_LEFT_ARROW,P2_KEY_DOWN_ARROW,P2_KEY_LEFT_ARROW,'y','\0'},50,3}; //昇竜拳(roll判定は150-180,-150-180にするべき)


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

std::vector<pk> pk_vector_sf_left{pk5,pk7};
std::vector<pk> pk_vector_sf_right{pk6,pk8};

std::vector<pk> p2_pk_vector_sf_left{p2_pk5,p2_pk7};
std::vector<pk> p2_pk_vector_sf_right{p2_pk6,p2_pk8};

std::vector<pk> pk_references;


//std::vector<pk> pk_vector_preset1{pk1,pk2,pk3,pk4,pk5};
//std::vector<pk> pk_vector_preset_FEZ{pk5};
std::vector<pk> pk_vector_preset_Action{pk5};


//std::vector<pk> pk_vector;
//void flushPKarray(pk*);
//void readPKarray();
const uint16_t EEPROM_MAX_SIZE=4095;
const uint16_t EEPROM_SIZE=1200;
const uint16_t EEPROM_SF_START = 0;
const uint16_t EEPROM_SF_SIZE = 400;
const uint16_t EEPROM_PRES1_START = 400;
const uint16_t EEPROM_PRES1_SIZE = 400;
const uint16_t EEPROM_PRES2_START = 800;
const uint16_t EEPROM_PRES2_SIZE = 400;
const uint16_t EEPROM_CAL_SPACE_START = 1200;
const uint16_t EEPROM_CAL_SPACE_SIZE = 30;
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

char input_serial_char=NULL;
uint8_t num_powerbtn_click=1;
boolean serial_ON = false;

//unsigned long Tcur,Tpre;

void setup() {


  #ifdef BTSerial
   bts.begin("M5C_BTSerial");
  #elif defined(MotionCont)
   mc.mode=MODE_MOUSE;
   mc.begin();
  #endif

  serWriteMutex = xSemaphoreCreateMutex();
  imuDataMutex = xSemaphoreCreateMutex();
  btnDataMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ImuLoop, TASK_NAME_IMU, TASK_STACK_DEPTH, NULL, 2, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(WriteSessionLoop, TASK_NAME_WRITE_SESSION, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ReadSessionLoop, TASK_NAME_READ_SESSION, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ButtonLoop, TASK_NAME_BUTTON, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(hidSessionLoop, TASK_NAME_HID, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);


  //calibrate
  /*if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
    //calibrateMPU6886();
    xSemaphoreGive(imuDataMutex);
  }*/

  //EEPROMから読んだCalibration値を格納
  uint16_t n=EEPROM_CAL_SPACE_START;
  EEPROM.get(n,aOX); n+=sizeof(aOX);
  EEPROM.get(n,aOY); n+=sizeof(aOY);
  EEPROM.get(n,aOZ); n+=sizeof(aOZ);
  EEPROM.get(n,gOX); n+=sizeof(gOX);
  EEPROM.get(n,gOY); n+=sizeof(gOY);
  EEPROM.get(n,gOZ); n+=sizeof(gOZ);
  

    

  if(EEPROM_SIZE>EEPROM_MAX_SIZE){
    if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over 4095(EEPROM_MAX_SIZE)");
  }
  else{
    EEPROM.begin(EEPROM_SIZE); //EEPROM.begin(size)で確保できる最大サイズは通常、最大4095バイト（4KB未満）
    readPKvector("SF");
  }

  if(LEFT_DISPLAY){
    Serial.println("LEFT_DISPLAY,");
  }
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
      q_array = M5.IMU.getAhrsData(&pitch,&roll,&yaw,aOX,aOY,aOZ,gOX,gOY,gOZ);
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
              if(serial_ON)Serial.println(String("In,")+String(pk_vector[i].hid_input) + ","+String(pk_vector[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              M5.Lcd.setCursor(10,60);
              M5.Lcd.println(String("In,")+String(pk_vector[i].hid_input) + ","+String(pk_vector[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              mc.execSF_HIDInputs(pk_vector[i].hid_inputs,(float)pk_vector[i].hid_input_acc_threshold,&acc_buffer);
              break;
            }
          }
        }
      }
      else if(mc.mode==MODE_ROLL_PITCH_VALIDATION){
        pk pk_selected;
        float roll_pitch_distance;
        float roll_pitch_distance_prev=999;
        if(events_bool[0]==true){
          //if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_left;}
          //else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_left;}

          for(uint8_t i = 0; i < pk_references.size(); i++){

            short roll_ref = pk_references[i].rpy[0];
            short pitch_ref = pk_references[i].rpy[1];
            //float yaw = pk_references[i].rpy[2];

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
            
            //pk_referencesの(roll,pitch)座標と現在の(roll,pitch)座標とのユークリッド距離が最小となるpk_referenceを選択する
            roll_pitch_distance = sqrt(roll_diff*roll_diff + pitch_diff*pitch_diff);
            if(roll_pitch_distance_prev > roll_pitch_distance){
              pk_selected = pk_references[i];
              roll_pitch_distance_prev = roll_pitch_distance;
            }
          }
          if(BLEHID_ENABLE)mc.inputKey(pk_selected.hid_input);
          if(DEBUG_HID)Serial.printf("selected_pk:%c,%d,%d, current_rp:%f,%f \r\n",pk_selected.hid_input,pk_selected.rpy[0],pk_selected.rpy[1],roll,pitch);
          if(DEBUG_HID)Serial.printf("selected_pk,%c,%d,%d \r\n",pk_selected.hid_input,pk_selected.rpy[0],pk_selected.rpy[1]);
          roll_pitch_distance_prev=999;
          events_bool[0]=false;
        }
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
              Mouse.move(0,0,1);
            }else if(pitch_delta > 0){
              Mouse.move(0,0,-1);
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
            //Mouse.press(MOUSE_LEFT);
            Mouse.click(MOUSE_LEFT);
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
            Mouse.click(MOUSE_RIGHT);
        }
      }
      else if(mc.mode==MODE_STREET_FIGHTER){
        std::vector<pk> pks;
        if(events_bool[0]==true){
          if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_left;}
          else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_left;}

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
              if(serial_ON)Serial.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              M5.Lcd.setCursor(10,60);
              M5.Lcd.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
              mc.execSF_HIDInputs(pks[i].hid_inputs,(float)pks[i].hid_input_acc_threshold,&acc_buffer);
              break;
            }
          }
          events_bool[0]=false;
        }
        else if(events_bool[1]==true){
          if(mc.current_game_mode==MODE_SF_P1){ pks = pk_vector_sf_right;}
          else if(mc.current_game_mode==MODE_SF_P2) { pks = p2_pk_vector_sf_right;}

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
              if(serial_ON)Serial.println(String("In,")+String(pks[i].hid_input) + ","+String(pks[i].rpy_priority) + "," + String(roll)); // NG "In,"+pk1.input. Buffer Over
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

/*
        if(event==REGISTRATION_ROLL_PITCH_EVENT){
          //pk pk5 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
          char input_key;
          if(input_serial_char!=NULL){
            input_key = input_serial_char;
          }else{
            Serial.println("input_key is null");
            input_key = '!';
          }

          //単一文字の場合
          pk pk_reg = {MODE_STREET_FIGHTER,6,'r',30,60,-30,30,{roll,pitch,yaw},{q_array[0],q_array[1],q_array[2],q_array[3]},input_key,{input_key,'\0'},50,0};

          //連続文字の場合
            //後日実装

          pk_references.push_back(pk_reg);
          input_serial_char = NULL;
        }
      }
*/

      
      mc.keyboardReleaseAll();        
      event = NO_EVENT;
    //}
    //xSemaphoreGive(serWriteMutex);

    // idle
    int32_t sleep = TASK_SLEEP_HID_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void WriteSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();

    //if (xSemaphoreTake(serWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 lin serial Input
      if(event==NO_EVENT){
        if(left_axis_trans){//左手系
          //if(serial_ON)Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          if(serial_ON)Serial.printf("sensor_data,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          #ifdef BTSerial 
            //if(serial_ON)bts.printf("%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
            if(serial_ON)bts.printf("%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, yaw-yO2,q_array[0],q_array[1],q_array[2],q_array[3]);

          #endif
        }else{//右手系
          Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, yaw-yO2);
        }
        M5.Lcd.setCursor(30, 60);
        M5.Lcd.println(String(pitch) + " " + String(roll) + " " + String(yaw-yO2));
        M5.Lcd.println(String(pitch) + " " + String(unwrpRoll) + " " + String(unwrpYaw-yO2));   
        }

        //if(serial_ON)Serial.println(acc_buffer.getMaxAbsAccel(5));
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
      M5.Lcd.setCursor(10, 110);
      
      String input_serial = Serial.readStringUntil('/0');
      input_serial.trim(); //remove (" ","\t","\v","\f","\r","\n")
      input_serial_char = input_serial[0];
      Serial.println(input_serial);
      M5.Lcd.println(input_serial);

      int num_length ;
      if(input_serial.indexOf(',')>0){
        num_length = split(input_serial,',',inputs);
        String command = String(inputs[0]);
        String value = String(inputs[1]);
        Serial.println("split:" + command + "," + value);
        if(command == "FLUSH") flushPKvector(pk_references,value);
        else if(command == "READ") readPKvector(value);
        else if(command == "DELETE") deleteEEPROM(value);
      }else{
        if(input_serial == "FLUSH") flushPKvector(pk_references,"SF");
        else if(input_serial == "READ")readPKvector("SF");
        else if(input_serial == "DELETE") deleteEEPROM("SF");        
        else if(input_serial == "SHOW") serialprint_pk_references("");
        else if(input_serial == "SHOW_REFERENCES") serialprint_pk_references("csv");
      }


      /*
      if(input_serial == "FLUSH") flushPKvector(pk_vector,"SF");
      else if(input_serial == String("READ")){
        Serial.println("READEEPROOO");
        readPKvector("SF");
      }
      else if(input_serial == "DELETE") deleteEEPROM("SF");
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
      if(serial_ON) serial_ON = false;
      else if(!serial_ON) serial_ON = true;
    }
    else if(M5.BtnA.wasReleasefor(10)){
      mc.changeGameMode();
    }

    if (M5.BtnB.wasReleasefor(2000)){
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
      BLEHID_ENABLE = !BLEHID_ENABLE;
      //event = INPUT_EVENT;
    }
    else{          
    }

    uint8_t axpButton = M5.Axp.GetBtnPress();
    //電源ボタンを1秒以上押した場合
    if(axpButton==1){
      if(mc.mode==MODE_MOUSE){
        mc.mode = MODE_ROLL_PITCH_VALIDATION;
        if(DEBUG_HID) Serial.println("REGISTRATION_ROLL_PITCH_EVENT");
      }else if(mc.mode==MODE_ROLL_PITCH_VALIDATION){
        mc.mode = MODE_MOUSE;
        if(DEBUG_HID) Serial.println("MODE_MOUSE");
      }
    }
    //電源ボタンを1秒未満押してから離した場合
    if(axpButton==2){
      mc.mouse_scale2 += 1;
      Serial.println(mc.mouse_scale2);
      if(mc.mouse_scale2 > 5) mc.mouse_scale2 = 1;
    }

    //ボタン判定
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
  if(preset_name=="SF"){ n = EEPROM_SF_START; size = EEPROM_SF_SIZE;}
  else if(preset_name=="PRES1"){ n = EEPROM_PRES1_START; size = EEPROM_PRES1_SIZE;}
  else if(preset_name=="PRES2"){ n = EEPROM_PRES2_START; size = EEPROM_PRES2_SIZE;}

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


/*void readPKvector(String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="SF") n = EEPROM_SF_START; //size = EEPROM_SF_SIZE;
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
  if(preset_name=="SF"){ n = EEPROM_SF_START; size = EEPROM_SF_SIZE;}
  else if(preset_name=="PRES1"){ n = EEPROM_PRES1_START; size = EEPROM_PRES1_SIZE;}
  else if(preset_name=="PRES2"){ n = EEPROM_PRES2_START; size = EEPROM_PRES2_SIZE;}

  // 要素数の読み出し
  uint8_t num_element;
  EEPROM.get(n, num_element);
  Serial.println("read:index_byte" + String(n) + ",num_element" + String(num_element));
  n += sizeof(uint8_t);
  
  // pk構造体の読み出し
  pk pk;
  for (int i = 0; i < num_element; i++){
    if(EEPROM_SIZE< (n + sizeof(pk))){
      if(DEBUG_EEPROM)Serial.println("EEPROM_SIZE is over, stopped read.");
    }    else{
      EEPROM.get(n, pk);
      pk_references.push_back(pk);
      if(DEBUG_EEPROM)Serial.println("read:" + String(n) + "," + String(pk_references[i].hid_input) + "," + String(pk_references[i].rpy_priority));
      n += sizeof(pk);
    }
  }
}

void deleteEEPROM(String preset_name){
  EEPROM.put(0,0);// Set num_element.
  int n = sizeof(uint8_t);

  //Serial.println(String(EEPROM.length())); 0
  //Serial.println(String(EEPROM.length()/sizeof(pk))); 0

  uint16_t size=0; 
  if(preset_name=="SF") {n = n + EEPROM_SF_START; size = EEPROM_SF_SIZE;}
  else if(preset_name=="PRES1"){ n = n + EEPROM_PRES1_START; size = EEPROM_PRES1_SIZE;}
  else if(preset_name=="PRES2"){ n = n + EEPROM_PRES2_START; size = EEPROM_PRES2_SIZE;}

  //pk pk_v = {'0',0,0,'0'};
  pk pk_v = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{P2_KEY_DOWN_ARROW,P2_KEY_RIGHT_ARROW,'y','\0'},50,3};
  for (int i=0 ; i < size/sizeof(pk) ; i++){
    EEPROM.put(n,pk_v);
    n += sizeof(pk);
  }
  EEPROM.commit();
  pk_references = {};
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
  /*
  if(option==""){
    //skip
  }else if(option=="csv"){
    Serial.print("pk_referencess,");
  }
  */

  for (int i=0 ; i < pk_references.size(); i++){
    //pk pk5 = {MODE_STREET_FIGHTER,5,'r',-30,30,-30,30,{0,0,0},{0,0,0,0},'1',{KEY_DOWN_ARROW,KEY_RIGHT_ARROW,'a','\0'},50,3}; //波動拳
    if(option==""){
      if(DEBUG_EEPROM){    //Serial.println(String(pk_references[i].hid_input) +","+String(pk_references[i].roll_min) + "," + String(pk_references[i].roll_max)+","+ String(pk_references[i].rpy_priority));
        Serial.printf("Mode:%u, Skill Priority: %u, RPY Priority: %c, Roll Min: %d, Roll Max: %d, Pitch Min: %d, Pitch Max: %d, Roll:%d, Pitch:%d, Yaw:%d, q0:%f, q1:%f, q2:%f, q3:%f, HID Input: %c, HID Inputs: %c%c%c%c%c%c%c%c, HID Input Interval:%c, HID Input Acc Threshold: %f \r\n",
          pk_references[i].mode,
          pk_references[i].skill_priority,
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
          pk_references[i].skill_priority,
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
  EEPROM.commit();

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

//void AllSerialOut()

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
