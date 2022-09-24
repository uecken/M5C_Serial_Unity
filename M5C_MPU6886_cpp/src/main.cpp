#include <M5StickCPlus.h>
#include <vector>
#include <EEPROM.h>
#include "MotionController.h"
MC::MotionController mc;
#include "IMUReader.h"

#ifndef ESP32_BLE_COMBO_H //define by MotionController.h
  #define BTSerial
  #ifdef BTSerial
    #include "BluetoothSerial.h"
    BluetoothSerial bts;
  #endif
#endif

//Refered to AxisOrange https://github.com/naninunenoy/AxisOrange
#define TASK_DEFAULT_CORE_ID 1
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_IMU "IMUTask"
#define TASK_NAME_WRITE_SESSION "WriteSessionTask"
#define TASK_NAME_READ_SESSION "ReadSessionTask"
#define TASK_NAME_BUTTON "ButtonTask"
#define TASK_NAME_HID "HIDTask"
#define TASK_SLEEP_IMU 10 // = 1000[ms] / 100[Hz]
#define TASK_SLEEP_WRITE_SESSION 40 // = 1000[ms] / 25[Hz]
#define TASK_SLEEP_READ_SESSION 100 // = 1000[ms] / 10[Hz]
#define TASK_SLEEP_BUTTON 100 // = 1000[ms] / 10[Hz]
#define MUTEX_DEFAULT_WAIT 10000UL
static void ImuLoop(void* arg);
static void ReadSessionLoop(void* arg);
static void WriteSessionLoop(void* arg);
static void ButtonLoop(void* arg);
static void MCSessionLoop(void* arg);
static SemaphoreHandle_t serWriteMutex = NULL;
static SemaphoreHandle_t imuDataMutex = NULL;
static SemaphoreHandle_t btnDataMutex = NULL;
static SemaphoreHandle_t btComboWriteMutex = NULL;



//======== ReadSession ========
uint8_t event=0;
#define NO_EVENT 0
#define INPUT_EVENT 1
#define REGISTRATION_ROLL_EVENT 2
#define REGISTRATION_PITCH_EVENT 3
#define REGISTRATION_EVENT 4 
#define xxxxx_EVENT 5
int split(String,char,String*);
char input_serial_char=NULL;
boolean serial_ON = false;
//========ReadSession::END========

//======== WeiteSession ========
//======== WeiteSession::END ========


//======== Button ========
uint8_t num_powerbtn_click=1;
//======== Button::END ========


//======== IMU ========
imur::IMUReader IMUr;
const uint8_t Fs = 50;
const long Ts = (1000/Fs);
const float CONVERT_G_TO_MS2 = 9.80665f;
const boolean left_axis_trans = true; //Unityは左手系、M5Stackは右手系

float wrappingYaw(float yaw);
float wrappingRoll(float roll);
void calibrateMPU6886();

float accX = 0.0F,accY = 0.0F,accZ = 0.0F;
float gyroX = 0.0F,gyroY = 0.0F,gyroZ = 0.0F;
float pitch = 0.0F,roll  = 0.0F,yaw   = 0.0F;
float* q_array = new float[4]; //quatanion
float aOX = 0.00, aOY = +0.01, aOZ =  0.07 ;  //-0.00   0.01   0.07 
float gOX = 3.36, gOY = 9.66 , gOZ = 4.11;  //3.36   9.66   4.11
float pO=0 , rO=0 , yO=-8.5, yO2=0;
float unwrapRoll,unwrapYaw;
//========IMU::END ========


//=======MotionController=======
boolean G26skillSwitch = 0;
boolean G26skillSwitchPrevious = 0;
int skillSelectWait = 30; //BLEゲームパッドが繋がっていない場合50ms必要
boolean G36Switch = 0;
boolean G36SwitchPrevious = 0;
boolean skillReset = 0;
boolean G0viewSwitch = 0;
boolean G0viewSwitchPrevious = 0;

struct pk{ //12byte (padding 2Byte)
  char rpy_selected;  //rpy 1byte
  short min_angle;    // 2byte
  short max_angle;    // 2byte
  short roll;    // 2byte
  short pitch;    // 2byte
  char hid_input;   // 1byte
};

// roll -180 ~ +180, pitch -90 ~ +90, yaw -180 ~ +180
pk pk1 = {'r',0,0,0,0,'1'};
pk pk2 = {'r',0,0,45,0,'2'};
pk pk3 = {'r',0,0,90,0,'3'};
pk pk4 = {'r',0,0,135,0,'4'};
pk pk5 = {'r',0,0,180,0,'5'};
pk pk6 = {'p',0,0,90,-70,'6'};
pk pk7 = {'p',0,0,90,-35,'7'};
pk pk8 = {'p',0,0,90,35,'8'};
pk pk9 = {'p',0,0,90,70,'9'};
pk pk10 = {'r',0,0,-45,0,'a'};
pk pk11 = {'r',0,0,-90,0,'c'};
pk pk12 = {'r',0,0,-135,0,'c'};
//pk pk_end = {'0',0,0,'0'}; //Do not use banpei
//pk pk_array[100] = {pk1,pk2,pk3,pk4,pk_end}; //Cannot use vector
//7pk pk_array2[100];
//uint8_t pk_size = sizeof(pk_array)/sizeof(pk_array[0]);
std::vector<pk> pk_vector{pk1,pk2,pk3,pk4,pk5,pk6,pk7,pk8,pk9,pk10,pk11,pk12};
std::vector<pk> pk_vector_preset1{pk1,pk2,pk3,pk4,pk5};
std::vector<pk> pk_vector_preset2{pk1,pk2,pk3,pk4,pk5};
std::vector<pk> pk_vector_preset_FEZ{pk5};
std::vector<pk> pk_vector_preset_Action{pk5};

//std::vector<pk> pk_vector;
//void flushPKarray(pk*);
//void readPKarray();
//=======MotionController::END=======

//=======EEPROM=======
//***** Now, MAX ONE SIZE is 255Byte for using uint8_t ******
boolean DEBUG_EEPROM = true;
const uint16_t EEPROM_SIZE=900; //Max4KByte //1000Byte NG????
const uint16_t EEPROM_ONE_SIZE = 242; //Data(12Byte x 20) + NumElement(2Byte)
const uint16_t EEPROM_TEST_START = 0;
const uint16_t EEPROM_TEST_SIZE = EEPROM_ONE_SIZE;
const uint16_t EEPROM_PRES1_START = 242;
const uint16_t EEPROM_PRES1_SIZE = EEPROM_ONE_SIZE;
const uint16_t EEPROM_PRES2_START = 484;
const uint16_t EEPROM_PRES2_SIZE = EEPROM_ONE_SIZE;
const uint16_t EEPROM_PRES_FEZ_START = 726;
const uint16_t EEPROM_PRES_FEZ_SIZE = EEPROM_ONE_SIZE;
void flushPKvector(std::vector<pk>,String);
void readPKvector(String);
void deleteEEPROM(String);
void remove_crlf(std::string&);
void serialprint_pkvector(std::vector<pk>&);
boolean updatePKvector(pk pk,char);
//==== Cannot Use EEPROMClass. =====
/*==== The error is [E][EEPROM.cpp:199] commit(): error in write ====
  ==== Use Only EEPROM ==== 
EEPROMClass TEST_ER('eeprom1');
EEPROMClass USER1_ER('eeprom2');
*/
//====EEPROM::END====




void setup() {
  M5.begin();  //Init M5StickC Plus.  初始化 M5StickC Plus
  delay(2000);
  M5.Imu.Init();  //Init IMU.  初始化IMU
  M5.Lcd.setRotation(3);  //Rotate the screen. 将屏幕旋转
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(80, 15); //set the cursor location.  设置光标位置
  M5.Lcd.println("IMU TEST");
  M5.Lcd.setCursor(30, 30);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(30, 50);
  M5.Lcd.println("  Pitch   Roll    Yaw");

  #ifdef BTSerial
   bts.begin("M5C_BTSerial");
  #endif

  serWriteMutex = xSemaphoreCreateMutex();
  imuDataMutex = xSemaphoreCreateMutex();
  btnDataMutex = xSemaphoreCreateMutex();
  btComboWriteMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(ImuLoop, TASK_NAME_IMU, TASK_STACK_DEPTH, 
    NULL, 2, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(WriteSessionLoop, TASK_NAME_WRITE_SESSION, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ReadSessionLoop, TASK_NAME_READ_SESSION, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ButtonLoop, TASK_NAME_BUTTON, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(MCSessionLoop, TASK_NAME_HID, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);

  EEPROM.begin(EEPROM_SIZE); //Max4Kbytes

  //pinMode(10,   OUTPUT); //LED
  pinMode(26,   INPUT_PULLUP); //半PD 電圧が1V程にプルダウン？されている。Pin36を ONにするとノイズが発生し、GNDとの判定で誤判定となる。3.3Vとのショート判定する必要がある。
  pinMode(0,   INPUT_PULLUP); //PU 起動モード設定のためか、常にプルアップされており、ソフト制御できない。スイッチでGNDに落として判定する必要あり。
  pinMode(36,   INPUT); //3.3V入力をスイッチとして利用可能
}

void loop() {
  delay(1);
  }


pk motionDecision(float roll, float pitch){
  //Roll90度でPitch -70以下、70以上にした場合はNG。
  //R0llが90度から0度以下に一気に変わってしまう。
  //-70 < pitch < 70 の範囲で判定する。
  boolean pitch_abs70_over = false;
  if (abs(pitch) > 70) {
    Serial.println("Pitch over abs(70)");
    pitch_abs70_over = true;
  }

  float r_distance;
  float p_distance;
  float sum_distance;
  float tmp_sum_distance = 999;
  float r_distance_min;
  float p_distance_min;
  uint8_t i_selected=0;

  uint8_t i=0;
  for(i=0; i < pk_vector.size(); i++){
    r_distance = abs(roll - pk_vector[i].roll);
    p_distance = abs(pitch - pk_vector[i].pitch);

    //例外補正
    if(pitch_abs70_over) r_distance = 0; //Picthが±70~90度でRollがジャンプするため
    else if(roll < -160) { //180度を超えると-180度にジャンプする際に拡大判定するため
      float r_distance1 = abs(abs(roll) - pk_vector[i].roll);
      float r_distance2 = abs(roll - pk_vector[i].roll);
      r_distance = ((r_distance1 > r_distance2)? r_distance2 : r_distance1);
    }
    sum_distance = r_distance + p_distance;

    if(sum_distance < tmp_sum_distance){
      tmp_sum_distance = sum_distance;
      r_distance_min = r_distance;
      p_distance_min = p_distance;
      i_selected = i;
    }
  }
  
  i=i_selected;
  if(DEBUG_EEPROM)Serial.println(String(pk_vector[i].hid_input) +","+String(pk_vector[i].min_angle) + "," + String(pk_vector[i].max_angle)+","+","+String(pk_vector[i].roll) + "," + String(pk_vector[i].pitch) +"," + String(pk_vector[i].rpy_selected)+","+String(int(roll))+","+String(int(pitch))+","+String(uint16_t(r_distance_min))+","+String(uint16_t(p_distance_min)));
  return pk_vector[i_selected];
}

static void ImuLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();
    
    if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
      M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
      M5.IMU.getAccelData(&accX,&accY,&accZ);
      accX -= aOX; accY -= aOY; accZ -= aOZ;
      gyroX -=gOX; gyroY -=gOY; gyroZ -=gOZ;       
      pitch -=pO;  roll -= rO;  yaw -= yO;//★最小値-188.5。原因はgyroZの補正ができていないからか？一時的に+8.5して補正する。
      q_array = M5.IMU.getAhrsData(&pitch,&roll,&yaw,aOX,aOY,aOZ,gOX,gOY,gOZ);
      unwrapRoll = IMUr.wrappingRoll(roll,pitch); 
      unwrapYaw = IMUr.wrappingYaw(yaw);
    }
    xSemaphoreGive(imuDataMutex);
    
    int32_t sleep = TASK_SLEEP_IMU - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void MCSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();
    if (xSemaphoreTake(btComboWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 line serial Input

      if(event==INPUT_EVENT){
        pk pk_selected = motionDecision(roll,pitch); 
        mc.inputKey(pk_selected.hid_input);

      }
      /*
      else if(event==REGISTRATION_ROLL_EVENT){
        char input_key;
        if(input_serial_char!=NULL) input_key = input_serial_char;
        else input_key = num_powerbtn_click;
        
        pk pk_reg = {'r',roll-22.5,roll+22.5,roll,pitch,input_key};
        pk_vector.push_back(pk_reg);

        //M5.Lcd.println(Serial.readStringUntil('/0'));
        for(uint8_t i = 0; i < pk_vector.size(); i++){
          M5.Lcd.setCursor(10,80+i*7);
          M5.Lcd.println(String(pk_vector[i].rpy_selected) +"," +String(pk_vector[i].min_angle) + "," + String(pk_vector[i].max_angle) + "," +String(pk_vector[i].hid_input));
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
      else if(event==REGISTRATION_EVENT){
        char input_key;
        if(input_serial_char!=NULL) input_key = input_serial_char;
        else input_key = num_powerbtn_click;

        pk pk_reg = {'x',0,0,roll,pitch,input_key};
        if(!updatePKvector(pk_reg,input_key)) pk_vector.push_back(pk_reg);
      }
      event = NO_EVENT;
    }
    xSemaphoreGive(btComboWriteMutex);

    // idle
    int32_t sleep = TASK_SLEEP_WRITE_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void WriteSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();

    if (xSemaphoreTake(serWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 lin serial Input
      if(event==NO_EVENT){
        if(left_axis_trans){//左手系
          if(serial_ON)Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          #ifdef BTSerial
            if(bts.available())bts.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          #endif
        }else{//右手系
          Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, yaw-yO2);
        }
        M5.Lcd.setCursor(30, 60);
        M5.Lcd.println(String(pitch) + " " + String(roll) + " " + String(yaw-yO2));    
        }
    }
    xSemaphoreGive(serWriteMutex);

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
        if(command == "FLUSH") flushPKvector(pk_vector,value);
        else if(command == "READ") readPKvector(value);
        else if(command == "DELETE") deleteEEPROM(value);
      }else{
        if(input_serial == "FLUSH") flushPKvector(pk_vector,"TEST");
        else if(input_serial == "READ")readPKvector("TEST");
        else if(input_serial == "DELETE") deleteEEPROM("TEST");        
        else if(input_serial == "SHOW") serialprint_pkvector(pk_vector);
        else if(input_serial == "FLUSH_ALLPRESET"){
          flushPKvector(pk_vector_preset1,"TEST");
          flushPKvector(pk_vector_preset1,"PRES1");
          flushPKvector(pk_vector_preset2,"PRES2");
          flushPKvector(pk_vector_preset_FEZ,"PRES_FEZ");
        }
      }
    }      
    #ifdef BTSerial
    if(bts.available()){
      M5.Lcd.setCursor(10, 0);
      //M5.Lcd.println(bts.read());　//Byte受信      
      M5.Lcd.println(bts.readStringUntil('/0'));
    }
    #endif BTSerial

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
    if(M5.BtnA.wasReleasefor(1000)){
      if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
        calibrateMPU6886();
        xSemaphoreGive(imuDataMutex);
      }
    }
    else if(M5.BtnA.wasReleasefor(500)){
      if(serial_ON) serial_ON = false;
      else if(serial_ON) serial_ON = true;
    }
    else if(M5.BtnA.wasPressed()){
      Serial.println("button pressed"); 
      yO2 = yaw; //yaw軸の手動補正
    }

    /*if (M5.BtnB.wasReleasefor(2000)){
      event = REGISTRATION_ROLL_EVENT;
    }
    else if (M5.BtnB.wasReleasefor(1000)){
      event = REGISTRATION_PITCH_EVENT;
    }*/
    if (M5.BtnB.wasReleasefor(1000)){
      event = REGISTRATION_EVENT;
    }
    else if(M5.BtnB.wasPressed()){
      event = INPUT_EVENT;
    }
    else{          
    }

    G26skillSwitchPrevious = G26skillSwitch;
    G26skillSwitch = !digitalRead(26);
    if(G26skillSwitch)  event = INPUT_EVENT;

    G36SwitchPrevious = G36Switch;
    G36Switch = digitalRead(36);

    G0viewSwitchPrevious = G0viewSwitch;
    G0viewSwitch = !digitalRead(0);

    // idle
    int32_t sleep = TASK_SLEEP_BUTTON - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}


boolean updatePKvector(pk new_pk,char key){
  for (int i=0 ; i < pk_vector.size(); i++){
    if(pk_vector[i].hid_input == key) {
      pk_vector[i] = new_pk;
      Serial.println("update PK");
      return true;
    }
  }
  return false;
}


void flushPKvector(std::vector<pk> pk_vector,String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="TEST") n = EEPROM_TEST_START; //size = EEPROM_TEST_SIZE;
  else if(preset_name=="PRES1") n = EEPROM_PRES1_START; //size = EEPROM_PRES1_SIZE;
  else if(preset_name=="PRES2") n = EEPROM_PRES2_START; //size = EEPROM_PRES2_SIZE;
  else if(preset_name=="PRES_FEZ") n = EEPROM_PRES_FEZ_START; //size = EEPROM_PRES2_SIZE;


  //==First, write num of element
  uint8_t num_element = pk_vector.size();
  EEPROM.put(n,num_element);
  Serial.println("write:index_byte"+String(n)+",num_element"+String(num_element));
  n += sizeof(uint8_t) * 2;

  //==Second, write pk array;
  for (int i=0 ; i < pk_vector.size(); i++){
    EEPROM.put(n,pk_vector[i]);
    if(DEBUG_EEPROM)Serial.println("write:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_selected));
    n += sizeof(pk);
  }
  EEPROM.commit();
}

void readPKvector(String preset_name){
  uint16_t n = 0; uint16_t size = 0; 
  if(preset_name=="TEST") n = EEPROM_TEST_START; //size = EEPROM_TEST_SIZE;
  else if(preset_name=="PRES1") n = EEPROM_PRES1_START; //size = EEPROM_PRES1_SIZE;
  else if(preset_name=="PRES2") n = EEPROM_PRES2_START; //size = EEPROM_PRES2_SIZE;
  else if(preset_name=="PRES_FEZ") n = EEPROM_PRES_FEZ_START; //size = EEPROM_PRES2_SIZE;

  uint8_t num_element;
  EEPROM.get(n,num_element);
  Serial.println("read:index_byte"+String(n)+",num_element"+String(num_element));
  n += sizeof(uint8_t)*2;
  
  //==Second, read pk array.
  pk pk;
  pk_vector = {};
  for (uint8_t i=0; i < uint8_t(num_element); i++){
    if(num_element==0) break;
    if(EEPROM_ONE_SIZE < i) break;
    if(i > uint8_t(num_element)) break;

    Serial.println("i="+String(i) +",numele="+ String(num_element));
    EEPROM.get(n,pk);
    pk_vector.push_back(pk);
    //if(pk_vector[i].rpy_selected=='0')break;
    if(DEBUG_EEPROM)Serial.println("read:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_selected));
    n += sizeof(pk);
  }
}

void deleteEEPROM(String preset_name){
  uint16_t n =0;
  uint16_t size=0; 
  if(preset_name=="TEST") {n = n + EEPROM_TEST_START; size = EEPROM_TEST_SIZE;}
  else if(preset_name=="PRES1"){ n = n + EEPROM_PRES1_START; size = EEPROM_PRES1_SIZE;}
  else if(preset_name=="PRES2"){ n = n + EEPROM_PRES2_START; size = EEPROM_PRES2_SIZE;}
  else if(preset_name=="PRES_FEZ") {n = EEPROM_PRES_FEZ_START; size = EEPROM_PRES_FEZ_SIZE;}

  const uint8_t num_element = 0;
  EEPROM.put(n,num_element);// Set num_element.
  n += sizeof(uint8_t)*2;

  pk pk_v = {'0',0,0,'0'};
  for (int i=0 ; i < size/sizeof(pk) ; i++){
    EEPROM.put(n,pk_v);
    n += sizeof(pk);
  }
  EEPROM.commit();
  pk_vector = {};
}


/* == Flush and Read by array == 
void flushPKarray(pk *a){
  int n = 0;
  for (int i=0 ; i < SIZE_PK_array; i++){
    EEPROM.put(n,a[i]);
    n += sizeof(pk);
    if(a[i].rpy_selected=='0')break;
  }
  EEPROM.commit();
}

void readPKarray(){
  int n = 0;
  for (int i=0; i <  SIZE_PK_array; i++){
    EEPROM.get(n,pk_array2[i]);
    n += sizeof(pk);
    if(pk_array2[i].rpy_selected=='0')break;
    if(DEBUG_EEPROM)Serial.println("read:"+String(sizeof(pk))+","+String(pk_array2[i].hid_input) +""+String(pk_array2[i].rpy_selected));
  }
}
*/


void serialprint_pkvector(std::vector<pk>& pk_vector){
  Serial.println(String(pk_vector.size()));
  for (int i=0 ; i < pk_vector.size(); i++){
    if(DEBUG_EEPROM)Serial.println(String(pk_vector[i].hid_input) +","+String(pk_vector[i].min_angle) + "," + String(pk_vector[i].max_angle)+","+","+String(pk_vector[i].roll) + "," + String(pk_vector[i].pitch) +"," + String(pk_vector[i].rpy_selected));
  }
}

void calibrateMPU6886(){
  float gyroSumX,gyroSumY,gyroSumZ;
  float accSumX,accSumY,accSumZ;
  int calibCount = 1000;

  Serial.println("Calibrating...");
  digitalWrite(10, LOW);
  vTaskDelay(2000);
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
      vTaskDelay(2);
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
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  /*M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  */
  digitalWrite(10, HIGH);
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
