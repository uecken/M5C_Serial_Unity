#include "MotionController.h"
#include <BleCombo.h>

//extern BleComboMouse Mouse;
//extern BleComboKeyboard Keyboard;

namespace MC{
    float mouse_speed = 10;

    MotionController::MotionController() {
        Keyboard.begin();
        Mouse.begin();
        Keyboard.println("Hello World");
    }

    MotionController::~MotionController() {        
    }

    void MotionController::reConnect() {
        Keyboard.end();
        Mouse.end();
        Keyboard.begin();
        Mouse.begin();
    }


    void MotionController::inputKey(char key){
        if(Keyboard.isConnected()){
            Keyboard.println(key);   
        }
    }

    void MotionController::pressKey(char key){
        if(Keyboard.isConnected()){
            Keyboard.press(key);
        }
    }

    void MotionController::keyboardReleaseAll(){
        if(Keyboard.isConnected()){
            Keyboard.releaseAll();
        }
    }


    void MotionController::moveMouse(float x, float y){
        //x = x * mouse_speed;
        //y = y * mouse_speed;
        /*
        if(x>127) x=127;
        else if(x <-127) x=-127;
        if(y>127) y=127;
        else if(y <-127) y=-127;
        */
       
        Mouse.move(x,y);
        return;
    }

};
