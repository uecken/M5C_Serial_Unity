#include "MotionController.h"
#include <BleCombo.h>

extern BleComboMouse Mouse;
extern BleComboKeyboard Keyboard;

namespace MC{
    MotionController::MotionController() {
        Keyboard.begin();
        Mouse.begin();
        Keyboard.println("Hello World");
    }
    MotionController::~MotionController() {
    }

    void MotionController::inputKey(char key){
        if(Keyboard.isConnected()){
            Keyboard.println(key);
        }
    }
};