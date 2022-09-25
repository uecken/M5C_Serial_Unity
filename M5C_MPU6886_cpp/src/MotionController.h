#ifndef MotionController_H
#define MotionController_H

namespace MC{
    class MotionController{
        public:
            float mouse_speed;
            MotionController();
            ~MotionController();
            void reConnect();
            void inputKey(char key);
            void pressKey(char key);
            void keyboardReleaseAll();
            void moveMouse(float x,float y);
    };
}    
#endif
