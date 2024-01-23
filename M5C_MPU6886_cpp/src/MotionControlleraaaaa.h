#ifndef MotionController_H
#define MotionController_H
#include <AccelRingBuffer.hpp>

class MotionController{
    public:
        MotionController();
        ~MotionController();
        void begin();
        void reConnect();
        void inputKey(char key);
        void pressKey(char key);
        void releaseKey(char key);
        void keyboardReleaseAll();
        void moveMouse(float x,float y);
        void setMouseSpeed(float speed);
        void SF_hadoken();
        void execHIDInputs(char* inputs,float acc_threshold, AccelRingBuffer* acc_buffer);

    private:
        float mouse_speed;
        //float x_prev
        //float y_prev;
};
#endif
