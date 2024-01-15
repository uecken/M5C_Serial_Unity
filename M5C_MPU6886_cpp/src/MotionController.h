#ifndef MotionController_H
#define MotionController_H

class MotionController{
    public:
        MotionController();
        ~MotionController();
        void begin();
        void reConnect();
        void inputKey(char key);
        void pressKey(char key);
        void keyboardReleaseAll();
        void moveMouse(float x,float y);
        void setMouseSpeed(float speed);

    private:
        float mouse_speed;
        //float x_prev
        //float y_prev;
};
#endif
