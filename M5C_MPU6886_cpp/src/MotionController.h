#ifndef MotionController_H
#define MotionController_H

namespace MC{
    class MotionController{
        public:
            MotionController();
            ~MotionController();
            void inputKey(char key);
            void moveMouse(int x, int y);
    };
}    
#endif
